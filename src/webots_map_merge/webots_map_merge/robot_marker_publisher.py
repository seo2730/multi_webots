"""TF에서 로봇을 읽어와 관제 화면용 마커로 발행하는 노드.

왜 필요한가
-----------
RViz2 는 디스플레이를 자동으로 추가하지 못한다. 설정 파일을 읽는 시점에 고정되므로,
로봇마다 하나씩 필요한 `RobotModel` 디스플레이는 새 로봇이 생겨도 저절로 안 생긴다.

그래서 "여러 대상을 하나의 디스플레이가 그리는" 방식으로 우회한다.
이 노드가 TF 트리를 훑어 `{ns}/base_link` 형태의 프레임을 전부 찾아내고,
각 로봇 위치에 화살표 + 이름표 마커를 만들어 `/robot_markers` 하나로 발행한다.
RViz 에는 MarkerArray 디스플레이 **하나만** 있으면 되고,
로봇이 늘어나도 설정을 고칠 필요가 없다.

발견 기준을 TF 로 잡은 이유는, 맵이 없는 로봇(드론)이나 등록 노드를 안 띄운
로봇(spot1)도 `world` 에 연결만 되어 있으면 전부 잡히기 때문이다.
"""

import re

import rclpy
import yaml
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

BASE_LINK_RE = re.compile(r'^([^/]+)/base_link$')

# 이름에서 결정적으로 색을 뽑는다. 같은 로봇은 항상 같은 색이 된다.
PALETTE = [
    (0.95, 0.35, 0.25),   # 빨강
    (0.30, 0.65, 0.95),   # 파랑
    (0.45, 0.80, 0.40),   # 초록
    (0.95, 0.75, 0.25),   # 노랑
    (0.75, 0.45, 0.90),   # 보라
    (0.35, 0.85, 0.85),   # 청록
]


def color_for(name: str):
    return PALETTE[sum(name.encode()) % len(PALETTE)]


class RobotMarkerPublisher(Node):

    def __init__(self):
        super().__init__('robot_marker_publisher')

        self.world_frame = self._param('world_frame', 'world')
        self.topic = self._param('marker_topic', '/robot_markers')
        rate = float(self._param('publish_rate', 2.0))
        # tf2 는 한 번 등록된 프레임 이름을 계속 들고 있어서, 로봇이 꺼져도
        # 프레임 목록에서는 사라지지 않는다. 그래서 "언제 갱신됐는지"로 생사를 판단한다.
        # 이게 없으면 꺼진 로봇의 마커가 마지막 위치에 영원히 남는다.
        self.max_pose_age = float(self._param('max_pose_age', 5.0))
        self.arrow_len = float(self._param('arrow_length', 1.0))
        self.text_height = float(self._param('text_height', 1.2))
        self.text_size = float(self._param('text_size', 0.5))

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(MarkerArray, self.topic, 10)

        self.published: set[str] = set()   # 지금까지 마커를 낸 로봇들
        self.create_timer(1.0 / max(rate, 0.01), self._tick)

        self.get_logger().info(
            f"로봇 마커 발행 시작 | TF 에서 '{{ns}}/base_link' 를 찾아 "
            f"'{self.topic}' 로 발행 | 기준 프레임 '{self.world_frame}'")

    def _param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    # ------------------------------------------------------------------
    def _robot_frames(self) -> list[str]:
        """TF 트리에 있는 모든 `{ns}/base_link` 프레임의 네임스페이스 목록."""
        try:
            frames = yaml.safe_load(self.buf.all_frames_as_yaml()) or {}
        except yaml.YAMLError:
            return []
        found = []
        for frame in frames:
            match = BASE_LINK_RE.match(frame)
            if match:
                found.append(match.group(1))
        return sorted(found)

    def _is_stale(self, stamp, now) -> bool:
        """이 변환을 살아있는 로봇의 것으로 볼 수 없는지 판단한다.

        - 너무 오래된 것: 로봇이 죽었다.
        - 너무 미래인 것: 그 로봇이 use_sim_time 없이 벽시계로 TF 를 쏘고 있다는 뜻.
          이걸 그냥 두면 나이가 계속 음수라 영원히 살아있는 것으로 잡히고,
          게다가 tf2 가 이후의 정상 데이터를 TF_OLD_DATA 로 거부해버린다.
        - stamp 가 0 인 것: 체인이 전부 static. 원래 갱신되지 않으므로 정상으로 본다.
        """
        stamp_ns = stamp.sec * 10**9 + stamp.nanosec
        if stamp_ns == 0:
            return False

        age_ns = now.nanoseconds - stamp_ns
        limit_ns = self.max_pose_age * 1e9
        if age_ns < -limit_ns:
            self.get_logger().warn(
                f'미래 시각의 TF 발견 ({-age_ns / 1e9:.1f}초 앞섬). '
                f'해당 로봇이 use_sim_time 없이 실행 중일 가능성이 높음.',
                throttle_duration_sec=30.0)
            return True
        return age_ns > limit_ns

    def _tick(self):
        namespaces = self._robot_frames()
        array = MarkerArray()
        alive = set()

        now = self.get_clock().now()
        for ns in namespaces:
            frame = f'{ns}/base_link'
            try:
                stamped = self.buf.lookup_transform(
                    self.world_frame, frame, rclpy.time.Time())
            except Exception:
                # world 에 아직 연결 안 된 로봇은 건너뛴다 (곧 붙는다)
                continue

            if self._is_stale(stamped.header.stamp, now):
                continue

            alive.add(ns)
            tf = stamped.transform
            r, g, b = color_for(ns)
            stamp = now.to_msg()

            arrow = Marker()
            arrow.header.frame_id = self.world_frame
            arrow.header.stamp = stamp
            arrow.ns = f'{ns}/arrow'
            arrow.id = 0
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = tf.translation.x
            arrow.pose.position.y = tf.translation.y
            arrow.pose.position.z = tf.translation.z + 0.35
            arrow.pose.orientation = tf.rotation
            arrow.scale.x = self.arrow_len      # 길이
            arrow.scale.y = 0.12                # 몸통 지름
            arrow.scale.z = 0.2                 # 머리 지름
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = r, g, b, 0.9
            array.markers.append(arrow)

            label = Marker()
            label.header.frame_id = self.world_frame
            label.header.stamp = stamp
            label.ns = f'{ns}/label'
            label.id = 0
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = tf.translation.x
            label.pose.position.y = tf.translation.y
            label.pose.position.z = tf.translation.z + self.text_height
            label.pose.orientation.w = 1.0
            label.scale.z = self.text_size
            label.color.r, label.color.g, label.color.b, label.color.a = r, g, b, 1.0
            label.text = ns
            array.markers.append(label)

        # 사라진 로봇의 마커는 명시적으로 지운다 (lifetime 에 기대지 않는다)
        for ns in self.published - alive:
            for suffix in ('arrow', 'label'):
                gone = Marker()
                gone.header.frame_id = self.world_frame
                gone.ns = f'{ns}/{suffix}'
                gone.id = 0
                gone.action = Marker.DELETE
                array.markers.append(gone)
            self.get_logger().info(f'[마커] {ns} 사라짐 -> 마커 제거')

        for ns in alive - self.published:
            self.get_logger().info(f'[마커] {ns} 발견 -> 마커 추가')

        self.published = alive
        if array.markers:
            self.pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = RobotMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
