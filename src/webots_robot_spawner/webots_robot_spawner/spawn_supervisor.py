"""실행 중인 Webots 시뮬레이션에 로봇을 소환하는 노드.

Webots에서 시뮬레이션을 멈추지 않고 로봇을 추가하는 방법은 Supervisor API 하나뿐이다.
월드 파일을 고치는 게 아니라, 메모리에 떠 있는 씬 트리에 노드를 직접 꽂아 넣는다.
그래서 이 노드는 평범한 ROS 2 노드가 아니라 **Webots extern 컨트롤러이면서 동시에
ROS 2 노드**다. 컨테이너에 Webots R2025a가 통째로 설치돼 있어(Dockerfile 4단계)
`from controller import Supervisor`가 그대로 된다.

한 번의 소환에서 벌어지는 일:

    [1] 자리 정하기   random이면 /map_merged의 빈 셀에서 고르고, 아니면 요청 좌표를 검사
    [2] 이름 정하기   씬 트리의 기존 로봇을 보고 다음 번호를 매김 (drone1, drone2 ...)
    [3] 몸 넣기       importMFNodeFromString으로 씬 트리에 삽입  ← 화면에 나타나는 순간
    [4] 뇌 띄우기     ros2 launch를 자식 프로세스로 실행 → driver가 [3]의 몸에 접속
    [5] 합류          robot_registrar가 알아서 마스터에 등록 (기존 코드, 손댈 것 없음)

[5]는 webots_map_merge가 이미 하고 있어서 여기서 할 일이 없다. 소환된 로봇과
처음부터 월드에 있던 로봇은 마스터 입장에서 구분되지 않는다.

⚠️ 단일 스레드로 도는 것이 **의도**다. Webots 컨트롤러 API는 스레드 안전하지 않으므로
   서비스 콜백이 step()과 같은 스레드에서 실행돼야 한다. MultiThreadedExecutor로
   바꾸지 말 것.
"""

import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node

from webots_robot_spawner.brain_launcher import LocalProcessLauncher
from webots_robot_spawner.free_space_sampler import FreeSpaceSampler
from webots_robot_spawner.robot_types import KNOWN_ROBOT_PROTOS, ROBOT_TYPES
from webots_spawner_msgs.srv import SpawnRobot


class SpawnSupervisor(Node):

    def __init__(self, supervisor):
        super().__init__('spawn_supervisor')

        self._sv = supervisor
        self._root_children = supervisor.getRoot().getField('children')

        self.declare_parameter('map_topic', '/map_merged')
        self.declare_parameter('allow_unknown', False)
        self.declare_parameter('sample_attempts', 200)
        self.declare_parameter('robot_separation', 1.0)
        self.declare_parameter('brain_grace_period', 8.0)
        self.declare_parameter('log_dir', '/tmp/spawned_robots')
        self.declare_parameter('auto_launch_brain', True)

        self._separation = float(self.get_parameter('robot_separation').value)
        self._attempts = int(self.get_parameter('sample_attempts').value)
        self._grace = float(self.get_parameter('brain_grace_period').value)
        self._auto_brain = bool(self.get_parameter('auto_launch_brain').value)

        self._sampler = FreeSpaceSampler(
            self,
            topic=self.get_parameter('map_topic').value,
            allow_unknown=bool(self.get_parameter('allow_unknown').value),
        )
        self._launcher = LocalProcessLauncher(
            self, log_dir=self.get_parameter('log_dir').value)

        self._rng = np.random.default_rng()
        self._spawned = {}   # robot_id -> (webots node, BrainHandle | None)

        self._srv = self.create_service(SpawnRobot, 'spawn_robot', self._on_spawn)

        found = self._scan_robots()
        self.get_logger().info(
            f'소환 준비 완료. 월드에 이미 있는 로봇 {len(found)}대: '
            f'{", ".join(sorted(n for n, _, _ in found)) or "없음"}')

    # ------------------------------------------------------------------ 씬 트리

    def _scan_robots(self):
        """씬 트리에서 우리 로봇들을 찾아 (이름, x, y)로 돌려준다.

        기존 로봇 목록의 출처로 TF나 /robot_registry 대신 씬 트리를 쓴다.
        뇌가 아직 안 붙었거나 죽은 로봇도 몸은 거기 있기 때문이다.
        자리 겹침을 막으려면 "몸이 있는 것" 전부를 알아야 한다.
        """
        robots = []
        for i in range(self._root_children.getCount()):
            node = self._root_children.getMFNode(i)
            if node is None or node.getTypeName() not in KNOWN_ROBOT_PROTOS:
                continue
            name_field = node.getField('name')
            if name_field is None:
                continue
            pos = node.getPosition()
            robots.append((name_field.getSFString(), pos[0], pos[1]))
        return robots

    def _allocate_id(self, robot_type, existing_names):
        index = 1
        while f'{robot_type.id_prefix}{index}' in existing_names:
            index += 1
        return f'{robot_type.id_prefix}{index}'

    # ------------------------------------------------------------------ 서비스

    def _on_spawn(self, request, response):
        response.success = False
        response.robot_id = ''

        robot_type = ROBOT_TYPES.get(request.type.strip().lower())
        if robot_type is None:
            response.message = (
                f"모르는 로봇 종류 '{request.type}'. "
                f"가능한 값: {', '.join(sorted(ROBOT_TYPES))}")
            self.get_logger().warn(response.message)
            return response

        if not robot_type.ready:
            response.message = robot_type.not_ready_reason
            self.get_logger().warn(f'[{robot_type.key}] {response.message}')
            return response

        existing = self._scan_robots()
        existing_names = {name for name, _, _ in existing}

        # 이름 정하기 --------------------------------------------------
        robot_id = request.robot_id.strip()
        if not robot_id:
            robot_id = self._allocate_id(robot_type, existing_names)
        elif robot_id in existing_names:
            response.message = f"이미 '{robot_id}'라는 로봇이 월드에 있습니다"
            self.get_logger().warn(response.message)
            return response

        # 자리 정하기 --------------------------------------------------
        clearance = request.min_clearance or robot_type.default_clearance
        avoid = [(x, y, self._separation) for _, x, y in existing]

        if request.random:
            spot, reason = self._sampler.sample(
                clearance, avoid=avoid, attempts=self._attempts, rng=self._rng)
            if spot is None:
                response.message = f'빈 자리를 찾지 못했습니다: {reason}'
                self.get_logger().warn(response.message)
                return response
            x, y = spot
            yaw = float(self._rng.uniform(-np.pi, np.pi))
        else:
            x, y, yaw = float(request.x), float(request.y), float(request.yaw)
            ok, reason = self._sampler.check(x, y, clearance, avoid=avoid)
            if not ok:
                if not request.force:
                    response.message = (
                        f'({x:.2f}, {y:.2f})에 놓을 수 없습니다: {reason}. '
                        'force: true로 강행할 수 있습니다.')
                    self.get_logger().warn(response.message)
                    return response
                self.get_logger().warn(
                    f'[{robot_id}] 검사 실패({reason})지만 force=true라 강행합니다')

        # 몸 넣기 ------------------------------------------------------
        node = self._insert_node(robot_type, robot_id, x, y, yaw)
        if node is None:
            response.message = (
                f'Webots 씬 트리 삽입에 실패했습니다. '
                f'{robot_type.proto}가 월드에 EXTERNPROTO로 선언돼 있는지 확인하세요.')
            self.get_logger().error(response.message)
            return response

        self.get_logger().info(
            f'[{robot_id}] 몸 삽입 완료: {robot_type.proto} '
            f'@ ({x:.3f}, {y:.3f}, {robot_type.spawn_z:.3f}), yaw {yaw:.3f}')

        # 뇌 띄우기 ----------------------------------------------------
        handle = None
        if self._auto_brain:
            try:
                handle = self._launcher.launch(robot_id, robot_type, x, y, yaw)
            except Exception as exc:                      # noqa: BLE001
                # 뇌를 못 띄웠으면 몸만 남는다. 그대로 두면 유령 로봇이 쌓이므로 되돌린다.
                node.remove()
                response.message = f'뇌 실행에 실패해 몸을 되돌렸습니다: {exc}'
                self.get_logger().error(response.message)
                return response
            self._arm_rollback(robot_id)

        self._spawned[robot_id] = (node, handle)

        response.success = True
        response.robot_id = robot_id
        response.x, response.y, response.yaw = x, y, yaw
        response.message = (
            f'{robot_id} 소환 완료 ({x:.2f}, {y:.2f}, yaw {yaw:.2f})'
            + ('' if self._auto_brain else ' — 뇌는 직접 띄우세요(auto_launch_brain=false)'))
        return response

    # ------------------------------------------------------------------ 삽입/롤백

    def _insert_node(self, robot_type, robot_id, x, y, yaw):
        """씬 트리 끝에 노드를 꽂고, 실제로 들어갔는지 확인해서 돌려준다."""
        before = self._root_children.getCount()
        self._root_children.importMFNodeFromString(
            -1, robot_type.spawn_string(robot_id, x, y, yaw))

        if self._root_children.getCount() != before + 1:
            return None

        node = self._root_children.getMFNode(before)
        name_field = node.getField('name') if node else None
        if name_field is None or name_field.getSFString() != robot_id:
            # 개수는 늘었는데 우리가 넣은 게 아니면 그대로 두는 편이 위험하다.
            if node is not None:
                node.remove()
            return None
        return node

    def _arm_rollback(self, robot_id: str):
        """뇌가 곧바로 죽는 경우(런치 파일 오류 등)에 몸을 되돌린다.

        서비스는 이미 성공을 돌려준 뒤다. 여기서 잡는 건 "실행은 됐지만
        몇 초 만에 죽는" 경우이고, 안 잡으면 시뮬레이션에 조종 불가능한
        유령 로봇이 하나씩 쌓인다.
        """
        box = {}

        def check():
            # 한 번만 돌면 되는 검사다. rclpy에 one-shot 타이머가 없어서
            # 콜백 안에서 직접 없앤다. 안 그러면 소환할 때마다 타이머가 쌓인다.
            self.destroy_timer(box['timer'])

            entry = self._spawned.get(robot_id)
            if entry is None:
                return
            node, handle = entry
            if handle is None or handle.is_alive():
                self.get_logger().info(f'[{robot_id}] 뇌 정상 동작 확인')
                return

            self.get_logger().error(
                f'[{robot_id}] 뇌가 {self._grace:.0f}초 안에 죽었습니다 '
                f'(종료 코드 {handle.returncode}). 몸을 되돌립니다. '
                f'원인은 {handle.log_path}를 보세요.')
            try:
                node.remove()
            except Exception as exc:                      # noqa: BLE001
                self.get_logger().warn(f'[{robot_id}] 몸 제거 실패: {exc}')
            self._spawned.pop(robot_id, None)

        box['timer'] = self.create_timer(self._grace, check)

    # ------------------------------------------------------------------ 정리

    def shutdown(self):
        for robot_id, (_, handle) in self._spawned.items():
            if handle is not None and handle.is_alive():
                self.get_logger().info(f'[{robot_id}] 뇌 종료')
                try:
                    self._launcher.terminate(handle)
                except Exception as exc:                  # noqa: BLE001
                    self.get_logger().warn(f'[{robot_id}] 종료 중 오류: {exc}')


def main(args=None):
    # Supervisor()를 만들기 전에 접속 주소가 정해져 있어야 한다.
    # 호스트를 환경 변수로 빼 둔 이유: 리눅스 네이티브 Docker에는
    # host.docker.internal이 기본으로 없고(compose의 extra_hosts로 별칭을 만든다),
    # 원격 PC의 Webots에 붙일 수도 있어야 하기 때문이다.
    host = os.environ.get('WEBOTS_HOST', 'host.docker.internal')
    port = os.environ.get('WEBOTS_PORT', '1234')
    name = os.environ.get('SPAWN_SUPERVISOR_NAME', 'spawn_supervisor')
    os.environ.setdefault('WEBOTS_CONTROLLER_URL', f'tcp://{host}:{port}/{name}')

    try:
        from controller import Supervisor
    except ImportError as exc:
        print(f'Webots controller 모듈을 찾을 수 없습니다: {exc}\n'
              'PYTHONPATH에 $WEBOTS_HOME/lib/controller/python이 있어야 합니다.',
              file=sys.stderr)
        return 1

    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    rclpy.init(args=args)
    node = SpawnSupervisor(supervisor)
    try:
        # step()이 루프의 박자를 잡고, 그 사이사이에 ROS 2 콜백을 처리한다.
        # 월드의 spawn_supervisor 노드는 synchronization FALSE라서 이 루프가
        # 늦어져도 시뮬레이션은 기다리지 않는다.
        while rclpy.ok() and supervisor.step(timestep) != -1:
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
