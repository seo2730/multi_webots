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
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node

from webots_robot_spawner.brain_launcher import LocalProcessLauncher
from webots_robot_spawner.fleet_loader import load_fleet
from webots_robot_spawner.free_space_sampler import FreeSpaceSampler
from webots_robot_spawner.robot_types import KNOWN_ROBOT_PROTOS, ROBOT_TYPES
from webots_spawner_msgs.srv import SpawnRobot


@dataclass
class SpawnResult:
    """소환 시도 하나의 결과. 서비스 응답과 편대 로그가 같은 값을 쓴다."""

    success: bool
    robot_id: str = ''
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    message: str = ''


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
        # 편대 매니페스트. 비우면 아무것도 자동 소환하지 않는다(서비스 호출만 받음).
        self.declare_parameter('fleet_manifest', '')
        # 매니페스트 소환을 시작하기 전 기다리는 시간(초). Webots 접속 직후 바로
        # 밀어 넣으면 씬 트리가 아직 안정되지 않은 상태에서 삽입이 겹칠 수 있다.
        self.declare_parameter('fleet_start_delay', 3.0)

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
        # robot_id -> (webots node, BrainHandle 또는 None, RobotType)
        # RobotType 을 같이 들고 있는 이유: 뇌 접속 확인 뒤에 그 로봇이 동기화를
        # 필요로 하는지(needs_sync) 알아야 하기 때문이다.
        self._spawned = {}

        self._srv = self.create_service(SpawnRobot, 'spawn_robot', self._on_spawn)

        found = self._scan_robots()
        self.get_logger().info(
            f'소환 준비 완료. 월드에 이미 있는 로봇 {len(found)}대: '
            f'{", ".join(sorted(n for n, _, _ in found)) or "없음"}')

        # 편대 매니페스트가 있으면 잠시 뒤 한 번 소환한다.
        manifest = str(self.get_parameter('fleet_manifest').value).strip()
        if manifest:
            delay = float(self.get_parameter('fleet_start_delay').value)
            self.get_logger().info(f'편대 매니페스트 예약: {manifest} ({delay:.1f}초 후)')
            self._fleet_timer = self.create_timer(delay, self._run_fleet_once)
            self._fleet_manifest = manifest

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
        """서비스 껍데기. 실제 일은 spawn_one() 이 한다.

        편대 매니페스트(fleet_loader)도 같은 spawn_one() 을 쓴다. 경로가 갈리면
        "서비스로는 되는데 매니페스트로는 안 되는" 차이가 생기기 때문이다.
        """
        result = self.spawn_one(
            type_key=request.type,
            robot_id=request.robot_id,
            random_place=request.random,
            x=request.x, y=request.y, yaw=request.yaw,
            min_clearance=request.min_clearance,
            force=request.force,
        )
        response.success = result.success
        response.robot_id = result.robot_id
        response.x, response.y, response.yaw = result.x, result.y, result.yaw
        response.message = result.message
        return response

    # ------------------------------------------------------------------ 소환 코어

    def spawn_one(self, type_key, robot_id='', random_place=False,
                  x=0.0, y=0.0, yaw=0.0, min_clearance=0.0, force=False,
                  bounds=None, strict_map=True) -> SpawnResult:
        """로봇 한 대를 소환한다. 서비스와 편대 매니페스트가 공유하는 유일한 경로.

        Args:
            bounds: (xmin, ymin, xmax, ymax). random_place 일 때 이 사각형 안에서만
                고른다. 월드에 로봇이 하나도 없는 냉시동에서는 SLAM 맵이 존재할 수
                없으므로 이 경로가 필요하다.
            strict_map: 좌표를 직접 지정한 경우, 맵 점유 검사 실패를 거절 사유로
                볼지(True) 경고만 남기고 진행할지(False).
                로봇끼리 겹치는 검사는 strict_map 과 무관하게 **항상** 막는다.

                False 가 필요한 이유: 편대 매니페스트는 부팅 설정이다. 사람이 좌표를
                골라 적어 둔 것이고, 실패해도 되물을 상대가 없다. 그런데 점유격자는
                SLAM 파생물이라 낡을 수 있다 — 실제로 월드를 비우고 편대를 처음부터
                올릴 때, 이전 세션 맵에 남은 "옛 로봇의 몸"이 장애물로 찍혀 있어서
                원래 스폰 좌표 4곳이 전부 거절됐다. 낡은 파생 데이터 때문에 편대가
                아예 안 뜨는 것보다, 경고를 남기고 띄우는 편이 낫다.
                반대로 서비스(/spawn_robot)는 사람이 지켜보는 대화형 경로라
                엄격하게 두고 force 로 강행할 수 있게 한다.
        """
        robot_type = ROBOT_TYPES.get(str(type_key).strip().lower())
        if robot_type is None:
            msg = (f"모르는 로봇 종류 '{type_key}'. "
                   f"가능한 값: {', '.join(sorted(ROBOT_TYPES))}")
            self.get_logger().warn(msg)
            return SpawnResult(False, message=msg)

        if not robot_type.ready:
            self.get_logger().warn(f'[{robot_type.key}] {robot_type.not_ready_reason}')
            return SpawnResult(False, message=robot_type.not_ready_reason)

        existing = self._scan_robots()
        existing_names = {name for name, _, _ in existing}

        # 이름 정하기 --------------------------------------------------
        robot_id = str(robot_id).strip()
        if not robot_id:
            robot_id = self._allocate_id(robot_type, existing_names)
        elif robot_id in existing_names:
            msg = f"이미 '{robot_id}'라는 로봇이 월드에 있습니다"
            self.get_logger().warn(msg)
            return SpawnResult(False, robot_id=robot_id, message=msg)

        # 자리 정하기 --------------------------------------------------
        clearance = float(min_clearance) or robot_type.default_clearance
        avoid = [(rx, ry, self._separation) for _, rx, ry in existing]

        if random_place:
            spot, reason = self._sampler.sample(
                clearance, avoid=avoid, attempts=self._attempts, rng=self._rng,
                bounds=bounds)
            if spot is None:
                msg = f'빈 자리를 찾지 못했습니다: {reason}'
                self.get_logger().warn(msg)
                return SpawnResult(False, robot_id=robot_id, message=msg)
            x, y = spot
            yaw = float(self._rng.uniform(-np.pi, np.pi))
        else:
            x, y, yaw = float(x), float(y), float(yaw)

            # 로봇 겹침은 어떤 경우에도 막는다(force 만 예외). 겹쳐 놓으면 물리가
            # 서로를 밀어내며 둘 다 엉뚱한 곳으로 간다.
            ok, reason = self._sampler.check_robots(x, y, clearance, avoid=avoid)
            if not ok and not force:
                msg = (f'({x:.2f}, {y:.2f})에 놓을 수 없습니다: {reason}. '
                       'force: true로 강행할 수 있습니다.')
                self.get_logger().warn(msg)
                return SpawnResult(False, robot_id=robot_id, message=msg)

            # 맵 점유 검사는 strict_map 에 따라 거절 사유이거나 경고다.
            map_ok, map_reason = self._sampler.check_map(x, y, clearance)
            if not map_ok:
                if strict_map and not force:
                    msg = (f'({x:.2f}, {y:.2f})에 놓을 수 없습니다: {map_reason}. '
                           'force: true로 강행할 수 있습니다.')
                    self.get_logger().warn(msg)
                    return SpawnResult(False, robot_id=robot_id, message=msg)
                self.get_logger().warn(
                    f'[{robot_id}] 맵 검사 실패({map_reason})지만 그대로 진행합니다')

        # 몸 넣기 ------------------------------------------------------
        node = self._insert_node(robot_type, robot_id, x, y, yaw)
        if node is None:
            msg = (f'Webots 씬 트리 삽입에 실패했습니다. {robot_type.proto}가 월드에 '
                   'IMPORTABLE EXTERNPROTO로 선언돼 있는지 확인하세요.')
            self.get_logger().error(msg)
            return SpawnResult(False, robot_id=robot_id, message=msg)

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
                msg = f'뇌 실행에 실패해 몸을 되돌렸습니다: {exc}'
                self.get_logger().error(msg)
                return SpawnResult(False, robot_id=robot_id, message=msg)
            self._arm_rollback(robot_id)

        self._spawned[robot_id] = (node, handle, robot_type)

        return SpawnResult(
            True, robot_id=robot_id, x=x, y=y, yaw=yaw,
            message=(f'{robot_id} 소환 완료 ({x:.2f}, {y:.2f}, yaw {yaw:.2f})'
                     + ('' if self._auto_brain
                        else ' — 뇌는 직접 띄우세요(auto_launch_brain=false)')))

    def reclaim(self, type_key, robot_id, **spawn_kwargs) -> SpawnResult:
        """뇌 없이 몸만 남은 로봇을 **버리고 새로 소환한다.**

        fleet 컨테이너가 재시작되면(월드 재로드 등) 소환기가 추적하던 뇌 목록은
        비는데 Webots 안의 몸은 그대로 남아 있다. 이름이 충돌하니 뭔가 해야 한다.

        🚨 처음에는 "몸을 그대로 두고 뇌만 다시 붙인다"로 만들었는데 **틀렸다.**
        실측하면 그렇게 붙인 로봇은 센서가 죽는다. 컨트롤러가 끊겼다 다시 붙는
        과정에서 장치 활성 상태가 살아나지 않고, 플러그인이 값을 읽을 때마다

            Error: wb_gps_get_values() called for a disabled device!

        가 뜬다. drone_driver 는 GPS 가 NaN 이면 그 스텝을 건너뛰도록 되어 있어서
        odom 을 영구히 발행하지 못한다. 실측 비교: 뇌만 붙인 drone1 은 이 오류가
        4726건, 같은 시점에 새로 소환한 drone2 는 0건이었다.

        그래서 몸을 지우고 처음부터 다시 만든다. 뇌가 없던 몸이므로 잃을 상태가 없고,
        매니페스트는 "원하는 상태"의 선언이니 좌표도 매니페스트 값으로 되돌아간다.
        """
        robot_type = ROBOT_TYPES.get(str(type_key).strip().lower())
        if robot_type is None:
            return SpawnResult(False, robot_id=robot_id,
                               message=f"모르는 로봇 종류 '{type_key}'")

        # 우리가 띄운 뇌가 아직 살아 있으면 건드리지 않는다.
        entry = self._spawned.get(robot_id)
        if entry is not None and entry[1] is not None and entry[1].is_alive():
            return SpawnResult(True, robot_id=robot_id,
                               message=f'{robot_id} 는 이미 정상 동작 중이라 건너뜁니다')

        removed = False
        for i in range(self._root_children.getCount()):
            node = self._root_children.getMFNode(i)
            if node is None or node.getTypeName() not in KNOWN_ROBOT_PROTOS:
                continue
            name_field = node.getField('name')
            if name_field is not None and name_field.getSFString() == robot_id:
                node.remove()
                removed = True
                break

        if not removed:
            return SpawnResult(False, robot_id=robot_id,
                               message=f'{robot_id} 의 몸을 씬 트리에서 못 찾았습니다')

        self._spawned.pop(robot_id, None)
        self.get_logger().info(
            f'[{robot_id}] 뇌 없이 몸만 남아 있어 제거하고 다시 소환합니다')

        # 🚨 remove() 는 다음 스텝에 반영된다. 곧바로 spawn_one 을 부르면 씬 트리
        # 스캔에 **방금 지운 몸이 아직 보이고**, 그게 자기 자신의 자리를 막는다.
        # 실측: ugv1 을 회수할 때 드리프트된 자기 몸(-7.71, 1.12)이 목표 좌표에서
        # 1.56m 거리로 잡혀 "기존 로봇과 너무 가깝습니다"로 거절됐고, 몸은 이미
        # 지워졌으므로 로봇이 아예 사라졌다.
        self._sv.step(int(self._sv.getBasicTimeStep()))

        still_there = {n for n, _, _ in self._scan_robots()}
        if robot_id in still_there:
            self.get_logger().warn(
                f'[{robot_id}] 한 스텝 뒤에도 몸이 씬 트리에 남아 있습니다')

        result = self.spawn_one(type_key=type_key, robot_id=robot_id, **spawn_kwargs)
        if result.success:
            return result

        # 몸은 이미 지웠으니 여기서 포기하면 로봇이 사라진다. 마지막으로 강행한다.
        self.get_logger().warn(
            f'[{robot_id}] 재소환 실패({result.message}) — 몸을 이미 지웠으므로 '
            'force 로 한 번 더 시도합니다')
        forced = dict(spawn_kwargs)
        forced['force'] = True
        return self.spawn_one(type_key=type_key, robot_id=robot_id, **forced)

    # ------------------------------------------------------------------ 편대

    def _run_fleet_once(self):
        """편대 매니페스트를 딱 한 번 처리한다."""
        self.destroy_timer(self._fleet_timer)
        try:
            load_fleet(self, self._fleet_manifest)
        except Exception as exc:                          # noqa: BLE001
            # 편대 소환이 실패해도 노드는 살아 있어야 한다. 서비스로 수동 소환은
            # 계속 되어야 하고, 여기서 죽으면 컨테이너가 재시작 루프에 빠진다.
            self.get_logger().error(f'편대 소환 중 오류: {exc}')

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
            node, handle, robot_type = entry
            if handle is None or handle.is_alive():
                self.get_logger().info(f'[{robot_id}] 뇌 정상 동작 확인')
                self._enable_sync_if_needed(robot_id, node, robot_type)
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

    def _enable_sync_if_needed(self, robot_id, node, robot_type):
        """뇌가 붙은 것을 확인한 뒤 Webots 노드의 synchronization 을 TRUE 로 되돌린다.

        주입할 때는 FALSE 여야 한다(뇌가 붙기 전에 시뮬이 멈추므로). 하지만 드론은
        자세 루프가 매 물리 스텝 돌아야 뒤집히지 않는다. 그래서 뇌 접속을 확인한
        이 시점에 되돌린다.

        되돌린 뒤에는 그 로봇의 뇌가 죽으면 시뮬 전체가 멈춘다. 정적으로 놓였던
        drone1 이 원래 그랬으므로 새로운 위험은 아니지만, 알고 있어야 한다.
        """
        if robot_type is None or not robot_type.needs_sync or node is None:
            return
        field = node.getField('synchronization')
        if field is None:
            self.get_logger().warn(
                f'[{robot_id}] synchronization 필드가 없어 동기화를 되돌리지 못했습니다')
            return
        try:
            field.setSFBool(True)
        except Exception as exc:                          # noqa: BLE001
            self.get_logger().warn(f'[{robot_id}] 동기화 전환 실패: {exc}')
            return
        self.get_logger().info(
            f'[{robot_id}] synchronization 을 TRUE 로 되돌렸습니다 '
            '(자세 루프가 매 물리 스텝 돌아야 하는 기체)')

    # ------------------------------------------------------------------ 정리

    def shutdown(self):
        for robot_id, (_, handle, _type) in self._spawned.items():
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
