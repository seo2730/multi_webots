"""소환할 수 있는 로봇 종류 정의표.

로봇을 하나 추가한다는 건 결국 두 가지를 아는 것이다.

  1. Webots 씬 트리에 꽂을 노드 문자열 (몸)
  2. 그 몸에 붙일 ROS 2 런치 (뇌)

이 파일은 그 두 가지를 로봇 종류마다 한 줄로 묶어 둔다. 새 로봇을 지원한다는 건
여기에 항목 하나를 더하는 일이 되어야 하며, 다른 파일은 건드리지 않는 게 목표다.

⚠️ PROTO는 반드시 월드(my_world.wbt)에 EXTERNPROTO로 미리 선언돼 있어야 한다.
   Webots는 런타임에 처음 보는 PROTO를 해석하지 못한다.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class RobotType:
    """소환 가능한 로봇 한 종류."""

    key: str                    # 서비스 요청의 type 값
    proto: str                  # Webots 노드/PROTO 타입 이름 (EXTERNPROTO 선언 필요)
    id_prefix: str              # 자동 채번 접두사 (drone -> drone1, drone2 ...)
    spawn_z: float              # 스폰 높이(m). 바퀴/발이 바닥에 닿는 높이.
    footprint_radius: float     # 몸통 반경(m). 자유 공간 판정의 기준.
    default_clearance: float    # 주변에 요구할 여유 반경(m). 반경 + 여유 마진.
    brain_package: str          # 뇌 런치가 든 패키지
    brain_launch: str           # 런치 파일 이름
    has_map: bool               # SLAM으로 맵을 만드는가 (맵 병합 참여 여부)
    ready: bool = True          # False면 아직 지원 안 함 (아래 주석 참고)
    not_ready_reason: str = ''

    def spawn_string(self, robot_id: str, x: float, y: float, yaw: float) -> str:
        """Webots `importMFNodeFromString`에 넘길 노드 문자열을 만든다.

        회전은 z축(수직) 한 축만 쓴다. 지면에 놓는 로봇이라 roll/pitch를 줄 이유가 없고,
        yaw만 쓰면 .wbt의 `rotation 0 0 1 <yaw>` 표기와 그대로 맞아떨어진다.

        🚨 `synchronization FALSE`가 핵심이다. 월드에 정적으로 놓인 로봇들과 다른 점이고,
        그렇게 해야 하는 이유가 있다.

        `<extern>` 컨트롤러는 synchronization TRUE면 Webots가 매 스텝 그 컨트롤러의
        응답을 기다린다. 로봇을 주입하는 순간부터 뇌가 접속할 때까지 몇 초간
        **시뮬레이션 전체가 멈춘다.** 그런데 소환기 자신도 같은 시뮬레이션에서
        step()을 돌고 있으므로 같이 멈추고, 뇌가 끝내 안 뜨면 롤백 감시 타이머조차
        돌지 못해 영원히 굳는다 (헤드리스 Webots로 실측 확인).

        FALSE로 두면 Webots가 이 로봇을 기다리지 않는다. 대신 뇌가 느려질 때 제어 주기가
        물리 주기와 어긋날 수 있다. 시뮬이 통째로 멈추는 것보다는 낫다는 판단이다.
        """
        return (
            f'{self.proto} {{\n'
            f'  translation {x:.6f} {y:.6f} {self.spawn_z:.6f}\n'
            f'  rotation 0 0 1 {yaw:.6f}\n'
            f'  name "{robot_id}"\n'
            f'  controller "<extern>"\n'
            f'  synchronization FALSE\n'
            f'}}\n'
        )


# 드론은 Mavic2ProMedium.proto 안에 센서·짐벌·프로펠러가 전부 들어 있어서
# 스폰 문자열이 그대로 5줄이면 끝난다. 그래서 Phase A의 검증 대상으로 골랐다.
DRONE = RobotType(
    key='drone',
    proto='Mavic2ProMedium',
    id_prefix='drone',
    spawn_z=0.13,           # my_world.wbt의 drone1과 같은 높이
    footprint_radius=0.35,  # 개조 Mavic 대각선 약 0.7m
    default_clearance=0.6,
    brain_package='webots_python',
    brain_launch='single_drone.launch.py',
    has_map=False,          # 거리 센서가 없어 SLAM을 못 돌린다
)

# ---------------------------------------------------------------------------
# 아래 둘은 Phase B. 지금 막아 두는 이유를 오류 메시지로 그대로 돌려준다.
#
# 두 로봇 다 센서가 월드 파일에 인라인으로 박혀 있어서(UGV는 bodySlot에 85줄,
# Spot은 middleExtension에 40줄) 스폰 문자열로 만들 수가 없다. 센서를 품은
# 래퍼 PROTO를 만들고 나면 proto 이름만 바꿔 ready=True로 열면 된다.
# ---------------------------------------------------------------------------
UGV = RobotType(
    key='ugv',
    proto='SummitXlSteel',
    id_prefix='ugv',
    spawn_z=0.12,
    footprint_radius=0.47,   # 0.72 x 0.61 m
    default_clearance=0.75,
    brain_package='webots_python',
    brain_launch='single_ugv.launch.py',
    has_map=True,
    ready=False,
    not_ready_reason=(
        'UGV는 GPS/IMU/Velodyne이 월드의 bodySlot에 인라인으로 박혀 있어 '
        'SummitXlSteelSensorized.proto 래퍼가 필요합니다 (Phase B).'),
)

SPOT = RobotType(
    key='spot',
    proto='Spot',
    id_prefix='spot',
    spawn_z=0.624,
    footprint_radius=0.6,    # 1.1 x 0.5 m
    default_clearance=0.9,
    brain_package='webots_spot',
    brain_launch='single_spot_launch.py',
    has_map=True,
    ready=False,
    not_ready_reason=(
        'Spot은 거리센서 래퍼 PROTO가 필요하고, spot_driver가 getFromDef("Spot")으로 '
        '자기 몸을 찾기 때문에 2대째부터 남의 몸을 잡습니다. getSelf()로 고쳐야 합니다 (Phase B).'),
)

ROBOT_TYPES = {t.key: t for t in (DRONE, UGV, SPOT)}

# 씬 트리에서 "이건 우리 로봇이다"라고 알아보는 데 쓰는 PROTO 이름들.
# 자동 채번과 로봇 간 간격 검사가 이 목록에 의존한다. 래퍼 PROTO를 만들면
# 여기에도 추가해야 기존 로봇을 놓치지 않는다.
KNOWN_ROBOT_PROTOS = frozenset(t.proto for t in ROBOT_TYPES.values())
