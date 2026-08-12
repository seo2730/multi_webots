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
    needs_def: bool = False     # 씬 트리에 DEF 이름을 붙여야 하는가 (아래 참고)
    needs_sync: bool = False    # 제어 루프가 매 물리 스텝 돌아야 하는가 (아래 참고)
    ready: bool = True          # False면 아직 지원 안 함 (아래 주석 참고)
    not_ready_reason: str = ''

    @staticmethod
    def def_name(robot_id: str) -> str:
        """robot_id로부터 Webots DEF 이름을 만든다 (spot2 -> SPOT2).

        DEF 이름에는 영문/숫자/밑줄만 쓴다. robot_id는 ROS 2 네임스페이스라
        어차피 그 범위 안이지만, 하이픈 같은 것이 섞여도 깨지지 않게 걸러 둔다.
        """
        return ''.join(c if c.isalnum() else '_' for c in robot_id).upper()

    def spawn_string(self, robot_id: str, x: float, y: float, yaw: float) -> str:
        """Webots `importMFNodeFromString`에 넘길 노드 문자열을 만든다.

        회전은 z축(수직) 한 축만 쓴다. 지면에 놓는 로봇이라 roll/pitch를 줄 이유가 없고,
        yaw만 쓰면 .wbt의 `rotation 0 0 1 <yaw>` 표기와 그대로 맞아떨어진다.

        🚨 주입 시점에는 **항상** `synchronization FALSE`로 넣는다. 이유가 있다.

        `<extern>` 컨트롤러는 synchronization TRUE면 Webots가 매 스텝 그 컨트롤러의
        응답을 기다린다. 로봇을 주입하는 순간부터 뇌가 접속할 때까지 몇 초간
        **시뮬레이션 전체가 멈춘다.** 그런데 소환기 자신도 같은 시뮬레이션에서
        step()을 돌고 있으므로 같이 멈추고, 뇌가 끝내 안 뜨면 롤백 감시 타이머조차
        돌지 못해 영원히 굳는다 (헤드리스 Webots로 실측 확인).

        다만 FALSE로 **계속 두면** 뇌가 느릴 때 제어 주기가 물리 주기와 어긋난다.
        지상 로봇은 그래도 버티지만 드론은 못 버틴다 — drone_setup.md 가 그 이유를
        미리 적어 뒀다: "Webots가 매 스텝 step()을 직접 호출하므로 통신 지연이 있어도
        제어 주기가 깨지지 않는다. 드론처럼 루프가 끊기면 추락하는 기체를 <extern>으로
        돌릴 수 있는 이유다." FALSE 는 정확히 그 보장을 없앤다.

        실제로 로봇 6대의 뇌를 한 컨테이너에서 돌리자 소환된 드론이 이륙하지 못하고
        바닥(z≈0.03)에 누워 미끄러졌다. 짐벌 경고에 찍힌 롤 각속도 14 rad/s 는
        "호버링"이 아니라 "뒹구는" 값이다.

        그래서 `needs_sync` 인 로봇은 **뇌가 붙은 것을 확인한 뒤** 이 필드를 TRUE로
        되돌린다 (spawn_supervisor._arm_rollback). 주입 순간의 멈춤은 피하고 비행에
        필요한 보장은 되찾는다.

        `needs_def`인 로봇은 `DEF <이름>`을 붙인다. Spot이 그렇다 — spot_driver가
        Supervisor로 자기 몸 노드를 이름으로 찾아야 하는데, DEF가 없으면 찾을 수단이 없고
        모두 같은 DEF면 2대째가 남의 몸을 잡는다.
        """
        prefix = f'DEF {self.def_name(robot_id)} ' if self.needs_def else ''
        return (
            f'{prefix}{self.proto} {{\n'
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
    # 🚁 드론만 True. 자세 루프가 매 물리 스텝 돌지 않으면 뒤집혀 추락한다.
    #    지상 로봇은 제어 주기가 느슨해도 넘어지지 않으므로 False 로 둔다
    #    (동기화를 켜면 그 로봇의 뇌가 죽을 때 시뮬 전체가 멈추는 대가가 있다).
    needs_sync=True,
)

# ---------------------------------------------------------------------------
# UGV와 Spot은 센서가 월드 파일에 인라인으로 박혀 있었다(UGV는 bodySlot 85줄,
# Spot은 middleExtension 40줄). 그대로는 스폰 문자열을 만들 수 없어서 센서를 품은
# 래퍼 PROTO를 만들었다. 그래서 여기서 가리키는 PROTO는 원본이 아니라 래퍼다.
#
#   ../protos/SummitXlSteelSensorized.proto
#   ../protos/SpotSensorized.proto
#
# 둘 다 월드에 IMPORTABLE EXTERNPROTO로 선언돼 있어야 한다.
# ---------------------------------------------------------------------------
UGV = RobotType(
    key='ugv',
    proto='SummitXlSteelSensorized',
    id_prefix='ugv',
    spawn_z=0.12,
    footprint_radius=0.47,   # 0.72 x 0.61 m
    default_clearance=0.75,
    brain_package='webots_python',
    brain_launch='single_ugv.launch.py',
    has_map=True,
)

SPOT = RobotType(
    key='spot',
    proto='SpotSensorized',
    id_prefix='spot',
    spawn_z=0.624,
    footprint_radius=0.6,    # 1.1 x 0.5 m
    default_clearance=0.9,
    brain_package='webots_spot',
    brain_launch='single_spot_launch.py',
    has_map=True,
    # spot_driver.py 가 Supervisor 로 자기 몸 노드를 DEF 이름으로 찾는다
    # (translation/rotation 을 읽고 쓰기 위해). UGV/드론 드라이버는 디바이스만 쓰므로
    # DEF 가 필요 없다. 뇌에는 ROBOT_DEF 환경 변수로 같은 이름을 넘긴다.
    needs_def=True,
)

ROBOT_TYPES = {t.key: t for t in (DRONE, UGV, SPOT)}

# 소환할 때 쓰지는 않지만 씬 트리에서 로봇으로 인식해야 하는 PROTO 이름들.
#
# 🚨 이게 없으면 조용히 깨진다. my_world.wbt 의 ugv1/ugv2/spot1 은 래퍼가 아니라
#    **원본 PROTO**로 놓여 있다. 이 목록에서 빠지면 스캔이 그 셋을 못 찾아서
#      - 자동 채번이 ugv1 을 다시 발급하고 (이름 충돌)
#      - 겹침 검사가 기존 로봇을 무시해 그 위에 소환한다
#    Phase C 에서 정적 로봇을 걷어내도, 예전 월드를 여는 경우가 있으니 남겨 둔다.
LEGACY_ROBOT_PROTOS = frozenset({'SummitXlSteel', 'Spot'})

# 씬 트리에서 "이건 우리 로봇이다"라고 알아보는 데 쓰는 PROTO 이름 전체.
# 자동 채번과 로봇 간 간격 검사가 이 목록에 의존한다.
KNOWN_ROBOT_PROTOS = (
    frozenset(t.proto for t in ROBOT_TYPES.values()) | LEGACY_ROBOT_PROTOS)
