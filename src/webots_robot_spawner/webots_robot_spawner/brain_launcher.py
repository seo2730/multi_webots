"""소환된 로봇의 "뇌"(ROS 2 노드들)를 띄우고 거둔다.

몸(Webots 노드)과 뇌(driver + SLAM + Nav2)는 1:1이어야 하지만, 뇌와 컨테이너는
그럴 이유가 없다. 이 파일은 그 사실을 코드로 굳힌 곳이다 — 뇌를 **컨테이너가 아니라
프로세스 단위로** 다룬다.

기존 정적 로봇들과 완전히 같은 런치 파일을 쓴다. 다른 점은 `docker-compose`가
아니라 여기서 환경 변수를 주입한다는 것뿐이다. 런치 파일들이 이미
`os.environ.get('ROBOT_ID')`로 이름을 받게 돼 있어서 런치 파일 수정 없이 붙는다.

DockerLauncher는 일부러 만들지 않았다. 컨테이너 안에서 docker 소켓을 부르려면
경로가 플랫폼마다 달라(Linux 유닉스 소켓 / Windows 네임드 파이프 / Mac Desktop VM)
크로스 플랫폼 전제가 깨진다. 필요해지면 이 인터페이스에 클래스만 더하면 된다.
"""

import os
import signal
import subprocess
from dataclasses import dataclass
from pathlib import Path


@dataclass
class BrainHandle:
    """띄운 뇌 하나. 롤백과 상태 조회에 쓴다."""

    robot_id: str
    process: subprocess.Popen
    log_path: Path

    def is_alive(self) -> bool:
        return self.process.poll() is None

    @property
    def returncode(self):
        return self.process.poll()


class LocalProcessLauncher:
    """같은 컨테이너 안에서 `ros2 launch`를 자식 프로세스로 띄운다."""

    def __init__(self, node, log_dir: str = '/tmp/spawned_robots'):
        self._node = node
        self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)

    def launch(self, robot_id, robot_type, x, y, yaw) -> BrainHandle:
        """뇌를 띄운다. 실행 자체가 실패하면 예외가 그대로 올라간다."""
        env = dict(os.environ)
        env.update({
            'ROBOT_ID': robot_id,
            # 맵 병합용 초기 위치. robot_registrar가 이 값을 읽어 마스터에 알린다.
            'ROBOT_INIT_X': f'{x:.6f}',
            'ROBOT_INIT_Y': f'{y:.6f}',
            'ROBOT_INIT_YAW': f'{yaw:.6f}',
            # 드라이버의 동기화 모드. 로봇 종류가 요구하는 값을 그대로 쓴다.
            #
            # 드론은 자세 루프가 매 물리 스텝 돌아야 해서 True 여야 한다. 지상 로봇은
            # False 로 둔다 — 동기화를 켜면 그 뇌가 죽을 때 시뮬 전체가 멈추는데,
            # 지상 로봇은 제어 주기가 느슨해도 넘어지지 않으므로 그 대가를 치를 이유가 없다.
            #
            # 주의: Webots 노드 쪽 synchronization 은 주입 시점엔 항상 FALSE 이고
            # (뇌 접속 전에 시뮬이 멈추는 것을 막기 위해), 뇌가 붙은 뒤 소환기가
            # needs_sync 로봇에 대해 TRUE 로 되돌린다. 드라이버는 처음부터 동기
            # 모드로 떠 있어도 문제가 없다 — 노드가 FALSE 인 동안 step() 이 즉시
            # 반환할 뿐이다.
            'ROBOT_SYNCHRONIZATION': 'true' if robot_type.needs_sync else 'false',
        })

        # Spot 처럼 Supervisor 로 자기 몸 노드를 찾는 드라이버를 위해, 씬 트리에 붙인
        # DEF 이름을 그대로 알려준다. 이게 어긋나면 드라이버가 남의 몸을 잡는다.
        if robot_type.needs_def:
            env['ROBOT_DEF'] = robot_type.def_name(robot_id)

        cmd = ['ros2', 'launch', robot_type.brain_package, robot_type.brain_launch]
        log_path = self._log_dir / f'{robot_id}.log'

        # 로그를 파일로 뺀다. 한 컨테이너에서 여러 뇌가 도는 순간
        # 표준출력이 섞여서 어느 로봇이 뭘 말하는지 알 수 없게 된다.
        log_file = open(log_path, 'ab', buffering=0)
        log_file.write(
            f'\n=== {robot_id} ({robot_type.key}) '
            f'@ ({x:.3f}, {y:.3f}, yaw {yaw:.3f}) ===\n'.encode())

        try:
            process = subprocess.Popen(
                cmd,
                env=env,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                # 별도 세션으로 띄워야 종료할 때 ros2 launch가 만든 손자 프로세스들까지
                # 프로세스 그룹 통째로 정리할 수 있다.
                start_new_session=True,
            )
        finally:
            # Popen이 fd를 복제했으므로 부모 쪽은 닫아도 된다.
            log_file.close()

        self._node.get_logger().info(
            f'[{robot_id}] 뇌 실행: {" ".join(cmd)} (pid {process.pid}, 로그 {log_path})')
        return BrainHandle(robot_id=robot_id, process=process, log_path=log_path)

    def terminate(self, handle: BrainHandle, timeout: float = 10.0):
        """뇌를 정리한다. 롤백 경로에서만 쓴다 (사용자용 despawn은 아직 없다)."""
        if not handle.is_alive():
            return
        pgid = os.getpgid(handle.process.pid)
        os.killpg(pgid, signal.SIGTERM)
        try:
            handle.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self._node.get_logger().warn(
                f'[{handle.robot_id}] SIGTERM에 응답이 없어 강제 종료합니다')
            os.killpg(pgid, signal.SIGKILL)
            handle.process.wait(timeout=timeout)
