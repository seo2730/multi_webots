# 이 저장소에서 작업할 때

Webots 다중 로봇 시뮬레이션(ROS 2 Humble + Docker). 문서가 상세하므로 **코드를 읽기
전에 해당 문서를 먼저 본다.** 문서 지도는 [Readme.md](Readme.md#-문서-지도)에 있다.

## 구조 한 줄 요약

- **월드**에는 로봇이 없다. 환경 + `spawn_supervisor`만 있고, 로봇은 **소환**으로 들어온다
- **편대**는 `src/webots_robot_spawner/config/fleet/*.yaml`이 정의하고, compose는 거기서 **생성**한다
- **몸**(Webots 노드)은 fleet 컨테이너가 주입하고, **뇌**(driver+SLAM+Nav2)는 로봇별 컨테이너가 돌린다
- 좌표계: 모든 로봇의 `{ns}/map` 원점 ≈ Webots 월드 원점 (드라이버가 GPS 절대좌표를 odom으로 쓴다)
- **로봇 3종은 겉만 같다.** `cmd_vel`은 셋 다 m/s지만 내부 환산이 다르고(Spot은 비선형
  보폭, 드론은 자세 목표), Nav2 파라미터도 Spot만 `nav2_spot.yaml`로 갈라져 있다.
  드론은 SLAM 대신 층별 매퍼를 쓴다 — 한 로봇에서 통한 값을 다른 로봇에 그대로 옮기지 않는다

## 절대 규칙

1. **크로스 플랫폼 전제.** Windows / macOS / Ubuntu 모두에서 동작해야 한다. OS별 차이는
   `docker-configs/{ubuntu,windows,mac}/` 층에서 흡수하고, 코드에 분기를 넣지 않는다.
   Docker 소켓처럼 플랫폼마다 경로가 다른 것에 의존하지 않는다.
2. **compose를 손으로 고치지 않는다.** `# >>> FLEET GENERATED` 마커 안쪽은
   `scripts/gen_fleet_compose.py`가 매니페스트에서 생성한다. 3벌(ubuntu/windows/mac) 모두.
3. **월드를 Webots GUI에서 저장했으면 `git diff`를 본다.** 저장할 때마다
   - `EXTERNPROTO` 경로가 `D:/Document/...` 절대경로로 바뀌고
   - 컨트롤러가 `"<none>"`으로 바뀐다
   그대로 커밋하면 다른 컴퓨터에서 월드가 안 열린다.
4. **새 노드에는 `use_sim_time: True`.** 빠뜨리면 벽시계 스탬프 때문에 SLAM/tf2가 데이터를
   전부 버린다. 예외는 소환기(실제 시간으로 롤백을 감시해야 한다).
5. **문서를 같이 고친다.** 동작을 바꾸면 해당 문서의 표·트러블슈팅 항목도 갱신한다.
   문서 사이 링크는 앵커까지 맞춘다.

## 환경 특성 (이 PC)

- **호스트에 파이썬이 없다** (Store 스텁만). 파이썬 스크립트는 프로젝트 도커 이미지 안에서
  돌린다 — 셸별 문법은 [WORLD_GEN.md 2장](WORLD_GEN.md#2-os별-실행-방법-중요) 참고.
  간단한 계산·검증 스크립트는 Node로 짜는 편이 빠르다
- **호스트에 Webots 에셋이 없다.** `webots://` 참조는 조용히 실패하므로 메시·텍스처는
  `protos/` 아래 로컬 사본을 상대경로로 참조한다
- 검증은 Webots 헤드리스(`--batch --mode=fast --no-rendering`)로 돌린다

## git

- **`main`에서 직접 작업한다.** 별도 브랜치를 만들지 않는다
- 서브모듈 2개(`src/Webots-SummitXL`, `src/webots_ros2_spot`)를 고쳤다면
  **서브모듈을 먼저 커밋·푸시**한 뒤 본 저장소의 포인터를 올린다
- 커밋 메시지는 한국어 한 줄 (기존 로그 형식을 따른다)

## 디버깅 순서

문제가 생기면 이 순서로 본다 — 대부분 여기서 끝난다.

1. **Webots가 Play(▶) 상태인가** — 멈춰 있으면 `step()`이 안 돌아 아무것도 안 나온다
2. **`ros2 topic hz /clock`** — 0 Hz면 시뮬이 멈췄거나 시계 발행자(`ugv1`)가 없다
3. **QoS** — `TRANSIENT_LOCAL` 토픽을 기본 QoS로 구독하면 **에러 없이** 아무것도 안 온다
4. **`ros2 topic hz`를 믿지 말 것** — 노드 100개가 넘으면 CLI가 거짓말한다. rclpy로 직접 구독
5. 로그: 매니페스트 로봇은 `docker logs -f {ns}_brain_{os}`,
   런타임 소환 로봇은 `fleet` 컨테이너 안 `/tmp/spawned_robots/{ns}.log`
