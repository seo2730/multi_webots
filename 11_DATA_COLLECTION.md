# 11. 카메라-라이다 데이터 수집 (KITTI 변환)

> 📖 [책 목차](Readme.md#-목차) · ← [10. 맵 병합](10_MAP_MERGE.md)

Webots 시뮬레이션에서 **카메라 이미지 + 라이다 포인트클라우드 + 3D 라벨**을 뽑아
KITTI 포맷 데이터셋으로 만드는 경로. 3D 객체 검출 모델(SparseLIF 계열) 학습용으로
만들었다.

담당 패키지: [src/webots_data_collection/](src/webots_data_collection/)

> ⚠️ 이 경로는 **로봇 소환·편대 구조가 들어오기 전에 만들어졌다.** 현재 편대
> (`ugv1`/`ugv2`/...)와 별개로 도는 흐름이고, 아직 그쪽에 맞춰 정리되지 않은 부분이
> 남아 있다([5절](#5-현재-구조와-어긋난-부분)). 쓰기 전에 그 장을 먼저 읽는 게 좋다.

## 목차
- [1. 무엇이 나오나](#1-무엇이-나오나)
- [2. 수집 노드](#2-수집-노드)
- [3. KITTI 변환](#3-kitti-변환)
- [4. 실행 방법](#4-실행-방법)
- [5. 현재 구조와 어긋난 부분](#5-현재-구조와-어긋난-부분)
- [6. 학습에 쓰기 전에 확인할 것](#6-학습에-쓰기-전에-확인할-것)
- [7. 파일 맵](#7-파일-맵)

---

## 1. 무엇이 나오나

수집 노드가 **동기화된 한 프레임**을 받을 때마다 세 파일을 같은 번호로 떨군다.

```
src/webots_data_collection/
├── dataset_output/          ← 수집 노드가 쓰는 원본
│   ├── image_2/000001.png       카메라 이미지 (bgr8)
│   ├── velodyne/000001.bin      포인트클라우드 (float32 x,y,z)
│   └── label_2/000001.txt       원본 라벨 (KITTI 아님)
└── training/                ← 변환기가 쓰는 KITTI 규격
    ├── label_2/000001.txt       KITTI 15칼럼 라벨
    └── calib/000001.txt         P0~P3 / R0_rect / Tr_velo_to_cam / Tr_imu_to_velo
```

**원본 라벨 한 줄의 형식** (객체 하나):

```
<모델이름> <xmin> <ymin> <xmax> <ymax> <absX> <absY> <absZ> <relX> <relY> <relZ>
```

- 앞의 4개는 이미지 좌표계 2D 바운딩 박스(픽셀)
- `abs*`는 로봇의 `{ns}/odom` 프레임으로 변환한 **절대 위치**(tf2)
- `rel*`는 카메라 기준 **상대 위치**. 학습에 실제로 쓰는 값은 이쪽이다

---

## 2. 수집 노드

[cam_lidar_data_collector.py](src/webots_data_collection/webots_data_collection/cam_lidar_data_collector.py)
(`SparseLIFDataCollector`).

**세 토픽을 `ApproximateTimeSynchronizer`로 묶는다** (`queue_size=100`, `slop=0.5`):

| 토픽 | 타입 | 출처 |
|---|---|---|
| `/{ROBOT_ID}/rgb_camera/image_color` | `sensor_msgs/Image` | Webots 카메라 |
| `/{ROBOT_ID}/rgb_camera/recognitions/webots` | `webots_ros2_msgs/CameraRecognitionObjects` | Webots **Recognition 노드** |
| `/{ROBOT_ID}/Velodyne_VLP_16/point_cloud` | `sensor_msgs/PointCloud2` | Velodyne VLP-16 |

라벨의 정답은 Webots의 **Recognition 기능**에서 온다. 카메라 노드에 `recognition`이
설정돼 있어야 하고, 인식 대상 오브젝트에는 `recognitionColors`가 있어야 한다.
`recognitions` 토픽이 비면 프레임이 아예 안 쌓인다.

**좌표 변환은 tf2에게 맡긴다.**

```python
transform = tf_buffer.lookup_transform(f'{robot}/odom', f'{robot}/rgb_camera', stamp)
abs_p = do_transform_point(p_stamped, transform).point
```

> 예전에는 여기서 축을 손으로 돌렸는데(`x←z, y←-x, z←-y`) TF가 이미 그 회전을 담고
> 있어서 **이중 회전**이 됐다. 지금은 원본 좌표를 그대로 넣고 TF만 적용한다.
> 코드에 그 흔적이 주석으로 남아 있다.

TF 조회에 실패하면 그 프레임은 **조용히 버려진다**(경고만 찍고 `return`). 파일 번호는
증가하지 않으므로 결번은 생기지 않는다.

---

## 3. KITTI 변환

[webots2kitti.py](src/webots_data_collection/scripts/webots2kitti.py) — ROS와 무관한
순수 파이썬 스크립트다. 컨테이너 밖 호스트에서 돌려도 된다.

### 캘리브레이션은 계산해서 만든다

Webots는 센서가 고정이라 프레임마다 같은 값을 쓴다. 스크립트 상단 상수에서 나온다:

| 상수 | 값 | 뜻 |
|---|---|---|
| `CAM_WIDTH` / `CAM_HEIGHT` | 1280 / 720 | 카메라 해상도 |
| `CAM_FOV_RAD` | 1.05 | 수평 화각(rad) |
| `DX` / `DY` / `DZ` | 0.3585 / 0.0 / -0.135 | 라이다 → 카메라 평행이동(m) |
| `DEFAULT_DIMS` | `wooden_box: [0.6, 0.6, 0.6]` | 객체 크기 H, W, L (인식 결과에 크기가 없어서 상수로 준다) |

- `P2` = FOV와 해상도로 만든 3×4 내부 파라미터 (`f = (W/2) / tan(FOV/2)`)
- `R0_rect` = 단위행렬
- `Tr_velo_to_cam` = **회전 없이 평행이동만** (Webots에서는 축이 나란하다는 가정)
- `P0`/`P1`/`P3`, `Tr_imu_to_velo` = 더미

🚨 **이 상수들은 월드의 카메라 노드와 일치해야 한다.** 해상도나 FOV를 PROTO에서 바꿨다면
여기도 같이 고쳐야 하고, 안 그러면 학습은 도는데 3D 투영이 조용히 틀어진다.

### 라벨 15칼럼

```
class truncated occluded alpha xmin ymin xmax ymax h w l x y z rotation_y
      0.00      0        0.00                                  0.00
```

`truncated`/`occluded`/`alpha`/`rotation_y`는 전부 0으로 채운다. **회전 정보가 없다** —
Webots Recognition이 축정렬 박스만 주기 때문이다. 방향까지 학습시키려면 supervisor로
객체의 실제 yaw를 따로 뽑아야 한다.

위치는 `abs*`가 아니라 **`rel*`(카메라 기준 상대좌표)를 쓴다.** KITTI가 카메라 좌표계를
전제하기 때문이다.

---

## 4. 실행 방법

전용 compose가 있다 (현재 **맥 기준**, `docker-configs/mac/Dockerfile` 이미지를 재사용).

```bash
docker compose -f docker-configs/camera-lidar/docker-compose.yml up --build -d
```

이 compose의 `ugv1` 서비스에는 `command:`가 없다 — **컨테이너만 띄우고 런치는 직접**
실행하는 구조다.

```bash
docker exec -it camera_lidar_ugv1_mac bash
source /ros2_ws/install/setup.bash
ros2 launch webots_data_collection cam_lidar_data_collector.launch.py

# 맥/도커에서 /clock 이 안 도는 환경이면
ros2 launch webots_data_collection cam_lidar_data_collector.launch.py use_clock_bridge:=true
```

이 런치가 띄우는 것: `robot_state_publisher` + `webots_ros2_driver` + 수집 노드
(+ 선택적으로 `sim_clock_bridge`). **SLAM·Nav2는 띄우지 않는다** — 수집에는 필요 없다.

로봇을 움직여야 프레임이 다양해지므로 텔레옵을 같이 쓴다
([04장 6절](04_UGV_SETUP.md#6-키보드-조종)).

변환은 컨테이너 밖에서:

```bash
cd src/webots_data_collection
python3 scripts/webots2kitti.py
```

> 스크립트가 **상대경로**(`./dataset_output`, `./training`)를 쓰므로 반드시
> `src/webots_data_collection`에서 실행해야 한다.

---

## 5. 현재 구조와 어긋난 부분

수집 경로는 편대·소환 구조가 들어오기 전 상태로 남아 있다. 손대기 전에 알아 둘 것:

| 어긋난 곳 | 지금 상태 | 영향 |
|---|---|---|
| `ROBOT_ID` | camera-lidar compose는 `SummitXLSteel` | 현재 편대의 `ugv1`과 이름이 다르다. 토픽 경로가 전부 달라진다 |
| 저장 경로 | `/ros2_ws/src/webots_data_collection/dataset_output` **하드코딩** | 컨테이너 밖 경로로 바꾸려면 코드 수정 필요. 소스를 마운트해 쓰므로 호스트에 그대로 쌓인다 |
| 로봇 배치 | 월드에 로봇이 박혀 있던 시절 전제 | 지금은 소환기가 넣으므로, 수집용으로도 편대를 먼저 올려야 한다 |
| OS | 맥 이미지 재사용 | 윈도우/우분투용 compose는 없다. 만들려면 해당 Dockerfile로 같은 서비스를 복제 |
| `use_sim_time` | 수집 노드는 `true` | `/clock`이 안 돌면 동기화가 성립하지 않아 **한 프레임도 안 쌓인다** ([04장 7절](04_UGV_SETUP.md#7-알아-둘-함정)) |

---

## 6. 학습에 쓰기 전에 확인할 것

실제 KITTI 로더에 그대로 물리기 전에 걸리는 것들이다. **아직 정리하지 않았다.**

1. **`.bin`이 3채널이다.** KITTI velodyne 파일은 `x, y, z, intensity` **4 float**인데
   여기서는 x, y, z만 쓴다. 대부분의 로더가 `reshape(-1, 4)`를 하므로 그대로 넣으면
   포인트가 뒤섞인다. 반사강도 열을 0으로라도 채우거나 로더 쪽을 고쳐야 한다.
2. **클래스 이름에 공백이 있으면 파싱이 밀린다.** 변환기는 `data[1] == "box"`일 때만
   두 토큰을 붙여 클래스로 보고, 나머지 인덱스는 고정이다. Webots 모델 이름이
   `wooden box`가 아닌 다른 형태(한 토큰, 또는 세 토큰)면 좌표 열을 잘못 읽는다.
   **에러 없이 조용히 틀린다.**
3. **`Tr_velo_to_cam`에 회전이 없다.** "Webots에서는 축이 나란하다"는 가정인데, URDF의
   `rgb_camera` 조인트에는 `rpy="-1.5708 0 -1.5708"` 회전이 들어 있다. 라이다-카메라
   투영을 실제로 그려서 맞는지 확인해야 한다.
4. **객체 크기가 상수다.** `wooden_box` 외의 클래스는 전부 1×1×1 m로 나간다.
5. **회전각(`rotation_y`)이 전부 0.** 방향 학습은 불가능하다.

---

## 7. 파일 맵

| 파일 | 역할 |
|---|---|
| [webots_data_collection/cam_lidar_data_collector.py](src/webots_data_collection/webots_data_collection/cam_lidar_data_collector.py) | 세 토픽 동기화 → 이미지/포인트/원본 라벨 저장 |
| [launch/cam_lidar_data_collector.launch.py](src/webots_data_collection/launch/cam_lidar_data_collector.launch.py) | 드라이버 + 수집 노드 (+ 선택적 시계 브릿지) |
| [scripts/webots2kitti.py](src/webots_data_collection/scripts/webots2kitti.py) | 원본 라벨 → KITTI 라벨 + calib 생성 |
| [docker-configs/camera-lidar/docker-compose.yml](docker-configs/camera-lidar/docker-compose.yml) | 수집 전용 컨테이너 (맥 이미지 재사용) |
| `dataset_output/` | 수집 원본 (git에 커밋되어 있다) |
| `training/` | KITTI 변환 결과 |

### 관련 문서

- [04_UGV_SETUP.md](04_UGV_SETUP.md) — 센서 사슬과 `/clock` 함정
- [Readme.md](Readme.md) — 전체 실행 방법
- [01_INTERFACES.md](01_INTERFACES.md) — 토픽 총람

---

← [10. 맵 병합](10_MAP_MERGE.md) | [📖 책 목차](Readme.md#-목차)
