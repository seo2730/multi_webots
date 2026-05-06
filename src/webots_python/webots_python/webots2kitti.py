import os
import glob
import math
import numpy as np

# ==========================================
# 1. 센서 및 경로 설정 (연구원님 환경에 맞게 수정!)
# ==========================================
# 폴더 경로 설정
INPUT_LABEL_DIR = '../dataset_output/label_2' # Webots에서 수집한 원본 텍스트 폴더
OUTPUT_LABEL_DIR = './training/label_2'       # KITTI 규격으로 저장될 라벨 폴더
OUTPUT_CALIB_DIR = './training/calib'         # 자동 생성될 캘리브레이션 폴더

# 카메라 내부 파라미터 (Intrinsic) 설정
CAM_WIDTH = 1280      # Webots 카메라 가로 해상도
CAM_HEIGHT = 720      # Webots 카메라 세로 해상도
CAM_FOV_RAD = 1.05   # Webots 카메라 Field of View (라디안, 예: 1.047 = 약 60도)

# 라이다 -> 카메라 외부 파라미터 (Extrinsic) 설정
# 로봇 중심 기준으로 센서가 얼마나 떨어져 있는지 (미터 단위)
DX = 0.3585   # 카메라가 35.85cm 앞에 있음
DY = 0.0      # 좌우 중앙 정렬됨
DZ = -0.135   # 카메라가 13.5cm 아래에 있음

# 객체 기본 크기 (수집된 텍스트에 크기가 없다면 임의로 지정. Height, Width, Length)
DEFAULT_DIMS = {"wooden_box": [0.6, 0.6, 0.6]} 

# ==========================================
# 2. 캘리브레이션 매트릭스 계산 로직
# ==========================================
def calculate_intrinsic_p2(width, height, fov_rad):
    """FOV와 해상도를 이용해 3x4 P2(카메라 렌즈 행렬)를 계산합니다."""
    # 초점 거리(focal length) 계산
    f = (width / 2.0) / math.tan(fov_rad / 2.0)
    cx = width / 2.0
    cy = height / 2.0
    
    P2 = np.zeros((3, 4))
    P2[0, 0] = f
    P2[1, 1] = f
    P2[0, 2] = cx
    P2[1, 2] = cy
    P2[2, 2] = 1.0
    return P2

def calculate_extrinsic_tr(dx, dy, dz):
    """LiDAR에서 Camera로의 변환 3x4 행렬 (Tr_velo_to_cam) 계산"""
    # Webots 환경에서는 보통 축이 나란하므로 회전은 Identity, 위치(Translation)만 적용합니다.
    Tr = np.eye(4)
    Tr[0, 3] = dx
    Tr[1, 3] = dy
    Tr[2, 3] = dz
    return Tr[:3, :] # 3x4 행렬로 반환

def matrix_to_string(matrix):
    """numpy 행렬을 스페이스바로 구분된 한 줄의 텍스트로 변환"""
    return ' '.join([f"{val:.6e}" for val in matrix.flatten()])

# ==========================================
# 3. 데이터 변환 실행
# ==========================================
def main():
    os.makedirs(OUTPUT_LABEL_DIR, exist_ok=True)
    os.makedirs(OUTPUT_CALIB_DIR, exist_ok=True)

    # 1. 캘리브레이션 행렬 미리 계산 (Webots는 센서가 고정이므로 한 번만 계산하면 됨)
    P2_matrix = calculate_intrinsic_p2(CAM_WIDTH, CAM_HEIGHT, CAM_FOV_RAD)
    Tr_matrix = calculate_extrinsic_tr(DX, DY, DZ)
    
    # KITTI calib 포맷용 문자열 생성 (나머지는 더미값 처리)
    P0_str = P1_str = P3_str = matrix_to_string(np.zeros((3, 4)))
    P2_str = matrix_to_string(P2_matrix)
    R0_rect_str = matrix_to_string(np.eye(3))
    Tr_velo_to_cam_str = matrix_to_string(Tr_matrix)
    Tr_imu_to_velo_str = matrix_to_string(np.eye(3, 4)) # IMU->Velo는 현재 미사용
    
    calib_content = (
        f"P0: {P0_str}\nP1: {P1_str}\nP2: {P2_str}\nP3: {P3_str}\n"
        f"R0_rect: {R0_rect_str}\n"
        f"Tr_velo_to_cam: {Tr_velo_to_cam_str}\n"
        f"Tr_imu_to_velo: {Tr_imu_to_velo_str}\n"
    )

    raw_files = glob.glob(os.path.join(INPUT_LABEL_DIR, '*.txt'))
    print(f"🚀 총 {len(raw_files)}개의 프레임 변환을 시작합니다...")

    for file_path in raw_files:
        filename = os.path.basename(file_path)
        
        # [A] Calib 파일 생성 (모든 프레임에 동일한 카메라/라이다 물리값 복사)
        with open(os.path.join(OUTPUT_CALIB_DIR, filename), 'w') as f:
            f.write(calib_content)

        # [B] Label 파일 변환
        out_lines = []
        with open(file_path, 'r') as f:
            for line in f.readlines():
                data = line.strip().split()
                if len(data) < 10: continue
                
                # 원본 파싱: "wooden box 829.5 308.5 1086.5 465.5 absX absY absZ relX relY relZ"
                # (클래스 이름에 띄어쓰기가 있으면 합치기)
                obj_class = f"{data[0]}_{data[1]}" if data[1] == "box" else data[0]
                
                # BBox
                xmin, ymin, xmax, ymax = data[2], data[3], data[4], data[5]
                
                # 우리는 절대좌표(abs)는 버리고 학습에 필요한 상대좌표(rel)만 씁니다.
                rel_x, rel_y, rel_z = data[9], data[10], data[11]
                
                # 기본 크기 (H, W, L)
                h, w, l = DEFAULT_DIMS.get(obj_class, [1.0, 1.0, 1.0])
                
                # KITTI 포맷 규격화 (총 15개 칼럼)
                # Class / Truncated / Occluded / Alpha / Bbox(4) / Dims(3) / Loc(3) / Yaw / Score(옵션)
                kitti_line = f"{obj_class} 0.00 0 0.00 {xmin} {ymin} {xmax} {ymax} {h} {w} {l} {rel_x} {rel_y} {rel_z} 0.00\n"
                out_lines.append(kitti_line)

        with open(os.path.join(OUTPUT_LABEL_DIR, filename), 'w') as f:
            f.writelines(out_lines)

    print(f"✅ 변환 완료! {OUTPUT_LABEL_DIR} 와 {OUTPUT_CALIB_DIR} 폴더를 확인하세요.")

if __name__ == '__main__':
    main()