# 실측 검증 — Scout Mini + MID-70 로봇 모델 (단계 3)

**측정일**: 2026-07-25
**환경**: gz-sim 8.11.0, GTX 1660 SUPER (580.173.02, ogre2), Ubuntu 24.04
**재현**: `bash scripts/03_verify_robot.sh` / `sim/models/scout_mini_mid70/`

## 1. 모델 로드
로드 에러 0. 링크 12개(섀시 + 4륜 + 마스트 + livox_frame + imu + navsat + 카메라 3),
조인트 9개, 센서 6종 전부 인스턴스화됨.

## 2. 센서 토픽 (전부 발행 확인)
| 토픽 | 내용 |
|---|---|
| `/livox/points_raw/points` | MID-70 상당 gpu_lidar PointCloud2 |
| `/imu` | 200 Hz IMU |
| `/navsat` | 5 Hz GNSS |
| `/cam/left/image`·`/cam/right/image` | 수관 촬영 1920×1080 |
| `/cam/forward/points`·`/depth_image` | 전방 RGB-D |
| `/odom`·`/joint_states`·`/tf` | DiffDrive 오도메트리 |

## 3. 주행 검증 (설계서 §5.3 게이트)
- **직진 1.0 m/s**: odom x 증가, **y 드리프트 3e-9 m (≈0)** → 윤거 0.490 정확, 곡률 오차 없음
- **제자리회전 0.5 rad/s**: x·y 불변, yaw 변화 → 스키드스티어 in-place 회전 정상

## 4. RTF 실측
| 구성 | RTF | 비고 |
|---|---|---|
| 로봇+센서만, 평지 | **~1.00** | 카메라 3 + gpu_lidar + IMU 부하가 단독으로는 실시간 유지 |
| **로봇+센서+과수원 1,233 엔티티, 헤드리스** | **0.55~0.84** | 하강 추세. 설계 목표 0.8~1.0 하단 |

**미적용 최적화 레버 (설계서 §9)** — 통합 RTF를 끌어올릴 여지:
- Levels(`--levels`) 미적용 → 전 나무가 상주 중
- 수관 카메라 2대가 1920×1080 → 촬영 시에만 켜고 평시 720p/저Hz로 낮출 수 있음
- GT 카메라(seg/bbox)는 아직 미부착 → 데이터 수집 시 1~2 Hz 트리거로 제한 예정
- ros_gz_bridge 미적용 상태의 수치 → 브리지 부하는 별도 (설계서 §9 issue #368)

**판정**: 로봇 모델은 기능적으로 완전. 통합 RTF는 워크 가능 범위이나,
데이터 수집/Nav2 단계 전에 §9 레버로 0.8+ 확보가 필요하다.
