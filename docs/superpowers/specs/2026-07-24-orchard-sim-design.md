# 과수원 시뮬레이션 환경 설계서

**프로젝트**: YBNML — 과수원 무인이동체 데이터 수집 및 이미지 기반 농업 솔루션
**작성일**: 2026-07-24
**단계**: Phase 1 — 시뮬레이션 환경 구축 (통합 관제 시스템은 Phase 2)

---

## 1. 목표와 범위

사과 과수원(노지)을 Gazebo에 재현하고, AgileX SCOUT MINI에 Livox MID-70을 탑재한 무인이동체가
그 안을 자율주행하면서 영상·점군 데이터를 수집하도록 만든다. 수집된 데이터는 착과 수 카운팅과
병해 증상 탐지에 쓰인다.

### 범위에 포함

- 절차적으로 생성되는 사과 과수원 월드(지형·수목·과실·지주)
- SCOUT MINI + MID-70 로봇 모델 (URDF/SDF, 물리 파라미터 포함)
- FAST-LIO2 기반 위치추정 + Nav2 자율주행
- 정답 라벨(세그멘테이션·바운딩박스·과실별 3D 위치) 자동 추출 파이프라인
- 코드 진행에 따라 갱신되는 개발 매뉴얼 웹 문서

### 범위에서 제외

- 통합 관제 시스템 (Phase 2)
- 실물 하드웨어 연동 — **시뮬레이션 전용으로 확정** (§2 D2)
- 실제 병해 데이터셋 기반 모델 학습 — 나중에 접붙일 수 있는 구조만 만든다 (§2 D4)

---

## 2. 확정된 결정

| ID | 항목 | 결정 | 근거 |
|---|---|---|---|
| D1 | 시뮬레이터 | **Gazebo Harmonic (gz-sim 8.11.0)** | Isaac Sim은 RT 코어 부재 + VRAM 6GB < 최소 16GB로 불가. Gazebo Classic 11은 2025-01-29 EOL이고 noble 바이너리가 존재하지 않음 |
| D2 | 미들웨어 | **ROS 2 Jazzy Jalisco** (EOL 2029-05) | noble에서 Tier 1인 유일한 LTS. Kilted도 Tier 1이지만 2026-11 EOL |
| D3 | 과종/수형 | **사과, 세장방추형** (고밀식) | 사용자 결정. RDA 농사로 기준 3.5×1.5 m |
| D4 | 분석 목표 | **착과 카운팅 + 병해 증상 탐지** | 사용자 결정. 종 수준 진단이 아닌 증상 탐지로 범위 한정 (§8) |
| D5 | 주행 | **Nav2 완전 자율주행** | 사용자 결정 |
| D6 | SLAM | **FAST-LIO2** | 사용자 결정. MID-70 같은 협FOV 솔리드스테이트가 설계 대상 |
| D7 | LiDAR 구성 | **MID-70 단독** (360° 라이다 추가 안 함) | 사용자 결정. 대체 완화책은 §7.3 |
| D8 | 하드웨어 | **시뮬레이션 전용** | 사용자 결정. MID-70의 Jazzy 드라이버 부재 문제가 무관해짐 |
| D9 | 구현 순서 | **월드 → 로봇 → 자율주행 → 데이터** | 사용자 결정 |
| D10 | 실데이터 | **당분간 사용 안 함** | 사용자 결정. AI Hub 신청은 보류 |
| D11 | 문서 | **claude.ai 웹 아티팩트**, 단계마다 같은 URL 갱신 | 사용자 결정 |

---

## 3. 시스템 아키텍처

```
┌─────────────────────────── Gazebo Harmonic (gz-sim 8) ───────────────────────────┐
│                                                                                   │
│  orchard.sdf  ─ 절차 생성                    scout_mini_mid70.sdf                 │
│   ├ heightmap 지형 120×120 m                  ├ 섀시 + 4륜 (스키드스티어)          │
│   ├ 수목 <include> × N (플랫, 중첩 금지)      ├ livox_mid70   (gpu_lidar)          │
│   ├ 계측블록 과실 <include> × 1,200 (최상위)  ├ imu_link      (imu, 200 Hz)        │
│   ├ 지주 + 와이어                             ├ navsat        (5 Hz)               │
│   └ 지면 2존 (초생/청경)                      ├ cam_canopy_L/R (1920×1080)         │
│                                               ├ cam_forward_rgbd                   │
│                                               └ GT 센서 4종 (seg/panoptic/bbox×2)  │
└───────────────────────────────────────┬───────────────────────────────────────────┘
                                        │  ros_gz_bridge (YAML)
                                        │  /clock ← use_sim_time:=true 전 노드 필수
                    ┌───────────────────┴────────────────────┐
                    ▼                                        ▼
        ┌─────────────────────┐                   ┌──────────────────────┐
        │  livox_sim_bridge   │                   │  dataset_writer      │
        │  (커스텀 노드)       │                   │  RGB+마스크+박스      │
        │  · 원형 FOV 마스킹   │                   │  타임스탬프 일치 검증  │
        │  · CustomMsg 합성    │                   └──────────────────────┘
        └──────────┬──────────┘
                   ▼
        ┌─────────────────────┐     ┌──────────────────────────────────┐
        │  FAST-LIO2          │────▶│  robot_localization              │
        │  /Odometry          │     │  ekf_odom : odom→base_link       │
        └─────────────────────┘     │  ekf_map  : map→odom             │
                   ▲                 └──────────────┬───────────────────┘
        ┌──────────┴──────────┐                    │
        │ row_centerline_node │────────────────────┤  Y·yaw 보정
        │ row_event_node      │────────────────────┘  X 이산 리셋
        └─────────────────────┘
                                        │
                                        ▼
                        ┌───────────────────────────────┐
                        │  Nav2 (Jazzy 1.3.x)           │
                        │  costmap: static + STVL + infl│
                        │  planner: RowCenterline/NavFn │
                        │  controller: RotationShim→MPPI│
                        └──────────────┬────────────────┘
                                       ▼  /cmd_vel  →  gz DiffDrive
```

---

## 4. 과수원 월드 사양

### 4.1 기하 제원

| 파라미터 | 값 | 범위 | 출처 |
|---|---|---|---|
| 열간 거리 | **3.50 m** | 2.80–3.90 | RDA 농사로 cntntsNo=30663 |
| 주간 거리 | **1.50 m** | 1.00–1.80 | 동일 (3.5×1.5 → 190주/10a) |
| 재식 밀도 | 1,900주/ha | | 산술 검산 1000/5.25 = 190 ✓ |
| 수고 | **3.20 m** | 3.0–3.5 | 세장방추형 표준 |
| 수관 폭 | **1.20 m** | 0.90–1.50 | Robinson 0.9–1.2 / 한국사과협회 1.5 |
| **실제 통로 폭** | **2.30 m** | **최악 2.00 m** | 3.5 − 1.2. **Nav2 파라미터를 지배** |
| 수관 하단고 | 0.70 m | 0.60–0.80 | |
| 주간 기부 직경 | 0.070 m | 0.060–0.080 | Cornell 최소 16 mm |
| 주간 선단 직경 | 0.020 m | @3.2 m | 선형 테이퍼 + 축방향 노이즈 σ 0.01 |
| 측지 개수 | **28** | 20–40 | 하수 −10°~−30° |
| 측지 길이 | 0.45 m | 0.30–0.60 | 세장방추형 정의 형질 |
| 측지 방위각 | 황금각 137.5° 층화 | | |
| 지주 (열내) | 3.66 m, 0.70 m 매설 → 2.96 m 노출, ⌀0.075 | **10 m 간격** | Cornell |
| 지주 (말단) | 1.83 m, 외측 30° 경사 | 열당 2개 | |
| 와이어 | 3단, 1.00 / 2.00 / 2.90 m, ⌀2.5 mm | 시각 전용, 충돌 없음 | **낮은 신뢰도 — 추정치** |
| **개발용 규모** | **4행 × 20주 = 80주** | 먼저 이걸로 RTF 측정 | |
| 전체 규모 | 6행 × 30주 = 180주 | | |
| 과수원 면적 | 21.0 × 55.5 m ≈ 0.117 ha | | |
| 선회 공간 | **6.0 m** 양단 | **5.0 m 미만 금지** | *Machines* 11(1):84 — 5 m 미만에서 선회 실패 |
| 열 방향 | 남–북 | | 국내 권장 |
| 주당 과실 수 | **60** | 50–100 | Cornell |
| 과실 직경 | **0.075 m** | 0.060–0.085 | 국내 후지 평균 75 mm |
| 착과 군집 | 단일 75.3%, 나머지 2–4개 | | arXiv:1808.04336 |
| 과실 배치 | 주간축에서 0.15–0.55 m, z 0.80–3.00 m, **엽군 내부** | | |
| **단면 가시 과실 비율 목표** | **55–65%** | 허용 40.85–79.83% | `full_2d` ÷ `visible_2d`로 검증 |

### 4.2 지형

평면 금지. 평평한 지면은 오도메트리·IMU 거동과 LiDAR 지면분할을 실제보다 쉽게 만들어 오도한다.

- **513 × 513, 8-bit 그레이스케일 PNG, 알파 없음** (Perlin 노이즈 → 저역통과)
  - 정사각형 + 2ⁿ+1 + 8-bit 그레이는 **하드 요구사항**. 512×512나 RGBA는 실패한다
- `<size>120 120 1.5</size>` → 120×120 m에 총기복 1.5 m ≈ 1–2% 완경사
- **`<collision>`과 `<visual>`에 동일한 `<heightmap>` 블록**
- `<physics><dart><collision_detector>bullet</collision_detector></dart></physics>`
- 지면 2존: 열간 초생(잔디) / 수관하부 청경(나지) 폭 1.20 m

### 4.3 수목 에셋 파이프라인

**다운로드로 해결되지 않는다.** Gazebo Fuel REST API를 13개 검색어로 직접 조회한 결과
apple/orchard/vineyard/crop 모델 **0개**. 전체 카탈로그의 수목은 4종이고 과실 달린 것은 없다.

Blender 4.x headless `bpy` 스크립트로 생성한다:

```
gen_tree.py --height 3.2 --canopy_w 1.2 --n_feathers 28 --n_apples 60 --seed N
  1. 주간   : 테이퍼 원기둥, 방사 10분할
  2. 측지   : 원기둥 28개, 방사 6분할
  3. 엽군   : 알파 테스트 잎-클러스터 카드 (2048² 아틀라스에 5~20잎/쿼드)
              150~400장/그루 × 2 tris.  개별 잎 카드 금지 (4,000 tris vs 600)
  4. 과실   : icosphere subdiv 1 (80 tris), ⌀0.075
              엽군 내부에 기각 샘플링 → 단면 가시율 55~65% 달성
  5. 익스포트: COLLADA .dae
  6. 정답출력: {tree_id, apple_id, world_xyz, diameter, cluster_id, label_id}
gen_world.py → 플랫 월드 SDF, 그루당 <include> 하나, Label 플러그인 내장
```

**Sapling Tree Gen을 쓰지 않는다.** 자연스러운 분기를 만들지만 세장방추형은 의도적으로 인위적인
수형이라 직접 스크립팅하는 편이 쉽다.

**구조 규칙 — 2026-07-25 이 머신에서 실측으로 확정했다.**
전체 측정 결과: [`docs/findings/2026-07-25-label-instance-separation.md`](../../findings/2026-07-25-label-instance-separation.md)

| SDF 구조 | 인스턴스 분리 |
|---|---|
| 한 모델 / 한 링크 / `<visual>` 3개, 각 visual 에 Label | ❌ 1개로 뭉개짐 |
| 한 모델 / 링크 3개, 각 링크의 visual 에 Label | ❌ 1개로 뭉개짐 |
| 최상위 `<model>` 3개 | ✅ 3개 |
| **최상위 `<include>` 3개** | ✅ **3개** ← 생성기가 쓸 패턴 |
| 부모 모델 안 중첩 `<include>` 3개 | ❌ 1개로 뭉개짐 |

**인스턴스 분리는 최상위(non-nested) 모델 단위로만 일어난다.** 따라서:

- **과실별 인스턴스 ID 가 필요하면 그 과실은 최상위 `<include>` 여야 한다.** 나무 안에 넣을 수 없다
- 나무 몸체(`trunk`/`branch`/`leaf_*`)는 인스턴스가 필요 없으므로 **`<visual>` 단위 Label 로 충분**하다
  (semantic 은 visual 단위로 정상 동작함을 확인)
- **행을 부모 모델로 감싸지 않는다** — 중첩 `<include>` 는 인스턴스를 부모 기준으로 합쳐버린다
- **런타임 스폰 금지** — 스폰된 모델은 라벨을 조용히 잃는다

**2계층 구조가 강제된다:**

| 구역 | 구조 | 얻는 것 | 엔티티 수 |
|---|---|---|---|
| 배경 행 | 나무 = `<include>` 1개, 과실은 메시에 구워 넣음 | semantic 라벨만 | 1/그루 |
| 계측 블록 (20그루) | 나무 `<include>` + **과실마다 최상위 `<include>`** | 과실별 인스턴스·modal/amodal 박스·가림률 | 1+60/그루 → 약 1,200개 |

전체 180그루에 과실별 인스턴스를 적용하면 10,800 엔티티가 되어 드로우콜이 폭발한다.
**측정 없이 확대하지 않는다.**

### 4.4 에셋 라이선스 대장

| 에셋 | 라이선스 | 용도 |
|---|---|---|
| ambientCG (Bark012, Grass004, Ground037) | **CC0** | 수피·잔디·나지 텍스처 |
| Poly Haven | **CC0** | HDRI, 추가 지면 |
| `westonrobot/scout_ros2` URDF/메시 | BSD (Clearpath/Weston) | **출처 표기 필요** |
| `chapulina/Heightmap Bowl` 텍스처 | CC-BY 4.0 | **출처 표기 필요** |
| `tduboudi/IAMPS2019-...-Fruit-Tree` | MIT | 참고 스크립트 |
| `FieldRobotEvent/virtual_maize_field` | **GPL-3.0 ⚠️** | **아키텍처만 참고. 코드 복사 금지** |
| `PlantSimulationLab/Helios` | **GPLv2 ⚠️** | 시각·파라미터 참고만. 파생 메시 배포 금지 |
| `kubja/gazebo-vegetation` | **불명 ⚠️** | **사용 금지** |

---

## 5. 로봇 모델 사양

### 5.1 SCOUT MINI 제원

공식 스펙: 612 × 580 × 245 mm, 26 kg, 적재 10 kg, 최고 2.7 m/s, 최소회전반경 0 m,
최저지상고 115 mm, 등판각 30°, 4 × 150 W BLDC, 24 V/15 Ah.

**`westonrobot/scout_ros2`의 `urdf/scout_mini/*.xacro`에서 가져온다.** Scout Mini 기술서가
존재하는 유일한 ROS 2 저장소이고 수치가 공식 스펙과 일치한다.

| 파라미터 | 값 |
|---|---|
| base_x / base_y / base_z | 0.595 / 0.395 / 0.130 |
| 축거 (wheelbase) | 0.452 |
| **윤거 (track)** | **0.490** ← DiffDrive의 `wheel_separation`은 이 값 |
| 휠 반경 | 0.0875 (직경 175 mm) |
| 휠 폭 | 0.0852 |

> **`agilexrobotics/ugv_gazebo_sim`을 쓰지 않는다.** AgileX가 작성한 수치가 전부 틀렸다:
> `wheel_radius=0.16` (직경을 반경으로 오기입 → 2배), 섀시 질량 **132.39 kg** (실제 26 kg),
> `wheel_separation`에 윤거 대신 축거를 넣음. 그대로 쓰면 오도메트리가 조용히 2배로 스케일된다.

### 5.2 반드시 고쳐야 할 것 — 관성 텐서

westonrobot 파일조차 Husky에서 물려받은 관성이 **40~285배 과대**하다 (휠: 질량 1 kg에
`ixx=izz=0.7171`). 과대한 휠 관성은 스키드스티어 요 응답을 지배한다. 재계산값:

- **휠** m=1.5 kg, r=0.0875, l=0.0852 → 회전축 `0.5·m·r² = 0.00574`, 횡축 `(1/12)·m·(3r²+l²) = 0.00378` kg·m²
- **섀시** m=20 kg, 박스 0.595×0.395×0.130 → `Ixx=0.288`, `Iyy=0.618`, `Izz=0.850` kg·m²

### 5.3 구동

`gz::sim::systems::DiffDrive`를 쓴다. `gz_ros2_control`은 쓰지 않는다 — 실제 `scout_base`는
독립 노드이지 ros2_control `SystemInterface`가 아니므로, 시뮬에만 존재하는 컨트롤러 스택이
생겨 sim/real 괴리만 만든다.

```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
  <left_joint>front_left_wheel</left_joint>   <left_joint>rear_left_wheel</left_joint>
  <right_joint>front_right_wheel</right_joint><right_joint>rear_right_wheel</right_joint>
  <wheel_separation>0.49</wheel_separation>   <!-- 윤거. 축거 아님 -->
  <wheel_radius>0.0875</wheel_radius>
  <max_linear_velocity>2.7</max_linear_velocity>
  <topic>/cmd_vel</topic><odom_topic>/odom</odom_topic>
  <odom_publish_frequency>50</odom_publish_frequency>
</plugin>
```

마찰: **종방향 mu ≈ 0.9–1.1, 횡방향 mu2 ≈ 0.3–0.5** (이방성). AgileX의 `mu1=mu2=0.2` 균일값은
평면에서 스키드스티어가 회전이라도 하게 만드는 편법이라 아무 데도 전이되지 않는다.
`fdir1`은 **쓰지 않는다** — 월드 좌표계로 해석되고 엔진 간 동작 불일치가 문서화돼 있다 (gz-physics #258).

### 5.4 센서 구성 — **MID-70 단독** (D7)

포즈는 `base_link` 기준 `x y z roll pitch yaw` (m, rad). `base_link`는 섀시 중심의 지면 투영점.

| # | 센서 | 타입 | 포즈 | 설정 | 역할 |
|---|---|---|---|---|---|
| 1 | `livox_mid70` | `gpu_lidar` | `0.20 0 0.80 0 0.436 0` (**하향 25°**) | 113×113, ±0.6143 rad, 0.05–90 m, 10 Hz | 항법 + 인지 + SLAM 입력 **전부** |
| 2 | `imu_link` | `imu` | `0 0 0.10 0 0 0` | 200 Hz, gyro σ 2e-4, accel σ 1.7e-2 | FAST-LIO2 필수 입력, EKF |
| 3 | 휠 오도메트리 | DiffDrive | — | 50 Hz, `<publish_tf>false</publish_tf>` | EKF 트위스트 소스 |
| 4 | `navsat` | `navsat` | `-0.20 0 0.60 0 0 0` | 5 Hz | 선회 구간 전역 EKF |
| 5 | `cam_canopy_left` | `camera` | `0.05 0.26 1.20 0 -0.209 1.571` | 1920×1080, HFOV 1.047 rad, 10 Hz | 착과·병징 주 촬영 |
| 6 | `cam_canopy_right` | `camera` | `0.05 -0.26 1.20 0 -0.209 -1.571` | 동일 | 반대편 열 |
| 7 | `cam_forward_rgbd` | `rgbd_camera` | `0.30 0 0.45 0 0 0` | 640×480, HFOV 1.204 rad, 10 Hz | **행 진입/이탈 검출** (X 리셋 트리거) |
| 8–11 | GT 센서 4종 | `segmentation` ×2, `boundingbox_camera` ×2 | **#5와 동일 포즈·내부파라미터** | 1920×1080, **1–2 Hz** | 정답 라벨 |

**MID-70 배치 근거.** 360° 라이다를 두지 않기로 했으므로(D7) MID-70이 항법과 인지를 겸해야 한다.
전방 + 하향 25°는 항법 최적(지면·수간·장애물)이고, 수관 촬영은 측면 카메라(#5·#6)가 맡는다.
대가는 **LiDAR 기반 수관 체적/LAI 산출이 불가**해진다는 것이다 — 측면을 향하는 LiDAR가 없다.
이는 D7의 직접적 귀결이며, 필요해지면 MID-70을 틸트/팬 마운트에 올리거나 측면 배치로 전환한다.
하향각은 마스트 조인트의 xacro 파라미터로 빼서 **40° 프리셋**도 제공한다 (Wang et al.,
*Sensors* 2024, 24(24):7929 — 0.8 m/40°에서 지면 사각이 3 m → 0.21 m, 음형 장애물 검출 92.7%).

**양측 촬영은 타협 불가.** 단면 가시율이 40.85~79.83%에 불과하므로(arXiv:1808.04336) 좌우
카메라가 모두 있어야 한다. 좌우 동시 촬영이면 **M+1개 통로를 한 번 왕복하며 M개 열의 양면을 전부**
찍는다. 단면 장비 대비 임무 시간이 절반이다.

**행 내 속도 상한 0.6 m/s** — 항법 제약이 아니라 **촬영** 제약이다. 1.0 m 촬영거리에서
0.60 mm/px, 1/1000 s 글로벌셔터 기준 0.6 m/s면 스미어가 0.6 mm = 1 px. 선회 구간은 1.0 m/s.

---

## 6. MID-70 인터페이스 계약

**시뮬레이터 작업 전에 이 계약부터 고정한다.** 이게 있으면 하류 소비자 전부가 시뮬레이터 구현과
분리된다.

```
토픽      : /livox/lidar
frame_id  : livox_frame
타입      : sensor_msgs/msg/PointCloud2, Livox PointXYZRTLT 필드 배치
              x, y, z      float32  (m)
              intensity    float32  (0.0–255.0)
              tag          uint8
              line         uint8    (MID-70은 단일 레이저라 항상 0)
              timestamp    float64
주기      : 10 Hz
추가      : livox_ros_driver2/msg/CustomMsg — FAST-LIO2용 (§7.1)
```

> **필드 불일치 함정**: `CustomMsg`는 `uint8 reflectivity`, PointCloud2 PointXYZRTLT는
> `float32 intensity` 0–255. 한쪽에 맞춰 짠 코드를 다른 쪽으로 옮기면 조용히 절단된다.
> 브리지 노드에서 양쪽을 고정하고 두 경로 모두 단위테스트한다.

`livox_sim_bridge` 노드가 하는 일:
1. `ros_gz_bridge`가 넘긴 PointCloud2를 구독
2. **원형 FOV 마스킹** — `sqrt(az² + el²) > 35.2°`인 점 제거 (정사각 격자의 약 21%가 모서리에서 잘림)
3. PointXYZRTLT로 필드 재작성
4. FAST-LIO2용 `CustomMsg` 합성 (점별 `offset_time` 부여)

### 실제 MID-70과의 차이 — 은폐하지 않고 문서화한다

- **비반복 누적이 없다.** 매 프레임 동일 방향을 샘플링하므로 적분시간 대비 커버리지 실험이 불가능하다.
  → 수관 간극률/LAI를 시뮬레이션에서 튜닝하면 실제로 전이되지 않는다
- 상수 `max` 90 m는 FOV 가장자리 성능을 과대평가한다 (90 m는 FOV 중심 값)
- gz-sim #2743: ogre2 GpuLidar에 거리에 따라 커지는 V자 거리 오차가 있고, 얕은 입사각에서
  Classic보다 부정확하다 — 잎과 수피를 스치는 빔이 정확히 그 기하다

**비반복 스캔 충실도가 실제로 필요해지면** 커스텀 `gz::sim::System` 플러그인(2~3주)으로 승격한다.
승격 조건은 "이름 붙일 수 있는 하류 알고리즘이 로제트 패턴에 의존함이 입증될 때"로 한정한다.

> ⚠️ **`Jerry1962325/livox_laser_simulation_jazzy`는 가짜다.** 이 프로젝트가 원하는 것을 정확히
> 광고하고("Gazebo Sim + ROS 2 Jazzy로 이식", "PointCloud2 + CustomMsg") 검색 상위에 뜨지만,
> `Update()`는 `// placeholder`이고 `PostUpdate()`는 `wall_distance = 15.0`짜리 하드코딩된
> 가짜 방을 내보낸다. **월드를 전혀 레이캐스팅하지 않는다.** RViz에서 그럴듯해 보이는 점군이
> 나오지만 월드 기하와 아무 관계가 없다.

---

## 7. 자율주행 아키텍처

### 7.1 위치추정 — FAST-LIO2 (D6)

**MID-70 단독(D7)이라는 제약 아래서의 구성:**

```
livox_mid70 (gpu_lidar) ──▶ livox_sim_bridge ──▶ CustomMsg ──┐
imu_link (200 Hz) ───────────────────────────────────────────┴──▶ FAST-LIO2
                                                                     │ /Odometry
                            ┌────────────────────────────────────────┘
                            ▼
   robot_localization ekf_odom  (FAST-LIO2 + 휠 트위스트 + IMU) ──▶ odom→base_link
   robot_localization ekf_map   (+ GNSS, 행 내부에서는 게이트) ──▶ map→odom
   row_centerline_node          (Y·yaw만, X 공분산 대폭 확대)
   row_event_node               (행 진입/이탈에서 X 이산 리셋)
```

**FAST-LIO2를 시뮬에서 쓰려면 해야 할 일** — 정직하게 적는다:
1. IMU는 시뮬에 추가하면 된다 (실물 MID-70에는 내장 IMU가 없다)
2. `CustomMsg` 점별 타임스탬프를 `livox_sim_bridge`에서 합성해야 한다 — `gpu_lidar`는 안 준다
3. FAST_LIO ROS2 브랜치는 "ROS >= Foxy (Humble 권장)"이라 **Jazzy 빌드 검증이 필요하다**.
   실패 시 대안은 `direct_lidar_odometry` 또는 PointCloud2를 직접 먹는 LIO 계열

### 7.2 행 방향(X) 미관측 문제 — 설계로 회피한다

과수원 행은 **X축으로 의도적으로 자기유사**하다. 행 중간의 어떤 관측도 X 위치를 복원하지 못한다.
이건 AMCL·slam_toolbox 루프클로저·FAST-LIO2 포즈그래프를 **똑같이** 퇴화시킨다. FAST-LIO 저자들도
"LiDAR 기반 해법은 쉽게 퇴화하며, FoV가 작을수록 이 문제가 뚜렷하다"고 쓴다 (arXiv:2010.08196 §I).

**그러므로 X를 연속 관측하려는 시도를 그만둔다.**

| 축 | 행 내부 | 선회 구간 |
|---|---|---|
| **Y(횡), yaw** | 수간 점군 → 행 중심선 피팅. 연속 | GNSS + EKF |
| **X(종)** | 휠오도메트리 + IMU 추측항법. `cam_forward_rgbd`가 검출한 **행 진입/이탈 이벤트에서 이산 리셋** | GNSS + EKF |
| map→odom | 전역 EKF, GNSS 공분산 ×100 또는 게이트 오프 | 전역 EKF, GNSS 신뢰 |

45 m 행에서 휠오도 오차 1~2% → X 드리프트 0.45~0.9 m. 행 끝에서 보정된다.
**행 안에서 X 오차는 충돌을 일으키지 않는다. 충돌은 Y가 일으키고, Y는 잘 관측된다.**

### 7.3 MID-70 단독(D7)에 대한 완화책

360° 라이다가 하던 세 가지를 이렇게 메운다:

| 잃는 것 | 완화책 |
|---|---|
| 측후방 코스트맵 커버리지 | **사전 매핑한 정적 레이어.** FAST-LIO2로 매핑 주행 1회 → 점군맵 → 2D 투영 → occupancy grid. 수목열은 정적이므로 이 지도가 전방위 정보를 항구적으로 제공한다 |
| 레이트레이스 클리어링 (FOV 밖 셀을 지울 수 없음) | **STVL `voxel_decay` 8–12초.** 지우지 못하는 유령 마크를 시간으로 만료시킨다. 감쇠의 목적이 "유지"가 아니라 "만료"라는 점이 중요 |
| 제자리 회전 중 사각 | `Spin` 복구 동작 유지 (70° 콘을 회전시켜 코스트맵 재구축). `BackUp`은 `backup_dist ≤ 0.15`로 제한 |

**이 완화책의 전제는 "장애물이 정적"이라는 것이다.** 동적 장애물(사람·동물)이 측후방에서
접근하는 시나리오는 이 구성으로 다룰 수 없다. D7의 알려진 한계로 기록한다.

### 7.4 Nav2 설정값

```yaml
# 풋프린트 — 원형이 아니라 폴리곤
footprint: "[[0.34,0.30],[0.34,-0.30],[-0.34,-0.30],[-0.34,0.30]]"
#   외접원 0.417 m를 쓰면 편측 0.725 m 중 0.142 m를 버리게 된다

local_costmap:
  resolution: 0.05 ; rolling_window: true ; width/height: 6.0
  plugins: ["static_layer", "stvl_layer", "inflation_layer"]   # static_layer = §7.3
  stvl_layer:
    plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
    voxel_decay: 12.0        # 초. 8–15. 목적은 유령 마크 만료
    decay_model: 0           # linear
    voxel_size: 0.05 ; mark_threshold: 0
    observation_sources: mid70
    mid70: {min_obstacle_height: 0.20,   # 예초된 초생 잔디 배제
            max_obstacle_height: 1.00,   # 마스트가 지나갈 수관 배제
            clearing: true, marking: true, obstacle_range: 10.0}
  inflation_layer:
    inflation_radius: 0.40   # 하드캡 0.55. 통로 최악 2.0 m → 편측 0.725 m
    cost_scaling_factor: 3.0

global_costmap:
  resolution: 0.10 ; rolling_window: true ; width/height: 40.0

controller_server:
  FollowPath:
    plugin: "nav2_rotation_shim_controller::RotationShimController"
    primary_controller: "nav2_mppi_controller::MPPIController"
    angular_dist_threshold: 0.785 ; rotate_to_heading_angular_vel: 0.6
    motion_model: "DiffDrive"
    batch_size: 2000 ; time_steps: 56 ; model_dt: 0.05
    vx_max: 0.6 ; vx_min: -0.20 ; wz_max: 1.0
    ObstaclesCritic:
      repulsion_weight: 1.0    # 기본 1.5에서 낮춤. MPPI README가 좁은 통로 지터를 경고
      critical_weight: 20.0
      inflation_radius: 0.40   # 코스트맵 레이어와 정확히 일치시켜야 함
    PathAlignCritic: {weight: 16.0}

planner_server:
  planner_plugins: ["InRow", "Headland"]
  InRow:    {plugin: "orchard_nav::RowCenterlinePlanner"}   # 커스텀 ~300 LOC
  Headland: {plugin: "nav2_navfn_planner::NavfnPlanner", tolerance: 0.25}
  # SmacPlannerHybrid 금지 — Nav2 공식 선택표에 Ackermann/Legged 전용으로 명시
```

### 7.5 Stage-0 스캐폴드 — 선택이 아니라 필수

**gz 모델 포즈를 읽어 완벽한 `map→odom`을 발행하는 50줄짜리 노드를 먼저 만든다.**
완벽한 위치추정 위에서 MPPI·인플레이션·경로그래프·행 기하를 튜닝해야, 주행이 이상할 때
어느 층이 깨졌는지 알 수 있다. 실제 스택은 나중에 갈아끼운다.

---

## 8. 데이터 수집과 정답 라벨

### 8.1 GT 센서 구성

RGB 카메라 하나와 **동일한 링크·포즈·내부파라미터·frame_id**에 4개를 겹쳐 단다:

| 센서 | 타입 | `<camera>` 설정 | 산출물 |
|---|---|---|---|
| `semseg` | `segmentation` | `<segmentation_type>semantic` | 클래스 마스크 |
| `panoptic` | `segmentation` | `<segmentation_type>panoptic` | 인스턴스 마스크 |
| `boxes_vis` | `boundingbox_camera` | `<box_type>visible_2d` | **가림 반영** 2D 박스 |
| `boxes_full` | `boundingbox_camera` | `<box_type>full_2d` | **amodal** 2D 박스 |

> `panoptic`과 `instance`는 **같은 모드의 별칭**이다. 세 모드가 아니라 두 모드다.

**`visible_2d ÷ full_2d`가 곧 가림률 측정이다.** 이 비율 하나로 생성한 과수원이 문헌의
40.85~79.83% 대역에 드는지 검증한다. 실제 데이터셋이 대부분 갖지 못한 라벨이고,
`AmodalAppleSize_RGB-D`는 이걸 사람이 수작업으로 만들었다. **두 번째 박스 센서를 빼지 않는다.**

### 8.2 panoptic labels_map 디코딩 — 브리프와 반대다 (실측 확정)

```
channel 0 = 인스턴스 하위 바이트
channel 1 = 인스턴스 상위 바이트     →  instance = ch1 * 256 + ch0
channel 2 = semantic label
```

리서치 브리프는 `ch0 = label, ch1·ch2 = instance` 라고 했으나 **실측은 반대다.**
브리프 공식을 그대로 쓰면 라벨과 인스턴스를 서로 바꿔 읽는다.
디코더는 [`scripts/01_analyze_labels.py`](../../../scripts/01_analyze_labels.py) 의 것을 쓴다.

또한 `<camera><save enabled="true">` 는 이 환경에서 **파일을 쓰지 않는다** (원인 미확인).
정답 데이터 추출은 `ros_gz_bridge` + 구독 노드 경로로 간다.

### 8.3 라벨 ID 체계 (0–255)

```
 0 background · 10 ground · 11 weed · 12 trellis_post · 13 wire
20 trunk · 21 branch
30 leaf_healthy · 31 leaf_marssonina · 32 leaf_alternaria · 33 leaf_rust
34 leaf_mite_chlorosis · 35 leaf_aphid_curl
40 fruit_healthy · 41 fruit_anthracnose · 42 fruit_whiterot
50–59 lesion_* (병반 데칼)
```

### 8.4 반드시 지킬 6가지 — 전부 조용히 실패하는 항목

1. **텍스처로 그린 병반은 마스크도 박스도 0개를 낳는다.** 세그멘테이션은 `<visual>` 하나를
   라벨 색으로 통째로 칠한다. UV/텍스처 공간 라벨 채널은 파이프라인 어디에도 없다.
   → **1주차에 나무 한 그루로 "병반=별도 visual" 프로토타입을 검증한다**
2. **플랫 월드를 내보낸다** — gz-sim #1579
3. **런타임 스폰 금지** — 라벨을 잃는다
4. **`labels_map`은 `RGB_INT8`.** `image_transport` 압축·rosbag2 손실압축·JPEG 저장은
   정수 라벨을 그럴듯하지만 틀린 클래스로 오염시킨다. **raw PNG만**
5. **인스턴스 ID는 라벨당 16-bit(65,535).** 과실과 검사목 병반에만 인스턴스 라벨링을 한정한다
6. **타임스탬프 정렬.** RGB 10 Hz와 GT 2 Hz는 타임스탬프를 공유하지 않는다. `<triggered>`를
   우선 시도하고, 미지원이면 정확한 sim time 일치로 매칭하고 **데이터셋 작성기에서 타임스탬프
   동일성을 assert 한다**. 조용히 어긋난 RGB/마스크 쌍은 합성 데이터셋을 무가치하게 만드는 전형이다

### 8.5 인스턴스 ID의 시간적 불안정성

panoptic 인스턴스 ID는 **매 프레임 재할당된다** (`instancesCount.clear()`가 프레임마다 실행).
**프레임 N의 인스턴스 7과 프레임 N+1의 인스턴스 7은 다른 과실이다.** 추적·재식별·다중뷰 연관의
정답으로 쓸 수 없다. 동일성은 **모델 이름**에서 가져온다 (`pose_publisher` / `/world/<name>/pose/info`).

`box_type=3d`는 쓰지 않는다 — gz-sensors #428 미해결: 3D 검출이 약 180° 회전되고 z가 음수로 나온다.
과실별 3D 정답은 모델 포즈에서 유도한다.

### 8.6 프레임당 산출 형식

```json
{"frame_id": "...", "timestamp": "...", "camera_pose": "...",
 "objects": [{"fruit_model_name": "apple_r03_t12_f47",
              "world_pose": "...", "camera_frame_pose": "...", "diameter_m": 0.0721,
              "modal_bbox": [...], "amodal_bbox": [...],
              "visible_pixel_count": 812, "occlusion_ratio": 0.41}]}
```

**시뮬레이션만이 가능하게 하는 것 — 이걸 일찍 만든다.** 실제 과수원에서는 나무를 따내지 않고는
참 착과 수를 알 수 없다. 시뮬에서는 정확히 안다. **다중뷰 연관 / 중복 카운트 제거 평가기**를
완벽한 정답 위에서 만든다. 목표 기준선: 사과 1,790개에 대해 카운팅 정확도 96.9%,
과실 크기 평균오차 1.1 cm (arXiv:2409.19786).

### 8.7 병해 — 할 수 있는 것과 없는 것

**렌더링 품질이 문제라는 통념은 반박됐다.** Klein et al. (2024, doi:10.3389/fpls.2024.1360113)은
Blender 렌더링만으로 학습해 실제 온실 사진에서 89.6%를 얻었고, "포토리얼리즘은 주요 품질 동인이
아니다"라고 명시한다. 실제 PlantVillage 사진으로 학습한 쪽은 실제 테스트셋에서 랜덤 수준이었다.
SDFormat의 `<pbr><metal><albedo_map>`은 **사용자 이미지 파일**을 받으므로, 실제 병징 매크로
사진을 알베도로 쓰면 미세 통계는 진짜 사진 통계가 된다.

**진짜 한계는 광학 물리다.** 1920×1080, HFOV 60°:

| 촬영거리 | 해상도 | 갈색무늬병 병반 5–10 mm | 분생포자층 0.1–0.2 mm |
|---|---|---|---|
| 1.0 m | 0.60 mm/px | 8–17 px ✅ | **0.17–0.33 px ❌** |
| 1.5 m | 0.90 mm/px | 6–11 px ✅ | 0.11–0.22 px ❌ |

농촌진흥청이 갈색무늬병의 결정적 특징으로 지목하는 흑색돌기(분생포자층)가 **서브픽셀이며 4K로도
해결되지 않는다.** 약 0.15 m까지 접근해야 하는 **능동 인지(암 로봇) 문제**이지 주행 중 촬영 문제가 아니다.

| 등급 | 대상 | 판정 |
|---|---|---|
| **1 — 주행 촬영으로 가능** | 붉은별무늬병(포화 주황–적색, 매크로 스케일), 갈색무늬병 **중·후기**(5–10 mm 병반 + 황화 + 조기낙엽), 점무늬낙엽병, 응애 피해(수관 스케일 황갈색 변색), 사과혹진딧물(**텍스처가 아니라 기하 변형으로 모델링**) | 구현 |
| **2 — 정지 촬영 또는 전용 카메라 필요** | 탄저병·겹무늬썩음병 (열 방향으로 노출된 과실에 한해) | 확장 |
| **3 — 범위 외** | 부란병(주간 궤양 — 카메라 기하 자체가 다름), 조기 진단 전반, 분생포자층에 의존하는 모든 것 | 제외. GSD 표를 근거로 명시 |

**공식 표현:**

> 시뮬레이션은 자율 데이터 수집 파이프라인의 개발·검증, 카메라 구성·촬영거리·커버리지 최적화,
> 그리고 착과 카운팅과 다중뷰 과실 연관의 학습·검증에 사용한다. 시스템 산출물은
> **증상 탐지와 심각도 매핑**이며 **종 수준 진단이 아니다** — 국내 식물병리 문헌은
> 사과 갈색무늬병 유사 증상이 육안으로 *Marssonina coronaria*와 구분되지 않아 현미경·배양·PCR이
> 필요함을 확립하고 있고, 결정적 육안 지표(분생포자층 0.1–0.2 mm)는 지상 로봇이 주행 가능한
> 어떤 촬영거리에서도 서브픽셀이다.

시뮬레이션 테스트셋의 병해 정확도를 실데이터 수치 없이 단독 보고하지 않는다.

---

## 9. 성능 계획

**우선순위 순 — 아래가 안 되어 있는데 다른 걸 손대면 시간 낭비다.**

1. **충돌 기하.** 주간 = `<cylinder>` 프리미티브. 수관/엽군 = **`<collision>` 요소 자체를 두지 않음**.
   `<collision>` 안에 `<mesh>` 절대 금지 (Baylands 사례: 이것만으로 RTF 5% → 90%)
2. **모든 나무에 `<static>true</static>`** — 동역학 솔버에서 제외
3. **메시/텍스처 예산.** 그루당 2–5k tris, 공유 1024² 텍스처 1–2장, **개별 파일이 아니라 3–5종
   메시를 재사용**. 6 GB VRAM은 2048²/4096² 세트에서 스래싱한다
4. **그림자 끄기.** `<scene><shadows>0</shadows>`
5. **센서 스로틀링.** gpu_lidar 10 Hz, 카메라 10 Hz, GT 카메라 1–2 Hz
6. **헤드리스 서버 + 분리형 GUI.** `gz sim -v4 -s -r --headless-rendering`
7. **물리.** `<max_step_size>0.002</max_step_size>` (500 Hz). 2 ms는 ≤2 m/s 스키드스티어에 충분하고
   기본 1 ms보다 2배 싸다. 4 ms를 넘기면 휠 접촉이 불안정해진다
8. **Levels** (`--levels`) — 행별 또는 20 m 타일별, 로봇을 `<performer>`로

> **진짜 병목은 센서가 아니라 브리지다.** ros_gz #368: gpu_lidar 브리지를 켜는 것만으로
> RTF 90% → **40–60%**, 무제한 센서 레이트에서 **31–33%**. `PointCloudPacked → PointCloud2`는
> 점마다 CPU 재패킹이다. **브리지 유무 양쪽에서 RTF를 측정해 둘 다 보고한다.**

### RTF 목표 (180그루 월드, Levels 켬, gpu_lidar 1 + RGB 2 @10 Hz)

| 구성 | RTF | GUI fps | 판정 |
|---|---|---|---|
| **GPU 미복구** — Intel iGPU | **0.1–0.3** | 5–15 | 디버그 전용. **이 상태의 RTF 측정은 무의미** |
| GPU 미복구, 헤드리스 | 0.3–0.5 | — | 한계 |
| **GPU 복구** — GTX 1660 SUPER, GUI 켬 | **0.8–1.0** | 30–60 | **설계 목표** |
| GPU 복구, 헤드리스 | **1.0–2.0+** | — | 데이터셋 생성용 |

**이 숫자들은 하드웨어 비율과 공개 사례로부터의 공학적 외삽이지 이 장비에서의 실측이 아니다.**
GPU가 살아난 날 대표 월드로 벤치마크한다.

**GPU 미복구 상태의 작업 상한:** 평면 지형 + 저폴리 나무 30그루, GT 카메라 없음.
전체 월드를 시도하지 않는다.

---

## 10. 환경 선행 작업

### 10.1 GPU (사용자 직접 조치)

- **원인**: DKMS가 `ubuntu Secure Boot Module Signature key`로 nvidia 580.173.02를 서명했으나
  그 키가 MOK에 등록되지 않음. `lockdown = [integrity]`가 모듈 로드를 거부
- **조치**: `mokutil --import` 완료(대기열 등재 확인) → 재부팅 → 파란 MOK Manager 화면에서 등록
- **⚠️ 두 번째 함정**: `prime-select query = on-demand`. 드라이버가 살아나도 **Gazebo는 계속
  Intel iGPU를 쓴다.** `sudo prime-select nvidia` + 재부팅이 반드시 필요하다
- **대안 경로** (콘솔 불필요): `sudo apt install linux-modules-nvidia-580-generic` —
  Canonical 사전서명 모듈이 이미 등록된 Canonical Master CA로 검증된다. 현재 커널
  6.8.0-136-generic용으로 `6.8.0-136.136+1` 존재 확인, 설치 시뮬레이션 4개 추가/0개 제거

### 10.2 ROS 2 설치

`packages.ros.org` HTTPS 인증서 불일치는 **문제가 아니다.** 공식 `ros2-apt-source` .deb가
스스로 `URIs: http://packages.ros.org/ros2/ubuntu`를 설정하며, 무결성은 GPG 서명된 InRelease와
인라인 `Signed-By:` 키로 보장된다. **`https://packages.ros.org` 소스 줄을 손으로 쓰지 않는다.**

**`packages.osrfoundation.org`를 추가하지 않는다.** Jazzy부터 Gazebo는 ROS 벤더 패키지
(`gz-sim-vendor` 등)로 배포된다. OSRF 저장소를 더하면 Gazebo가 두 벌 설치되어 진단하기 괴로운
플러그인 경로/버전 불일치를 만든다.

Wayland 세션이므로 `export QT_QPA_PLATFORM=xcb`가 **필수**다. Gazebo 공식 문서가 Ogre/Qt와
Wayland의 상호작용 문제를 명시한다. GDM에서 "Ubuntu on Xorg"를 선택하는 편이 더 낫다.

---

## 11. 구현 단계 (D9: 월드 우선)

| 단계 | 산출물 | 검증 게이트 |
|---|---|---|
| **0. 환경** | ROS 2 Jazzy + Gazebo Harmonic, 워크스페이스 | `gz sim shapes.sdf`가 뜨고 `glxinfo`가 NVIDIA를 보고 |
| **1. 나무 1그루** | `gen_tree.py` v0, 사과 2개, 라벨 프로토타입 | **과실 2개가 별도 인스턴스로 분리되는지.** 여기서 실패하면 §8.3-1 함정 |
| **2. 월드 생성기** | `gen_world.py`, 4행×20주, 지형, 지주 | RTF 측정, 단면 가시율 55–65% 검증 |
| **3. 로봇 모델** | SCOUT MINI URDF + MID-70 + 카메라 + IMU | 1.0 m/s 직진·제자리회전에서 `/odom`이 참값 대비 수 % 이내 |
| **4. 인터페이스 계약** | `livox_sim_bridge` | 원형 마스킹 후 100 kpts/s, CustomMsg 양방향 단위테스트 |
| **5. Stage-0 스캐폴드** | 완벽 `map→odom` 노드 | Nav2가 완벽 위치추정 위에서 행을 주행 |
| **6. Nav2** | 코스트맵·플래너·컨트롤러 튜닝 | 2.3 m 통로 왕복 + 선회 성공 |
| **7. FAST-LIO2** | 실제 위치추정으로 교체 | Stage-0 대비 궤적 오차 |
| **8. 데이터 파이프라인** | GT 센서 + 데이터셋 작성기 | 타임스탬프 동일성 assert 통과 |
| **9. 병해 에셋** | 잎 6종 + 과실 3종 PBR, 병반 데칼 | 병반별 마스크가 실제로 나오는지 |

**전체 규모 추정: 약 50~56 person-day (솔로 10~11주).** 최대 단일 항목은 Nav2가 아니라
**에셋 파이프라인 8~12일**이다. 일정 압박 시 올바른 축소는 Nav2가 아니라 **월드 규모**
(6행×30주 → 4행×20주)와 **병반 기하 커버리지**다.

---

## 12. 리스크 (먼저 물 순서)

1. **GPU 미복구 — 성능이 아니라 정확성 문제.** Gazebo 공식 문서: 하이브리드 장비에서 Intel로
   폴백하면 **"그림자나 레이저 스캔이 부정확해진다"**. 두 용도 모두 렌더링 기반 센서다.
   렌더링 백엔드 아티팩트를 Nav2 버그로 오인해 몇 주를 태울 수 있다. **다른 무엇보다 먼저 고친다**
2. **세그멘테이션이 visual을 통째로 칠한다.** 병반을 텍스처로 그리면 마스크가 0개다.
   월드를 다 만든 뒤 발견하면 정답 파이프라인 전체가 무너진다. **1주차에 나무 한 그루로 검증**
3. **드로우콜 폭발.** 과실 visual 10,800개 + 알파테스트 잎 카드 + 렌더링 센서 5개.
   RTF가 1.0 아래로 떨어지면 Nav2 제어 루프가 센서 레이트와 조용히 분리되어,
   실기에서 무효한 컨트롤러 튜닝이 나온다. **한 행으로 먼저 측정**
4. **에셋 파이프라인이 임계경로.** 8–12일, 하류 전부가 여기 막히고, 지름길이 없다
5. **행 방향 X 미관측.** 위치추정기를 바꿔도 사라지지 않는다. 행 끝 검출이 불안정하면 완화책이 붕괴
6. **gz-sim #1579 + 런타임 스폰 라벨 손실.** 과수원은 자연스럽게 행→나무→과실 중첩을 원하는데,
   그게 정확히 인스턴스 ID를 조용히 망가뜨리는 패턴이다
7. **통로가 생각보다 좁다.** 2.0–2.3 m다. `inflation_radius`가 0.55를 넘으면 양쪽 수관의 인플레이션
   장이 만나 행 전체가 고비용이 된다. 증상은 "플래너가 행에 진입하지 않음"으로 나타나 플래너 버그처럼
   보이지만 실제로는 코스트맵 파라미터다. **RViz에서 코스트맵부터 눈으로 확인**
8. **시뮬 GNSS는 거짓말한다.** gz NavSat은 위치 + 가우시안 노이즈만 모델링한다. 다중경로도,
   수관 감쇠도, fix→float 상태기계도 없다. 실제 착엽기 이중대역 RTK는 0.17–0.18 m로 열화된다
9. **MID-70 단독의 동적 장애물 사각 (D7).** §7.3의 완화책은 정적 장애물 전제다
10. **부분적 `use_sim_time`.** Nav2 시뮬 실패의 최다 원인이고, 시계 오류가 아니라 불규칙한
    컨트롤러 거동으로 나타난다. 모든 lifecycle 노드·`robot_state_publisher`·양쪽 EKF·커스텀 노드 전부
11. **과대 주장.** 시뮬 테스트셋에서 병해 정확도 95%는 자기 텍스처를 얼마나 외웠는지를 잴 뿐이다

---

## 13. 참고 자료

전체 리서치 원문: [`docs/research/`](../../research/)
- `01-brief-stack-sensor-world.md` — 스택·Livox·Scout·월드·성능·농업솔루션 (검증 에이전트 7건 포함)
- `02-brief-nav2-apple-disease.md` — Nav2·사과 과수원 형상·병해 이미징 (검증 에이전트 7건 포함)
- `*-verdicts-*.json` — 적대적 검증 판정 원문
