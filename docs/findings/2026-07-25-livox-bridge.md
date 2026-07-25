# 실측 — livox_sim_bridge 통합 검증 (단계 4)

**측정일**: 2026-07-25 · **환경**: gz-sim 8.11.0 + ROS 2 Jazzy, orchard_nav.sdf (GUI 실행 중)
**재현**: `python3 scripts/05_verify_livox_bridge.py` (월드 + gz 브리지 + livox_sim_bridge 기동 상태)

## 입력 실측 — gz gpu_lidar 가 실제로 내보내는 형식

```
frame_id    scout_mini_mid70/livox_frame/livox_mid70
width×height 113 × 113 = 12,769 점   point_step 32   is_dense false
필드        x@0 y@4 z@8 intensity@16 (FLOAT32), ring@24 (UINT16)
```

## 출력 계약 준수 — 전 항목 통과

| 항목 | 결과 |
|---|---|
| frame_id | `livox_frame` ✔ |
| PointXYZRTLT 배치 | x@0 y@4 z@8 intensity@12 tag@16 line@17 timestamp@24, step 32 ✔ |
| is_dense | true (무효점 제거 후) ✔ |
| 축이탈각 최대 | 35.20° = FOV 반각 정확히 준수 ✔ |
| line | 전부 0 (MID-70 단일 레이저) ✔ |
| timestamp | 단조증가 ✔ |
| CustomMsg | point_num 일치, offset_time 단조, reflectivity uint8 ✔ |
| 입력 대비 추종률 | **100.0%** (프레임 드롭 없음) ✔ |

## 검증이 잡아낸 것 — 내 기댓값 오류 3건

통합 검증은 코드 버그가 아니라 **내 검증 기준의 오류**를 드러냈다. 셋 다 기록해 둔다.

### 1. 100 kpts/s 는 발사율이지 수신율이 아니다

처음엔 수신 점수를 사양과 직접 비교해 "73 kpts/s, 사양 미달"로 실패 처리했다. 틀렸다.

```
격자 발사     12,769 점
원형 FOV 통과  9,852 점  = 98.5 kpts/s  ← 사양 100 kpts/s 와 일치
실제 수신      7,252 점  = 72.5 kpts/s  ← 하늘·90 m 초과는 무반사
```

실제 MID-70 도 하늘로 쏜 광선은 되돌아오지 않는다. `process_frame` 이 `emitted`(발사)와
`kept`(수신)를 분리해 반환하도록 고치고, 사양 비교는 `emitted` 로 한다.

### 2. 벽시계 주기를 시뮬시간 목표와 비교했다

"4.5 Hz, 목표 10 Hz 미달"로 실패 처리했으나, 당시 RTF 가 0.42~0.56 이었다.

```
시뮬 10 Hz × RTF 0.46 ≈ 4.6 Hz (벽시계)   ← 관측값과 정확히 일치
```

브리지는 정상이었다. 검증은 절대 Hz 가 아니라 **입력 대비 추종률**을 봐야 한다 (실측 100.0%).

### 3. intensity 가 전부 0 — gz 의 한계이지 브리지 오류가 아니다

gz gpu_lidar 는 반사강도를 모델링하지 않는다. 유효점 9,393 개의 intensity 고유값이 **1개(0.0)**.

대응: 기본값 `intensity_mode=passthrough` 로 0 을 유지한다 — **없는 정보를 지어내지 않는다.**
시각화나 거리 가중 휴리스틱에 0 이 아닌 값이 필요하면 `intensity_mode:=range` 로 거리 기반
합성을 켤 수 있으나, **이것은 반사율이 아니므로** 재질 단서로 쓰는 알고리즘에는 금물이다.

## 부수 개선

- **CustomMsg 지연 발행**: 점별 파이썬 루프라 프레임당 약 5% 를 먹는다.
  `get_subscription_count() > 0` 일 때만 만든다. FAST-LIO2 를 안 띄우면 비용 0.
- **`livox_ros_driver2` 메시지 스텁**: 실제 드라이버는 MID-70 을 지원하지 않고 Jazzy 바이너리도
  없다. 시뮬 전용(D8)이므로 메시지 정의만 원본과 동일한 이름·필드로 제공해 FAST-LIO2 가
  수정 없이 빌드되게 한다.

## 알려진 차이 (설계서 §6 "실제 MID-70 과의 차이" 갱신)

- 비반복 로제트 없음 — 매 프레임 동일 방향 샘플링
- intensity 없음 (위)
- 점별 시각은 스캔 순서로 균등 합성 — 실제 로제트의 시간 분포와 다르다
