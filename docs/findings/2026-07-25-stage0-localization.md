# 실측 — Stage-0 로컬라이제이션 스캐폴드 (단계 5)

**측정일**: 2026-07-25 · **환경**: gz-sim 8.11.0 헤드리스 + ROS 2 Jazzy, orchard_nav.sdf
**재현**: `ros2 launch orchard_sim stage0.launch.py` → `python3 scripts/06_verify_stage0.py`

설계서 §7.5 가 "선택이 아니라 필수"라고 한 스캐폴드. gz 참값으로 완벽한 `map→odom` 을
발행해, Nav2 를 **완벽한 위치추정 위에서** 먼저 튜닝할 수 있게 한다. 단계 7 에서 FAST-LIO2 로 교체.

## 결과 — 전 항목 통과

| 항목 | 결과 |
|---|---|
| TF 트리 | `map → odom → base_link → {livox_frame, imu_link, navsat_link, cam×3}` ✔ |
| 정지 시 참값 일치 | 평면 **0.0 mm**, z 0.0 mm, yaw **0.000°** |
| 주행 중 추종 (0.5 m/s + 0.15 rad/s 곡선, 690 표본) | 평면 평균 **2.0 mm** / 최대 11.7 mm, yaw 평균 0.284° / 최대 1.678° |
| map→odom 누적 | 평면 38.2 m, yaw +98° — 휠 드리프트를 정상 흡수 |
| 센서 외부파라미터 vs SDF | 6개 전부 **0.000 mm** |

## 막힌 지점 2개와 원인

### 1. gz 참값 브리지의 frame_id 가 빈 문자열

처음엔 월드의 `dynamic_pose/info`(13 엔트리, 가벼움)를 브리지했다. 52 Hz 로 잘 도는데
`frame_id`·`child_frame_id` 가 **빈 문자열**이었다. `ros_gz` 의 `Pose_V → TFMessage` 변환기가
gz 의 `Pose.name` 을 프레임 이름으로 옮기지 않기 때문이다.

**해결**: 로봇 모델에 `gz-sim-pose-publisher-system` 을 붙였다. 이쪽은 header 에
`frame_id`/`child_frame_id` 를 제대로 채운다 (`orchard_10x41 → scout_mini_mid70` 확인).

참고로 월드의 `pose/info` 는 **5,478 엔트리**라 브리지 대상이 아니다.

### 2. 회전 중 yaw 오차 3.37° — 타임스탬프를 섞었다

초기 구현은 참값(시각 t1)과 오도메트리 TF(최신, 시각 t2)를 그대로 곱했다. 정지 상태에서는
문제가 없지만 회전 중에는 그 시차만큼 오차가 실린다. 최대 3.37° 를 관측했다.

**해결**: 참값의 시각으로 오도메트리를 조회하고, 같은 시각으로 `map→odom` 을 발행한다.

```
yaw 최대 오차   3.370° → 1.678°   (절반 이하)
평면 평균 오차  8.0 mm → 2.0 mm   (1/4)
```

## 설계 선택 — 센서 외부파라미터를 SDF 에서 직접 읽는다

`sdf_static_tf` 노드가 `sim/models/scout_mini_mid70/model.sdf` 를 파싱해 static TF 를 만든다.
YAML 에 좌표를 다시 적어두면 model.sdf 와 조용히 어긋나고, 그러면 캘리브레이션·융합 결과가
전부 무의미해지는데 증상이 안 보인다. 진실의 근원을 하나로 묶어 드리프트를 원천 차단한다.
검증에서 6개 프레임 전부 0.000 mm 일치를 확인했다.
