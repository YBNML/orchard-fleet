# Scout Mini 메시 출처 및 라이선스

`sim/models/scout_mini_mid70/meshes/` 의 다음 파일은 제3자 저작물이다.

| 파일 | 출처 | 라이선스 |
|---|---|---|
| `mini_base_link.dae` | [westonrobot/scout_ros2](https://github.com/westonrobot/scout_ros2) `scout_description/meshes/scout_mini/` | BSD 3-Clause |
| `mini_wheel.dae` | 동일 | BSD 3-Clause |

**저작권 표기 (BSD 3-Clause 요구사항)**

```
Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.
Copyright (c) 2020, Weston Robot Pte. Ltd., All rights reserved.
Authors: Paul Bovbel <pbovbel@clearpathrobotics.com>
         Ruixiang Du <ruixiang.du@westonrobot.com>
```

원본 URDF 는 Clearpath 의 husky_description 에서 파생됐다.

## 이 프로젝트에서 쓰는 방식

- **시각 지오메트리만** 원본 메시를 쓴다. 충돌은 프리미티브로 대체했다
  (설계서 §9-1: 시각 메시를 충돌로 쓰면 RTF 가 무너진다)
- **관성은 원본을 쓰지 않는다.** 원본 휠 관성은 Husky 유산으로 실제의 40~285배이며
  스키드스티어 요 응답을 지배해 시뮬 튜닝값을 실기에서 무용하게 만든다.
  재계산값(휠 스핀 0.00574 / 횡 0.00378, 섀시 0.288/0.618/0.850)을 쓴다
- **`.dae` 를 변환 없이 그대로 쓴다.** Ubuntu 24.04 의 Blender 4.0.2 에는 COLLADA
  지원이 없어(import·export 둘 다) 변환이 불가능한데, gz-common 이 libassimp 로
  직접 읽는다. 로드 4.2초, 에러 없음 (2026-07-25 확인)

## 좌표 규약

원본 DAE 는 mm 단위이고 노드 행렬에 스케일 0.001 과 축 회전이 들어 있다.

| | 값 |
|---|---|
| base_link 메시 실치수 | 0.561 × 0.390 × 0.182 m |
| 휠 메시 실치수 | 직경 0.170, 폭 0.099 m |
| base_link 높이 | **z = 0.178** (URDF 규약). 이 값이라야 휠 오프셋 −0.0905 가 휠 중심 z=0.0875(=반경, 접지)와 맞는다 |
| 휠 메시 오프셋 | `(-0.221, -0.225, 0.0925)` — 메시가 로봇 프레임 기준으로 저작돼 있어 링크 원점으로 되돌린다 |

메시 삼각형: base 293,665 / 휠 163,096 × 4 = 약 95만.
엔티티 수가 아니라 삼각형이므로 RTF 영향은 작다 (2026-07-25 RTF 병목 실측 참조).

## 재질 — 원본에 없어 SDF 에서 지정한다

원본 DAE 를 파싱한 결과 **재질 데이터가 전혀 없다**:

```
mini_base_link.dae :  material 0 / effect 0 / image(texture) 0 / instance_material 0
mini_wheel.dae     :  동일
```

순수 형상만 담긴 파일이라 렌더러가 기본 흰색으로 폴백한다(2026-07-25 GUI 확인).
따라서 SDF `<material>` 로 지정한다.

| 부위 | ambient/diffuse | metalness | roughness | 의도 |
|---|---|---|---|---|
| 섀시 | 0.13 / 0.19 차콜 | 0.55 | 0.42 | 도장 금속 |
| 휠 | 0.055 / 0.085 | 0.0 | 0.92 | 고무 타이어 (무광) |
| 마스트 | 0.30 / 0.42 | 0.85 | 0.30 | 알루미늄 |

**부위별 착색은 불가능하다.** 두 메시 모두 **단일 지오메트리**이기 때문이다
(base 293,665 tri 하나, 휠 163,096 tri 하나). 재질 그룹이 없으므로 visual 하나당 색 하나다.
실제 Scout Mini 의 오렌지 트림·헤드라이트를 살리려면 메시를 분할해야 하고, 그러려면
DAE→OBJ 변환기를 직접 만들어(Blender 에 COLLADA 지원이 없음) Blender 에서 연결 요소별로
쪼갠 뒤 재질을 입혀 GLB 로 재출력해야 한다. 겸사겸사 95만 삼각형 감축도 가능하지만
현재는 필요성이 낮아 보류한다.
