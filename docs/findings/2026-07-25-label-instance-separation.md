# 실측 검증 — panoptic 인스턴스 분리 조건

**측정일**: 2026-07-25
**환경**: gz-sim 8.11.0 / gz-sensors 8.2.2 / gz-rendering vendor 0.0.7 / ros_gz 1.0.22,
Ubuntu 24.04 noble, GTX 1660 SUPER (드라이버 580.173.02, ogre2)
**재현**: [`sim/worlds/label_probe.sdf`](../../sim/worlds/label_probe.sdf) +
[`scripts/01_analyze_labels.py`](../../scripts/01_analyze_labels.py)

두 리서치 브리프가 "과실을 어떤 SDF 구조로 배치해야 개별 인스턴스로 분리되는가"에 대해
정면으로 충돌했다. 월드 생성기 아키텍처 전체가 여기 달려 있어 실측으로 결정했다.

---

## 1. 결과

각 구성마다 동일한 구체 3개를 배치하고, 서로 다른 semantic label 을 주고,
panoptic `labels_map` 에서 distinct 인스턴스 ID 개수를 셌다.

| 구성 | SDF 구조 | Label 부착 위치 | 기대 | 실측 | 판정 |
|---|---|---|---|---|---|
| A | 모델 1 / 링크 1 / `<visual>` 3개 | 각 `<visual>` | 3 | **1** | ❌ |
| B | 모델 1 / 링크 3개 | 각 링크의 `<visual>` | 3 | **1** | ❌ |
| C | 최상위 `<model>` 3개 | 각 `<model>` | 3 | **3** | ✅ |
| D | 최상위 `<include>` 3개 | 각 `<include>` | 3 | **3** | ✅ |
| E | 부모 모델 안 중첩 `<include>` 3개 | 각 중첩 `<include>` | 3 | **1** | ❌ |

**결론: 인스턴스 분리는 최상위(non-nested) 모델 단위로만 일어난다.**
`<visual>` 단위 Label 도, 링크 분리도, 중첩 `<include>` 도 전부 인스턴스 1개로 뭉개진다.

---

## 2. 어느 브리프가 맞았나

- **브리프 1 (TRAP 1) 이 맞다.** `Ogre2SegmentationMaterialSwitcher` 가
  `TopLevelModelVisual()->Name()` 이 바뀔 때만 인스턴스 카운터를 올린다는 설명과 정확히 일치한다.
- **브리프 2 §4.1 의 아키텍처 권고는 틀렸다.** "N개 apple `<visual>` 각각에 Label 플러그인을 달면
  인스턴스 ID 가 per-visual 로 자동 부여된다"는 서술은 실측과 반대다.
  이대로 180그루 × 60개 = 10,800개 과실을 `<visual>` 로 만들었다면 **마스크는 그럴듯하게 나오지만
  인스턴스는 나무당 1개**가 되어 착과 카운팅 정답이 통째로 무의미해졌을 것이다.

---

## 3. 부수적으로 확인된 정정 2건

### 3.1 panoptic labels_map 채널 인코딩이 브리프와 반대다

브리프는 `ch0 = label, ch1·ch2 = instance (16-bit big-endian)` 이라고 했다. 실측은 반대다:

```
channel 0 = 인스턴스 하위 바이트
channel 1 = 인스턴스 상위 바이트     →  instance = ch1 * 256 + ch0
channel 2 = semantic label
```

관측된 원시 튜플 (구성 C, label 42):
```
(1, 0, 42)  (2, 0, 42)  (3, 0, 42)
 └ 인스턴스 1,2,3        └ 라벨 42
```
브리프의 공식을 그대로 쓰면 라벨을 인스턴스로, 인스턴스를 라벨로 읽는다.
디코더는 반드시 [`scripts/01_analyze_labels.py`](../../scripts/01_analyze_labels.py) 의 것을 쓸 것.

### 3.2 gz-sim #1579 의 증상은 보고된 것과 다르다

이슈는 "중첩 라벨 include 가 `labels_map` 에 0 을 반환하고 서로 다른 복제본에 같은 색을 준다"고
보고한다. 실측에서는 **semantic 라벨은 정상**이었고(구성 E: label 44, 4,661 px 정상 검출),
**인스턴스만 1개로 합쳐졌다.** 즉 라벨이 사라지는 게 아니라 인스턴스 그룹핑이 부모 모델 기준으로
올라간다. 실질적 영향은 같으므로 회피 방법(플랫 월드)은 동일하다.

### 3.3 `<save>` 태그가 파일을 쓰지 않는다

`<camera><save enabled="true"><path>...</path></save></camera>` 를 켜고
`gz sim -s -r --iterations 300` 을 돌려도 출력 디렉토리가 생성되지 않았다
(헤드리스 EGL / 디스플레이 렌더링 양쪽 모두). 원인 미확인.
**정답 데이터 추출은 `<save>` 가 아니라 `ros_gz_bridge` + 구독 노드로 간다.**

### 3.4 `link.sdf` 에는 `<plugin>` 요소가 없다

sdformat 1.11 기준 `<plugin>` 은 `<model>` 과 `<visual>` 에만 허용되고 `<link>` 에는 없다.
링크 단위 라벨링은 문법적으로 불가능하다.

---

## 4. 월드 생성기에 대한 귀결

**과실별 인스턴스 ID 가 필요하면 그 과실은 최상위 `<include>` 여야 한다.** 나무 안에 넣을 수 없다.
따라서 설계서 §4.3 의 2계층 구조가 강제된다:

| 구역 | 구조 | 얻는 것 | 비용 |
|---|---|---|---|
| **배경 행** | 나무 1그루 = `<include>` 1개, 과실은 나무 메시에 구워 넣음 | semantic 라벨만 (`fruit_healthy`) | 엔티티 1개/그루 |
| **계측 블록** | 나무 `<include>` 1개 + **과실마다 최상위 `<include>`** | 과실별 인스턴스 ID, modal/amodal 박스, 가림률 | 엔티티 1 + N개/그루 |

계측 블록을 20그루 × 60과 = **1,200개 최상위 엔티티**로 잡으면 감당 가능하다.
전체 180그루에 적용하면 10,800개가 되어 드로우콜이 폭발한다 — 측정 없이 확대하지 말 것.

**나무 몸체 자체는 여전히 `<visual>` 단위 라벨링이 유효하다.** 인스턴스가 필요 없는
`trunk` / `branch` / `leaf_*` 는 semantic 만 있으면 되고, 이건 구성 A·B 에서 정상 동작을 확인했다.
