# 다중 로봇 + Behavior Tree 임무 큐 + 교통관리 — 설계 (스펙 ③)

| 항목 | 내용 |
|---|---|
| 날짜 | 2026-08-13 |
| 전제 | 스펙 ①·② 병합 완료(robomw 계약·cmd_result·정찰 work·유지보수 명령) |
| 목표 | 동일 센서 2호기(scout02)의 **완전 동시 운용**을 통로 잠금 교통관리와 서버측 Behavior Tree 임무 큐로 실증 |
| 범위 | 시뮬·스택 다중화 · 서버 AlleyLock · BT 실행기+프리셋 · 대시보드(트리 상태·점유 맵) · 이월 2건(이중시작 가드·pong 보존정책) |
| 범위 밖 | 통로 단위 동적 잠금(주행 중 clearance 협상 — 로봇 계약 변경, 후속 스펙) · BT 편집기 UI · 비동기 relocalize |

## 0. 원칙

1. **다중화는 네임스페이스로** — 로봇별 토픽·TF 를 `scout0N/` 아래로. 전역 이름은 map 프레임과 시뮬 월드 자원뿐.
2. **교통관리 v1 은 임무 단위 잠금** — 발진 시점에 통로 집합+횡단 패드를 원자 점유, 겹치면 대기. 단순하고 증명 가능한 안전(공간 분리)이 우선.
3. **BT 는 서버의 언어** — 로봇 계약(①②)은 불변. BT 액션이 기존 REST/cmd_result 를 소비할 뿐, 로봇은 BT 의 존재를 모른다.
4. **1호기 회귀 무결** — 다중화 후에도 기존 단일 로봇 게이트(하네스·정찰·유지보수 명령)가 그대로 통과해야 한다.

## 1. 시뮬·스택 다중화

### 네임스페이스 설계
- ROS 토픽: `/scout01/cmd_vel`·`/scout01/odom`·`/scout01/livox/lidar`·`/scout01/imu` … (에이전트·로컬라이저·브리지는 네임스페이스 push 로 기동 — 코드 내 상대 토픽 유지가 원칙, 절대경로 하드코딩은 이 기회에 상대화).
- TF: 프레임 `scout01/odom`·`scout01/base_link` (map 은 공유). map_localizer 는 `map→scout0N/odom` 발행. 로봇별 `tf`/`tf_static` 는 전역 토픽 유지(프레임 이름으로 구분 — ROS 관례).
- gz 월드: scout 모델 include 를 로봇 인자 목록으로 확장(`gen_world --robots scout01:-14,-31.5,90 scout02:14,-31.5,90` 형식). 모델별 이름·플러그인 토픽 접두(`/model/scout0N/...`) → 브리지가 네임스페이스로 매핑. scout02 시작 위치는 통로 8 남단.
- 링크 포트: scout01 ws 8080(기존), scout02 8081. 서버 DB 에 scout02 로봇 행 추가(conn config 로 포트 지정 — 기존 register_robot 경로 재사용).
- 참값: /gz_ground_truth 를 모델별로 분리(브리지 remap) — 검증 스크립트가 robot 인자를 받게 확장.

### 헬퍼·스크립트 영향
- restart 계열 헬퍼는 robot 인자화(ROBOT=scout02 …). 기존 무인자 호출은 scout01 동작 유지(하위 호환).
- 42_probe·46 하네스 등 검증 도구는 `--robot`(기본 scout01) 추가.

### 알려진 리스크 (게이트에서 측정)
- **RTF**: gpu_lidar 360° 2대. 게이트에서 RTF 실측, 0.5 미만이면 scout02 라이다 표본 감축(240×40)으로 절충하고 기록.
- **상호 라이다 오염**: 상대 로봇이 구조점 군집으로 잡힌다. 통로 분리(버퍼 포함) 하에서 위상 히스토그램 오염은 미미할 것으로 예상 — 동시 운용 게이트에서 est RMS 로 실측하고, 문제 시 이동 군집 필터를 후속 항목으로.

## 2. 교통관리 — AlleyLock (서버)

- 자료: `alley_locks` 테이블 `(robot_id, alleys_json, pads_json, mission_id, ts)`.
- **점유 규칙**: 임무의 통로 집합 A 에 대해 패드 집합 P(A) = {(k,k+1) | k,k+1 이 A 에서 연속}. 두 점유가 충돌 ⇔ 통로 교집합 ≠ ∅ **또는** 패드 교집합 ≠ ∅. (예: [0..3] 과 [5..8] 은 양쪽 다 공집합 → 동시 허용. [0..4] 와 [4..8] 은 통로 4 공유 → 차단.)
- 획득: `POST /missions` 처리에서 원자적으로 시도. 실패 시 임무를 **QUEUED_LOCK** 상태로 생성(로봇 미발진)하고 사유(충돌 로봇·통로)를 기록 — BT Condition 이 이 상태를 소비한다. 잠금 없이 발진되는 임무는 없다(수동 발진도 동일 경로).
- 해제: 임무 COMPLETED/CANCELLED/FAILED 판정 시(기존 mission 상태 전이 훅). 서버 재기동 시 RUNNING 임무의 잠금은 DB 에서 복원.
- 관제 표시용 조회: `GET /alley-locks`.

## 3. Behavior Tree 임무 큐 (서버)

### 노드 의미론 (v1 — 5종만, YAGNI)
```
Sequence(children)   — 앞에서부터, 실패 시 실패
Selector(children)   — 앞에서부터, 성공 시 성공
Retry(n, child)      — 실패 시 최대 n회 재시도
Condition(type, arg) — alley_free(alleys) | robot_idle(robot) | robot_online(robot)
                       불충족 시 RUNNING(대기) — 폴링 틱마다 재평가
Action(mission spec) — {robot, alleys?, work?} → POST /missions 경로 재사용(잠금 포함),
                       임무 종료 상태로 성공/실패 판정 (COMPLETED=성공)
```
- 실행기: 서버 내 asyncio 태스크, 1 Hz 틱. 트리 인스턴스별 상태기계(node_states: idle/running/success/failure).
- 영속: `bt_instances(id, name, tree_json, state_json, status, created)` — 서버 재기동 시 RUNNING 인스턴스 복원(진행 중 Action 은 mission_id 로 재부착).
- API: `POST /bt {preset, params}` · `GET /bt`(목록+노드 상태) · `POST /bt/{id}/cancel`(진행 임무 cancel 포함).

### 프리셋 3종
1. **분담 전체 정찰**: 파라미터 (robotA, robotB, split_k) → 병렬 아님 — 인스턴스 2개 생성(각 로봇 Sequence[Condition(robot_idle), Action(정찰 alleys 분할)]). 기본 분할 [0..3]/[5..8](통로 4 버퍼).
2. **순차 정찰+재시도**: Sequence[Retry(2, Action(정찰)), Action(복귀 없음 — v1 은 정찰만)] — 단일 로봇.
3. **단일 통로 반복**: Retry(n, Action(단일 통로 정찰)) — 데모·시험용.

## 4. 대시보드

- **BT 패널**(pane, data-min-role operator): 인스턴스 목록 + 선택 시 트리 렌더(중첩 리스트 — 노드별 색: 대기 회색/실행 leaf점멸/성공 leaf/실패 danger), 프리셋 발진 폼(프리셋 선택+로봇·분할 파라미터), 취소 버튼.
- **통로 점유 오버레이**: 지도에 잠금 통로를 로봇 색 반투명 밴드로(GET /alley-locks 폴링 8s — 기존 개입 큐 폴링 관례).
- 기존 다중 로봇 요소(로봇 칩·지도 중첩) 활용, teleop/유지보수 카드는 기존대로 선택 로봇 기준.

## 5. 이월 2건

- **임무 이중시작 가드(로봇)**: mission 진행 중 새 mission_start → `rejected(BUSY, "임무 진행 중")` (cmd_result). 기존 '조용한 교체' 제거 — 교체하려면 cancel 후 start (BT·서버 큐가 준수). ②에서 관찰된 state.work 잔류도 함께 소멸.
- **pong 보존정책(서버)**: `kind='pong'` 은 DB 기록 자체를 제외(링크 판정은 어댑터 메모리로 충분). 그 외 이벤트는 보존기간 7일 — 서버 기동 시+일 1회 정리 태스크. 설정으로 조정 가능(`EVENT_TTL_DAYS`).

## 6. 오류 처리

- AlleyLock 획득 실패: 임무 QUEUED_LOCK + 사유 — BT 는 대기, 수동 발진은 대시보드에 사유 토스트.
- BT Action 의 임무가 FAILED/로봇 오프라인: 노드 실패 → Retry/Selector 규칙대로. 재기동 복원 시 사라진 임무(로봇 재시작 등)는 실패로 판정.
- 로봇 이중시작 BUSY 는 BT 에선 정상 흐름(Condition robot_idle 로 예방, 경합 시 Action 실패→Retry).
- 2호기 링크 다운: scout01 운용에 영향 없어야 함(어댑터 독립성 — 기존 robot_id 분리 구조 그대로).

## 7. 검증 게이트

| 게이트 | 기준 |
|---|---|
| 1호기 회귀 | 네임스페이스 전환 후 기존 게이트 재통과: 하네스 남측 4쌍 P0 n=1 · 3통로 정찰 무개입 · self_test/relocalize 실기 |
| 2호기 단독 | scout02 로 3통로([5..8] 중) 정찰 무개입 완주 |
| **동시 운용** | 분담 프리셋으로 2대 동시 정찰([0..3]/[5..8]) — 양쪽 무개입 완주 + est RMS 상호 오염 없음(각 통로 안 RMS ≤0.3) + RTF 실측 기록 |
| 교통관리 | 겹치는 임무 요청 → QUEUED_LOCK 대기 → 선행 임무 종료 시 자동 발진(BT Condition 경유) 실증 |
| BT | 프리셋 3종 E2E(성공·재시도·취소) + 서버 재기동 복원 1건 |
| 이중시작 가드 | 임무 중 mission_start → BUSY, state.work 잔류 없음 |
| pong 정책 | 5분 운용 후 pong 행 0 확인 + TTL 정리 태스크 동작 |
| 대시보드 | Playwright: BT 패널 렌더·발진·노드 상태 변화·점유 오버레이·콘솔 오류 0 |
| 서버 pytest | 기존 114 + AlleyLock·BT 단위(충돌 규칙·노드 의미론·복원) |
