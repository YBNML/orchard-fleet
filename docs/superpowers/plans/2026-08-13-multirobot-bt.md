# 다중 로봇 + BT 임무 큐 + 교통관리 구현 계획 (스펙 ③)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** scout02(동일 센서)를 네임스페이스 다중화로 띄우고, 서버 AlleyLock 교통관리와 Behavior Tree 임무 큐로 2대 완전 동시 운용을 실증한다 (스펙: docs/superpowers/specs/2026-08-13-multirobot-bt-design.md).

**Architecture:** 로봇별 `/scout0N/` 토픽·`scout0N/*` TF 프레임(다행히 모델 SDF 토픽은 이미 상대경로 — gz 가 모델명으로 자동 스코핑하므로 브리지 매핑과 ROS 네임스페이스 push 가 핵심). 서버는 기존 robot_id 다중 구조 위에 AlleyLock(임무 단위 원자 점유)과 BT 실행기(asyncio 1 Hz 틱, DB 영속)를 얹는다. 로봇 계약 불변 — BT 액션은 기존 REST/cmd_result 소비자일 뿐.

**Tech Stack:** 스펙 ①②와 동일 + launch_ros 네임스페이스/리매핑.

## Global Constraints

- **1호기 회귀 무결**: 다중화 후 기존 단일 로봇 게이트(하네스 남측 4쌍 P0 n=1·3통로 정찰 무개입·self_test/relocalize 실기)가 그대로 통과해야 한다. 이것이 T2 의 합격 기준이다.
- **로봇 계약 불변**: robomw 프로토콜·명령·cmd_result 스키마 변경 금지(이중시작 가드는 동작 강화 — BUSY 거부 추가만). robomw ROS import 0 유지.
- **잠금 없이 발진되는 임무는 없다**: 수동 REST 발진도 AlleyLock 경로를 지난다. 충돌 규칙은 "통로 교집합 ≠ ∅ 또는 패드 교집합 ≠ ∅" (패드 P(A) = 임무 통로 집합에서 연속쌍 (k,k+1)).
- BT 노드는 5종(Sequence·Selector·Retry·Condition·Action)만 — 추가 금지(YAGNI).
- 빌드·재기동·프로세스 관례는 스펙 ①② 계획과 동일(콜콘 두 패키지, 스크래치 헬퍼 파일 경유, pgrep 자기일치 금지). 스크래치: `/tmp/claude-1000/-home-myhome-YBNML/691d883b-bd7f-499c-9b36-a59b0bd14a8a/scratchpad/`.
- 커밋 한국어, 태스크당 1회 이상.

---

## 파일 구조 (신설·수정)

```
scripts/gen_world.py                       # T1 — --robots "scout01:-14,-31.5,90 scout02:14,-31.5,90"
ros2_ws/src/orchard_sim/launch/control.launch.py  # T2 — robot_id 기반 네임스페이스·프레임·브리지 매핑
ros2_ws/src/orchard_sim/orchard_sim/…       # T2 — 절대 토픽·프레임 하드코딩 상대화(실사 후)
scripts/{42,46,…}                           # T2 — --robot 인자(기본 scout01)
server/fleet_server/
  traffic.py                                # T4 신설 — AlleyLock(충돌 규칙·획득·해제·조회)
  bt/{__init__,nodes,engine,presets}.py     # T5 신설 — BT 실행기
  api/bt_routes.py                          # T5 신설 — POST/GET/cancel
  api/mission_routes.py                     # T4 수정 — 잠금 경로·QUEUED_LOCK
  db.py(모델 실파일)                          # T4·T5 — alley_locks·bt_instances 테이블
  events 기록부(실파일 실사)                   # T6 — pong 미기록 + TTL 정리 태스크
robomw/profiles/orchard/mission.py          # T4 — 이중시작 BUSY 가드
server/web/index.html                       # T7 — BT 패널·점유 오버레이
```

---

### Task 1: 월드 다중 로봇 생성 (gen_world --robots)

**Files:**
- Modify: `scripts/gen_world.py` (현 `--robot` 단수 → `--robots` 목록, 하위 호환 유지)
- Test: 스크래치 검증 스크립트(월드 파싱)

**Interfaces:**
- Produces: `--robots "scout01:-14.0,-31.5,90 scout02:14.0,-31.5,90"` — `이름:x,y,yaw도` 공백 구분 목록. 각 항목이 `<include>` 로 scout_mini_mid70 모델을 그 이름·자세로 배치(z 는 기존 지형 안착 로직 재사용). 기존 `--robot NAME` 단수 인자는 `--robots "NAME:-14.0,-31.5,90"` 과 동치로 유지(하위 호환 — 기존 재생성 명령 불변).
- gz 는 모델 이름별로 상대 토픽을 스코핑하므로(`/world/W/model/scout02/...` 또는 `/model/scout02/...` — 실물 `gz topic -l` 로 확인) 월드 수준에서는 이름만 다르면 충돌 없음.

- [ ] **Step 1**: gen_world 의 로봇 include 조립부를 실사(`grep -n "robot" scripts/gen_world.py`)하고 목록 파싱+다중 include 로 확장. 참값 브리지 대상(`/gz_ground_truth` 발행 주체)이 모델별로 어떻게 갈리는지 함께 실사해 T2 로 넘길 메모를 리포트에 남겨라.
- [ ] **Step 2**: 재생성+대조 — `python3 scripts/gen_world.py --rows 10 --trees-per-row 41 --robots "scout01:-14.0,-31.5,90" --environment --detail 2 --instrumented-rows 0 --out /tmp/.../w1.sdf` 가 기존 `--robot scout_mini_mid70` 결과와 **로봇 include 의 name 제외 동일**한지 diff 로 증명(단, 기존 커밋 월드는 name=scout_mini_mid70 이므로 이름 이행은 T2 와 함께 — 이 태스크에서는 신형식 동작만 증명).
- [ ] **Step 3**: 2대 월드 생성 → gz 기동 스모크(헬퍼 복제) → `gz model --list` 또는 topic 목록으로 scout01·scout02 존재 + 라이다 토픽 2계열 확인 + RTF 1차 실측(bench_rtf.sh 참조) 기록.
- [ ] **Step 4**: Commit — `"gen_world 다중 로봇 — --robots 목록 (하위 호환 유지)"`

### Task 2: ROS 스택 네임스페이스화 + 1호기 회귀

**Files:**
- Modify: `launch/control.launch.py`(네임스페이스 push·브리지 gz→`/scout0N/*` 매핑·프레임 인자), `orchard_sim` 내 절대 토픽/프레임 하드코딩(실사: `grep -rn '"/cmd_vel"\|"/odom"\|"/livox\|"/imu\|"odom"\|"base_link"' ros2_ws/src/orchard_sim/orchard_sim/ | grep -v test`), map_localizer(odom_frame/base_frame 파라미터는 이미 존재 — 기본값을 robot_id 파생으로), 스크래치 헬퍼들(ROBOT 인자, 기본 scout01), `scripts/42_probe_robot_state.py`·`scripts/46_climb_harness.py`·`scripts/39_verify_localization_live.py`(--robot 인자)
- 월드 커밋: scout01 이름으로 재생성(orchard_nav.sdf 의 로봇 include name 이 scout_mini_mid70→scout01 로 바뀜)

**Interfaces:**
- Produces: `ros2 launch orchard_sim control.launch.py robot_id:=scout02 port:=8081 ns:=scout02` 식으로 로봇당 1세트 기동. 토픽 표준: `/scout0N/{cmd_vel,odom,livox/lidar,imu,…}`, TF: `map→scout0N/odom→scout0N/base_link`. `/gz_ground_truth` → `/scout0N/gz_ground_truth`.
- 검증 도구 계약: `--robot scout01` 기본 — 옛 호출 형태 전부 동작 유지.

- [ ] **Step 1**: 하드코딩 실사 → 상대화/파라미터화 목록 작성(리포트에 표) → 수정. TF 프레임은 파라미터 기본값을 `f"{robot_id}/odom"` 식으로.
- [ ] **Step 2**: 월드 재생성(scout01) + 콜콘 재빌드 + 스택 재기동(헬퍼 ROBOT 인자화 포함) — scout01 단독으로 전 구성 부활.
- [ ] **Step 3**: **1호기 회귀 게이트(합격 기준)** — 하네스 남측 4쌍 P0 n=1(4/4) · 3통로 정찰 무개입(cmd_result completed) · self_test all_ok · relocalize 성공 1건 · 대시보드 스모크(서버는 기존 접속 정보로 무수정 동작해야 — 어댑터가 8080 scout01 에 붙는 경로 불변).
- [ ] **Step 4**: Commit — `"네임스페이스 다중화 — scout0N 토픽·TF, 검증도구 --robot (1호기 회귀 전건)"`

### Task 3: scout02 기동 + 단독 게이트

**Files:**
- Modify: 스크래치 헬퍼(2호기 세트: restart_agent2.sh 등 — ROBOT/PORT 인자로 공통화 가능하면 공통화), 서버 DB 시드(scout02 행 — 기존 admin API 또는 시드 스크립트), `relaunch` 계열 2호기판
- Test: scout02 실기

**Interfaces:**
- Consumes: T1 월드(2대) + T2 네임스페이스.
- Produces: scout02 스택(agent 8081·localizer·bridge) 기동 절차 + 서버에 scout02 등록(conn config 로 ws://…:8081). 시작 위치 통로 8 남단(14.0,-31.5,90).

- [ ] **Step 1**: 2대 월드로 시뮬 재기동 + scout02 스택 기동 + 서버 등록 → 대시보드에 로봇 칩 2개·텔레메트리 2계열 확인.
- [ ] **Step 2**: **scout02 단독 게이트**: 정찰 3통로(work scout, alleys [5,6,7]) 무개입 완주 + est RMS 정상(참값 대조 --robot scout02). scout01 은 idle 로 세워둔 채 — 정지 로봇이 상대 라이다에 잡혀도 오염 없는지 1차 확인.
- [ ] **Step 3**: Commit — `"scout02 기동 — 2호기 스택·서버 등록·단독 정찰 게이트"`

### Task 4: AlleyLock + 이중시작 가드

**Files:**
- Create: `server/fleet_server/traffic.py`, 마이그레이션(alley_locks 테이블)
- Modify: `server/fleet_server/api/mission_routes.py`(발진 전 잠금·QUEUED_LOCK), mission 상태 전이 훅(해제), `robomw/profiles/orchard/mission.py`(이중시작 BUSY)
- Test: `server/tests/test_traffic.py`, robomw `tests/test_work_mission.py` 확장

**Interfaces:**
- Produces: `traffic.pads(alleys: list[int]) -> set[tuple]` — 연속쌍만. `traffic.conflict(a: list[int], b: list[int]) -> bool` — 통로 교집합 or 패드 교집합. `AlleyLocks.acquire(session, robot_id, mission_id, alleys) -> (ok, reason)` 원자(동일 트랜잭션), `release(session, mission_id)`, `list_active(session)`.
- REST: 발진 시 획득 실패 → 임무 상태 `QUEUED_LOCK`(신규 상태 — 기존 상태기계에 additive) + detail 에 사유. `GET /alley-locks` 목록. 잠금 해제는 COMPLETED/CANCELLED/FAILED 전이 훅에서.
- 로봇: mission 진행 중(`self.mission is not None`) mission_start → `emit_cmd_result(..., "rejected", code="BUSY", data={"reason":"임무 진행 중"})` + 기존 임무 유지(조용한 교체 제거).

- [ ] **Step 1: 실패 테스트** (`test_traffic.py` — 충돌 규칙 진리표):
```python
from fleet_server.traffic import conflict, pads

def test_pads_consecutive_only():
    assert pads([0,1,2,3]) == {(0,1),(1,2),(2,3)}
    assert pads([0,2,4]) == set()          # 비연속은 패드 없음
    assert pads([5]) == set()

def test_conflict_rules():
    assert not conflict([0,1,2,3], [5,6,7,8])   # 통로4 버퍼 — 허용
    assert conflict([0,1,2,3,4], [4,5,6,7,8])   # 통로4 공유
    assert not conflict([0,1,2,3], [4,5,6,7,8]) # 패드 (2,3) vs (4,5).. 분리 — 허용
    assert conflict([0,1], [1,2])               # 통로 1 공유
    assert not conflict([0,1,2], [3])           # 통로·패드 모두 분리 ([3] 패드 ∅)
```
- [ ] **Step 2~3**: 구현(획득은 mission 생성 트랜잭션 안에서) → 진리표+획득/해제/복원 pytest, robomw 이중시작 테스트(`test_double_start_rejected_busy`: 임무 중 start → BUSY·기존 임무 유지) RED→GREEN.
- [ ] **Step 4**: 실기 — scout01 임무 중 REST 로 겹치는 임무 요청 → QUEUED_LOCK 확인, cancel 로 정리. 이중시작: ws 로 mission_start 2연발 → 두 번째 BUSY.
- [ ] **Step 5**: Commit — `"AlleyLock 교통관리 + 임무 이중시작 BUSY 가드"`

### Task 5: BT 실행기 + 프리셋 + API

**Files:**
- Create: `server/fleet_server/bt/nodes.py`(5종 — 순수 로직), `bt/engine.py`(asyncio 1 Hz 틱·영속·복원), `bt/presets.py`(3종), `api/bt_routes.py`, 마이그레이션(bt_instances)
- Test: `server/tests/test_bt_nodes.py`, `test_bt_engine.py`(InMemoryFleetPort 주입 관례)

**Interfaces:**
- Produces (nodes.py — 순수, DB·HTTP 무지):
```python
# tick(ctx) -> "running"|"success"|"failure"; ctx 는 콜백 사전:
#   ctx.alley_free(alleys)->bool · ctx.robot_idle(robot)->bool · ctx.robot_online(robot)->bool
#   ctx.start_mission(spec)->mission_id|None(잠금 대기)  · ctx.mission_status(mission_id)->str
Sequence(children) / Selector(children) / Retry(n, child) /
Condition(kind, arg) / Action(spec)      # spec={robot, alleys?, work?}
# 직렬화: to_state()/from_state() — node_states 트리(JSON) 왕복
```
- engine: `BTEngine(session_factory, fleet)` — `create(preset, params)->id`, `cancel(id)`, 1 Hz 틱 태스크, 서버 lifespan 에 통합, 기동 시 RUNNING 복원(Action 은 mission_id 재부착·상태 재판정).
- presets: `full_split_patrol(robotA, robotB, split_k=4)` → 인스턴스 2개 생성(각: Sequence[Condition(robot_online), Condition(robot_idle), Action(정찰 분할)]) — A=[0..split_k-1], B=[split_k+1..8](split_k 는 버퍼로 비움) · `sequential_retry(robot, alleys, n=2)` → Sequence[Retry(n, Action(정찰))] · `single_alley_loop(robot, alley, n)` → Retry(n, Action([alley])).
- API: `POST /bt {preset, params}` → {ids} · `GET /bt` → 인스턴스+node_states · `POST /bt/{id}/cancel`.

- [ ] **Step 1: 실패 테스트** (nodes 의미론 — Fake ctx):
```python
def test_sequence_fails_fast():   # 첫 실패에서 중단
def test_retry_counts():          # n회 재시도 후 실패 확정
def test_condition_waits():       # 불충족 → running (실패 아님)
def test_action_lifecycle():      # start→running(mission_id)→COMPLETED=success / FAILED=failure
def test_state_roundtrip():       # to_state→from_state 후 동일 거동
```
(각 케이스는 Fake ctx 의 반환 시퀀스를 조작해 단언 — 구현자는 케이스당 3~6줄 본문 작성.)
- [ ] **Step 2~3**: RED → nodes 구현(각 노드 ~15줄) → engine(InMemoryFleetPort 로 Action 디스패치 검증·복원 테스트: 엔진 재생성 후 RUNNING 인스턴스 이어감) → presets·routes.
- [ ] **Step 4**: server pytest 전건 + Commit — `"BT 임무 큐 — 노드 5종·엔진·프리셋 3종·API (영속 복원)"`

### Task 6: pong 미기록 + 이벤트 TTL

**Files:**
- Modify: 이벤트 DB 기록부(실사: `grep -rn "events" server/fleet_server/fleet/ --include=*.py` — on_telemetry 의 evt 분기), lifespan(TTL 정리 태스크), settings(EVENT_TTL_DAYS=7)
- Test: server/tests 2케이스

- [ ] **Step 1: 실패 테스트** — pong 이벤트 주입 시 DB 미기록(다른 kind 는 기록) · TTL 초과 행 정리 태스크 1회 실행 후 삭제 확인.
- [ ] **Step 2~3**: 구현(기록 제외 목록 `EVENT_KIND_SKIP={"pong"}` + 기동 시·24h 주기 DELETE) → GREEN → 실기 5분 pong 행 0 확인.
- [ ] **Step 4**: Commit — `"이벤트 보존정책 — pong 미기록·TTL 7일 정리"`

### Task 7: 대시보드 — BT 패널 + 점유 오버레이

**Files:**
- Modify: `server/web/index.html`

**Interfaces:**
- BT 패널(pane, operator): 프리셋 select(3종)+파라미터 입력(로봇 select·분할 k)+발진 버튼 → POST /bt. 인스턴스 목록(8s 폴링 — 개입 큐 관례) + 트리 렌더(중첩 ul — 노드별 상태색: 대기 muted/실행 gold/성공 leaf/실패 danger) + 취소.
- 점유 오버레이: 지도 캔버스에 GET /alley-locks 의 통로 밴드를 로봇 색 반투명으로(기존 로봇 색 배정 로직 재사용).
- 기존 관례(api()·data-min-role·CSS 변수) 준수, 신규 전역 CSS 최소.

- [ ] **Step 1**: 구현 → **Step 2**: Playwright — BT 발진(분담 프리셋, 즉시 취소)·트리 상태 렌더·오버레이 표시·콘솔 오류 0 + 스크린샷 2장(docs/figures/dashboard_bt_panel.png·dashboard_alley_locks.png) → **Step 3**: Commit — `"대시보드 — BT 패널·통로 점유 오버레이"`

### Task 8: 동시 운용 최종 게이트 + 문서

**Files:**
- Create: `docs/findings/2026-08-13-multirobot-bt.md`
- Modify: `ros2_ws/src/robomw/README.md`(다중 로봇 온보딩 한 절), 루트 README 현재 상태 갱신

- [ ] **Step 1: 스펙 §7 게이트 전 항목** — 특히:
  - **동시 운용**: `full_split_patrol(scout01, scout02, 4)` 발진 → 2대 동시 정찰([0..3]/[5..8]) 양쪽 무개입 완주(각 ~20분·전속) + 각 로봇 통로 안 est RMS ≤0.3(상호 라이다 오염 판정) + RTF 실측 기록
  - 교통관리: 동시 운용 중 겹치는 임무 REST 요청 → QUEUED_LOCK → 선행 종료 후 BT Condition 경유 자동 발진 실증
  - BT: sequential_retry 실패→재시도 실증(로컬라이저 잠시 내려 1차 실패 유도) · 서버 재기동 복원 1건
  - 이중시작 BUSY·pong 0행·대시보드 Playwright·1호기 회귀 스팟(하네스 4/4)
  - 실패 항목은 고치지 말고 BLOCKED(원인 분석 포함).
- [ ] **Step 2**: findings(수치 표+RTF+오염 측정) + README 갱신 → **Step 3**: Commit — `"스펙 ③ 완료 — 동시 운용·교통관리·BT 게이트 전 항목"`
