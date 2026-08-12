# 신규 명령 동작 + 대시보드 명령 UI 구현 계획 (스펙 ②)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 스펙 ①이 라우팅만 뚫어둔 명령들(정찰 work·self_test·relocalize·blackbox_dump)의 실제 동작과 대시보드 명령 UI를 붙인다 (스펙: docs/superpowers/specs/2026-08-12-command-behaviors-dashboard-design.md).

**Architecture:** 명령 처리는 robomw feature 로(신규 `maintenance` feature + 기존 mission feature 확장), 하드웨어·ROS 접점은 orchard_sim 어댑터로(SimWork·ScoutDiag·RosSensors 확장). 어댑터 주입은 스펙①의 관례(bb.extra 경유 — cloud_sinks 전례)를 따른다. 대시보드는 기존 단일 파일에 카드 1개+버튼 1개+토스트만 추가.

**Tech Stack:** 스펙 ①과 동일 (robomw는 ROS import 0, pytest·기존 헬퍼·Playwright 재사용)

## Global Constraints

- **안전 불변**: 어떤 신규 동작도 SafetyArbiter·속도 단일 경로·estop 2단계를 우회하지 않는다. self_test 는 움직임을 만들지 않는다(구동 테스트 금지). relocalize 는 주행 중 `rejected(BUSY)`.
- **robomw ROS import 0건** (pytest test_no_ros_imports 상시) — ROS 접점은 orchard_sim/adapters/ 에만.
- **계약 additive**: 텔레메트리 state 에 `work` 키 추가만. 기존 명령·이벤트·스키마 불변, 기존 대시보드 동작 불변(새 UI는 추가).
- **능력 게이트**: 로봇 `hello.capabilities.work.types=["scout"]`, 미선언 유형의 mission_start 는 `rejected(UNSUPPORTED, "미지원 작업 유형")`.
- cmd_result 상태·코드는 스펙①의 닫힌 집합만 사용 (`P.make_cmd_result`).
- 빌드/재기동/프로세스 조작: 스펙① 계획과 동일 — colcon 두 패키지, 헬퍼 파일 경유(pgrep 자기일치 함정), 스크래치 `/tmp/claude-1000/-home-myhome-YBNML/691d883b-bd7f-499c-9b36-a59b0bd14a8a/scratchpad/`.
- 커밋 한국어, 태스크당 1회 이상.

---

## 파일 구조 (이번 스펙에서 만들거나 고치는 것)

```
ros2_ws/src/robomw/robomw/
  features/maintenance.py        # 신규 — self_test·relocalize·blackbox_dump 명령 처리
  features/telemetry_state.py    # 수정 — state 에 work 키 additive
  profiles/orchard/mission.py    # 수정 — work 수용(전통로 자동·speed_scale·능력 게이트)·work_stop
  core/blackbox.py               # 신규 — 궤적 링버퍼(1 Hz·900 s)·npz 저장 (numpy 사용, ROS 무관)
  tests/test_work_mission.py     # 신규
  tests/test_maintenance.py      # 신규
ros2_ws/src/orchard_sim/orchard_sim/
  adapters/sim_work.py           # 신규 — Work SDK 구현(정찰 시뮬)
  adapters/scout_diag.py         # 신규 — Diag SDK 구현(수신율·측위·링크·드라이브 점검, blackbox 저장 위임)
  adapters/ros_sensors.py        # 수정 — 수신율 카운터 + reinit 발행(Localizer.reinit)
  control_agent.py               # 수정 — 어댑터 주입(bb.extra)·궤적 링 1 Hz 급전·supported 목록 갱신·capabilities
  map_localizer.py               # 수정 — ~/reinit 토픽(JSON) 수신 → 재초기화
server/fleet_server/
  api.py(또는 missions 라우터 실파일) # 수정 — /missions 의 work 통과·alleys 생략 허용
  ws.py                          # 수정 — _ALWAYS_REVALIDATE 에 relocalize
server/web/index.html            # 수정 — 정찰 버튼·유지보수 카드·cmd_result 토스트·cmd_id 부여
```

주: 명령 → SDK 어댑터 접근은 bb.extra 관례를 쓴다 — control_agent 가
`bb.extra["sdk_work"] = SimWork(...)`, `bb.extra["sdk_diag"] = ScoutDiag(...)`,
`bb.extra["sdk_localizer"] = self.sensors` 를 넣고 feature 가 꺼내 쓴다
(스펙① cloud_sinks 전례). 정식 Context 창구 승격은 스펙 ③ 이후 후보.

---

### Task 1: 정찰 work — SimWork + mission 확장 + state.work

**Files:**
- Create: `ros2_ws/src/orchard_sim/orchard_sim/adapters/sim_work.py`
- Modify: `ros2_ws/src/robomw/robomw/profiles/orchard/mission.py` (on_command 의 mission_start·신규 work_stop), `ros2_ws/src/robomw/robomw/features/telemetry_state.py`, `ros2_ws/src/orchard_sim/orchard_sim/control_agent.py` (SimWork 주입·capabilities.work.types·supported 에 work_stop)
- Test: `ros2_ws/src/robomw/robomw/tests/test_work_mission.py`

**Interfaces:**
- Consumes: `robomw.sdk.interfaces.Work`, `robomw.sdk.types.WorkStatus`, `P.validate_work`, mission 의 기존 `_report`/coverage 축적(①T7), `ctx.emit_cmd_result`
- Produces: `SimWork(bb)` — Work 구현: `start("scout", params)` 는 내부 `active/type/scale` 만 설정, `status()` 는 `WorkStatus(active, "scout", progress=bb.extra.get("mission_coverage",0.0), "")`. mission 이 매 tick `bb.extra["mission_coverage"]` 갱신(coverage 계산 재사용).
- Produces: mission_start 처리 규칙 —
  ```
  work 있음 → validate_work 불합격: rejected(BAD_PARAM)
            → type ∉ bb.extra["work_types"](로봇 능력): rejected(UNSUPPORTED, "미지원 작업 유형")
            → alleys 생략: alleys = list(range(R-1)) 자동
            → speed_scale: self._speed_scale 저장, tick 의 속도 요청에 곱함(기본 1.0)
            → bb.extra["sdk_work"].start(type, params) 호출
  work_stop → sdk_work.stop(); state work 키 제거용 플래그; cmd_id 있으면 completed
  임무 완료/취소 → sdk_work.stop()
  ```
- Produces: telemetry_state — `sdk_work.status().active` 이면 state dict 에 `"work": {"type": t, "progress": round(p,3)}` 추가, 아니면 키 없음.

- [ ] **Step 1: 실패 테스트 작성** (`test_work_mission.py`) — mission feature 를 가짜 ctx(bb·safety 스텁, ①의 기존 테스트에 스텁 전례 없음 → 최소 스텁을 이 파일에 정의)로 구동:
```python
import robomw.link.protocol as P
from robomw.profiles.orchard.mission import DriveMission


class FakeWork:
    def __init__(self): self.started = None; self.stopped = 0
    def start(self, t, p): self.started = (t, p); return True
    def stop(self): self.stopped += 1
    def status(self):
        from robomw.sdk.types import WorkStatus
        return WorkStatus(self.started is not None and not self.stopped, "scout", 0.5, "")


def mk_mission(work_types=("scout",)):
    # DriveMission 생성에 필요한 최소 ctx 스텁은 mission.py 의 __init__ 이 쓰는
    # 인터페이스(pr 파라미터 함수·bb·safety.snapshot/set_paused·event)를 실측해 맞춘다.
    ...  # 구현자: mission.py __init__ 을 읽고 스텁 구성 (기존 t7 스모크 참조)


def test_scout_defaults_all_alleys():
    m, ctx = mk_mission()
    m.on_command(P.CMD_MISSION_START, {"work": {"type": "scout"}, "cmd_id": "w1"})
    assert m.mission is not None
    assert m.mission["alleys"] == list(range(9))
    assert ctx.bb.extra["sdk_work"].started[0] == "scout"


def test_unsupported_work_type_rejected():
    m, ctx = mk_mission(work_types=("scout",))
    m.on_command(P.CMD_MISSION_START, {"work": {"type": "spray"}, "cmd_id": "w2"})
    assert m.mission is None
    last = ctx.results[-1]
    assert last["status"] == "rejected" and last["code"] == "UNSUPPORTED"


def test_bad_speed_scale_rejected():
    m, ctx = mk_mission()
    m.on_command(P.CMD_MISSION_START,
                 {"work": {"type": "scout", "params": {"speed_scale": 3.0}}, "cmd_id": "w3"})
    assert m.mission is None and ctx.results[-1]["code"] == "BAD_PARAM"


def test_work_stop_keeps_mission():
    m, ctx = mk_mission()
    m.on_command(P.CMD_MISSION_START, {"work": {"type": "scout"}, "cmd_id": "w4"})
    m.on_command(P.CMD_WORK_STOP, {"cmd_id": "w5"})
    assert m.mission is not None                      # 주행은 계속
    assert ctx.bb.extra["sdk_work"].stopped == 1
```
(mk_mission 의 스텁 세부는 구현자가 mission.py 실물에 맞춰 완성 — "..." 는 스텁 구성 지시이지 미완 명세가 아님. ctx.results 는 emit_cmd_result 를 가로채는 리스트.)
- [ ] **Step 2: 실패 확인** — `python3 -m pytest ros2_ws/src/robomw/tests/test_work_mission.py -v`
- [ ] **Step 3: 구현** — Produces 규칙대로. speed_scale 적용 지점: tick 이 만드는 모든 전진 VelocityRequest 의 v 에 `self._speed_scale` 곱(회전·복구 후진은 제외 — 안전 동작 불변). SimWork 는 ~40줄.
- [ ] **Step 4: 전체 pytest + 빌드 + 실기 스모크** — 재빌드·restart_agent 후 ws 스모크(①T6 스모크 스크립트 확장): `mission_start{work:{type:"spray"},cmd_id}` → UNSUPPORTED 확인, `mission_start{work:{type:"scout"},cmd_id}` → accepted + state 에 work 키 등장 확인 → `mission_cancel`.
- [ ] **Step 5: Commit** — `"정찰 작업 — SimWork·전통로 자동·능력 게이트·state.work"`

### Task 2: maintenance feature + ScoutDiag self_test

**Files:**
- Create: `ros2_ws/src/robomw/robomw/features/maintenance.py`, `ros2_ws/src/orchard_sim/orchard_sim/adapters/scout_diag.py`
- Modify: `ros2_ws/src/orchard_sim/orchard_sim/adapters/ros_sensors.py` (수신율 카운터: feed_cloud/feed_imu 호출 시각 deque(3초 창) + `rates() -> {"lidar":hz,"imu":hz}`), `control_agent.py` (ScoutDiag 주입·DEFAULT_FEATURES 에 maintenance·supported 에 self_test·capabilities.diag.items)
- Test: `ros2_ws/src/robomw/robomw/tests/test_maintenance.py`

**Interfaces:**
- Consumes: `robomw.sdk.interfaces.Diag`, `robomw.sdk.types.SelfTestItem`, bb.extra["sdk_diag"], `ctx.emit_cmd_result`
- Produces: `MaintenanceFeature` — feature 관례(commands 선언: self_test·relocalize·blackbox_dump; ①의 teleop.py 를 형식 참조). self_test 처리:
  ```
  임무 RUNNING(bb.extra["mode"]==P.MODE_MISSION 및 mission 존재) → rejected(BUSY, "주행 중")
  아니면 items = payload.get("items") 또는 전체 → diag.self_test(items)
  → completed, data={"items":[{"name","ok","detail"}...], "all_ok": all(...)}
  ```
- Produces: `ScoutDiag(sensors, safety, drive, robot_id)` — Diag 구현. 항목 판정(스펙 §2 그대로): lidar ≥8 Hz · imu ≥80 Hz · localizer(pose 존재+quality≥0.3) · link(safety 의 최근 클라이언트 수신 <1.5 s — safety.snapshot() 실측 필드 사용) · drive(limits() v_max>0). blackbox_dump 는 Task 4 전까지 `NotImplementedError`.

- [ ] **Step 1: 실패 테스트** (`test_maintenance.py`) — FakeDiag 로 feature 로직만:
```python
import robomw.link.protocol as P
from robomw.features.maintenance import MaintenanceFeature
from robomw.sdk.types import SelfTestItem


class FakeDiag:
    def self_test(self, items):
        return [SelfTestItem("lidar", True, "9.8 Hz"), SelfTestItem("imu", False, "42 Hz")]
    def blackbox_dump(self, window_s): raise NotImplementedError


def test_self_test_reports_items():
    f, ctx = mk_feature(diag=FakeDiag())          # mk_feature 스텁은 Task1 테스트 참조 형식
    f.on_command(P.CMD_SELF_TEST, {"cmd_id": "s1"})
    last = ctx.results[-1]
    assert last["status"] == "completed" and last["data"]["all_ok"] is False
    assert {i["name"] for i in last["data"]["items"]} == {"lidar", "imu"}


def test_self_test_busy_while_driving():
    f, ctx = mk_feature(diag=FakeDiag(), in_mission=True)
    f.on_command(P.CMD_SELF_TEST, {"cmd_id": "s2"})
    assert ctx.results[-1]["code"] == "BUSY"
```
- [ ] **Step 2: 실패 확인 → Step 3: 구현 → Step 4: 전체 pytest 통과**
- [ ] **Step 5: 실기 게이트** — 재빌드·restart_agent → 정지 상태 self_test all_ok=true 확인. 라이다 검출 시험: livox 브리지를 헬퍼로 내리고(brige 재기동 절차는 restart_sim.sh 5단계 참조 — 브리지만 내렸다 올리는 미니 헬퍼를 스크래치에 작성) self_test → lidar ok=false 확인 → 브리지 복구 → all_ok 재확인.
- [ ] **Step 6: Commit** — `"유지보수 기능 — self_test 5항목 (ScoutDiag·수신율 카운터)"`

### Task 3: relocalize — 운영자 복구의 명령화

**Files:**
- Modify: `ros2_ws/src/orchard_sim/orchard_sim/map_localizer.py` (`~/reinit` String 토픽 구독 — JSON {"x","y","yaw"} 수신 시 INIT 파라미터 절차와 동일한 재초기화), `adapters/ros_sensors.py` (`reinit(pose)` 가 그 토픽 발행 + 발행 후 2초 내 diagnostics quality 폴링), `robomw/features/maintenance.py` (relocalize 처리), `control_agent.py` (supported 에 relocalize)
- Test: `test_maintenance.py` 확장 + 실기

**Interfaces:**
- Produces: maintenance 의 relocalize 처리 —
  ```
  주행 중 → rejected(BUSY, "주행 중 재초기화 불가")
  payload {x,y,yaw} 또는 {alley,end} (격자 변환: x=x0+(k+0.5)S; y=±(col_l/2+1.5) — north=+, south=−; yaw: north 단이면 -pi/2, south 단이면 +pi/2)
  → localizer.reinit(Pose(x,y,yaw,1.0)) → True: completed(data={"quality":q}) / False: failed(TIMEOUT, data=진단 스냅샷)
  ```
  격자 상수는 bb.extra 의 hello 기하(이미 control_agent 가 보유)에서 — feature 가 하드코딩하지 않는다.
- 주의: map_localizer 수정은 이 파일의 기존 재초기화 코드(생성자 INIT 처리)를 함수로 추출해 재사용 — 로직 복제 금지.

- [ ] **Step 1: 단위 테스트** — 격자 변환({alley:3,end:"south"} → (-3.5, -31.5, +π/2))과 BUSY 게이트를 FakeLocalizer 로.
- [ ] **Step 2~3: 구현** (실패 확인 후) — map_localizer 재초기화 함수 추출 + 토픽 구독, RosSensors.reinit 발행·폴링.
- [ ] **Step 4: 실기 게이트** — 정지 상태에서 est 를 의도적으로 틀기(reinit 으로 (真+2 m) 주입) → relocalize {alley,end 정답} → cmd_result completed + est-참값 오차 ≤0.3 m (참값은 /gz_ground_truth 대조, ①T7 스모크의 대조 코드 재사용).
- [ ] **Step 5: Commit** — `"relocalize — 운영자 복구를 명령으로 (reinit 토픽·격자 변환·BUSY 게이트)"`

### Task 4: blackbox_dump — 궤적 링 + npz

**Files:**
- Create: `ros2_ws/src/robomw/robomw/core/blackbox.py`
- Modify: `adapters/scout_diag.py` (blackbox_dump 구현 — core.blackbox 호출), `control_agent.py` (1 Hz 로 `blackbox.feed_pose(t,x,y,yaw)` — 기존 텔레메트리 타이머에 편승), maintenance feature (blackbox_dump 명령), supported 목록
- Test: `robomw/tests/test_blackbox.py` (신규)

**Interfaces:**
- Produces: `robomw.core.blackbox.Blackbox(maxlen_s=900)` — `feed_pose(t,x,y,yaw)`(1 Hz 가정 deque), `feed_event(dict)`(50건 링 — 기존 이벤트 링과 별도 사본), `dump(path, window_s) -> {"path","bytes","events","poses"}` (np.savez: events=json 문자열 배열, poses=float32 Nx4). window_s 는 min(window_s, 900).
- Produces: maintenance 처리 — `path = f"/tmp/blackbox_{robot_id}_{int(time.time())}.npz"`; 예외는 라우터가 INTERNAL 로 승격(기존 규약, 별도 try 불필요).

- [ ] **Step 1: 실패 테스트** (`test_blackbox.py`):
```python
import numpy as np
from robomw.core.blackbox import Blackbox


def test_dump_roundtrip(tmp_path):
    b = Blackbox()
    for i in range(100):
        b.feed_pose(1000.0 + i, float(i), 0.0, 0.0)
    b.feed_event({"kind": "estop", "t": 1050.0})
    out = b.dump(str(tmp_path / "bb.npz"), window_s=50)
    d = np.load(out["path"], allow_pickle=False)
    assert d["poses"].shape[1] == 4
    assert d["poses"][:, 0].min() >= 1000.0 + 100 - 50 - 1   # window 절단
    assert out["events"] == 1 and out["poses"] == len(d["poses"])


def test_window_capped_at_900():
    b = Blackbox()
    assert b.effective_window(5000) == 900
```
- [ ] **Step 2~3: 구현 → Step 4: 전체 pytest + 실기**(blackbox_dump 명령 → 파일 존재·크기·npz 로드 확인) **→ Step 5: Commit** — `"blackbox_dump — 궤적·이벤트 링 npz (900초 상한)"`

### Task 5: 서버 배선

**Files:**
- Modify: `/missions` REST 라우터 실파일(`grep -rn "missions" server/fleet_server/ --include=*.py` 로 확인) — 요청 스키마에 `work: dict|None` 추가·`alleys` 생략 허용(생략 시 로봇에 alleys 미포함 payload 전달), `server/fleet_server/ws.py` — `_ALWAYS_REVALIDATE` 에 `"relocalize"` 추가
- Test: `server/tests/` 기존 패턴에 2케이스 추가 (work 통과·alleys 생략)

- [ ] **Step 1: 실패 테스트** — 기존 missions 테스트 파일에: `POST /missions {robot_id, work:{type:"scout"}}` → 202/200 + 어댑터로 나간 payload 에 work 포함·alleys 부재 확인 (InMemoryFleetPort 주입 패턴 재사용 — server/tests 의 기존 미션 테스트를 먼저 읽을 것).
- [ ] **Step 2~3: 구현 → Step 4: server pytest 전건(112+2) → Step 5: Commit** — `"서버 — 정찰 work 통과·alleys 생략·relocalize 재검사"`

### Task 6: 대시보드 — 정찰 버튼·유지보수 카드·cmd_result 토스트

**Files:**
- Modify: `server/web/index.html`

**Interfaces (구현 형태):**
- 정찰 버튼: 임무 pane 상단 `"전체 정찰 시작"`(data-min-role="operator") → 인라인 슬라이더(0.1~1.0, step 0.1, 기본 1.0, 값 표시) + confirm → `api("POST","/missions",{robot_id:selKey, work:{type:"scout",params:{speed_scale:v}}})`.
- 유지보수 카드: 새 pane `id="pane-maint"` data-needs="maintenance" (hello.features 에 maintenance 가 로봇에서 오는지 확인 — registry.describe 가 feature 이름을 features 목록에 싣는 기존 흐름) —
  - 셀프테스트(operator): `sendCmd(selKey,"self_test",{})` + cmd_id → 결과 표(`<table>` 항목·✓/✗·detail)
  - 위치 재초기화(admin): 통로 select(0..8)·단 select(북/남) → confirm → `sendCmd(selKey,"relocalize",{alley,end})`
  - 블랙박스(operator): `sendCmd(selKey,"blackbox_dump",{})` → 경로·크기 표시
- cmd_result 토스트: `sendCmd` 가 `cmd_id:"d"+Date.now()` 를 payload 에 넣고 pending Map 에 등록. evt 채널 수신에서 `e.kind==="cmd_result" && pending.has(e.cmd_id)` → 토스트(성공: leaf 색 "명령 완료", 거부/실패: danger 색 `code — data.reason`), 5초 타임아웃 시 "응답 없음" 토스트. 유지보수 카드의 결과 표시는 같은 수신부에서 cmd 이름으로 분기.
- 기존 스타일 관례: CSS 변수(leaf/gold/danger), data-min-role 게이팅, api() 헬퍼, sendCmd 공통부 — 모두 재사용. 새 전역 스타일 추가는 토스트 1블록만.

- [ ] **Step 1: 구현** (UI 는 TDD 대신 브라우저 검증 — 이 저장소 관례)
- [ ] **Step 2: Playwright 검증** (server/.venv/bin/python, 스크래치 스크립트): 로그인 → 유지보수 카드 렌더 확인 → self_test 클릭 → 표에 5항목·all ✓ → relocalize(관리자) 실행 → 성공 토스트 → 정찰 버튼 → /missions 생성·상태 표시 → 즉시 취소 → 콘솔 오류 0. 스크린샷 2장(라이트) docs/figures/dashboard_maint_card.png·dashboard_scout_button.png 저장.
- [ ] **Step 3: Commit** — `"대시보드 — 전체 정찰 버튼·유지보수 카드·cmd_result 토스트"`

### Task 7: 최종 게이트 + findings

**Files:**
- Create: `docs/findings/2026-08-12-command-behaviors.md`
- Modify: `ros2_ws/src/robomw/README.md` (명령 표의 ② 항목 상태를 '구현됨'으로, work·maintenance 짧게)

- [ ] **Step 1: 스펙 §5 게이트 표 전 항목 실행** — 단위 pytest 전건 · self_test 실기(정상+라이다 검출) · relocalize 실기(2 m 복구) · **정찰 실기: `work:{type:"scout",params:{speed_scale:0.5}}` + alleys [0,1,2] 부분 정찰** — 속도 절반(구간 소요시간 ~2배)·state.work.progress 상승·완료 보고 확인 · spray UNSUPPORTED · 대시보드 Playwright · 회귀(하네스 남측 4쌍 P0 n=1 · scripts/21·30 · 49·50)
- [ ] **Step 2: findings 작성(수치 표) + README 갱신 → Step 3: Commit** — `"스펙 ② 완료 — 게이트 전 항목·문서"`
