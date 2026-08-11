# robomw 미들웨어 추출 + 명령 계약 v1 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 이기종 로봇이 같은 명령을 받아 같은 결과 보고를 내게 하는 로봇측 미들웨어 `robomw`를 현 스택에서 추출하고, scout_mini를 그 위에 재배선한다 (스펙 ①: docs/superpowers/specs/2026-08-11-robot-middleware-command-contract-design.md).

**Architecture:** 4층 — 공통 코어(링크·권한·안전·라우터·텔레메트리) / 사이트 프로파일(과수원 임무 엔진) / 능력군(hello 선언) / SDK(로봇별 어댑터). 기존 feature들은 Blackboard를 통해 이미 하드웨어와 분리되어 있으므로, SDK 어댑터가 Blackboard를 채우고 중재 출력이 Drive로 나가는 구조로 이관 리스크를 최소화한다.

**Tech Stack:** Python 3.12 · ament_python(colcon) · ROS2 Jazzy(어댑터만) · pytest · 기존 검증 스크립트(21·30·42·46) 재사용

## Global Constraints

- **하위 호환은 요구사항**: 봉투 `{v:1, topic, ts, seq, payload}`·명령 이름·권한표(observer<operator<admin, fail-closed)·안전 상수(HEARTBEAT_MS=1000, TELEOP_DEADMAN_MS=400, LINK_LOSS_STOP_MS=1500)·estop 2단계 의미론 불변. 기존 관제 서버·대시보드는 무수정으로 동작해야 한다.
- **robomw에 ROS import 0건**: `grep -rE "rclpy|rcl_interfaces" ros2_ws/src/robomw/robomw/` 이 빈 출력이어야 한다 (T1의 pytest가 이를 상시 검사).
- **안전은 코어 소관**: 속도는 기능→VelocityRequest→SafetyArbiter.arbitrate→Drive.set_velocity 단일 경로. 기능·프로파일 코드가 Drive를 직접 호출하면 리뷰 반려.
- **hello는 additive**: 기존 키 삭제·개명 금지, site/capabilities/middleware 키 추가만.
- **이관은 shim 동반**: `orchard_sim.link.protocol` 등 옛 import 경로는 robomw re-export로 유지 (기존 스크립트 21·30·39·42·46·47이 무수정 통과해야 한다).
- 커밋은 태스크마다 1회 이상, 메시지는 한국어 관례(기존 git log 참조).
- 빌드: `cd ros2_ws && colcon build --packages-select robomw orchard_sim` 후 `source ros2_ws/install/setup.bash`. 시뮬 재기동이 필요한 태스크는 스크래치 헬퍼 `restart_agent.sh`(에이전트만)·`relaunch_run.sh`(전체) 사용 — 경로는 `/tmp/claude-1000/-home-myhome-YBNML/691d883b-bd7f-499c-9b36-a59b0bd14a8a/scratchpad/`.
- pgrep 자기일치 함정: 프로세스 킬은 반드시 헬퍼 스크립트 파일 경유(같은 명령 문자열에 패턴이 실리면 exit 144로 자멸).

---

## 파일 구조 (최종 상태)

```
ros2_ws/src/robomw/
  package.xml, setup.py, setup.cfg, resource/robomw
  robomw/__init__.py
  robomw/link/{__init__.py, protocol.py, wsserver.py}     # T1·T2 이관 + T4 확장
  robomw/core/{__init__.py, base.py, safety.py, registry.py, audit.py, router.py}  # T2 이관 + T5 신규
  robomw/sdk/{__init__.py, types.py, interfaces.py}        # T3 신규
  robomw/profiles/__init__.py
  robomw/profiles/orchard/{__init__.py, mission.py}        # T7 이관
  robomw/features/{__init__.py, teleop.py, telemetry_state.py, telemetry_health.py, telemetry_map.py}  # T7 이관
  tests/{test_no_ros_imports.py, test_protocol_contract.py, test_sdk_types.py, test_router.py}
ros2_ws/src/orchard_sim/orchard_sim/
  link/{__init__.py, protocol.py, wsserver.py}             # shim (re-export)
  control/{safety.py, registry.py, audit.py, base.py}      # shim (re-export)
  control/features/*.py                                    # shim (re-export)
  adapters/{__init__.py, ros_drive.py, ros_sensors.py}     # T6 신규
  control_agent.py                                          # T6 슬림화
server/fleet_server/adapters/legacy_ws.py                   # T8 수정
```

주: 스펙의 core/host.py·telemetry.py는 실사 결과 조정한다 — 현 control_agent의
ROS 타이머·구독 구조가 이미 호스트 역할을 하므로, T6에서 control_agent를
"어댑터 조립 + robomw 부품 호출"로 슬림화하는 것이 host.py 신설보다 위험이
작다(별도 이벤트 루프 이식 불필요). telemetry_*는 feature 형태 그대로
robomw/features/로 이관한다. 이 조정은 스펙 §1의 의도(코어 부품의 robomw
소유·ROS 격리)를 유지한다.

---

### Task 1: robomw 패키지 스캐폴드 + protocol 이관 + ROS 격리 테스트

**Files:**
- Create: `ros2_ws/src/robomw/package.xml`, `setup.py`, `setup.cfg`, `resource/robomw`, `robomw/__init__.py`, `robomw/link/__init__.py`, `robomw/tests/test_no_ros_imports.py`
- Move: `ros2_ws/src/orchard_sim/orchard_sim/link/protocol.py` → `ros2_ws/src/robomw/robomw/link/protocol.py`
- Modify: `ros2_ws/src/orchard_sim/orchard_sim/link/protocol.py` (shim으로 재작성)

**Interfaces:**
- Produces: `robomw.link.protocol` — 기존 protocol.py의 모든 공개 이름 그대로 (envelope, parse, authorize, register_command_role, ROLE_*, CMD_*, HEARTBEAT_MS 등)
- Produces: shim `orchard_sim.link.protocol` = `from robomw.link.protocol import *` + `__getattr__` 위임

- [ ] **Step 1: 패키지 뼈대 작성**

`ros2_ws/src/robomw/package.xml`:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>robomw</name>
  <version>0.1.0</version>
  <description>로봇측 미들웨어 — 명령 계약·안전·링크 (ROS 비의존 코어)</description>
  <maintainer email="rla1231013@gmail.com">myhome</maintainer>
  <license>MIT</license>
  <test_depend>python3-pytest</test_depend>
  <export><build_type>ament_python</build_type></export>
</package>
```

`ros2_ws/src/robomw/setup.py`:
```python
from setuptools import find_packages, setup

setup(
    name="robomw",
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/robomw"]),
        ("share/robomw", ["package.xml"]),
    ],
    zip_safe=True,
    description="로봇측 미들웨어 — 명령 계약·안전·링크 (ROS 비의존 코어)",
    license="MIT",
)
```

`setup.cfg`는 orchard_sim 것을 복사해 이름만 robomw로. `resource/robomw`는 빈 파일. `robomw/__init__.py`·`robomw/link/__init__.py`는 빈 파일.

- [ ] **Step 2: 실패하는 ROS 격리 테스트 작성**

`ros2_ws/src/robomw/tests/test_no_ros_imports.py`:
```python
"""robomw 는 ROS 를 모른다 — 코어 격리 상시 검사 (스펙 §1)."""
import pathlib
import re

PKG = pathlib.Path(__file__).resolve().parent.parent / "robomw"
BANNED = re.compile(r"^\s*(import|from)\s+(rclpy|rcl_interfaces|std_msgs|geometry_msgs|sensor_msgs)")


def test_no_ros_imports():
    hits = []
    for p in PKG.rglob("*.py"):
        for i, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1):
            if BANNED.match(line):
                hits.append(f"{p}:{i}: {line.strip()}")
    assert not hits, "robomw 안에 ROS import:\n" + "\n".join(hits)


def test_protocol_importable_without_ros():
    import robomw.link.protocol as P
    assert P.PROTOCOL_VERSION == 1
```

- [ ] **Step 3: 실패 확인** — `cd ros2_ws/src/robomw && python3 -m pytest tests/ -v` → protocol 모듈 부재로 FAIL.

- [ ] **Step 4: protocol 이관 + shim**

```bash
git mv ros2_ws/src/orchard_sim/orchard_sim/link/protocol.py ros2_ws/src/robomw/robomw/link/protocol.py
```

새 `ros2_ws/src/orchard_sim/orchard_sim/link/protocol.py` (shim 전문):
```python
"""이관됨 → robomw.link.protocol (스펙 ①, 2026-08-11). 이 shim 은 기존
스크립트·서버 호환용 — 새 코드는 robomw 를 직접 import 할 것."""
from robomw.link.protocol import *              # noqa: F401,F403
from robomw.link.protocol import _ROLE_WARNINGS  # noqa: F401  (take_role_warnings 내부용)
import robomw.link.protocol as _p


def __getattr__(name):                          # 별표에 안 잡히는 밑줄 이름 위임
    return getattr(_p, name)
```
(protocol.py에 `_ROLE_WARNINGS`가 없으면 그 줄은 빼고, 실제 밑줄 전역이 있는지 `grep "^_" protocol.py`로 확인해 있는 것만 명시 재수출.)

- [ ] **Step 5: 빌드 + 테스트 통과 확인**

```bash
cd ros2_ws && colcon build --packages-select robomw orchard_sim && source install/setup.bash
python3 -m pytest src/robomw/tests/ -v          # 2건 PASS
python3 -c "import orchard_sim.link.protocol as P; import robomw.link.protocol as Q; assert P.authorize is Q.authorize"
```

- [ ] **Step 6: 기존 보안 검증 무수정 통과** — `bash -c 'source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash && python3 scripts/21_verify_security.py'` → 전 항목 통과 (이 스크립트가 옛 경로로 import하면 shim 검증을 겸한다).

- [ ] **Step 7: Commit** — `git add -A ros2_ws/src && git commit -m "robomw 스캐폴드 + protocol 이관 (shim 유지, ROS 격리 테스트)"`

### Task 2: wsserver·safety·registry·audit·base 이관

**Files:**
- Move: `orchard_sim/link/wsserver.py` → `robomw/link/wsserver.py`; `orchard_sim/control/{safety,registry,audit,base}.py` → `robomw/core/…`
- Create: `robomw/core/__init__.py`; 옛 경로 4+1개를 shim으로 재작성
- Test: 기존 `scripts/21_verify_security.py`, `scripts/30_verify_audit_roles.py`, `scripts/42_probe_robot_state.py`

**Interfaces:**
- Produces: `robomw.core.safety.SafetyArbiter`(arbitrate/snapshot/set_paused/note_client), `robomw.core.registry.Registry`(dispatch/describe), `robomw.core.base.{Blackboard, Context, VelocityRequest…}`, `robomw.core.audit`, `robomw.link.wsserver.ControlServer`
- 주의: 이관 파일의 상호 import를 `orchard_sim.…` → `robomw.…` 로 고친다. wsserver가 protocol을 상대 import(`from . import protocol`)하면 그대로 동작.

- [ ] **Step 1: git mv 5건 + `robomw/core/__init__.py` 생성**
- [ ] **Step 2: 이관 파일 내부 import 수정** — `grep -n "orchard_sim" ros2_ws/src/robomw/robomw/ -r` 이 0건이 될 때까지 `robomw.core.…`/`robomw.link.…`로 치환.
- [ ] **Step 3: shim 5개 작성** — Task 1 Step 4와 같은 형식(문서화 문자열 + `from robomw.<새경로> import *` + `__getattr__` 위임). base.py shim은 `from robomw.core.base import *` 외에 `from robomw.core.base import Blackboard, Context, VelocityRequest`를 명시(별표가 놓칠 수 있는 이름 안전핀).
- [ ] **Step 4: ROS 격리 확인** — wsserver·safety·registry·audit·base는 판독 결과 rclpy 무의존. `python3 -m pytest ros2_ws/src/robomw/tests/ -v` PASS (rclpy import가 튀어나오면 그 부분을 호출측 파라미터로 밀어낸다 — 예: 로거는 이미 주입식).
- [ ] **Step 5: 빌드 + 에이전트 기동 회귀**

```bash
cd ros2_ws && colcon build --packages-select robomw orchard_sim && source install/setup.bash
bash /tmp/claude-1000/-home-myhome-YBNML/691d883b-bd7f-499c-9b36-a59b0bd14a8a/scratchpad/restart_agent.sh
bash -c 'source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash && timeout 30 python3 scripts/42_probe_robot_state.py --secs 5'
```
기대: 에이전트 기동 완료, 모드 idle 표시. 이어서 `scripts/21_verify_security.py`·`scripts/30_verify_audit_roles.py` 전 항목 통과.

- [ ] **Step 6: Commit** — `"코어 부품 robomw 이관 (wsserver·safety·registry·audit·base, shim 유지)"`

### Task 3: SDK 자료형 + 인터페이스

**Files:**
- Create: `robomw/sdk/__init__.py`, `robomw/sdk/types.py`, `robomw/sdk/interfaces.py`
- Test: `robomw/tests/test_sdk_types.py`

**Interfaces (Produces — 이후 태스크가 그대로 사용):**
- `Pose(x: float, y: float, yaw: float, quality: float)` · `DriveLimits(v_max, w_max)` · `SelfTestItem(name, ok, detail)` · `WorkStatus(active, type, progress, detail)`
- ABC: `Drive.set_velocity(v,w)/stop()/limits()` · `Localizer.pose()/reinit(pose)/diagnostics()` · `Perception.clearance()/near_frac()` · `Work.start(type_,params)/stop()/status()` · `Diag.self_test(items)/blackbox_dump(window_s)`

- [ ] **Step 1: 실패 테스트 작성** (`robomw/tests/test_sdk_types.py`):
```python
import math
import pytest
from robomw.sdk.types import DriveLimits, Pose, SelfTestItem, WorkStatus
from robomw.sdk.interfaces import Diag, Drive, Localizer, Perception, Work


def test_pose_fields():
    p = Pose(1.0, -2.0, math.pi / 2, 0.8)
    assert (p.x, p.y) == (1.0, -2.0) and 0.0 <= p.quality <= 1.0


def test_interfaces_are_abstract():
    for cls in (Drive, Localizer, Perception, Work, Diag):
        with pytest.raises(TypeError):
            cls()


def test_minimal_drive_impl():
    class D(Drive):
        def __init__(self): self.last = None
        def set_velocity(self, v, w): self.last = (v, w)
        def stop(self): self.last = (0.0, 0.0)
        def limits(self): return DriveLimits(0.7, 1.0)
    d = D(); d.set_velocity(0.5, 0.1)
    assert d.last == (0.5, 0.1) and d.limits().v_max == 0.7
```
- [ ] **Step 2: 실패 확인** — ModuleNotFoundError.
- [ ] **Step 3: 구현** — `types.py`는 `@dataclass(frozen=True)` 4종(위 필드 그대로, Pose.quality 기본 1.0, WorkStatus 기본 active=False/type=""/progress=0.0/detail=""). `interfaces.py`는 스펙 §3의 5개 ABC를 `abc.ABC`+`@abstractmethod`로 그대로 옮긴다(시그니처 스펙 §3과 동일, docstring에 각 메서드의 단위·None 의미 명시: pose() None=미초기화, clearance() float("inf")=개활).
- [ ] **Step 4: PASS 확인 + Commit** — `"robomw SDK 자료형·인터페이스 5종"`

### Task 4: 명령 계약 확장 (protocol)

**Files:**
- Modify: `robomw/link/protocol.py`
- Test: `robomw/tests/test_protocol_contract.py`

**Interfaces (Produces):**
- `RESULT_CODES = ("OK","DENIED","BAD_PARAM","BUSY","ESTOPPED","UNSUPPORTED","TIMEOUT","INTERNAL")`
- `make_cmd_result(cmd_id, cmd, status, code="OK", data=None) -> dict` (event payload용, kind="cmd_result")
- `MISSION_REPORT_KEYS = ("alleys_done","distance_m","duration_s","interventions","coverage")`
- 신규 명령 상수: `CMD_SELF_TEST="self_test"`, `CMD_RELOCALIZE="relocalize"`, `CMD_BLACKBOX_DUMP="blackbox_dump"`, `CMD_WORK_STOP="work_stop"` + 권한표 등록(self_test·blackbox_dump·work_stop=operator, relocalize=admin)
- `WORK_TYPES = ("scout","spray","mow","transport")` · `validate_work(payload) -> (ok, reason)` — `{"type": str∈WORK_TYPES, "params": dict(선택, speed_scale는 0.1~1.0 float 선택)}` 검사
- `topic(site_id, robot_id, kind) -> str` — 기존 topic 조립부를 함수화(기본 site_id="orchard"). 기존 상수·호출은 유지(내부에서 이 함수 사용).
- hello v2 키 상수: `HELLO_SITE="site"`, `HELLO_CAPABILITIES="capabilities"`, `HELLO_MIDDLEWARE="middleware"` + `CAPABILITY_FAMILIES = ("drive","work","diag","legged","manipulation")` (뒤 2개는 예약 — 주석 명시)

- [ ] **Step 1: 실패 테스트 작성** (`test_protocol_contract.py`):
```python
import robomw.link.protocol as P


def test_new_commands_registered():
    for cmd, role in ((P.CMD_SELF_TEST, P.ROLE_OPERATOR), (P.CMD_RELOCALIZE, P.ROLE_ADMIN),
                      (P.CMD_BLACKBOX_DUMP, P.ROLE_OPERATOR), (P.CMD_WORK_STOP, P.ROLE_OPERATOR)):
        ok, _ = P.authorize(role, cmd)
        assert ok, cmd
    ok, _ = P.authorize(P.ROLE_OBSERVER, P.CMD_SELF_TEST)
    assert not ok


def test_cmd_result_shape():
    r = P.make_cmd_result("c1", "mission_start", "completed",
                          data={k: 0 for k in P.MISSION_REPORT_KEYS})
    assert r["kind"] == "cmd_result" and r["status"] == "completed" and r["code"] == "OK"
    assert set(P.MISSION_REPORT_KEYS) <= set(r["data"])


def test_cmd_result_rejects_bad_status():
    import pytest
    with pytest.raises(ValueError):
        P.make_cmd_result("c1", "ping", "definitely-not-a-status")


def test_validate_work():
    assert P.validate_work({"type": "scout"})[0]
    assert P.validate_work({"type": "spray", "params": {"speed_scale": 0.5}})[0]
    assert not P.validate_work({"type": "teleport"})[0]
    assert not P.validate_work({"type": "mow", "params": {"speed_scale": 3.0}})[0]


def test_topic_site_generalized():
    assert P.topic("orchard", "scout01", "cmd") == "orchard/scout01/cmd"
    assert P.topic("factory7", "biped01", "event") == "factory7/biped01/event"
```
- [ ] **Step 2: 실패 확인** → AttributeError.
- [ ] **Step 3: 구현** — 위 Produces 명세 그대로 protocol.py에 추가. `make_cmd_result`는 status∉(accepted,rejected,in_progress,completed,failed) 또는 code∉RESULT_CODES면 ValueError. 신규 명령은 기존 `ROLE_REQUIRED` 표에 직접 추가(모듈 로드 시 등록 — register_command_role의 "기존 덮어쓰기 불가" 규약과 충돌 없게 표에 넣는다). 기존 topic 문자열 상수/조립부가 하드코딩 "orchard"라면 `topic()` 함수를 만들고 기존 자리를 함수 호출로 치환하되 기본값으로 결과 불변 확인.
- [ ] **Step 4: PASS + 기존 21·30 재통과 확인 + Commit** — `"명령 계약 v1 확장 — cmd_result·신규 명령 4종·work 스키마·site topic"`

### Task 5: 명령 라우터 (cmd_id 멱등 + 결과 발행 + UNSUPPORTED)

**Files:**
- Create: `robomw/core/router.py`
- Test: `robomw/tests/test_router.py`

**Interfaces:**
- Consumes: `robomw.link.protocol`(authorize, make_cmd_result, CMD_*), `robomw.core.registry.Registry.dispatch(cmd, payload) -> bool`
- Produces: `CommandRouter(registry, emit_event: Callable[[dict], None], supported: Callable[[str], bool])` — 메서드 `handle(cmd: str, payload: dict, role: str) -> dict|None`(발행한 cmd_result 반환, cmd_id 없으면 None). 멱등 캐시 OrderedDict 최근 32건.

- [ ] **Step 1: 실패 테스트 작성** (`test_router.py`) — 핵심 4행위:
```python
from robomw.core.router import CommandRouter
import robomw.link.protocol as P


class FakeReg:
    def __init__(self): self.calls = []
    def dispatch(self, cmd, payload): self.calls.append(cmd); return True


def mk(supported=lambda c: True):
    events = []
    reg = FakeReg()
    r = CommandRouter(reg, events.append, supported)
    return r, reg, events


def test_accepted_result_and_dispatch():
    r, reg, events = mk()
    res = r.handle("mission_pause", {"cmd_id": "c1"}, P.ROLE_OPERATOR)
    assert res["status"] == "accepted" and reg.calls == ["mission_pause"]
    assert events and events[-1]["kind"] == "cmd_result"


def test_denied_no_dispatch():
    r, reg, events = mk()
    res = r.handle("set_mode", {"cmd_id": "c2"}, P.ROLE_OBSERVER)
    assert res["status"] == "rejected" and res["code"] == "DENIED" and not reg.calls


def test_idempotent_replay():
    r, reg, events = mk()
    a = r.handle("mission_pause", {"cmd_id": "c3"}, P.ROLE_OPERATOR)
    b = r.handle("mission_pause", {"cmd_id": "c3"}, P.ROLE_OPERATOR)
    assert reg.calls == ["mission_pause"]          # 재실행 없음
    assert b == a                                   # 직전 결과 재발행


def test_unsupported():
    r, reg, events = mk(supported=lambda c: c != P.CMD_SELF_TEST)
    res = r.handle(P.CMD_SELF_TEST, {"cmd_id": "c4"}, P.ROLE_OPERATOR)
    assert res["status"] == "rejected" and res["code"] == "UNSUPPORTED" and not reg.calls


def test_no_cmd_id_legacy_path():
    r, reg, events = mk()
    assert r.handle("mission_pause", {}, P.ROLE_OPERATOR) is None
    assert reg.calls == ["mission_pause"] and not events
```
- [ ] **Step 2: 실패 확인.**
- [ ] **Step 3: 구현** (`router.py` 전문 골격):
```python
"""명령 라우터 — 권한 재판정·cmd_id 멱등·cmd_result 발행 (스펙 §2.2).

입구(wsserver 콜백)의 1차 판정과 별개로 소비 시점에 2차 판정한다(기존
control_agent 이중 판정 규약 유지). cmd_id 가 있으면 결과를 이벤트로
발행하고 최근 32건을 캐시해 재수신 시 재실행 없이 재발행한다.
"""
from collections import OrderedDict

from robomw.link import protocol as P


class CommandRouter:
    def __init__(self, registry, emit_event, supported):
        self._reg = registry
        self._emit = emit_event
        self._supported = supported
        self._cache = OrderedDict()          # cmd_id -> cmd_result dict

    def _result(self, cmd_id, cmd, status, code, data=None):
        res = P.make_cmd_result(cmd_id, cmd, status, code, data)
        self._cache[cmd_id] = res
        while len(self._cache) > 32:
            self._cache.popitem(last=False)
        self._emit(res)
        return res

    def handle(self, cmd, payload, role):
        cmd_id = payload.get("cmd_id")
        if cmd_id and cmd_id in self._cache:
            res = self._cache[cmd_id]
            self._emit(res)
            return res
        ok, reason = P.authorize(role, cmd)
        if not ok:
            return self._result(cmd_id, cmd, "rejected", "DENIED",
                                {"reason": reason}) if cmd_id else None
        if not self._supported(cmd):
            return self._result(cmd_id, cmd, "rejected", "UNSUPPORTED") if cmd_id else None
        handled = self._reg.dispatch(cmd, payload)
        if cmd_id:
            if handled:
                return self._result(cmd_id, cmd, "accepted", "OK")
            return self._result(cmd_id, cmd, "rejected", "BAD_PARAM",
                                {"reason": "처리 기능 없음"})
        return None
```
- [ ] **Step 4: PASS + Commit** — `"명령 라우터 — 멱등 cmd_id·cmd_result·UNSUPPORTED (unit 5종)"`

### Task 6: scout 재배선 1 — ROS 어댑터 + control_agent 가 robomw 부품 사용

**Files:**
- Create: `orchard_sim/adapters/__init__.py`, `ros_drive.py`, `ros_sensors.py`
- Modify: `orchard_sim/control_agent.py` (심볼 기준: `_handle_cmd` 앞뒤의 명령 소비 경로, hello 조립부, `/cmd_vel` 발행부)

**Interfaces:**
- Consumes: Task 3 SDK ABC·types, Task 5 `CommandRouter`, Task 4 hello 키·`topic()`
- Produces: `RosDrive(node, topic="/cmd_vel")` — Drive 구현(내부 publisher); `RosSensors(node)` — Localizer+Perception 구현(기존 TF·진단·점군 콜백 로직을 이 클래스로 이동, control_agent 는 콜백에서 `sensors.feed_*` 호출)
- Produces: control_agent 에 `self.router = CommandRouter(self.registry, self._emit_cmd_result, self._cmd_supported)` — `_handle_cmd`의 registry.dispatch 직접 호출을 router.handle 로 교체(estop 계열·ping 등 코어 직접 처리 명령은 라우터 앞단 기존 경로 유지하되, cmd_id 가 있으면 처리 후 `P.make_cmd_result` 발행)
- Produces: hello v2 — 기존 hello dict 에 `site={type:"orchard", geometry:<기존 기하>}`, `capabilities={drive:{v_max,w_max}, }`, `middleware={name:"robomw",version:"0.1"}` 추가 (기존 키 불변)
- `_cmd_supported(cmd)`: 신규 4종(self_test·relocalize·blackbox_dump·work_stop)은 False → UNSUPPORTED (동작 구현은 스펙 ②)

- [ ] **Step 1: 어댑터 작성** — `ros_drive.py`:
```python
"""Drive SDK 구현 — /cmd_vel 발행. 중재(arbitrate) 결과만 이 클래스로 온다."""
from geometry_msgs.msg import Twist

from robomw.sdk.interfaces import Drive
from robomw.sdk.types import DriveLimits


class RosDrive(Drive):
    def __init__(self, node, v_max, w_max, topic="/cmd_vel"):
        self._pub = node.create_publisher(Twist, topic, 10)
        self._lim = DriveLimits(v_max, w_max)

    def set_velocity(self, v, w):
        t = Twist()
        t.linear.x = float(max(-self._lim.v_max, min(self._lim.v_max, v)))
        t.angular.z = float(max(-self._lim.w_max, min(self._lim.w_max, w)))
        self._pub.publish(t)

    def stop(self):
        self.set_velocity(0.0, 0.0)

    def limits(self):
        return self._lim
```
`ros_sensors.py`는 Localizer·Perception 을 한 클래스로: control_agent 의 기존 `_on_cloud` 원뿔 계산·`_on_loc_diag` 파싱·TF/pose 추적 코드를 옮겨 담고, `pose()/diagnostics()/clearance()/near_frac()` 로 노출. control_agent 콜백은 `self.sensors.feed_cloud(msg)` 식 위임 한 줄로.
- [ ] **Step 2: control_agent 재배선** — `/cmd_vel` 발행부를 `self.drive.set_velocity(...)` 로, bb의 clearance/near_frac 갱신을 `self.sensors` 경유로, `_handle_cmd` 를 위 Produces 대로 교체, hello 조립에 v2 키 추가.
- [ ] **Step 2b: 어댑터 예외 격리 (스펙 §5)** — registry.dispatch 호출을 라우터가 try/except 로 감싸고(Task 5 router.py 의 `handled = self._reg.dispatch(...)` 부분), 예외 시 `cmd_result failed(INTERNAL)` 발행 + `emit_event` 로 `{"kind":"assistance","msg":"명령 처리 내부 오류: <cmd>","level":"critical","code":"CMD_INTERNAL"}` 승격, 프로세스는 계속 산다. test_router.py 에 케이스 추가:
```python
def test_dispatch_exception_becomes_internal():
    class BoomReg:
        def dispatch(self, cmd, payload): raise RuntimeError("boom")
    events = []
    r = CommandRouter(BoomReg(), events.append, lambda c: True)
    res = r.handle("mission_pause", {"cmd_id": "c9"}, P.ROLE_OPERATOR)
    assert res["status"] == "failed" and res["code"] == "INTERNAL"
    assert any(e.get("kind") == "assistance" for e in events)
```
- [ ] **Step 3: 빌드 + 기동 + 계약 스모크** — 재빌드 후 `restart_agent.sh`. 스모크 스크립트(스크래치에 임시 작성, resume2.py 변형): ws 접속 → hello 수신에 `site.type=="orchard"`·`middleware.name=="robomw"` 확인 → `{"cmd":"ping","cmd_id":"t1"}` 송신 → `cmd_result{status:"accepted"|"completed"}` 수신 확인 → `{"cmd":"self_test","cmd_id":"t2"}` → `cmd_result{code:"UNSUPPORTED"}` 확인.
- [ ] **Step 4: 회귀** — `scripts/21`·`scripts/30` 통과, `scripts/42_probe_robot_state.py` 정상, teleop 데드맨 동작(42 출력의 게이트 표시) 확인.
- [ ] **Step 5: Commit** — `"scout 재배선 1 — SDK 어댑터(RosDrive·RosSensors)·라우터·hello v2"`

### Task 7: scout 재배선 2 — 임무 엔진 프로파일 이관 + 완료 보고

**Files:**
- Move: `orchard_sim/control/features/drive_mission.py` → `robomw/profiles/orchard/mission.py`; `drive_teleop.py` → `robomw/features/teleop.py`; `telemetry_{state,health,map}.py` → `robomw/features/`
- Modify: `robomw/profiles/orchard/mission.py` (완료 보고 추가), control_agent 의 feature 적재 경로
- 옛 경로는 shim (re-export)

**Interfaces:**
- Consumes: `robomw.core.base.Context`(ctx.bb, ctx.event, ctx.safety), Task 4 `MISSION_REPORT_KEYS`·`validate_work`
- Produces: 임무 완료 시 `ctx.event("cmd_result", ..., **P.make_cmd_result(...))` 대신 **코어 일관 경로**: mission 이 `ctx.emit_cmd_result(cmd_id, "mission_start", "completed", data=report)` 를 부른다 — Context 에 `emit_cmd_result` 콜백(주입: control_agent 가 라우터의 `_result` 를 연결) 추가
- 완료 보고 데이터 수집(신규 코드, mission.py 내):
```python
# 임무 시작 시: self._report = dict(alleys_done=[], distance_m=0.0,
#     t0=time.time(), interventions=0)
# tick 마다: bb pose 로 dd 적분(직전 pose 와의 hypot, 0.5 m 초과 점프는 무시 — 텔레포트/재초기화 보호)
# alley 완주(traverse wp 완료) 시: alleys_done.append(k)
# assistance 이벤트 훅: control_agent 가 ctx.bb.extra["mission_interventions"] 증가 → 보고에 반영
# 완료 시: coverage = len(alleys_done)/len(alleys); duration_s = time.time()-t0
```
- `mission_start` payload 의 `work` 필드: 있으면 `validate_work` 로 검사, 불합격이면 rejected(BAD_PARAM). 합격이면 `bb.extra["work"]=payload["work"]` 저장만 (실행은 스펙 ②).
- **측위 미준비 거부 (스펙 §5)**: mission_start 처리 시 bb 의 pose 가 None(측위 미초기화)이면 `emit_cmd_result(cmd_id, "mission_start", "rejected", code="BUSY", data={"reason":"측위 미준비"})` 후 임무를 만들지 않는다 — 현재의 암묵 동작을 명문화. T7 Step 4 실주행 게이트에 케이스 추가: 로컬라이저를 내린 상태에서 mission_start → BUSY 거부 수신 확인 후 로컬라이저 기동.

- [ ] **Step 1: git mv + import 수정 + shim 작성** (T2 와 동일 형식)
- [ ] **Step 2: 완료 보고·work 검증 코드 추가** (위 명세 그대로)
- [ ] **Step 3: 단위 확인** — `python3 -c "from robomw.profiles.orchard.mission import *"` + pytest 전건 재통과
- [ ] **Step 4: 실주행 회귀 게이트 (합격 기준)**
  - `scripts/46_climb_harness.py --mission-pairs --policies P0 --n 2` → 16/16 (에이전트 내리고 실행 — run_harness.sh 헬퍼)
  - 3통로 미션: relaunch 헬퍼로 alleys [0,1,2] + cmd_id 부여 → 무개입 완료 + `cmd_result{status:"completed", data:{alleys_done:[0,1,2], coverage:1.0, ...}}` 수신 확인 (수신 스크립트는 Step 3 스모크 확장)
- [ ] **Step 5: Commit** — `"scout 재배선 2 — 과수원 프로파일 이관·임무 완료 보고·work 스키마 수용"`

### Task 8: 관제 서버 프로토콜 단일화

**Files:**
- Modify: `server/fleet_server/adapters/legacy_ws.py` (봉투 조립·topic 문자열 → `robomw.link.protocol` 함수)
- Modify: `server/fleet_server/ws.py` `_WS_ACTIONS` — 신규 명령 4종 추가 + `server/fleet_server/auth.py` ROLE_REQUIRED 에 동일 역할 등록 (self_test·blackbox_dump·work_stop=operator, relocalize=admin)

**Interfaces:**
- Consumes: `robomw.link.protocol.{envelope 조립 함수, topic()}` — legacy_ws 가 손으로 만들던 `{v,topic,ts_ns,seq,payload}` 를 protocol 함수로 대체(필드 결과 동일해야 함)
- 설치: `server/.venv/bin/pip install -e ros2_ws/src/robomw`

- [ ] **Step 1: pip -e 설치 + import 스모크** — `server/.venv/bin/python -c "import robomw.link.protocol as P; print(P.PROTOCOL_VERSION)"`
- [ ] **Step 2: legacy_ws 교체** — 조립 지점을 찾아(`grep -n "topic" server/fleet_server/adapters/legacy_ws.py`) protocol 함수 호출로 치환. 치환 전후 송신 JSON 동일성: 임시 pytest 로 구 조립 함수와 신 함수 결과 dict 비교.
- [ ] **Step 3: 서버 액션 확장** — `_WS_ACTIONS` 와 서버측 ROLE_REQUIRED 에 신규 4종 (스펙 §2.4 역할 그대로). 대시보드 UI 는 스펙 ② 범위 — 서버는 통로만 뚫는다.
- [ ] **Step 4: 통합 스모크** — `restart8000.sh` → 대시보드 Playwright 스모크(server/.venv/bin/python, 기존 shot 스크립트 참조): 로그인(admin/123) → 텔레메트리 수신 → estop → 2단계 해제 카드 → 콘솔 오류 0, denied 0. 로봇 online 상태에서 ws 로 `{action:"self_test", cmd_id}` 송신 시 서버가 로봇으로 중계하고 `cmd_result UNSUPPORTED` 가 evt 채널로 돌아오는 것 확인.
- [ ] **Step 5: Commit** — `"관제 서버 프로토콜 단일화 — robomw.link import·신규 명령 중계"`

### Task 9: 최종 회귀 일괄 + 문서

**Files:**
- Create: `ros2_ws/src/robomw/README.md` (계약 요약 — 스펙 §2 표 + SDK 5종 시그니처 + "새 로봇 온보딩 = SDK 5종 구현 + hello 선언" 한 쪽)
- Modify: `docs/findings/` 신규 노트 (재배선 결과 수치)

- [ ] **Step 1: 게이트 전 항목 실행** (스펙 §4 표 그대로)
  - pytest: `python3 -m pytest ros2_ws/src/robomw/tests/ -v` 전건
  - 보안: scripts/21·30 전 항목
  - 하네스: 46 --mission-pairs --policies P0,P9 --n 2 → 32/32
  - 3통로 무개입 + cmd_result completed (T7 게이트 재실행)
  - 대시보드 스모크 (T8 게이트 재실행)
  - ROS 격리: pytest 의 test_no_ros_imports 가 대변
- [ ] **Step 2: README + findings 작성** — 게이트 결과 수치 포함
- [ ] **Step 3: Commit** — `"robomw v0.1 — 회귀 게이트 전 항목 통과·온보딩 문서"`
