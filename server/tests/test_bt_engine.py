"""BT 엔진·프리셋·API — InMemoryFleetPort 주입 관례(conftest 의 app 픽스처).

엔진 테스트는 TestClient 없이 세션 팩토리를 직접 쓴다(틱을 결정적으로 몰기
위해). API 테스트만 TestClient 로 인가·스코프를 확인한다.
"""
import asyncio
import time

import pytest

from fleet_server import models as m
from fleet_server.bt import presets
from fleet_server.bt.engine import BTEngine
from tests.conftest import do_login


# ── 공용 시딩 ────────────────────────────────────────────────────────────────

def _seed(app, *robots, farm_name="농장A", online=True) -> int:
    with app.state.session_factory() as db:
        farm = m.Farm(name=farm_name)
        db.add(farm)
        db.flush()
        for rid in robots:
            db.add(m.Robot(id=rid, farm_id=farm.id, name=rid))
        db.commit()
        fid = farm.id
    if online:
        for rid in robots:
            app.state.fleet.feed(rid, "tel/state", {})       # 온라인 전환
    return fid


def _uid(app) -> int:
    with app.state.session_factory() as db:
        return db.query(m.User).filter_by(login="admin").one().id


def _mission(app, robot) -> m.Mission | None:
    with app.state.session_factory() as db:
        return (db.query(m.Mission).filter(m.Mission.robot_id == robot)
                .order_by(m.Mission.id.desc()).first())


def _inst(app, iid) -> m.BTInstance:
    with app.state.session_factory() as db:
        return db.get(m.BTInstance, iid)


_SEQ = [1000]


def _evt(app, robot, payload):
    """실기와 같은 경로로 로봇 이벤트를 먹인다(evt 채널 + 증가하는 seq)."""
    _SEQ[0] += 1
    app.state.fleet.feed(robot, "evt", payload, seq=_SEQ[0])


def _robot_accepts(app, robot, mission_id):
    """실기 원문: {"kind":"cmd_result","cmd_id":"m21","cmd":"mission_start",
    "status":"accepted","code":"OK","data":{}}"""
    _evt(app, robot, {"kind": "cmd_result", "cmd_id": f"m{mission_id}",
                      "cmd": "mission_start", "status": "accepted", "code": "OK",
                      "data": {}, "msg": "mission_start → accepted"})


def _robot_completes(app, robot, mission_id):
    """실기 원문: evt(mission_done) 뒤에 cmd_result(completed, 완주 보고)."""
    _evt(app, robot, {"kind": "mission_done", "msg": "임무 완료"})
    _evt(app, robot, {"kind": "cmd_result", "cmd_id": f"m{mission_id}",
                      "cmd": "mission_start", "status": "completed", "code": "OK",
                      "data": {"alleys_done": [0], "distance_m": 66.0,
                               "duration_s": 700.0, "interventions": 0,
                               "coverage": 1.0}})


_TS = [time.time() + 100.0]          # ingest.track 은 1 Hz 다운샘플 — ts 를 벌려 준다


def _tel(app, robot, mode):
    """로봇 상태 텔레메트리 1건(감시가 보는 mode 의 출처는 tracks 테이블이다)."""
    _TS[0] += 2.0
    app.state.fleet.feed(robot, "tel/state",
                         {"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": mode, "ts": _TS[0]})


async def _rest_mission(app, robot_id, alleys):
    """REST 와 같은 공용 경로로 임무를 발진시킨다(BT 밖에서 통로를 선점)."""
    from fleet_server import mission_ops
    with app.state.session_factory() as db:
        robot = db.get(m.Robot, robot_id)
        ms, reason = await mission_ops.create_and_dispatch(
            db, app.state.fleet, robot=robot, alleys=alleys, work=None,
            created_by=_uid(app))
        assert reason is None
        return ms.id


# ── 엔진 수명주기 ────────────────────────────────────────────────────────────

async def test_engine_dispatches_action_mission_and_completes(app):
    """I-4 — 완료는 **실기가 실제로 보내는 것**(evt cmd_result·mission_done)으로만
    몬다. 죽은 채널("mission")로 몰면 evt 소비 함수를 통째로 지워도 이 테스트가
    통과해 버려서, 실기 수명주기의 회귀를 못 잡는다."""
    _seed(app, "scout01")
    eng = app.state.bt_engine
    ids = eng.create("single_alley_loop", {"robot": "scout01", "alley": 0, "n": 1},
                     created_by=_uid(app))
    assert len(ids) == 1

    await eng.tick_once()
    ms_id = _mission(app, "scout01").id
    assert _mission(app, "scout01").state == "QUEUED"
    robot_id, _, action, payload = app.state.fleet.sent[-1]
    assert (robot_id, action, payload["alleys"]) == ("scout01", "mission_start", [0])
    assert _inst(app, ids[0]).state == "RUNNING"

    _robot_accepts(app, "scout01", ms_id)             # 로봇 수락 → RUNNING
    ms = _mission(app, "scout01")
    assert ms.state == "RUNNING" and ms.started_at is not None
    await eng.tick_once()
    assert _inst(app, ids[0]).state == "RUNNING"

    _robot_completes(app, "scout01", ms_id)           # 완주 보고 → DONE
    assert _mission(app, "scout01").state == "DONE"
    with app.state.session_factory() as db:
        from fleet_server.traffic import AlleyLocks
        assert AlleyLocks.list_active(db) == []       # 종착이 잠금을 풀었다
    await eng.tick_once()
    assert _inst(app, ids[0]).state == "SUCCESS"       # Action 성공 = 임무 완료


async def test_instance_fails_when_mission_start_rejected(app):
    """이중시작(로봇에 활성 임무 있음)은 create 409 → Action 실패 → Retry 소진."""
    _seed(app, "scout01")
    await _rest_mission(app, "scout01", [0, 1])
    eng = app.state.bt_engine
    ids = eng.create("single_alley_loop", {"robot": "scout01", "alley": 4, "n": 1},
                     created_by=_uid(app))
    await eng.tick_once()
    assert _inst(app, ids[0]).state == "FAILED"


async def test_queued_lock_waits_then_auto_launches_when_lock_frees(app):
    """T4 이관 (b)(c) — 겹치는 임무는 QUEUED_LOCK 으로 남고, 선행 임무가 끝나
    잠금이 풀리면 엔진이 스스로 승격(재획득 후 발진)한다."""
    _seed(app, "scout01", "scout03")
    ms1 = await _rest_mission(app, "scout01", [0, 1])
    _robot_accepts(app, "scout01", ms1)                # 로봇 수락(실기 payload)
    eng = app.state.bt_engine
    ids = eng.create("single_alley_loop", {"robot": "scout03", "alley": 1, "n": 1},
                     created_by=_uid(app))

    await eng.tick_once()
    assert _mission(app, "scout03").state == "QUEUED_LOCK"
    sent_before = len(app.state.fleet.sent)
    await eng.tick_once()                              # 아직 잠겨 있다 — 승격 실패
    assert len(app.state.fleet.sent) == sent_before
    assert _inst(app, ids[0]).state == "RUNNING"       # 실패가 아니라 대기

    _robot_completes(app, "scout01", ms1)              # 선행 종료(실기 payload)
    await eng.tick_once()
    ms2 = _mission(app, "scout03")
    assert ms2.state == "QUEUED"                       # 승격됨
    assert app.state.fleet.sent[-1][0] == "scout03"
    assert app.state.fleet.sent[-1][2] == "mission_start"
    with app.state.session_factory() as db:
        from fleet_server.traffic import AlleyLocks
        assert [r["mission_id"] for r in AlleyLocks.list_active(db)] == [ms2.id]
        assert db.get(m.Mission, ms1).state == "DONE"       # 선행은 종착, 잠금 해제됨


async def test_engine_restart_resumes_running_instance(app):
    """재기동 복원 — 엔진을 새로 만들어도 RUNNING 인스턴스를 이어간다
    (Action 은 mission_id 재부착·상태 재판정, 임무를 다시 만들지 않는다)."""
    _seed(app, "scout01")
    ids = app.state.bt_engine.create(
        "single_alley_loop", {"robot": "scout01", "alley": 0, "n": 1},
        created_by=_uid(app))
    await app.state.bt_engine.tick_once()
    first = _mission(app, "scout01").id

    _robot_accepts(app, "scout01", first)

    fresh = BTEngine(app.state.session_factory, app.state.fleet)   # 서버 재기동
    assert fresh.restore() == 1
    await fresh.tick_once()
    assert _mission(app, "scout01").id == first        # 임무를 새로 만들지 않았다

    # I-4 — 재판정도 실기 경로로: 서버가 죽은 사이 끝난 임무를 이어받아 확정한다
    _robot_completes(app, "scout01", first)
    await fresh.tick_once()
    assert _inst(app, ids[0]).state == "SUCCESS"


async def test_cancel_instance_cancels_inflight_mission_and_releases_lock(app):
    _seed(app, "scout01")
    eng = app.state.bt_engine
    ids = eng.create("single_alley_loop", {"robot": "scout01", "alley": 0, "n": 2},
                     created_by=_uid(app))
    await eng.tick_once()
    assert _mission(app, "scout01").state == "QUEUED"

    await eng.cancel(ids[0])
    assert _inst(app, ids[0]).state == "CANCELED"
    assert _mission(app, "scout01").state == "CANCELED"
    with app.state.session_factory() as db:
        from fleet_server.traffic import AlleyLocks
        assert AlleyLocks.list_active(db) == []
    await eng.tick_once()                              # 취소된 인스턴스는 더 틱하지 않는다
    assert _inst(app, ids[0]).state == "CANCELED"


async def test_cancel_cleans_queued_lock_mission(app):
    """경합으로 생긴 QUEUED_LOCK 도 엔진 취소로 회수된다(로봇당 활성 임무 해방)."""
    _seed(app, "scout01", "scout03")
    await _rest_mission(app, "scout01", [0, 1])
    eng = app.state.bt_engine
    ids = eng.create("single_alley_loop", {"robot": "scout03", "alley": 1, "n": 1},
                     created_by=_uid(app))
    await eng.tick_once()
    assert _mission(app, "scout03").state == "QUEUED_LOCK"
    await eng.cancel(ids[0])
    assert _mission(app, "scout03").state == "CANCELED"


async def test_split_patrol_waits_while_robot_offline(app):
    """Condition 은 불충족이면 running(대기) — 오프라인 로봇에 임무를 만들지 않는다."""
    _seed(app, "scout01", "scout02", online=False)
    eng = app.state.bt_engine
    ids = eng.create("full_split_patrol", {"robot_a": "scout01", "robot_b": "scout02"},
                     created_by=_uid(app))
    assert len(ids) == 2
    await eng.tick_once()
    assert _mission(app, "scout01") is None
    assert all(_inst(app, i).state == "RUNNING" for i in ids)

    app.state.fleet.feed("scout01", "tel/state", {})
    app.state.fleet.feed("scout02", "tel/state", {})
    await eng.tick_once()
    assert _mission(app, "scout01").spec_json["alleys"] == [0, 1, 2, 3, 4]
    assert _mission(app, "scout02").spec_json["alleys"] == [6, 7, 8]


async def test_split_patrol_two_instances_do_not_lock_each_other(app):
    """분담 분할은 통로 5 버퍼로 잠금이 겹치지 않는다 — 둘 다 발진한다."""
    _seed(app, "scout01", "scout02")
    eng = app.state.bt_engine
    eng.create("full_split_patrol", {"robot_a": "scout01", "robot_b": "scout02"},
               created_by=_uid(app))
    await eng.tick_once()
    assert _mission(app, "scout01").state == "QUEUED"
    assert _mission(app, "scout02").state == "QUEUED"


async def test_tick_loop_runs_at_configured_period(app):
    """1 Hz 틱 태스크 — start/stop 으로 수명주기를 관리한다(lifespan 통합용)."""
    _seed(app, "scout01")
    eng = BTEngine(app.state.session_factory, app.state.fleet, period_s=0.02)
    eng.create("single_alley_loop", {"robot": "scout01", "alley": 0, "n": 1},
               created_by=_uid(app))
    await eng.start()
    for _ in range(50):
        await asyncio.sleep(0.02)
        if _mission(app, "scout01") is not None:
            break
    await eng.stop()
    assert _mission(app, "scout01") is not None
    assert eng._task is None


# ── 틱 예외 처리 (I-2) ──────────────────────────────────────────────────────

def _break_status_once(monkeypatch):
    """다음 mission_status 호출 한 번만 터뜨린다(일시적 DB·링크 사고 흉내)."""
    from fleet_server.bt.engine import EngineCtx
    original, state = EngineCtx.mission_status, {"fired": False}

    async def flaky(self, mission_id):
        if not state["fired"]:
            state["fired"] = True
            raise RuntimeError("일시적 사고")
        return await original(self, mission_id)

    monkeypatch.setattr(EngineCtx, "mission_status", flaky)
    return state


async def test_tick_exception_keeps_instance_running_and_preserves_mission_id(app, monkeypatch):
    """I-2 — 틱 예외는 '이번 틱 실패'지 인스턴스 종착이 아니다. 종착시키면 이미
    발진한 임무가 QUEUED 로 잠금을 쥔 채 남고, BT cancel 은 그 인스턴스가
    RUNNING 이 아니라 아무것도 회수하지 못한다. tree_json(=mission_id)도 예외
    경로에서 저장돼야 다음 틱이 같은 임무를 이어받는다."""
    _seed(app, "scout01")
    eng = app.state.bt_engine
    ids = eng.create("single_alley_loop", {"robot": "scout01", "alley": 0, "n": 1},
                     created_by=_uid(app))
    await eng.tick_once()                              # 발진
    ms_id = _mission(app, "scout01").id

    _break_status_once(monkeypatch)
    await eng.tick_once()                              # 예외 발생 틱
    inst = _inst(app, ids[0])
    assert inst.state == "RUNNING"                     # 종착시키지 않는다
    assert inst.tree_json["child"]["mission_id"] == ms_id   # 진행 상태 보존
    assert inst.note                                   # 사유는 남긴다

    _robot_accepts(app, "scout01", ms_id)
    _robot_completes(app, "scout01", ms_id)
    await eng.tick_once()                              # 다음 틱은 정상 진행
    assert _inst(app, ids[0]).state == "SUCCESS"
    assert _mission(app, "scout01").id == ms_id        # 임무를 새로 만들지 않았다


async def test_repeated_tick_exceptions_finally_fail_instance_keeping_tree(app, monkeypatch):
    """폭주 방지 — 연속 실패가 한계를 넘으면 종착시키되 tree_json 은 보존한다
    (사람이 어디서 멈췄는지 보고 임무를 회수할 수 있어야 한다)."""
    from fleet_server.bt.engine import EngineCtx
    _seed(app, "scout01")
    eng = BTEngine(app.state.session_factory, app.state.fleet, max_tick_errors=2)
    ids = eng.create("single_alley_loop", {"robot": "scout01", "alley": 0, "n": 1},
                     created_by=_uid(app))
    await eng.tick_once()
    ms_id = _mission(app, "scout01").id

    async def always_broken(self, mission_id):
        raise RuntimeError("계속 터진다")

    monkeypatch.setattr(EngineCtx, "mission_status", always_broken)
    await eng.tick_once()
    assert _inst(app, ids[0]).state == "RUNNING"       # 1회째 — 재시도
    await eng.tick_once()
    inst = _inst(app, ids[0])
    assert inst.state == "FAILED"                      # 2회째 — 종착
    assert inst.tree_json["child"]["mission_id"] == ms_id


# ── 좀비 임무 감시 (I-5) ────────────────────────────────────────────────────

async def test_watchdog_fails_running_mission_when_robot_reports_idle(app):
    """로봇이 임무 도중 재기동하면 종착 신호가 영영 오지 않는다 — 임무는 RUNNING,
    통로 잠금은 잡힌 채, Action 은 영원히 running 이다. 서버가 이미 받는
    텔레메트리(mode)로 그 괴리를 K틱 관찰한 뒤 관문 경유로 FAILED 시킨다."""
    from fleet_server.traffic import AlleyLocks
    _seed(app, "scout01")
    eng = BTEngine(app.state.session_factory, app.state.fleet, stall_ticks=3)
    ids = eng.create("single_alley_loop", {"robot": "scout01", "alley": 0, "n": 1},
                     created_by=_uid(app))
    await eng.tick_once()
    ms_id = _mission(app, "scout01").id
    _robot_accepts(app, "scout01", ms_id)
    assert _mission(app, "scout01").state == "RUNNING"

    for i in range(3):                                 # 로봇은 임무를 잊었다(idle)
        _tel(app, "scout01", "idle")
        await eng.tick_once()
    ms = _mission(app, "scout01")
    assert ms.state == "FAILED"
    with app.state.session_factory() as db:
        assert AlleyLocks.list_active(db) == []        # 관문 경유 — 잠금 해제됨
    await eng.tick_once()
    assert _inst(app, ids[0]).state == "FAILED"        # Action 실패 → 인스턴스 종착


async def test_watchdog_does_not_fire_while_robot_runs_the_mission(app):
    _seed(app, "scout01")
    eng = BTEngine(app.state.session_factory, app.state.fleet, stall_ticks=2)
    eng.create("single_alley_loop", {"robot": "scout01", "alley": 0, "n": 1},
               created_by=_uid(app))
    await eng.tick_once()
    ms_id = _mission(app, "scout01").id
    _robot_accepts(app, "scout01", ms_id)
    for i in range(6):
        _tel(app, "scout01", "mission")
        await eng.tick_once()
    assert _mission(app, "scout01").state == "RUNNING"


async def test_watchdog_counter_resets_on_recovery(app):
    """한 틱 idle 이 스쳐도(수락 직전 등) 누적이 리셋돼야 오귀속이 없다."""
    _seed(app, "scout01")
    eng = BTEngine(app.state.session_factory, app.state.fleet, stall_ticks=3)
    eng.create("single_alley_loop", {"robot": "scout01", "alley": 0, "n": 1},
               created_by=_uid(app))
    await eng.tick_once()
    ms_id = _mission(app, "scout01").id
    _robot_accepts(app, "scout01", ms_id)
    for mode in ("idle", "idle", "mission", "idle", "idle"):
        _tel(app, "scout01", mode)
        await eng.tick_once()
    assert _mission(app, "scout01").state == "RUNNING"   # 연속 3틱을 못 채웠다


async def _running_mission_then_offline(app, eng):
    eng.create("single_alley_loop", {"robot": "scout01", "alley": 0, "n": 1},
               created_by=_uid(app))
    await eng.tick_once()
    ms_id = _mission(app, "scout01").id
    _robot_accepts(app, "scout01", ms_id)
    app.state.fleet.presence.touch("scout01", t=0.0)     # 링크 단절(강제 오프라인)
    return ms_id


def _interventions(app, code=None):
    from fleet_server.models import Intervention
    with app.state.session_factory() as db:
        q = db.query(Intervention)
        if code:
            q = q.filter(Intervention.code == code)
        return q.all()


async def test_watchdog_offline_does_not_fail_mission_and_notifies_once(app):
    """라운드 2 — **오프라인은 증거의 부재지 임무 포기의 증거가 아니다.**

    로봇의 링크 단절 정책은 임무를 버리지 않는다(robomw safety.py:240-243 —
    paused 만 세우고, 해제는 명시적 resume 뿐). 그런데 서버가 45초쯤 뒤 임무를
    실패시켜 AlleyLock 을 풀면, 로봇은 통로 안에 임무를 쥔 채 서 있는데 서버는
    그 통로를 다른 로봇에게 내준다 — 링크 복구 후 사람이 resume 하면 서버가
    비었다고 믿는 통로를 달린다. T4 C3 가 일부러 사람에게 맡긴 판단이다.
    그래서 자동 종착 대신 개입 알림 1회를 남기고 계속 기다린다."""
    from fleet_server.traffic import AlleyLocks
    _seed(app, "scout01")
    eng = BTEngine(app.state.session_factory, app.state.fleet, stall_ticks=2)
    ms_id = await _running_mission_then_offline(app, eng)

    for _ in range(8):
        await eng.tick_once()

    assert _mission(app, "scout01").state == "RUNNING"    # 종착시키지 않는다
    with app.state.session_factory() as db:
        assert [r["mission_id"] for r in AlleyLocks.list_active(db)] == [ms_id]
    rows = _interventions(app, "LINK_LOST_POLICY")
    assert len(rows) == 1                                # 에피소드당 1회
    assert rows[0].context_json.get("mission_id") == ms_id
    assert (rows[0].context_json or {}).get("repeat", 0) == 0   # 틱마다 재발행 없음


async def test_operator_cancel_recovers_offline_stalled_mission(app):
    """회수는 사람의 몫 — C3 오프라인 cancel 이 로컬 전이로 임무를 끝내고 잠금을 푼다."""
    from fleet_server import mission_ops
    from fleet_server.traffic import AlleyLocks
    _seed(app, "scout01")
    eng = BTEngine(app.state.session_factory, app.state.fleet, stall_ticks=2)
    ms_id = await _running_mission_then_offline(app, eng)
    for _ in range(4):
        await eng.tick_once()

    with app.state.session_factory() as db:
        ms = db.get(m.Mission, ms_id)
        delivery = await mission_ops.apply_verb(db, app.state.fleet, ms, "cancel")
        assert delivery == "not_sent"
        assert ms.state == "CANCELED"
        assert AlleyLocks.list_active(db) == []


async def test_watchdog_fails_queued_mission_when_robot_is_idle(app):
    """(a) 수락 직후 로봇 재기동·수락 신호 유실이면 QUEUED 로 굳어 어떤 경로도
    풀지 못한다. QUEUED 는 정의상 이미 발진된 상태이므로, 같은 적극적 증거
    (온라인 ∧ mode=idle)를 더 긴 임계로 적용해 종착시킨다."""
    from fleet_server.traffic import AlleyLocks
    _seed(app, "scout01")
    eng = BTEngine(app.state.session_factory, app.state.fleet,
                   stall_ticks=2, queued_stall_ticks=4)
    eng.create("single_alley_loop", {"robot": "scout01", "alley": 0, "n": 1},
               created_by=_uid(app))
    await eng.tick_once()
    ms_id = _mission(app, "scout01").id
    assert _mission(app, "scout01").state == "QUEUED"     # 수락 신호가 오지 않았다

    for _ in range(3):
        _tel(app, "scout01", "idle")
        await eng.tick_once()
    assert _mission(app, "scout01").state == "QUEUED"     # 아직 임계 미달
    _tel(app, "scout01", "idle")
    await eng.tick_once()
    assert _mission(app, "scout01").state == "FAILED"
    with app.state.session_factory() as db:
        assert AlleyLocks.list_active(db) == []
    assert _mission(app, "scout01").id == ms_id


async def test_watchdog_holds_judgement_when_track_is_stale(app):
    """(b) tel/state 만 멈추고 링크가 살아 있으면, 임무 이전의 idle 행이 계속
    '최신'으로 읽혀 주행 중인 임무를 죽인다. 낡은 증거는 증거가 아니다."""
    _seed(app, "scout01")
    eng = BTEngine(app.state.session_factory, app.state.fleet,
                   stall_ticks=2, track_fresh_s=20.0)
    eng.create("single_alley_loop", {"robot": "scout01", "alley": 0, "n": 1},
               created_by=_uid(app))
    await eng.tick_once()
    ms_id = _mission(app, "scout01").id
    app.state.fleet.feed("scout01", "tel/state",         # 10분 묵은 idle 보고
                         {"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": "idle",
                          "ts": time.time() - 600})
    _robot_accepts(app, "scout01", ms_id)
    for _ in range(6):
        app.state.fleet.presence.touch("scout01")        # 링크는 살아 있다
        await eng.tick_once()
    assert _mission(app, "scout01").state == "RUNNING"    # 판단 보류


async def test_watchdog_ignores_queued_mission_not_yet_accepted(app):
    """발진 직후 로봇이 아직 idle 인 구간(QUEUED)은 감시 대상이 아니다."""
    _seed(app, "scout01")
    eng = BTEngine(app.state.session_factory, app.state.fleet, stall_ticks=2)
    eng.create("single_alley_loop", {"robot": "scout01", "alley": 0, "n": 1},
               created_by=_uid(app))
    await eng.tick_once()
    for i in range(5):
        _tel(app, "scout01", "idle")
        await eng.tick_once()
    assert _mission(app, "scout01").state == "QUEUED"    # 수락 대기 중 — 건드리지 않는다


# ── 프리셋·파리티 ────────────────────────────────────────────────────────────

def _specs(state) -> list[dict]:
    if state["kind"] == "action":
        return [state["spec"]]
    kids = state.get("children") or ([state["child"]] if state.get("child") else [])
    return [s for k in kids for s in _specs(k)]


def test_full_split_patrol_default_is_parity_safe_split_5():
    """T3 실측 — 선회 평지 패드가 통로 파리티에 고정돼 있어 기본 분할은
    split_k=5 (A=[0..4] / B=[6,7,8]) 다. split_k=4 의 B=[5..8] 은 첫 통로가
    홀수라 두 횡단이 모두 램프행이다."""
    plans = presets.full_split_patrol("scout01", "scout02")
    assert [p.robot_id for p in plans] == ["scout01", "scout02"]
    assert [_specs(p.tree.to_state())[0]["alleys"] for p in plans] == \
        [[0, 1, 2, 3, 4], [6, 7, 8]]


def test_full_split_patrol_rejects_parity_violating_split():
    with pytest.raises(presets.PresetError):
        presets.full_split_patrol("scout01", "scout02", split_k=4)   # B=[5,6,7,8]


def test_parity_rule():
    assert presets.parity_safe([0, 1, 2, 3, 4])       # 오름차순 + 첫 통로 짝수
    assert presets.parity_safe([6, 7, 8])
    assert not presets.parity_safe([5, 6, 7])         # 오름차순인데 첫 통로 홀수
    assert presets.parity_safe([7, 6, 5])             # T3 게이트가 통과한 역순
    assert not presets.parity_safe([0, 2, 4])         # 비인접 — 판정 불가
    assert presets.parity_safe([3])                   # 횡단 없음


def test_preset_trees_have_condition_gate_before_action():
    plan = presets.full_split_patrol("scout01", "scout02")[0]
    st = plan.tree.to_state()
    assert [c["kind"] for c in st["children"]] == \
        ["condition", "condition", "condition", "action"]
    assert [c["cond"] for c in st["children"][:3]] == \
        ["robot_online", "robot_idle", "alley_free"]


def test_sequential_retry_shape():
    plan = presets.sequential_retry("scout01", [0, 1, 2])[0]
    st = plan.tree.to_state()
    assert st["kind"] == "sequence"
    assert st["children"][0]["kind"] == "retry"
    assert st["children"][0]["n"] == 2
    assert _specs(st)[0] == {"robot": "scout01", "alleys": [0, 1, 2]}


def test_sequential_retry_rejects_parity_violating_alleys():
    with pytest.raises(presets.PresetError):
        presets.sequential_retry("scout01", [5, 6, 7])


def test_single_alley_loop_shape():
    plan = presets.single_alley_loop("scout01", 3, 2)[0]
    st = plan.tree.to_state()
    assert st["kind"] == "retry" and st["n"] == 2
    assert _specs(st)[0] == {"robot": "scout01", "alleys": [3]}


def test_unknown_preset_rejected():
    with pytest.raises(presets.PresetError):
        presets.build("teleport_everywhere", {})


# ── API ──────────────────────────────────────────────────────────────────────

def _seed_api(client):
    """admin API 로 농장·로봇·operator 를 만든다(test_mission_api 관례)."""
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    fb = client.post("/api/v1/farms", json={"name": "농장B"}, headers=h).json()
    for rid, fid in (("scout01", fa["id"]), ("scout02", fa["id"]), ("scout09", fb["id"])):
        client.post("/api/v1/robots", headers=h,
                    json={"id": rid, "farm_id": fid, "name": rid})
    client.post("/api/v1/users", headers=h, json={
        "login": "op", "password": "oppw", "role": "operator", "farm_ids": [fa["id"]]})
    return fa, fb


def test_bt_api_create_list_cancel(client, app):
    _seed_api(client)
    app.state.fleet.feed("scout01", "tel/state", {})
    csrf = do_login(client, "op", "oppw")
    h = {"X-CSRF": csrf}
    r = client.post("/api/v1/bt", headers=h, json={
        "preset": "single_alley_loop", "params": {"robot": "scout01", "alley": 0, "n": 1}})
    assert r.status_code == 200, r.text
    ids = r.json()["ids"]
    assert len(ids) == 1

    asyncio.run(app.state.bt_engine.tick_once())       # 한 틱 — 임무 발진

    rows = client.get("/api/v1/bt", headers=h).json()
    assert len(rows) == 1
    row = rows[0]
    assert row["preset"] == "single_alley_loop"
    assert row["robot_id"] == "scout01"
    assert row["state"] == "RUNNING"
    assert row["node_states"]["kind"] == "retry"       # 트리(JSON) 가 실려 나온다
    assert row["node_states"]["child"]["mission_id"] is not None

    c = client.post(f"/api/v1/bt/{ids[0]}/cancel", headers=h)
    assert c.status_code == 200, c.text
    assert c.json()["state"] == "CANCELED"
    assert client.get("/api/v1/missions", headers=h).json()[0]["state"] == "CANCELED"


def test_bt_api_split_preset_creates_two_instances(client, app):
    _seed_api(client)
    csrf = do_login(client, "op", "oppw")
    r = client.post("/api/v1/bt", headers={"X-CSRF": csrf}, json={
        "preset": "full_split_patrol", "params": {"robot_a": "scout01", "robot_b": "scout02"}})
    assert r.status_code == 200, r.text
    assert len(r.json()["ids"]) == 2


def test_bt_api_rejects_parity_violating_split_400(client):
    _seed_api(client)
    csrf = do_login(client, "op", "oppw")
    r = client.post("/api/v1/bt", headers={"X-CSRF": csrf}, json={
        "preset": "full_split_patrol",
        "params": {"robot_a": "scout01", "robot_b": "scout02", "split_k": 4}})
    assert r.status_code == 400, r.text


def test_bt_api_rejects_unknown_preset_and_bad_params_400(client):
    _seed_api(client)
    csrf = do_login(client, "op", "oppw")
    h = {"X-CSRF": csrf}
    assert client.post("/api/v1/bt", headers=h,
                       json={"preset": "nope", "params": {}}).status_code == 400
    assert client.post("/api/v1/bt", headers=h, json={
        "preset": "single_alley_loop", "params": {"robot": "scout01"}}).status_code == 400


def test_bt_api_cross_farm_and_observer_forbidden(client, app):
    fa, _ = _seed_api(client)
    csrf0 = do_login(client)
    client.post("/api/v1/users", headers={"X-CSRF": csrf0}, json={
        "login": "obs", "password": "obspw", "role": "observer", "farm_ids": [fa["id"]]})
    csrf = do_login(client, "op", "oppw")              # op 는 농장A 만
    r = client.post("/api/v1/bt", headers={"X-CSRF": csrf}, json={
        "preset": "single_alley_loop", "params": {"robot": "scout09", "alley": 0, "n": 1}})
    assert r.status_code == 403, r.text

    csrf_obs = do_login(client, "obs", "obspw")
    r2 = client.post("/api/v1/bt", headers={"X-CSRF": csrf_obs}, json={
        "preset": "single_alley_loop", "params": {"robot": "scout01", "alley": 0, "n": 1}})
    assert r2.status_code == 403, r2.text


def test_bt_api_list_is_farm_scoped(client, app):
    _seed_api(client)
    csrf0 = do_login(client)
    r = client.post("/api/v1/bt", headers={"X-CSRF": csrf0}, json={
        "preset": "single_alley_loop", "params": {"robot": "scout09", "alley": 0, "n": 1}})
    assert r.status_code == 200, r.text                # admin — 농장B 로봇
    csrf = do_login(client, "op", "oppw")
    assert client.get("/api/v1/bt", headers={"X-CSRF": csrf}).json() == []


def test_bt_api_unknown_instance_404(client):
    _seed_api(client)
    csrf = do_login(client, "op", "oppw")
    assert client.post("/api/v1/bt/9999/cancel",
                       headers={"X-CSRF": csrf}).status_code == 404
