"""FleetService — 텔레메트리 허브(최신값 캐시·DB 수집·구독자 팬아웃·임무 동기화)."""
import pytest

from fleet_server import missions
from fleet_server import models as m
from fleet_server.db import Base, make_engine, make_session_factory
from fleet_server.fleet.port import InMemoryFleetPort
from fleet_server.fleet.service import FleetService


@pytest.fixture()
def factory():
    engine = make_engine("sqlite://")           # in-memory, 이 fixture 전용 엔진
    Base.metadata.create_all(engine)
    return make_session_factory(engine)


def _seed_mission(factory, robot_id: str = "scout01") -> int:
    with factory() as db:
        farm = m.Farm(name="농장A"); db.add(farm); db.flush()
        db.add(m.Robot(id=robot_id, farm_id=farm.id, name="r")); db.flush()
        db.add(m.User(login="op", pw_hash="h", role="operator")); db.flush()
        ms = missions.create(db, robot_id=robot_id, farm_id=farm.id,
                             spec={"alleys": [0]}, created_by=1)
        return ms.id


def _seed_robot(factory, robot_id: str = "scout01") -> None:
    """실제로는 텔레메트리가 오는 로봇은 항상 robots 테이블에 먼저 등록돼 있다
    (register_robot 이전에 admin API 가 Robot 행을 만든다) — FK 강제(foreign_keys=ON)
    아래에서 tracks/events 가 존재하지 않는 robot_id 를 참조하지 않도록 맞춘다."""
    with factory() as db:
        farm = m.Farm(name="농장A"); db.add(farm); db.flush()
        db.add(m.Robot(id=robot_id, farm_id=farm.id, name="r"))
        db.commit()


def test_tel_state_feed_creates_track(factory):
    _seed_robot(factory)
    fp = InMemoryFleetPort()
    svc = FleetService(factory)
    svc.attach(fp)
    fp.feed("scout01", "tel/state",
           {"x": 1.0, "y": 2.0, "yaw": 0.1, "mode": "auto", "ts": 1000.0})
    with factory() as db:
        rows = db.query(m.Track).filter_by(robot_id="scout01").all()
    assert len(rows) == 1 and rows[0].x == 1.0
    assert svc.latest["scout01"]["tel/state"]["x"] == 1.0


def test_evt_feed_dedups_by_seq(factory):
    _seed_robot(factory)
    fp = InMemoryFleetPort()
    svc = FleetService(factory)
    svc.attach(fp)
    fp.feed("scout01", "evt", {"kind": "estop"}, seq=1)
    fp.feed("scout01", "evt", {"kind": "estop"}, seq=1)     # 중복 seq → 무해화
    with factory() as db:
        rows = db.query(m.Event).filter_by(robot_id="scout01").all()
    assert len(rows) == 1


def test_mission_sync_running_then_done(factory):
    ms_id = _seed_mission(factory)
    fp = InMemoryFleetPort()
    svc = FleetService(factory)
    svc.attach(fp)
    fp.feed("scout01", "mission", {"state": "running"})
    with factory() as db:
        assert db.get(m.Mission, ms_id).state == "RUNNING"
    fp.feed("scout01", "mission", {"state": "done"})
    with factory() as db:
        assert db.get(m.Mission, ms_id).state == "DONE"


def test_mission_sync_invalid_transition_swallowed(factory):
    ms_id = _seed_mission(factory)               # QUEUED, 아직 start 안 함
    fp = InMemoryFleetPort()
    svc = FleetService(factory)
    svc.attach(fp)
    fp.feed("scout01", "mission", {"state": "done"})   # QUEUED→complete: 불가 전이
    with factory() as db:
        assert db.get(m.Mission, ms_id).state == "QUEUED"    # 전이 안 됨, 예외 없이 통과


def test_mission_sync_no_active_mission_ignored(factory):
    fp = InMemoryFleetPort()
    svc = FleetService(factory)
    svc.attach(fp)
    fp.feed("ghost", "mission", {"state": "done"})     # 임무 자체가 없음 — 조용히 무시
    assert svc.latest["ghost"]["mission"] == {"state": "done"}    # 캐시는 갱신됨


# ── 로봇 evt → 임무 상태기계 (실기 수명주기) ────────────────────────────────
#
# 아래 payload 는 전부 실기에서 실제로 받은 것을 그대로 옮긴 것이다
# (server/fleet.db events 테이블, 2026-08-13 scout01/scout02):
#   {"kind":"mission_started","msg":"임무 시작 — 통로 [0] · 웨이포인트 1개",...}
#   {"kind":"cmd_result","cmd_id":"m21","cmd":"mission_start","status":"accepted",
#    "code":"OK","data":{}}
#   {"kind":"cmd_result","cmd_id":"m9","cmd":"mission_start","status":"completed",
#    "code":"OK","data":{"alleys_done":[7,6,5],"distance_m":244.7,
#                        "duration_s":3963.8,"interventions":0,"coverage":1.0}}
#   {"kind":"mission_done","msg":"임무 완료",...} / {"kind":"mission_cancelled",...}

def _svc(factory):
    fp = InMemoryFleetPort()
    svc = FleetService(factory)
    svc.attach(fp)
    return fp, svc


def _accepted(mission_id: int) -> dict:
    return {"kind": "cmd_result", "cmd_id": f"m{mission_id}", "cmd": "mission_start",
            "status": "accepted", "code": "OK", "data": {},
            "msg": "mission_start → accepted", "level": "info"}


_REPORT = {"alleys_done": [0], "distance_m": 60.2, "duration_s": 700.5,
           "interventions": 0, "coverage": 1.0}


def _completed(mission_id: int, data=None) -> dict:
    return {"kind": "cmd_result", "cmd_id": f"m{mission_id}", "cmd": "mission_start",
            "status": "completed", "code": "OK", "data": data if data is not None else _REPORT,
            "msg": "mission_start → completed", "level": "info"}


def _state(factory, ms_id):
    with factory() as db:
        return db.get(m.Mission, ms_id).state


def test_evt_cmd_result_accepted_starts_mission(factory):
    """로봇이 mission_start 를 수락하면 서버 임무도 RUNNING 이어야 한다 —
    cmd_id("m{id}")로 정확히 상관된다. started_at 도 이때 찍힌다."""
    ms_id = _seed_mission(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", _accepted(ms_id), seq=1)
    with factory() as db:
        ms = db.get(m.Mission, ms_id)
        assert ms.state == "RUNNING"
        assert ms.started_at is not None


def test_evt_mission_started_kind_starts_mission(factory):
    """cmd_result 를 못 받는 경우(상관 키 없는 옛 경로)에도 evt kind 로 이어진다."""
    ms_id = _seed_mission(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", {"kind": "mission_started", "msg": "임무 시작 — 통로 [0]"}, seq=1)
    assert _state(factory, ms_id) == "RUNNING"


def test_evt_completed_finishes_mission_and_records_report(factory):
    """완료 보고(cmd_result completed)의 data 는 임무 이벤트로 남겨야 한다 —
    주행거리·소요·개입수·커버리지가 운행 기록의 알맹이다."""
    ms_id = _seed_mission(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", _accepted(ms_id), seq=1)
    fp.feed("scout01", "evt", _completed(ms_id), seq=2)
    assert _state(factory, ms_id) == "DONE"
    with factory() as db:
        payloads = [e.payload_json for e in db.query(m.MissionEvent)
                    .filter_by(mission_id=ms_id).all()]
    assert any(p.get("distance_m") == 60.2 and p.get("coverage") == 1.0
               for p in payloads)


def test_evt_completed_catches_up_a_lost_accept(factory):
    """실기 확인(임무 23) — 수락 신호가 수집에서 유실되면 서버 임무는 QUEUED 인데
    완료 보고만 도착한다. 그러면 종착 전이가 불가라 임무는 QUEUED 로 굳고 통로
    잠금을 영원히 쥔다. cmd_id 로 정확히 상관된 완료는 그 임무의 것이 확실하니
    잃어버린 start 를 따라잡은 뒤 종착시킨다(둘 다 관문 경유)."""
    from fleet_server.traffic import AlleyLocks
    ms_id = _seed_mission(factory)
    with factory() as db:
        ms = db.get(m.Mission, ms_id)
        AlleyLocks.acquire(db, ms.robot_id, ms.id, [0], ms.farm_id)
        db.commit()
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", _completed(ms_id), seq=1)      # accepted 없이 완료만
    with factory() as db:
        ms = db.get(m.Mission, ms_id)
        assert ms.state == "DONE"
        assert ms.started_at is not None and ms.ended_at is not None
        assert AlleyLocks.list_active(db) == []


def test_evt_done_kind_does_not_catch_up_queued_mission(factory):
    """따라잡기는 **상관 키가 있는** 경로만의 권한이다. kind 경로는 어느 임무의
    완료인지 확신할 수 없으므로 QUEUED 임무를 종착시키지 않는다."""
    ms_id = _seed_mission(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", {"kind": "mission_done", "msg": "임무 완료"}, seq=1)
    assert _state(factory, ms_id) == "QUEUED"


def test_evt_mission_done_kind_finishes_mission(factory):
    ms_id = _seed_mission(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", {"kind": "mission_started"}, seq=1)
    fp.feed("scout01", "evt", {"kind": "mission_done", "msg": "임무 완료"}, seq=2)
    assert _state(factory, ms_id) == "DONE"


def test_evt_done_then_completed_still_records_report(factory):
    """실기 순서는 evt(mission_done) → cmd_result(completed) 다(1 ms 차). 먼저 온
    evt 가 임무를 종착시켜도 뒤따르는 보고를 버리면 안 된다."""
    ms_id = _seed_mission(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", _accepted(ms_id), seq=1)
    fp.feed("scout01", "evt", {"kind": "mission_done", "msg": "임무 완료"}, seq=2)
    fp.feed("scout01", "evt", _completed(ms_id), seq=3)
    assert _state(factory, ms_id) == "DONE"
    with factory() as db:
        assert any(e.payload_json.get("distance_m") == 60.2
                   for e in db.query(m.MissionEvent).filter_by(mission_id=ms_id))


def test_evt_mission_cancelled_kind_is_not_mapped(factory):
    """I-3 — mission_cancelled 는 kind 경로에서 뺐다.

    취소는 언제나 서버가 먼저 아는 사건이다(REST/BT 가 apply_verb 로 전이를
    커밋한 뒤에 명령이 나간다). 그러니 이 kind 로 얻을 것은 없고, 상관 키가
    없어 지연 도착 시 **직후에 발진한 다음 임무**를 취소해 버릴 수 있다
    (QUEUED 에서 cancel 은 합법 전이다) — 잠금까지 함께 풀린다."""
    ms_id = _seed_mission(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", _accepted(ms_id), seq=1)
    fp.feed("scout01", "evt", {"kind": "mission_cancelled", "msg": "임무 취소"}, seq=2)
    assert _state(factory, ms_id) == "RUNNING"          # 상태를 건드리지 않는다


def test_delayed_cancel_evt_does_not_kill_the_next_mission(factory):
    """I-3 회귀 — 앞 임무를 취소하고 곧바로 새 임무를 발진한 뒤, 뒤늦게 도착한
    mission_cancelled 가 새 임무를 죽이면 안 된다."""
    from fleet_server import missions
    first = _seed_mission(factory)
    with factory() as db:
        missions.apply(db, db.get(m.Mission, first), "cancel")
        second = missions.create(db, robot_id="scout01", farm_id=1,
                                 spec={"alleys": [1]}, created_by=1).id
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", {"kind": "mission_cancelled", "msg": "임무 취소"}, seq=9)
    assert _state(factory, second) == "QUEUED"          # 방금 발진한 임무는 무사하다


def test_evt_lifecycle_releases_alley_lock(factory):
    """완료가 서버 상태기계를 통과해야 통로 잠금이 풀린다 — BT 의 '선행 종료 시
    자동 발진'이 이 해제에 걸려 있다."""
    from fleet_server.traffic import AlleyLocks
    ms_id = _seed_mission(factory)
    with factory() as db:
        ms = db.get(m.Mission, ms_id)
        AlleyLocks.acquire(db, ms.robot_id, ms.id, [0], ms.farm_id)
        db.commit()
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", _accepted(ms_id), seq=1)
    fp.feed("scout01", "evt", _completed(ms_id), seq=2)
    with factory() as db:
        assert AlleyLocks.list_active(db) == []


def test_evt_duplicate_signals_are_idempotent(factory):
    """중복 evt(재전송·evt+cmd_result 이중 신호)로 상태가 흔들리면 안 된다."""
    ms_id = _seed_mission(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", {"kind": "mission_started"}, seq=1)
    fp.feed("scout01", "evt", _accepted(ms_id), seq=2)          # 같은 뜻 — 이미 RUNNING
    assert _state(factory, ms_id) == "RUNNING"
    fp.feed("scout01", "evt", _completed(ms_id), seq=3)
    fp.feed("scout01", "evt", _completed(ms_id), seq=4)         # 완료 재전송
    fp.feed("scout01", "evt", {"kind": "mission_done"}, seq=5)
    assert _state(factory, ms_id) == "DONE"
    with factory() as db:
        reports = [e for e in db.query(m.MissionEvent).filter_by(mission_id=ms_id)
                   if e.payload_json.get("distance_m") is not None]
    assert len(reports) == 1                       # 보고는 한 번만 적힌다


def test_evt_from_other_robot_does_not_touch_mission(factory):
    """다른 로봇의 evt 는 이 임무를 건드리지 않는다(휴리스틱 오귀속 방지)."""
    ms_id = _seed_mission(factory, "scout01")
    with factory() as db:
        db.add(m.Robot(id="scout99", farm_id=1, name="r2"))
        db.commit()
    fp, _ = _svc(factory)
    fp.feed("scout99", "evt", {"kind": "mission_started"}, seq=1)
    fp.feed("scout99", "evt", _accepted(ms_id), seq=2)     # cmd_id 는 scout01 의 임무
    assert _state(factory, ms_id) == "QUEUED"             # 남의 로봇 보고 — 무시


def test_evt_rejected_still_fails_mission(factory):
    """T4 I4 와의 공존 — 거부는 여전히 FAILED 로 종착시킨다."""
    ms_id = _seed_mission(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", {"kind": "cmd_result", "cmd_id": f"m{ms_id}",
                               "cmd": "mission_start", "status": "rejected",
                               "code": "BUSY", "data": {"reason": "임무 진행 중"}}, seq=1)
    assert _state(factory, ms_id) == "FAILED"


def test_evt_unknown_kinds_and_other_commands_ignored(factory):
    ms_id = _seed_mission(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", {"kind": "pong", "msg": "pong"}, seq=1)
    fp.feed("scout01", "evt", {"kind": "cmd_result", "cmd_id": "x1",
                               "cmd": "self_test", "status": "completed",
                               "data": {"items": []}}, seq=2)
    fp.feed("scout01", "evt", {"kind": "work_stopped", "msg": "작업 중단"}, seq=3)
    assert _state(factory, ms_id) == "QUEUED"


# ── Task 6 — 이벤트 보존정책: pong 미기록 ────────────────────────────────────

def test_evt_pong_not_persisted_but_other_kinds_are(factory):
    """pong 은 링크 판정에만 쓰인다(어댑터 메모리로 충분하다는 것이 스펙 판단) —
    events 테이블에 적을 가치가 없다(실기: 81,503건 중 81,174건이 pong). 다른
    kind 는 그대로 기록된다."""
    _seed_robot(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", {"kind": "pong", "msg": "pong"}, seq=1)
    fp.feed("scout01", "evt", {"kind": "estop", "msg": "정지"}, seq=2)
    with factory() as db:
        rows = db.query(m.Event).filter_by(robot_id="scout01").all()
    assert [r.kind for r in rows] == ["estop"]


def test_subscribe_receives_and_unsub_stops(factory):
    fp = InMemoryFleetPort()
    svc = FleetService(factory)
    svc.attach(fp)
    got = []
    unsub = svc.subscribe(lambda robot_id, channel, payload: got.append(
        (robot_id, channel, payload)))
    fp.feed("scout01", "tel/health", {"batt": 90})
    assert got == [("scout01", "tel/health", {"batt": 90})]
    unsub()
    fp.feed("scout01", "tel/health", {"batt": 80})
    assert len(got) == 1                          # unsub 후 미호출
