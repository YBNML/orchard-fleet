import pytest

from fleet_server import missions
from fleet_server import models as m
from fleet_server.db import make_session_factory


def _mk(db) -> m.Mission:
    farm = m.Farm(name="농장A"); db.add(farm); db.flush()
    db.add(m.Robot(id="scout01", farm_id=farm.id, name="r")); db.flush()
    db.add(m.User(login="op", pw_hash="h", role="operator")); db.flush()
    return missions.create(db, robot_id="scout01", farm_id=farm.id,
                           spec={"alleys": [0, 1]}, created_by=1)


@pytest.mark.parametrize("chain,final", [
    (["start", "complete"], "DONE"),
    (["start", "pause", "resume", "complete"], "DONE"),
    (["cancel"], "CANCELED"),                       # QUEUED→CANCELED (스펙 §4.3)
    (["start", "estop"], "PAUSED"),                 # estop → PAUSED
    (["start", "estop", "resume", "complete"], "DONE"),
    (["start", "fail"], "FAILED"),
    (["start", "pause", "cancel"], "CANCELED"),
])
def test_transitions(db, chain, final):
    ms = _mk(db)
    for ev in chain:
        ms = missions.apply(db, ms, ev)
    assert ms.state == final


def test_invalid_transition_raises(db):
    ms = _mk(db)
    ms = missions.apply(db, ms, "start")
    ms = missions.apply(db, ms, "complete")
    with pytest.raises(missions.InvalidTransition):
        missions.apply(db, ms, "resume")             # DONE 에서 재개 불가


def test_apply_optimistic_guard_stale_object(db):
    """REST 핸들러의 await 구간에 로봇 보고가 끼어들어 _sync_mission 이 먼저
    DONE 을 커밋하면, 재개된 핸들러가 들고 있던 stale 객체(RUNNING 로 믿음)로
    무조건 UPDATE 하면 DONE→PAUSED 로 역행할 수 있다. 낙관적 가드(기대 상태를
    조건으로 하는 UPDATE)가 이를 막고 InvalidTransition 을 낸다."""
    ms = _mk(db)
    ms = missions.apply(db, ms, "start")            # RUNNING — `db` 세션의 in-memory 상태

    # 별도 세션(레이스 상대) — 같은 임무를 완료 처리한다. `db` 세션의 identity map 은
    # (expire_on_commit=False 라) 이 커밋을 자동으로 반영하지 않는다 — 그래서 stale.
    factory2 = make_session_factory(db.get_bind())
    with factory2() as db2:
        ms2 = db2.get(m.Mission, ms.id)
        missions.apply(db2, ms2, "complete")         # 실제 DB 는 DONE

    assert ms.state == "RUNNING"                     # `db` 세션 관점에서는 여전히 stale
    with pytest.raises(missions.InvalidTransition):
        missions.apply(db, ms, "pause")               # DB 실제 상태는 DONE 이라 거부돼야 한다
    assert ms.state == "DONE"                          # 실패 시 호출자에게 실제 상태를 보여준다


def test_timestamps_and_events(db):
    ms = _mk(db)
    assert ms.started_at is None
    ms = missions.apply(db, ms, "start")
    assert ms.started_at is not None and ms.ended_at is None
    ms = missions.apply(db, ms, "complete")
    assert ms.ended_at is not None
    kinds = [e.kind for e in db.query(m.MissionEvent)
             .filter_by(mission_id=ms.id).order_by(m.MissionEvent.id)]
    assert kinds == ["start", "complete"]            # 전이마다 이력 1행
