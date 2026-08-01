import pytest

from fleet_server import missions
from fleet_server import models as m


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
