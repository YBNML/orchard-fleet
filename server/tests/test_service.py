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


def test_tel_state_feed_creates_track(factory):
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
