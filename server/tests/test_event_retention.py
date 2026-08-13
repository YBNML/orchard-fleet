"""Task 6 — 이벤트 보존정책: TTL(EVENT_TTL_DAYS=7) 초과 행 정리."""
import datetime as dt

import pytest

from fleet_server import models as m
from fleet_server.db import Base, make_engine, make_session_factory
from fleet_server.event_retention import RetentionTask, purge_expired


@pytest.fixture()
def factory():
    engine = make_engine("sqlite://")           # in-memory, 이 fixture 전용 엔진
    Base.metadata.create_all(engine)
    return make_session_factory(engine)


def _seed_robot(factory, robot_id: str = "scout01") -> None:
    with factory() as db:
        farm = m.Farm(name="농장A"); db.add(farm); db.flush()
        db.add(m.Robot(id=robot_id, farm_id=farm.id, name="r"))
        db.commit()


def test_purge_expired_deletes_only_rows_older_than_ttl(factory):
    _seed_robot(factory)
    now = dt.datetime.now(dt.UTC)
    with factory() as db:
        db.add(m.Event(robot_id="scout01", ts=now - dt.timedelta(days=8),
                       channel="evt", seq=1, kind="estop"))
        db.add(m.Event(robot_id="scout01", ts=now - dt.timedelta(days=6),
                       channel="evt", seq=2, kind="estop"))
        db.commit()
    with factory() as db:
        n = purge_expired(db, ttl_days=7)
    assert n == 1
    with factory() as db:
        rows = db.query(m.Event).all()
    assert len(rows) == 1 and rows[0].seq == 2


def test_purge_expired_ttl_zero_or_negative_is_noop(factory):
    """TTL 을 0 이하로 설정하면(운영 실수 방지) 아무것도 지우지 않는다 —
    무제한 삭제로 오해될 수 있는 값을 안전한 방향(비활성)으로 떨군다."""
    _seed_robot(factory)
    with factory() as db:
        db.add(m.Event(robot_id="scout01", ts=dt.datetime(2000, 1, 1, tzinfo=dt.UTC),
                       channel="evt", seq=1, kind="estop"))
        db.commit()
    with factory() as db:
        assert purge_expired(db, ttl_days=0) == 0
    with factory() as db:
        assert db.query(m.Event).count() == 1


def test_retention_task_run_once_purges_expired_rows(factory):
    _seed_robot(factory)
    old = dt.datetime.now(dt.UTC) - dt.timedelta(days=30)
    with factory() as db:
        db.add(m.Event(robot_id="scout01", ts=old, channel="evt", seq=1, kind="estop"))
        db.commit()
    task = RetentionTask(factory, ttl_days=7)
    n = task.run_once()
    assert n == 1
    with factory() as db:
        assert db.query(m.Event).count() == 0


@pytest.mark.asyncio
async def test_retention_task_start_runs_once_immediately(factory):
    """기동 시 1회 — sleep 을 기다리지 않고도 첫 정리가 즉시 돈다."""
    _seed_robot(factory)
    old = dt.datetime.now(dt.UTC) - dt.timedelta(days=30)
    with factory() as db:
        db.add(m.Event(robot_id="scout01", ts=old, channel="evt", seq=1, kind="estop"))
        db.commit()
    task = RetentionTask(factory, ttl_days=7, period_s=3600.0)
    await task.start()
    import asyncio
    for _ in range(50):
        with factory() as db:
            if db.query(m.Event).count() == 0:
                break
        await asyncio.sleep(0.02)
    with factory() as db:
        assert db.query(m.Event).count() == 0
    await task.stop()
