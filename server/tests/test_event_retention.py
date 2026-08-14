"""Task 6 — 이벤트 보존정책: TTL(EVENT_TTL_DAYS=7) 초과 행 정리.

리뷰 I2(정책 역전) — 안전·수명주기 kind(estop 계열·assistance·resolved·
denied·link_lost·mission_*·cmd_result)는 표준 TTL 이 아니라 더 긴
EVENT_TTL_SAFE_DAYS(기본 90일)를 따른다. 아래 테스트는 "안전하지 않은"
일반 kind 로 "paused" 를 쓴다 — estop 등 실제 안전 kind 를 표준 TTL 테스트에
쓰면 이제 그 자체로 틀린 단언이 된다(리뷰가 지적한 실수를 테스트에서도
반복하지 않기 위함).
"""
import asyncio
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


# ── 표준 TTL(비-안전 kind) ───────────────────────────────────────────────

def test_purge_expired_deletes_only_rows_older_than_ttl(factory):
    _seed_robot(factory)
    now = dt.datetime.now(dt.UTC)
    with factory() as db:
        db.add(m.Event(robot_id="scout01", ts=now - dt.timedelta(days=8),
                       channel="evt", seq=1, kind="paused"))
        db.add(m.Event(robot_id="scout01", ts=now - dt.timedelta(days=6),
                       channel="evt", seq=2, kind="paused"))
        db.commit()
    with factory() as db:
        n = purge_expired(db, ttl_days=7, safe_ttl_days=90)
    assert n == 1
    with factory() as db:
        rows = db.query(m.Event).all()
    assert len(rows) == 1 and rows[0].seq == 2


def test_purge_expired_ttl_zero_or_negative_is_noop(factory):
    """TTL 을 0 이하로 설정하면(운영 실수 방지) 그 축은 아무것도 지우지 않는다
    — 무제한 삭제로 오해될 수 있는 값을 안전한 방향(비활성)으로 떨군다."""
    _seed_robot(factory)
    with factory() as db:
        db.add(m.Event(robot_id="scout01", ts=dt.datetime(2000, 1, 1, tzinfo=dt.UTC),
                       channel="evt", seq=1, kind="paused"))
        db.commit()
    with factory() as db:
        assert purge_expired(db, ttl_days=0, safe_ttl_days=90) == 0
    with factory() as db:
        assert db.query(m.Event).count() == 1


def test_retention_task_run_once_purges_expired_rows(factory):
    _seed_robot(factory)
    old = dt.datetime.now(dt.UTC) - dt.timedelta(days=30)
    with factory() as db:
        db.add(m.Event(robot_id="scout01", ts=old, channel="evt", seq=1, kind="paused"))
        db.commit()
    task = RetentionTask(factory, ttl_days=7, safe_ttl_days=90)
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
        db.add(m.Event(robot_id="scout01", ts=old, channel="evt", seq=1, kind="paused"))
        db.commit()
    task = RetentionTask(factory, ttl_days=7, safe_ttl_days=90, period_s=3600.0)
    await task.start()
    for _ in range(50):
        with factory() as db:
            if db.query(m.Event).count() == 0:
                break
        await asyncio.sleep(0.02)
    with factory() as db:
        assert db.query(m.Event).count() == 0
    await task.stop()


# ── I2 — 안전·수명주기 kind 는 더 긴 TTL ────────────────────────────────

def test_purge_expired_safe_kinds_use_longer_ttl(factory):
    """실측(T6 §8.4): pong 미기록 뒤 첫 TTL 실행이 지운 122행은 전부 이
    범주였다(pong 0건) — 표준 TTL(7일)이 지났어도 안전 TTL(기본 90일)이
    남아 있으면 지워지면 안 된다."""
    _seed_robot(factory)
    now = dt.datetime.now(dt.UTC)
    safe_kinds = ("estop", "estop_cleared", "estop_clear_requested", "assistance",
                 "resolved", "denied", "link_lost", "mission_started",
                 "mission_done", "mission_cancelled", "cmd_result")
    with factory() as db:
        for i, kind in enumerate(safe_kinds):
            db.add(m.Event(robot_id="scout01", ts=now - dt.timedelta(days=8),
                           channel="evt", seq=i, kind=kind))
        db.commit()
    with factory() as db:
        n = purge_expired(db, ttl_days=7, safe_ttl_days=90)
    assert n == 0
    with factory() as db:
        assert db.query(m.Event).count() == len(safe_kinds)


def test_purge_expired_safe_kinds_eventually_expire_too(factory):
    """안전 kind 도 무기한 보존은 아니다 — safe_ttl_days 를 넘으면 지운다."""
    _seed_robot(factory)
    old = dt.datetime.now(dt.UTC) - dt.timedelta(days=91)
    with factory() as db:
        db.add(m.Event(robot_id="scout01", ts=old, channel="evt", seq=1, kind="estop"))
        db.add(m.Event(robot_id="scout01", ts=old, channel="evt", seq=2, kind="mission_done"))
        db.commit()
    with factory() as db:
        n = purge_expired(db, ttl_days=7, safe_ttl_days=90)
    assert n == 2


def test_purge_expired_safe_ttl_zero_or_negative_is_noop_for_safe_axis(factory):
    """safe_ttl_days<=0 은 안전 kind 축만 비활성화한다(표준 축은 그대로 동작)."""
    _seed_robot(factory)
    now = dt.datetime.now(dt.UTC)
    with factory() as db:
        db.add(m.Event(robot_id="scout01", ts=now - dt.timedelta(days=100),
                       channel="evt", seq=1, kind="estop"))
        db.add(m.Event(robot_id="scout01", ts=now - dt.timedelta(days=8),
                       channel="evt", seq=2, kind="paused"))
        db.commit()
    with factory() as db:
        n = purge_expired(db, ttl_days=7, safe_ttl_days=0)
    assert n == 1                          # paused(표준 축)만 지워진다
    with factory() as db:
        kinds = {e.kind for e in db.query(m.Event).all()}
    assert kinds == {"estop"}
