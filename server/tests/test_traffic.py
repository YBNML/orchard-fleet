"""AlleyLock 충돌 규칙 진리표 + 획득/해제/복원 — 스펙 ③ §2."""
from fleet_server import models as m
from fleet_server.traffic import AlleyLocks, conflict, pads


def test_pads_consecutive_only():
    assert pads([0, 1, 2, 3]) == {(0, 1), (1, 2), (2, 3)}
    assert pads([0, 2, 4]) == set()          # 비연속은 패드 없음
    assert pads([5]) == set()


def test_conflict_rules():
    assert not conflict([0, 1, 2, 3], [5, 6, 7, 8])   # 통로4 버퍼 — 허용
    assert conflict([0, 1, 2, 3, 4], [4, 5, 6, 7, 8])   # 통로4 공유
    assert not conflict([0, 1, 2, 3], [4, 5, 6, 7, 8])  # 패드 (2,3) vs (4,5).. 분리 — 허용


def test_conflict_shared_alley():
    assert conflict([0, 1], [1, 2])               # 통로 1 공유


def test_no_conflict_disjoint_alleys_and_pads():
    assert not conflict([0, 1, 2], [3])           # 통로·패드 모두 분리 ([3] 패드 ∅)


# ── 획득·해제 ────────────────────────────────────────────────────────────────

def _mk_farm_and_robots(db, *robot_ids):
    farm = m.Farm(name="농장A")
    db.add(farm)
    db.flush()
    for rid in robot_ids:
        db.add(m.Robot(id=rid, farm_id=farm.id, name=rid))
    db.add(m.User(login="op", pw_hash="h", role="operator"))
    db.flush()
    return farm


def _mk_mission(db, robot_id="scout01", alleys=(0, 1)):
    from fleet_server import missions
    user = db.query(m.User).filter_by(login="op").one()
    return missions.create(db, robot_id=robot_id, farm_id=1,
                           spec={"alleys": list(alleys)}, created_by=user.id)


def test_acquire_ok_when_no_conflict(db):
    _mk_farm_and_robots(db, "scout01")
    ms = _mk_mission(db, "scout01", [0, 1])
    ok, reason = AlleyLocks.acquire(db, "scout01", ms.id, [0, 1])
    assert ok is True
    assert reason is None
    assert AlleyLocks.list_active(db) == [
        {"mission_id": ms.id, "robot_id": "scout01", "alleys": [0, 1]}]


def test_acquire_rejects_conflicting_alleys(db):
    _mk_farm_and_robots(db, "scout01", "scout02")
    ms1 = _mk_mission(db, "scout01", [0, 1])
    ms2 = _mk_mission(db, "scout02", [1, 2])
    ok1, _ = AlleyLocks.acquire(db, "scout01", ms1.id, [0, 1])
    assert ok1 is True
    ok2, reason2 = AlleyLocks.acquire(db, "scout02", ms2.id, [1, 2])
    assert ok2 is False
    assert reason2                              # 사유 문자열이 채워진다
    active = AlleyLocks.list_active(db)
    assert len(active) == 1                     # 실패한 시도는 자취를 남기지 않는다
    assert active[0]["mission_id"] == ms1.id


def test_release_removes_lock(db):
    _mk_farm_and_robots(db, "scout01")
    ms = _mk_mission(db, "scout01", [0, 1])
    AlleyLocks.acquire(db, "scout01", ms.id, [0, 1])
    AlleyLocks.release(db, ms.id)
    assert AlleyLocks.list_active(db) == []


def test_release_then_acquire_no_longer_conflicts(db):
    _mk_farm_and_robots(db, "scout01", "scout02")
    ms1 = _mk_mission(db, "scout01", [0, 1])
    ms2 = _mk_mission(db, "scout02", [0, 1])
    AlleyLocks.acquire(db, "scout01", ms1.id, [0, 1])
    AlleyLocks.release(db, ms1.id)
    ok, reason = AlleyLocks.acquire(db, "scout02", ms2.id, [0, 1])
    assert ok is True
    assert reason is None


def test_release_unknown_mission_is_noop(db):
    AlleyLocks.release(db, 999)                  # 없는 mission_id — 조용히 통과


# ── 서버 재기동 시 잠금 복원 ──────────────────────────────────────────────────


def test_restore_recreates_lock_for_running_mission_missing_row(db):
    """서버가 죽었다 살아나면(정상 해제 훅을 못 탄 RUNNING 임무) alley_locks 에
    행이 없을 수 있다 — 기동 시 정합 확인 로직이 spec_json.alleys 로 되살려야
    다음 겹치는 요청이 올바르게 막힌다."""
    from fleet_server import missions
    _mk_farm_and_robots(db, "scout01")
    ms = _mk_mission(db)
    ms = missions.apply(db, ms, "start")          # RUNNING — 그러나 lock 행은 없다
    assert AlleyLocks.list_active(db) == []

    AlleyLocks.restore(db)

    active = AlleyLocks.list_active(db)
    assert active == [{"mission_id": ms.id, "robot_id": "scout01", "alleys": [0, 1]}]


def test_restore_is_idempotent(db):
    from fleet_server import missions
    _mk_farm_and_robots(db, "scout01")
    ms = _mk_mission(db)
    missions.apply(db, ms, "start")
    AlleyLocks.restore(db)
    AlleyLocks.restore(db)                        # 두 번 불러도 중복 생성 없음
    assert len(AlleyLocks.list_active(db)) == 1


def test_restore_ignores_non_running_missions(db):
    _mk_farm_and_robots(db, "scout01")
    _mk_mission(db)                               # QUEUED — 복원 대상 아님
    AlleyLocks.restore(db)
    assert AlleyLocks.list_active(db) == []
