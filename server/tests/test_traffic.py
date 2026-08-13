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


def test_wildcard_conflicts_with_everything():
    """리뷰 라운드 1 C1 — alleys 생략(None, 와일드카드)은 무엇과도 충돌한다."""
    assert conflict(None, [5, 6])
    assert conflict([5, 6], None)
    assert conflict(None, None)


# ── 획득·해제 ────────────────────────────────────────────────────────────────

def _mk_farm_and_robots(db, farm_name="농장A", *robot_ids):
    farm = m.Farm(name=farm_name)
    db.add(farm)
    db.flush()
    for rid in robot_ids:
        db.add(m.Robot(id=rid, farm_id=farm.id, name=rid))
    if db.query(m.User).filter_by(login="op").first() is None:
        db.add(m.User(login="op", pw_hash="h", role="operator"))
    db.flush()
    return farm


def _mk_mission(db, robot_id, farm_id, alleys):
    from fleet_server import missions
    user = db.query(m.User).filter_by(login="op").one()
    spec = {} if alleys is None else {"alleys": list(alleys)}
    return missions.create(db, robot_id=robot_id, farm_id=farm_id,
                           spec=spec, created_by=user.id)


def test_acquire_ok_when_no_conflict(db):
    farm = _mk_farm_and_robots(db, "농장A", "scout01")
    ms = _mk_mission(db, "scout01", farm.id, [0, 1])
    ok, reason = AlleyLocks.acquire(db, "scout01", ms.id, [0, 1], farm.id)
    assert ok is True
    assert reason is None
    assert AlleyLocks.list_active(db) == [
        {"mission_id": ms.id, "robot_id": "scout01", "farm_id": farm.id, "alleys": [0, 1]}]


def test_acquire_rejects_conflicting_alleys(db):
    farm = _mk_farm_and_robots(db, "농장A", "scout01", "scout02")
    ms1 = _mk_mission(db, "scout01", farm.id, [0, 1])
    ms2 = _mk_mission(db, "scout02", farm.id, [1, 2])
    ok1, _ = AlleyLocks.acquire(db, "scout01", ms1.id, [0, 1], farm.id)
    assert ok1 is True
    ok2, reason2 = AlleyLocks.acquire(db, "scout02", ms2.id, [1, 2], farm.id)
    assert ok2 is False
    assert reason2                              # 사유 문자열이 채워진다
    active = AlleyLocks.list_active(db)
    assert len(active) == 1                     # 실패한 시도는 자취를 남기지 않는다
    assert active[0]["mission_id"] == ms1.id


def test_acquire_wildcard_conflicts_with_concrete_alleys(db):
    """C1 — 통로 생략(와일드카드) 임무가 먼저 잠그면 어떤 구체 통로 요청도 막힌다."""
    farm = _mk_farm_and_robots(db, "농장A", "scout01", "scout02")
    ms1 = _mk_mission(db, "scout01", farm.id, None)
    ok1, _ = AlleyLocks.acquire(db, "scout01", ms1.id, None, farm.id)
    assert ok1 is True
    assert AlleyLocks.list_active(db) == [
        {"mission_id": ms1.id, "robot_id": "scout01", "farm_id": farm.id, "alleys": None}]

    ms2 = _mk_mission(db, "scout02", farm.id, [7])
    ok2, reason2 = AlleyLocks.acquire(db, "scout02", ms2.id, [7], farm.id)
    assert ok2 is False
    assert reason2


def test_acquire_concrete_alleys_conflicts_with_existing_wildcard(db):
    farm = _mk_farm_and_robots(db, "농장A", "scout01", "scout02")
    ms1 = _mk_mission(db, "scout01", farm.id, [3])
    AlleyLocks.acquire(db, "scout01", ms1.id, [3], farm.id)

    ms2 = _mk_mission(db, "scout02", farm.id, None)
    ok2, reason2 = AlleyLocks.acquire(db, "scout02", ms2.id, None, farm.id)
    assert ok2 is False
    assert reason2


def test_release_removes_lock(db):
    farm = _mk_farm_and_robots(db, "농장A", "scout01")
    ms = _mk_mission(db, "scout01", farm.id, [0, 1])
    AlleyLocks.acquire(db, "scout01", ms.id, [0, 1], farm.id)
    AlleyLocks.release(db, ms.id)
    assert AlleyLocks.list_active(db) == []


def test_release_then_acquire_no_longer_conflicts(db):
    farm = _mk_farm_and_robots(db, "농장A", "scout01", "scout02")
    ms1 = _mk_mission(db, "scout01", farm.id, [0, 1])
    ms2 = _mk_mission(db, "scout02", farm.id, [0, 1])
    AlleyLocks.acquire(db, "scout01", ms1.id, [0, 1], farm.id)
    AlleyLocks.release(db, ms1.id)
    ok, reason = AlleyLocks.acquire(db, "scout02", ms2.id, [0, 1], farm.id)
    assert ok is True
    assert reason is None


def test_release_unknown_mission_is_noop(db):
    AlleyLocks.release(db, 999)                  # 없는 mission_id — 조용히 통과


def test_acquire_same_alleys_different_farms_do_not_conflict(db):
    """I5 — 서로 다른 farm(별개 과수원)은 통로 번호가 같아도 무관하다."""
    farmA = _mk_farm_and_robots(db, "농장A", "scout01")
    farmB = _mk_farm_and_robots(db, "농장B", "scout02")
    msA = _mk_mission(db, "scout01", farmA.id, [0, 1])
    msB = _mk_mission(db, "scout02", farmB.id, [0, 1])
    okA, _ = AlleyLocks.acquire(db, "scout01", msA.id, [0, 1], farmA.id)
    okB, reasonB = AlleyLocks.acquire(db, "scout02", msB.id, [0, 1], farmB.id)
    assert okA is True
    assert okB is True                            # 다른 농장 — 충돌 아님
    assert reasonB is None
    assert len(AlleyLocks.list_active(db)) == 2


# ── 서버 재기동 시 잠금 복원 ──────────────────────────────────────────────────

def test_restore_recreates_lock_for_running_mission_missing_row(db):
    """서버가 죽었다 살아나면(정상 해제 훅을 못 탄 RUNNING 임무) alley_locks 에
    행이 없을 수 있다 — 기동 시 정합 확인 로직이 spec_json.alleys 로 되살려야
    다음 겹치는 요청이 올바르게 막힌다."""
    from fleet_server import missions
    farm = _mk_farm_and_robots(db, "농장A", "scout01")
    ms = _mk_mission(db, "scout01", farm.id, [0, 1])
    ms = missions.apply(db, ms, "start")          # RUNNING — 그러나 lock 행은 없다
    assert AlleyLocks.list_active(db) == []

    AlleyLocks.restore(db)

    active = AlleyLocks.list_active(db)
    assert active == [{"mission_id": ms.id, "robot_id": "scout01",
                       "farm_id": farm.id, "alleys": [0, 1]}]


def test_restore_recreates_wildcard_lock(db):
    from fleet_server import missions
    farm = _mk_farm_and_robots(db, "농장A", "scout01")
    ms = _mk_mission(db, "scout01", farm.id, None)     # 통로 생략 — spec_json 에 alleys 키 없음
    ms = missions.apply(db, ms, "start")
    AlleyLocks.restore(db)
    active = AlleyLocks.list_active(db)
    assert active == [{"mission_id": ms.id, "robot_id": "scout01",
                       "farm_id": farm.id, "alleys": None}]


def test_restore_is_idempotent(db):
    from fleet_server import missions
    farm = _mk_farm_and_robots(db, "농장A", "scout01")
    ms = _mk_mission(db, "scout01", farm.id, [0, 1])
    missions.apply(db, ms, "start")
    AlleyLocks.restore(db)
    AlleyLocks.restore(db)                        # 두 번 불러도 중복 생성 없음
    assert len(AlleyLocks.list_active(db)) == 1


def test_restore_ignores_non_running_missions(db):
    farm = _mk_farm_and_robots(db, "농장A", "scout01")
    _mk_mission(db, "scout01", farm.id, [0, 1])    # QUEUED — 복원 대상 아님
    AlleyLocks.restore(db)
    assert AlleyLocks.list_active(db) == []
