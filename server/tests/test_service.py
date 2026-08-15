"""FleetService — 텔레메트리 허브(최신값 캐시·DB 수집·구독자 팬아웃·임무 동기화)."""
import datetime as dt

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


# ── Task 6 — 확장 A: 세션(연결) 에폭으로 seq 되감김을 재전송과 구분 ──────────
#
# 로봇 seq(robomw control_agent 의 self.seq)는 프로세스 전역·전 채널 공유
# 카운터라 재기동하면 0 으로 되감긴다. dedup 키가 (robot_id, channel, seq) 뿐이면
# 재기동 뒤 재사용된 seq 가 옛 세션의 것과 겹쳐 "재전송"으로 오인되어 진짜
# 신규 이벤트가 버려진다(실사고: 임무 22 완료보고 유실, 임무 23 수락 2건
# 유실 — QUEUED 고착). hello 는 접속 직후 1회 발행되는 로봇 계약(T_HELLO)이라
# 재기동을 건드리지 않고 관측할 수 있는 유일한 신호다 — 그 hello 가 실어 온
# seq 가 이 로봇에서 지금까지 관측한 고수위표보다 크지 않으면(뒤로 뜨거나
# 같으면) 카운터가 리셋된 것으로 보고 세션 에폭을 올린다. UNIQUE 제약을
# (robot_id, channel, epoch, seq) 로 확장해, 다른 에폭끼리는 같은 seq 를 써도
# 충돌하지 않는다.

def test_evt_seq_rewind_after_restart_hello_is_not_treated_as_retransmit(factory):
    """세션 A 에서 이미 쓰인 seq 를, hello 로 감지된 재기동(세션 B) 뒤에 다시
    써도 재전송으로 버려지면 안 된다 — 에폭이 다르기 때문이다."""
    _seed_robot(factory)
    fp, svc = _svc(factory)
    fp.feed("scout01", "evt", {"kind": "estop", "msg": "정지A"}, seq=100)
    # 로봇 재기동 — hello 가 되감긴 seq(1)로 도착한다(고수위 100보다 작다)
    fp.feed("scout01", "hello", {"robot_id": "scout01"}, seq=1)
    assert svc._epoch.get("scout01", 0) == 1            # 에폭이 올라갔다
    # 세션 B 가 세션 A 와 같은 seq(100)를 다시 쓴다 — 다른 에폭이라 유효한 신규 이벤트
    fp.feed("scout01", "evt", {"kind": "estop", "msg": "정지B"}, seq=100)
    with factory() as db:
        rows = db.query(m.Event).filter_by(robot_id="scout01").order_by(m.Event.id).all()
    assert [r.msg for r in rows] == ["정지A", "정지B"]


def test_evt_same_epoch_duplicate_seq_still_deduped(factory):
    """dedup 의 목적(링크 재전송 중복 차단)은 유지된다 — 같은 세션, 같은 seq 의
    진짜 재전송은 에폭 도입 뒤에도 계속 걸러져야 한다."""
    _seed_robot(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "hello", {"robot_id": "scout01"}, seq=1)   # 세션 시작(첫 hello — 에폭 불변)
    fp.feed("scout01", "evt", {"kind": "estop", "msg": "정지"}, seq=5)
    fp.feed("scout01", "evt", {"kind": "estop", "msg": "정지(재전송)"}, seq=5)
    with factory() as db:
        rows = db.query(m.Event).filter_by(robot_id="scout01").all()
    assert len(rows) == 1 and rows[0].msg == "정지"


def test_evt_accept_after_session_restart_is_not_lost(factory):
    """실사고 재현(임무 23) — 로봇 재기동 뒤 되감긴 seq 가 이전 세션의 것과
    우연히 겹쳐도, 그 세션에서 낸 임무 수락 신호가 조용히 사라지면 안 된다.
    수정 전에는 fresh=False 로 버려져 임무가 QUEUED 에 영원히 고착됐다."""
    ms1 = _seed_mission(factory, "scout01")
    with factory() as db:
        farm_id = db.get(m.Mission, ms1).farm_id
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", _accepted(ms1), seq=5)
    assert _state(factory, ms1) == "RUNNING"
    # 로봇 재기동 — hello 가 되감긴 seq 로 도착(세션 갱신)
    fp.feed("scout01", "hello", {"robot_id": "scout01"}, seq=1)
    with factory() as db:
        ms2 = missions.create(db, robot_id="scout01", farm_id=farm_id,
                              spec={"alleys": [1]}, created_by=1).id
    fp.feed("scout01", "evt", _accepted(ms2), seq=5)      # 옛 세션과 같은 seq
    assert _state(factory, ms2) == "RUNNING"               # 유실되지 않는다


# ── 리뷰 수정 라운드 1 (C1·I3) ────────────────────────────────────────────

def test_seed_epoch_uses_max_not_last_row(factory):
    """C1 — 기존 행이 있는 상태의 시딩 경로(원래 커버리지 0 이던 경로)를
    검증한다. "마지막 행"(id 최댓값)의 epoch·seq 가 아니라 **그 로봇의
    MAX(epoch)+1·MAX(seq)** 로 시딩해야 한다. 라이브 events 82k 행이 전부
    epoch=0(여러 세션이 뒤섞인 공간)인 상황을 재현: id 순서(삽입 순서)와
    seq 크기가 어긋나는(재기동 뒤섞임) 행을 미리 깔아 둔다 — id 가 더 큰
    (나중에 적힌) 행이 오히려 seq 는 더 작다."""
    _seed_robot(factory)
    with factory() as db:
        db.add_all([
            m.Event(id=1, robot_id="scout01", ts=dt.datetime.now(dt.UTC),
                    channel="evt", seq=42570, epoch=0, kind="estop"),
            m.Event(id=2, robot_id="scout01", ts=dt.datetime.now(dt.UTC),
                    channel="evt", seq=100, epoch=0, kind="estop"),   # 마지막 행이지만 seq 는 더 작다
        ])
        db.commit()
    fp, svc = _svc(factory)
    fp.feed("scout01", "evt", {"kind": "assistance", "msg": "x"}, seq=101)
    assert svc._epoch.get("scout01", 0) == 1             # MAX(epoch)=0 + 1— "마지막 행"의 epoch(0)가 아니다
    assert svc._last_seq.get("scout01") == 42570          # MAX(seq) — "마지막 행"의 seq(100)가 아니다


def test_evt_repeated_reconnect_after_restart_does_not_rebump_epoch(factory):
    """I3(a) — 에폭 bump 후 고수위표를 새 세션 기준으로 재설정해야 한다.
    안 하면 첫 재기동 이후의 모든 정상 재접속(hello)이 옛(재기동 전) 고수위와
    비교돼 매번 에폭을 또 올린다(상시 오탐 — 정상 재접속을 새 세션으로
    오인)."""
    _seed_robot(factory)
    fp, svc = _svc(factory)
    fp.feed("scout01", "evt", {"kind": "estop", "msg": "x"}, seq=100)
    fp.feed("scout01", "hello", {"robot_id": "scout01"}, seq=1)       # 진짜 재기동 — 에폭 1
    assert svc._epoch.get("scout01", 0) == 1
    fp.feed("scout01", "evt", {"kind": "estop", "msg": "y"}, seq=50)  # 새 세션 진행
    fp.feed("scout01", "hello", {"robot_id": "scout01"}, seq=60)      # 단순 재접속(seq 는 계속 이어짐)
    assert svc._epoch.get("scout01", 0) == 1              # 오탐 없음 — 재차 bump 되지 않는다


def test_evt_seq_rewind_still_detected_after_prior_bump(factory):
    """I3(b) 회귀 — 고수위표 재설정 이후에도 진짜 되감김(재기동)은 여전히
    감지돼야 한다(오탐 제거가 과해서 진짜 재기동까지 놓치면 안 된다)."""
    _seed_robot(factory)
    fp, svc = _svc(factory)
    fp.feed("scout01", "evt", {"kind": "estop", "msg": "x"}, seq=100)
    fp.feed("scout01", "hello", {"robot_id": "scout01"}, seq=1)       # 1차 재기동 — 에폭 1
    fp.feed("scout01", "evt", {"kind": "estop", "msg": "y"}, seq=50)
    fp.feed("scout01", "hello", {"robot_id": "scout01"}, seq=3)       # 2차 재기동(고수위 50보다 작다)
    assert svc._epoch.get("scout01", 0) == 2


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


# ── 명령 응답은 개입 티켓이 아니다 (T7 수정 라운드 1 · I1) ──────────────────
#
# `cmd_result`·`report` 의 `code` 는 **명령 처리 결과 코드**(성공 "OK", 실패
# "BUSY"·"TIMEOUT"·"BAD_PARAM")이지 정지 사유 코드가 아니다. 그런데
# `stopcodes.get` 은 모르는 코드를 UNKNOWN(needs_operator=True)으로 떨어뜨려서,
# 거르지 않으면 정상적으로 수락된 명령 하나하나가 "분류되지 않은 정지" 티켓을
# 열었다 (2026-08-15 T7 게이트 실측: 발진 3초 만에 재현, 개입 지표 오염).

def _open_ivs(factory, robot_id="scout01"):
    with factory() as db:
        return [r.code for r in db.query(m.Intervention)
                .filter(m.Intervention.robot_id == robot_id).all()]


def test_cmd_result_ok_opens_no_intervention(factory):
    """accepted/completed(code OK) — 티켓 0건. 임무 상태기계는 그대로 돈다."""
    ms_id = _seed_mission(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", _accepted(ms_id), seq=1)
    fp.feed("scout01", "evt", _completed(ms_id), seq=2)
    assert _open_ivs(factory) == []
    assert _state(factory, ms_id) == "DONE"       # 소비 경로는 살아 있다


@pytest.mark.parametrize("cmd,status,code,msg", [
    ("relocalize", "failed", "TIMEOUT", "relocalize → failed (TIMEOUT)"),
    ("relocalize", "rejected", "BUSY", "relocalize → rejected (BUSY)"),
    ("self_test", "completed", "OK", "self_test → completed"),
    ("mission_cancel", "accepted", "OK", "mission_cancel → accepted"),
])
def test_cmd_result_any_status_opens_no_intervention(factory, cmd, status, code, msg):
    """실패·거부 응답도 티켓 채널이 아니다 — 사람을 불러야 하면 로봇이
    `assistance` 를 따로 낸다. (실측된 네 가지 형태를 그대로 넣는다)"""
    _seed_robot(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", {"kind": "cmd_result", "cmd_id": "x1", "cmd": cmd,
                               "status": status, "code": code, "data": {},
                               "msg": msg, "level": "info"}, seq=1)
    assert _open_ivs(factory) == []


def test_report_kind_opens_no_intervention(factory):
    """완료 보고(report)도 같다 — code 는 결과 코드다."""
    _seed_robot(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", {"kind": "report", "code": "OK",
                               "data": _REPORT, "msg": "임무 보고"}, seq=1)
    assert _open_ivs(factory) == []


def test_real_stop_codes_still_open_intervention(factory):
    """오탐 제거가 과해서 **진짜 개입**까지 놓치면 안 된다 — 회귀 방어.

    T7 게이트에서 실제로 올라온 두 사유를 그대로 쓴다."""
    _seed_robot(factory)
    fp, _ = _svc(factory)
    fp.feed("scout01", "evt", {"kind": "assistance", "code": "TRACTION_LOSS",
                               "msg": "오도메트리는 0.52 m 전진을 보고하는데 "
                                      "스캔 변위는 0.00 m — 바퀴 헛돎"}, seq=1)
    fp.feed("scout01", "evt", {"kind": "assistance", "code": "LOCALIZATION_LOST",
                               "msg": "8초째 위치 보정 실패"}, seq=2)
    fp.feed("scout01", "evt", {"kind": "estop", "msg": "비상정지"}, seq=3)
    assert sorted(_open_ivs(factory)) == ["ESTOP_REMOTE", "LOCALIZATION_LOST",
                                          "TRACTION_LOSS"]
