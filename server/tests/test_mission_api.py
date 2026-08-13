import asyncio

import httpx
from fastapi.testclient import TestClient

from tests.conftest import _test_settings, do_login


def _seed_operator(client):
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    fb = client.post("/api/v1/farms", json={"name": "농장B"}, headers=h).json()
    client.post("/api/v1/robots", headers=h,
                json={"id": "scout01", "farm_id": fa["id"], "name": "r"})
    client.post("/api/v1/robots", headers=h,
                json={"id": "scout02", "farm_id": fb["id"], "name": "r2"})
    client.post("/api/v1/users", headers=h, json={
        "login": "op", "password": "oppw", "role": "operator", "farm_ids": [fa["id"]]})
    return fa, fb


def _add_robot(client, robot_id, farm_id, name="r"):
    """scout01 과 같은 farm 에 로봇을 하나 더 등록한다(admin 세션 재사용) —
    AlleyLock 은 같은 farm 안에서만 충돌하므로(I5), 잠금 충돌 시나리오는
    서로 다른 farm 인 scout01/scout02(_seed_operator) 로는 만들 수 없다."""
    csrf0 = do_login(client)
    client.post("/api/v1/robots", headers={"X-CSRF": csrf0},
               json={"id": robot_id, "farm_id": farm_id, "name": name})


def test_mission_flow(client, app):
    _seed_operator(client)
    app.state.fleet.feed("scout01", "tel/state", {})           # 온라인 전환
    csrf = do_login(client, "op", "oppw")
    h = {"X-CSRF": csrf}
    r = client.post("/api/v1/missions", headers=h,
                    json={"robot_id": "scout01", "alleys": [0, 1]})
    assert r.status_code == 200, r.text
    ms = r.json()
    assert ms["state"] == "QUEUED"
    assert app.state.fleet.sent[-1][2] == "mission_start"      # 로봇으로 전달됨
    # 일시정지 → RUNNING 이 아니므로 409 (QUEUED 에서 pause 는 전이 불가)
    assert client.post(f"/api/v1/missions/{ms['id']}/pause", headers=h).status_code == 409
    # 취소는 QUEUED 에서 가능
    assert client.post(f"/api/v1/missions/{ms['id']}/cancel", headers=h).status_code == 200


def test_offline_robot_409(client):
    _seed_operator(client)
    csrf = do_login(client, "op", "oppw")
    r = client.post("/api/v1/missions", headers={"X-CSRF": csrf},
                    json={"robot_id": "scout01", "alleys": [0]})
    assert r.status_code == 409                                # 오프라인 → 즉시 실패


def test_cross_farm_403(client, app):
    _seed_operator(client)
    app.state.fleet.feed("scout02", "tel/state", {})
    csrf = do_login(client, "op", "oppw")                      # op 는 농장A만
    r = client.post("/api/v1/missions", headers={"X-CSRF": csrf},
                    json={"robot_id": "scout02", "alleys": [0]})
    assert r.status_code == 403


def test_observer_cannot_create(client, app):
    fa, _ = _seed_operator(client)
    csrf0 = do_login(client)
    client.post("/api/v1/users", headers={"X-CSRF": csrf0}, json={
        "login": "obs", "password": "obspw", "role": "observer", "farm_ids": [fa["id"]]})
    app.state.fleet.feed("scout01", "tel/state", {})
    csrf = do_login(client, "obs", "obspw")
    r = client.post("/api/v1/missions", headers={"X-CSRF": csrf},
                    json={"robot_id": "scout01", "alleys": [0]})
    assert r.status_code == 403


def test_mission_timestamps_have_tz_suffix_after_fresh_query(client, app):
    """POST 응답(같은 세션)이 아니라, 이후 GET(다른 세션 — SQLite 가 naive 로
    돌려주는 상황)에서 시각 문자열에 tz 접미사가 있어야 한다 (Critical 2 회귀)."""
    _seed_operator(client)
    app.state.fleet.feed("scout01", "tel/state", {})
    csrf = do_login(client, "op", "oppw")
    h = {"X-CSRF": csrf}
    client.post("/api/v1/missions", headers=h, json={"robot_id": "scout01", "alleys": [0]})

    def _has_tz(ts: str) -> bool:
        return ts.endswith("Z") or ts[-6] in "+-"

    rows = client.get("/api/v1/missions").json()
    assert rows
    for m in rows:
        assert _has_tz(m["created_at"])
        if m["started_at"]:
            assert _has_tz(m["started_at"])
        if m["ended_at"]:
            assert _has_tz(m["ended_at"])


def test_duplicate_active_mission_409(client, app):
    """로봇당 활성 임무(QUEUED/RUNNING/PAUSED)는 하나뿐이어야 한다 — 아니면
    _sync_mission 의 "최신 활성 임무" 휴리스틱이 오귀속될 수 있다."""
    _seed_operator(client)
    app.state.fleet.feed("scout01", "tel/state", {})
    csrf = do_login(client, "op", "oppw")
    h = {"X-CSRF": csrf}
    r1 = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout01", "alleys": [0]})
    assert r1.status_code == 200, r1.text
    r2 = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout01", "alleys": [1]})
    assert r2.status_code == 409, r2.text
    from fleet_server.models import Mission
    with app.state.session_factory() as db:
        active = db.query(Mission).filter(Mission.robot_id == "scout01",
                                          Mission.state.in_(["QUEUED", "RUNNING", "PAUSED"])).all()
        assert len(active) == 1                    # 두 번째 요청이 실제로 임무를 만들지 않음


def test_work_passed_through_and_alleys_omitted(client, app):
    """work:{type:"scout"} 만 보내면(alleys 생략) 로봇으로 나가는 mission_start
    payload 에 work 는 그대로 실리고 alleys 키는 아예 없어야 한다(전 통로 자동)."""
    _seed_operator(client)
    app.state.fleet.feed("scout01", "tel/state", {})
    csrf = do_login(client, "op", "oppw")
    h = {"X-CSRF": csrf}
    r = client.post("/api/v1/missions", headers=h,
                    json={"robot_id": "scout01", "work": {"type": "scout"}})
    assert r.status_code == 200, r.text
    sent_payload = app.state.fleet.sent[-1][3]
    assert sent_payload["work"] == {"type": "scout"}
    assert "alleys" not in sent_payload


def test_alleys_omitted_key_absent_when_both_given_absent(client, app):
    """alleys·work 둘 다 생략해도(로봇이 이후 BAD_PARAM 등으로 판정) 서버는
    검증하지 않고 그대로 보내며, payload 에 alleys 키는 여전히 없어야 한다."""
    _seed_operator(client)
    app.state.fleet.feed("scout01", "tel/state", {})
    csrf = do_login(client, "op", "oppw")
    h = {"X-CSRF": csrf}
    r = client.post("/api/v1/missions", headers=h, json={"robot_id": "scout01"})
    assert r.status_code == 200, r.text
    sent_payload = app.state.fleet.sent[-1][3]
    assert "alleys" not in sent_payload
    assert "work" not in sent_payload
    assert sent_payload["mission_id"] == r.json()["id"]


def test_overlapping_mission_becomes_queued_lock_and_not_dispatched(client, app):
    """AlleyLock 교통관리 — 통로가 겹치는 임무는 로봇에 나가지 않고 QUEUED_LOCK
    으로 남는다. cancel 로 정리할 수 있다(스펙 ③ §2). 같은 farm 의 두 로봇으로
    시나리오를 만든다 — 다른 farm 끼리는 겹쳐도 충돌이 아니다(I5)."""
    fa, _ = _seed_operator(client)
    _add_robot(client, "scout03", fa["id"])       # scout01 과 같은 farm
    app.state.fleet.feed("scout01", "tel/state", {})
    app.state.fleet.feed("scout03", "tel/state", {})
    csrf = do_login(client)                      # admin — 두 농장 다 본다
    h = {"X-CSRF": csrf}
    r1 = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout01", "alleys": [0, 1]})
    assert r1.status_code == 200, r1.text
    sent_before = len(app.state.fleet.sent)

    r2 = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout03", "alleys": [1, 2]})   # 통로 1 공유
    assert r2.status_code == 200, r2.text
    ms2 = r2.json()
    assert ms2["state"] == "QUEUED_LOCK"
    assert ms2["lock_reason"]
    assert len(app.state.fleet.sent) == sent_before   # mission_start 가 로봇으로 나가지 않았다

    locks = client.get("/api/v1/alley-locks", headers=h).json()
    assert len(locks) == 1
    assert locks[0] == {"mission_id": r1.json()["id"], "robot_id": "scout01",
                        "farm_id": fa["id"], "alleys": [0, 1]}

    cr = client.post(f"/api/v1/missions/{ms2['id']}/cancel", headers=h)
    assert cr.status_code == 200, cr.text
    assert cr.json()["state"] == "CANCELED"


def test_lock_released_on_cancel_allows_next_overlapping_mission(client, app):
    fa, _ = _seed_operator(client)
    _add_robot(client, "scout03", fa["id"])
    app.state.fleet.feed("scout01", "tel/state", {})
    app.state.fleet.feed("scout03", "tel/state", {})
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    ms1 = client.post("/api/v1/missions", headers=h,
                      json={"robot_id": "scout01", "alleys": [0, 1]}).json()
    assert client.post(f"/api/v1/missions/{ms1['id']}/cancel",
                       headers=h).status_code == 200
    assert client.get("/api/v1/alley-locks", headers=h).json() == []

    r2 = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout03", "alleys": [0, 1]})
    assert r2.status_code == 200, r2.text
    assert r2.json()["state"] == "QUEUED"          # 잠금이 풀려 정상 발진된다


def test_full_scout_wildcard_lock_blocks_overlapping_mission(client, app):
    """C1 — alleys 생략(대시보드 "전체 정찰") 임무도 잠금 없이 나가면 안 된다.
    전체 점유(와일드카드)로 잠기므로, 같은 farm 의 다른 임무는 통로를 어떻게
    골라도(심지어 완전히 무관해 보이는 통로라도) QUEUED_LOCK 이어야 한다."""
    fa, _ = _seed_operator(client)
    _add_robot(client, "scout03", fa["id"])
    app.state.fleet.feed("scout01", "tel/state", {})
    app.state.fleet.feed("scout03", "tel/state", {})
    csrf = do_login(client, "op", "oppw")
    h = {"X-CSRF": csrf}
    r1 = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout01", "work": {"type": "scout"}})
    assert r1.status_code == 200, r1.text
    assert r1.json()["state"] == "QUEUED"
    locks = client.get("/api/v1/alley-locks", headers=h).json()
    assert locks == [{"mission_id": r1.json()["id"], "robot_id": "scout01",
                      "farm_id": fa["id"], "alleys": None}]

    r2 = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout03", "alleys": [7]})
    assert r2.status_code == 200, r2.text
    assert r2.json()["state"] == "QUEUED_LOCK"     # 와일드카드와는 무엇이든 충돌


def test_different_farms_same_alleys_do_not_conflict_and_get_is_farm_scoped(client, app):
    """I5·I6 — 다른 farm(별개 과수원)의 같은 통로 번호는 무관하다. GET
    /alley-locks 는 list_missions 와 같은 관례로 조회자의 farm 범위만 본다."""
    fa, fb = _seed_operator(client)
    app.state.fleet.feed("scout01", "tel/state", {})   # farm A
    app.state.fleet.feed("scout02", "tel/state", {})   # farm B
    csrf_admin = do_login(client)
    h_admin = {"X-CSRF": csrf_admin}
    r1 = client.post("/api/v1/missions", headers=h_admin,
                     json={"robot_id": "scout01", "alleys": [0, 1]})
    r2 = client.post("/api/v1/missions", headers=h_admin,
                     json={"robot_id": "scout02", "alleys": [0, 1]})
    assert r1.json()["state"] == "QUEUED"
    assert r2.json()["state"] == "QUEUED"           # 다른 farm — 충돌 아님
    assert len(client.get("/api/v1/alley-locks", headers=h_admin).json()) == 2

    csrf_op = do_login(client, "op", "oppw")         # op 는 farm A 만
    locks_op = client.get("/api/v1/alley-locks", headers={"X-CSRF": csrf_op}).json()
    assert len(locks_op) == 1
    assert locks_op[0]["robot_id"] == "scout01"


def test_second_request_for_locked_robot_gets_409_not_another_queued_lock(client, app):
    """I7 — 같은 로봇에 QUEUED_LOCK 이 이미 있으면 새 요청은(통로가 뭐든) 기존
    활성 임무 409 로 막혀야 한다 — 아니면 QUEUED_LOCK 이 계속 쌓인다."""
    fa, _ = _seed_operator(client)
    _add_robot(client, "scout03", fa["id"])
    app.state.fleet.feed("scout01", "tel/state", {})
    app.state.fleet.feed("scout03", "tel/state", {})
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    client.post("/api/v1/missions", headers=h, json={"robot_id": "scout01", "alleys": [0, 1]})
    r2 = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout03", "alleys": [1, 2]})
    assert r2.json()["state"] == "QUEUED_LOCK"

    r3 = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout03", "alleys": [5, 6]})   # scout01 과 안 겹침
    assert r3.status_code == 409, r3.text

    from fleet_server.models import Mission
    with app.state.session_factory() as db:
        n = db.query(Mission).filter(Mission.robot_id == "scout03").count()
        assert n == 1                              # 두 번째 QUEUED_LOCK 이 쌓이지 않았다


def test_non_adjacent_or_empty_alleys_rejected_400(client, app):
    """M11 — pads() 는 정렬 후 연속쌍만 보므로, 요청 순서상 인접하지 않은
    통로가 섞이면([0,2,4]) 실제 헤드랜드 패드가 잠금 계산에서 빠진다. REST
    입력 검증으로 400 거부한다(로봇 계약 변경 아님). 빈 목록도 여기서 막힌다."""
    _seed_operator(client)
    app.state.fleet.feed("scout01", "tel/state", {})
    csrf = do_login(client, "op", "oppw")
    h = {"X-CSRF": csrf}
    r1 = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout01", "alleys": [0, 2, 4]})
    assert r1.status_code == 400
    r2 = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout01", "alleys": []})
    assert r2.status_code == 400


def test_robot_rejects_mission_start_fails_mission_and_releases_lock(client, app):
    """I4 — 로봇이 mission_start 자체를 거부하면(BUSY 등) 서버 임무는 로봇의
    "running" 보고를 영영 못 받아 QUEUED 에 멈추고, AlleyLock 도 함께
    고착된다. cmd_id(f"m{id}")로 상관을 잡아 FAILED 로 종착시키고 잠금을
    해제해야 한다."""
    _seed_operator(client)
    app.state.fleet.feed("scout01", "tel/state", {})
    csrf = do_login(client, "op", "oppw")
    h = {"X-CSRF": csrf}
    ms = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout01", "alleys": [0, 1]}).json()
    assert ms["state"] == "QUEUED"
    assert client.get("/api/v1/alley-locks", headers=h).json()      # 잠겨 있다

    app.state.fleet.feed("scout01", "evt", {
        "kind": "cmd_result", "cmd_id": f"m{ms['id']}", "cmd": "mission_start",
        "status": "rejected", "code": "BUSY", "data": {"reason": "임무 진행 중"}})

    from fleet_server.models import Mission
    with app.state.session_factory() as db:
        assert db.get(Mission, ms["id"]).state == "FAILED"
    assert client.get("/api/v1/alley-locks", headers=h).json() == []


def test_cancel_offline_running_mission_succeeds_locally_and_releases_lock(client, app):
    """C3 — 오프라인 로봇의 RUNNING 임무도 cancel 은 로컬 전이로 허용해야
    한다. 아니면 통로가 영구히 잠기고, 재기동 restore() 가 그 좀비 잠금을
    되살린다."""
    _seed_operator(client)
    app.state.fleet.feed("scout01", "tel/state", {})
    csrf = do_login(client, "op", "oppw")
    h = {"X-CSRF": csrf}
    ms = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout01", "alleys": [0, 1]}).json()
    app.state.fleet.feed("scout01", "mission", {"state": "running"})   # QUEUED -> RUNNING
    app.state.fleet.presence.touch("scout01", t=0.0)                   # 강제 오프라인

    r = client.post(f"/api/v1/missions/{ms['id']}/cancel", headers=h)
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["state"] == "CANCELED"
    assert body["delivery"] == "not_sent"
    assert client.get("/api/v1/alley-locks", headers=h).json() == []


def test_verb_offline_pause_still_409_state_unchanged(client, app):
    """cancel 과 달리(C3) pause 는 오프라인이면 여전히 409 로 막힌다 — 로컬
    전이 허용은 통로 잠금 좀비를 막기 위한 cancel 만의 특례다."""
    _seed_operator(client)
    app.state.fleet.feed("scout01", "tel/state", {})
    csrf = do_login(client, "op", "oppw")
    h = {"X-CSRF": csrf}
    ms = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout01", "alleys": [0]}).json()
    app.state.fleet.feed("scout01", "mission", {"state": "running"})
    app.state.fleet.presence.touch("scout01", t=0.0)
    r = client.post(f"/api/v1/missions/{ms['id']}/pause", headers=h)
    assert r.status_code == 409
    from fleet_server.models import Mission
    with app.state.session_factory() as db:
        assert db.get(Mission, ms["id"]).state == "RUNNING"   # 상태 불변


class _SlowFleetPort:
    """C2 — 실제 어댑터의 ws 전송 await 지점을 흉내낸다. InMemoryFleetPort 를
    감싸(합성) send_command 에만 진짜 양보 지점(asyncio.sleep)을 끼운다 —
    그래야 두 요청의 동시성이 이벤트루프 수준에서 재현된다(가짜 send_command
    가 진짜로 아무것도 await 하지 않으면 코루틴이 절대 양보하지 않아 두
    요청이 사실상 순차 실행되고, 경합 자체가 재현되지 않는다)."""

    def __init__(self):
        from fleet_server.fleet.port import InMemoryFleetPort
        self._inner = InMemoryFleetPort()

    def __getattr__(self, name):
        return getattr(self._inner, name)

    async def send_command(self, robot_id, cmd_id, action, payload):
        await asyncio.sleep(0.02)
        return await self._inner.send_command(robot_id, cmd_id, action, payload)


class _RaceOnVerbPort:
    """검증 동사(pause/resume/cancel)의 await 구간에 로봇 보고가 끼어드는 상황.

    send_command 안에서 콜백을 불러 **다른 세션이 임무를 먼저 종착**시킨다 —
    실기에서는 링크에 명령을 쓰는 동안 도착한 mission_done/cmd_result 가
    FleetService 세션으로 그 일을 한다."""

    def __init__(self):
        from fleet_server.fleet.port import InMemoryFleetPort
        self._inner = InMemoryFleetPort()
        self.on_verb = None

    def __getattr__(self, name):
        return getattr(self._inner, name)

    async def send_command(self, robot_id, cmd_id, action, payload):
        if self.on_verb is not None and action != "mission_start":
            self.on_verb()
        return await self._inner.send_command(robot_id, cmd_id, action, payload)


def test_verb_losing_race_returns_409_not_500(tmp_path):
    """I-1 — await 구간에 임무가 먼저 종착하면 재개된 apply 가 InvalidTransition 을
    던진다. 그것이 그대로 올라가면 조작자에게 500 이 뜬다(서버 오류처럼 보이지만
    실제로는 '이미 끝난 임무'라는 정상적인 경합 결과다) — 409 여야 한다."""
    from fleet_server import missions
    from fleet_server.app import create_app
    from fleet_server.models import Mission

    fleet = _RaceOnVerbPort()
    app = create_app(_test_settings(db_url=f"sqlite:///{tmp_path}/race.db"), fleet=fleet)
    client = TestClient(app)
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    client.post("/api/v1/robots", headers=h,
                json={"id": "scout01", "farm_id": fa["id"], "name": "r"})
    fleet._inner.feed("scout01", "tel/state", {})
    ms = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout01", "alleys": [0]}).json()

    def _robot_finishes_first():                # 다른 세션 — 로봇 보고 경로를 흉내
        with app.state.session_factory() as db:
            row = db.get(Mission, ms["id"])
            missions.apply(db, row, "start")
            missions.apply(db, row, "complete")

    fleet.on_verb = _robot_finishes_first
    r = client.post(f"/api/v1/missions/{ms['id']}/cancel", headers=h)
    assert r.status_code == 409, r.text
    with app.state.session_factory() as db:
        assert db.get(Mission, ms["id"]).state == "DONE"   # 먼저 온 종착이 이긴다


async def _alogin(ac, login="admin", pw="admpw"):
    r = await ac.post("/api/v1/auth/login", json={"login": login, "password": pw})
    assert r.status_code == 200, r.text
    return r.json()["csrf"]


async def test_concurrent_overlapping_requests_never_both_acquire_lock(tmp_path):
    """C2 — 잠금을 발진(await send_command) 전에 커밋해 두지 않으면, 겹치는
    통로를 요청하는 두 REST 호출이 서로의(미커밋) 잠금을 못 보고 둘 다
    획득할 수 있다. 인메모리(sqlite://, StaticPool 단일 커넥션)는 안 쓴다 —
    FastAPI 는 동기 의존성(get_db)을 스레드풀에서 돌리므로(run_in_threadpool)
    두 요청이 실제로 별개 OS 스레드에서 같은 커넥션을 두드리게 돼 SQLite
    자체가 아니라 파이썬 드라이버 레벨에서 깨진다. 파일 DB(커넥션 풀 — 요청당
    별개 커넥션)를 써야 운영과 같은 방식으로 SQLite 의 실제 쓰기 직렬화가
    경합을 재현한다."""
    from fleet_server.app import create_app

    fleet = _SlowFleetPort()
    app = create_app(_test_settings(db_url=f"sqlite:///{tmp_path}/c2.db"), fleet=fleet)
    transport = httpx.ASGITransport(app=app)
    async with httpx.AsyncClient(transport=transport, base_url="http://test") as ac:
        csrf = await _alogin(ac)
        h = {"X-CSRF": csrf}
        fa = (await ac.post("/api/v1/farms", json={"name": "농장A"}, headers=h)).json()
        await ac.post("/api/v1/robots", headers=h,
                      json={"id": "scout01", "farm_id": fa["id"], "name": "r1"})
        await ac.post("/api/v1/robots", headers=h,
                      json={"id": "scout02", "farm_id": fa["id"], "name": "r2"})
        fleet._inner.feed("scout01", "tel/state", {})
        fleet._inner.feed("scout02", "tel/state", {})

        r1, r2 = await asyncio.gather(
            ac.post("/api/v1/missions", headers=h,
                   json={"robot_id": "scout01", "alleys": [0, 1]}),
            ac.post("/api/v1/missions", headers=h,
                   json={"robot_id": "scout02", "alleys": [1, 2]}),
        )
    assert r1.status_code == 200 and r2.status_code == 200, (r1.text, r2.text)
    states = sorted([r1.json()["state"], r2.json()["state"]])
    assert states == ["QUEUED", "QUEUED_LOCK"]      # 둘 다 QUEUED 인 이중획득은 없다


def test_verb_offline_409_state_unchanged(client, app):
    """QUEUED 에서 pause 는애초에 지원하지 않는 전이라(TRANSITIONS 사전
    검사) 오프라인 여부와 무관하게 409 다 — 상태 불변도 함께 확인한다.
    오프라인 자체의 분기(cancel 은 로컬 허용, 다른 동사는 409)는 아래
    "리뷰 라운드 1 — C3" 절의 두 테스트가 맡는다."""
    _seed_operator(client)
    app.state.fleet.feed("scout01", "tel/state", {})
    csrf = do_login(client, "op", "oppw")
    h = {"X-CSRF": csrf}
    ms = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout01", "alleys": [0]}).json()
    r = client.post(f"/api/v1/missions/{ms['id']}/pause", headers=h)
    assert r.status_code == 409
    from fleet_server.models import Mission
    with app.state.session_factory() as db:
        assert db.get(Mission, ms["id"]).state == "QUEUED"   # 상태 불변
