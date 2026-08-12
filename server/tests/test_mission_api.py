from tests.conftest import do_login


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


def test_verb_offline_409_state_unchanged(client, app):
    _seed_operator(client)
    app.state.fleet.feed("scout01", "tel/state", {})
    csrf = do_login(client, "op", "oppw")
    h = {"X-CSRF": csrf}
    ms = client.post("/api/v1/missions", headers=h,
                     json={"robot_id": "scout01", "alleys": [0]}).json()
    # 강제로 오프라인 (마지막 수신 시각을 과거로)
    app.state.fleet.presence.touch("scout01", t=0.0)
    r = client.post(f"/api/v1/missions/{ms['id']}/cancel", headers=h)
    assert r.status_code == 409
    from fleet_server.models import Mission
    with app.state.session_factory() as db:
        assert db.get(Mission, ms["id"]).state == "QUEUED"   # 상태 불변
