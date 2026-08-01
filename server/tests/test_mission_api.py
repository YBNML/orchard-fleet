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
