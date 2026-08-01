from tests.conftest import do_login


def _seed(client):
    """admin 으로 농장 2·로봇 1·observer 1 을 만든다. (A농장만 배정)"""
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    fb = client.post("/api/v1/farms", json={"name": "농장B"}, headers=h).json()
    client.post("/api/v1/robots", headers=h, json={
        "id": "scout01", "farm_id": fa["id"], "name": "스카우트1",
        "config_json": {"ws_url": "ws://127.0.0.1:8080/ws", "token": "RTOK"}})
    client.post("/api/v1/users", headers=h, json={
        "login": "obs", "password": "obspw", "role": "observer",
        "farm_ids": [fa["id"]]})
    return fa, fb


def test_crud_and_scope(client):
    fa, fb = _seed(client)
    # observer 는 자기 스코프 농장만 본다
    do_login(client, "obs", "obspw")
    farms = client.get("/api/v1/farms").json()
    assert [f["name"] for f in farms] == ["농장A"]
    robots = client.get("/api/v1/robots").json()
    assert [r["id"] for r in robots] == ["scout01"]
    # 타 농장 조회는 403
    assert client.get(f"/api/v1/robots?farm_id={fb['id']}").status_code == 403


def test_write_requires_admin(client):
    _seed(client)
    csrf = do_login(client, "obs", "obspw")
    r = client.post("/api/v1/farms", json={"name": "몰래"}, headers={"X-CSRF": csrf})
    assert r.status_code == 403


def test_write_requires_csrf(client):
    do_login(client)
    assert client.post("/api/v1/farms", json={"name": "농장C"}).status_code == 403


def test_robot_token_not_exposed_to_observer(client):
    fa, _ = _seed(client)
    do_login(client, "obs", "obspw")
    robots = client.get("/api/v1/robots").json()
    assert "config_json" not in robots[0]      # 접속 토큰은 admin 전용 정보
