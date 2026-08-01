from starlette.testclient import TestClient

from fleet_server.app import create_app
from fleet_server.fleet.port import InMemoryFleetPort
from tests.conftest import _test_settings, do_login


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


def test_robot_out_last_seen_naive_gets_tz_suffix():
    """SQLite 왕복 후 naive 로 돌아온 last_seen 도 UTC 접미사가 붙어야 한다
    (KST 재생 어긋남 회귀, Critical 2)."""
    import datetime as dt

    from fleet_server.api.admin_routes import _robot_out
    from fleet_server.models import Robot

    r = Robot(id="scout01", farm_id=1, name="r")
    r.last_seen = dt.datetime(2026, 8, 1, 3, 0, 0)   # naive
    out = _robot_out(r, admin=True)
    assert out["last_seen"] == "2026-08-01T03:00:00+00:00"


class _SpyFleetPort(InMemoryFleetPort):
    """PATCH /robots 가 실제로 unregister→register 순서로 재배선하는지 확인하려고
    호출 자체를 기록한다 (InMemoryFleetPort.register_robot 은 무조건 덮어쓰기라
    재배선 여부를 구분 못 함 — LegacyFleetPort 는 이미 등록된 id 면 조기 반환하므로
    unregister 없이는 실제로 재배선되지 않는다)."""

    def __init__(self):
        super().__init__()
        self.unregister_calls: list[str] = []

    def unregister_robot(self, robot_id):
        self.unregister_calls.append(robot_id)
        super().unregister_robot(robot_id)


def test_patch_robot_rewires_connection_on_config_change():
    fleet = _SpyFleetPort()
    app = create_app(_test_settings(), fleet=fleet)
    client = TestClient(app)
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    client.post("/api/v1/robots", headers=h, json={
        "id": "scout01", "farm_id": fa["id"], "name": "r",
        "config_json": {"ws_url": "ws://a/ws", "token": "T1"}})

    r = client.patch("/api/v1/robots/scout01", headers=h,
                     json={"config_json": {"ws_url": "ws://b/ws", "token": "T2"}})
    assert r.status_code == 200, r.text
    assert fleet.unregister_calls == ["scout01"]              # 재배선 발생
    assert fleet.robots["scout01"]["config"]["token"] == "T2"

    # 재배선을 유발하지 않는 변경(name 만)은 unregister 를 부르지 않는다
    r2 = client.patch("/api/v1/robots/scout01", headers=h, json={"name": "새이름"})
    assert r2.status_code == 200, r2.text
    assert fleet.unregister_calls == ["scout01"]              # 변화 없음


def test_patch_robot_unknown_farm_404():
    fleet = _SpyFleetPort()
    app = create_app(_test_settings(), fleet=fleet)
    client = TestClient(app)
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    client.post("/api/v1/robots", headers=h,
                json={"id": "scout01", "farm_id": fa["id"], "name": "r"})
    r = client.patch("/api/v1/robots/scout01", headers=h, json={"farm_id": 9999})
    assert r.status_code == 404
    assert fleet.unregister_calls == []                       # 실패했으니 재배선도 없음


def test_create_and_patch_user_reject_unknown_farm_ids(client):
    """foreign_keys=ON 이후 존재하지 않는 farm_id 로 UserFarm 을 만들면
    IntegrityError(500) 가 나갈 수 있다 — API 단에서 400/404 로 미리 막는다."""
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    r = client.post("/api/v1/users", headers=h, json={
        "login": "u1", "password": "pw12345", "role": "observer", "farm_ids": [9999]})
    assert r.status_code == 404

    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    ok = client.post("/api/v1/users", headers=h, json={
        "login": "u2", "password": "pw12345", "role": "observer", "farm_ids": [fa["id"]]})
    assert ok.status_code == 200
    uid = ok.json()["id"]

    bad_patch = client.patch(f"/api/v1/users/{uid}", headers=h, json={"farm_ids": [9999]})
    assert bad_patch.status_code == 404


def test_patch_user_rejects_atomically_no_partial_mutation(client):
    """farm_ids 검증 실패 시, 같은 요청에 함께 들어온 role 변경도 반영되면 안 된다
    (audit.record() 자체가 commit 을 해서, 검증이 mutation 뒤에 있으면 부분 반영이
    새 버그로 새어나갈 수 있었다)."""
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    u = client.post("/api/v1/users", headers=h, json={
        "login": "u3", "password": "pw12345", "role": "observer",
        "farm_ids": [fa["id"]]}).json()
    r = client.patch(f"/api/v1/users/{u['id']}", headers=h,
                     json={"role": "admin", "farm_ids": [9999]})
    assert r.status_code == 404
    users = client.get("/api/v1/users", headers=h).json()
    row = next(x for x in users if x["id"] == u["id"])
    assert row["role"] == "observer"          # role 변경이 새어나가지 않음
