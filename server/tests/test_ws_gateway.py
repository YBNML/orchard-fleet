import pytest

from fleet_server.models import AuditLog
from tests.conftest import do_login

ORIGIN = {"origin": "http://testserver"}


def _seed(client):
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    fb = client.post("/api/v1/farms", json={"name": "농장B"}, headers=h).json()
    client.post("/api/v1/robots", headers=h, json={"id": "r1", "farm_id": fa["id"], "name": "r1"})
    client.post("/api/v1/robots", headers=h, json={"id": "r2", "farm_id": fa["id"], "name": "r2"})
    client.post("/api/v1/robots", headers=h, json={"id": "rb", "farm_id": fb["id"], "name": "rb"})
    client.post("/api/v1/users", headers=h, json={
        "login": "obs", "password": "obspw", "role": "observer", "farm_ids": [fa["id"]]})
    client.post("/api/v1/users", headers=h, json={
        "login": "op", "password": "oppw", "role": "operator", "farm_ids": [fa["id"]]})
    return fa, fb


def test_origin_rejected(client):
    _seed(client)
    do_login(client, "obs", "obspw")
    with pytest.raises(Exception):
        with client.websocket_connect("/ws", headers={"origin": "http://evil"}) as ws:
            ws.receive_json()


def test_no_session_rejected(client):
    _seed(client)
    client.cookies.clear()
    with pytest.raises(Exception):
        with client.websocket_connect("/ws", headers=ORIGIN) as ws:
            ws.receive_json()


def test_telemetry_fanout_scoped(client, app):
    _seed(client)
    do_login(client, "obs", "obspw")
    with client.websocket_connect("/ws", headers=ORIGIN) as ws:
        assert ws.receive_json()["type"] == "ready"
        app.state.fleet.feed("r1", "tel/state", {"x": 5.0})
        msg = ws.receive_json()
        assert msg["topic"].endswith("/r1/tel/state") and msg["payload"]["x"] == 5.0
        # 타 농장(rb) 텔레메트리는 오지 않는다 — r1 것만 한 번 더 확인
        app.state.fleet.feed("rb", "tel/state", {"x": 9.0})
        app.state.fleet.feed("r1", "tel/health", {"ok": 1})
        assert ws.receive_json()["topic"].endswith("/r1/tel/health")


def test_observer_estop_allowed_teleop_denied(client, app):
    _seed(client)
    app.state.fleet.feed("r1", "tel/state", {})
    do_login(client, "obs", "obspw")
    with client.websocket_connect("/ws", headers=ORIGIN) as ws:
        ws.receive_json()
        ws.send_json({"type": "cmd", "action": "estop", "robot": "r1", "cmd_id": "c1"})
        r = ws.receive_json()
        assert r == {"type": "cmd_result", "robot": "r1", "cmd_id": "c1", "result": "sent"}
        assert app.state.fleet.sent[-1][2] == "estop"          # D9
        ws.send_json({"type": "teleop", "robot": "r1", "payload": {"vx": 0.3, "wz": 0}})
        assert ws.receive_json()["type"] == "denied"
    with app.state.session_factory() as db:
        acts = [(a.action, a.result) for a in db.query(AuditLog)]
        assert ("estop", "accepted") in acts and ("teleop", "rejected") in acts


def test_mission_via_ws_denied(client, app):
    _seed(client)
    app.state.fleet.feed("r1", "tel/state", {})
    do_login(client, "op", "oppw")
    with client.websocket_connect("/ws", headers=ORIGIN) as ws:
        ws.receive_json()
        ws.send_json({"type": "cmd", "action": "mission_start", "robot": "r1", "cmd_id": "c2"})
        r = ws.receive_json()
        assert r["type"] == "denied" and "REST" in r["reason"]


def test_stop_all_partial(client, app):
    _seed(client)
    app.state.fleet.feed("r1", "tel/state", {})               # r1 만 온라인
    do_login(client, "obs", "obspw")
    with client.websocket_connect("/ws", headers=ORIGIN) as ws:
        ws.receive_json()
        ws.send_json({"type": "cmd", "action": "stop_all", "cmd_id": "c3"})
        r = ws.receive_json()
        assert r["type"] == "stop_all_result"
        assert r["results"] == {"r1": "sent", "r2": "offline"}  # 부분 실패를 숨기지 않는다


def test_teleop_session_audit_once(client, app):
    _seed(client)
    app.state.fleet.feed("r1", "tel/state", {})
    do_login(client, "op", "oppw")
    with client.websocket_connect("/ws", headers=ORIGIN) as ws:
        ws.receive_json()
        for _ in range(3):
            ws.send_json({"type": "teleop", "robot": "r1", "payload": {"vx": 0.2, "wz": 0}})
        ws.send_json({"type": "cmd", "action": "ping", "robot": "r1", "cmd_id": "p"})
        ws.receive_json()                                      # ping 응답까지 대기(순서 보장)
    teleops = [s for s in app.state.fleet.sent if s[2] == "teleop"]
    assert len(teleops) == 3                                   # 지령은 전부 전달
    with app.state.session_factory() as db:
        rows = db.query(AuditLog).filter(AuditLog.action == "teleop_session").all()
        assert len(rows) == 1                                  # 감사는 세션 단위 (S7)
