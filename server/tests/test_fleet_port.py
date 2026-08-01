import pytest

from fleet_server.fleet.port import InMemoryFleetPort
from fleet_server.fleet.presence import PresenceRegistry
from tests.conftest import do_login


def test_presence_15s_boundary():
    p = PresenceRegistry(offline_after_s=15.0)
    assert p.online("r1", t=100.0) is False          # 한 번도 못 봄
    p.touch("r1", t=100.0)
    assert p.online("r1", t=114.9) is True
    assert p.online("r1", t=115.1) is False          # 15초 초과 → 오프라인 (스펙 §3.1)


@pytest.mark.asyncio
async def test_offline_command_fails_immediately():
    fp = InMemoryFleetPort()
    fp.register_robot("scout01", 1, "legacy_ws", {})
    assert await fp.send_command("scout01", "c1", "estop", {}) == "offline"
    fp.feed("scout01", "tel/state", {"x": 0})
    assert await fp.send_command("scout01", "c2", "estop", {}) == "sent"
    assert fp.sent[-1][2] == "estop"


def test_telemetry_handler_called():
    fp = InMemoryFleetPort()
    got = []
    fp.set_telemetry_handler(lambda r, ch, pl, seq: got.append((r, ch, seq)))
    fp.feed("scout01", "evt", {"kind": "estop"}, seq=3)
    assert got == [("scout01", "evt", 3)]


def test_status_route(client, app):
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    client.post("/api/v1/robots", headers=h,
                json={"id": "scout01", "farm_id": fa["id"], "name": "r"})
    r = client.get("/api/v1/robots/scout01/status")
    assert r.status_code == 200 and r.json()["online"] is False
    app.state.fleet.feed("scout01", "tel/state", {})
    assert client.get("/api/v1/robots/scout01/status").json()["online"] is True
