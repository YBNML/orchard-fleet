import datetime as dt

from fleet_server import ingest
from fleet_server import models as m
from tests.conftest import do_login


def _farm_robot(db):
    f = m.Farm(name="농장A"); db.add(f); db.flush()
    db.add(m.Robot(id="scout01", farm_id=f.id, name="r")); db.commit()
    return f


def test_event_dedup(db):
    _farm_robot(db)
    assert ingest.event(db, "scout01", "evt", 5, {"kind": "estop", "msg": "x"}) is True
    assert ingest.event(db, "scout01", "evt", 5, {"kind": "estop", "msg": "재전송"}) is False
    assert db.query(m.Event).count() == 1                      # 스펙 §3.3 중복 제거


def test_track_downsample(db):
    _farm_robot(db)
    t0 = dt.datetime(2026, 8, 1, 12, 0, 0, tzinfo=dt.UTC)
    assert ingest.track(db, "scout01", {"x": 0, "y": 0, "yaw": 0, "ts": t0.timestamp()}) is True
    assert ingest.track(db, "scout01", {"x": 1, "y": 0, "yaw": 0,
                                        "ts": t0.timestamp() + 0.2}) is False   # 1 Hz 미만
    assert ingest.track(db, "scout01", {"x": 2, "y": 0, "yaw": 0,
                                        "ts": t0.timestamp() + 1.1}) is True
    assert db.query(m.Track).count() == 2


def test_history_routes(client, app):
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    client.post("/api/v1/robots", headers=h,
                json={"id": "scout01", "farm_id": fa["id"], "name": "r"})
    with app.state.session_factory() as db:
        ingest.event(db, "scout01", "evt", 1, {"kind": "estop", "msg": "정지"})
        ingest.track(db, "scout01", {"x": 1.5, "y": 2.5, "yaw": 0.1, "ts": 1000.0})
    evs = client.get("/api/v1/events?robot_id=scout01").json()
    assert evs and evs[0]["kind"] == "estop"
    trs = client.get("/api/v1/tracks?robot_id=scout01").json()
    assert trs and trs[0]["x"] == 1.5
    audit_resp = client.get("/api/v1/audit")
    assert audit_resp.status_code == 200      # admin 은 가능

    # Critical 2 회귀 — 새 세션에서 재조회한 값(naive 로 돌아옴)도 tz 접미사가
    # 있어야 한다. 없으면 대시보드의 Date.parse() 가 KST 로 오해석해 9시간 어긋난다.
    def _has_tz(ts: str) -> bool:
        return ts.endswith("Z") or ts[-6] in "+-"

    assert _has_tz(evs[0]["ts"])
    assert _has_tz(trs[0]["ts"])
    assert audit_resp.json() and all(_has_tz(a["ts"]) for a in audit_resp.json())


def test_audit_admin_only(client):
    fa_csrf = do_login(client)
    client.post("/api/v1/users", headers={"X-CSRF": fa_csrf}, json={
        "login": "obs", "password": "obspw", "role": "observer", "farm_ids": []})
    do_login(client, "obs", "obspw")
    assert client.get("/api/v1/audit").status_code == 403
