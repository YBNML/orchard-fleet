from fleet_server import audit
from fleet_server.models import AuditLog, User
from tests.conftest import do_login


def test_masking_and_clip(db):
    audit.record(db, action="cmd", result="rejected",
                 detail='{"token": "비밀토큰123", "password": "pw다"} ' + "X" * 500)
    row = db.query(AuditLog).one()
    assert "비밀토큰123" not in row.detail and "pw다" not in row.detail
    assert len(row.detail) <= 160


def test_login_failure_recorded(client, app):
    client.post("/api/v1/auth/login", json={"login": "admin", "password": "오답"})
    with app.state.session_factory() as db:
        rows = db.query(AuditLog).filter(AuditLog.action == "login").all()
        assert rows and rows[-1].result == "rejected"
        blob = " ".join(r.detail for r in rows)
        assert "오답" not in blob                      # 비밀번호 원문 금지


def test_farm_create_recorded(client, app):
    csrf = do_login(client)
    client.post("/api/v1/farms", json={"name": "감사농장"}, headers={"X-CSRF": csrf})
    with app.state.session_factory() as db:
        row = db.query(AuditLog).filter(AuditLog.action == "farm_create").one()
        assert row.result == "accepted" and "감사농장" in row.detail


def test_masking_multiword_secret(db):
    audit.record(db, action="cmd", result="rejected",
                 detail='{"password": "hello world", "token": "a b c"}')
    row = db.query(AuditLog).order_by(AuditLog.id.desc()).first()
    assert "hello" not in row.detail and "world" not in row.detail
    assert "a b c" not in row.detail


def test_farm_patch_records_actor(client, app):
    csrf = do_login(client)
    r = client.post("/api/v1/farms", json={"name": "행위자농장"}, headers={"X-CSRF": csrf})
    farm_id = r.json()["id"]
    client.patch(f"/api/v1/farms/{farm_id}", json={"map_bundle_ref": "ref1"},
                 headers={"X-CSRF": csrf})
    with app.state.session_factory() as db:
        admin_id = db.query(User).filter(User.login == "admin").one().id
        row = db.query(AuditLog).filter(AuditLog.action == "farm_patch").one()
        assert row.result == "accepted"
        assert row.user_id == admin_id and row.role == "admin"


def test_create_robot_rejected_recorded(client, app):
    csrf = do_login(client)
    client.post("/api/v1/robots",
                json={"id": "r1", "farm_id": 9999, "name": "없는농장로봇"},
                headers={"X-CSRF": csrf})
    with app.state.session_factory() as db:
        row = db.query(AuditLog).filter(AuditLog.action == "robot_create").one()
        assert row.result == "rejected"
