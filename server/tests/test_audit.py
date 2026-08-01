from fleet_server import audit
from fleet_server.models import AuditLog
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
