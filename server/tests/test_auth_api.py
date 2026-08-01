import time

from fastapi.testclient import TestClient

from fleet_server.app import create_app
from tests.conftest import _test_settings, do_login


def test_bootstrap_admin_and_me(client):
    csrf = do_login(client)
    r = client.get("/api/v1/auth/me")
    assert r.status_code == 200
    assert r.json()["user"]["role"] == "admin"
    assert r.json()["csrf"] == csrf


def test_login_failure_delayed_and_401():
    app = create_app(_test_settings(login_delay_s=0.3))
    c = TestClient(app)
    t0 = time.monotonic()
    r = c.post("/api/v1/auth/login", json={"login": "admin", "password": "오답"})
    assert r.status_code == 401
    assert time.monotonic() - t0 >= 0.25          # 실패 지연 (스펙 §5)


def test_cookie_flags(client):
    r = client.post("/api/v1/auth/login", json={"login": "admin", "password": "admpw"})
    sc = r.headers["set-cookie"].lower()
    assert "httponly" in sc and "samesite=strict" in sc


def test_me_without_session_401(client):
    assert client.get("/api/v1/auth/me").status_code == 401


def test_logout_requires_csrf(client):
    csrf = do_login(client)
    assert client.post("/api/v1/auth/logout").status_code == 403          # 헤더 없음
    r = client.post("/api/v1/auth/logout", headers={"X-CSRF": csrf})
    assert r.status_code == 200
    assert client.get("/api/v1/auth/me").status_code == 401               # 세션 소멸


def test_logout_recorded_in_audit(client, app):
    from fleet_server.models import AuditLog

    csrf = do_login(client)
    r = client.post("/api/v1/auth/logout", headers={"X-CSRF": csrf})
    assert r.status_code == 200
    with app.state.session_factory() as db:
        rows = db.query(AuditLog).filter(AuditLog.action == "logout").all()
        assert rows and rows[-1].result == "accepted" and rows[-1].target == "admin"


def test_expired_session_rejected_and_deleted(client, app):
    import datetime as dt

    from fleet_server.models import AuthSession

    do_login(client)
    with app.state.session_factory() as db:
        row = db.query(AuthSession).one()
        row.expires_at = dt.datetime(2020, 1, 1)      # tz-naive 과거 (SQLite 가 벗긴 형태)
        db.commit()
    assert client.get("/api/v1/auth/me").status_code == 401
    with app.state.session_factory() as db:
        assert db.query(AuthSession).count() == 0      # 만료 행 삭제 확인
