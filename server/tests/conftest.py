import pytest
from fastapi.testclient import TestClient

from fleet_server.app import create_app
from fleet_server.config import Settings
from fleet_server.db import Base, make_engine, make_session_factory


@pytest.fixture()
def db():
    engine = make_engine("sqlite://")          # in-memory
    Base.metadata.create_all(engine)
    factory = make_session_factory(engine)
    with factory() as session:
        yield session


def _test_settings(**kw) -> Settings:
    base = dict(db_url="sqlite://", session_secret="테스트비밀",
                login_delay_s=0.0, admin_login="admin", admin_password="admpw",
                allowed_origins=["http://testserver"])
    base.update(kw)
    return Settings(**base)


@pytest.fixture()
def app():
    return create_app(_test_settings())


@pytest.fixture()
def client(app):
    return TestClient(app)


def do_login(client, login="admin", pw="admpw") -> str:
    """로그인하고 CSRF 토큰을 돌려준다 (쿠키는 client 가 유지)."""
    r = client.post("/api/v1/auth/login", json={"login": login, "password": pw})
    assert r.status_code == 200, r.text
    return r.json()["csrf"]
