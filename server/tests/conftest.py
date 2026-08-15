import pytest
from fastapi.testclient import TestClient

from fleet_server.app import create_app
from fleet_server.config import Settings
from fleet_server.db import Base, make_engine, make_session_factory


@pytest.fixture(autouse=True)
def _reset_ingest_state():
    from fleet_server import ingest
    ingest._last_track_ts.clear()


@pytest.fixture()
def db():
    engine = make_engine("sqlite://")          # in-memory
    Base.metadata.create_all(engine)
    factory = make_session_factory(engine)
    with factory() as session:
        yield session


def _test_settings(**kw) -> Settings:
    from pathlib import Path
    base = dict(db_url="sqlite://", session_secret="테스트비밀",
                login_delay_s=0.0, admin_login="admin", admin_password="admpw",
                allowed_origins=["http://testserver"],
                # farm 은 기본적으로 없는 것으로 둔다(Task 5) — 저장소의 실제
                # maps/orchard_real/farm.json 을 조용히 끌어와 무관한 테스트가
                # 그 내용(rows·terrain 등)에 우연히 결합되는 일을 막는다.
                # farm 이 필요한 테스트는 test_farm_routes.py 에서 명시적으로
                # farm_manifest_path 를 넘긴다.
                farm_manifest_path=Path("__no_such_farm_manifest__.json"))
    base.update(kw)
    return Settings(**base)


@pytest.fixture()
def app():
    from fleet_server.fleet.port import InMemoryFleetPort
    return create_app(_test_settings(), fleet=InMemoryFleetPort())


@pytest.fixture()
def client(app):
    return TestClient(app)


def do_login(client, login="admin", pw="admpw") -> str:
    """로그인하고 CSRF 토큰을 돌려준다 (쿠키는 client 가 유지)."""
    r = client.post("/api/v1/auth/login", json={"login": login, "password": pw})
    assert r.status_code == 200, r.text
    return r.json()["csrf"]
