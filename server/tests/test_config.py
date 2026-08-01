import os
from fleet_server.config import Settings, load_settings


def test_defaults():
    s = Settings()
    assert s.db_url.startswith("sqlite")
    assert s.offline_after_s == 15.0
    assert s.login_delay_s == 0.5


def test_env_override(monkeypatch):
    monkeypatch.setenv("FLEET_DB_URL", "sqlite:///x.db")
    monkeypatch.setenv("FLEET_ALLOWED_ORIGINS", "http://a:8000, http://b:8000")
    s = load_settings()
    assert s.db_url == "sqlite:///x.db"
    assert s.allowed_origins == ["http://a:8000", "http://b:8000"]
