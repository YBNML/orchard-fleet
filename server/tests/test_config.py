import os
from fleet_server.config import Settings, load_settings


def test_defaults():
    s = Settings()
    assert s.db_url.startswith("sqlite")
    assert s.offline_after_s == 15.0
    assert s.login_delay_s == 0.5
    assert s.event_ttl_days == 7                # Task 6 — 이벤트 보존정책
    assert s.event_ttl_safe_days == 90           # T6 리뷰 I2 — 안전 kind 는 더 길게


def test_env_override(monkeypatch):
    monkeypatch.setenv("FLEET_DB_URL", "sqlite:///x.db")
    monkeypatch.setenv("FLEET_ALLOWED_ORIGINS", "http://a:8000, http://b:8000")
    monkeypatch.setenv("FLEET_EVENT_TTL_DAYS", "3")
    monkeypatch.setenv("FLEET_EVENT_TTL_SAFE_DAYS", "30")
    s = load_settings()
    assert s.db_url == "sqlite:///x.db"
    assert s.allowed_origins == ["http://a:8000", "http://b:8000"]
    assert s.event_ttl_days == 3
    assert s.event_ttl_safe_days == 30
