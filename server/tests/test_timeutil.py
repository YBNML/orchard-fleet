import datetime as dt

from fleet_server.timeutil import iso_utc


def test_naive_gets_utc_suffix():
    """SQLite 왕복 후처럼 naive 로 돌아온 값은 UTC 로 간주해 접미사를 붙인다."""
    naive = dt.datetime(2026, 8, 1, 3, 0, 0)
    assert iso_utc(naive) == "2026-08-01T03:00:00+00:00"


def test_aware_passthrough():
    aware = dt.datetime(2026, 8, 1, 3, 0, 0, tzinfo=dt.UTC)
    assert iso_utc(aware) == aware.isoformat()


def test_none_passthrough():
    assert iso_utc(None) is None
