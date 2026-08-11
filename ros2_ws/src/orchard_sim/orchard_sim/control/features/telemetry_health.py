"""이관됨 → robomw.features.telemetry_health (스펙 ①, 2026-08-11). 이 shim 은
기존 스크립트 호환용 — 새 코드는 robomw 를 직접 import 할 것."""
from robomw.features.telemetry_health import *                 # noqa: F401,F403
from robomw.features.telemetry_health import TelemetryHealth   # noqa: F401
