"""이관됨 → robomw.features.telemetry_map (스펙 ①, 2026-08-11). 이 shim 은
기존 스크립트 호환용 — 새 코드는 robomw 를 직접 import 할 것.

점군 구독·TF 는 기능에서 빠져 어댑터(orchard_sim.adapters.ros_cloud)로 갔다 —
기능은 map 프레임 점 배열만 받는다."""
from robomw.features.telemetry_map import *              # noqa: F401,F403
from robomw.features.telemetry_map import TelemetryMap   # noqa: F401
