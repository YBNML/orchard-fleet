from __future__ import annotations

import time


class PresenceRegistry:
    """마지막 수신 시각 기반 온라인 판정 — 15초(스펙 §3.1)."""

    def __init__(self, offline_after_s: float = 15.0):
        self.offline_after_s = offline_after_s
        self._last: dict[str, float] = {}

    def touch(self, robot_id: str, t: float | None = None) -> None:
        self._last[robot_id] = time.time() if t is None else t

    def last_seen(self, robot_id: str) -> float | None:
        return self._last.get(robot_id)

    def online(self, robot_id: str, t: float | None = None) -> bool:
        ls = self._last.get(robot_id)
        if ls is None:
            return False
        now = time.time() if t is None else t
        return (now - ls) <= self.offline_after_s
