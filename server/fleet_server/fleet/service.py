"""텔레메트리 허브 — FleetPort 수신을 DB 수집·최신값 캐시·구독자 팬아웃으로.

mission 채널 payload {"state": ...} 는 임무 상태기계로 동기화한다
(로봇이 완료를 보고하면 서버 임무도 DONE 이 된다).
"""
from __future__ import annotations

from typing import Callable

from .. import ingest, missions
from ..models import Mission

_ROBOT_STATE_EVENT = {"running": "start", "paused": "pause", "done": "complete",
                      "canceled": "cancel", "failed": "fail"}


class FleetService:
    def __init__(self, session_factory):
        self._factory = session_factory
        self.latest: dict[str, dict[str, dict]] = {}
        self._subs: list[Callable[[str, str, dict], None]] = []

    def attach(self, fleet) -> None:
        fleet.set_telemetry_handler(self.on_telemetry)

    def subscribe(self, cb: Callable[[str, str, dict], None]):
        self._subs.append(cb)
        def unsub():
            if cb in self._subs:
                self._subs.remove(cb)
        return unsub

    def on_telemetry(self, robot_id: str, channel: str, payload: dict,
                     seq: int | None) -> None:
        self.latest.setdefault(robot_id, {})[channel] = payload
        if channel == "tel/state":
            with self._factory() as db:
                ingest.track(db, robot_id, payload)
        elif channel == "evt":
            with self._factory() as db:
                ingest.event(db, robot_id, channel, seq, payload)
        elif channel == "mission":
            self._sync_mission(robot_id, payload)
        for cb in list(self._subs):
            cb(robot_id, channel, payload)

    def _sync_mission(self, robot_id: str, payload: dict) -> None:
        ev = _ROBOT_STATE_EVENT.get(str(payload.get("state", "")).lower())
        if ev is None:
            return
        with self._factory() as db:
            ms = (db.query(Mission).filter(Mission.robot_id == robot_id,
                                           Mission.state.in_(["QUEUED", "RUNNING", "PAUSED"]))
                  .order_by(Mission.id.desc()).first())
            if ms is None:
                return
            try:
                missions.apply(db, ms, ev, payload=payload)
            except missions.InvalidTransition:
                pass                            # 이미 같은 상태 등 — 무시
