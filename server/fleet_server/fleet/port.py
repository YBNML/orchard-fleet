from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable

from .presence import PresenceRegistry

TelemetryHandler = Callable[[str, str, dict, int | None], None]


@dataclass
class RobotStatus:
    online: bool
    last_seen: float | None


class FleetPort(ABC):
    """로봇 연결 추상화 — 스펙 §4.2. M1 구현: legacy_ws, M2: zenoh."""

    @abstractmethod
    def register_robot(self, robot_id: str, farm_id: int, conn_kind: str,
                       config: dict) -> None: ...

    @abstractmethod
    def unregister_robot(self, robot_id: str) -> None:
        """등록 해제 — 링크/태스크를 정리하고 목록에서 제거한다. 접속 정보가
        바뀐 로봇을 재배선(unregister→register)할 때 쓴다."""

    @abstractmethod
    async def send_command(self, robot_id: str, cmd_id: str, action: str,
                           payload: dict) -> str:
        """'sent' | 'offline' — 오프라인이면 즉시 실패, 서버측 큐 금지(스펙 §3.2)."""

    @abstractmethod
    def robot_status(self, robot_id: str) -> RobotStatus: ...

    @abstractmethod
    def set_telemetry_handler(self, cb: TelemetryHandler) -> None: ...


class InMemoryFleetPort(FleetPort):
    """테스트·개발용 — 명령을 sent 리스트에 쌓고 feed() 로 텔레메트리를 주입한다."""

    def __init__(self, offline_after_s: float = 15.0):
        self.presence = PresenceRegistry(offline_after_s)
        self.sent: list[tuple[str, str, str, dict]] = []
        self.robots: dict[str, dict] = {}
        self._handler: TelemetryHandler | None = None

    def register_robot(self, robot_id, farm_id, conn_kind, config):
        self.robots[robot_id] = {"farm_id": farm_id, "conn_kind": conn_kind,
                                 "config": config}

    def unregister_robot(self, robot_id):
        self.robots.pop(robot_id, None)

    async def send_command(self, robot_id, cmd_id, action, payload):
        if not self.presence.online(robot_id):
            return "offline"
        self.sent.append((robot_id, cmd_id, action, payload))
        return "sent"

    def robot_status(self, robot_id):
        return RobotStatus(online=self.presence.online(robot_id),
                           last_seen=self.presence.last_seen(robot_id))

    def set_telemetry_handler(self, cb):
        self._handler = cb

    def feed(self, robot_id, channel, payload, seq=None):
        self.presence.touch(robot_id)
        if self._handler:
            self._handler(robot_id, channel, payload, seq)
