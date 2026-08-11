"""레거시 로봇 어댑터 — M1 한시, M2 에서 Zenoh 로 교체(스펙 §9 M1).

서버가 로봇의 기존 WebSocket 서버(ws://로봇:8080/ws?token=...)에 클라이언트로
접속한다. robomw.link.protocol 의 봉투 {v, topic:"orchard/{robot}/{suffix}",
ts, seq, payload} 를 fleet 채널로 매핑한다(명령 계약은 robomw 가 정본).

한계: 레거시 로봇에는 ack 채널이 없다 → send_command 는 소켓 기록 성공을
"sent" 로 간주한다 (cmd_id 상관 응답은 M2).
"""
from __future__ import annotations

import asyncio
import json
import logging
import time

import websockets
from robomw.link import protocol as P

from .port import FleetPort, RobotStatus, TelemetryHandler
from .presence import PresenceRegistry

log = logging.getLogger("fleet.legacy_ws")

SUFFIX_TO_CHANNEL = {
    "state": "tel/state", "health": "tel/health", "map": "tel/map",
    "event": "evt", "mission": "mission", "hello": "hello",
}


class LegacyRobotLink:
    def __init__(self, robot_id: str, ws_url: str, token: str,
                 on_message, on_touch):
        self.robot_id = robot_id
        self.ws_url = ws_url
        self.token = token
        self.on_message = on_message          # (robot_id, channel, payload, seq)
        self.on_touch = on_touch              # (robot_id)
        self._ws = None
        self._stop = False
        self._seq = 0

    async def run(self) -> None:
        backoff = 1.0
        url = self.ws_url + (f"?token={self.token}" if self.token else "")
        while not self._stop:
            try:
                async with websockets.connect(url, open_timeout=5) as ws:
                    self._ws = ws
                    backoff = 1.0
                    # 하트비트 — 로봇의 SafetyArbiter 는 관제에서 오는 트래픽이
                    # LINK_LOSS_STOP_MS(1.5초) 넘게 끊기면 링크 두절로 보고 스스로
                    # 선다. 수신만 하고 아무것도 보내지 않으면 로봇은 연결돼 있어도
                    # '관제가 없다'고 판단해 움직이지 않는다 — 실제로 겪은 문제다.
                    hb = asyncio.create_task(self._heartbeat(ws))
                    try:
                      async for raw in ws:
                        try:
                            msg = json.loads(raw)
                        except (ValueError, TypeError):
                            continue
                        self.on_touch(self.robot_id)
                        parts = str(msg.get("topic", "")).split("/", 2)
                        if len(parts) < 3:
                            continue
                        ch = SUFFIX_TO_CHANNEL.get(parts[2])
                        if ch:
                            try:
                                self.on_message(self.robot_id, ch,
                                                msg.get("payload", {}), msg.get("seq"))
                            except Exception:
                                # 다운스트림(핸들러) 예외는 링크 재연결로 위장하지 않는다 —
                                # 연결은 살아 있으니 다음 메시지를 계속 받는다.
                                log.exception("로봇 %s 텔레메트리 처리 중 예외 (channel=%s)",
                                             self.robot_id, ch)
                    finally:
                        hb.cancel()
            except Exception as e:
                log.warning("로봇 %s 링크 오류: %s — %.1fs 후 재연결",
                           self.robot_id, e, backoff)
            self._ws = None
            if self._stop:
                break
            await asyncio.sleep(backoff)          # 재연결 지수 백오프 (스펙 §3.1)
            backoff = min(backoff * 2, 30.0)

    async def _heartbeat(self, ws, period=1.0):
        """로봇에게 '관제가 살아 있다'를 알린다 (ping 은 observer 권한이라 무해)."""
        while True:
            await asyncio.sleep(period)
            try:
                await self.send_command("ping", {})
            except Exception:
                return

    async def send_command(self, action: str, payload: dict) -> bool:
        ws = self._ws
        if ws is None:
            return False
        self._seq += 1
        suffix = "teleop" if action == "teleop" else "cmd"
        body = payload if action == "teleop" else {"cmd": action, **payload}
        # 봉투 조립은 robomw.link.protocol 이 정본이다(관제·로봇 계약 단일화).
        # 참고: 이전 손조립 봉투는 시각 키를 "ts_ns" 로 뒀는데, protocol.envelope()
        # 는 정식 계약대로 "ts" 를 쓴다(값은 그대로 로봇 기준 나노초 정수).
        # 로봇측 파서(P.parse())는 이 필드를 애초에 버리므로(control_agent
        # 의 `t, payload, _, _ = P.parse(...)`) 기능상 영향은 없다 — v·topic·
        # seq·payload 는 바이트 단위로 이전과 동일하다.
        topic_str = P.topic("orchard", self.robot_id, suffix)
        env = P.envelope(topic_str, body, time.time_ns(), self._seq)
        try:
            await ws.send(json.dumps(env))
            return True
        except Exception as e:
            log.warning("로봇 %s 명령 전송 실패 (action=%s): %s", self.robot_id, action, e)
            return False

    def stop(self) -> None:
        self._stop = True


class LegacyFleetPort(FleetPort):
    def __init__(self, offline_after_s: float = 15.0):
        self.presence = PresenceRegistry(offline_after_s)
        self._links: dict[str, LegacyRobotLink] = {}
        self._tasks: dict[str, asyncio.Task] = {}
        self._handler: TelemetryHandler | None = None

    def register_robot(self, robot_id, farm_id, conn_kind, config):
        if robot_id in self._links or conn_kind != "legacy_ws":
            return
        link = LegacyRobotLink(robot_id, config.get("ws_url", ""),
                               config.get("token", ""),
                               on_message=self._on_message,
                               on_touch=self.presence.touch)
        self._links[robot_id] = link
        self._tasks[robot_id] = asyncio.get_running_loop().create_task(link.run())

    def unregister_robot(self, robot_id):
        link = self._links.pop(robot_id, None)
        if link is not None:
            link.stop()
        task = self._tasks.pop(robot_id, None)
        if task is not None:
            task.cancel()

    def _on_message(self, robot_id, channel, payload, seq):
        if self._handler:
            self._handler(robot_id, channel, payload, seq)

    async def send_command(self, robot_id, cmd_id, action, payload):
        link = self._links.get(robot_id)
        if link is None or not self.presence.online(robot_id):
            return "offline"                    # 즉시 실패 — 서버측 큐 금지
        # teleop 은 순수 조향 payload({"vx":..,"wz":..}) 그대로 보낸다 — cmd_id 를
        # 섞으면 로봇의 teleop 파서가 알 수 없는 키로 거부할 수 있다.
        body = payload if action == "teleop" else {**payload, "cmd_id": cmd_id}
        ok = await link.send_command(action, body)
        return "sent" if ok else "offline"

    def robot_status(self, robot_id):
        return RobotStatus(online=self.presence.online(robot_id),
                           last_seen=self.presence.last_seen(robot_id))

    def set_telemetry_handler(self, cb):
        self._handler = cb

    async def shutdown(self):
        for link in self._links.values():
            link.stop()
        for task in self._tasks.values():
            task.cancel()
