#!/usr/bin/env python3
"""레거시 로봇 흉내 — 기존 봉투 형식으로 텔레메트리를 쏘고 명령에 반응한다.

E2E(33)·보안(34) 검증이 사용한다. 실로봇(시뮬) 대신 결정적으로 돈다.
"""
from __future__ import annotations

import asyncio
import json
import time


class FakeRobot:
    def __init__(self, robot_id="scout01", port=18080, token="RTOK"):
        self.robot_id, self.port, self.token = robot_id, port, token
        self.seq = 0
        self.estop = False
        self.mission = None            # None | "running" | "paused" | "done"
        self.received: list[dict] = []  # 수신한 cmd/teleop 봉투 전부
        self.teleop_count = 0
        self._x = 0.0
        self._ws = None                # 접속 중인 연결 — send_event() 가 직접 쓴다

    def env(self, suffix, payload):
        self.seq += 1
        return json.dumps({"v": 1, "topic": f"orchard/{self.robot_id}/{suffix}",
                           "ts_ns": time.time_ns(), "seq": self.seq,
                           "payload": payload})

    async def handler(self, ws):
        if self.token and f"token={self.token}" not in ws.request.path:
            await ws.close(code=4401)
            return

        async def pump():
            while True:
                self._x += 0.1
                await ws.send(self.env("state", {
                    "x": round(self._x, 2), "y": -28.0, "yaw": 0.0,
                    "mode": "mission" if self.mission == "running" else "idle",
                    "estop": self.estop, "ts": time.time()}))
                if self.mission == "running" and self._x >= 1.0 and self.mission != "done":
                    self.mission = "done"
                    await ws.send(self.env("mission", {"state": "done"}))
                await asyncio.sleep(0.2)

        self._ws = ws
        pump_task = asyncio.create_task(pump())
        try:
            async for raw in ws:
                msg = json.loads(raw)
                self.received.append(msg)
                topic = msg.get("topic", "")
                pl = msg.get("payload", {})
                if topic.endswith("/teleop"):
                    self.teleop_count += 1
                    continue
                cmd = pl.get("cmd", "")
                if cmd == "estop":
                    self.estop = True
                    if self.mission == "running":
                        self.mission = "paused"
                        await ws.send(self.env("mission", {"state": "paused"}))
                    await ws.send(self.env("event", {"kind": "estop", "severity": "warn",
                                                     "msg": "비상정지", "ts": time.time()}))
                elif cmd == "clear_estop":
                    self.estop = False
                    await ws.send(self.env("event", {"kind": "estop_cleared",
                                                     "severity": "info", "msg": "해제",
                                                     "ts": time.time()}))
                elif cmd == "mission_start":
                    self.mission = "running"
                    self._x = 0.0
                    await ws.send(self.env("mission", {"state": "running"}))
                elif cmd == "mission_resume":
                    self.mission = "running"
                    await ws.send(self.env("mission", {"state": "running"}))
        finally:
            self._ws = None
            pump_task.cancel()

    async def send_event(self, kind: str, msg: str, severity: str = "warn") -> None:
        """테스트 스크립트가 임의 내용으로 evt 를 쏘고 싶을 때 쓴다(예: XSS 페이로드
        회귀) — 실제 로봇이 임의 문자열을 보낼 수 있다는 걸 흉내낸다."""
        if self._ws is None:
            raise RuntimeError("로봇이 아직 접속하지 않았습니다")
        await self._ws.send(self.env("event", {"kind": kind, "severity": severity,
                                                "msg": msg, "ts": time.time()}))

    async def serve(self):
        import websockets
        return await websockets.serve(self.handler, "127.0.0.1", self.port)


if __name__ == "__main__":
    async def main():
        fr = FakeRobot()
        await fr.serve()
        print(f"가짜 로봇 ws://127.0.0.1:{fr.port}/ws?token={fr.token}")
        await asyncio.Future()
    asyncio.run(main())
