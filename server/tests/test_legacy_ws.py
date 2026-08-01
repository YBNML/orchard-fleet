import asyncio
import json

import pytest
import websockets

from fleet_server.fleet.legacy_ws import (SUFFIX_TO_CHANNEL, LegacyFleetPort,
                                          LegacyRobotLink)


def test_suffix_mapping_fixed():
    assert SUFFIX_TO_CHANNEL == {
        "state": "tel/state", "health": "tel/health", "map": "tel/map",
        "event": "evt", "mission": "mission", "hello": "hello"}


@pytest.mark.asyncio
async def test_link_receives_and_sends():
    got, touched, received_by_robot = [], [], []

    async def fake_robot(ws):
        # 토큰이 쿼리로 왔는지 확인
        assert "token=RTOK" in ws.request.path
        await ws.send(json.dumps({"v": 1, "topic": "orchard/scout01/state",
                                  "ts_ns": 1, "seq": 10,
                                  "payload": {"x": 1.0, "y": 2.0}}))
        await ws.send(json.dumps({"v": 1, "topic": "orchard/scout01/event",
                                  "ts_ns": 2, "seq": 11,
                                  "payload": {"kind": "estop"}}))
        async for raw in ws:
            received_by_robot.append(json.loads(raw))

    async with websockets.serve(fake_robot, "127.0.0.1", 18099):
        link = LegacyRobotLink(
            "scout01", "ws://127.0.0.1:18099/ws", "RTOK",
            on_message=lambda r, ch, pl, seq: got.append((r, ch, pl, seq)),
            on_touch=lambda r: touched.append(r))
        task = asyncio.create_task(link.run())
        for _ in range(100):                       # 수신 대기 (최대 5초)
            if len(got) >= 2:
                break
            await asyncio.sleep(0.05)
        assert ("scout01", "tel/state", {"x": 1.0, "y": 2.0}, 10) in got
        assert ("scout01", "evt", {"kind": "estop"}, 11) in got
        assert touched                             # presence touch 호출됨
        ok = await link.send_command("estop", {})
        assert ok is True
        for _ in range(100):
            if received_by_robot:
                break
            await asyncio.sleep(0.05)
        env = received_by_robot[0]
        assert env["topic"] == "orchard/scout01/cmd"
        assert env["payload"]["cmd"] == "estop"    # 기존 봉투 형식 유지
        link.stop(); task.cancel()


@pytest.mark.asyncio
async def test_port_offline_immediate():
    fp = LegacyFleetPort(offline_after_s=15.0)
    fp.register_robot("ghost", 1, "legacy_ws",
                      {"ws_url": "ws://127.0.0.1:1/ws", "token": ""})
    assert await fp.send_command("ghost", "c1", "estop", {}) == "offline"
