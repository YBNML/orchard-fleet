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


@pytest.mark.asyncio
async def test_teleop_payload_stays_pure():
    """teleop 은 cmd_id 가 섞이지 않은 순수 payload 로, cmd 가 아닌 teleop 토픽에 나가야 한다."""
    received_by_robot = []

    async def fake_robot(ws):
        async for raw in ws:
            received_by_robot.append(json.loads(raw))

    async with websockets.serve(fake_robot, "127.0.0.1", 18100):
        link = LegacyRobotLink(
            "scout01", "ws://127.0.0.1:18100/ws", "",
            on_message=lambda r, ch, pl, seq: None, on_touch=lambda r: None)
        task = asyncio.create_task(link.run())
        for _ in range(100):                       # 연결 수립 대기
            if link._ws is not None:
                break
            await asyncio.sleep(0.05)
        fp = LegacyFleetPort(offline_after_s=15.0)
        fp._links["scout01"] = link
        fp.presence.touch("scout01")
        ok = await fp.send_command("scout01", "c1", "teleop", {"vx": 0.3, "wz": 0.0})
        assert ok == "sent"
        for _ in range(100):
            if received_by_robot:
                break
            await asyncio.sleep(0.05)
        env = received_by_robot[0]
        assert env["topic"] == "orchard/scout01/teleop"
        assert env["payload"] == {"vx": 0.3, "wz": 0.0}     # cmd_id 없음, "cmd" 키 없음
        link.stop(); task.cancel()


@pytest.mark.asyncio
async def test_port_register_robot_uses_running_loop():
    # register_robot 은 asyncio.get_running_loop() 에 의존한다. create_robot 라우터가
    # 다시 동기 def 로 돌아가면(=스레드풀에서 실행) 그 안에서 이 호출이
    # RuntimeError("no running event loop") 를 던진다 — 이 테스트는 최소한 러닝 루프
    # 안에서는 태스크 생성·조회·정리가 예외 없이 도는지 CI 에 고정해 둔다.
    fp = LegacyFleetPort(offline_after_s=15.0)
    fp.register_robot("ghost2", 1, "legacy_ws",
                      {"ws_url": "ws://127.0.0.1:1/ws", "token": ""})
    assert "ghost2" in fp._tasks
    assert fp.robot_status("ghost2").online is False
    await fp.shutdown()
