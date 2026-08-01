"""브라우저 WebSocket 게이트웨이 — 스펙 §4.4·§4.5.

인증: 세션 쿠키 + Origin 허용 목록(교차 출처 WS 하이재킹 차단, 스펙 §3.6).
mission_* 는 REST 정본이므로 WS 에서 거부한다.
"""
from __future__ import annotations

import asyncio

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from . import audit, auth
from .deps import SESSION_COOKIE, _session_pair, farm_scope
from .models import Robot

router = APIRouter()

_WS_ACTIONS = {"estop", "clear_estop", "stop_all", "set_mode", "ping"}


class _Conn:
    def __init__(self, user, scope, robot_farm):
        self.user = user
        self.scope = scope                      # None=admin 전체
        self.robot_farm = robot_farm            # robot_id -> farm_id
        self.teleop_audited: set[str] = set()

    def sees(self, robot_id: str) -> bool:
        farm = self.robot_farm.get(robot_id)
        return farm is not None and (self.scope is None or farm in self.scope)


@router.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    app = websocket.app
    settings = app.state.settings
    origin = websocket.headers.get("origin", "")
    if settings.allowed_origins and origin not in settings.allowed_origins:
        await websocket.close(code=4403)
        return

    db = app.state.session_factory()
    try:
        class _Req:                              # _session_pair 는 Request 형태만 필요
            cookies = websocket.cookies
        pair = _session_pair(_Req, db)
        if pair is None:
            await websocket.close(code=4401)
            return
        user = pair[1]
        scope = farm_scope(db, user)
        robot_farm = {r.id: r.farm_id for r in db.query(Robot)}
    finally:
        db.close()

    conn = _Conn(user, scope, robot_farm)
    await websocket.accept()
    loop = asyncio.get_running_loop()
    queue: asyncio.Queue = asyncio.Queue(maxsize=1000)
    svc = app.state.fleet_service

    def on_tel(robot_id: str, channel: str, payload: dict):
        if not conn.sees(robot_id):
            return
        item = {"topic": f"fleet/v1/{conn.robot_farm[robot_id]}/{robot_id}/{channel}",
                "payload": payload}
        try:
            loop.call_soon_threadsafe(queue.put_nowait, item)
        except RuntimeError:
            pass

    unsub = svc.subscribe(on_tel)

    async def sender():
        while True:
            await websocket.send_json(await queue.get())

    send_task = asyncio.create_task(sender())
    await websocket.send_json({"type": "ready"})

    def _audit(action, result, target="", detail=""):
        with app.state.session_factory() as adb:
            audit.record(adb, action=action, result=result, user_id=user.id,
                         role=user.role, target=target, detail=detail)

    try:
        while True:
            msg = await websocket.receive_json()
            mtype = msg.get("type")

            if mtype == "teleop":
                robot = str(msg.get("robot", ""))
                if not auth.authorize(user.role, "teleop") or not conn.sees(robot):
                    await websocket.send_json({"type": "denied", "action": "teleop",
                                               "reason": "권한이 없습니다"})
                    _audit("teleop", "rejected", robot)
                    continue
                conn.teleop_audited.add(robot)                  # 세션 단위 감사 (S7) — 종료 시 1건 기록
                await app.state.fleet.send_command(robot, "", "teleop",
                                                   msg.get("payload", {}))
                continue

            if mtype != "cmd":
                continue
            action = str(msg.get("action", ""))
            cmd_id = str(msg.get("cmd_id", ""))

            if action.startswith("mission_"):
                await websocket.send_json({"type": "denied", "action": action,
                                           "reason": "임무는 REST API 를 사용하세요"})
                continue
            if action not in _WS_ACTIONS or not auth.authorize(user.role, action):
                await websocket.send_json({"type": "denied", "action": action,
                                           "reason": "권한이 없습니다"})
                _audit(action or "unknown", "rejected", detail="WS 명령 거부")
                continue

            if action == "stop_all":                            # 스코프 내 전 로봇 팬아웃
                results = {}
                for rid in sorted(conn.robot_farm):
                    if conn.sees(rid):
                        results[rid] = await app.state.fleet.send_command(
                            rid, f"{cmd_id}-{rid}", "estop", {})
                await websocket.send_json({"type": "stop_all_result", "results": results})
                _audit("stop_all", "accepted", detail=str(results))
                continue

            robot = str(msg.get("robot", ""))
            if not conn.sees(robot):
                await websocket.send_json({"type": "denied", "action": action,
                                           "reason": "해당 농장 권한이 없습니다"})
                _audit(action, "rejected", robot, "농장 스코프 밖")
                continue
            result = await app.state.fleet.send_command(robot, cmd_id, action,
                                                        msg.get("payload", {}))
            await websocket.send_json({"type": "cmd_result", "robot": robot,
                                       "cmd_id": cmd_id, "result": result})
            _audit(action, "accepted" if result == "sent" else "rejected",
                   robot, f"전달={result}")
    except WebSocketDisconnect:
        pass
    finally:
        unsub()
        send_task.cancel()
        for robot in conn.teleop_audited:
            _audit("teleop_session", "accepted", robot, "텔레옵 종료")
