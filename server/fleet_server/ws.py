"""브라우저 WebSocket 게이트웨이 — 스펙 §4.4·§4.5.

인증: 세션 쿠키 + Origin 허용 목록(교차 출처 WS 하이재킹 차단, 스펙 §3.6).
mission_* 는 REST 정본이므로 WS 에서 거부한다.

접속 시점 스냅샷(세션·권한·robot_farm)은 접속 중 살아 있는 권한처럼 오작동하면
안 된다 — 정지(disabled)된 계정의 열린 WS 가 무기한 유효하거나, stop_all 이
접속 이후 등록된 로봇을 누락하는 사고로 이어진다. 그래서 cmd·teleop 을 실제로
처리하기 직전마다 세션·사용자·robot_farm 을 DB 기준으로 재검사한다(`revalidate`).
매 프레임 DB 조회를 피하려고 짧은 캐시를 두되, stop_all·clear_estop·set_mode 처럼
저빈도·고위험 명령은 캐시를 무시하고 항상 재검사한다.

송신은 큐 하나 + sender() 태스크 하나로 직렬화한다 — ready·스냅샷·명령응답·
텔레메트리를 전부 같은 큐에 넣어 단일 라이터를 유지하며(경쟁 없음), FIFO 라
큐에 넣은 순서가 곧 클라이언트가 받는 순서다.
"""
from __future__ import annotations

import asyncio
import logging

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from . import audit, auth
from .deps import _session_pair, farm_scope
from .models import Robot

router = APIRouter()
logger = logging.getLogger(__name__)

_WS_ACTIONS = {"estop", "clear_estop", "stop_all", "set_mode", "ping"}
# 저빈도·고위험 명령 — 캐시를 무시하고 매번 DB 로 세션·권한·로봇목록을 다시 본다.
_ALWAYS_REVALIDATE = {"stop_all", "clear_estop", "set_mode"}
_REVALIDATE_INTERVAL_S = 5.0


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
    if not settings.allowed_origins:
        # fail-closed: 허용 목록이 비어 있으면(미설정) 전면 차단한다 — fail-open 금지.
        logger.warning("FLEET_ALLOWED_ORIGINS 미설정 — WS 전면 차단")
    if not settings.allowed_origins or origin not in settings.allowed_origins:
        await websocket.close(code=4403)
        return

    class _Req:                              # _session_pair 는 Request 형태만 필요
        cookies = websocket.cookies

    db = app.state.session_factory()
    try:
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

    def _audit(action, result, target="", detail=""):
        with app.state.session_factory() as adb:
            audit.record(adb, action=action, result=result, user_id=conn.user.id,
                         role=conn.user.role, target=target, detail=detail)

    # 다음 cmd/teleop 이 접속 후 첫 메시지면 무조건 재검사(캐시 미적용)되도록
    # -inf 로 시작한다 — "접속 직후 정지된 계정" 을 즉시 잡아내기 위함.
    last_check = float("-inf")

    async def revalidate(*, force: bool = False) -> bool:
        """세션·사용자·robot_farm 을 DB 기준으로 다시 본다. 실패하면(세션 만료·
        계정 정지) False — 호출자가 거부 응답 후 연결을 닫는다."""
        nonlocal last_check
        now = loop.time()
        if not force and (now - last_check) < _REVALIDATE_INTERVAL_S:
            return True
        rdb = app.state.session_factory()
        try:
            pair = _session_pair(_Req, rdb)
            if pair is None:
                return False
            _, fresh_user = pair
            conn.user = fresh_user
            conn.scope = farm_scope(rdb, fresh_user)
            conn.robot_farm = {r.id: r.farm_id for r in rdb.query(Robot)}
        finally:
            rdb.close()
        last_check = now
        return True

    queue.put_nowait({"type": "ready"})
    # 초기 스냅샷(최신값 캐시) — accept 직후, await 없이 연속 실행하므로 다른
    # 태스크가 끼어들 수 없다(경쟁 불가). ready 다음, 구독 이전에 큐에 들어간다.
    for rid, chans in svc.latest.items():
        for ch, pl in chans.items():
            if conn.sees(rid):
                queue.put_nowait({"topic": f"fleet/v1/{conn.robot_farm[rid]}/{rid}/{ch}",
                                  "payload": pl})

    def on_tel(robot_id: str, channel: str, payload: dict):
        if not conn.sees(robot_id):
            return
        item = {"topic": f"fleet/v1/{conn.robot_farm[robot_id]}/{robot_id}/{channel}",
                "payload": payload}
        try:
            loop.call_soon_threadsafe(queue.put_nowait, item)
        except RuntimeError:
            pass

    unsub = svc.subscribe(on_tel)                # 스냅샷 직후, await 없이 연속 — 유실 없음

    async def sender():                          # 유일한 라이터
        while True:
            item = await queue.get()
            await websocket.send_json(item)
            queue.task_done()

    send_task = asyncio.create_task(sender())

    async def _deny_and_close(action: str, reason: str, audit_detail: str) -> None:
        queue.put_nowait({"type": "denied", "action": action, "reason": reason})
        await queue.join()          # denied 응답이 실제로 나간 뒤에 닫는다(레이스 방지)
        _audit(action, "rejected", detail=audit_detail)
        await websocket.close(code=4401)

    try:
        while True:
            try:
                msg = await websocket.receive_json()
            except WebSocketDisconnect:
                raise
            except Exception:
                # 비-JSON·비-dict 프레임 — 트레이스백으로 연결을 죽이지 않고 무시한다.
                logger.warning("파싱 불가능한 WS 프레임 수신 — 무시하고 계속")
                continue
            if not isinstance(msg, dict):
                continue
            mtype = msg.get("type")

            if mtype == "teleop":
                if not await revalidate():
                    await _deny_and_close("teleop", "세션이 만료되었거나 계정이 정지되었습니다",
                                          "세션 재검사 실패 — 연결 종료")
                    break
                robot = str(msg.get("robot", ""))
                if not auth.authorize(conn.user.role, "teleop") or not conn.sees(robot):
                    queue.put_nowait({"type": "denied", "action": "teleop",
                                      "reason": "권한이 없습니다"})
                    _audit("teleop", "rejected", robot)
                    continue
                if robot not in conn.teleop_audited:
                    # 시작 시 1회 기록 — 비정상 종료(크래시)에도 흔적 보존
                    conn.teleop_audited.add(robot)
                    _audit("teleop_session", "accepted", robot, "텔레옵 시작")
                await app.state.fleet.send_command(robot, "", "teleop",
                                                   msg.get("payload", {}))
                continue

            if mtype != "cmd":
                continue
            action = str(msg.get("action", ""))
            cmd_id = str(msg.get("cmd_id", ""))

            if action.startswith("mission_"):
                queue.put_nowait({"type": "denied", "action": action,
                                  "reason": "임무는 REST API 를 사용하세요"})
                _audit(action, "rejected", detail="WS 임무 명령 거부 — REST 정본")
                continue

            if not await revalidate(force=action in _ALWAYS_REVALIDATE):
                await _deny_and_close(action, "세션이 만료되었거나 계정이 정지되었습니다",
                                      "세션 재검사 실패 — 연결 종료")
                break

            if action not in _WS_ACTIONS or not auth.authorize(conn.user.role, action):
                queue.put_nowait({"type": "denied", "action": action,
                                  "reason": "권한이 없습니다"})
                _audit(action or "unknown", "rejected", detail="WS 명령 거부")
                continue

            if action == "stop_all":                            # 스코프 내 전 로봇 팬아웃
                results = {}                    # robot_farm 은 위 revalidate(force=True) 로 이미 최신
                for rid in sorted(conn.robot_farm):
                    if conn.sees(rid):
                        results[rid] = await app.state.fleet.send_command(
                            rid, f"{cmd_id}-{rid}", "estop", {})
                queue.put_nowait({"type": "stop_all_result", "results": results})
                _audit("stop_all", "accepted", detail=str(results))
                continue

            robot = str(msg.get("robot", ""))
            if not conn.sees(robot):
                queue.put_nowait({"type": "denied", "action": action,
                                  "reason": "해당 농장 권한이 없습니다"})
                _audit(action, "rejected", robot, "농장 스코프 밖")
                continue
            result = await app.state.fleet.send_command(robot, cmd_id, action,
                                                        msg.get("payload", {}))
            queue.put_nowait({"type": "cmd_result", "robot": robot,
                              "cmd_id": cmd_id, "result": result})
            _audit(action, "accepted" if result == "sent" else "rejected",
                   robot, f"전달={result}")
    except WebSocketDisconnect:
        pass
    finally:
        unsub()
        send_task.cancel()
