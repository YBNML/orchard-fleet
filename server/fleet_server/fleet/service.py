"""텔레메트리 허브 — FleetPort 수신을 DB 수집·최신값 캐시·구독자 팬아웃으로.

mission 채널 payload {"state": ...} 는 임무 상태기계로 동기화한다
(로봇이 완료를 보고하면 서버 임무도 DONE 이 된다).
"""
from __future__ import annotations

import re
from typing import Callable

from .. import ingest, missions
from ..models import Mission

_ROBOT_STATE_EVENT = {"running": "start", "paused": "pause", "done": "complete",
                      "canceled": "cancel", "failed": "fail"}

# 리뷰 라운드 1 (I4) — mission_start 의 cmd_id 는 mission_routes.create_mission
# 이 항상 f"m{mission.id}" 로 짓는다(검증 동사 cmd_id 는 "m{id}-{verb}" 라
# 대시가 있어 이 정규식에 안 걸린다 — mission_start 만 골라낸다). 이 관례를
# 파싱해 mission_id 를 되찾는다 — 재기동으로 잃어도 되는 인메모리 상관표
# 대신 이미 존재하는 결정적 규약을 재사용한 것(재기동 후 유실되는 경우는
# C3 의 로컬 cancel 로 회수 가능하다).
_MSTART_CMD_ID_RE = re.compile(r"^m(\d+)$")


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
                fresh = ingest.event(db, robot_id, channel, seq, payload)
                if fresh:
                    self._route_intervention(db, robot_id, payload)
                    self._maybe_fail_rejected_mission_start(db, robot_id, payload)
        elif channel == "mission":
            self._sync_mission(robot_id, payload)
        for cb in list(self._subs):
            cb(robot_id, channel, payload)

    # 로봇 이벤트 → 개입 큐. 이벤트가 곧 티켓이 되는 지점이다.
    #   kind 가 코드표에 있으면 그것을, 아니면 payload.code 를 본다 —
    #   구형 에이전트(kind="estop")와 신형(code="ESTOP_REMOTE") 둘 다 받는다.
    _KIND_TO_CODE = {"estop": "ESTOP_REMOTE", "estop_cleared": None,
                     "link_lost": "LINK_LOST_POLICY", "tilt": "TILT_LIMIT"}

    def _route_intervention(self, db, robot_id: str, payload: dict) -> None:
        from .. import interventions, stopcodes
        from ..models import Robot
        kind = str(payload.get("kind", ""))
        code = payload.get("code") or self._KIND_TO_CODE.get(kind)
        if kind in ("estop_cleared", "resolved") and payload.get("code"):
            interventions.auto_resolve(db, robot_id, payload["code"])
            return
        if not code or not stopcodes.is_intervention(code):
            return
        robot = db.get(Robot, robot_id)
        if robot is None:
            return
        interventions.open_or_bump(
            db, robot_id=robot_id, farm_id=robot.farm_id, code=code,
            msg=str(payload.get("msg", ""))[:256],
            severity=str(payload.get("severity", "warn")),
            context={k: payload[k] for k in ("x", "y", "alley") if k in payload})

    def _maybe_fail_rejected_mission_start(self, db, robot_id: str, payload: dict) -> None:
        """로봇이 mission_start 자체를 거부하면(BUSY·BAD_PARAM·ESTOPPED·
        UNSUPPORTED) 서버 쪽 임무는 로봇의 "running" 보고를 영영 못 받아
        QUEUED 에 멈춘다 — AlleyLock 을 쓰는 지금은 그 잠금도 함께 고착된다.
        cmd_id 로 임무를 되찾아 FAILED 로 종착시켜 잠금을 해제한다."""
        if payload.get("kind") != "cmd_result":
            return
        if payload.get("cmd") != "mission_start" or payload.get("status") != "rejected":
            return
        m = _MSTART_CMD_ID_RE.match(str(payload.get("cmd_id", "")))
        if not m:
            return
        ms = db.get(Mission, int(m.group(1)))
        if ms is None or ms.robot_id != robot_id:
            return
        reason = (payload.get("data") or {}).get("reason") or payload.get("code") or "거부"
        try:
            missions.apply(db, ms, "fail", payload={"reason": reason, "code": payload.get("code")})
        except missions.InvalidTransition:
            pass                                # 이미 종착 상태 등 — 무시

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
