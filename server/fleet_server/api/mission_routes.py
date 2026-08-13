from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel

from .. import audit, missions, traffic
from ..deps import csrf_protect, current_user, farm_scope, get_db, require_min_role
from ..models import Mission, Robot, User
from ..timeutil import iso_utc

router = APIRouter(tags=["missions"])
_operator = Depends(require_min_role("operator"))
_csrf = Depends(csrf_protect)


class MissionBody(BaseModel):
    robot_id: str
    alleys: list[int] | None = None             # 생략 시 로봇이 전 통로 자동 설정
    work: dict | None = None                    # 검증 없이 로봇에 그대로 전달 (로봇이 BAD_PARAM/UNSUPPORTED 판정)


def _scoped_robot(db, user, robot_id, *, action: str) -> Robot:
    r = db.get(Robot, robot_id)
    if r is None:
        audit.record(db, action=action, result="rejected", user_id=user.id,
                     role=user.role, target=robot_id, detail="로봇 없음")
        raise HTTPException(404, "로봇이 없습니다")
    scope = farm_scope(db, user)
    if scope is not None and r.farm_id not in scope:
        audit.record(db, action=action, result="rejected", user_id=user.id,
                     role=user.role, target=r.id, detail="농장 권한 없음")
        raise HTTPException(403, "해당 농장 권한이 없습니다")
    return r


def _mission_out(ms: Mission) -> dict:
    return {"id": ms.id, "robot_id": ms.robot_id, "farm_id": ms.farm_id,
            "state": ms.state, "spec": ms.spec_json,
            "created_at": iso_utc(ms.created_at),
            "started_at": iso_utc(ms.started_at),
            "ended_at": iso_utc(ms.ended_at)}


@router.post("/missions", dependencies=[_operator, _csrf])
async def create_mission(body: MissionBody, request: Request, db=Depends(get_db),
                         user: User = Depends(current_user)):
    robot = _scoped_robot(db, user, body.robot_id, action="mission_start")
    existing = (db.query(Mission)
                .filter(Mission.robot_id == robot.id,
                        Mission.state.in_(["QUEUED", "RUNNING", "PAUSED"]))
                .first())
    if existing is not None:                   # 로봇당 활성 임무는 1개만 (레이스·오귀속 방지)
        audit.record(db, action="mission_start", result="rejected", user_id=user.id,
                     role=user.role, target=robot.id,
                     detail=f"활성 임무 이미 존재 mission={existing.id}")
        raise HTTPException(409, "해당 로봇에 이미 활성 임무가 있습니다")
    fleet = request.app.state.fleet
    spec: dict = {}
    if body.alleys is not None:
        spec["alleys"] = body.alleys
    if body.work is not None:
        spec["work"] = body.work
    ms = missions.create(db, robot_id=robot.id, farm_id=robot.farm_id,
                         spec=spec, created_by=user.id)
    if body.alleys is not None:                 # 통로 없는(work 전 통로 자동) 임무는 잠금 대상 아님
        ok, reason = traffic.AlleyLocks.acquire(db, robot.id, ms.id, body.alleys)
        if not ok:                              # 잠금 없이 나가는 임무는 없다 — 발진 대신 QUEUED_LOCK
            ms = missions.apply(db, ms, "lock_conflict", payload={"reason": reason})
            audit.record(db, action="mission_start", result="rejected", user_id=user.id,
                         role=user.role, target=robot.id, detail=reason)
            return _mission_out(ms)
    payload: dict = {"mission_id": ms.id}
    if body.alleys is not None:                # 생략 시 키 자체를 넣지 않음 — 로봇이 전 통로 자동
        payload["alleys"] = body.alleys
    if body.work is not None:                  # 서버는 검증하지 않고 그대로 전달
        payload["work"] = body.work
    result = await fleet.send_command(robot.id, f"m{ms.id}", "mission_start", payload)
    if result == "offline":                    # 오프라인 → 즉시 실패 + 잔재 제거
        missions.apply(db, ms, "cancel")
        audit.record(db, action="mission_start", result="rejected", user_id=user.id,
                     role=user.role, target=robot.id, detail="로봇 오프라인")
        raise HTTPException(409, "로봇이 오프라인입니다")
    audit.record(db, action="mission_start", result="accepted", user_id=user.id,
                 role=user.role, target=robot.id,
                 detail=f"alleys={body.alleys} work={body.work}")
    return _mission_out(ms)


_EVENT_BY_VERB = {"pause": "mission_pause", "resume": "mission_resume",
                  "cancel": "mission_cancel"}


@router.post("/missions/{mission_id}/{verb}", dependencies=[_operator, _csrf])
async def mission_verb(mission_id: int, verb: str, request: Request,
                       db=Depends(get_db), user: User = Depends(current_user)):
    if verb not in _EVENT_BY_VERB:
        audit.record(db, action=f"mission_{verb}", result="rejected", user_id=user.id,
                     role=user.role, target=str(mission_id), detail="지원하지 않는 동작")
        raise HTTPException(404, "지원하지 않는 동작")
    action = _EVENT_BY_VERB[verb]
    ms = db.get(Mission, mission_id)
    if ms is None:
        audit.record(db, action=action, result="rejected", user_id=user.id,
                     role=user.role, target=str(mission_id), detail="임무 없음")
        raise HTTPException(404, "임무가 없습니다")
    _scoped_robot(db, user, ms.robot_id, action=action)
    if (ms.state, verb) not in missions.TRANSITIONS:      # 커밋 없이 사전 검사
        audit.record(db, action=action, result="rejected", user_id=user.id,
                     role=user.role, target=ms.robot_id,
                     detail=f"mission={ms.id} 상태={ms.state} 전이불가")
        raise HTTPException(409, f"{ms.state} 에서 {verb} 불가")
    result = await request.app.state.fleet.send_command(
        ms.robot_id, f"m{ms.id}-{verb}", action, {"mission_id": ms.id})
    if result == "offline":                    # 전달 실패 → 상태 변경 없이 즉시 실패
        audit.record(db, action=action, result="rejected", user_id=user.id,
                     role=user.role, target=ms.robot_id,
                     detail=f"mission={ms.id} 로봇 오프라인")
        raise HTTPException(409, "로봇이 오프라인입니다")
    missions.apply(db, ms, verb)                # "sent" 확인 후에만 상태 전이 커밋
    audit.record(db, action=action, result="accepted", user_id=user.id,
                 role=user.role, target=ms.robot_id,
                 detail=f"mission={ms.id} 전달={result}")
    return {**_mission_out(ms), "delivery": result}


@router.get("/missions")
def list_missions(farm_id: int | None = None, robot_id: str | None = None,
                  db=Depends(get_db), user: User = Depends(current_user)):
    scope = farm_scope(db, user)
    q = db.query(Mission)
    if scope is not None:
        q = q.filter(Mission.farm_id.in_(scope))
    if farm_id is not None:
        q = q.filter(Mission.farm_id == farm_id)
    if robot_id is not None:
        q = q.filter(Mission.robot_id == robot_id)
    return [_mission_out(ms) for ms in q.order_by(Mission.id.desc()).limit(200)]


@router.get("/alley-locks")
def list_alley_locks(db=Depends(get_db), user: User = Depends(current_user)):
    return traffic.AlleyLocks.list_active(db)
