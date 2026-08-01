from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel

from .. import audit, missions
from ..deps import csrf_protect, current_user, farm_scope, get_db, require_min_role
from ..models import Mission, Robot, User

router = APIRouter(tags=["missions"])
_operator = Depends(require_min_role("operator"))
_csrf = Depends(csrf_protect)


class MissionBody(BaseModel):
    robot_id: str
    alleys: list[int]


def _scoped_robot(db, user, robot_id) -> Robot:
    r = db.get(Robot, robot_id)
    if r is None:
        raise HTTPException(404, "로봇이 없습니다")
    scope = farm_scope(db, user)
    if scope is not None and r.farm_id not in scope:
        raise HTTPException(403, "해당 농장 권한이 없습니다")
    return r


def _mission_out(ms: Mission) -> dict:
    return {"id": ms.id, "robot_id": ms.robot_id, "farm_id": ms.farm_id,
            "state": ms.state, "spec": ms.spec_json,
            "created_at": ms.created_at.isoformat(),
            "started_at": ms.started_at.isoformat() if ms.started_at else None,
            "ended_at": ms.ended_at.isoformat() if ms.ended_at else None}


@router.post("/missions", dependencies=[_operator, _csrf])
async def create_mission(body: MissionBody, request: Request, db=Depends(get_db),
                         user: User = Depends(current_user)):
    robot = _scoped_robot(db, user, body.robot_id)
    fleet = request.app.state.fleet
    ms = missions.create(db, robot_id=robot.id, farm_id=robot.farm_id,
                         spec={"alleys": body.alleys}, created_by=user.id)
    result = await fleet.send_command(robot.id, f"m{ms.id}", "mission_start",
                                      {"alleys": body.alleys, "mission_id": ms.id})
    if result == "offline":                    # 오프라인 → 즉시 실패 + 잔재 제거
        missions.apply(db, ms, "cancel")
        audit.record(db, action="mission_start", result="rejected", user_id=user.id,
                     role=user.role, target=robot.id, detail="로봇 오프라인")
        raise HTTPException(409, "로봇이 오프라인입니다")
    audit.record(db, action="mission_start", result="accepted", user_id=user.id,
                 role=user.role, target=robot.id, detail=f"alleys={body.alleys}")
    return _mission_out(ms)


_EVENT_BY_VERB = {"pause": "mission_pause", "resume": "mission_resume",
                  "cancel": "mission_cancel"}


@router.post("/missions/{mission_id}/{verb}", dependencies=[_operator, _csrf])
async def mission_verb(mission_id: int, verb: str, request: Request,
                       db=Depends(get_db), user: User = Depends(current_user)):
    if verb not in _EVENT_BY_VERB:
        raise HTTPException(404, "지원하지 않는 동작")
    ms = db.get(Mission, mission_id)
    if ms is None:
        raise HTTPException(404, "임무가 없습니다")
    _scoped_robot(db, user, ms.robot_id)
    try:
        missions.apply(db, ms, verb)
    except missions.InvalidTransition as e:
        raise HTTPException(409, str(e))
    result = await request.app.state.fleet.send_command(
        ms.robot_id, f"m{ms.id}-{verb}", _EVENT_BY_VERB[verb], {"mission_id": ms.id})
    audit.record(db, action=_EVENT_BY_VERB[verb],
                 result="accepted" if result == "sent" else "rejected",
                 user_id=user.id, role=user.role, target=ms.robot_id,
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
