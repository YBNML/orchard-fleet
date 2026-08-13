from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel

from .. import audit, mission_ops, traffic
from ..deps import csrf_protect, current_user, farm_scope, get_db, require_min_role
from ..mission_ops import MissionOpError
from ..models import Mission, Robot, User
from ..timeutil import iso_utc

router = APIRouter(tags=["missions"])
_operator = Depends(require_min_role("operator"))
_csrf = Depends(csrf_protect)


class MissionBody(BaseModel):
    robot_id: str
    alleys: list[int] | None = None             # 생략 시 로봇이 전 통로 자동 설정
    work: dict | None = None                    # 검증 없이 로봇에 그대로 전달 (로봇이 BAD_PARAM/UNSUPPORTED 판정)


def scoped_robot(db, user, robot_id, *, action: str) -> Robot:
    """로봇 조회 + 농장 권한 확인(거부는 감사에 남긴다). BT 라우터도 같이 쓴다."""
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
    """생성·검증·잠금·발진의 실체는 mission_ops.create_and_dispatch 에 있다
    (BT Action 이 같은 함수를 부른다 — 우회 생산자를 만들면 T4 의 불변이
    깨진다). 여기 남는 것은 인가·감사·상태코드 변환이다."""
    if not mission_ops.alleys_sequence_valid(body.alleys):
        audit.record(db, action="mission_start", result="rejected", user_id=user.id,
                     role=user.role, target=body.robot_id,
                     detail=f"통로 목록이 인접 순서가 아님 alleys={body.alleys}")
        raise HTTPException(400, "통로 목록은 순서상 인접한 통로만 연속으로 넣을 수 있습니다")
    robot = scoped_robot(db, user, body.robot_id, action="mission_start")
    try:
        ms, lock_reason = await mission_ops.create_and_dispatch(
            db, request.app.state.fleet, robot=robot, alleys=body.alleys,
            work=body.work, created_by=user.id)
    except MissionOpError as e:
        audit.record(db, action="mission_start", result="rejected", user_id=user.id,
                     role=user.role, target=robot.id, detail=e.detail)
        raise HTTPException(e.status, e.message) from None
    if lock_reason is not None:                 # 발진 대신 QUEUED_LOCK
        audit.record(db, action="mission_start", result="rejected", user_id=user.id,
                     role=user.role, target=robot.id, detail=lock_reason)
        return {**_mission_out(ms), "lock_reason": lock_reason}   # 대시보드 토스트용(I7)
    audit.record(db, action="mission_start", result="accepted", user_id=user.id,
                 role=user.role, target=robot.id,
                 detail=f"alleys={body.alleys} work={body.work}")
    return _mission_out(ms)


_EVENT_BY_VERB = mission_ops.EVENT_BY_VERB


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
    scoped_robot(db, user, ms.robot_id, action=action)
    try:
        delivery = await mission_ops.apply_verb(db, request.app.state.fleet, ms, verb)
    except MissionOpError as e:
        audit.record(db, action=action, result="rejected", user_id=user.id,
                     role=user.role, target=ms.robot_id, detail=e.detail)
        raise HTTPException(e.status, e.message) from None
    if delivery == "not_sent":                  # C3 — 오프라인 로봇의 로컬 취소
        audit.record(db, action=action, result="accepted", user_id=user.id,
                     role=user.role, target=ms.robot_id,
                     detail=f"mission={ms.id} 로봇 오프라인 — 로컬 취소")
        return {**_mission_out(ms), "delivery": "not_sent"}
    audit.record(db, action=action, result="accepted", user_id=user.id,
                 role=user.role, target=ms.robot_id,
                 detail=f"mission={ms.id} 전달={delivery}")
    return {**_mission_out(ms), "delivery": delivery}


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
    scope = farm_scope(db, user)                # I6 — list_missions 와 같은 관례
    rows = traffic.AlleyLocks.list_active(db)
    if scope is not None:
        rows = [r for r in rows if r["farm_id"] in scope]
    return rows
