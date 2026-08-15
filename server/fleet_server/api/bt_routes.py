"""BT 임무 큐 API — 프리셋으로 인스턴스를 만들고, 트리 상태를 보고, 중단한다."""
from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel

from .. import audit
from ..bt import presets
from ..deps import csrf_protect, current_user, farm_scope, get_db, require_min_role
from ..models import BTInstance, User
from ..timeutil import iso_utc
from .mission_routes import scoped_robot

router = APIRouter(tags=["bt"])
_operator = Depends(require_min_role("operator"))
_csrf = Depends(csrf_protect)


class BTBody(BaseModel):
    preset: str
    params: dict = {}


def _out(row: BTInstance) -> dict:
    return {"id": row.id, "preset": row.preset, "params": row.params_json,
            "robot_id": row.robot_id, "farm_id": row.farm_id, "state": row.state,
            "note": row.note, "node_states": row.tree_json,
            "created_at": iso_utc(row.created_at),
            "updated_at": iso_utc(row.updated_at)}


@router.post("/bt", dependencies=[_operator, _csrf])
def create_bt(body: BTBody, request: Request, db=Depends(get_db),
              user: User = Depends(current_user)):
    """프리셋 → 인스턴스 N개. 프리셋이 고른 로봇마다 농장 권한을 확인한다
    (인가는 생성 시점에 한 번 — 이후 틱은 그 인스턴스의 발주자 권한으로 돈다)."""
    try:
        plans = presets.build(body.preset, body.params, farm=request.app.state.farm)
    except presets.PresetError as e:
        audit.record(db, action="bt_create", result="rejected", user_id=user.id,
                     role=user.role, target=body.preset, detail=str(e))
        raise HTTPException(400, str(e)) from None
    for plan in plans:
        scoped_robot(db, user, plan.robot_id, action="bt_create")
    ids = request.app.state.bt_engine.create_from_plans(
        body.preset, body.params, plans, created_by=user.id)
    audit.record(db, action="bt_create", result="accepted", user_id=user.id,
                 role=user.role, target=body.preset,
                 detail=f"ids={ids} params={body.params}")
    return {"ids": ids}


@router.get("/bt")
def list_bt(state: str | None = None, limit: int = 100, db=Depends(get_db),
            user: User = Depends(current_user)):
    scope = farm_scope(db, user)                # list_missions 와 같은 관례
    q = db.query(BTInstance)
    if scope is not None:
        q = q.filter(BTInstance.farm_id.in_(scope))
    if state:
        q = q.filter(BTInstance.state == state.upper())
    return [_out(r) for r in q.order_by(BTInstance.id.desc()).limit(min(limit, 500))]


@router.post("/bt/{instance_id}/cancel", dependencies=[_operator, _csrf])
async def cancel_bt(instance_id: int, request: Request, db=Depends(get_db),
                    user: User = Depends(current_user)):
    row = db.get(BTInstance, instance_id)
    if row is None:
        audit.record(db, action="bt_cancel", result="rejected", user_id=user.id,
                     role=user.role, target=str(instance_id), detail="인스턴스 없음")
        raise HTTPException(404, "BT 인스턴스가 없습니다")
    scoped_robot(db, user, row.robot_id, action="bt_cancel")
    await request.app.state.bt_engine.cancel(instance_id)
    db.expire_all()                             # 엔진이 다른 세션에서 바꿨다
    audit.record(db, action="bt_cancel", result="accepted", user_id=user.id,
                 role=user.role, target=str(instance_id),
                 detail=f"preset={row.preset} robot={row.robot_id}")
    return _out(db.get(BTInstance, instance_id))
