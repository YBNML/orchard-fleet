"""개입 큐·지표 API."""
from __future__ import annotations

import datetime as dt

from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel

from .. import audit, interventions, metrics, stopcodes
from ..deps import csrf_protect, current_user, farm_scope, get_db, require_min_role
from ..models import Intervention, Robot, User

router = APIRouter(tags=["ops"])
_operator = Depends(require_min_role("operator"))
_csrf = Depends(csrf_protect)


def _scoped(db, user: User, row: Intervention) -> Intervention:
    scope = farm_scope(db, user)
    if scope is not None and row.farm_id not in scope:
        raise HTTPException(403, "해당 농장 권한이 없습니다")
    return row


def _get(db, user: User, iid: int) -> Intervention:
    row = db.get(Intervention, iid)
    if row is None:
        raise HTTPException(404, "개입 건이 없습니다")
    return _scoped(db, user, row)


@router.get("/stopcodes")
def list_stopcodes():
    """코드표 자체를 화면이 읽을 수 있게 — 라벨을 프론트에 복사해 두지 않는다."""
    return stopcodes.as_list()


@router.get("/interventions")
def list_interventions(state: str | None = None, robot_id: str | None = None,
                       limit: int = 100, db=Depends(get_db),
                       user: User = Depends(current_user)):
    scope = farm_scope(db, user)
    q = db.query(Intervention)
    if scope is not None:
        q = q.filter(Intervention.farm_id.in_(scope))
    if robot_id:
        q = q.filter(Intervention.robot_id == robot_id)
    if state == "active":
        q = q.filter(Intervention.state.in_(("OPEN", "ACKED")))
    elif state:
        q = q.filter(Intervention.state == state.upper())
    q = q.order_by(Intervention.state != "OPEN",      # 미처리를 위로
                   Intervention.opened_at)            # 오래된 것부터
    return [interventions.out(r) for r in q.limit(min(limit, 500))]


class NoteBody(BaseModel):
    note: str = ""


def _act(verb):
    fn = {"ack": interventions.ack, "resolve": interventions.resolve,
          "escalate": interventions.escalate}[verb]

    async def handler(iid: int, body: NoteBody | None = None, db=Depends(get_db),
                      user: User = Depends(require_min_role("operator"))):
        row = _get(db, user, iid)
        note = (body.note if body else "") or ""
        row = fn(db, row, user.id, note) if verb != "ack" else fn(db, row, user.id)
        audit.record(db, action=f"intervention_{verb}", result="accepted",
                     user_id=user.id, role=user.role, target=str(iid),
                     detail=f"{row.code} {note}"[:160])
        return interventions.out(row)
    return handler


for _v in ("ack", "resolve", "escalate"):
    router.add_api_route(f"/interventions/{{iid}}/{_v}", _act(_v),
                         methods=["POST"], dependencies=[_csrf],
                         name=f"intervention_{_v}")


@router.get("/metrics")
def ops_metrics(days: int | None = None, db=Depends(get_db),
                user: User = Depends(current_user)):
    scope = farm_scope(db, user)
    since = (dt.datetime.now(dt.UTC) - dt.timedelta(days=days)) if days else None
    return metrics.summary(db, farm_ids=scope, since=since)
