from __future__ import annotations

import datetime as dt

from fastapi import APIRouter, Depends, HTTPException

from ..deps import current_user, farm_scope, get_db, require_min_role
from ..models import AuditLog, Event, Robot, Track, User

router = APIRouter(tags=["history"])


def _scoped_robot_ids(db, user, robot_id: str | None) -> list[str]:
    scope = farm_scope(db, user)
    q = db.query(Robot.id)
    if scope is not None:
        q = q.filter(Robot.farm_id.in_(scope))
    ids = [r[0] for r in q]
    if robot_id is not None:
        if robot_id not in ids:
            raise HTTPException(403, "해당 로봇 권한이 없습니다")
        return [robot_id]
    return ids


@router.get("/tracks")
def tracks(robot_id: str | None = None, from_ts: float | None = None,
           to_ts: float | None = None, db=Depends(get_db),
           user: User = Depends(current_user)):
    ids = _scoped_robot_ids(db, user, robot_id)
    q = db.query(Track).filter(Track.robot_id.in_(ids))
    if from_ts is not None:
        q = q.filter(Track.ts >= dt.datetime.fromtimestamp(from_ts, dt.UTC))
    if to_ts is not None:
        q = q.filter(Track.ts <= dt.datetime.fromtimestamp(to_ts, dt.UTC))
    return [{"robot_id": t.robot_id, "ts": t.ts.isoformat(), "x": t.x, "y": t.y,
             "yaw": t.yaw, "mode": t.mode}
            for t in q.order_by(Track.ts).limit(10000)]


@router.get("/events")
def events(robot_id: str | None = None, limit: int = 200, db=Depends(get_db),
           user: User = Depends(current_user)):
    ids = _scoped_robot_ids(db, user, robot_id)
    q = (db.query(Event).filter(Event.robot_id.in_(ids))
         .order_by(Event.id.desc()).limit(min(limit, 1000)))
    return [{"robot_id": e.robot_id, "ts": e.ts.isoformat(), "kind": e.kind,
             "severity": e.severity, "msg": e.msg} for e in q]


@router.get("/audit", dependencies=[Depends(require_min_role("admin"))])
def audit_rows(limit: int = 200, db=Depends(get_db)):
    q = db.query(AuditLog).order_by(AuditLog.id.desc()).limit(min(limit, 1000))
    return [{"ts": a.ts.isoformat(), "user_id": a.user_id, "role": a.role,
             "action": a.action, "target": a.target, "result": a.result,
             "detail": a.detail} for a in q]
