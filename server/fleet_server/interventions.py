"""개입 큐 서비스 — 로봇의 호출을 티켓으로 바꾸고, 처리 시간을 잰다.

핵심 규칙 두 가지.

1. **같은 사유의 열린 건은 하나만 둔다.** 로봇이 장애물 앞에서 1초마다 같은
   이벤트를 올려도 큐가 폭주하면 안 된다. 반복은 count 로만 쌓는다.
2. **처리 시간은 두 개를 잰다.** OPEN→ACKED(사람이 붙기까지)와
   OPEN→RESOLVED(해소까지). 평균 대신 p50/p95 를 쓴다 — 평균은 꼬리를 숨긴다.
"""
from __future__ import annotations

import datetime as dt

from . import stopcodes
from .models import Intervention

ST_OPEN, ST_ACKED, ST_RESOLVED, ST_ESCALATED = "OPEN", "ACKED", "RESOLVED", "ESCALATED"
_ACTIVE = (ST_OPEN, ST_ACKED)


def _now() -> dt.datetime:
    return dt.datetime.now(dt.UTC)


def open_or_bump(db, *, robot_id: str, farm_id: int, code: str,
                 msg: str = "", severity: str = "warn",
                 context: dict | None = None) -> Intervention | None:
    """로봇 호출을 접수한다. 개입으로 셀 사유가 아니면 아무것도 만들지 않는다."""
    sc = stopcodes.get(code)
    if not sc.needs_operator:
        return None
    existing = (db.query(Intervention)
                .filter(Intervention.robot_id == robot_id,
                        Intervention.code == sc.code,
                        Intervention.state.in_(_ACTIVE))
                .order_by(Intervention.id.desc()).first())
    if existing is not None:                    # 같은 사유 반복 — 큐를 늘리지 않는다
        ctx = dict(existing.context_json or {})
        ctx["repeat"] = int(ctx.get("repeat", 0)) + 1
        existing.context_json = ctx
        db.commit()
        return existing
    row = Intervention(robot_id=robot_id, farm_id=farm_id, code=sc.code,
                       category=sc.category, severity=severity,
                       needs_site_visit=sc.needs_site_visit,
                       msg=(msg or sc.label)[:256], context_json=context or {})
    db.add(row)
    db.commit()
    return row


def ack(db, row: Intervention, user_id: int | None) -> Intervention:
    if row.state == ST_OPEN:
        row.state, row.acked_at, row.acked_by = ST_ACKED, _now(), user_id
        db.commit()
    return row


def resolve(db, row: Intervention, user_id: int | None, note: str = "") -> Intervention:
    if row.state in _ACTIVE:
        if row.acked_at is None:                # 바로 해결해도 응답 시각은 남긴다
            row.acked_at, row.acked_by = _now(), user_id
        row.state, row.resolved_at, row.resolved_by = ST_RESOLVED, _now(), user_id
        row.note = (note or row.note)[:256]
        db.commit()
    return row


def escalate(db, row: Intervention, user_id: int | None, note: str = "") -> Intervention:
    """원격으로 못 푼다 — 사람이 기체까지 가야 한다."""
    if row.state in _ACTIVE:
        row.state = ST_ESCALATED
        row.needs_site_visit = True
        row.note = (note or row.note)[:256]
        row.acked_at = row.acked_at or _now()
        row.acked_by = row.acked_by or user_id
        db.commit()
    return row


def auto_resolve(db, robot_id: str, code: str) -> int:
    """사유가 사라졌다고 로봇이 알려오면 열린 건을 닫는다 (사람 손 없이)."""
    rows = (db.query(Intervention)
            .filter(Intervention.robot_id == robot_id,
                    Intervention.code == stopcodes.get(code).code,
                    Intervention.state.in_(_ACTIVE)).all())
    for r in rows:
        r.state, r.resolved_at = ST_RESOLVED, _now()
        r.note = (r.note or "로봇이 스스로 해소")[:256]
    if rows:
        db.commit()
    return len(rows)


def open_count(db, farm_ids=None) -> int:
    q = db.query(Intervention).filter(Intervention.state.in_(_ACTIVE))
    if farm_ids is not None:
        q = q.filter(Intervention.farm_id.in_(farm_ids))
    return q.count()


def out(row: Intervention) -> dict:
    from .timeutil import iso_utc
    return dict(id=row.id, robot_id=row.robot_id, farm_id=row.farm_id,
                code=row.code, label=stopcodes.get(row.code).label,
                category=row.category, severity=row.severity,
                needs_site_visit=row.needs_site_visit, state=row.state,
                msg=row.msg, note=row.note,
                repeat=int((row.context_json or {}).get("repeat", 0)),
                opened_at=iso_utc(row.opened_at), acked_at=iso_utc(row.acked_at),
                resolved_at=iso_utc(row.resolved_at))
