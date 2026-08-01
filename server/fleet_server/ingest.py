"""텔레메트리 → DB 수집. 중복 제거는 DB 유니크 제약으로 영속화(스펙 §3.3)."""
from __future__ import annotations

import datetime as dt

from sqlalchemy.exc import IntegrityError

from .models import Event, Track

_last_track_ts: dict[str, float] = {}          # 로봇별 마지막 저장 ts (1 Hz 다운샘플)


def _ts(payload: dict) -> dt.datetime:
    raw = payload.get("ts")
    if raw is None:
        return dt.datetime.now(dt.UTC)
    return dt.datetime.fromtimestamp(float(raw), dt.UTC)


def track(db, robot_id: str, payload: dict) -> bool:
    t = _ts(payload).timestamp()
    last = _last_track_ts.get(robot_id)
    if last is not None and (t - last) < 1.0:
        return False
    _last_track_ts[robot_id] = t
    db.add(Track(robot_id=robot_id, ts=_ts(payload),
                 x=float(payload.get("x", 0.0)), y=float(payload.get("y", 0.0)),
                 yaw=float(payload.get("yaw", 0.0)), mode=str(payload.get("mode", ""))))
    db.commit()
    return True


def event(db, robot_id: str, channel: str | None, seq: int | None,
          payload: dict) -> bool:
    row = Event(robot_id=robot_id, ts=_ts(payload), channel=channel, seq=seq,
                kind=str(payload.get("kind", "unknown")),
                severity=str(payload.get("severity", "info")),
                msg=str(payload.get("msg", ""))[:256], payload_json=payload)
    db.add(row)
    try:
        db.commit()
        return True
    except IntegrityError:                     # (robot, channel, seq) 중복 → 무해화
        db.rollback()
        return False
