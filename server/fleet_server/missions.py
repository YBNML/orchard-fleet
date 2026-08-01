"""임무 상태기계 — 스펙 §4.3 전이 표 그대로.

estop 은 임무를 PAUSED 로 보낸다. clear_estop 은 임무 이벤트가 아니다(래치만
해제) — 재개는 반드시 별도 resume 으로만 일어난다 (안전 서프라이즈 차단).
"""
from __future__ import annotations

import datetime as dt

from .models import Mission, MissionEvent


class InvalidTransition(Exception):
    pass


TRANSITIONS: dict[tuple[str, str], str] = {
    ("QUEUED", "start"): "RUNNING",
    ("QUEUED", "cancel"): "CANCELED",
    ("RUNNING", "pause"): "PAUSED",
    ("RUNNING", "estop"): "PAUSED",
    ("RUNNING", "complete"): "DONE",
    ("RUNNING", "cancel"): "CANCELED",
    ("RUNNING", "fail"): "FAILED",
    ("PAUSED", "resume"): "RUNNING",
    ("PAUSED", "cancel"): "CANCELED",
    ("PAUSED", "fail"): "FAILED",
}
_TERMINAL = {"DONE", "CANCELED", "FAILED"}


def create(db, *, robot_id: str, farm_id: int, spec: dict, created_by: int) -> Mission:
    ms = Mission(robot_id=robot_id, farm_id=farm_id, spec_json=spec,
                 created_by=created_by)
    db.add(ms)
    db.commit()
    return ms


def apply(db, mission: Mission, event: str, *, payload: dict | None = None) -> Mission:
    key = (mission.state, event)
    if key not in TRANSITIONS:
        raise InvalidTransition(f"{mission.state} 에서 {event} 불가")
    new = TRANSITIONS[key]
    now = dt.datetime.now(dt.UTC)
    if mission.state == "QUEUED" and new == "RUNNING":
        mission.started_at = now
    if new in _TERMINAL:
        mission.ended_at = now
    mission.state = new
    db.add(MissionEvent(mission_id=mission.id, kind=event,
                        payload_json=payload or {}))
    db.commit()
    return mission
