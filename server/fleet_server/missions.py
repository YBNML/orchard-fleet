"""임무 상태기계 — 스펙 §4.3 전이 표 그대로.

estop 은 임무를 PAUSED 로 보낸다. clear_estop 은 임무 이벤트가 아니다(래치만
해제) — 재개는 반드시 별도 resume 으로만 일어난다 (안전 서프라이즈 차단).
"""
from __future__ import annotations

import datetime as dt

from sqlalchemy import update as sa_update

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
    """전이를 적용한다. 낙관적 가드(기대 상태를 조건으로 하는 UPDATE)로 레이스를
    막는다 — REST 핸들러의 await 구간에 로봇 보고(_sync_mission)가 끼어들어 먼저
    커밋해 버리면, 재개된 핸들러가 들고 있는 stale 객체로 무조건 UPDATE 해
    DONE→PAUSED 처럼 역행하는 사고를 방지한다.
    """
    key = (mission.state, event)
    if key not in TRANSITIONS:
        raise InvalidTransition(f"{mission.state} 에서 {event} 불가")
    new = TRANSITIONS[key]
    now = dt.datetime.now(dt.UTC)
    expected_state = mission.state
    values: dict = {"state": new}
    if expected_state == "QUEUED" and new == "RUNNING":
        values["started_at"] = now
    if new in _TERMINAL:
        values["ended_at"] = now

    result = db.execute(
        sa_update(Mission)
        .where(Mission.id == mission.id, Mission.state == expected_state)
        .values(**values)
        .execution_options(synchronize_session=False))
    if result.rowcount == 0:
        # 다른 세션이 먼저 전이시켰다 — stale 객체로 덮어쓰지 않는다. 호출자에게는
        # DB 의 실제 최신 상태를 보여준다.
        db.refresh(mission)
        raise InvalidTransition(
            f"임무#{mission.id} 상태가 동시에 변경되었습니다 "
            f"(기대={expected_state}, 실제={mission.state})")

    for k, v in values.items():
        setattr(mission, k, v)
    db.add(MissionEvent(mission_id=mission.id, kind=event,
                        payload_json=payload or {}))
    db.commit()
    return mission
