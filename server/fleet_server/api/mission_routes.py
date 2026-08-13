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


# 발진 가능한 활성 임무로 치는 상태 — 같은 로봇에 2번째 요청을 막는 기준이자
# (mission_verb 의 오프라인-취소 특례도 이 목록 소속을 전제한다), 로봇당
# 1개만 허용한다는 불변의 근거다. QUEUED_LOCK 도 포함한다(리뷰 라운드 1 —
# I7): 아니면 같은 로봇에 QUEUED_LOCK 이 계속 쌓일 수 있다.
_ACTIVE_MISSION_STATES = ["QUEUED", "QUEUED_LOCK", "RUNNING", "PAUSED"]


def _mission_out(ms: Mission) -> dict:
    return {"id": ms.id, "robot_id": ms.robot_id, "farm_id": ms.farm_id,
            "state": ms.state, "spec": ms.spec_json,
            "created_at": iso_utc(ms.created_at),
            "started_at": iso_utc(ms.started_at),
            "ended_at": iso_utc(ms.ended_at)}


def _alleys_sequence_valid(alleys: list[int]) -> bool:
    """리뷰 라운드 1 M11 — REST 입력 검증. pads() 는 정렬 후 연속쌍만 패드로
    보므로, 요청에 적힌 순서상 인접하지 않은 통로가 섞이면([0,2,4] 처럼)
    실제 로봇이 건너는 헤드랜드 패드가 잠금 계산에서 통째로 빠진다. 그런
    임무는 애초에 발진 가능한 요청이 아니어야 한다 — 로봇 계약이 아니라
    서버 REST 검증이다(빈 목록도 여기서 함께 막는다)."""
    if not alleys:
        return False
    return all(abs(b - a) == 1 for a, b in zip(alleys, alleys[1:]))


@router.post("/missions", dependencies=[_operator, _csrf])
async def create_mission(body: MissionBody, request: Request, db=Depends(get_db),
                         user: User = Depends(current_user)):
    if body.alleys is not None and not _alleys_sequence_valid(body.alleys):
        audit.record(db, action="mission_start", result="rejected", user_id=user.id,
                     role=user.role, target=body.robot_id,
                     detail=f"통로 목록이 인접 순서가 아님 alleys={body.alleys}")
        raise HTTPException(400, "통로 목록은 순서상 인접한 통로만 연속으로 넣을 수 있습니다")
    robot = _scoped_robot(db, user, body.robot_id, action="mission_start")
    existing = (db.query(Mission)
                .filter(Mission.robot_id == robot.id,
                        Mission.state.in_(_ACTIVE_MISSION_STATES))
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
    # 잠금 없이 나가는 임무는 없다(C1) — alleys 를 생략한 임무(work 전 통로
    # 자동)도 예외가 아니다. 서버는 그 임무가 실제로 어느 통로를 도는지 모르니
    # None(와일드카드)으로 그 farm 전체를 잠근다(traffic.py 모듈 docstring).
    ok, reason = traffic.AlleyLocks.acquire(db, robot.id, ms.id, body.alleys, robot.farm_id)
    if not ok:                                  # 발진 대신 QUEUED_LOCK
        ms = missions.apply(db, ms, "lock_conflict", payload={"reason": reason})
        audit.record(db, action="mission_start", result="rejected", user_id=user.id,
                     role=user.role, target=robot.id, detail=reason)
        return {**_mission_out(ms), "lock_reason": reason}   # 대시보드 토스트용(I7)
    # C2 — 로봇에 보내기(발진) 전에 반드시 커밋한다. 잠금을 미커밋인 채로
    # 아래 await 를 건너면, 그 동안 다른 요청이 이 잠금을 못 보고 겹쳐
    # 획득할 수 있다(둘 다 QUEUED) — 또는 SQLite 가 그 요청의 쓰기를 막아
    # busy_timeout 뒤 500 을 낸다. 커밋해 두면 발진 실패(오프라인) 시에도
    # 잠금은 이미 진짜로 걸려 있으므로, 아래 cancel 경로가 정상적으로
    # (release 훅을 태워) 풀어준다.
    db.commit()
    payload: dict = {"mission_id": ms.id}
    if body.alleys is not None:                # 생략 시 키 자체를 넣지 않음 — 로봇이 전 통로 자동
        payload["alleys"] = body.alleys
    if body.work is not None:                  # 서버는 검증하지 않고 그대로 전달
        payload["work"] = body.work
    result = await fleet.send_command(robot.id, f"m{ms.id}", "mission_start", payload)
    if result == "offline":                    # 오프라인 → 즉시 실패 + 잔재 제거(잠금도 release)
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
    if result == "offline":
        if verb == "cancel":
            # C3 — 오프라인 로봇의 cancel 은 로컬 전이로 허용한다. 예전에는
            # 여기서도 409 로 막았는데, 그러면 링크가 끊긴 로봇의 RUNNING
            # 임무를 아무도 취소할 수 없어 그 통로가 영구히 잠긴다 — 서버
            # 재기동 restore() 는 RUNNING 인 채로 남은 그 임무의 잠금을
            # 오히려 되살려 좀비를 고정시킨다. 로봇에는 실제로 전달되지
            # 않았으므로(delivery="not_sent") 로봇이 링크를 되찾았을 때
            # 스스로 임무를 계속 돌 수는 있다 — 그 경우 다음 텔레메트리가
            # 서버 기대와 어긋나면 사람이 알아챌 몫이다(이 이상은 v1 범위 밖).
            missions.apply(db, ms, verb)
            audit.record(db, action=action, result="accepted", user_id=user.id,
                         role=user.role, target=ms.robot_id,
                         detail=f"mission={ms.id} 로봇 오프라인 — 로컬 취소")
            return {**_mission_out(ms), "delivery": "not_sent"}
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
    scope = farm_scope(db, user)                # I6 — list_missions 와 같은 관례
    rows = traffic.AlleyLocks.list_active(db)
    if scope is not None:
        rows = [r for r in rows if r["farm_id"] in scope]
    return rows
