"""임무 발진·승격·검증 동사의 **공용 경로** — REST 와 BT 엔진이 같은 함수를 쓴다.

T4 가 확정한 불변(비인접 통로 400 검증, "잠금 없이 발진되는 임무는 없다",
발진 전 커밋(C2), 오프라인 즉시 실패, 오프라인 cancel 로컬 전이(C3))은 전부
`POST /missions` 핸들러 안에 있었다. BT Action 이 그것을 우회해 자체 생산자를
두면 그 불변이 조용히 깨진다 — 그래서 핵심 로직을 여기로 끌어내고, HTTP 층
(mission_routes)은 인가·감사·상태코드 변환만 남긴다.

이 모듈은 FastAPI 를 import 하지 않는다. 실패는 `MissionOpError(status,
message, detail)` 로 알리고, 라우터가 그것을 HTTPException 과 감사 기록으로
번역한다(status 는 라우터가 그대로 쓰라고 실어 보내는 값이다).
"""
from __future__ import annotations

from . import missions, traffic
from .models import Mission, Robot

# 로봇에 발진 가능한 "활성" 임무로 치는 상태 — 로봇당 1개 불변의 근거.
# QUEUED_LOCK 도 활성이다(T4 I7): 아니면 같은 로봇에 잠금 대기가 쌓인다.
ACTIVE_MISSION_STATES = ["QUEUED", "QUEUED_LOCK", "RUNNING", "PAUSED"]

EVENT_BY_VERB = {"pause": "mission_pause", "resume": "mission_resume",
                 "cancel": "mission_cancel"}


class MissionOpError(Exception):
    """공용 경로의 거부 — status 는 REST 응답 코드, detail 은 감사 기록용."""

    def __init__(self, status: int, message: str, detail: str = ""):
        super().__init__(message)
        self.status = status
        self.message = message
        self.detail = detail or message


def alleys_sequence_valid(alleys: list[int] | None) -> bool:
    """T4 M11 — pads() 는 정렬 후 연속쌍만 패드로 보므로, 요청 순서상 인접하지
    않은 통로가 섞이면([0,2,4] 처럼) 실제로 건너는 헤드랜드 패드가 잠금 계산에서
    통째로 빠진다. 그런 임무는 애초에 발진 가능한 요청이 아니다(빈 목록도 함께
    막는다). alleys=None(생략 — 전 통로 자동)은 검증 대상이 아니다."""
    if alleys is None:
        return True
    if not alleys:
        return False
    return all(abs(b - a) == 1 for a, b in zip(alleys, alleys[1:]))


def dispatch_payload(mission: Mission) -> dict:
    """로봇에 보내는 mission_start payload — spec 에 없는 키는 넣지 않는다
    (alleys 키 자체가 없어야 로봇이 전 통로를 자동으로 돈다)."""
    spec = mission.spec_json or {}
    payload: dict = {"mission_id": mission.id}
    if "alleys" in spec:
        payload["alleys"] = spec["alleys"]
    if "work" in spec:
        payload["work"] = spec["work"]
    return payload


async def create_and_dispatch(db, fleet, *, robot: Robot, alleys: list[int] | None,
                              work: dict | None, created_by: int
                              ) -> tuple[Mission, str | None]:
    """임무 생성 + 잠금 획득 + 발진. 반환 (임무, 잠금대기 사유|None).

    사유가 채워져 돌아오면 그 임무는 QUEUED_LOCK 이고 로봇에는 아무것도
    나가지 않았다(호출자가 대기시키거나 정리한다). 거부는 예외로 알린다.
    """
    if not alleys_sequence_valid(alleys):
        raise MissionOpError(
            400, "통로 목록은 순서상 인접한 통로만 연속으로 넣을 수 있습니다",
            f"통로 목록이 인접 순서가 아님 alleys={alleys}")
    existing = (db.query(Mission)
                .filter(Mission.robot_id == robot.id,
                        Mission.state.in_(ACTIVE_MISSION_STATES))
                .first())
    if existing is not None:              # 로봇당 활성 임무는 1개만 (레이스·오귀속 방지)
        raise MissionOpError(409, "해당 로봇에 이미 활성 임무가 있습니다",
                             f"활성 임무 이미 존재 mission={existing.id}")
    spec: dict = {}
    if alleys is not None:
        spec["alleys"] = list(alleys)
    if work is not None:
        spec["work"] = work
    ms = missions.create(db, robot_id=robot.id, farm_id=robot.farm_id,
                         spec=spec, created_by=created_by)
    # 잠금 없이 나가는 임무는 없다(C1) — alleys 생략 임무는 None(와일드카드)으로
    # 그 farm 전체를 잠근다(traffic.py 모듈 docstring).
    ok, reason = traffic.AlleyLocks.acquire(db, robot.id, ms.id, alleys, robot.farm_id)
    if not ok:
        return missions.apply(db, ms, "lock_conflict", payload={"reason": reason}), reason
    # C2 — 로봇에 보내기 전에 반드시 커밋한다(미커밋 잠금은 다른 요청에 안 보인다).
    db.commit()
    result = await fleet.send_command(robot.id, f"m{ms.id}", "mission_start",
                                      dispatch_payload(ms))
    if result == "offline":               # 오프라인 → 즉시 실패 + 잔재 제거(잠금 release)
        missions.apply(db, ms, "cancel")
        raise MissionOpError(409, "로봇이 오프라인입니다", "로봇 오프라인")
    return ms, None


async def promote_locked(db, fleet, mission: Mission) -> bool:
    """QUEUED_LOCK 임무의 승격 — 잠금 재획득에 성공하면 QUEUED 로 되돌리고 발진.

    서버에는 이 경로가 없었다(T4 이관 (c)): REST 는 잠금 충돌 시 QUEUED_LOCK 을
    만들어 두고 손을 뗐고, 정리 수단은 cancel 뿐이었다. 겹치는 임무가 선행
    임무 종료 후 스스로 출발하려면 누군가 매 틱 재시도해야 한다 — BT 엔진이다.
    발진 규칙은 create_and_dispatch 와 같다(잠금 선커밋 후 send).
    """
    if mission.state != "QUEUED_LOCK":
        return False
    alleys = (mission.spec_json or {}).get("alleys")
    ok, _reason = traffic.AlleyLocks.acquire(db, mission.robot_id, mission.id,
                                             alleys, mission.farm_id)
    if not ok:
        return False
    missions.apply(db, mission, "lock_acquired")      # QUEUED 로 복귀(내부에서 커밋)
    result = await fleet.send_command(mission.robot_id, f"m{mission.id}",
                                      "mission_start", dispatch_payload(mission))
    if result == "offline":
        missions.apply(db, mission, "cancel")         # 잔재 제거 — 잠금도 함께 풀린다
        return False
    return True


async def apply_verb(db, fleet, mission: Mission, verb: str) -> str:
    """pause/resume/cancel — 전달 후 전이. 반환 "sent" | "not_sent"(오프라인 cancel).

    C3 — 오프라인 로봇의 cancel 만 로컬 전이로 허용한다. 아니면 링크가 끊긴
    로봇의 RUNNING 임무를 아무도 취소할 수 없어 통로가 영구히 잠기고, 재기동
    restore() 가 그 좀비 잠금을 되살린다.
    """
    if (mission.state, verb) not in missions.TRANSITIONS:   # 커밋 없이 사전 검사
        raise MissionOpError(409, f"{mission.state} 에서 {verb} 불가",
                             f"mission={mission.id} 상태={mission.state} 전이불가")
    result = await fleet.send_command(mission.robot_id, f"m{mission.id}-{verb}",
                                      EVENT_BY_VERB[verb], {"mission_id": mission.id})
    if result == "offline":
        if verb != "cancel":
            raise MissionOpError(409, "로봇이 오프라인입니다",
                                 f"mission={mission.id} 로봇 오프라인")
        missions.apply(db, mission, verb)
        return "not_sent"
    missions.apply(db, mission, verb)                 # "sent" 확인 후에만 상태 전이 커밋
    return result
