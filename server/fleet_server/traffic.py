"""AlleyLock — 임무 단위 통로 잠금(교통관리 v1, 스펙 ③ §2).

점유 규칙: 임무의 통로 집합 A 에 대해 패드 집합 P(A) = {(k,k+1) | k,k+1 이
A 에서 연속}. 두 점유가 충돌 ⇔ 통로 교집합 ≠ ∅ 또는 패드 교집합 ≠ ∅.
패드는 계단식 지형에서 통로 사이를 잇는 헤드랜드 선회 구간을 뜻한다 —
통로 자체가 겹치지 않아도 그 사이 선회 패드를 두 로봇이 동시에 쓰면
충돌이다.

잠금 없이 발진되는 임무는 없다: 획득은 `POST /missions` 처리에서 임무
생성과 같은 트랜잭션 안에 있다(같은 `db` 세션 — commit 은 호출자가 한다).
"""
from __future__ import annotations

from .models import AlleyLock, Mission


def pads(alleys: list[int]) -> set[tuple[int, int]]:
    """연속쌍만 패드로 본다 — 비연속 통로 사이는 패드가 없다(따로 선회한다)."""
    s = sorted(alleys)
    return {(a, b) for a, b in zip(s, s[1:]) if b - a == 1}


def conflict(a: list[int], b: list[int]) -> bool:
    """통로 교집합 또는 패드 교집합이 있으면 충돌."""
    if set(a) & set(b):
        return True
    return bool(pads(a) & pads(b))


class AlleyLocks:
    """alley_locks 테이블 위의 획득/해제/조회/복원. DB 세션만 받는다 — HTTP·
    FastAPI 지식이 없다(mission_routes.py 가 그 경계를 맡는다)."""

    @staticmethod
    def list_active(session) -> list[dict]:
        rows = session.query(AlleyLock).all()
        return [{"mission_id": r.mission_id, "robot_id": r.robot_id,
                 "alleys": list(r.alleys_json)} for r in rows]

    @staticmethod
    def acquire(session, robot_id: str, mission_id: int,
               alleys: list[int]) -> tuple[bool, str | None]:
        """원자 획득 — 호출자의 트랜잭션 안에서 실행되고 commit 하지 않는다.

        기존 활성 잠금 전부와 충돌 검사 후, 통과하면 이 임무의 잠금 행을
        만든다(flush 만 — commit 은 호출자 몫이라 실패 시 롤백 한 번으로
        아무 자취도 남지 않는다).
        """
        for row in session.query(AlleyLock).all():
            if row.mission_id == mission_id:
                continue
            if conflict(alleys, list(row.alleys_json)):
                return False, (f"통로 잠금 충돌 — robot={row.robot_id} "
                               f"mission#{row.mission_id} alleys={list(row.alleys_json)}")
        session.add(AlleyLock(mission_id=mission_id, robot_id=robot_id,
                              alleys_json=list(alleys),
                              pads_json=[list(p) for p in pads(alleys)]))
        session.flush()
        return True, None

    @staticmethod
    def release(session, mission_id: int) -> None:
        """잠금 해제 — 없는 mission_id 는 조용히 통과(멱등)."""
        session.query(AlleyLock).filter(AlleyLock.mission_id == mission_id).delete()

    @staticmethod
    def restore(session) -> None:
        """기동 시 정합 확인 — RUNNING 임무인데 잠금 행이 없으면(비정상 종료로
        해제 훅을 못 탄 경우) spec_json.alleys 로 되살린다. 이미 있으면 손대지
        않는다(멱등) — 정상 재기동에서는 항상 이미 있다."""
        locked_mission_ids = {mid for (mid,) in
                              session.query(AlleyLock.mission_id).all()}
        for ms in session.query(Mission).filter(Mission.state == "RUNNING"):
            if ms.id in locked_mission_ids:
                continue
            alleys = (ms.spec_json or {}).get("alleys")
            if not alleys:
                continue
            session.add(AlleyLock(mission_id=ms.id, robot_id=ms.robot_id,
                                  alleys_json=list(alleys),
                                  pads_json=[list(p) for p in pads(alleys)]))
        session.flush()
