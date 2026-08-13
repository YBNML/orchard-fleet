"""AlleyLock — 임무 단위 통로 잠금(교통관리 v1, 스펙 ③ §2).

점유 규칙: 임무의 통로 집합 A 에 대해 패드 집합 P(A) = {(k,k+1) | k,k+1 이
A 에서 연속}. 두 점유가 충돌 ⇔ 통로 교집합 ≠ ∅ 또는 패드 교집합 ≠ ∅.
패드는 계단식 지형에서 통로 사이를 잇는 헤드랜드 선회 구간을 뜻한다 —
통로 자체가 겹치지 않아도 그 사이 선회 패드를 두 로봇이 동시에 쓰면
충돌이다.

**와일드카드(리뷰 라운드 1 — C1).** `alleys` 가 `None` 이면 "이 임무가 정확히
어느 통로를 도는지 서버는 모르지만, 전부일 수 있다"는 뜻이다(대시보드 "전체
정찰" 처럼 alleys 를 생략하는 요청 — 로봇이 전 통로를 자동으로 돈다). 이런
임무를 잠금 대상에서 빼면 "잠금 없이 발진되는 임무는 없다"는 전역 불변이
깨진다. 그래서 `None` 은 그 farm 의 다른 모든 점유와 무조건 충돌하는
센티널이다 — 즉 전체 통로를 잠근 것과 같다.

잠금 없이 발진되는 임무는 없다: 획득은 `POST /missions` 처리에서 임무
생성과 같은 트랜잭션 안에 있다(같은 `db` 세션). **획득에 성공하면 로봇에
보내기(발진, `await send_command`) 전에 반드시 커밋해야 한다** — 미커밋인
채로 await 를 건너면 그동안 다른 요청이 이 잠금을 못 보고 겹쳐 획득할 수
있다(리뷰 라운드 1 — C2). 그래서 `acquire` 는 flush 만 하고, 커밋 시점은
호출자(mission_routes.py)가 정한다.
"""
from __future__ import annotations

from .models import AlleyLock, Mission


def pads(alleys: list[int]) -> set[tuple[int, int]]:
    """연속쌍만 패드로 본다 — 비연속 통로 사이는 패드가 없다(따로 선회한다)."""
    s = sorted(alleys)
    return {(a, b) for a, b in zip(s, s[1:]) if b - a == 1}


def conflict(a: list[int] | None, b: list[int] | None) -> bool:
    """통로 교집합 또는 패드 교집합이 있으면 충돌. 어느 한쪽이라도 와일드카드
    (None — 통로 생략, 전체 점유)면 무조건 충돌."""
    if a is None or b is None:
        return True
    if set(a) & set(b):
        return True
    return bool(pads(a) & pads(b))


def _alleys_of(row: AlleyLock) -> list[int] | None:
    return list(row.alleys_json) if row.alleys_json is not None else None


class AlleyLocks:
    """alley_locks 테이블 위의 획득/해제/조회/복원. DB 세션만 받는다 — HTTP·
    FastAPI 지식이 없다(mission_routes.py 가 그 경계를 맡는다). 충돌 검사는
    항상 같은 farm_id 안에서만 한다 — 다른 농장(별개 과수원)의 통로 번호가
    우연히 같아도 서로 무관하다."""

    @staticmethod
    def list_active(session) -> list[dict]:
        rows = session.query(AlleyLock).all()
        return [{"mission_id": r.mission_id, "robot_id": r.robot_id,
                 "farm_id": r.farm_id, "alleys": _alleys_of(r)} for r in rows]

    @staticmethod
    def acquire(session, robot_id: str, mission_id: int, alleys: list[int] | None,
               farm_id: int) -> tuple[bool, str | None]:
        """원자 획득 — 호출자의 트랜잭션 안에서 실행되고 commit 하지 않는다
        (호출자가 발진 전에 커밋해야 한다 — 모듈 docstring C2 참고).

        같은 farm 의 기존 활성 잠금과 충돌 검사 후, 통과하면 이 임무의 잠금
        행을 만든다(flush 만 — commit 은 호출자 몫이라 실패 시 롤백 한 번으로
        아무 자취도 남지 않는다).
        """
        for row in session.query(AlleyLock).filter(AlleyLock.farm_id == farm_id):
            if row.mission_id == mission_id:
                continue
            if conflict(alleys, _alleys_of(row)):
                row_alleys = _alleys_of(row)
                return False, (f"통로 잠금 충돌 — robot={row.robot_id} "
                               f"mission#{row.mission_id} "
                               f"alleys={row_alleys if row_alleys is not None else '전체(와일드카드)'}")
        session.add(AlleyLock(
            mission_id=mission_id, robot_id=robot_id, farm_id=farm_id,
            alleys_json=(list(alleys) if alleys is not None else None),
            pads_json=([list(p) for p in pads(alleys)] if alleys is not None else None)))
        session.flush()
        return True, None

    @staticmethod
    def release(session, mission_id: int) -> None:
        """잠금 해제 — 없는 mission_id 는 조용히 통과(멱등)."""
        session.query(AlleyLock).filter(AlleyLock.mission_id == mission_id).delete()

    @staticmethod
    def restore(session) -> None:
        """기동 시 정합 확인 — RUNNING 임무인데 잠금 행이 없으면(비정상 종료로
        해제 훅을 못 탄 경우) spec_json 으로 되살린다. 이미 있으면 손대지
        않는다(멱등) — 정상 재기동에서는 항상 이미 있다. spec_json 에 "alleys"
        키가 아예 없으면 그 임무는 통로 생략(와일드카드) 요청이었다는 뜻이라
        와일드카드 잠금으로 되살린다."""
        locked_mission_ids = {mid for (mid,) in
                              session.query(AlleyLock.mission_id).all()}
        for ms in session.query(Mission).filter(Mission.state == "RUNNING"):
            if ms.id in locked_mission_ids:
                continue
            spec = ms.spec_json or {}
            if "alleys" in spec and not spec["alleys"]:
                continue                      # 빈 목록 — 정상 스펙이 아니다, 손대지 않는다
            alleys = spec.get("alleys")       # 키 없음 == 원 요청이 alleys 생략(와일드카드)
            session.add(AlleyLock(
                mission_id=ms.id, robot_id=ms.robot_id, farm_id=ms.farm_id,
                alleys_json=(list(alleys) if alleys is not None else None),
                pads_json=([list(p) for p in pads(alleys)] if alleys is not None else None)))
        session.flush()
