"""이벤트 보존정책 — TTL(EVENT_TTL_DAYS, 기본 7일) 초과 행 정리 (Task 6).

pong 미기록은 `fleet.service.EVENT_KIND_SKIP` 이 입구에서 막는다(스펙 판단 —
링크 판정은 어댑터 메모리로 충분하다). 여기는 그 뒤에도 쌓이는 events 행이
무한정 늘지 않게 나이로 걷어낸다. `BTEngine`(bt/engine.py)의 1 Hz 틱과 같은
start/stop asyncio 태스크 골격을 쓴다 — 다만 이쪽은 상태가 없는 멱등 작업이라
(지울 행이 없으면 매 실행이 그저 no-op) 재기동 복원 로직이 필요 없다.

**리뷰 I2(정책 역전) — 안전·수명주기 kind 는 더 길게 보존한다.** 실측(T6
§8.4): pong 미기록 배치 뒤 첫 TTL 실행이 지운 122행은 **전부** 안전·감사
기록(estop 계열·assistance·resolved·link_lost·mission_*·cmd_result)이었고
pong 은 0건이었다 — pong 이 소스에서 걸러지고 나니 TTL 이 용량은 못 줄이면서
사후조사 기록만 지우는 역전이 벌어졌다. 컨트롤러 룰링: 스펙의
`EVENT_TTL_DAYS=7` 기본값은 그대로 두되, 이 kind 들에는 별도로 더 긴
`EVENT_TTL_SAFE_DAYS`(기본 90일)를 적용한다. 스펙 문언에서의 이탈이므로
findings 에 근거(§8.4)와 함께 남긴다 — 사용자 검토 대상.
"""
from __future__ import annotations

import asyncio
import contextlib
import datetime as dt
import logging

from sqlalchemy import or_

from .models import Event

log = logging.getLogger("fleet_server.event_retention")

PERIOD_S = 24 * 3600.0          # 24시간 주기

# 안전·수명주기 kind — 표준 TTL 이 아니라 EVENT_TTL_SAFE_DAYS 를 따른다.
# 목록은 리뷰(I2)가 지정한 그대로다: estop 계열(estop·estop_cleared·
# estop_clear·estop_clear_requested 등 "estop" 로 시작하는 전부)과
# mission_*(mission_started·mission_done·mission_cancelled·mission_resumed
# 등 "mission_" 로 시작하는 전부)는 접두어로, 나머지 넷은 정확히 일치로 본다.
_SAFE_KINDS_EXACT = ("assistance", "resolved", "denied", "link_lost", "cmd_result")
_SAFE_KIND_PREFIXES = ("estop", "mission_")


def _safe_kind_clause():
    # 리뷰 라운드 2 (N3) — SQLAlchemy 의 Column.startswith() 는 기본으로
    # LIKE 와일드카드(%, _)를 이스케이프하지 않는다. "mission_" 의 "_" 는
    # LIKE 문법에서 "아무 문자 하나"를 뜻하므로, autoescape 없이는
    # `kind LIKE 'mission_%'` 가 "mission" 뒤에 밑줄 아닌 다른 한 글자가
    # 와도(예: 가상의 kind "missionX") 안전 유형으로 잘못 인식한다 —
    # 지금 실제 kind 값들과는 우연히 안 부딪혔을 뿐 구조적으로 틀린
    # 필터였다. autoescape=True 로 "_"·"%" 를 리터럴로 고정한다.
    return or_(Event.kind.in_(_SAFE_KINDS_EXACT),
               *(Event.kind.startswith(p, autoescape=True) for p in _SAFE_KIND_PREFIXES))


def purge_expired(db, ttl_days: int, safe_ttl_days: int = 90) -> int:
    """`ts` 가 만료된 events 행을 지운다. 지운 행 수를 돌려준다.

    안전·수명주기 kind(`_safe_kind_clause`)는 `safe_ttl_days` 를, 나머지는
    `ttl_days` 를 각각 독립적으로 따른다. 둘 다 `<=0` 은 "무제한 삭제"가
    아니라 **그 축의 비활성**이다 — 설정 실수로 테이블 전체(또는 안전
    기록 전체)가 지워지는 사고를 막는다(fail-closed, protocol.py
    ROLE_REQUIRED_DEFAULT 와 같은 방향)."""
    now = dt.datetime.now(dt.UTC)
    safe = _safe_kind_clause()
    total = 0
    if ttl_days > 0:
        cutoff = now - dt.timedelta(days=ttl_days)
        total += db.query(Event).filter(Event.ts < cutoff, ~safe).delete(
            synchronize_session=False)
    if safe_ttl_days > 0:
        cutoff_safe = now - dt.timedelta(days=safe_ttl_days)
        total += db.query(Event).filter(Event.ts < cutoff_safe, safe).delete(
            synchronize_session=False)
    db.commit()
    return total


class RetentionTask:
    """기동 시 1회 + `period_s`(기본 24h) 주기로 `purge_expired` 를 돈다."""

    def __init__(self, session_factory, ttl_days: int, *,
                 safe_ttl_days: int = 90, period_s: float = PERIOD_S):
        self._factory = session_factory
        self.ttl_days = ttl_days
        self.safe_ttl_days = safe_ttl_days
        self.period_s = period_s
        self._task: asyncio.Task | None = None

    def run_once(self) -> int:
        with self._factory() as db:
            n = purge_expired(db, self.ttl_days, self.safe_ttl_days)
        if n:
            # Minor 6 — I2 수정과 함께: 안전 기록까지 지우는 축이 생긴 만큼
            # 삭제는 조용한 INFO 가 아니라 눈에 띄는 WARNING 으로 남긴다.
            log.warning("이벤트 보존정책 — %d행 정리(TTL %d일 / 안전유형 %d일 초과)",
                       n, self.ttl_days, self.safe_ttl_days)
        return n

    async def start(self) -> None:
        if self._task is not None and not self._task.done():
            return
        self._task = asyncio.create_task(self._run(), name="event-retention")

    async def _run(self) -> None:
        while True:
            try:
                self.run_once()
            except asyncio.CancelledError:
                raise
            except Exception:               # 정리 실패로 서버 심장이 멈추면 안 된다
                log.exception("이벤트 보존정책 정리 실패")
            await asyncio.sleep(self.period_s)

    async def stop(self) -> None:
        task, self._task = self._task, None
        if task is None:
            return
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task
