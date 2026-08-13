"""이벤트 보존정책 — TTL(EVENT_TTL_DAYS, 기본 7일) 초과 행 정리 (Task 6).

pong 미기록은 `fleet.service.EVENT_KIND_SKIP` 이 입구에서 막는다(스펙 판단 —
링크 판정은 어댑터 메모리로 충분하다). 여기는 그 뒤에도 쌓이는 events 행이
무한정 늘지 않게 나이로 걷어낸다. `BTEngine`(bt/engine.py)의 1 Hz 틱과 같은
start/stop asyncio 태스크 골격을 쓴다 — 다만 이쪽은 상태가 없는 멱등 작업이라
(지울 행이 없으면 매 실행이 그저 no-op) 재기동 복원 로직이 필요 없다.
"""
from __future__ import annotations

import asyncio
import contextlib
import datetime as dt
import logging

from .models import Event

log = logging.getLogger("fleet_server.event_retention")

PERIOD_S = 24 * 3600.0          # 24시간 주기


def purge_expired(db, ttl_days: int) -> int:
    """`ts` 가 `ttl_days` 보다 오래된 events 행을 지운다. 지운 행 수를 돌려준다.

    `ttl_days<=0` 은 "무제한 삭제"가 아니라 **비활성**이다 — 설정 실수(0 또는
    음수)로 테이블 전체가 지워지는 사고를 막는다(fail-closed, protocol.py
    ROLE_REQUIRED_DEFAULT 와 같은 방향)."""
    if ttl_days <= 0:
        return 0
    cutoff = dt.datetime.now(dt.UTC) - dt.timedelta(days=ttl_days)
    n = db.query(Event).filter(Event.ts < cutoff).delete(synchronize_session=False)
    db.commit()
    return n


class RetentionTask:
    """기동 시 1회 + `period_s`(기본 24h) 주기로 `purge_expired` 를 돈다."""

    def __init__(self, session_factory, ttl_days: int, *, period_s: float = PERIOD_S):
        self._factory = session_factory
        self.ttl_days = ttl_days
        self.period_s = period_s
        self._task: asyncio.Task | None = None

    def run_once(self) -> int:
        with self._factory() as db:
            n = purge_expired(db, self.ttl_days)
        if n:
            log.info("이벤트 보존정책 — %d행 정리(TTL %d일 초과)", n, self.ttl_days)
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
