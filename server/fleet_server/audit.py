"""감사 기록 — 모든 명령·관리 행위의 수락/거부를 DB 에 남긴다 (스펙 S7).

텔레옵은 세션 단위(시작·종료·거부)로만 기록한다 — 20 Hz 개별 지령 제외.
"""
from __future__ import annotations

import re

from .models import AuditLog

_MASK = re.compile(
    r'("?(?:token|password|pw|secret)"?\s*[:=]\s*)("(?:[^"\\]|\\.)*"|[^",}\s]+)',
    re.I)
_CLIP = 160


def _sanitize(detail: str) -> str:
    detail = _MASK.sub(r"\1***", detail)
    return detail[:_CLIP]


def record(db, *, action: str, result: str, user_id: int | None = None,
           role: str = "", target: str = "", detail: str = "") -> None:
    db.add(AuditLog(action=action, result=result, user_id=user_id,
                    role=role, target=target[:128], detail=_sanitize(detail)))
    db.commit()
