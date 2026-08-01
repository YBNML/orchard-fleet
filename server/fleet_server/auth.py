"""인증 코어 — Argon2 해시 + 역할 매트릭스(D9, fail-closed).

로봇측 orchard_sim.link.protocol 의 매트릭스와 의미가 같아야 한다(2중 판정).
차이: D9 반영으로 estop·stop_all 이 observer 까지 내려간다.
"""
from __future__ import annotations

from argon2 import PasswordHasher
from argon2.exceptions import Argon2Error

_ph = PasswordHasher()

ROLE_OBSERVER = "observer"
ROLE_OPERATOR = "operator"
ROLE_ADMIN = "admin"
ROLE_RANK = {ROLE_OBSERVER: 0, ROLE_OPERATOR: 1, ROLE_ADMIN: 2}

# D9: 세우는 건 누구나, 푸는 건 admin 만
ROLE_REQUIRED = {
    "estop": ROLE_OBSERVER,
    "stop_all": ROLE_OBSERVER,
    "ping": ROLE_OBSERVER,
    "mission_start": ROLE_OPERATOR,
    "mission_pause": ROLE_OPERATOR,
    "mission_resume": ROLE_OPERATOR,
    "mission_cancel": ROLE_OPERATOR,
    "teleop": ROLE_OPERATOR,
    "clear_estop": ROLE_ADMIN,
    "set_mode": ROLE_ADMIN,
}


def hash_password(pw: str) -> str:
    return _ph.hash(pw)


def verify_password(pw: str, pw_hash: str) -> bool:
    try:
        return _ph.verify(pw_hash, pw)
    except (Argon2Error, ValueError):
        return False


def normalize_role(v) -> str:
    return v if v in ROLE_RANK else ROLE_OBSERVER      # 미지 역할 → observer 강등


def authorize(role, action) -> bool:
    need = ROLE_REQUIRED.get(action, ROLE_ADMIN)        # 미지 명령 → admin 요구
    return ROLE_RANK[normalize_role(role)] >= ROLE_RANK[need]


import datetime as dt
import secrets

from .models import AuthSession, User


def create_session(db, user: User, ttl_s: int) -> AuthSession:
    row = AuthSession(id=secrets.token_urlsafe(32), user_id=user.id,
                      csrf=secrets.token_urlsafe(16),
                      expires_at=dt.datetime.now(dt.UTC) + dt.timedelta(seconds=ttl_s))
    db.add(row)
    db.commit()
    return row


def delete_session(db, token: str) -> None:
    db.query(AuthSession).filter(AuthSession.id == token).delete()
    db.commit()
