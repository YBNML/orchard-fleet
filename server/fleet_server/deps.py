from __future__ import annotations

import datetime as dt
import secrets

from fastapi import Depends, HTTPException, Request

from .auth import ROLE_RANK, normalize_role
from .models import AuthSession, User

SESSION_COOKIE = "fleet_session"


def get_db(request: Request):
    db = request.app.state.session_factory()
    try:
        yield db
    finally:
        db.close()


def _session_pair(request: Request, db) -> tuple[AuthSession, User] | None:
    token = request.cookies.get(SESSION_COOKIE)
    if not token:
        return None
    row = db.get(AuthSession, token)
    if row is None:
        return None
    exp = row.expires_at
    if exp.tzinfo is None:                       # SQLite 는 tz 를 벗겨서 돌려준다
        exp = exp.replace(tzinfo=dt.UTC)
    if exp < dt.datetime.now(dt.UTC):
        db.delete(row); db.commit()
        return None
    user = db.get(User, row.user_id)
    if user is None or user.disabled:
        return None
    return row, user


def current_session(request: Request, db=Depends(get_db)) -> AuthSession:
    pair = _session_pair(request, db)
    if pair is None:
        raise HTTPException(401, "로그인이 필요합니다")
    return pair[0]


def current_user(request: Request, db=Depends(get_db)) -> User:
    pair = _session_pair(request, db)
    if pair is None:
        raise HTTPException(401, "로그인이 필요합니다")
    return pair[1]


def require_min_role(min_role: str):
    def dep(user: User = Depends(current_user)) -> User:
        if ROLE_RANK[normalize_role(user.role)] < ROLE_RANK[min_role]:
            raise HTTPException(403, "권한이 없습니다")
        return user
    return dep


def csrf_protect(request: Request, sess: AuthSession = Depends(current_session)) -> None:
    header = request.headers.get("X-CSRF") or ""
    if not secrets.compare_digest(header, sess.csrf):
        raise HTTPException(403, "CSRF 토큰 불일치")


def farm_scope(db, user: User) -> set[int] | None:
    """admin 은 None(전체), 그 외는 배정 농장 id 집합."""
    if normalize_role(user.role) == "admin":
        return None
    from .models import UserFarm
    rows = db.query(UserFarm.farm_id).filter(UserFarm.user_id == user.id).all()
    return {r[0] for r in rows}
