from __future__ import annotations

import time

from fastapi import APIRouter, Depends, HTTPException, Request, Response
from pydantic import BaseModel

from .. import audit, auth
from ..deps import SESSION_COOKIE, csrf_protect, current_session, current_user, get_db
from ..models import User

router = APIRouter(prefix="/auth", tags=["auth"])


class LoginBody(BaseModel):
    login: str
    password: str


def _user_out(u: User) -> dict:
    return {"login": u.login, "role": u.role, "display_name": u.display_name}


@router.post("/login")
def login(body: LoginBody, request: Request, response: Response, db=Depends(get_db)):
    settings = request.app.state.settings
    user = db.query(User).filter(User.login == body.login, ~User.disabled).first()
    if user is None or not auth.verify_password(body.password, user.pw_hash):
        audit.record(db, action="login", result="rejected", target=body.login,
                     detail="비밀번호 불일치 또는 없는 계정")
        time.sleep(settings.login_delay_s)              # 실패 지연 (스펙 §5)
        raise HTTPException(401, "아이디 또는 비밀번호가 틀립니다")
    row = auth.create_session(db, user, settings.session_ttl_s)
    response.set_cookie(SESSION_COOKIE, row.id, httponly=True, samesite="strict",
                        max_age=settings.session_ttl_s)
    audit.record(db, action="login", result="accepted", user_id=user.id,
                 role=user.role, target=user.login)
    return {"csrf": row.csrf, "user": _user_out(user)}


@router.post("/logout", dependencies=[Depends(csrf_protect)])
def logout(request: Request, response: Response, db=Depends(get_db),
           sess=Depends(current_session)):
    auth.delete_session(db, sess.id)
    response.delete_cookie(SESSION_COOKIE)
    return {"ok": True}


@router.get("/me")
def me(user: User = Depends(current_user), sess=Depends(current_session)):
    return {"csrf": sess.csrf, "user": _user_out(user)}
