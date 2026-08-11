"""인증 코어 — Argon2 해시 + 역할 매트릭스(D9, fail-closed).

로봇측 robomw.link.protocol(ROLE_REQUIRED) 의 매트릭스와 의미가 같아야 한다
(2중 판정 — 명령 계약 자체는 robomw 가 소유하지만, 권한 문턱은 서버·로봇
양쪽에서 각자 판정해야 한쪽이 뚫려도 다른 쪽이 막는다).
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
    # 해제는 2단계다 — 이 승인만으로는 로봇이 풀리지 않는다. 현장 확인(로봇의
    # 로컬 리셋)이 따로 있어야 한다. ISO 13849-1 §5.2.2 의 특별 리셋 절차.
    # 옛 이름 clear_estop 은 의도적으로 없앴다 — 남겨두면 '원격 단독 해제'가
    # 가능하다는 오해를 계약이 계속 광고하게 된다. 미지 명령은 fail-closed 로
    # admin 요구 + WS 허용 목록 밖이라 거부된다.
    "clear_estop_request": ROLE_ADMIN,
    "clear_estop_cancel": ROLE_ADMIN,
    "set_mode": ROLE_ADMIN,
    "set_service_mode": ROLE_ADMIN,

    # v1 확장(robomw.link.protocol §2.4) — self_test·블랙박스 덤프·work 정지는
    # 조종·임무와 같은 급(operator). relocalize 는 위치 추정 자체를 리셋해
    # 임무 궤적을 깨뜨릴 수 있어 모드 변경과 같은 급(admin)으로 둔다.
    "self_test": ROLE_OPERATOR,
    "blackbox_dump": ROLE_OPERATOR,
    "work_stop": ROLE_OPERATOR,
    "relocalize": ROLE_ADMIN,
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
