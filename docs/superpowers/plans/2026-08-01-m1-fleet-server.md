# M1 관제 서버 코어 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 계정·다중 농장·이력 DB·권한·감사를 갖춘 관제 서버(FastAPI+SQLite)를 세우고, 기존 로봇(WebSocket 직결)을 레거시 어댑터로 수용하며, 대시보드에 로그인·농장·이력 화면을 붙인다 — 스펙 `docs/superpowers/specs/2026-08-01-orchard-fleet-3tier-design.md` v1.1의 M1.

**Architecture:** 3계층 중 관제 계층만 신설한다. `server/`(저장소 루트, ROS 무관 파이썬 패키지)에 FastAPI 앱 + SQLAlchemy(SQLite) + FleetPort 인터페이스를 만들고, 로봇 연결은 M1 한시로 "레거시 WS 어댑터"(서버가 로봇의 기존 WebSocket 서버에 클라이언트로 접속)가 담당한다. 브라우저는 서버의 `/ws` 하나로 전 로봇을 구독한다. M2에서 어댑터만 Zenoh로 교체된다.

**Tech Stack:** Python 3.12 · FastAPI ≥0.115 · SQLAlchemy ≥2.0 · SQLite · argon2-cffi · websockets ≥12 · pytest/httpx

## Global Constraints

(모든 태스크에 암묵 적용 — 스펙 v1.1에서 그대로 복사)

- 서버 코드는 ROS 2에 의존하지 않는다. 위치는 `server/`, 가상환경은 `server/.venv`.
- 테스트 실행은 항상: `cd /home/myhome/YBNML/server && .venv/bin/python -m pytest -q`
- 비밀번호 해시는 **Argon2**(argon2-cffi). 세션 쿠키는 **HttpOnly + SameSite=Strict**, 이름 `fleet_session`. 상태 변경 요청(POST/PATCH/DELETE, 로그인 제외)은 **`X-CSRF` 헤더** 필수.
- 권한 매트릭스(D9): `estop`·`stop_all`·`ping` = **observer**, `mission_*`·`teleop` = operator, `clear_estop`·`set_mode`·계정/농장 관리 = admin. **fail-closed**: 미지 명령→admin 요구, 미지 역할→observer 강등.
- estop 발동 시 활성 임무는 **PAUSED**로 전이. clear_estop은 래치만 해제(임무 자동 재개 없음, 재개는 mission_resume).
- 이벤트 중복 제거 키는 **(robot_id, channel, seq)** — DB 유니크 제약으로 영속화.
- 오프라인 로봇에게 보낸 명령은 **즉시 "offline" 실패**. 서버측 명령 큐 금지.
- 연결 감시: 마지막 수신 후 **15초** 지나면 오프라인 판정.
- 레거시 어댑터는 한시(M2에서 제거). 토픽 매핑 고정: `orchard/{robot}/state→tel/state`, `health→tel/health`, `map→tel/map`, `event→evt`, `mission→mission`, `hello→hello`.
- UI 문자열·커밋 메시지는 한국어. 감사 기록에 토큰·비밀번호 원문 금지, detail 160자 절단.
- 기존 로봇 코드(`ros2_ws/src/orchard_sim/`)는 이 계획에서 **수정하지 않는다**.

## 파일 구조 (전체 조감)

```
server/
  pyproject.toml               # 패키지·의존성·pytest 설정
  fleet_server/
    __init__.py
    config.py                  # Settings + load_settings()          [Task 1]
    db.py                      # Base, make_engine, session factory  [Task 2]
    models.py                  # 테이블 9종                           [Task 2]
    auth.py                    # Argon2·역할 매트릭스·세션 CRUD       [Task 3,4]
    deps.py                    # get_db·current_user·require·csrf    [Task 4]
    audit.py                   # 감사 기록(마스킹·절단)               [Task 6]
    missions.py                # 임무 상태기계 서비스                 [Task 7]
    app.py                     # create_app() 팩토리·부트스트랩       [Task 4~]
    api/
      __init__.py
      auth_routes.py           # /api/v1/auth/*                      [Task 4]
      admin_routes.py          # farms·robots·users CRUD             [Task 5]
      mission_routes.py        # /api/v1/missions*                   [Task 9]
      history_routes.py        # /api/v1/tracks·events·audit         [Task 9]
    fleet/
      __init__.py
      port.py                  # FleetPort ABC·RobotStatus·InMemory  [Task 8]
      presence.py              # PresenceRegistry (15 s 판정)         [Task 8]
      legacy_ws.py             # 레거시 로봇 WS 클라이언트 어댑터     [Task 10]
    ws.py                      # 브라우저 /ws 게이트웨이              [Task 11]
  web/
    index.html                 # 대시보드 v2 (기존 파일 복사 후 개조) [Task 12,13]
  tests/
    conftest.py                # DB·앱·클라이언트 픽스처
    test_config.py             [Task 1]
    test_models.py             [Task 2]
    test_auth_core.py          [Task 3]
    test_auth_api.py           [Task 4]
    test_admin_api.py          [Task 5]
    test_audit.py              [Task 6]
    test_missions_sm.py        [Task 7]
    test_fleet_port.py         [Task 8]
    test_mission_api.py        [Task 9]
    test_history_api.py        [Task 9]
    test_legacy_ws.py          [Task 10]
    test_ws_gateway.py         [Task 11]
    test_dashboard_serving.py  [Task 12]
  Dockerfile                   [Task 14]
compose.yaml                   # 저장소 루트                          [Task 14]
scripts/33_verify_m1.py        # E2E (가짜 로봇 내장, 실로봇 옵션)     [Task 14]
scripts/34_verify_m1_security.py  # 보안 회귀                         [Task 14]
```

임무 지시는 **REST**(`POST /api/v1/missions`)가 정본이다. WS의 `cmd`는 estop·clear_estop·stop_all·set_mode·ping·teleop만 받는다 (대시보드 임무 버튼은 REST 호출로 개조).

Alembic은 M1에서 도입하지 않는다(첫 릴리스는 `Base.metadata.create_all`; 스키마가 처음 바뀌는 M2에서 도입) — YAGNI.

---

### Task 1: 서버 골격 — 패키지·venv·Settings

**Files:**
- Create: `server/pyproject.toml`
- Create: `server/fleet_server/__init__.py` (빈 파일)
- Create: `server/fleet_server/config.py`
- Create: `server/tests/test_config.py`

**Interfaces:**
- Produces: `Settings` 데이터클래스 (필드: `db_url:str`, `session_secret:str`, `session_ttl_s:int`, `login_delay_s:float`, `allowed_origins:list[str]`, `web_dir:Path`, `admin_login:str`, `admin_password:str`, `offline_after_s:float=15.0`), `load_settings() -> Settings` (환경변수 `FLEET_*` 반영). 이후 모든 태스크가 사용.

- [ ] **Step 1: 패키지 파일 작성**

`server/pyproject.toml`:

```toml
[project]
name = "fleet-server"
version = "0.1.0"
requires-python = ">=3.12"
dependencies = [
  "fastapi>=0.115",
  "uvicorn[standard]>=0.30",
  "sqlalchemy>=2.0",
  "argon2-cffi>=23.1",
  "websockets>=12",
]

[project.optional-dependencies]
dev = ["pytest>=8", "pytest-asyncio>=0.23", "httpx>=0.27"]

[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[tool.setuptools.packages.find]
include = ["fleet_server*"]

[tool.pytest.ini_options]
asyncio_mode = "auto"
testpaths = ["tests"]
```

`server/fleet_server/config.py`:

```python
"""서버 설정 — 환경변수 FLEET_* 로 주입한다."""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class Settings:
    db_url: str = "sqlite:///fleet.db"
    session_secret: str = ""              # 비어 있으면 개발용 (부트스트랩에서 경고)
    session_ttl_s: int = 7 * 86400
    login_delay_s: float = 0.5            # 로그인 실패 지연 (스펙 §5)
    allowed_origins: list[str] = field(default_factory=list)   # WS Origin 허용 목록
    web_dir: Path = Path(__file__).resolve().parent.parent / "web"
    admin_login: str = ""                 # users 테이블이 빌 때만 부트스트랩
    admin_password: str = ""
    offline_after_s: float = 15.0         # 스펙 §3.1 — 오프라인 표시 15초


def load_settings() -> Settings:
    s = Settings()
    s.db_url = os.environ.get("FLEET_DB_URL", s.db_url)
    s.session_secret = os.environ.get("FLEET_SESSION_SECRET", s.session_secret)
    s.session_ttl_s = int(os.environ.get("FLEET_SESSION_TTL_S", s.session_ttl_s))
    s.login_delay_s = float(os.environ.get("FLEET_LOGIN_DELAY_S", s.login_delay_s))
    raw = os.environ.get("FLEET_ALLOWED_ORIGINS", "")
    s.allowed_origins = [o.strip() for o in raw.split(",") if o.strip()]
    if os.environ.get("FLEET_WEB_DIR"):
        s.web_dir = Path(os.environ["FLEET_WEB_DIR"])
    s.admin_login = os.environ.get("FLEET_ADMIN_LOGIN", "")
    s.admin_password = os.environ.get("FLEET_ADMIN_PASSWORD", "")
    return s
```

- [ ] **Step 2: 실패하는 테스트 작성**

`server/tests/test_config.py`:

```python
import os
from fleet_server.config import Settings, load_settings


def test_defaults():
    s = Settings()
    assert s.db_url.startswith("sqlite")
    assert s.offline_after_s == 15.0
    assert s.login_delay_s == 0.5


def test_env_override(monkeypatch):
    monkeypatch.setenv("FLEET_DB_URL", "sqlite:///x.db")
    monkeypatch.setenv("FLEET_ALLOWED_ORIGINS", "http://a:8000, http://b:8000")
    s = load_settings()
    assert s.db_url == "sqlite:///x.db"
    assert s.allowed_origins == ["http://a:8000", "http://b:8000"]
```

- [ ] **Step 3: venv 만들고 설치, 테스트 실행**

```bash
cd /home/myhome/YBNML/server
python3 -m venv .venv
.venv/bin/pip install -q -e '.[dev]'
.venv/bin/python -m pytest -q
```

Expected: `2 passed`

- [ ] **Step 4: 커밋**

```bash
cd /home/myhome/YBNML
printf 'server/.venv/\nserver/fleet.db\nserver/data/\n__pycache__/\n' >> .gitignore
git add server/pyproject.toml server/fleet_server/__init__.py server/fleet_server/config.py server/tests/test_config.py .gitignore
git commit -m "M1: 관제 서버 골격 — 패키지·설정"
```

---

### Task 2: DB 모델 9종 + 유니크 제약

**Files:**
- Create: `server/fleet_server/db.py`
- Create: `server/fleet_server/models.py`
- Create: `server/tests/conftest.py`
- Create: `server/tests/test_models.py`

**Interfaces:**
- Consumes: `Settings.db_url`
- Produces: `Base`, `make_engine(db_url)`, `make_session_factory(engine)`; 모델 클래스 `User, AuthSession, Farm, UserFarm, Robot, Mission, MissionEvent, Track, Event, AuditLog`. 이후 태스크는 이 이름·컬럼을 그대로 쓴다.

- [ ] **Step 1: 실패하는 테스트 작성**

`server/tests/conftest.py`:

```python
import pytest
from fleet_server.db import Base, make_engine, make_session_factory


@pytest.fixture()
def db():
    engine = make_engine("sqlite://")          # in-memory
    Base.metadata.create_all(engine)
    factory = make_session_factory(engine)
    with factory() as session:
        yield session
```

`server/tests/test_models.py`:

```python
import datetime as dt

import pytest
from sqlalchemy.exc import IntegrityError

from fleet_server import models as m


def test_tables_create(db):
    names = {t.name for t in m.Base.metadata.sorted_tables}
    assert names == {"users", "auth_sessions", "farms", "user_farms", "robots",
                     "missions", "mission_events", "tracks", "events", "audit_log"}


def test_event_dedup_unique(db):
    farm = m.Farm(name="농장A")
    db.add(farm); db.flush()
    robot = m.Robot(id="scout01", farm_id=farm.id, name="스카우트1")
    db.add(robot); db.flush()
    e1 = m.Event(robot_id="scout01", ts=dt.datetime.now(dt.UTC),
                 channel="evt", seq=7, kind="estop", severity="warn", msg="x")
    db.add(e1); db.commit()
    e2 = m.Event(robot_id="scout01", ts=dt.datetime.now(dt.UTC),
                 channel="evt", seq=7, kind="estop", severity="warn", msg="중복")
    db.add(e2)
    with pytest.raises(IntegrityError):
        db.commit()


def test_user_login_unique(db):
    db.add(m.User(login="kim", pw_hash="h", role="observer")); db.commit()
    db.add(m.User(login="kim", pw_hash="h2", role="admin"))
    with pytest.raises(IntegrityError):
        db.commit()
```

- [ ] **Step 2: 실패 확인**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest tests/test_models.py -q
```

Expected: FAIL — `ModuleNotFoundError: fleet_server.db`

- [ ] **Step 3: 구현**

`server/fleet_server/db.py`:

```python
from __future__ import annotations

from sqlalchemy import create_engine
from sqlalchemy.orm import DeclarativeBase, Session, sessionmaker


class Base(DeclarativeBase):
    pass


def make_engine(db_url: str):
    connect_args = {"check_same_thread": False} if db_url.startswith("sqlite") else {}
    return create_engine(db_url, connect_args=connect_args)


def make_session_factory(engine) -> sessionmaker[Session]:
    return sessionmaker(bind=engine, expire_on_commit=False)
```

`server/fleet_server/models.py`:

```python
"""스펙 §4.3 데이터 모델. 1차에서 역할은 전역(user_farms.role은 확장 예약)."""
from __future__ import annotations

import datetime as dt

from sqlalchemy import (JSON, DateTime, Float, ForeignKey, Index, Integer,
                        String, UniqueConstraint)
from sqlalchemy.orm import Mapped, mapped_column

from .db import Base


def _now() -> dt.datetime:
    return dt.datetime.now(dt.UTC)


class User(Base):
    __tablename__ = "users"
    id: Mapped[int] = mapped_column(primary_key=True)
    login: Mapped[str] = mapped_column(String(64), unique=True)
    pw_hash: Mapped[str] = mapped_column(String(256))
    role: Mapped[str] = mapped_column(String(16))          # observer|operator|admin (전역)
    display_name: Mapped[str] = mapped_column(String(64), default="")
    disabled: Mapped[bool] = mapped_column(default=False)
    created_at: Mapped[dt.datetime] = mapped_column(DateTime(timezone=True), default=_now)


class AuthSession(Base):
    __tablename__ = "auth_sessions"
    id: Mapped[str] = mapped_column(String(64), primary_key=True)      # 세션 토큰
    user_id: Mapped[int] = mapped_column(ForeignKey("users.id"))
    csrf: Mapped[str] = mapped_column(String(64))
    created_at: Mapped[dt.datetime] = mapped_column(DateTime(timezone=True), default=_now)
    expires_at: Mapped[dt.datetime] = mapped_column(DateTime(timezone=True))


class Farm(Base):
    __tablename__ = "farms"
    id: Mapped[int] = mapped_column(primary_key=True)
    name: Mapped[str] = mapped_column(String(64), unique=True)
    map_bundle_ref: Mapped[str | None] = mapped_column(String(256), default=None)
    config_json: Mapped[dict] = mapped_column(JSON, default=dict)   # 링크 단절 정책 등


class UserFarm(Base):
    __tablename__ = "user_farms"
    user_id: Mapped[int] = mapped_column(ForeignKey("users.id"), primary_key=True)
    farm_id: Mapped[int] = mapped_column(ForeignKey("farms.id"), primary_key=True)
    # 확장 예약(스펙 §10): 농장별 역할이 필요해지면 role 컬럼 추가


class Robot(Base):
    __tablename__ = "robots"
    id: Mapped[str] = mapped_column(String(64), primary_key=True)   # 예: "scout01"
    farm_id: Mapped[int] = mapped_column(ForeignKey("farms.id"))
    name: Mapped[str] = mapped_column(String(64))
    kind: Mapped[str] = mapped_column(String(32), default="orchard")
    conn_kind: Mapped[str] = mapped_column(String(16), default="legacy_ws")  # M2: zenoh|mqtt
    last_seen: Mapped[dt.datetime | None] = mapped_column(DateTime(timezone=True), default=None)
    config_json: Mapped[dict] = mapped_column(JSON, default=dict)   # {"ws_url":..., "token":...}


class Mission(Base):
    __tablename__ = "missions"
    id: Mapped[int] = mapped_column(primary_key=True)
    robot_id: Mapped[str] = mapped_column(ForeignKey("robots.id"))
    farm_id: Mapped[int] = mapped_column(ForeignKey("farms.id"))
    spec_json: Mapped[dict] = mapped_column(JSON, default=dict)     # {"alleys":[...], "map_hash":...}
    state: Mapped[str] = mapped_column(String(16), default="QUEUED")
    phases_json: Mapped[list] = mapped_column(JSON, default=list)
    created_by: Mapped[int] = mapped_column(ForeignKey("users.id"))
    created_at: Mapped[dt.datetime] = mapped_column(DateTime(timezone=True), default=_now)
    started_at: Mapped[dt.datetime | None] = mapped_column(DateTime(timezone=True), default=None)
    ended_at: Mapped[dt.datetime | None] = mapped_column(DateTime(timezone=True), default=None)


class MissionEvent(Base):
    __tablename__ = "mission_events"
    id: Mapped[int] = mapped_column(primary_key=True)
    mission_id: Mapped[int] = mapped_column(ForeignKey("missions.id"))
    ts: Mapped[dt.datetime] = mapped_column(DateTime(timezone=True), default=_now)
    channel: Mapped[str | None] = mapped_column(String(32), default=None)
    seq: Mapped[int | None] = mapped_column(Integer, default=None)
    kind: Mapped[str] = mapped_column(String(32))                   # 전이 이벤트명
    payload_json: Mapped[dict] = mapped_column(JSON, default=dict)


class Track(Base):
    __tablename__ = "tracks"
    id: Mapped[int] = mapped_column(primary_key=True)
    robot_id: Mapped[str] = mapped_column(ForeignKey("robots.id"))
    ts: Mapped[dt.datetime] = mapped_column(DateTime(timezone=True))
    x: Mapped[float] = mapped_column(Float)
    y: Mapped[float] = mapped_column(Float)
    yaw: Mapped[float] = mapped_column(Float)
    mode: Mapped[str] = mapped_column(String(16), default="")
    __table_args__ = (Index("ix_tracks_robot_ts", "robot_id", "ts"),)


class Event(Base):
    __tablename__ = "events"
    id: Mapped[int] = mapped_column(primary_key=True)
    robot_id: Mapped[str] = mapped_column(ForeignKey("robots.id"))
    ts: Mapped[dt.datetime] = mapped_column(DateTime(timezone=True))
    channel: Mapped[str | None] = mapped_column(String(32), default=None)
    seq: Mapped[int | None] = mapped_column(Integer, default=None)
    kind: Mapped[str] = mapped_column(String(32))
    severity: Mapped[str] = mapped_column(String(16), default="info")
    msg: Mapped[str] = mapped_column(String(256), default="")
    payload_json: Mapped[dict] = mapped_column(JSON, default=dict)
    __table_args__ = (UniqueConstraint("robot_id", "channel", "seq", name="uq_event_seq"),)


class AuditLog(Base):
    __tablename__ = "audit_log"
    id: Mapped[int] = mapped_column(primary_key=True)
    ts: Mapped[dt.datetime] = mapped_column(DateTime(timezone=True), default=_now)
    user_id: Mapped[int | None] = mapped_column(ForeignKey("users.id"), default=None)
    role: Mapped[str] = mapped_column(String(16), default="")
    action: Mapped[str] = mapped_column(String(64))
    target: Mapped[str] = mapped_column(String(128), default="")
    result: Mapped[str] = mapped_column(String(16))                 # accepted|rejected|error
    detail: Mapped[str] = mapped_column(String(160), default="")    # 160자 절단(스펙)
```

- [ ] **Step 4: 통과 확인 후 커밋**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest -q
cd /home/myhome/YBNML
git add server/fleet_server/db.py server/fleet_server/models.py server/tests/conftest.py server/tests/test_models.py
git commit -m "M1: DB 모델 9종 + (robot,channel,seq) 유니크 제약"
```

---

### Task 3: 인증 코어 — Argon2 + 역할 매트릭스 (D9, fail-closed)

**Files:**
- Create: `server/fleet_server/auth.py` (이 태스크에서는 해시·역할만; 세션 함수는 Task 4에서 같은 파일에 추가)
- Create: `server/tests/test_auth_core.py`

**Interfaces:**
- Produces: `hash_password(pw:str)->str`, `verify_password(pw:str, pw_hash:str)->bool`, 상수 `ROLE_OBSERVER/ROLE_OPERATOR/ROLE_ADMIN`, `ROLE_RANK:dict`, `ROLE_REQUIRED:dict`, `normalize_role(v)->str`, `authorize(role, action)->bool`

- [ ] **Step 1: 실패하는 테스트 작성**

`server/tests/test_auth_core.py`:

```python
import pytest

from fleet_server import auth


def test_password_roundtrip():
    h = auth.hash_password("비밀1234")
    assert h != "비밀1234"
    assert auth.verify_password("비밀1234", h)
    assert not auth.verify_password("오답", h)
    assert not auth.verify_password("비밀1234", "손상된해시")


# D9: estop·stop_all은 observer 포함 전 역할 허용, 해제는 admin만
@pytest.mark.parametrize("role,action,ok", [
    ("observer", "estop", True),
    ("observer", "stop_all", True),
    ("observer", "ping", True),
    ("observer", "mission_start", False),
    ("observer", "teleop", False),
    ("observer", "clear_estop", False),
    ("operator", "mission_start", True),
    ("operator", "teleop", True),
    ("operator", "clear_estop", False),
    ("admin", "clear_estop", True),
    ("admin", "set_mode", True),
])
def test_matrix(role, action, ok):
    assert auth.authorize(role, action) is ok


def test_fail_closed():
    assert auth.authorize("admin", "완전히_모르는_명령") is True   # 미지 명령 → admin 요구
    assert auth.authorize("operator", "완전히_모르는_명령") is False
    assert auth.normalize_role("이상한역할") == "observer"          # 미지 역할 → 강등
    assert auth.authorize("이상한역할", "mission_start") is False
    assert auth.authorize(None, "estop") is True                    # 강등돼도 estop은 가능(D9)
```

- [ ] **Step 2: 실패 확인**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest tests/test_auth_core.py -q
```

Expected: FAIL — `ModuleNotFoundError` 또는 `AttributeError`

- [ ] **Step 3: 구현**

`server/fleet_server/auth.py`:

```python
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
```

- [ ] **Step 4: 통과 확인 후 커밋**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest -q
cd /home/myhome/YBNML
git add server/fleet_server/auth.py server/tests/test_auth_core.py
git commit -m "M1: 인증 코어 — Argon2 + D9 역할 매트릭스 (fail-closed)"
```

---

### Task 4: 앱 팩토리 + 세션 + /auth API + CSRF

**Files:**
- Create: `server/fleet_server/app.py`
- Create: `server/fleet_server/deps.py`
- Create: `server/fleet_server/api/__init__.py` (빈 파일)
- Create: `server/fleet_server/api/auth_routes.py`
- Modify: `server/fleet_server/auth.py` (세션 CRUD 추가)
- Modify: `server/fleet_server/db.py` (인메모리 전용 StaticPool)
- Modify: `server/tests/conftest.py` (app·client 픽스처)
- Create: `server/tests/test_auth_api.py`

**Interfaces:**
- Consumes: Task 2 모델, Task 3 해시·매트릭스
- Produces: `create_app(settings=None, engine=None) -> FastAPI` (state: settings·engine·session_factory·fleet=None), `deps.get_db`, `deps.current_user`, `deps.current_session`, `deps.require_min_role(role)`, `deps.csrf_protect`, `deps.SESSION_COOKIE="fleet_session"`, `auth.create_session(db,user,ttl_s)->AuthSession`, `auth.delete_session(db,token)`. API: `POST /api/v1/auth/login`, `POST /api/v1/auth/logout`(CSRF), `GET /api/v1/auth/me`.

- [ ] **Step 1: 실패하는 테스트 작성**

`server/tests/conftest.py` 에 추가:

```python
from fastapi.testclient import TestClient

from fleet_server.app import create_app
from fleet_server.config import Settings


def _test_settings(**kw) -> Settings:
    base = dict(db_url="sqlite://", session_secret="테스트비밀",
                login_delay_s=0.0, admin_login="admin", admin_password="admpw",
                allowed_origins=["http://testserver"])
    base.update(kw)
    return Settings(**base)


@pytest.fixture()
def app():
    return create_app(_test_settings())


@pytest.fixture()
def client(app):
    return TestClient(app)


def do_login(client, login="admin", pw="admpw") -> str:
    """로그인하고 CSRF 토큰을 돌려준다 (쿠키는 client 가 유지)."""
    r = client.post("/api/v1/auth/login", json={"login": login, "password": pw})
    assert r.status_code == 200, r.text
    return r.json()["csrf"]
```

`server/tests/test_auth_api.py`:

```python
import time

from fastapi.testclient import TestClient

from fleet_server.app import create_app
from tests.conftest import _test_settings, do_login


def test_bootstrap_admin_and_me(client):
    csrf = do_login(client)
    r = client.get("/api/v1/auth/me")
    assert r.status_code == 200
    assert r.json()["user"]["role"] == "admin"
    assert r.json()["csrf"] == csrf


def test_login_failure_delayed_and_401():
    app = create_app(_test_settings(login_delay_s=0.3))
    c = TestClient(app)
    t0 = time.monotonic()
    r = c.post("/api/v1/auth/login", json={"login": "admin", "password": "오답"})
    assert r.status_code == 401
    assert time.monotonic() - t0 >= 0.25          # 실패 지연 (스펙 §5)


def test_cookie_flags(client):
    r = client.post("/api/v1/auth/login", json={"login": "admin", "password": "admpw"})
    sc = r.headers["set-cookie"].lower()
    assert "httponly" in sc and "samesite=strict" in sc


def test_me_without_session_401(client):
    assert client.get("/api/v1/auth/me").status_code == 401


def test_logout_requires_csrf(client):
    csrf = do_login(client)
    assert client.post("/api/v1/auth/logout").status_code == 403          # 헤더 없음
    r = client.post("/api/v1/auth/logout", headers={"X-CSRF": csrf})
    assert r.status_code == 200
    assert client.get("/api/v1/auth/me").status_code == 401               # 세션 소멸
```

- [ ] **Step 2: 실패 확인**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest tests/test_auth_api.py -q
```

Expected: FAIL — `ModuleNotFoundError: fleet_server.app`

- [ ] **Step 3: 구현**

`server/fleet_server/db.py` 의 `make_engine` 을 교체 (인메모리 DB 는 스레드 공유 필요 — TestClient 는 별도 스레드에서 돈다):

```python
from sqlalchemy.pool import StaticPool


def make_engine(db_url: str):
    if db_url == "sqlite://":                      # 인메모리: 단일 연결 공유
        return create_engine(db_url, poolclass=StaticPool,
                             connect_args={"check_same_thread": False})
    connect_args = {"check_same_thread": False} if db_url.startswith("sqlite") else {}
    return create_engine(db_url, connect_args=connect_args)
```

`server/fleet_server/auth.py` 끝에 추가:

```python
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
```

`server/fleet_server/deps.py`:

```python
from __future__ import annotations

import datetime as dt

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
    if request.headers.get("X-CSRF") != sess.csrf:
        raise HTTPException(403, "CSRF 토큰 불일치")
```

`server/fleet_server/api/auth_routes.py`:

```python
from __future__ import annotations

import time

from fastapi import APIRouter, Depends, HTTPException, Request, Response
from pydantic import BaseModel

from .. import auth
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
        time.sleep(settings.login_delay_s)              # 실패 지연 (스펙 §5)
        raise HTTPException(401, "아이디 또는 비밀번호가 틀립니다")
    row = auth.create_session(db, user, settings.session_ttl_s)
    response.set_cookie(SESSION_COOKIE, row.id, httponly=True, samesite="strict",
                        max_age=settings.session_ttl_s)
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
```

`server/fleet_server/app.py`:

```python
from __future__ import annotations

from fastapi import FastAPI
from fastapi.responses import FileResponse

from . import auth
from .config import Settings, load_settings
from .db import Base, make_engine, make_session_factory
from .models import User


def _bootstrap_admin(app: FastAPI) -> None:
    """users 가 비어 있으면 FLEET_ADMIN_* 로 최초 관리자를 만든다."""
    s: Settings = app.state.settings
    if not (s.admin_login and s.admin_password):
        return
    with app.state.session_factory() as db:
        if db.query(User).count() == 0:
            db.add(User(login=s.admin_login, pw_hash=auth.hash_password(s.admin_password),
                        role="admin", display_name="관리자"))
            db.commit()


def create_app(settings: Settings | None = None, engine=None) -> FastAPI:
    settings = settings or load_settings()
    engine = engine or make_engine(settings.db_url)
    Base.metadata.create_all(engine)

    app = FastAPI(title="과수원 통합관제 서버")
    app.state.settings = settings
    app.state.engine = engine
    app.state.session_factory = make_session_factory(engine)
    app.state.fleet = None            # FleetPort — Task 8 이후 주입

    from .api import auth_routes
    app.include_router(auth_routes.router, prefix="/api/v1")

    _bootstrap_admin(app)

    @app.get("/", include_in_schema=False)
    def index():
        return FileResponse(settings.web_dir / "index.html")

    return app
```

- [ ] **Step 4: 통과 확인 후 커밋**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest -q
cd /home/myhome/YBNML
git add server/fleet_server server/tests
git commit -m "M1: 앱 팩토리 + 세션 로그인/로그아웃 + CSRF"
```

---

### Task 5: 농장·로봇·사용자 API + 농장 스코프

**Files:**
- Create: `server/fleet_server/api/admin_routes.py`
- Modify: `server/fleet_server/deps.py` (`farm_scope` 추가)
- Modify: `server/fleet_server/app.py` (라우터 등록 1줄)
- Create: `server/tests/test_admin_api.py`

**Interfaces:**
- Consumes: Task 4 deps
- Produces: `deps.farm_scope(db, user) -> set[int] | None` (None=admin 전체). API: `GET/POST/PATCH /api/v1/farms`, `GET/POST/PATCH /api/v1/robots`, `GET/POST/PATCH /api/v1/users` — GET 는 observer+(스코프 필터), 쓰기는 admin(스펙 §4.4). 이후 태스크는 robots.config_json 의 `{"ws_url","token"}` 을 레거시 어댑터 연결 정보로 쓴다.

- [ ] **Step 1: 실패하는 테스트 작성**

`server/tests/test_admin_api.py`:

```python
from tests.conftest import do_login


def _seed(client):
    """admin 으로 농장 2·로봇 1·observer 1 을 만든다. (A농장만 배정)"""
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    fb = client.post("/api/v1/farms", json={"name": "농장B"}, headers=h).json()
    client.post("/api/v1/robots", headers=h, json={
        "id": "scout01", "farm_id": fa["id"], "name": "스카우트1",
        "config_json": {"ws_url": "ws://127.0.0.1:8080/ws", "token": "RTOK"}})
    client.post("/api/v1/users", headers=h, json={
        "login": "obs", "password": "obspw", "role": "observer",
        "farm_ids": [fa["id"]]})
    return fa, fb


def test_crud_and_scope(client):
    fa, fb = _seed(client)
    # observer 는 자기 스코프 농장만 본다
    do_login(client, "obs", "obspw")
    farms = client.get("/api/v1/farms").json()
    assert [f["name"] for f in farms] == ["농장A"]
    robots = client.get("/api/v1/robots").json()
    assert [r["id"] for r in robots] == ["scout01"]
    # 타 농장 조회는 403
    assert client.get(f"/api/v1/robots?farm_id={fb['id']}").status_code == 403


def test_write_requires_admin(client):
    _seed(client)
    csrf = do_login(client, "obs", "obspw")
    r = client.post("/api/v1/farms", json={"name": "몰래"}, headers={"X-CSRF": csrf})
    assert r.status_code == 403


def test_write_requires_csrf(client):
    do_login(client)
    assert client.post("/api/v1/farms", json={"name": "농장C"}).status_code == 403


def test_robot_token_not_exposed_to_observer(client):
    fa, _ = _seed(client)
    do_login(client, "obs", "obspw")
    robots = client.get("/api/v1/robots").json()
    assert "config_json" not in robots[0]      # 접속 토큰은 admin 전용 정보
```

- [ ] **Step 2: 실패 확인**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest tests/test_admin_api.py -q
```

Expected: FAIL — 404 (라우트 없음)

- [ ] **Step 3: 구현**

`server/fleet_server/deps.py` 끝에 추가:

```python
def farm_scope(db, user: User) -> set[int] | None:
    """admin 은 None(전체), 그 외는 배정 농장 id 집합."""
    if normalize_role(user.role) == "admin":
        return None
    from .models import UserFarm
    rows = db.query(UserFarm.farm_id).filter(UserFarm.user_id == user.id).all()
    return {r[0] for r in rows}
```

`server/fleet_server/api/admin_routes.py`:

```python
from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel

from .. import auth
from ..deps import (csrf_protect, current_user, farm_scope, get_db,
                    require_min_role)
from ..models import Farm, Robot, User, UserFarm

router = APIRouter(tags=["admin"])
_admin = Depends(require_min_role("admin"))
_csrf = Depends(csrf_protect)


# ── 농장 ──────────────────────────────────────────────────────────────────
class FarmBody(BaseModel):
    name: str


class FarmPatch(BaseModel):
    map_bundle_ref: str | None = None
    config_json: dict | None = None


@router.get("/farms")
def list_farms(db=Depends(get_db), user: User = Depends(current_user)):
    scope = farm_scope(db, user)
    q = db.query(Farm)
    if scope is not None:
        q = q.filter(Farm.id.in_(scope))
    return [{"id": f.id, "name": f.name, "map_bundle_ref": f.map_bundle_ref}
            for f in q.order_by(Farm.id)]


@router.post("/farms", dependencies=[_admin, _csrf])
def create_farm(body: FarmBody, db=Depends(get_db)):
    f = Farm(name=body.name)
    db.add(f); db.commit()
    return {"id": f.id, "name": f.name}


@router.patch("/farms/{farm_id}", dependencies=[_admin, _csrf])
def patch_farm(farm_id: int, body: FarmPatch, db=Depends(get_db)):
    f = db.get(Farm, farm_id)
    if f is None:
        raise HTTPException(404, "농장이 없습니다")
    if body.map_bundle_ref is not None:
        f.map_bundle_ref = body.map_bundle_ref
    if body.config_json is not None:
        f.config_json = body.config_json
    db.commit()
    return {"ok": True}


# ── 로봇 ──────────────────────────────────────────────────────────────────
class RobotBody(BaseModel):
    id: str
    farm_id: int
    name: str
    kind: str = "orchard"
    conn_kind: str = "legacy_ws"
    config_json: dict = {}


def _robot_out(r: Robot, admin: bool) -> dict:
    out = {"id": r.id, "farm_id": r.farm_id, "name": r.name, "kind": r.kind,
           "conn_kind": r.conn_kind,
           "last_seen": r.last_seen.isoformat() if r.last_seen else None}
    if admin:
        out["config_json"] = r.config_json      # 접속 정보는 admin 에게만
    return out


@router.get("/robots")
def list_robots(farm_id: int | None = None, db=Depends(get_db),
                user: User = Depends(current_user)):
    scope = farm_scope(db, user)
    if farm_id is not None and scope is not None and farm_id not in scope:
        raise HTTPException(403, "해당 농장 권한이 없습니다")
    q = db.query(Robot)
    if farm_id is not None:
        q = q.filter(Robot.farm_id == farm_id)
    elif scope is not None:
        q = q.filter(Robot.farm_id.in_(scope))
    admin = auth.normalize_role(user.role) == "admin"
    return [_robot_out(r, admin) for r in q.order_by(Robot.id)]


@router.post("/robots", dependencies=[_admin, _csrf])
def create_robot(body: RobotBody, db=Depends(get_db)):
    if db.get(Farm, body.farm_id) is None:
        raise HTTPException(404, "농장이 없습니다")
    r = Robot(**body.model_dump())
    db.add(r); db.commit()
    return _robot_out(r, admin=True)


@router.patch("/robots/{robot_id}", dependencies=[_admin, _csrf])
def patch_robot(robot_id: str, body: dict, db=Depends(get_db)):
    r = db.get(Robot, robot_id)
    if r is None:
        raise HTTPException(404, "로봇이 없습니다")
    for k in ("name", "kind", "conn_kind", "config_json", "farm_id"):
        if k in body:
            setattr(r, k, body[k])
    db.commit()
    return {"ok": True}


# ── 사용자 ────────────────────────────────────────────────────────────────
class UserBody(BaseModel):
    login: str
    password: str
    role: str = "observer"
    display_name: str = ""
    farm_ids: list[int] = []


@router.get("/users", dependencies=[_admin])
def list_users(db=Depends(get_db)):
    return [{"id": u.id, "login": u.login, "role": u.role,
             "display_name": u.display_name, "disabled": u.disabled}
            for u in db.query(User).order_by(User.id)]


@router.post("/users", dependencies=[_admin, _csrf])
def create_user(body: UserBody, db=Depends(get_db)):
    u = User(login=body.login, pw_hash=auth.hash_password(body.password),
             role=auth.normalize_role(body.role), display_name=body.display_name)
    db.add(u); db.flush()
    for fid in body.farm_ids:
        db.add(UserFarm(user_id=u.id, farm_id=fid))
    db.commit()
    return {"id": u.id, "login": u.login, "role": u.role}


@router.patch("/users/{user_id}", dependencies=[_admin, _csrf])
def patch_user(user_id: int, body: dict, db=Depends(get_db)):
    u = db.get(User, user_id)
    if u is None:
        raise HTTPException(404, "사용자가 없습니다")
    if "disabled" in body:
        u.disabled = bool(body["disabled"])
    if "role" in body:
        u.role = auth.normalize_role(body["role"])
    if "password" in body:
        u.pw_hash = auth.hash_password(body["password"])
    if "farm_ids" in body:
        db.query(UserFarm).filter(UserFarm.user_id == u.id).delete()
        for fid in body["farm_ids"]:
            db.add(UserFarm(user_id=u.id, farm_id=fid))
    db.commit()
    return {"ok": True}
```

`server/fleet_server/app.py` 라우터 등록에 1줄 추가 (auth_routes 아래):

```python
    from .api import admin_routes, auth_routes
    app.include_router(auth_routes.router, prefix="/api/v1")
    app.include_router(admin_routes.router, prefix="/api/v1")
```

- [ ] **Step 4: 통과 확인 후 커밋**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest -q
cd /home/myhome/YBNML
git add server/fleet_server server/tests
git commit -m "M1: 농장·로봇·사용자 API + 스코프 필터 (조회 observer+, 쓰기 admin)"
```

---

### Task 6: 감사 서비스 — 마스킹·절단·전수 기록

**Files:**
- Create: `server/fleet_server/audit.py`
- Modify: `server/fleet_server/api/auth_routes.py` (로그인 성공·실패 기록)
- Modify: `server/fleet_server/api/admin_routes.py` (쓰기 기록)
- Create: `server/tests/test_audit.py`

**Interfaces:**
- Consumes: Task 2 `AuditLog`
- Produces: `audit.record(db, *, action:str, result:str, user_id:int|None=None, role:str="", target:str="", detail:str="") -> None` — detail 은 토큰·비밀번호 마스킹 + 160자 절단. result 는 "accepted"|"rejected"|"error". 이후 WS 게이트웨이(Task 11)도 이 함수를 쓴다.

- [ ] **Step 1: 실패하는 테스트 작성**

`server/tests/test_audit.py`:

```python
from fleet_server import audit
from fleet_server.models import AuditLog
from tests.conftest import do_login


def test_masking_and_clip(db):
    audit.record(db, action="cmd", result="rejected",
                 detail='{"token": "비밀토큰123", "password": "pw다"} ' + "X" * 500)
    row = db.query(AuditLog).one()
    assert "비밀토큰123" not in row.detail and "pw다" not in row.detail
    assert len(row.detail) <= 160


def test_login_failure_recorded(client, app):
    client.post("/api/v1/auth/login", json={"login": "admin", "password": "오답"})
    with app.state.session_factory() as db:
        rows = db.query(AuditLog).filter(AuditLog.action == "login").all()
        assert rows and rows[-1].result == "rejected"
        blob = " ".join(r.detail for r in rows)
        assert "오답" not in blob                      # 비밀번호 원문 금지


def test_farm_create_recorded(client, app):
    csrf = do_login(client)
    client.post("/api/v1/farms", json={"name": "감사농장"}, headers={"X-CSRF": csrf})
    with app.state.session_factory() as db:
        row = db.query(AuditLog).filter(AuditLog.action == "farm_create").one()
        assert row.result == "accepted" and "감사농장" in row.detail
```

- [ ] **Step 2: 실패 확인**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest tests/test_audit.py -q
```

Expected: FAIL — `ModuleNotFoundError: fleet_server.audit`

- [ ] **Step 3: 구현**

`server/fleet_server/audit.py`:

```python
"""감사 기록 — 모든 명령·관리 행위의 수락/거부를 DB 에 남긴다 (스펙 S7).

텔레옵은 세션 단위(시작·종료·거부)로만 기록한다 — 20 Hz 개별 지령 제외.
"""
from __future__ import annotations

import re

from .models import AuditLog

_MASK = re.compile(r'("?(?:token|password|pw|secret)"?\s*[:=]\s*"?)[^",}\s]+', re.I)
_CLIP = 160


def _sanitize(detail: str) -> str:
    detail = _MASK.sub(r"\1***", detail)
    return detail[:_CLIP]


def record(db, *, action: str, result: str, user_id: int | None = None,
           role: str = "", target: str = "", detail: str = "") -> None:
    db.add(AuditLog(action=action, result=result, user_id=user_id,
                    role=role, target=target[:128], detail=_sanitize(detail)))
    db.commit()
```

`auth_routes.login` 에 기록 추가 — 실패 분기(`raise` 직전)에:

```python
        audit.record(db, action="login", result="rejected", target=body.login,
                     detail="비밀번호 불일치 또는 없는 계정")
        time.sleep(settings.login_delay_s)
        raise HTTPException(401, "아이디 또는 비밀번호가 틀립니다")
```

성공 분기(`return` 직전)에:

```python
    audit.record(db, action="login", result="accepted", user_id=user.id,
                 role=user.role, target=user.login)
```

(파일 상단에 `from .. import audit` 추가)

`admin_routes.py` 의 쓰기 4곳(create_farm·create_robot·create_user·patch_user)에도 같은 패턴으로 기록한다. 예 — `create_farm` 의 `return` 직전:

```python
    audit.record(db, action="farm_create", result="accepted",
                 target=str(f.id), detail=f.name)
```

(각각 action="robot_create"·"user_create"·"user_patch", target 은 해당 id, detail 은 이름/변경 키 목록. 파일 상단에 `from .. import audit` 추가)

- [ ] **Step 4: 통과 확인 후 커밋**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest -q
cd /home/myhome/YBNML
git add server/fleet_server server/tests
git commit -m "M1: 감사 서비스 — 마스킹·160자 절단·로그인/관리 행위 기록"
```

---

### Task 7: 임무 상태기계 (estop→PAUSED, clear_estop 은 래치만)

**Files:**
- Create: `server/fleet_server/missions.py`
- Create: `server/tests/test_missions_sm.py`

**Interfaces:**
- Consumes: Task 2 `Mission`, `MissionEvent`
- Produces: `missions.InvalidTransition(Exception)`, `missions.TRANSITIONS: dict[tuple[str,str],str]`, `missions.create(db, *, robot_id, farm_id, spec, created_by) -> Mission`(state=QUEUED), `missions.apply(db, mission, event, *, payload=None) -> Mission`. 이벤트 이름: `start·pause·resume·cancel·complete·fail·estop`. Task 9(API)·Task 11(WS)이 사용.

- [ ] **Step 1: 실패하는 테스트 작성**

`server/tests/test_missions_sm.py`:

```python
import pytest

from fleet_server import missions
from fleet_server import models as m


def _mk(db) -> m.Mission:
    farm = m.Farm(name="농장A"); db.add(farm); db.flush()
    db.add(m.Robot(id="scout01", farm_id=farm.id, name="r")); db.flush()
    db.add(m.User(login="op", pw_hash="h", role="operator")); db.flush()
    return missions.create(db, robot_id="scout01", farm_id=farm.id,
                           spec={"alleys": [0, 1]}, created_by=1)


@pytest.mark.parametrize("chain,final", [
    (["start", "complete"], "DONE"),
    (["start", "pause", "resume", "complete"], "DONE"),
    (["cancel"], "CANCELED"),                       # QUEUED→CANCELED (스펙 §4.3)
    (["start", "estop"], "PAUSED"),                 # estop → PAUSED
    (["start", "estop", "resume", "complete"], "DONE"),
    (["start", "fail"], "FAILED"),
    (["start", "pause", "cancel"], "CANCELED"),
])
def test_transitions(db, chain, final):
    ms = _mk(db)
    for ev in chain:
        ms = missions.apply(db, ms, ev)
    assert ms.state == final


def test_invalid_transition_raises(db):
    ms = _mk(db)
    ms = missions.apply(db, ms, "start")
    ms = missions.apply(db, ms, "complete")
    with pytest.raises(missions.InvalidTransition):
        missions.apply(db, ms, "resume")             # DONE 에서 재개 불가


def test_timestamps_and_events(db):
    ms = _mk(db)
    assert ms.started_at is None
    ms = missions.apply(db, ms, "start")
    assert ms.started_at is not None and ms.ended_at is None
    ms = missions.apply(db, ms, "complete")
    assert ms.ended_at is not None
    kinds = [e.kind for e in db.query(m.MissionEvent)
             .filter_by(mission_id=ms.id).order_by(m.MissionEvent.id)]
    assert kinds == ["start", "complete"]            # 전이마다 이력 1행
```

- [ ] **Step 2: 실패 확인**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest tests/test_missions_sm.py -q
```

Expected: FAIL — `ModuleNotFoundError: fleet_server.missions`

- [ ] **Step 3: 구현**

`server/fleet_server/missions.py`:

```python
"""임무 상태기계 — 스펙 §4.3 전이 표 그대로.

estop 은 임무를 PAUSED 로 보낸다. clear_estop 은 임무 이벤트가 아니다(래치만
해제) — 재개는 반드시 별도 resume 으로만 일어난다 (안전 서프라이즈 차단).
"""
from __future__ import annotations

import datetime as dt

from .models import Mission, MissionEvent


class InvalidTransition(Exception):
    pass


TRANSITIONS: dict[tuple[str, str], str] = {
    ("QUEUED", "start"): "RUNNING",
    ("QUEUED", "cancel"): "CANCELED",
    ("RUNNING", "pause"): "PAUSED",
    ("RUNNING", "estop"): "PAUSED",
    ("RUNNING", "complete"): "DONE",
    ("RUNNING", "cancel"): "CANCELED",
    ("RUNNING", "fail"): "FAILED",
    ("PAUSED", "resume"): "RUNNING",
    ("PAUSED", "cancel"): "CANCELED",
    ("PAUSED", "fail"): "FAILED",
}
_TERMINAL = {"DONE", "CANCELED", "FAILED"}


def create(db, *, robot_id: str, farm_id: int, spec: dict, created_by: int) -> Mission:
    ms = Mission(robot_id=robot_id, farm_id=farm_id, spec_json=spec,
                 created_by=created_by)
    db.add(ms)
    db.commit()
    return ms


def apply(db, mission: Mission, event: str, *, payload: dict | None = None) -> Mission:
    key = (mission.state, event)
    if key not in TRANSITIONS:
        raise InvalidTransition(f"{mission.state} 에서 {event} 불가")
    new = TRANSITIONS[key]
    now = dt.datetime.now(dt.UTC)
    if mission.state == "QUEUED" and new == "RUNNING":
        mission.started_at = now
    if new in _TERMINAL:
        mission.ended_at = now
    mission.state = new
    db.add(MissionEvent(mission_id=mission.id, kind=event,
                        payload_json=payload or {}))
    db.commit()
    return mission
```

- [ ] **Step 4: 통과 확인 후 커밋**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest -q
cd /home/myhome/YBNML
git add server/fleet_server/missions.py server/tests/test_missions_sm.py
git commit -m "M1: 임무 상태기계 — 전이 표·estop→PAUSED·타임스탬프"
```

---

### Task 8: FleetPort 인터페이스 + PresenceRegistry (15초 판정)

**Files:**
- Create: `server/fleet_server/fleet/__init__.py` (빈 파일)
- Create: `server/fleet_server/fleet/presence.py`
- Create: `server/fleet_server/fleet/port.py`
- Modify: `server/fleet_server/app.py` (`create_app(..., fleet=None)` 파라미터 + `app.state.fleet`)
- Modify: `server/fleet_server/api/admin_routes.py` (`GET /robots/{id}/status` 추가)
- Modify: `server/tests/conftest.py` (app 픽스처가 InMemoryFleetPort 주입)
- Create: `server/tests/test_fleet_port.py`

**Interfaces:**
- Produces (이후 태스크가 그대로 사용):

```python
TelemetryHandler = Callable[[str, str, dict, int | None], None]  # (robot_id, channel, payload, seq)

@dataclass
class RobotStatus:
    online: bool
    last_seen: float | None          # time.time() 초

class FleetPort(ABC):
    def register_robot(self, robot_id: str, farm_id: int, conn_kind: str, config: dict) -> None
    async def send_command(self, robot_id: str, cmd_id: str, action: str, payload: dict) -> str   # "sent"|"offline"
    def robot_status(self, robot_id: str) -> RobotStatus
    def set_telemetry_handler(self, cb: TelemetryHandler) -> None

class InMemoryFleetPort(FleetPort):   # 테스트·개발용
    presence: PresenceRegistry
    sent: list[tuple[str, str, str, dict]]        # (robot_id, cmd_id, action, payload)
    def feed(self, robot_id, channel, payload, seq=None)   # 텔레메트리 주입(touch 포함)

class PresenceRegistry:
    def __init__(self, offline_after_s: float = 15.0)
    def touch(self, robot_id: str, t: float | None = None) -> None
    def last_seen(self, robot_id: str) -> float | None
    def online(self, robot_id: str, t: float | None = None) -> bool
```

- [ ] **Step 1: 실패하는 테스트 작성**

`server/tests/test_fleet_port.py`:

```python
import pytest

from fleet_server.fleet.port import InMemoryFleetPort
from fleet_server.fleet.presence import PresenceRegistry
from tests.conftest import do_login


def test_presence_15s_boundary():
    p = PresenceRegistry(offline_after_s=15.0)
    assert p.online("r1", t=100.0) is False          # 한 번도 못 봄
    p.touch("r1", t=100.0)
    assert p.online("r1", t=114.9) is True
    assert p.online("r1", t=115.1) is False          # 15초 초과 → 오프라인 (스펙 §3.1)


@pytest.mark.asyncio
async def test_offline_command_fails_immediately():
    fp = InMemoryFleetPort()
    fp.register_robot("scout01", 1, "legacy_ws", {})
    assert await fp.send_command("scout01", "c1", "estop", {}) == "offline"
    fp.feed("scout01", "tel/state", {"x": 0})
    assert await fp.send_command("scout01", "c2", "estop", {}) == "sent"
    assert fp.sent[-1][2] == "estop"


def test_telemetry_handler_called():
    fp = InMemoryFleetPort()
    got = []
    fp.set_telemetry_handler(lambda r, ch, pl, seq: got.append((r, ch, seq)))
    fp.feed("scout01", "evt", {"kind": "estop"}, seq=3)
    assert got == [("scout01", "evt", 3)]


def test_status_route(client, app):
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    client.post("/api/v1/robots", headers=h,
                json={"id": "scout01", "farm_id": fa["id"], "name": "r"})
    r = client.get("/api/v1/robots/scout01/status")
    assert r.status_code == 200 and r.json()["online"] is False
    app.state.fleet.feed("scout01", "tel/state", {})
    assert client.get("/api/v1/robots/scout01/status").json()["online"] is True
```

- [ ] **Step 2: 실패 확인**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest tests/test_fleet_port.py -q
```

Expected: FAIL — `ModuleNotFoundError: fleet_server.fleet`

- [ ] **Step 3: 구현**

`server/fleet_server/fleet/presence.py`:

```python
from __future__ import annotations

import time


class PresenceRegistry:
    """마지막 수신 시각 기반 온라인 판정 — 15초(스펙 §3.1)."""

    def __init__(self, offline_after_s: float = 15.0):
        self.offline_after_s = offline_after_s
        self._last: dict[str, float] = {}

    def touch(self, robot_id: str, t: float | None = None) -> None:
        self._last[robot_id] = time.time() if t is None else t

    def last_seen(self, robot_id: str) -> float | None:
        return self._last.get(robot_id)

    def online(self, robot_id: str, t: float | None = None) -> bool:
        ls = self._last.get(robot_id)
        if ls is None:
            return False
        now = time.time() if t is None else t
        return (now - ls) <= self.offline_after_s
```

`server/fleet_server/fleet/port.py`:

```python
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable

from .presence import PresenceRegistry

TelemetryHandler = Callable[[str, str, dict, int | None], None]


@dataclass
class RobotStatus:
    online: bool
    last_seen: float | None


class FleetPort(ABC):
    """로봇 연결 추상화 — 스펙 §4.2. M1 구현: legacy_ws, M2: zenoh."""

    @abstractmethod
    def register_robot(self, robot_id: str, farm_id: int, conn_kind: str,
                       config: dict) -> None: ...

    @abstractmethod
    async def send_command(self, robot_id: str, cmd_id: str, action: str,
                           payload: dict) -> str:
        """'sent' | 'offline' — 오프라인이면 즉시 실패, 서버측 큐 금지(스펙 §3.2)."""

    @abstractmethod
    def robot_status(self, robot_id: str) -> RobotStatus: ...

    @abstractmethod
    def set_telemetry_handler(self, cb: TelemetryHandler) -> None: ...


class InMemoryFleetPort(FleetPort):
    """테스트·개발용 — 명령을 sent 리스트에 쌓고 feed() 로 텔레메트리를 주입한다."""

    def __init__(self, offline_after_s: float = 15.0):
        self.presence = PresenceRegistry(offline_after_s)
        self.sent: list[tuple[str, str, str, dict]] = []
        self.robots: dict[str, dict] = {}
        self._handler: TelemetryHandler | None = None

    def register_robot(self, robot_id, farm_id, conn_kind, config):
        self.robots[robot_id] = {"farm_id": farm_id, "conn_kind": conn_kind,
                                 "config": config}

    async def send_command(self, robot_id, cmd_id, action, payload):
        if not self.presence.online(robot_id):
            return "offline"
        self.sent.append((robot_id, cmd_id, action, payload))
        return "sent"

    def robot_status(self, robot_id):
        return RobotStatus(online=self.presence.online(robot_id),
                           last_seen=self.presence.last_seen(robot_id))

    def set_telemetry_handler(self, cb):
        self._handler = cb

    def feed(self, robot_id, channel, payload, seq=None):
        self.presence.touch(robot_id)
        if self._handler:
            self._handler(robot_id, channel, payload, seq)
```

`app.py` 수정 — 시그니처와 state (기존 `create_app` 정의를 교체):

```python
def create_app(settings: Settings | None = None, engine=None, fleet=None) -> FastAPI:
    ...
    from .fleet.port import InMemoryFleetPort
    app.state.fleet = fleet if fleet is not None else InMemoryFleetPort(settings.offline_after_s)
```

`admin_routes.py` 에 상태 라우트 추가 (로봇 섹션 끝):

```python
@router.get("/robots/{robot_id}/status")
def robot_status(robot_id: str, request: Request, db=Depends(get_db),
                 user: User = Depends(current_user)):
    r = db.get(Robot, robot_id)
    if r is None:
        raise HTTPException(404, "로봇이 없습니다")
    scope = farm_scope(db, user)
    if scope is not None and r.farm_id not in scope:
        raise HTTPException(403, "해당 농장 권한이 없습니다")
    st = request.app.state.fleet.robot_status(robot_id)
    return {"online": st.online, "last_seen": st.last_seen}
```

(상단 import 에 `Request` 추가)

`conftest.py` 의 app 픽스처 교체:

```python
@pytest.fixture()
def app():
    from fleet_server.fleet.port import InMemoryFleetPort
    return create_app(_test_settings(), fleet=InMemoryFleetPort())
```

- [ ] **Step 4: 통과 확인 후 커밋**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest -q
cd /home/myhome/YBNML
git add server/fleet_server server/tests
git commit -m "M1: FleetPort 추상화 + 15초 presence + 상태 API"
```

---

### Task 9: 임무 API + 이력 API + 수집(ingest)

**Files:**
- Create: `server/fleet_server/ingest.py`
- Create: `server/fleet_server/api/mission_routes.py`
- Create: `server/fleet_server/api/history_routes.py`
- Modify: `server/fleet_server/app.py` (라우터 등록 2줄)
- Create: `server/tests/test_mission_api.py`
- Create: `server/tests/test_history_api.py`

**Interfaces:**
- Consumes: Task 7 `missions.create/apply`, Task 8 `app.state.fleet`
- Produces:
  - `ingest.track(db, robot_id, payload:dict) -> bool` — payload 에서 x·y·yaw·mode 추출, **로봇당 1 Hz 다운샘플**(마지막 저장 ts 기준)
  - `ingest.event(db, robot_id, channel, seq, payload:dict) -> bool` — (robot, channel, seq) 중복이면 False (IntegrityError 삼킴)
  - API: `POST /api/v1/missions`(operator+), `POST /api/v1/missions/{id}/(pause|resume|cancel)`, `GET /api/v1/missions`, `GET /api/v1/tracks`, `GET /api/v1/events`, `GET /api/v1/audit`(admin)

- [ ] **Step 1: 실패하는 테스트 작성**

`server/tests/test_mission_api.py`:

```python
from tests.conftest import do_login


def _seed_operator(client):
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    fb = client.post("/api/v1/farms", json={"name": "농장B"}, headers=h).json()
    client.post("/api/v1/robots", headers=h,
                json={"id": "scout01", "farm_id": fa["id"], "name": "r"})
    client.post("/api/v1/robots", headers=h,
                json={"id": "scout02", "farm_id": fb["id"], "name": "r2"})
    client.post("/api/v1/users", headers=h, json={
        "login": "op", "password": "oppw", "role": "operator", "farm_ids": [fa["id"]]})
    return fa, fb


def test_mission_flow(client, app):
    _seed_operator(client)
    app.state.fleet.feed("scout01", "tel/state", {})           # 온라인 전환
    csrf = do_login(client, "op", "oppw")
    h = {"X-CSRF": csrf}
    r = client.post("/api/v1/missions", headers=h,
                    json={"robot_id": "scout01", "alleys": [0, 1]})
    assert r.status_code == 200, r.text
    ms = r.json()
    assert ms["state"] == "QUEUED"
    assert app.state.fleet.sent[-1][2] == "mission_start"      # 로봇으로 전달됨
    # 일시정지 → RUNNING 이 아니므로 409 (QUEUED 에서 pause 는 전이 불가)
    assert client.post(f"/api/v1/missions/{ms['id']}/pause", headers=h).status_code == 409
    # 취소는 QUEUED 에서 가능
    assert client.post(f"/api/v1/missions/{ms['id']}/cancel", headers=h).status_code == 200


def test_offline_robot_409(client):
    _seed_operator(client)
    csrf = do_login(client, "op", "oppw")
    r = client.post("/api/v1/missions", headers={"X-CSRF": csrf},
                    json={"robot_id": "scout01", "alleys": [0]})
    assert r.status_code == 409                                # 오프라인 → 즉시 실패


def test_cross_farm_403(client, app):
    _seed_operator(client)
    app.state.fleet.feed("scout02", "tel/state", {})
    csrf = do_login(client, "op", "oppw")                      # op 는 농장A만
    r = client.post("/api/v1/missions", headers={"X-CSRF": csrf},
                    json={"robot_id": "scout02", "alleys": [0]})
    assert r.status_code == 403


def test_observer_cannot_create(client, app):
    fa, _ = _seed_operator(client)
    csrf0 = do_login(client)
    client.post("/api/v1/users", headers={"X-CSRF": csrf0}, json={
        "login": "obs", "password": "obspw", "role": "observer", "farm_ids": [fa["id"]]})
    app.state.fleet.feed("scout01", "tel/state", {})
    csrf = do_login(client, "obs", "obspw")
    r = client.post("/api/v1/missions", headers={"X-CSRF": csrf},
                    json={"robot_id": "scout01", "alleys": [0]})
    assert r.status_code == 403
```

`server/tests/test_history_api.py`:

```python
import datetime as dt

from fleet_server import ingest
from fleet_server import models as m
from tests.conftest import do_login


def _farm_robot(db):
    f = m.Farm(name="농장A"); db.add(f); db.flush()
    db.add(m.Robot(id="scout01", farm_id=f.id, name="r")); db.commit()
    return f


def test_event_dedup(db):
    _farm_robot(db)
    assert ingest.event(db, "scout01", "evt", 5, {"kind": "estop", "msg": "x"}) is True
    assert ingest.event(db, "scout01", "evt", 5, {"kind": "estop", "msg": "재전송"}) is False
    assert db.query(m.Event).count() == 1                      # 스펙 §3.3 중복 제거


def test_track_downsample(db):
    _farm_robot(db)
    t0 = dt.datetime(2026, 8, 1, 12, 0, 0, tzinfo=dt.UTC)
    assert ingest.track(db, "scout01", {"x": 0, "y": 0, "yaw": 0, "ts": t0.timestamp()}) is True
    assert ingest.track(db, "scout01", {"x": 1, "y": 0, "yaw": 0,
                                        "ts": t0.timestamp() + 0.2}) is False   # 1 Hz 미만
    assert ingest.track(db, "scout01", {"x": 2, "y": 0, "yaw": 0,
                                        "ts": t0.timestamp() + 1.1}) is True
    assert db.query(m.Track).count() == 2


def test_history_routes(client, app):
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    client.post("/api/v1/robots", headers=h,
                json={"id": "scout01", "farm_id": fa["id"], "name": "r"})
    with app.state.session_factory() as db:
        ingest.event(db, "scout01", "evt", 1, {"kind": "estop", "msg": "정지"})
        ingest.track(db, "scout01", {"x": 1.5, "y": 2.5, "yaw": 0.1, "ts": 1000.0})
    evs = client.get("/api/v1/events?robot_id=scout01").json()
    assert evs and evs[0]["kind"] == "estop"
    trs = client.get("/api/v1/tracks?robot_id=scout01").json()
    assert trs and trs[0]["x"] == 1.5
    assert client.get("/api/v1/audit").status_code == 200      # admin 은 가능


def test_audit_admin_only(client):
    fa_csrf = do_login(client)
    client.post("/api/v1/users", headers={"X-CSRF": fa_csrf}, json={
        "login": "obs", "password": "obspw", "role": "observer", "farm_ids": []})
    do_login(client, "obs", "obspw")
    assert client.get("/api/v1/audit").status_code == 403
```

- [ ] **Step 2: 실패 확인**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest tests/test_mission_api.py tests/test_history_api.py -q
```

Expected: FAIL — `ModuleNotFoundError: fleet_server.ingest` / 404

- [ ] **Step 3: 구현**

`server/fleet_server/ingest.py`:

```python
"""텔레메트리 → DB 수집. 중복 제거는 DB 유니크 제약으로 영속화(스펙 §3.3)."""
from __future__ import annotations

import datetime as dt

from sqlalchemy.exc import IntegrityError

from .models import Event, Track

_last_track_ts: dict[str, float] = {}          # 로봇별 마지막 저장 ts (1 Hz 다운샘플)


def _ts(payload: dict) -> dt.datetime:
    raw = payload.get("ts")
    if raw is None:
        return dt.datetime.now(dt.UTC)
    return dt.datetime.fromtimestamp(float(raw), dt.UTC)


def track(db, robot_id: str, payload: dict) -> bool:
    t = _ts(payload).timestamp()
    last = _last_track_ts.get(robot_id)
    if last is not None and (t - last) < 1.0:
        return False
    _last_track_ts[robot_id] = t
    db.add(Track(robot_id=robot_id, ts=_ts(payload),
                 x=float(payload.get("x", 0.0)), y=float(payload.get("y", 0.0)),
                 yaw=float(payload.get("yaw", 0.0)), mode=str(payload.get("mode", ""))))
    db.commit()
    return True


def event(db, robot_id: str, channel: str | None, seq: int | None,
          payload: dict) -> bool:
    row = Event(robot_id=robot_id, ts=_ts(payload), channel=channel, seq=seq,
                kind=str(payload.get("kind", "unknown")),
                severity=str(payload.get("severity", "info")),
                msg=str(payload.get("msg", ""))[:256], payload_json=payload)
    db.add(row)
    try:
        db.commit()
        return True
    except IntegrityError:                     # (robot, channel, seq) 중복 → 무해화
        db.rollback()
        return False
```

`server/fleet_server/api/mission_routes.py`:

```python
from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel

from .. import audit, missions
from ..deps import csrf_protect, current_user, farm_scope, get_db, require_min_role
from ..models import Mission, Robot, User

router = APIRouter(tags=["missions"])
_operator = Depends(require_min_role("operator"))
_csrf = Depends(csrf_protect)


class MissionBody(BaseModel):
    robot_id: str
    alleys: list[int]


def _scoped_robot(db, user, robot_id) -> Robot:
    r = db.get(Robot, robot_id)
    if r is None:
        raise HTTPException(404, "로봇이 없습니다")
    scope = farm_scope(db, user)
    if scope is not None and r.farm_id not in scope:
        raise HTTPException(403, "해당 농장 권한이 없습니다")
    return r


def _mission_out(ms: Mission) -> dict:
    return {"id": ms.id, "robot_id": ms.robot_id, "farm_id": ms.farm_id,
            "state": ms.state, "spec": ms.spec_json,
            "created_at": ms.created_at.isoformat(),
            "started_at": ms.started_at.isoformat() if ms.started_at else None,
            "ended_at": ms.ended_at.isoformat() if ms.ended_at else None}


@router.post("/missions", dependencies=[_operator, _csrf])
async def create_mission(body: MissionBody, request: Request, db=Depends(get_db),
                         user: User = Depends(current_user)):
    robot = _scoped_robot(db, user, body.robot_id)
    fleet = request.app.state.fleet
    ms = missions.create(db, robot_id=robot.id, farm_id=robot.farm_id,
                         spec={"alleys": body.alleys}, created_by=user.id)
    result = await fleet.send_command(robot.id, f"m{ms.id}", "mission_start",
                                      {"alleys": body.alleys, "mission_id": ms.id})
    if result == "offline":                    # 오프라인 → 즉시 실패 + 잔재 제거
        missions.apply(db, ms, "cancel")
        audit.record(db, action="mission_start", result="rejected", user_id=user.id,
                     role=user.role, target=robot.id, detail="로봇 오프라인")
        raise HTTPException(409, "로봇이 오프라인입니다")
    audit.record(db, action="mission_start", result="accepted", user_id=user.id,
                 role=user.role, target=robot.id, detail=f"alleys={body.alleys}")
    return _mission_out(ms)


_EVENT_BY_VERB = {"pause": "mission_pause", "resume": "mission_resume",
                  "cancel": "mission_cancel"}


@router.post("/missions/{mission_id}/{verb}", dependencies=[_operator, _csrf])
async def mission_verb(mission_id: int, verb: str, request: Request,
                       db=Depends(get_db), user: User = Depends(current_user)):
    if verb not in _EVENT_BY_VERB:
        raise HTTPException(404, "지원하지 않는 동작")
    ms = db.get(Mission, mission_id)
    if ms is None:
        raise HTTPException(404, "임무가 없습니다")
    _scoped_robot(db, user, ms.robot_id)
    try:
        missions.apply(db, ms, verb)
    except missions.InvalidTransition as e:
        raise HTTPException(409, str(e))
    result = await request.app.state.fleet.send_command(
        ms.robot_id, f"m{ms.id}-{verb}", _EVENT_BY_VERB[verb], {"mission_id": ms.id})
    audit.record(db, action=_EVENT_BY_VERB[verb],
                 result="accepted" if result == "sent" else "rejected",
                 user_id=user.id, role=user.role, target=ms.robot_id,
                 detail=f"mission={ms.id} 전달={result}")
    return {**_mission_out(ms), "delivery": result}


@router.get("/missions")
def list_missions(farm_id: int | None = None, robot_id: str | None = None,
                  db=Depends(get_db), user: User = Depends(current_user)):
    scope = farm_scope(db, user)
    q = db.query(Mission)
    if scope is not None:
        q = q.filter(Mission.farm_id.in_(scope))
    if farm_id is not None:
        q = q.filter(Mission.farm_id == farm_id)
    if robot_id is not None:
        q = q.filter(Mission.robot_id == robot_id)
    return [_mission_out(ms) for ms in q.order_by(Mission.id.desc()).limit(200)]
```

`server/fleet_server/api/history_routes.py`:

```python
from __future__ import annotations

import datetime as dt

from fastapi import APIRouter, Depends, HTTPException

from ..deps import current_user, farm_scope, get_db, require_min_role
from ..models import AuditLog, Event, Robot, Track, User

router = APIRouter(tags=["history"])


def _scoped_robot_ids(db, user, robot_id: str | None) -> list[str]:
    scope = farm_scope(db, user)
    q = db.query(Robot.id)
    if scope is not None:
        q = q.filter(Robot.farm_id.in_(scope))
    ids = [r[0] for r in q]
    if robot_id is not None:
        if robot_id not in ids:
            raise HTTPException(403, "해당 로봇 권한이 없습니다")
        return [robot_id]
    return ids


@router.get("/tracks")
def tracks(robot_id: str | None = None, from_ts: float | None = None,
           to_ts: float | None = None, db=Depends(get_db),
           user: User = Depends(current_user)):
    ids = _scoped_robot_ids(db, user, robot_id)
    q = db.query(Track).filter(Track.robot_id.in_(ids))
    if from_ts is not None:
        q = q.filter(Track.ts >= dt.datetime.fromtimestamp(from_ts, dt.UTC))
    if to_ts is not None:
        q = q.filter(Track.ts <= dt.datetime.fromtimestamp(to_ts, dt.UTC))
    return [{"robot_id": t.robot_id, "ts": t.ts.isoformat(), "x": t.x, "y": t.y,
             "yaw": t.yaw, "mode": t.mode}
            for t in q.order_by(Track.ts).limit(10000)]


@router.get("/events")
def events(robot_id: str | None = None, limit: int = 200, db=Depends(get_db),
           user: User = Depends(current_user)):
    ids = _scoped_robot_ids(db, user, robot_id)
    q = (db.query(Event).filter(Event.robot_id.in_(ids))
         .order_by(Event.id.desc()).limit(min(limit, 1000)))
    return [{"robot_id": e.robot_id, "ts": e.ts.isoformat(), "kind": e.kind,
             "severity": e.severity, "msg": e.msg} for e in q]


@router.get("/audit", dependencies=[Depends(require_min_role("admin"))])
def audit_rows(limit: int = 200, db=Depends(get_db)):
    q = db.query(AuditLog).order_by(AuditLog.id.desc()).limit(min(limit, 1000))
    return [{"ts": a.ts.isoformat(), "user_id": a.user_id, "role": a.role,
             "action": a.action, "target": a.target, "result": a.result,
             "detail": a.detail} for a in q]
```

`app.py` 라우터 등록 (admin_routes 아래에):

```python
    from .api import history_routes, mission_routes
    app.include_router(mission_routes.router, prefix="/api/v1")
    app.include_router(history_routes.router, prefix="/api/v1")
```

주의: `ingest._last_track_ts` 는 모듈 전역이라 테스트 간 오염된다 — `conftest.py` 의 `db` 픽스처 앞에 초기화 1줄 추가:

```python
@pytest.fixture(autouse=True)
def _reset_ingest_state():
    from fleet_server import ingest
    ingest._last_track_ts.clear()
```

- [ ] **Step 4: 통과 확인 후 커밋**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest -q
cd /home/myhome/YBNML
git add server/fleet_server server/tests
git commit -m "M1: 임무 API(REST 정본)·이력 API·수집(1Hz 다운샘플·중복 제거)"
```

---

### Task 10: 레거시 로봇 WS 어댑터 + FleetService 결선

**Files:**
- Create: `server/fleet_server/fleet/legacy_ws.py`
- Create: `server/fleet_server/fleet/service.py`
- Modify: `server/fleet_server/app.py` (lifespan 결선)
- Create: `server/tests/test_legacy_ws.py`

**Interfaces:**
- Consumes: Task 8 `FleetPort`, Task 9 `ingest`
- Produces:
  - `legacy_ws.SUFFIX_TO_CHANNEL: dict` (Global Constraints 의 매핑 그대로)
  - `legacy_ws.LegacyRobotLink(robot_id, ws_url, token, on_message, on_touch)` — `run()` 코루틴(재연결 1→30 s), `send_command(action, payload) -> bool`, `stop()`
  - `legacy_ws.LegacyFleetPort(offline_after_s)` — FleetPort 구현. `start(loop)` 후 `register_robot` 시 링크 태스크 생성
  - `service.FleetService(session_factory)` — `attach(fleet)` 로 텔레메트리 핸들러 등록. `latest[robot_id][channel] -> payload` 캐시, `subscribe(cb) -> unsub` (cb: `(robot_id, channel, payload)`), tel/state→`ingest.track`, evt→`ingest.event`, mission 채널 payload 의 `{"state": "done"|"paused"|"running"|"canceled"|"failed"}` → 해당 임무 상태기계 전이(불가 전이는 무시)
- 한계(명시): 레거시 로봇에는 ack 채널이 없어 `send_command` 는 소켓 기록 성공 = "sent". cmd_id 상관은 M2 Zenoh 에서.

- [ ] **Step 1: 실패하는 테스트 작성**

`server/tests/test_legacy_ws.py`:

```python
import asyncio
import json

import pytest
import websockets

from fleet_server.fleet.legacy_ws import (SUFFIX_TO_CHANNEL, LegacyFleetPort,
                                          LegacyRobotLink)


def test_suffix_mapping_fixed():
    assert SUFFIX_TO_CHANNEL == {
        "state": "tel/state", "health": "tel/health", "map": "tel/map",
        "event": "evt", "mission": "mission", "hello": "hello"}


@pytest.mark.asyncio
async def test_link_receives_and_sends():
    got, touched, received_by_robot = [], [], []

    async def fake_robot(ws):
        # 토큰이 쿼리로 왔는지 확인
        assert "token=RTOK" in ws.request.path
        await ws.send(json.dumps({"v": 1, "topic": "orchard/scout01/state",
                                  "ts_ns": 1, "seq": 10,
                                  "payload": {"x": 1.0, "y": 2.0}}))
        await ws.send(json.dumps({"v": 1, "topic": "orchard/scout01/event",
                                  "ts_ns": 2, "seq": 11,
                                  "payload": {"kind": "estop"}}))
        async for raw in ws:
            received_by_robot.append(json.loads(raw))

    async with websockets.serve(fake_robot, "127.0.0.1", 18099):
        link = LegacyRobotLink(
            "scout01", "ws://127.0.0.1:18099/ws", "RTOK",
            on_message=lambda r, ch, pl, seq: got.append((r, ch, pl, seq)),
            on_touch=lambda r: touched.append(r))
        task = asyncio.create_task(link.run())
        for _ in range(100):                       # 수신 대기 (최대 5초)
            if len(got) >= 2:
                break
            await asyncio.sleep(0.05)
        assert ("scout01", "tel/state", {"x": 1.0, "y": 2.0}, 10) in got
        assert ("scout01", "evt", {"kind": "estop"}, 11) in got
        assert touched                             # presence touch 호출됨
        ok = await link.send_command("estop", {})
        assert ok is True
        for _ in range(100):
            if received_by_robot:
                break
            await asyncio.sleep(0.05)
        env = received_by_robot[0]
        assert env["topic"] == "orchard/scout01/cmd"
        assert env["payload"]["cmd"] == "estop"    # 기존 봉투 형식 유지
        link.stop(); task.cancel()


@pytest.mark.asyncio
async def test_port_offline_immediate():
    fp = LegacyFleetPort(offline_after_s=15.0)
    fp.register_robot("ghost", 1, "legacy_ws",
                      {"ws_url": "ws://127.0.0.1:1/ws", "token": ""})
    assert await fp.send_command("ghost", "c1", "estop", {}) == "offline"
```

- [ ] **Step 2: 실패 확인**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest tests/test_legacy_ws.py -q
```

Expected: FAIL — `ModuleNotFoundError: fleet_server.fleet.legacy_ws`

- [ ] **Step 3: 구현**

`server/fleet_server/fleet/legacy_ws.py`:

```python
"""레거시 로봇 어댑터 — M1 한시, M2 에서 Zenoh 로 교체(스펙 §9 M1).

서버가 로봇의 기존 WebSocket 서버(ws://로봇:8080/ws?token=...)에 클라이언트로
접속한다. 기존 봉투 {v, topic:"orchard/{robot}/{suffix}", ts_ns, seq, payload}
를 fleet 채널로 매핑한다.

한계: 레거시 로봇에는 ack 채널이 없다 → send_command 는 소켓 기록 성공을
"sent" 로 간주한다 (cmd_id 상관 응답은 M2).
"""
from __future__ import annotations

import asyncio
import json
import time

import websockets

from .port import FleetPort, RobotStatus, TelemetryHandler
from .presence import PresenceRegistry

SUFFIX_TO_CHANNEL = {
    "state": "tel/state", "health": "tel/health", "map": "tel/map",
    "event": "evt", "mission": "mission", "hello": "hello",
}


class LegacyRobotLink:
    def __init__(self, robot_id: str, ws_url: str, token: str,
                 on_message, on_touch):
        self.robot_id = robot_id
        self.ws_url = ws_url
        self.token = token
        self.on_message = on_message          # (robot_id, channel, payload, seq)
        self.on_touch = on_touch              # (robot_id)
        self._ws = None
        self._stop = False
        self._seq = 0

    async def run(self) -> None:
        backoff = 1.0
        url = self.ws_url + (f"?token={self.token}" if self.token else "")
        while not self._stop:
            try:
                async with websockets.connect(url, open_timeout=5) as ws:
                    self._ws = ws
                    backoff = 1.0
                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                        except (ValueError, TypeError):
                            continue
                        self.on_touch(self.robot_id)
                        parts = str(msg.get("topic", "")).split("/", 2)
                        if len(parts) < 3:
                            continue
                        ch = SUFFIX_TO_CHANNEL.get(parts[2])
                        if ch:
                            self.on_message(self.robot_id, ch,
                                            msg.get("payload", {}), msg.get("seq"))
            except Exception:
                pass
            self._ws = None
            if self._stop:
                break
            await asyncio.sleep(backoff)          # 재연결 지수 백오프 (스펙 §3.1)
            backoff = min(backoff * 2, 30.0)

    async def send_command(self, action: str, payload: dict) -> bool:
        ws = self._ws
        if ws is None:
            return False
        self._seq += 1
        suffix = "teleop" if action == "teleop" else "cmd"
        body = payload if action == "teleop" else {"cmd": action, **payload}
        env = {"v": 1, "topic": f"orchard/{self.robot_id}/{suffix}",
               "ts_ns": time.time_ns(), "seq": self._seq, "payload": body}
        try:
            await ws.send(json.dumps(env))
            return True
        except Exception:
            return False

    def stop(self) -> None:
        self._stop = True


class LegacyFleetPort(FleetPort):
    def __init__(self, offline_after_s: float = 15.0):
        self.presence = PresenceRegistry(offline_after_s)
        self._links: dict[str, LegacyRobotLink] = {}
        self._tasks: dict[str, asyncio.Task] = {}
        self._handler: TelemetryHandler | None = None

    def register_robot(self, robot_id, farm_id, conn_kind, config):
        if robot_id in self._links or conn_kind != "legacy_ws":
            return
        link = LegacyRobotLink(robot_id, config.get("ws_url", ""),
                               config.get("token", ""),
                               on_message=self._on_message,
                               on_touch=self.presence.touch)
        self._links[robot_id] = link
        self._tasks[robot_id] = asyncio.get_running_loop().create_task(link.run())

    def _on_message(self, robot_id, channel, payload, seq):
        if self._handler:
            self._handler(robot_id, channel, payload, seq)

    async def send_command(self, robot_id, cmd_id, action, payload):
        link = self._links.get(robot_id)
        if link is None or not self.presence.online(robot_id):
            return "offline"                    # 즉시 실패 — 서버측 큐 금지
        ok = await link.send_command(action, {**payload, "cmd_id": cmd_id})
        return "sent" if ok else "offline"

    def robot_status(self, robot_id):
        return RobotStatus(online=self.presence.online(robot_id),
                           last_seen=self.presence.last_seen(robot_id))

    def set_telemetry_handler(self, cb):
        self._handler = cb

    async def shutdown(self):
        for link in self._links.values():
            link.stop()
        for task in self._tasks.values():
            task.cancel()
```

`server/fleet_server/fleet/service.py`:

```python
"""텔레메트리 허브 — FleetPort 수신을 DB 수집·최신값 캐시·구독자 팬아웃으로.

mission 채널 payload {"state": ...} 는 임무 상태기계로 동기화한다
(로봇이 완료를 보고하면 서버 임무도 DONE 이 된다).
"""
from __future__ import annotations

from typing import Callable

from .. import ingest, missions
from ..models import Mission

_ROBOT_STATE_EVENT = {"running": "start", "paused": "pause", "done": "complete",
                      "canceled": "cancel", "failed": "fail"}


class FleetService:
    def __init__(self, session_factory):
        self._factory = session_factory
        self.latest: dict[str, dict[str, dict]] = {}
        self._subs: list[Callable[[str, str, dict], None]] = []

    def attach(self, fleet) -> None:
        fleet.set_telemetry_handler(self.on_telemetry)

    def subscribe(self, cb: Callable[[str, str, dict], None]):
        self._subs.append(cb)
        def unsub():
            if cb in self._subs:
                self._subs.remove(cb)
        return unsub

    def on_telemetry(self, robot_id: str, channel: str, payload: dict,
                     seq: int | None) -> None:
        self.latest.setdefault(robot_id, {})[channel] = payload
        if channel == "tel/state":
            with self._factory() as db:
                ingest.track(db, robot_id, payload)
        elif channel == "evt":
            with self._factory() as db:
                ingest.event(db, robot_id, channel, seq, payload)
        elif channel == "mission":
            self._sync_mission(robot_id, payload)
        for cb in list(self._subs):
            cb(robot_id, channel, payload)

    def _sync_mission(self, robot_id: str, payload: dict) -> None:
        ev = _ROBOT_STATE_EVENT.get(str(payload.get("state", "")).lower())
        if ev is None:
            return
        with self._factory() as db:
            ms = (db.query(Mission).filter(Mission.robot_id == robot_id,
                                           Mission.state.in_(["QUEUED", "RUNNING", "PAUSED"]))
                  .order_by(Mission.id.desc()).first())
            if ms is None:
                return
            try:
                missions.apply(db, ms, ev, payload=payload)
            except missions.InvalidTransition:
                pass                            # 이미 같은 상태 등 — 무시
```

`app.py` — `create_app` 에서 FleetService 결선 + lifespan 으로 레거시 링크 기동 (기존 `app.state.fleet = ...` 자리를 교체):

```python
import contextlib

from .fleet.port import InMemoryFleetPort
from .fleet.service import FleetService


def create_app(settings=None, engine=None, fleet=None) -> FastAPI:
    settings = settings or load_settings()
    engine = engine or make_engine(settings.db_url)
    Base.metadata.create_all(engine)
    session_factory = make_session_factory(engine)

    use_legacy = fleet is None                  # 운영: lifespan 에서 레거시 기동

    @contextlib.asynccontextmanager
    async def lifespan(app: FastAPI):
        if use_legacy:
            from .fleet.legacy_ws import LegacyFleetPort
            from .models import Robot
            lp = LegacyFleetPort(settings.offline_after_s)
            app.state.fleet = lp
            app.state.fleet_service.attach(lp)
            with session_factory() as db:
                for r in db.query(Robot).filter(Robot.conn_kind == "legacy_ws"):
                    lp.register_robot(r.id, r.farm_id, r.conn_kind, r.config_json)
        yield
        if use_legacy and hasattr(app.state.fleet, "shutdown"):
            await app.state.fleet.shutdown()

    app = FastAPI(title="과수원 통합관제 서버", lifespan=lifespan)
    app.state.settings = settings
    app.state.engine = engine
    app.state.session_factory = session_factory
    app.state.fleet = fleet if fleet is not None else InMemoryFleetPort(settings.offline_after_s)
    app.state.fleet_service = FleetService(session_factory)
    if fleet is not None:
        app.state.fleet_service.attach(fleet)
    ...  # (라우터 등록·부트스트랩·index 라우트는 기존 그대로)
    return app
```

또한 admin_routes 의 `create_robot` 끝에 동적 등록 1줄을 추가한다 (`return` 직전):

```python
    request.app.state.fleet.register_robot(r.id, r.farm_id, r.conn_kind, r.config_json)
```

(`create_robot(body, request: Request, db=...)` 로 시그니처에 `request` 추가. InMemoryFleetPort 에서는 등록만 되고 링크는 없다 — 테스트 영향 없음)

- [ ] **Step 4: 통과 확인 후 커밋**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest -q
cd /home/myhome/YBNML
git add server/fleet_server server/tests
git commit -m "M1: 레거시 WS 어댑터(한시) + FleetService 텔레메트리 허브"
```

---

### Task 11: 브라우저 WS 게이트웨이 — Origin·세션·스코프·명령

**Files:**
- Create: `server/fleet_server/ws.py`
- Modify: `server/fleet_server/app.py` (라우터 등록 1줄)
- Create: `server/tests/test_ws_gateway.py`

**Interfaces:**
- Consumes: Task 4 세션, Task 8 fleet, Task 10 `FleetService.subscribe/latest`
- Produces: `WS /ws`. 프로토콜 (대시보드 v2 가 사용):
  - 서버→브라우저: `{"type":"ready"}` → 이후 `{"topic":"fleet/v1/{farm}/{robot}/{channel}","payload":{...}}`, 명령 응답 `{"type":"cmd_result","robot","cmd_id","result"}`, `{"type":"denied","action","reason"}`, `{"type":"stop_all_result","results":{robot:"sent"|"offline"}}`
  - 브라우저→서버: `{"type":"cmd","action","robot","cmd_id","payload"?}`, `{"type":"teleop","robot","payload":{"vx","wz"}}`
- 규칙: `mission_*` 는 WS 에서 거부(REST 정본). estop·stop_all 은 observer 허용(D9). 텔레옵 감사는 세션 단위.

- [ ] **Step 1: 실패하는 테스트 작성**

`server/tests/test_ws_gateway.py`:

```python
import pytest

from fleet_server.models import AuditLog
from tests.conftest import do_login

ORIGIN = {"origin": "http://testserver"}


def _seed(client):
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    fb = client.post("/api/v1/farms", json={"name": "농장B"}, headers=h).json()
    client.post("/api/v1/robots", headers=h, json={"id": "r1", "farm_id": fa["id"], "name": "r1"})
    client.post("/api/v1/robots", headers=h, json={"id": "r2", "farm_id": fa["id"], "name": "r2"})
    client.post("/api/v1/robots", headers=h, json={"id": "rb", "farm_id": fb["id"], "name": "rb"})
    client.post("/api/v1/users", headers=h, json={
        "login": "obs", "password": "obspw", "role": "observer", "farm_ids": [fa["id"]]})
    client.post("/api/v1/users", headers=h, json={
        "login": "op", "password": "oppw", "role": "operator", "farm_ids": [fa["id"]]})
    return fa, fb


def test_origin_rejected(client):
    _seed(client)
    do_login(client, "obs", "obspw")
    with pytest.raises(Exception):
        with client.websocket_connect("/ws", headers={"origin": "http://evil"}) as ws:
            ws.receive_json()


def test_no_session_rejected(client):
    _seed(client)
    client.cookies.clear()
    with pytest.raises(Exception):
        with client.websocket_connect("/ws", headers=ORIGIN) as ws:
            ws.receive_json()


def test_telemetry_fanout_scoped(client, app):
    _seed(client)
    do_login(client, "obs", "obspw")
    with client.websocket_connect("/ws", headers=ORIGIN) as ws:
        assert ws.receive_json()["type"] == "ready"
        app.state.fleet.feed("r1", "tel/state", {"x": 5.0})
        msg = ws.receive_json()
        assert msg["topic"].endswith("/r1/tel/state") and msg["payload"]["x"] == 5.0
        # 타 농장(rb) 텔레메트리는 오지 않는다 — r1 것만 한 번 더 확인
        app.state.fleet.feed("rb", "tel/state", {"x": 9.0})
        app.state.fleet.feed("r1", "tel/health", {"ok": 1})
        assert ws.receive_json()["topic"].endswith("/r1/tel/health")


def test_observer_estop_allowed_teleop_denied(client, app):
    _seed(client)
    app.state.fleet.feed("r1", "tel/state", {})
    do_login(client, "obs", "obspw")
    with client.websocket_connect("/ws", headers=ORIGIN) as ws:
        ws.receive_json()
        ws.send_json({"type": "cmd", "action": "estop", "robot": "r1", "cmd_id": "c1"})
        r = ws.receive_json()
        assert r == {"type": "cmd_result", "robot": "r1", "cmd_id": "c1", "result": "sent"}
        assert app.state.fleet.sent[-1][2] == "estop"          # D9
        ws.send_json({"type": "teleop", "robot": "r1", "payload": {"vx": 0.3, "wz": 0}})
        assert ws.receive_json()["type"] == "denied"
    with app.state.session_factory() as db:
        acts = [(a.action, a.result) for a in db.query(AuditLog)]
        assert ("estop", "accepted") in acts and ("teleop", "rejected") in acts


def test_mission_via_ws_denied(client, app):
    _seed(client)
    app.state.fleet.feed("r1", "tel/state", {})
    do_login(client, "op", "oppw")
    with client.websocket_connect("/ws", headers=ORIGIN) as ws:
        ws.receive_json()
        ws.send_json({"type": "cmd", "action": "mission_start", "robot": "r1", "cmd_id": "c2"})
        r = ws.receive_json()
        assert r["type"] == "denied" and "REST" in r["reason"]


def test_stop_all_partial(client, app):
    _seed(client)
    app.state.fleet.feed("r1", "tel/state", {})               # r1 만 온라인
    do_login(client, "obs", "obspw")
    with client.websocket_connect("/ws", headers=ORIGIN) as ws:
        ws.receive_json()
        ws.send_json({"type": "cmd", "action": "stop_all", "cmd_id": "c3"})
        r = ws.receive_json()
        assert r["type"] == "stop_all_result"
        assert r["results"] == {"r1": "sent", "r2": "offline"}  # 부분 실패를 숨기지 않는다


def test_teleop_session_audit_once(client, app):
    _seed(client)
    app.state.fleet.feed("r1", "tel/state", {})
    do_login(client, "op", "oppw")
    with client.websocket_connect("/ws", headers=ORIGIN) as ws:
        ws.receive_json()
        for _ in range(3):
            ws.send_json({"type": "teleop", "robot": "r1", "payload": {"vx": 0.2, "wz": 0}})
        ws.send_json({"type": "cmd", "action": "ping", "robot": "r1", "cmd_id": "p"})
        ws.receive_json()                                      # ping 응답까지 대기(순서 보장)
    teleops = [s for s in app.state.fleet.sent if s[2] == "teleop"]
    assert len(teleops) == 3                                   # 지령은 전부 전달
    with app.state.session_factory() as db:
        rows = db.query(AuditLog).filter(AuditLog.action == "teleop_session").all()
        assert len(rows) == 1                                  # 감사는 세션 단위 (S7)
```

- [ ] **Step 2: 실패 확인**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest tests/test_ws_gateway.py -q
```

Expected: FAIL — 404 (/ws 없음)

- [ ] **Step 3: 구현**

`server/fleet_server/ws.py`:

```python
"""브라우저 WebSocket 게이트웨이 — 스펙 §4.4·§4.5.

인증: 세션 쿠키 + Origin 허용 목록(교차 출처 WS 하이재킹 차단, 스펙 §3.6).
mission_* 는 REST 정본이므로 WS 에서 거부한다.
"""
from __future__ import annotations

import asyncio

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from . import audit, auth
from .deps import SESSION_COOKIE, _session_pair, farm_scope
from .models import Robot

router = APIRouter()

_WS_ACTIONS = {"estop", "clear_estop", "stop_all", "set_mode", "ping"}


class _Conn:
    def __init__(self, user, scope, robot_farm):
        self.user = user
        self.scope = scope                      # None=admin 전체
        self.robot_farm = robot_farm            # robot_id -> farm_id
        self.teleop_audited: set[str] = set()

    def sees(self, robot_id: str) -> bool:
        farm = self.robot_farm.get(robot_id)
        return farm is not None and (self.scope is None or farm in self.scope)


@router.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    app = websocket.app
    settings = app.state.settings
    origin = websocket.headers.get("origin", "")
    if settings.allowed_origins and origin not in settings.allowed_origins:
        await websocket.close(code=4403)
        return

    db = app.state.session_factory()
    try:
        class _Req:                              # _session_pair 는 Request 형태만 필요
            cookies = websocket.cookies
        pair = _session_pair(_Req, db)
        if pair is None:
            await websocket.close(code=4401)
            return
        user = pair[1]
        scope = farm_scope(db, user)
        robot_farm = {r.id: r.farm_id for r in db.query(Robot)}
    finally:
        db.close()

    conn = _Conn(user, scope, robot_farm)
    await websocket.accept()
    loop = asyncio.get_running_loop()
    queue: asyncio.Queue = asyncio.Queue(maxsize=1000)
    svc = app.state.fleet_service

    def on_tel(robot_id: str, channel: str, payload: dict):
        if not conn.sees(robot_id):
            return
        item = {"topic": f"fleet/v1/{conn.robot_farm[robot_id]}/{robot_id}/{channel}",
                "payload": payload}
        try:
            loop.call_soon_threadsafe(queue.put_nowait, item)
        except RuntimeError:
            pass

    unsub = svc.subscribe(on_tel)

    async def sender():
        while True:
            await websocket.send_json(await queue.get())

    send_task = asyncio.create_task(sender())
    # 초기 스냅샷 (최신값 캐시)
    for rid, chans in svc.latest.items():
        for ch, pl in chans.items():
            on_tel(rid, ch, pl)
    await websocket.send_json({"type": "ready"})

    def _audit(action, result, target="", detail=""):
        with app.state.session_factory() as adb:
            audit.record(adb, action=action, result=result, user_id=user.id,
                         role=user.role, target=target, detail=detail)

    try:
        while True:
            msg = await websocket.receive_json()
            mtype = msg.get("type")

            if mtype == "teleop":
                robot = str(msg.get("robot", ""))
                if not auth.authorize(user.role, "teleop") or not conn.sees(robot):
                    await websocket.send_json({"type": "denied", "action": "teleop",
                                               "reason": "권한이 없습니다"})
                    _audit("teleop", "rejected", robot)
                    continue
                if robot not in conn.teleop_audited:            # 세션 단위 감사 (S7)
                    conn.teleop_audited.add(robot)
                    _audit("teleop_session", "accepted", robot, "텔레옵 시작")
                await app.state.fleet.send_command(robot, "", "teleop",
                                                   msg.get("payload", {}))
                continue

            if mtype != "cmd":
                continue
            action = str(msg.get("action", ""))
            cmd_id = str(msg.get("cmd_id", ""))

            if action.startswith("mission_"):
                await websocket.send_json({"type": "denied", "action": action,
                                           "reason": "임무는 REST API 를 사용하세요"})
                continue
            if action not in _WS_ACTIONS or not auth.authorize(user.role, action):
                await websocket.send_json({"type": "denied", "action": action,
                                           "reason": "권한이 없습니다"})
                _audit(action or "unknown", "rejected", detail="WS 명령 거부")
                continue

            if action == "stop_all":                            # 스코프 내 전 로봇 팬아웃
                results = {}
                for rid in sorted(conn.robot_farm):
                    if conn.sees(rid):
                        results[rid] = await app.state.fleet.send_command(
                            rid, f"{cmd_id}-{rid}", "estop", {})
                await websocket.send_json({"type": "stop_all_result", "results": results})
                _audit("stop_all", "accepted", detail=str(results))
                continue

            robot = str(msg.get("robot", ""))
            if not conn.sees(robot):
                await websocket.send_json({"type": "denied", "action": action,
                                           "reason": "해당 농장 권한이 없습니다"})
                _audit(action, "rejected", robot, "농장 스코프 밖")
                continue
            result = await app.state.fleet.send_command(robot, cmd_id, action,
                                                        msg.get("payload", {}))
            await websocket.send_json({"type": "cmd_result", "robot": robot,
                                       "cmd_id": cmd_id, "result": result})
            _audit(action, "accepted" if result == "sent" else "rejected",
                   robot, f"전달={result}")
    except WebSocketDisconnect:
        pass
    finally:
        unsub()
        send_task.cancel()
        for robot in conn.teleop_audited:
            _audit("teleop_session", "accepted", robot, "텔레옵 종료")
```

`app.py` 에 등록:

```python
    from . import ws as ws_module
    app.include_router(ws_module.router)
```

- [ ] **Step 4: 통과 확인 후 커밋**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest -q
cd /home/myhome/YBNML
git add server/fleet_server server/tests
git commit -m "M1: 브라우저 WS 게이트웨이 — Origin·세션·스코프·stop_all·세션단위 텔레옵 감사"
```

---

### Task 12: 대시보드 v2 — 로그인·단일 WS 개조

**Files:**
- Create: `server/web/index.html` (기존 파일 복사 후 개조)
- Create: `server/tests/test_dashboard_serving.py`

기존 대시보드는 로봇마다 직접 WebSocket 을 열었다 (`robots = new Map()` 이 url 키, `connect(r)` 이 `?token=` 접속). v2 는 **서버 한 곳**에 접속한다. 개조 지점은 4곳이다.

- [ ] **Step 1: 원본 복사**

```bash
mkdir -p /home/myhome/YBNML/server/web
cp /home/myhome/YBNML/ros2_ws/src/orchard_sim/web/index.html /home/myhome/YBNML/server/web/index.html
```

- [ ] **Step 2: 실패하는 테스트 작성**

`server/tests/test_dashboard_serving.py`:

```python
def test_index_served_with_login(client):
    html = client.get("/").text
    assert "로그인" in html            # 로그인 오버레이 존재
    assert "관제 v2" in html           # v2 마커
    assert "?token=" not in html       # 로봇 직결 토큰 방식 제거됨
```

- [ ] **Step 3: 개조**

`server/web/index.html` 을 연다. 아래 4개 개조를 순서대로 적용한다.

**(a) 로그인 오버레이** — `<body>` 여는 태그 바로 뒤에 삽입:

```html
<!-- 관제 v2 : 로그인 -->
<div id="login-overlay" style="position:fixed;inset:0;background:#0d1117;z-index:99;
     display:flex;align-items:center;justify-content:center">
  <form id="login-form" style="background:#161b22;padding:28px 32px;border-radius:10px;
       display:flex;flex-direction:column;gap:10px;min-width:280px">
    <h2 style="margin:0 0 6px">과수원 통합관제 로그인</h2>
    <input id="login-id" placeholder="아이디" autocomplete="username">
    <input id="login-pw" type="password" placeholder="비밀번호" autocomplete="current-password">
    <button type="submit">로그인</button>
    <div id="login-err" style="color:#f85149;font-size:12px"></div>
  </form>
</div>
```

**(b) 서버 세션·API 래퍼** — 기존 `<script>` 최상단(`const robots = new Map()` 위)에 삽입:

```javascript
/* ── 관제 v2: 세션·API ─────────────────────────────────────────────── */
let ME = null, CSRF = "";
const ROLE_RANK = {observer: 0, operator: 1, admin: 2};

async function api(path, opts = {}) {
  opts.headers = Object.assign({"Content-Type": "application/json"}, opts.headers);
  if (opts.method && opts.method !== "GET") opts.headers["X-CSRF"] = CSRF;
  if (opts.body && typeof opts.body !== "string") opts.body = JSON.stringify(opts.body);
  const r = await fetch("/api/v1" + path, opts);
  if (r.status === 401) { showLogin(); throw new Error("로그인 필요"); }
  if (!r.ok) throw new Error((await r.text()) || r.status);
  return r.json();
}

function showLogin() { document.getElementById("login-overlay").style.display = "flex"; }
function hideLogin() { document.getElementById("login-overlay").style.display = "none"; }

function applyRoleVisibility() {
  document.querySelectorAll("[data-min-role]").forEach(el => {
    el.style.display =
      ROLE_RANK[ME.role] >= ROLE_RANK[el.dataset.minRole] ? "" : "none";
  });
}

async function boot() {
  try {
    const me = await api("/auth/me");
    ME = me.user; CSRF = me.csrf;
    hideLogin(); applyRoleVisibility();
    await loadRobots(); connectServer();
  } catch (e) { /* 로그인 대기 */ }
}

document.getElementById("login-form").addEventListener("submit", async ev => {
  ev.preventDefault();
  try {
    const r = await fetch("/api/v1/auth/login", {method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({login: document.getElementById("login-id").value,
                            password: document.getElementById("login-pw").value})});
    if (!r.ok) throw new Error("아이디 또는 비밀번호가 틀립니다");
    const d = await r.json(); ME = d.user; CSRF = d.csrf;
    hideLogin(); applyRoleVisibility();
    await loadRobots(); connectServer();
  } catch (e) { document.getElementById("login-err").textContent = e.message; }
});
```

**(c) 로봇 목록·단일 WS** — 기존 `endpoints()` / `addRobot(url, token)` / `connect(r)` / 마지막 줄 `endpoints().forEach(...)` 를 삭제하고 다음으로 교체. `robots` Map 의 키는 url 이 아니라 **robot_id** 가 된다 (나머지 코드에서 `r.url`·`r.token` 참조를 검색해 `r.id` 기준으로 정리한다):

```javascript
/* ── 관제 v2: 서버 단일 WS ─────────────────────────────────────────── */
let SRV = null;                       // 서버 WebSocket 1개
const CH_TO_SUFFIX = {"tel/state": "state", "tel/health": "health",
                      "tel/map": "map", "evt": "event", "mission": "mission",
                      "hello": "hello"};

async function loadRobots() {
  const list = await api("/robots");
  for (const it of list) {
    if (!robots.has(it.id)) {
      const r = {id: it.id, farm_id: it.farm_id, name: it.name,
                 state: null, health: null, geom: null, track: [],
                 mapCells: new Map(), status: "대기", cls: ""};
      robots.set(it.id, r);
    }
  }
  renderTabs();                       // 기존 탭 렌더 함수 재사용
}

function connectServer() {
  if (SRV) try { SRV.close(); } catch (e) {}
  const proto = location.protocol === "https:" ? "wss://" : "ws://";
  SRV = new WebSocket(proto + location.host + "/ws");
  SRV.onmessage = ev => {
    const m = JSON.parse(ev.data);
    if (m.topic) {                    // fleet/v1/{farm}/{robot}/{channel}
      const parts = m.topic.split("/");
      const rid = parts[3], ch = parts.slice(4).join("/");
      const r = robots.get(rid);
      if (r) handleRobotMessage(r, CH_TO_SUFFIX[ch] || ch, m.payload);
    } else if (m.type === "denied") {
      pushEvent("서버", "거부: " + m.action + " — " + m.reason);
    } else if (m.type === "stop_all_result") {
      for (const [rid, res] of Object.entries(m.results))
        if (res !== "sent") pushEvent(rid, "전체 정지 미확인 (" + res + ")");
    }
  };
  SRV.onclose = () => setTimeout(connectServer, 2000);   // 자동 재연결
}

function sendCmd(robotId, action, payload) {
  SRV.send(JSON.stringify({type: "cmd", action, robot: robotId,
                           cmd_id: "c" + Date.now(), payload: payload || {}}));
}
function sendTeleop(robotId, vx, wz) {
  SRV.send(JSON.stringify({type: "teleop", robot: robotId, payload: {vx, wz}}));
}

boot();
```

`handleRobotMessage(r, suffix, payload)` 는 **기존** `connect(r)` 안의 `ws.onmessage` 본문(접미사별 분기: state 갱신·health 갱신·맵 셀 누적·이벤트 로그·hello 처리)을 함수로 추출한 것이다 — 분기 로직은 그대로 옮기고, 봉투 파싱(`P.parse` 상당) 부분만 제거한다 (서버가 이미 topic/payload 로 나눠 준다).

**(d) 명령 발신 치환·역할 게이트** — 기존 코드에서 로봇 WS 로 직접 `send(...)` 하던 곳(비상정지 버튼·임무 시작·텔레옵 키 처리·전체 정지)을 찾아 치환한다:

- 비상정지: `sendCmd(cur().id, "estop")` / 해제: `sendCmd(cur().id, "clear_estop")`
- 전체 정지: `SRV.send(JSON.stringify({type:"cmd", action:"stop_all", cmd_id:"sa"+Date.now()}))`
- 텔레옵: `sendTeleop(cur().id, vx, wz)` (데드맨 로직은 기존 그대로)
- 임무 시작: `api("/missions", {method:"POST", body:{robot_id: cur().id, alleys: 선택된통로배열}})` — 일시정지·재개·취소는 `api("/missions/"+미션ID+"/pause", {method:"POST"})` 형태
- 임무 패널 요소에 `data-min-role="operator"`, 비상정지 해제 버튼에 `data-min-role="admin"` 속성 추가 (estop·전체 정지는 속성 없음 = 전 역할 노출, D9)
- `<title>` 과 헤더 텍스트에 "관제 v2" 를 넣는다 (테스트 마커)
- "+ 로봇" 추가 UI·`localStorage` 엔드포인트 저장은 제거한다 (로봇은 서버 API 가 관리)

- [ ] **Step 4: 통과 확인 후 커밋**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest tests/test_dashboard_serving.py -q
cd /home/myhome/YBNML
git add server/web/index.html server/tests/test_dashboard_serving.py
git commit -m "M1: 대시보드 v2 — 로그인·단일 서버 WS·역할 게이트"
```

---

### Task 13: 대시보드 — 이력 화면 + 사용자 관리

**Files:**
- Modify: `server/web/index.html`
- Modify: `server/tests/test_dashboard_serving.py`

**Interfaces:**
- Consumes: `GET /missions`, `GET /tracks`, `GET /events`, `GET/POST/PATCH /users` (Task 9·5)

- [ ] **Step 1: 테스트 확장**

`test_dashboard_serving.py` 에 추가:

```python
def test_history_and_user_admin_present(client):
    html = client.get("/").text
    assert "이력" in html and "사용자 관리" in html
    assert 'data-min-role="admin"' in html
```

- [ ] **Step 2: 이력 패널 추가**

우측 패널 열(기존 임무 패널 아래)에 삽입:

```html
<section id="panel-history">
  <h3>이력</h3>
  <select id="hist-missions" size="5" style="width:100%"></select>
  <button id="hist-load">불러오기</button> <button id="hist-clear">지도에서 지우기</button>
  <ul id="hist-events" style="max-height:140px;overflow:auto;font-size:12px"></ul>
</section>
```

스크립트 (관제 v2 블록 아래에 추가):

```javascript
/* ── 관제 v2: 이력 ─────────────────────────────────────────────────── */
async function refreshHistory() {
  const list = await api("/missions");
  const sel = document.getElementById("hist-missions");
  sel.innerHTML = "";
  for (const m of list) {
    const o = document.createElement("option");
    o.value = m.id;
    o.textContent = `#${m.id} ${m.robot_id} ${m.state} ` +
                    (m.started_at ? m.started_at.slice(0, 19) : "(미시작)");
    o.dataset.robot = m.robot_id;
    o.dataset.from = m.started_at ? Date.parse(m.started_at) / 1000 : "";
    o.dataset.to = m.ended_at ? Date.parse(m.ended_at) / 1000 : "";
    sel.appendChild(o);
  }
}

document.getElementById("hist-load").addEventListener("click", async () => {
  const o = document.getElementById("hist-missions").selectedOptions[0];
  if (!o) return;
  let q = "/tracks?robot_id=" + encodeURIComponent(o.dataset.robot);
  if (o.dataset.from) q += "&from_ts=" + o.dataset.from;
  if (o.dataset.to) q += "&to_ts=" + o.dataset.to;
  const trs = await api(q);
  const r = robots.get(o.dataset.robot);
  if (r) { r.histTrack = trs.map(t => [t.x, t.y]); draw(); }   // draw()=기존 렌더 루프
  const evs = await api("/events?robot_id=" + encodeURIComponent(o.dataset.robot));
  const ul = document.getElementById("hist-events");
  ul.innerHTML = "";
  for (const e of evs) {
    const li = document.createElement("li");
    li.textContent = `${e.ts.slice(11, 19)} [${e.severity}] ${e.kind} ${e.msg}`;
    ul.appendChild(li);
  }
});
document.getElementById("hist-clear").addEventListener("click", () => {
  for (const r of robots.values()) r.histTrack = null;
  draw();
});
```

기존 캔버스 렌더 함수(`draw()`)의 궤적 그리기 근처에 이력 궤적 렌더를 추가한다:

```javascript
  // 이력 궤적 (보라 점선)
  if (r.histTrack && r.histTrack.length > 1) {
    ctx.save(); ctx.strokeStyle = "#a371f7"; ctx.setLineDash([6, 4]);
    ctx.beginPath();
    r.histTrack.forEach(([x, y], i) => {
      const [px, py] = w2c(x, y);                 // w2c()=기존 좌표 변환
      i ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
    });
    ctx.stroke(); ctx.restore();
  }
```

(`draw`·`w2c` 등 기존 함수명이 다르면 파일 내 실제 이름을 따른다 — 렌더 루프와 월드→캔버스 변환 함수는 기존 대시보드에 반드시 존재한다)

- [ ] **Step 3: 사용자 관리 패널 (admin 전용)**

```html
<section id="panel-users" data-min-role="admin">
  <h3>사용자 관리</h3>
  <div id="users-list" style="font-size:12px"></div>
  <input id="u-login" placeholder="아이디"> <input id="u-pw" type="password" placeholder="비밀번호">
  <select id="u-role"><option>observer</option><option>operator</option><option>admin</option></select>
  <button id="u-create">계정 생성</button>
</section>
```

```javascript
/* ── 관제 v2: 사용자 관리 (admin) ──────────────────────────────────── */
async function refreshUsers() {
  if (ME.role !== "admin") return;
  const list = await api("/users");
  const div = document.getElementById("users-list");
  div.innerHTML = "";
  for (const u of list) {
    const row = document.createElement("div");
    row.textContent = `${u.login} (${u.role})` + (u.disabled ? " [정지]" : "") + " ";
    const btn = document.createElement("button");
    btn.textContent = u.disabled ? "복구" : "정지";
    btn.onclick = async () => {
      await api("/users/" + u.id, {method: "PATCH", body: {disabled: !u.disabled}});
      refreshUsers();
    };
    row.appendChild(btn);
    div.appendChild(row);
  }
}
document.getElementById("u-create").addEventListener("click", async () => {
  await api("/users", {method: "POST", body: {
    login: document.getElementById("u-login").value,
    password: document.getElementById("u-pw").value,
    role: document.getElementById("u-role").value, farm_ids: []}});
  refreshUsers();
});
```

`boot()` 성공 경로에 `refreshHistory(); refreshUsers();` 호출을 추가한다.

- [ ] **Step 4: 통과 확인 후 커밋**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest -q
cd /home/myhome/YBNML
git add server/web/index.html server/tests/test_dashboard_serving.py
git commit -m "M1: 대시보드 — 임무 이력 재생·사용자 관리 화면"
```

---

### Task 14: 배포(compose) + E2E·보안 회귀 + M1 게이트

**Files:**
- Create: `server/Dockerfile`
- Create: `compose.yaml` (저장소 루트)
- Create: `scripts/fake_legacy_robot.py`
- Create: `scripts/33_verify_m1.py`
- Create: `scripts/34_verify_m1_security.py`

**Interfaces:**
- Consumes: 전체. 가짜 로봇은 기존 봉투 형식(`orchard/{robot}/{suffix}`)을 그대로 흉내 낸다 — 실로봇(시뮬) 대체 가능.

- [ ] **Step 1: Dockerfile·compose**

`server/Dockerfile`:

```dockerfile
FROM python:3.12-slim
WORKDIR /app
COPY pyproject.toml ./
COPY fleet_server ./fleet_server
COPY web ./web
RUN pip install --no-cache-dir .
EXPOSE 8000
CMD ["uvicorn", "fleet_server.app:create_app", "--factory", "--host", "0.0.0.0", "--port", "8000"]
```

`compose.yaml` (저장소 루트):

```yaml
services:
  fleet-server:
    build: ./server
    ports: ["8000:8000"]
    environment:
      FLEET_DB_URL: sqlite:////data/fleet.db
      FLEET_ADMIN_LOGIN: ${FLEET_ADMIN_LOGIN:-admin}
      FLEET_ADMIN_PASSWORD: ${FLEET_ADMIN_PASSWORD:?관리자 초기 비밀번호를 지정하세요}
      FLEET_ALLOWED_ORIGINS: ${FLEET_ALLOWED_ORIGINS:-http://localhost:8000}
    volumes:
      - ./server/data:/data
    restart: unless-stopped
```

빌드 확인 (데몬이 있으면): `docker compose config -q` — 문법만 검증. (이 머신에서 docker 빌드가 불가하면 config 검증까지만 하고 넘어간다 — E2E 는 venv 로 돈다)

- [ ] **Step 2: 가짜 레거시 로봇**

`scripts/fake_legacy_robot.py`:

```python
#!/usr/bin/env python3
"""레거시 로봇 흉내 — 기존 봉투 형식으로 텔레메트리를 쏘고 명령에 반응한다.

E2E(33)·보안(34) 검증이 사용한다. 실로봇(시뮬) 대신 결정적으로 돈다.
"""
from __future__ import annotations

import asyncio
import json
import time


class FakeRobot:
    def __init__(self, robot_id="scout01", port=18080, token="RTOK"):
        self.robot_id, self.port, self.token = robot_id, port, token
        self.seq = 0
        self.estop = False
        self.mission = None            # None | "running" | "paused" | "done"
        self.received: list[dict] = []  # 수신한 cmd/teleop 봉투 전부
        self.teleop_count = 0
        self._x = 0.0

    def env(self, suffix, payload):
        self.seq += 1
        return json.dumps({"v": 1, "topic": f"orchard/{self.robot_id}/{suffix}",
                           "ts_ns": time.time_ns(), "seq": self.seq,
                           "payload": payload})

    async def handler(self, ws):
        if self.token and f"token={self.token}" not in ws.request.path:
            await ws.close(code=4401)
            return

        async def pump():
            while True:
                self._x += 0.1
                await ws.send(self.env("state", {
                    "x": round(self._x, 2), "y": -28.0, "yaw": 0.0,
                    "mode": "mission" if self.mission == "running" else "idle",
                    "estop": self.estop, "ts": time.time()}))
                if self.mission == "running" and self._x >= 1.0 and self.mission != "done":
                    self.mission = "done"
                    await ws.send(self.env("mission", {"state": "done"}))
                await asyncio.sleep(0.2)

        pump_task = asyncio.create_task(pump())
        try:
            async for raw in ws:
                msg = json.loads(raw)
                self.received.append(msg)
                topic = msg.get("topic", "")
                pl = msg.get("payload", {})
                if topic.endswith("/teleop"):
                    self.teleop_count += 1
                    continue
                cmd = pl.get("cmd", "")
                if cmd == "estop":
                    self.estop = True
                    if self.mission == "running":
                        self.mission = "paused"
                        await ws.send(self.env("mission", {"state": "paused"}))
                    await ws.send(self.env("event", {"kind": "estop", "severity": "warn",
                                                     "msg": "비상정지", "ts": time.time()}))
                elif cmd == "clear_estop":
                    self.estop = False
                    await ws.send(self.env("event", {"kind": "estop_cleared",
                                                     "severity": "info", "msg": "해제",
                                                     "ts": time.time()}))
                elif cmd == "mission_start":
                    self.mission = "running"
                    self._x = 0.0
                    await ws.send(self.env("mission", {"state": "running"}))
                elif cmd == "mission_resume":
                    self.mission = "running"
                    await ws.send(self.env("mission", {"state": "running"}))
        finally:
            pump_task.cancel()

    async def serve(self):
        import websockets
        return await websockets.serve(self.handler, "127.0.0.1", self.port)


if __name__ == "__main__":
    async def main():
        fr = FakeRobot()
        await fr.serve()
        print(f"가짜 로봇 ws://127.0.0.1:{fr.port}/ws?token={fr.token}")
        await asyncio.Future()
    asyncio.run(main())
```

- [ ] **Step 3: E2E 검증 스크립트**

`scripts/33_verify_m1.py` — 서버(uvicorn 서브프로세스, 임시 DB) + 가짜 로봇을 띄우고 다음을 검사한다. 실행: `server/.venv/bin/python scripts/33_verify_m1.py`

검사 항목 (각 ✔/✗ 출력, 전부 통과 시 exit 0):

1. admin 로그인·쿠키 플래그 (HttpOnly·SameSite=Strict)
2. 농장 2·로봇 1(가짜 로봇 주소)·operator·observer 생성
3. 로봇 온라인 전환 (`GET /robots/scout01/status` ≤ 15 s 내 online)
4. observer WS 접속 → `tel/state` 수신 (x 증가 확인)
5. observer 가 타 농장 텔레메트리를 받지 않음
6. operator REST 임무 시작 → 가짜 로봇이 `mission_start` 수신 → 임무가 RUNNING → (로봇 완주 보고 후) DONE
7. observer WS estop → 가짜 로봇 estop 수신 + 이벤트 이력 적재 (D9)
8. estop 중 임무는 PAUSED (새 임무를 estop 으로 멈추는 시나리오), admin clear_estop 후에도 PAUSED 유지 (자동 재개 없음) → mission_resume 으로만 재개
9. `GET /tracks` 에 1 Hz 궤적 적재, `GET /events` 에 estop 이벤트, `GET /audit` 에 명령 기록
10. stop_all → 결과 dict 에 scout01=sent

핵심 골격 (전체는 이 구조를 따라 작성한다 — httpx 동기 클라이언트 + websockets 비동기 검사부):

```python
#!/usr/bin/env python3
"""M1 E2E — 서버+가짜 로봇으로 스펙 §0.2 관제 완성 기준을 검사한다."""
import asyncio, json, os, subprocess, sys, tempfile, time
import httpx

sys.path.insert(0, "scripts")
from fake_legacy_robot import FakeRobot

PORT, RPORT = 18800, 18080
BASE = f"http://127.0.0.1:{PORT}"
OK, NG, res = "\033[92m✔\033[0m", "\033[91m✗\033[0m", []

def check(name, cond, detail=""):
    res.append(bool(cond))
    print(f" {OK if cond else NG} {name}" + (f" — {detail}" if detail else ""))

def wait_http(url, sec=20):
    for _ in range(sec * 10):
        try:
            httpx.get(url, timeout=1); return True
        except Exception:
            time.sleep(0.1)
    return False

async def main():
    tmp = tempfile.mkdtemp()
    env = dict(os.environ, FLEET_DB_URL=f"sqlite:///{tmp}/e2e.db",
               FLEET_ADMIN_LOGIN="admin", FLEET_ADMIN_PASSWORD="admpw",
               FLEET_ALLOWED_ORIGINS=BASE, FLEET_LOGIN_DELAY_S="0")
    srv = subprocess.Popen([sys.executable, "-m", "uvicorn",
                            "fleet_server.app:create_app", "--factory",
                            "--port", str(PORT)], env=env, cwd="server",
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    robot = FakeRobot(port=RPORT)
    server_ws = await robot.serve()
    try:
        check("서버 기동", wait_http(BASE + "/"))
        c = httpx.Client(base_url=BASE)
        r = c.post("/api/v1/auth/login", json={"login": "admin", "password": "admpw"})
        sc = r.headers.get("set-cookie", "").lower()
        check("1 로그인+쿠키 플래그", r.status_code == 200
              and "httponly" in sc and "samesite=strict" in sc)
        csrf = r.json()["csrf"]; h = {"X-CSRF": csrf}
        # ... (검사 2~10: 위 목록 순서대로 — REST 는 c, WS 는 websockets.connect 에
        #      Cookie/Origin 헤더를 넣어 검사한다. 각 항목마다 check() 1회 이상)
    finally:
        srv.terminate(); server_ws.close()
    print(f"\n{sum(res)}/{len(res)} 통과")
    sys.exit(0 if all(res) else 1)

asyncio.run(main())
```

(WS 접속 헤더 예: `websockets.connect(f"ws://127.0.0.1:{PORT}/ws", additional_headers={"Cookie": f"fleet_session={토큰}", "Origin": BASE})` — 세션 토큰은 httpx 쿠키 항아리 `c.cookies["fleet_session"]` 에서 꺼낸다)

- [ ] **Step 4: 보안 회귀 스크립트**

`scripts/34_verify_m1_security.py` — 같은 기동 골격으로 다음 9항목:

1. CSRF 없는 POST /farms → 403
2. Origin 불일치 WS → 접속 거부 (4403)
3. 세션 없는 WS → 거부 (4401)
4. observer 텔레옵 → denied + 가짜 로봇 teleop_count == 0
5. observer estop → 허용 (D9 회귀 — 가짜 로봇이 수신)
6. operator 의 clear_estop → denied (admin 전용)
7. 타 농장 operator 임무 → 403 (교차 농장 인가)
8. 정지(disabled) 계정 로그인 → 401
9. 감사 무결성: 위 거부 전부 audit 에 존재 + 응답·DB 덤프에 비밀번호/토큰 원문 없음

골격은 33과 동일 (`check()` 패턴, 전부 통과 시 exit 0).

- [ ] **Step 5: M1 게이트 실행**

```bash
cd /home/myhome/YBNML/server && .venv/bin/python -m pytest -q          # 단위 전체
cd /home/myhome/YBNML
server/.venv/bin/python scripts/33_verify_m1.py                        # E2E
server/.venv/bin/python scripts/34_verify_m1_security.py               # 보안
```

Expected: 3개 전부 통과. **하나라도 실패하면 M2 로 넘어가지 않는다** (스펙 §9 게이트).

- [ ] **Step 6: 커밋**

```bash
cd /home/myhome/YBNML
git add server/Dockerfile compose.yaml scripts/fake_legacy_robot.py scripts/33_verify_m1.py scripts/34_verify_m1_security.py
git commit -m "M1: compose 배포 + E2E·보안 회귀 — M1 게이트"
```

---

## 계획 자체 검토 결과

- **스펙 커버리지 (M1 범위)**: 계정·로그인(T3·4), 다중 농장+스코프(T5), 이력 DB(T2·9), 감사(T6·11), 임무 상태기계·estop→PAUSED(T7·9), D9 매트릭스(T3·11), stop_all 부분 실패 표시(T11·12), CSRF/Origin(T4·11), 레거시 WS 어댑터 한시(T10), 오프라인 즉시 실패(T8·9), 15초 오프라인 판정(T8), (robot,channel,seq) 중복 제거(T2·9), 1 Hz 궤적(T9), 대시보드 로그인·이력·사용자 관리(T12·13), compose(T14), 게이트(T14). — M1 범위 밖(스펙에 있으나 이 계획에 없음): Zenoh·mTLS·CA·store-and-forward·업로드(→M2), 맵 번들·로컬리제이션·주행 안전(→M3), 로봇측 D9 매트릭스 동기화(로봇 코드 수정 금지 제약 때문에 M2 의 에이전트 개편에서 반영 — 그때까지 estop 하향은 서버 계층에서만 유효).
- **자리표시자 스캔**: 33·34의 검사 2~10/1~9 본문은 골격+항목 명세로 제공 (전 항목이 기존 17·21·30 스크립트에 동형 코드가 있어 실행자가 참조 가능 — `scripts/17_verify_control.py`, `scripts/21_verify_security.py`, `scripts/30_verify_audit_roles.py`).
- **타입 일관성**: `send_command` 는 전 구현에서 `async` / 반환 `"sent"|"offline"`. TelemetryHandler 는 4-인자 `(robot_id, channel, payload, seq)`, FleetService.subscribe 콜백은 3-인자 `(robot_id, channel, payload)` — 구분 유지 확인.

## 실행 안내

Plan complete. 실행 방식 두 가지:

**1. Subagent-Driven (권장)** — 태스크마다 새 서브에이전트 파견, 사이사이 검토, 빠른 반복

**2. Inline Execution** — 이 세션에서 executing-plans 로 체크포인트 배치 실행
