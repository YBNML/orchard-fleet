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


class AlleyLock(Base):
    """AlleyLock — 임무 단위 통로 점유(스펙 ③ §2). 임무 하나당 행 하나.

    pads_json 은 alleys_json 에서 파생되는 값이지만(traffic.pads) 조회·복원
    시 재계산을 피하려고 함께 저장한다 — JSON 은 튜플을 못 담으므로 [a,b]
    리스트로 직렬화한다. alleys_json/pads_json 이 null 이면 "와일드카드"(통로
    생략 임무 — 로봇이 전 통로를 돈다)로, 그 farm 의 모든 잠금과 충돌한다.

    farm_id 는 리뷰 라운드 1(I5)에서 추가 — 처음 설계(alley_locks 테이블
    (robot_id, alleys_json, pads_json, mission_id, ts))에는 없었는데, 그 결과
    서로 다른 농장(별개 과수원)의 통로 번호가 같으면 충돌 검사에 걸려 무관한
    농장끼리 서로 막아섰다. 충돌 검사는 이제 같은 farm 안에서만 한다.
    """
    __tablename__ = "alley_locks"
    id: Mapped[int] = mapped_column(primary_key=True)
    mission_id: Mapped[int] = mapped_column(ForeignKey("missions.id"), unique=True)
    robot_id: Mapped[str] = mapped_column(ForeignKey("robots.id"))
    farm_id: Mapped[int] = mapped_column(ForeignKey("farms.id"))
    alleys_json: Mapped[list | None] = mapped_column(JSON, default=None)
    pads_json: Mapped[list | None] = mapped_column(JSON, default=None)
    ts: Mapped[dt.datetime] = mapped_column(DateTime(timezone=True), default=_now)


class BTInstance(Base):
    """Behavior Tree 인스턴스 — 프리셋 하나가 만든 '로봇 1대분 임무 큐'.

    tree_json 이 곧 실행 상태다(nodes.to_state/from_state 왕복): Sequence 의
    진행 위치, Retry 의 남은 시도, Action 이 붙든 mission_id 까지 들어 있다.
    엔진은 매 틱 이 JSON 을 되살려 tick 하고 다시 적는다 — 그래서 서버가
    재기동해도 RUNNING 인스턴스는 하던 자리에서 이어 달린다(별도 복원 로직이
    아니라 영속 상태 자체가 이어달리기의 근거다).

    robot_id·farm_id 는 프리셋이 고른 '주 로봇'이다(인스턴스 1개 = 로봇 1대).
    조회 스코프(farm)와 인가는 그 값으로 건다.
    """
    __tablename__ = "bt_instances"
    id: Mapped[int] = mapped_column(primary_key=True)
    preset: Mapped[str] = mapped_column(String(40))
    params_json: Mapped[dict] = mapped_column(JSON, default=dict)
    robot_id: Mapped[str] = mapped_column(ForeignKey("robots.id"))
    farm_id: Mapped[int] = mapped_column(ForeignKey("farms.id"))
    state: Mapped[str] = mapped_column(String(16), default="RUNNING")  # RUNNING|SUCCESS|FAILED|CANCELED
    tree_json: Mapped[dict] = mapped_column(JSON, default=dict)
    note: Mapped[str] = mapped_column(String(160), default="")         # 실패 사유 등
    created_by: Mapped[int] = mapped_column(ForeignKey("users.id"))
    created_at: Mapped[dt.datetime] = mapped_column(DateTime(timezone=True), default=_now)
    updated_at: Mapped[dt.datetime] = mapped_column(DateTime(timezone=True), default=_now)
    __table_args__ = (Index("ix_bt_state", "state"),)


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


class Intervention(Base):
    """개입 큐 — 로봇이 스스로 못 풀어 사람을 부른 사건.

    '여러 대를 나란히 감시하는 화면'은 상시 감시라 1인당 2~5대가 상한이지만,
    로봇이 막혔을 때만 부르고 사람이 큐에서 꺼내 처리하는 구조는 그 상한을
    넘는다. 큐가 곧 관제의 작업 단위다.
    """
    __tablename__ = "interventions"
    id: Mapped[int] = mapped_column(primary_key=True)
    robot_id: Mapped[str] = mapped_column(ForeignKey("robots.id"))
    farm_id: Mapped[int] = mapped_column(ForeignKey("farms.id"))
    code: Mapped[str] = mapped_column(String(40))
    category: Mapped[str] = mapped_column(String(16), default="")
    severity: Mapped[str] = mapped_column(String(16), default="warn")
    needs_site_visit: Mapped[bool] = mapped_column(default=False)
    state: Mapped[str] = mapped_column(String(16), default="OPEN")  # OPEN|ACKED|RESOLVED|ESCALATED
    msg: Mapped[str] = mapped_column(String(256), default="")
    context_json: Mapped[dict] = mapped_column(JSON, default=dict)
    opened_at: Mapped[dt.datetime] = mapped_column(DateTime(timezone=True), default=_now)
    acked_at: Mapped[dt.datetime | None] = mapped_column(DateTime(timezone=True), default=None)
    resolved_at: Mapped[dt.datetime | None] = mapped_column(DateTime(timezone=True), default=None)
    acked_by: Mapped[int | None] = mapped_column(ForeignKey("users.id"), default=None)
    resolved_by: Mapped[int | None] = mapped_column(ForeignKey("users.id"), default=None)
    note: Mapped[str] = mapped_column(String(256), default="")
    __table_args__ = (Index("ix_interv_robot_state", "robot_id", "state"),)


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
