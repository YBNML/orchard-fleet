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
