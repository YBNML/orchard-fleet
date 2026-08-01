from __future__ import annotations

from sqlalchemy import create_engine, event
from sqlalchemy.orm import DeclarativeBase, Session, sessionmaker
from sqlalchemy.pool import StaticPool


class Base(DeclarativeBase):
    pass


def make_engine(db_url: str):
    if db_url == "sqlite://":                      # 인메모리: 단일 연결 공유
        engine = create_engine(db_url, poolclass=StaticPool,
                               connect_args={"check_same_thread": False})
    else:
        connect_args = {"check_same_thread": False} if db_url.startswith("sqlite") else {}
        engine = create_engine(db_url, connect_args=connect_args)

    if db_url.startswith("sqlite"):
        # WAL: REST 쓰기와 ingest 커밋이 이벤트루프 위에서 경합해도 리더가 막히지
        # 않는다. busy_timeout: 그래도 겹치면 즉시 실패 대신 최대 3초 대기.
        # foreign_keys: SQLite 는 기본 OFF — 댕글링 참조가 조용히 통과하는 것을 막는다.
        @event.listens_for(engine, "connect")
        def _sqlite_pragmas(dbapi_conn, _):
            cur = dbapi_conn.cursor()
            cur.execute("PRAGMA journal_mode=WAL")       # 인메모리에서는 무시됨
            cur.execute("PRAGMA busy_timeout=3000")
            cur.execute("PRAGMA foreign_keys=ON")
            cur.close()

    return engine


def make_session_factory(engine) -> sessionmaker[Session]:
    return sessionmaker(bind=engine, expire_on_commit=False)
