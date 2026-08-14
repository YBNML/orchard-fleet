"""T6 리뷰 I4 — 구 스키마(events.epoch 없음) 자동 복구.

이 저장소에는 마이그레이션 기구가 없고(`Base.metadata.create_all` 하나뿐 —
T4/T5 이연 항목) 배포본마다 `fleet.db` 파일을 직접 다룬다. T6 확장 A 가
`events` 에 `epoch` 컬럼을 더하면서 UNIQUE 제약을 바꿨는데(SQLite 는 ALTER
로 제약을 못 바꾼다), 이 컬럼이 없는 **다른 환경의 기존** `fleet.db` 에 새
코드를 얹으면 `epoch NOT NULL` 위반으로 모든 evt INSERT 가 터진다.
`legacy_ws.LegacyRobotLink.run()` 의 `on_message` 호출부는 다운스트림
예외를 로그만 남기고 삼키므로(링크 자체는 살려 둔다는 설계 — 그 자체는
옳다), 증상은 크래시가 아니라 **"서버는 떠 있는데 이벤트가 하나도 안
쌓인다"는 조용한 정지**다.

기동 시 한 번 스키마를 살펴 필요하면 라이브 `fleet.db` 에서 실제로 수행한
절차(T6 리포트 §8.1 — rename+recreate+copy)를 그대로 코드로 옮겨 자동
실행한다. 데이터가 이미 손상(같은 (robot_id,channel,seq) 중복 — T6 §8.2 의
사례)돼 있어도 최초 행만 남기고 계속 진행한다(서버가 뜨지 않는 것보다는
낫다 — 손상 자체는 §8.2 처럼 운영자가 별도로 다뤄야 한다).
"""
from __future__ import annotations

import logging

import sqlalchemy as sa
from sqlalchemy.exc import IntegrityError

from .db import Base
from .models import Event

log = logging.getLogger("fleet_server.migrations")

_OLD_TABLE = "events_pre_epoch_migration"


def ensure_events_epoch_column(engine) -> None:
    """`events` 테이블에 `epoch` 컬럼이 없으면 재구축한다.

    신규 DB(테이블 자체가 없음)는 손대지 않는다 — 뒤따르는
    `Base.metadata.create_all` 이 처음부터 새 스키마로 만든다."""
    inspector = sa.inspect(engine)
    if "events" not in inspector.get_table_names():
        return
    cols = {c["name"] for c in inspector.get_columns("events")}
    if "epoch" in cols:
        return
    log.warning("events 테이블에 epoch 컬럼이 없다(구 스키마) — 자동 재구축 시작")
    inserted, dropped = _rebuild_events_table(engine)
    log.warning("events 재구축 완료 — %d행 이관, %d행 드롭"
               "((robot_id,channel,seq) 중복 — 최초 행만 보존, 0이면 손상 없음)",
               inserted, dropped)


def _rebuild_events_table(engine) -> tuple[int, int]:
    """rename → 새 스키마로 재생성 → 데이터 복사(epoch=0) → 옛 테이블 정리.

    복사는 먼저 그대로 시도하고, 옛 DB 에 이미 (robot_id,channel,seq) 중복이
    있어 UNIQUE 위반이 나면(§8.2 와 같은 손상 이력이 있는 배포본) 최초 행만
    남기는 방식으로 재시도한다. 반환은 (이관 행수, 드롭된 중복 행수)."""
    with engine.begin() as conn:
        conn.exec_driver_sql(f"ALTER TABLE events RENAME TO {_OLD_TABLE}")
    Base.metadata.create_all(engine, tables=[Event.__table__])

    copy_sql = ("INSERT{ignore} INTO events "
               "(id, robot_id, ts, channel, seq, epoch, kind, severity, msg, payload_json) "
               "SELECT id, robot_id, ts, channel, seq, 0, kind, severity, msg, payload_json "
               f"FROM {_OLD_TABLE}")
    try:
        with engine.begin() as conn:
            conn.exec_driver_sql(copy_sql.format(ignore=""))
    except IntegrityError:
        log.warning("events 재구축 — 옛 데이터에 (robot_id,channel,seq) 중복이 있다. "
                   "최초 행만 남기고 재시도한다")
        with engine.begin() as conn:
            conn.exec_driver_sql(copy_sql.format(ignore=" OR IGNORE"))

    with engine.begin() as conn:
        new_n = conn.exec_driver_sql("SELECT COUNT(*) FROM events").scalar()
        old_n = conn.exec_driver_sql(f"SELECT COUNT(*) FROM {_OLD_TABLE}").scalar()
        conn.exec_driver_sql(f"DROP TABLE {_OLD_TABLE}")
    return new_n, old_n - new_n
