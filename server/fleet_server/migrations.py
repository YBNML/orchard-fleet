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
실행한다.

**리뷰 라운드 2 (N1) — 손상 DB 는 절대 행을 버리지 않는다.** 옛 데이터에
이미 (robot_id,channel,seq) 중복이 있는 경우(§8.2 의 사례), 최초 구현은
`INSERT OR IGNORE`(최초 행만 보존)로 나머지를 조용히·불가역으로 버렸다 —
이것이 정확히 T6 §8.2/I5 사고(먼저 온 pong 이 3시간 뒤 도착한 mission 23
완료 보고를 밀어냄)를 사람 검토 없이 자동으로 반복하는 경로였다. `epoch`
컬럼은 정확히 이런 충돌을 흡수하려고 만든 것이므로, 중복 그룹의 2번째
이후 행에는 `epoch=1,2,…` 를 순서대로 배정해 **한 행도 잃지 않는다.**
재배치가 실제로 일어난 경우 옛 테이블(`events_pre_epoch_migration`)은
지우지 않고 남긴다(방어선 — 자동 마이그레이션이 사람 검토 없이 데이터를
재구성했다는 뜻이므로, 원본을 지우지 않아야 나중에 대조할 수 있다).
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

    # 리뷰 라운드 2 (N2) — 이전 마이그레이션 시도의 잔재가 이미 있는 상태에서
    # 또 RENAME 을 시도하면 SQLite 가 "table already exists" 로 던지고, 그
    # raw 오류만으로는 운영자가 무슨 일이 있었는지 알 수 없어 서버가 그냥
    # 안 뜨는 것처럼 보인다. 원인·조치를 미리 알아볼 수 있게 여기서 먼저
    # 걸러 명확한 오류와 복구 지침을 남긴다(자동으로 덮어쓰거나 지우지
    # 않는다 — 잔재에 데이터가 남아 있을 수 있다).
    if _OLD_TABLE in inspector.get_table_names():
        msg = (
            f"events 자동 마이그레이션을 진행할 수 없다 — '{_OLD_TABLE}' 테이블이 "
            f"이미 있다(이전 마이그레이션 시도의 잔재로 보인다). 데이터가 들어 "
            f"있을 수 있어 자동으로 덮어쓰거나 지우지 않는다.\n"
            f"복구 지침:\n"
            f"  1) '{_OLD_TABLE}' 의 내용을 살펴 데이터가 온전한지 확인한다.\n"
            f"  2) 이관이 이미 끝난 상태라면(=이 오류가 나기 전에 events 가 "
            f"정상 동작했다면) 'DROP TABLE {_OLD_TABLE}' 로 잔재만 지운 뒤 "
            f"서버를 다시 기동한다.\n"
            f"  3) 이관이 덜 끝난 상태로 보이면(events 테이블이 비정상이거나 "
            f"없다면) 'ALTER TABLE {_OLD_TABLE} RENAME TO events' 로 되돌린 "
            f"뒤 서버를 다시 기동해 마이그레이션을 재시도한다.")
        log.error(msg)
        raise RuntimeError(msg)

    log.warning("events 테이블에 epoch 컬럼이 없다(구 스키마) — 자동 재구축 시작")
    result = _rebuild_events_table(engine)
    log.warning("events 재구축 완료 — %d행 이관, %d행 epoch 재배치로 흡수"
               "(중복 — 행 손실 없음, 0이면 손상 없음)%s",
               result["copied"], result["reassigned"],
               f" — 원본 보존: {_OLD_TABLE}" if result["old_table_kept"] else "")


def _rebuild_events_table(engine) -> dict:
    """rename → 새 스키마로 재생성 → 데이터 복사(epoch=0) → 옛 테이블 정리.

    복사는 먼저 그대로 시도한다(무손상 DB 의 일반 경로 — 빠르다). 옛 DB 에
    이미 (robot_id,channel,seq) 중복이 있어 UNIQUE 위반이 나면
    `_copy_with_epoch_reassignment` 로 전환해 **한 행도 버리지 않고** 옮긴다.

    반환: {"copied": 이관 행수, "reassigned": epoch 재배치된 행수,
           "old_table_kept": bool}."""
    with engine.begin() as conn:
        conn.exec_driver_sql(f"ALTER TABLE events RENAME TO {_OLD_TABLE}")
    Base.metadata.create_all(engine, tables=[Event.__table__])

    plain_sql = ("INSERT INTO events "
                "(id, robot_id, ts, channel, seq, epoch, kind, severity, msg, payload_json) "
                "SELECT id, robot_id, ts, channel, seq, 0, kind, severity, msg, payload_json "
                f"FROM {_OLD_TABLE}")
    try:
        with engine.begin() as conn:
            conn.exec_driver_sql(plain_sql)
        reassigned = 0
    except IntegrityError:
        log.warning("events 재구축 — 옛 데이터에 (robot_id,channel,seq) 중복이 "
                   "있다. 행 손실 없이 epoch 을 재배치해 흡수한다")
        reassigned = _copy_with_epoch_reassignment(engine)

    with engine.begin() as conn:
        new_n = conn.exec_driver_sql("SELECT COUNT(*) FROM events").scalar()
        old_n = conn.exec_driver_sql(f"SELECT COUNT(*) FROM {_OLD_TABLE}").scalar()
    assert new_n == old_n, (
        f"events 재구축 후 행수 불일치: 새 {new_n} vs 옛 {old_n} — 절대 일어나면 "
        f"안 된다(N1: 행 손실 없음이 이 마이그레이션의 불변 조건)")

    old_table_kept = reassigned > 0
    if old_table_kept:
        # 방어선(N1) — 재배치가 실제로 있었다면 옛 테이블을 지우지 않는다.
        # 사람 검토 없이 데이터를 재구성했다는 뜻이므로, 원본이 남아 있어야
        # 나중에 무엇이 재배치됐는지 대조할 수 있다.
        pass
    else:
        with engine.begin() as conn:
            conn.exec_driver_sql(f"DROP TABLE {_OLD_TABLE}")
    return {"copied": new_n, "reassigned": reassigned, "old_table_kept": old_table_kept}


def _copy_with_epoch_reassignment(engine) -> int:
    """(robot_id,channel,seq) 가 중복인 옛 데이터를 **행 손실 없이** 옮긴다.

    옛 테이블을 id(삽입 순서) 순으로 훑으면서, 같은 (robot_id,channel,seq)
    키를 이미 쓴 적이 있으면 그 키의 다음 미사용 epoch(1, 2, …)를 배정한다
    — epoch 컬럼이 정확히 이 충돌을 흡수하려고 만들어졌다. `INSERT OR
    IGNORE`(최초 행만 보존, T6 §8.2/I5 사고를 반복하는 경로)와 달리 모든
    행이 새 `events` 에 그대로 남는다.

    읽기·쓰기 모두 `exec_driver_sql`(드라이버 수준, ORM 타입 변환을 거치지
    않음)로 한다 — `payload_json` 은 SQLite 안에서 이미 직렬화된 TEXT 다.
    ORM 의 `Event.__table__.insert()` 를 쓰면 JSON 타입의 바인드 처리기가
    이 문자열을 **다시 한 번** `json.dumps` 해 이중 인코딩된다(읽을 때도
    ORM 을 거치면 이미 파싱된 dict 가 되므로 왕복이 어긋난다) — 그래서
    양끝을 raw 문자열로 고정해 왕복 일치를 보장한다.

    옛 events 테이블이 tracks 급(수십만 행)으로 크지 않다는 전제로 파이썬
    메모리에서 처리한다 — 이 경로는 손상 이력이 있는 드문 배포본에서만
    타는 예외 경로다."""
    with engine.begin() as conn:
        rows = conn.exec_driver_sql(
            f"SELECT id, robot_id, ts, channel, seq, kind, severity, msg, payload_json "
            f"FROM {_OLD_TABLE} ORDER BY id").fetchall()

    next_epoch: dict[tuple, int] = {}      # (robot_id, channel, seq) -> 다음 배정할 epoch
    params = []
    reassigned = 0
    for r in rows:
        key = (r.robot_id, r.channel, r.seq)
        epoch = next_epoch.get(key, 0)
        next_epoch[key] = epoch + 1
        if epoch > 0:
            reassigned += 1
        params.append((r.id, r.robot_id, r.ts, r.channel, r.seq, epoch,
                       r.kind, r.severity, r.msg, r.payload_json))

    insert_sql = ("INSERT INTO events "
                 "(id, robot_id, ts, channel, seq, epoch, kind, severity, msg, payload_json) "
                 "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)")
    with engine.begin() as conn:
        conn.exec_driver_sql(insert_sql, params)
    return reassigned
