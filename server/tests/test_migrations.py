"""T6 리뷰 I4 — 구 스키마(events.epoch 없음) fleet.db 를 열면 자동 복구된다.

이 저장소는 마이그레이션 기구가 없어(create_all 하나뿐) 다른 배포본의 기존
DB 파일이 새 코드를 만나면 깨질 수 있다는 것이 반복되는 이연 항목이었다
(T4 §8.4, T5 §6). T6 확장 A 의 events.epoch 컬럼 추가가 실제로 그 사고를
낼 뻔했다 — 이 테스트는 옛 스키마 DB 파일을 직접 만들어(fixture) 서버가
그 위에서 정상 기동·수집하는지 확인한다."""
import datetime as dt
import sqlite3

import pytest

from fleet_server import ingest
from fleet_server import models as m
from fleet_server.app import create_app
from fleet_server.config import Settings
from fleet_server.fleet.port import InMemoryFleetPort


def _make_old_schema_db(path: str) -> None:
    """T6 이전 실물 events 스키마(epoch 컬럼 없음, UNIQUE(robot_id,channel,seq))
    를 그대로 재현한다 — 실제 CREATE TABLE 문(server/fleet.db 재구축 전
    스키마, T6 리포트 §8.1)을 옮겨 적었다."""
    con = sqlite3.connect(path)
    cur = con.cursor()
    cur.execute("""CREATE TABLE farms (
        id INTEGER PRIMARY KEY, name VARCHAR(64) UNIQUE,
        map_bundle_ref VARCHAR(256), config_json JSON)""")
    cur.execute("INSERT INTO farms (id, name, config_json) VALUES (1, '농장A', '{}')")
    cur.execute("""CREATE TABLE robots (
        id VARCHAR(64) PRIMARY KEY, farm_id INTEGER, name VARCHAR(64),
        kind VARCHAR(32), conn_kind VARCHAR(16), last_seen DATETIME,
        config_json JSON)""")
    cur.execute("INSERT INTO robots (id, farm_id, name, kind, conn_kind, config_json) "
               "VALUES ('scout01', 1, 'r', 'orchard', 'legacy_ws', '{}')")
    cur.execute("""
        CREATE TABLE events (
            id INTEGER NOT NULL,
            robot_id VARCHAR(64) NOT NULL,
            ts DATETIME NOT NULL,
            channel VARCHAR(32),
            seq INTEGER,
            kind VARCHAR(32) NOT NULL,
            severity VARCHAR(16) NOT NULL,
            msg VARCHAR(256) NOT NULL,
            payload_json JSON NOT NULL,
            PRIMARY KEY (id),
            CONSTRAINT uq_event_seq UNIQUE (robot_id, channel, seq),
            FOREIGN KEY(robot_id) REFERENCES robots (id)
        )""")
    cur.execute("""INSERT INTO events
        (id, robot_id, ts, channel, seq, kind, severity, msg, payload_json)
        VALUES (1, 'scout01', '2026-08-01 00:00:00', 'evt', 5,
                'estop', 'warn', '정지', '{}')""")
    con.commit()
    con.close()


def _settings_for(db_path) -> Settings:
    return Settings(db_url=f"sqlite:///{db_path}", session_secret="테스트비밀",
                    login_delay_s=0.0, admin_login="admin", admin_password="admpw",
                    allowed_origins=["http://testserver"])


def test_old_schema_events_table_migrates_on_startup(tmp_path):
    db_path = tmp_path / "old_fleet.db"
    _make_old_schema_db(str(db_path))

    app = create_app(_settings_for(db_path), fleet=InMemoryFleetPort())

    with app.state.session_factory() as db:
        row = db.query(m.Event).filter_by(id=1).one()
        assert row.epoch == 0                    # 기존 행이 보존되고 epoch=0 을 얻는다
        assert row.kind == "estop" and row.seq == 5


def test_ingestion_works_after_old_schema_migration(tmp_path):
    """단순 스키마 확인을 넘어 — 마이그레이션 뒤 실제 evt 수집(epoch 인자 포함)이
    죽지 않고 동작해야 한다(리뷰가 우려한 "조용한 정지"의 반증)."""
    db_path = tmp_path / "old_fleet2.db"
    _make_old_schema_db(str(db_path))

    app = create_app(_settings_for(db_path), fleet=InMemoryFleetPort())

    with app.state.session_factory() as db:
        ok = ingest.event(db, "scout01", "evt", 6,
                          {"kind": "assistance", "msg": "테스트"}, epoch=0)
    assert ok is True
    with app.state.session_factory() as db:
        assert db.query(m.Event).count() == 2


def test_fresh_db_without_events_table_is_untouched(tmp_path):
    """신규 DB(테이블 자체가 없음)는 이 마이그레이션이 손대지 않는다 —
    뒤따르는 create_all 이 처음부터 새 스키마로 만든다."""
    db_path = tmp_path / "brand_new.db"
    app = create_app(_settings_for(db_path), fleet=InMemoryFleetPort())
    with app.state.session_factory() as db:
        assert db.query(m.Event).count() == 0


# ── 리뷰 라운드 2 ────────────────────────────────────────────────────────

def _make_old_schema_db_with_duplicate(path: str) -> None:
    """T6 §8.2/I5 사고와 같은 모양의 손상을 옛 스키마 DB 에 재현한다 — 같은
    (robot_id,channel,seq) 로 먼저 온 pong(id 작음, ts 이름)과 나중에 온
    의미 있는 cmd_result(id 큼, ts 늦음)가 공존한다. 옛 UNIQUE 제약이
    실제로는(인덱스 손상으로) 이런 중복을 막지 못했다는 것이 §8.2 의 발견."""
    con = sqlite3.connect(path)
    cur = con.cursor()
    cur.execute("""CREATE TABLE farms (
        id INTEGER PRIMARY KEY, name VARCHAR(64) UNIQUE,
        map_bundle_ref VARCHAR(256), config_json JSON)""")
    cur.execute("INSERT INTO farms (id, name, config_json) VALUES (1, '농장A', '{}')")
    cur.execute("""CREATE TABLE robots (
        id VARCHAR(64) PRIMARY KEY, farm_id INTEGER, name VARCHAR(64),
        kind VARCHAR(32), conn_kind VARCHAR(16), last_seen DATETIME,
        config_json JSON)""")
    cur.execute("INSERT INTO robots (id, farm_id, name, kind, conn_kind, config_json) "
               "VALUES ('scout01', 1, 'r', 'orchard', 'legacy_ws', '{}')")
    # 옛 UNIQUE(robot_id,channel,seq) 는 실물처럼 선언하되(스키마 형태 재현),
    # 아래 두 INSERT 가 실제로 같은 키를 쓰게 하려면 그 제약을 걸지 않아야
    # 한다 — 살아있는 fleet.db 의 인덱스가 손상돼 있던 것과 같은 최종
    # 상태(제약이 선언은 됐지만 실질적으로 뚫린 데이터)를 데이터 수준에서
    # 직접 재현한다.
    cur.execute("""
        CREATE TABLE events (
            id INTEGER NOT NULL,
            robot_id VARCHAR(64) NOT NULL,
            ts DATETIME NOT NULL,
            channel VARCHAR(32),
            seq INTEGER,
            kind VARCHAR(32) NOT NULL,
            severity VARCHAR(16) NOT NULL,
            msg VARCHAR(256) NOT NULL,
            payload_json JSON NOT NULL,
            PRIMARY KEY (id),
            FOREIGN KEY(robot_id) REFERENCES robots (id)
        )""")
    cur.execute("""INSERT INTO events
        (id, robot_id, ts, channel, seq, kind, severity, msg, payload_json)
        VALUES (10, 'scout01', '2026-08-13 19:11:17.955903', 'evt', 31688,
                'pong', 'info', 'pong', '{"kind":"pong"}')""")
    cur.execute("""INSERT INTO events
        (id, robot_id, ts, channel, seq, kind, severity, msg, payload_json)
        VALUES (20, 'scout01', '2026-08-13 22:28:58.861058', 'evt', 31688,
                'cmd_result', 'info', 'mission_start → completed',
                '{"kind":"cmd_result","cmd_id":"m23","data":{"distance_m":0.5}}')""")
    con.commit()
    con.close()


def test_old_schema_with_duplicate_seq_reassigns_epoch_without_data_loss(tmp_path):
    """N1 — 손상 DB(같은 (robot_id,channel,seq) 중복)를 만나도 한 행도 잃지
    않는다. 최초 구현은 INSERT OR IGNORE 로 나중 행(여기서는 값어치 있는
    cmd_result)을 조용히 버렸다 — T6 §8.2/I5 사고를 자동으로 반복하는
    경로였다. 지금은 중복 그룹의 2번째 행에 epoch=1 을 배정해 둘 다
    보존해야 한다."""
    db_path = tmp_path / "corrupt_old.db"
    _make_old_schema_db_with_duplicate(str(db_path))

    app = create_app(_settings_for(db_path), fleet=InMemoryFleetPort())

    with app.state.session_factory() as db:
        rows = {r.id: r for r in db.query(m.Event).filter_by(robot_id="scout01").all()}
        assert set(rows) == {10, 20}                     # 둘 다 살아 있다 — 행 손실 없음
        assert rows[10].kind == "pong" and rows[10].epoch == 0
        assert rows[20].kind == "cmd_result" and rows[20].epoch == 1  # 재배치
        assert rows[20].seq == rows[10].seq == 31688       # 같은 seq, 다른 epoch 로 공존

    # 방어선 — 재배치가 있었으니 옛 테이블은 지우지 않고 남겨야 한다.
    import sqlalchemy as sa
    inspector = sa.inspect(app.state.engine)
    assert "events_pre_epoch_migration" in inspector.get_table_names()


def test_leftover_pre_epoch_migration_table_raises_clear_error(tmp_path):
    """N2 — 이전 마이그레이션 시도의 잔재(events_pre_epoch_migration)가 이미
    있는 상태에서 또 마이그레이션을 시도하면, raw SQLite "table already
    exists" 대신 원인과 복구 지침이 담긴 명확한 오류로 멈춰야 한다."""
    db_path = tmp_path / "leftover.db"
    _make_old_schema_db(str(db_path))
    con = sqlite3.connect(str(db_path))
    con.execute("CREATE TABLE events_pre_epoch_migration (id INTEGER PRIMARY KEY)")
    con.commit()
    con.close()

    with pytest.raises(RuntimeError) as exc_info:
        create_app(_settings_for(db_path), fleet=InMemoryFleetPort())

    msg = str(exc_info.value)
    assert "events_pre_epoch_migration" in msg
    assert "복구 지침" in msg
