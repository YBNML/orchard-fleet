import datetime as dt

import pytest
from sqlalchemy.exc import IntegrityError

from fleet_server import models as m


def test_tables_create(db):
    names = {t.name for t in m.Base.metadata.sorted_tables}
    assert names == {"users", "auth_sessions", "farms", "user_farms", "robots",
                     "missions", "mission_events", "tracks", "events", "audit_log",
                     "interventions"}


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
