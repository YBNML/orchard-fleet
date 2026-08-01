import datetime as dt

from fleet_server import interventions as iv
from fleet_server import metrics, stopcodes
from fleet_server import models as m
from tests.conftest import do_login


# ── 코드표 ──────────────────────────────────────────────────────────────────
def test_stopcode_lookup_and_intervention_definition():
    assert stopcodes.get("ESTOP_REMOTE").needs_site_visit is True
    assert stopcodes.get("estop_remote").code == "ESTOP_REMOTE"      # 대소문자 무관
    assert stopcodes.get("듣도보도못한코드").code == "UNKNOWN"        # 미지 코드도 안 깨짐
    # 개입의 정의 — 정상 종료·지시 정지는 개입이 아니다
    assert stopcodes.is_intervention("OBSTACLE_FRONT") is True
    assert stopcodes.is_intervention("MISSION_DONE") is False
    assert stopcodes.is_intervention("OPERATOR_PAUSE") is False


def _seed(db):
    f = m.Farm(name="농장A"); db.add(f); db.flush()
    db.add(m.Robot(id="scout01", farm_id=f.id, name="r"))
    u = m.User(login="op", pw_hash="h", role="operator")   # FK 강제라 실제로 있어야 한다
    db.add(u); db.commit()
    return f


# ── 큐 동작 ─────────────────────────────────────────────────────────────────
def test_repeat_does_not_flood_queue(db):
    f = _seed(db)
    for _ in range(30):                       # 로봇이 같은 사유로 30번 울어도
        iv.open_or_bump(db, robot_id="scout01", farm_id=f.id, code="OBSTACLE_FRONT")
    rows = db.query(m.Intervention).all()
    assert len(rows) == 1                     # 큐는 한 건
    assert rows[0].context_json["repeat"] == 29


def test_non_intervention_code_creates_nothing(db):
    f = _seed(db)
    assert iv.open_or_bump(db, robot_id="scout01", farm_id=f.id,
                           code="MISSION_DONE") is None
    assert db.query(m.Intervention).count() == 0


def test_ack_resolve_records_both_times(db):
    f = _seed(db)
    row = iv.open_or_bump(db, robot_id="scout01", farm_id=f.id, code="OBSTACLE_FRONT")
    iv.ack(db, row, user_id=1)
    assert row.state == "ACKED" and row.acked_at is not None
    iv.resolve(db, row, user_id=1, note="치웠음")
    assert row.state == "RESOLVED" and row.resolved_at is not None
    assert row.note == "치웠음"


def test_resolve_without_ack_still_stamps_ack(db):
    f = _seed(db)
    row = iv.open_or_bump(db, robot_id="scout01", farm_id=f.id, code="GEOFENCE")
    iv.resolve(db, row, user_id=1)
    assert row.acked_at is not None            # 응답 시각이 비면 지표가 깨진다


def test_escalate_marks_site_visit(db):
    f = _seed(db)
    row = iv.open_or_bump(db, robot_id="scout01", farm_id=f.id, code="OBSTACLE_FRONT")
    assert row.needs_site_visit is False
    iv.escalate(db, row, user_id=1, note="원격으로 못 뺌")
    assert row.state == "ESCALATED" and row.needs_site_visit is True


def test_auto_resolve_closes_open_rows(db):
    f = _seed(db)
    iv.open_or_bump(db, robot_id="scout01", farm_id=f.id, code="OBSTACLE_FRONT")
    assert iv.auto_resolve(db, "scout01", "OBSTACLE_FRONT") == 1
    assert db.query(m.Intervention).one().state == "RESOLVED"


# ── 지표 ────────────────────────────────────────────────────────────────────
def test_metrics_counts_and_percentiles(db):
    f = _seed(db)
    t0 = dt.datetime(2026, 8, 1, 9, 0, tzinfo=dt.UTC)
    ms = m.Mission(robot_id="scout01", farm_id=f.id, spec_json={"alleys": [0, 1, 2, 3]},
                   state="DONE", created_by=1, started_at=t0,
                   ended_at=t0 + dt.timedelta(minutes=40))
    db.add(ms)
    for i, secs in enumerate((10, 20, 30, 300)):      # 처리시간 분포
        row = m.Intervention(robot_id="scout01", farm_id=f.id, code="OBSTACLE_FRONT",
                             category="인지", opened_at=t0,
                             acked_at=t0 + dt.timedelta(seconds=secs),
                             resolved_at=t0 + dt.timedelta(seconds=secs * 2),
                             state="RESOLVED")
        db.add(row)
    db.add(m.Intervention(robot_id="scout01", farm_id=f.id, code="ESTOP_REMOTE",
                          category="지시", needs_site_visit=True, opened_at=t0,
                          state="OPEN"))
    db.commit()

    s = metrics.summary(db)
    assert s["interventions"] == 5
    assert s["open_now"] == 1
    assert s["alleys_worked"] == 4
    assert s["per_alley"] == round(5 / 4, 3)
    assert s["site_visits"] == 1                       # 규격상 사람이 가야 하는 건
    assert s["active_min"] == 40.0
    assert s["mtbi_min"] == round(2400 / 5 / 60, 1)
    assert s["ack_p50_s"] is not None and s["ack_p95_s"] >= s["ack_p50_s"]
    assert s["work_days"] == 1
    cats = {c["category"]: c["count"] for c in s["by_category"]}
    assert cats == {"인지": 4, "지시": 1}


# ── API ─────────────────────────────────────────────────────────────────────
def _seed_api(client):
    csrf = do_login(client)
    h = {"X-CSRF": csrf}
    fa = client.post("/api/v1/farms", json={"name": "농장A"}, headers=h).json()
    client.post("/api/v1/robots", headers=h,
                json={"id": "scout01", "farm_id": fa["id"], "name": "r"})
    client.post("/api/v1/users", headers=h, json={
        "login": "obs", "password": "obspw", "role": "observer", "farm_ids": [fa["id"]]})
    return fa, csrf


def test_stopcodes_endpoint(client):
    do_login(client)
    codes = client.get("/api/v1/stopcodes").json()
    assert any(c["code"] == "NEGATIVE_OBSTACLE" and c["needs_site_visit"] for c in codes)


def test_queue_flow_via_api(client, app):
    fa, csrf = _seed_api(client)
    with app.state.session_factory() as db:
        iv.open_or_bump(db, robot_id="scout01", farm_id=fa["id"], code="OBSTACLE_FRONT")
    rows = client.get("/api/v1/interventions?state=active").json()
    assert len(rows) == 1 and rows[0]["state"] == "OPEN"
    iid = rows[0]["id"]
    assert rows[0]["label"] == "전방 장애물"                 # 라벨은 서버가 준다

    r = client.post(f"/api/v1/interventions/{iid}/ack", headers={"X-CSRF": csrf})
    assert r.status_code == 200 and r.json()["state"] == "ACKED"
    r = client.post(f"/api/v1/interventions/{iid}/resolve", headers={"X-CSRF": csrf},
                    json={"note": "치웠음"})
    assert r.status_code == 200 and r.json()["state"] == "RESOLVED"


def test_observer_cannot_act_on_queue(client, app):
    fa, csrf0 = _seed_api(client)
    with app.state.session_factory() as db:
        iv.open_or_bump(db, robot_id="scout01", farm_id=fa["id"], code="OBSTACLE_FRONT")
    iid = client.get("/api/v1/interventions").json()[0]["id"]
    csrf = do_login(client, "obs", "obspw")
    r = client.post(f"/api/v1/interventions/{iid}/ack", headers={"X-CSRF": csrf})
    assert r.status_code == 403                          # 보기만 하는 역할


def test_queue_requires_csrf(client, app):
    fa, _ = _seed_api(client)
    with app.state.session_factory() as db:
        iv.open_or_bump(db, robot_id="scout01", farm_id=fa["id"], code="OBSTACLE_FRONT")
    iid = client.get("/api/v1/interventions").json()[0]["id"]
    assert client.post(f"/api/v1/interventions/{iid}/ack").status_code == 403


def test_metrics_endpoint(client, app):
    fa, _ = _seed_api(client)
    with app.state.session_factory() as db:
        iv.open_or_bump(db, robot_id="scout01", farm_id=fa["id"], code="ESTOP_REMOTE")
    s = client.get("/api/v1/metrics").json()
    assert s["interventions"] == 1 and s["site_visits"] == 1
