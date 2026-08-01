#!/usr/bin/env python3
"""M1 E2E — 서버+가짜 로봇으로 스펙 §0.2 관제 완성 기준을 검사한다.

    server/.venv/bin/python scripts/33_verify_m1.py   (저장소 루트에서 실행)

서버는 uvicorn 서브프로세스로, 로봇은 fake_legacy_robot.FakeRobot 으로 같은
asyncio 이벤트루프 안에서 띄운다. 로봇쪽 코루틴(0.2s 주기 pump)과 경합하지
않도록, 접속 이후의 모든 대기는 asyncio.sleep 기반이어야 한다 — 여기서
time.sleep 을 쓰면 이벤트루프가 멎어 가짜 로봇도 같이 멈춘다(교착).
"""
from __future__ import annotations

import asyncio
import json
import os
import re
import socket
import subprocess
import sys
import tempfile
import time

import httpx
import websockets

sys.path.insert(0, "scripts")
from fake_legacy_robot import FakeRobot  # noqa: E402

PORT, RPORT = 18800, 18080
BASE = f"http://127.0.0.1:{PORT}"
OK, NG, res = "\033[92m✔\033[0m", "\033[91m✗\033[0m", []


def check(name, cond, detail=""):
    res.append(bool(cond))
    print(f" {OK if cond else NG} {name}" + (f" — {detail}" if detail else ""))


def port_free(port: int) -> bool:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def wait_http(url, sec=20):
    for _ in range(sec * 10):
        try:
            httpx.get(url, timeout=1)
            return True
        except Exception:
            time.sleep(0.1)
    return False


async def poll_until(fn, timeout=15.0, interval=0.2):
    """fn() 이 참을 낼 때까지 폴링한다 (asyncio.sleep 이라 로봇 코루틴과 공존)."""
    end = time.time() + timeout
    val = fn()
    while not val and time.time() < end:
        await asyncio.sleep(interval)
        val = fn()
    return val


async def ws_connect(cookie_val: str, origin: str = BASE):
    return await websockets.connect(
        f"ws://127.0.0.1:{PORT}/ws",
        additional_headers={"Cookie": f"fleet_session={cookie_val}", "Origin": origin})


async def ws_send(ws, obj: dict):
    await ws.send(json.dumps(obj))


async def wait_frame(ws, pred, timeout=10.0):
    """pred(frame)==True 인 프레임이 나올 때까지, 상관없는 텔레메트리는 건너뛴다."""
    end = time.time() + timeout
    while time.time() < end:
        remain = max(0.05, end - time.time())
        try:
            raw = await asyncio.wait_for(ws.recv(), timeout=remain)
        except (asyncio.TimeoutError, websockets.exceptions.ConnectionClosed):
            return None
        obj = json.loads(raw)
        if pred(obj):
            return obj
    return None


async def wait_topic(ws, suffix, timeout=10.0, pred=None):
    def _p(o):
        return o.get("topic", "").endswith(suffix) and (pred is None or pred(o.get("payload", {})))
    return await wait_frame(ws, _p, timeout)


def has_tz_suffix(ts) -> bool:
    """Critical 2 회귀 — naive(tz 없는) 문자열이면 대시보드의 Date.parse() 가
    로컬시간(KST)으로 오해석해 이력 재생 시각이 9시간 어긋난다."""
    if not ts:
        return False
    return ts.endswith("Z") or bool(re.search(r"[+-]\d{2}:\d{2}$", ts))


def mission_state(cli: httpx.Client, mission_id: int, robot_id="scout01"):
    rows = cli.get("/api/v1/missions", params={"robot_id": robot_id}).json()
    cur = next((m for m in rows if m["id"] == mission_id), None)
    return cur["state"] if cur else None


async def main():
    for port, label in ((PORT, "관제 서버"), (RPORT, "가짜 로봇")):
        if not port_free(port):
            print(f"오류: 포트 {port} ({label}) 가 이미 사용 중입니다 — "
                  f"기존 프로세스를 정리하거나 스크립트 상단 PORT/RPORT 를 바꾸세요.",
                  file=sys.stderr)
            sys.exit(1)

    print("M1 E2E 검증")
    print("=" * 74)

    tmp = tempfile.mkdtemp()
    env = dict(os.environ, FLEET_DB_URL=f"sqlite:///{tmp}/e2e.db",
               FLEET_ADMIN_LOGIN="admin", FLEET_ADMIN_PASSWORD="admpw",
               FLEET_ALLOWED_ORIGINS=BASE, FLEET_LOGIN_DELAY_S="0")
    srv = subprocess.Popen([sys.executable, "-m", "uvicorn",
                            "fleet_server.app:create_app", "--factory",
                            "--port", str(PORT)], env=env, cwd="server",
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    robot = FakeRobot(robot_id="scout01", port=RPORT, token="RTOK")
    server_ws = None
    ws_conns: list = []
    try:
        server_ws = await robot.serve()

        started = wait_http(BASE + "/")
        check("서버 기동", started)
        if not started:
            if srv.poll() is not None:
                print(f"  (서버 프로세스가 조기 종료됨 — returncode={srv.returncode})",
                      file=sys.stderr)
            raise SystemExit  # finally 로 정리만 하고 아래 요약에서 실패 처리

        c = httpx.Client(base_url=BASE, timeout=10)

        # ── 1. 로그인 + 쿠키 플래그 ──────────────────────────────────────────
        r = c.post("/api/v1/auth/login", json={"login": "admin", "password": "admpw"})
        sc = r.headers.get("set-cookie", "").lower()
        check("1 admin 로그인 + 쿠키 플래그(HttpOnly·SameSite=Strict)",
              r.status_code == 200 and "httponly" in sc and "samesite=strict" in sc, sc)
        admin_csrf = r.json()["csrf"]
        h_admin = {"X-CSRF": admin_csrf}

        # ── 2. 농장 2 · 로봇 1(가짜 로봇 주소) · operator · observer 생성 ────
        f1 = c.post("/api/v1/farms", json={"name": "농장1"}, headers=h_admin).json()
        f2 = c.post("/api/v1/farms", json={"name": "농장2"}, headers=h_admin).json()
        r_robot = c.post("/api/v1/robots", headers=h_admin, json={
            "id": "scout01", "farm_id": f1["id"], "name": "스카웃1",
            "conn_kind": "legacy_ws",
            "config_json": {"ws_url": f"ws://127.0.0.1:{RPORT}/ws", "token": "RTOK"}})
        r_op = c.post("/api/v1/users", headers=h_admin, json={
            "login": "op1", "password": "op1pw", "role": "operator", "farm_ids": [f1["id"]]})
        r_obs = c.post("/api/v1/users", headers=h_admin, json={
            "login": "obs1", "password": "obs1pw", "role": "observer", "farm_ids": [f1["id"]]})
        r_obs2 = c.post("/api/v1/users", headers=h_admin, json={
            "login": "obs2", "password": "obs2pw", "role": "observer", "farm_ids": [f2["id"]]})
        check("2 농장 2·로봇 1·operator·observer 생성",
              r_robot.status_code == 200 and r_op.status_code == 200
              and r_obs.status_code == 200 and r_obs2.status_code == 200,
              f"robot={r_robot.status_code} op={r_op.status_code} "
              f"obs={r_obs.status_code} obs2={r_obs2.status_code}")

        # ── 3. 로봇 온라인 전환 (≤15s) ───────────────────────────────────────
        online = await poll_until(
            lambda: c.get("/api/v1/robots/scout01/status", headers=h_admin).json().get("online"),
            timeout=15, interval=0.3)
        check("3 로봇 온라인 전환 (≤15s)", bool(online))

        # 로그인: operator·observer 둘
        cop = httpx.Client(base_url=BASE, timeout=10)
        r = cop.post("/api/v1/auth/login", json={"login": "op1", "password": "op1pw"})
        op1_csrf = r.json()["csrf"]

        cobs = httpx.Client(base_url=BASE, timeout=10)
        r = cobs.post("/api/v1/auth/login", json={"login": "obs1", "password": "obs1pw"})
        obs1_csrf = r.json()["csrf"]
        obs1_cookie = cobs.cookies["fleet_session"]

        cobs2 = httpx.Client(base_url=BASE, timeout=10)
        r = cobs2.post("/api/v1/auth/login", json={"login": "obs2", "password": "obs2pw"})
        obs2_csrf = r.json()["csrf"]
        obs2_cookie = cobs2.cookies["fleet_session"]

        admin_cookie = c.cookies["fleet_session"]

        # ── 4. observer WS 접속 → tel/state 수신 (x 증가 확인) ───────────────
        ws_obs1 = await ws_connect(obs1_cookie)
        ws_conns.append(ws_obs1)
        ready = await wait_frame(ws_obs1, lambda o: True, timeout=5)
        m1 = await wait_topic(ws_obs1, "/tel/state", timeout=8)
        m2 = await wait_topic(ws_obs1, "/tel/state", timeout=8)
        x1 = m1["payload"]["x"] if m1 else None
        x2 = m2["payload"]["x"] if m2 else None
        check("4 observer WS 접속 → tel/state 수신 (x 증가)",
              bool(ready) and ready.get("type") == "ready" and m1 and m2 and x2 > x1,
              f"x1={x1} x2={x2}")

        # ── 5. observer(타 농장) 는 텔레메트리를 받지 않음 ───────────────────
        ws_obs2 = await ws_connect(obs2_cookie)
        ws_conns.append(ws_obs2)
        await wait_frame(ws_obs2, lambda o: True, timeout=5)          # ready
        leaked = await wait_topic(ws_obs2, "/tel/state", timeout=3)
        check("5 observer 가 타 농장 텔레메트리를 받지 않음", leaked is None,
              "새어나온 프레임: " + json.dumps(leaked) if leaked else "")

        # 관리자 WS (clear_estop·stop_all 등 admin 전용 명령에 사용)
        ws_admin = await ws_connect(admin_cookie)
        ws_conns.append(ws_admin)
        await wait_frame(ws_admin, lambda o: True, timeout=5)         # ready

        # ── 6. operator REST 임무 시작 → mission_start 수신 → RUNNING → DONE ─
        r = cop.post("/api/v1/missions", headers={"X-CSRF": op1_csrf},
                     json={"robot_id": "scout01", "alleys": [0]})
        check("6-1 임무 생성 200(QUEUED)", r.status_code == 200 and r.json().get("state") == "QUEUED",
              r.text)
        ms1 = r.json()

        got_start = await poll_until(
            lambda: any(m.get("payload", {}).get("cmd") == "mission_start" for m in robot.received),
            timeout=5)
        check("6-2 가짜 로봇이 mission_start 수신", bool(got_start))

        running1 = await poll_until(lambda: mission_state(cop, ms1["id"]) == "RUNNING", timeout=5)
        check("6-3 임무 RUNNING 전이", bool(running1))

        done1 = await poll_until(lambda: mission_state(cop, ms1["id"]) == "DONE", timeout=8)
        check("6-4 로봇 완주 보고 후 임무 DONE", bool(done1))

        # ── 7. observer WS estop → 가짜 로봇 수신 + 이벤트 이력 적재 (D9) ────
        await ws_send(ws_obs1, {"type": "cmd", "action": "estop", "robot": "scout01", "cmd_id": "e7"})
        r7 = await wait_frame(ws_obs1, lambda o: o.get("type") == "cmd_result" and o.get("cmd_id") == "e7")
        check("7-1 observer estop 허용(D9) → 전달", bool(r7) and r7.get("result") == "sent", r7)

        got_robot_estop = await poll_until(lambda: robot.estop is True, timeout=5)
        check("7-2 가짜 로봇 estop 수신", bool(got_robot_estop))

        ev_ok = await poll_until(
            lambda: any(e.get("kind") == "estop" for e in
                       c.get("/api/v1/events", params={"robot_id": "scout01"}, headers=h_admin).json()),
            timeout=5)
        check("7-3 이벤트 이력 적재", bool(ev_ok))

        # ── 8. 새 임무 → estop 으로 PAUSED, clear_estop 은 래치만, resume 으로만 재개 ─
        r = cop.post("/api/v1/missions", headers={"X-CSRF": op1_csrf},
                     json={"robot_id": "scout01", "alleys": [1]})
        check("8-1 두 번째 임무 생성", r.status_code == 200, r.text)
        ms2 = r.json()

        running2 = await poll_until(lambda: mission_state(cop, ms2["id"]) == "RUNNING", timeout=5)
        check("8-2 새 임무 RUNNING 전이", bool(running2))

        await ws_send(ws_obs1, {"type": "cmd", "action": "estop", "robot": "scout01", "cmd_id": "e8"})
        r8 = await wait_frame(ws_obs1, lambda o: o.get("type") == "cmd_result" and o.get("cmd_id") == "e8")
        check("8-3 WS estop 전달", bool(r8) and r8.get("result") == "sent", r8)

        paused2 = await poll_until(lambda: mission_state(cop, ms2["id"]) == "PAUSED", timeout=5)
        check("8-4 estop 중 임무 PAUSED", bool(paused2))

        await ws_send(ws_admin, {"type": "cmd", "action": "clear_estop", "robot": "scout01", "cmd_id": "ce8"})
        r_ce = await wait_frame(ws_admin, lambda o: o.get("type") == "cmd_result" and o.get("cmd_id") == "ce8")
        check("8-5 admin clear_estop 전달", bool(r_ce) and r_ce.get("result") == "sent", r_ce)

        await asyncio.sleep(1.5)                    # 자동 재개가 없는지 관찰할 시간
        still_paused = mission_state(cop, ms2["id"])
        check("8-6 clear_estop 후에도 PAUSED 유지 (자동 재개 없음)",
              still_paused == "PAUSED", still_paused)

        r = cop.post(f"/api/v1/missions/{ms2['id']}/resume", headers={"X-CSRF": op1_csrf})
        check("8-7 mission_resume 으로만 재개 → RUNNING",
              r.status_code == 200 and r.json().get("state") == "RUNNING", r.text)

        # ── 9. 이력·감사 적재 확인 ────────────────────────────────────────
        tracks = c.get("/api/v1/tracks", params={"robot_id": "scout01"}, headers=h_admin).json()
        check("9-1 GET /tracks 에 1Hz 궤적 적재", isinstance(tracks, list) and len(tracks) >= 2,
              f"{len(tracks)}건")

        events = c.get("/api/v1/events", params={"robot_id": "scout01"}, headers=h_admin).json()
        check("9-2 GET /events 에 estop 이벤트", any(e.get("kind") == "estop" for e in events),
              f"{len(events)}건")

        audit_rows = c.get("/api/v1/audit", headers=h_admin).json()
        check("9-3 GET /audit 에 명령 기록",
              any(a.get("action") in ("mission_start", "estop", "clear_estop") for a in audit_rows),
              f"{len(audit_rows)}건")

        # ── 9-4~9-7. 시각 문자열에 tz 접미사(+00:00/Z) — Critical 2 회귀 ─────────
        # SQLite 왕복 후(다른 세션의 재조회) naive 로 돌아온 datetime 에 그대로
        # isoformat() 을 쓰면 접미사가 빠져 KST 에서 이력 재생이 9시간 어긋난다.
        missions_list = c.get("/api/v1/missions", params={"robot_id": "scout01"},
                              headers=h_admin).json()
        mission_ts_ok = bool(missions_list) and all(
            has_tz_suffix(m.get("created_at"))
            and (m.get("started_at") is None or has_tz_suffix(m["started_at"]))
            and (m.get("ended_at") is None or has_tz_suffix(m["ended_at"]))
            for m in missions_list)
        check("9-4 /missions 시각 문자열에 tz 접미사(+00:00/Z)", mission_ts_ok,
              str([m.get("created_at") for m in missions_list]))

        track_ts_ok = bool(tracks) and all(has_tz_suffix(t["ts"]) for t in tracks)
        check("9-5 /tracks 시각 문자열에 tz 접미사(+00:00/Z)", track_ts_ok,
              str([t["ts"] for t in tracks[:2]]))

        event_ts_ok = bool(events) and all(has_tz_suffix(e["ts"]) for e in events)
        check("9-6 /events 시각 문자열에 tz 접미사(+00:00/Z)", event_ts_ok,
              str([e["ts"] for e in events[:2]]))

        audit_ts_ok = bool(audit_rows) and all(has_tz_suffix(a["ts"]) for a in audit_rows)
        check("9-7 /audit 시각 문자열에 tz 접미사(+00:00/Z)", audit_ts_ok,
              str([a["ts"] for a in audit_rows[:2]]))

        robots_list = c.get("/api/v1/robots", headers=h_admin).json()
        robot_row = next((r for r in robots_list if r["id"] == "scout01"), None)
        check("9-8 /robots 의 last_seen 시각 문자열에 tz 접미사(값이 있을 때)",
              robot_row is not None and (robot_row.get("last_seen") is None
                                         or has_tz_suffix(robot_row["last_seen"])),
              str(robot_row.get("last_seen") if robot_row else None))

        # ── 10. stop_all → 결과 dict 에 scout01=sent ────────────────────────
        await ws_send(ws_admin, {"type": "cmd", "action": "stop_all", "cmd_id": "s10"})
        r10 = await wait_frame(ws_admin, lambda o: o.get("type") == "stop_all_result")
        check("10 stop_all → 결과 dict scout01=sent",
              bool(r10) and r10.get("results", {}).get("scout01") == "sent", r10)

    except SystemExit:
        pass
    finally:
        for ws in ws_conns:
            try:
                await ws.close()
            except Exception:
                pass
        if server_ws is not None:
            server_ws.close()
            try:
                await asyncio.wait_for(server_ws.wait_closed(), timeout=5)
            except Exception:
                pass
        srv.terminate()
        try:
            srv.wait(timeout=10)
        except subprocess.TimeoutExpired:
            srv.kill()
            srv.wait(timeout=5)

    print("\n" + "=" * 74)
    print(f"{sum(res)}/{len(res)} 통과")
    sys.exit(0 if res and all(res) else 1)


asyncio.run(main())
