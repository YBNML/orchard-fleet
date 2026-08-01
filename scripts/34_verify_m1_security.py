#!/usr/bin/env python3
"""M1 보안 회귀 — CSRF·Origin·세션·D9·교차농장·계정정지·감사 무결성.

    server/.venv/bin/python scripts/34_verify_m1_security.py   (저장소 루트에서 실행)

골격은 33 과 동일 (서버 서브프로세스 + FakeRobot, check() 패턴). 보안은
"되는가"가 아니라 "안 되는 게 확실한가"를 확인하는 것이 핵심이라 실패 경로가
주 관심사다 (참조: scripts/21_verify_security.py 의 취지).

감사 무결성 범위에 대한 메모: WS 접속 이전(Origin 불일치·무세션) 거부와
CSRF 의존성 단계의 거부는 인증된 사용자 컨텍스트가 없거나 핸들러 코드에
도달하기 전에 FastAPI 의존성에서 끊기므로, 설계상(Task 4·11) audit.record 를
거치지 않는다 — 그래서 "감사 무결성" 검사(9번)는 실제로 사용자 컨텍스트가
있는 거부(텔레옵 거부·clear_estop 거부·교차농장 403·정지계정 로그인)만
대상으로 한다. 이 네 가지는 전부 실제로 audit_log 에 기록되는 경로다.
"""
from __future__ import annotations

import asyncio
import json
import os
import socket
import sqlite3
import subprocess
import sys
import tempfile
import time

import httpx
import websockets

sys.path.insert(0, "scripts")
from fake_legacy_robot import FakeRobot  # noqa: E402

PORT, RPORT = 18900, 18081
BASE = f"http://127.0.0.1:{PORT}"
OK, NG, res = "\033[92m✔\033[0m", "\033[91m✗\033[0m", []

# 감사에 남으면 안 되는 원문 비밀들 (마스킹 확인용)
SECRETS = {
    "op1pw": "op1_S3cr3t!", "op2pw": "op2_S3cr3t!",
    "obs1pw": "obs1_S3cr3t!", "dis1pw": "dis1_S3cr3t!",
    "adminpw": "admin_S3cr3t!",
}


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
    end = time.time() + timeout
    val = fn()
    while not val and time.time() < end:
        await asyncio.sleep(interval)
        val = fn()
    return val


async def ws_raw_connect(headers: dict):
    return await websockets.connect(f"ws://127.0.0.1:{PORT}/ws", additional_headers=headers)


async def ws_connect(cookie_val: str, origin: str = BASE):
    return await ws_raw_connect({"Cookie": f"fleet_session={cookie_val}", "Origin": origin})


async def ws_send(ws, obj: dict):
    await ws.send(json.dumps(obj))


async def wait_frame(ws, pred, timeout=10.0):
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


async def expect_ws_rejected(headers: dict):
    """접속이 거부되면 (True, 사유), 뜻밖에 수립되면 (False, 사유)."""
    try:
        ws = await ws_raw_connect(headers)
        try:
            await ws.close()
        except Exception:
            pass
        return False, "연결이 수립됨 (거부되어야 하는데 101 성공)"
    except websockets.exceptions.InvalidStatus as e:
        return True, f"HTTP {e.response.status_code}"
    except Exception as e:
        return True, f"{type(e).__name__}: {e}"


def dump_table_text(dbfile: str, table: str) -> str:
    conn = sqlite3.connect(dbfile, timeout=5)
    try:
        cur = conn.execute(f"SELECT * FROM {table}")
        rows = cur.fetchall()
        return json.dumps(rows, ensure_ascii=False, default=str)
    finally:
        conn.close()


async def main():
    for port, label in ((PORT, "관제 서버"), (RPORT, "가짜 로봇")):
        if not port_free(port):
            print(f"오류: 포트 {port} ({label}) 가 이미 사용 중입니다 — "
                  f"기존 프로세스를 정리하거나 스크립트 상단 PORT/RPORT 를 바꾸세요.",
                  file=sys.stderr)
            sys.exit(1)

    print("M1 보안 회귀 검증")
    print("=" * 74)

    tmp = tempfile.mkdtemp()
    dbfile = f"{tmp}/sec.db"
    env = dict(os.environ, FLEET_DB_URL=f"sqlite:///{dbfile}",
               FLEET_ADMIN_LOGIN="admin", FLEET_ADMIN_PASSWORD=SECRETS["adminpw"],
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
            raise SystemExit

        c = httpx.Client(base_url=BASE, timeout=10)
        r = c.post("/api/v1/auth/login", json={"login": "admin", "password": SECRETS["adminpw"]})
        check("사전조건: admin 로그인", r.status_code == 200, r.text)
        admin_csrf = r.json()["csrf"]
        h_admin = {"X-CSRF": admin_csrf}
        admin_cookie = c.cookies["fleet_session"]

        # ── 1. CSRF 없는 POST /farms → 403 ──────────────────────────────────
        r1 = c.post("/api/v1/farms", json={"name": "CSRF미첨부"})     # X-CSRF 헤더 없음
        check("1 CSRF 없는 POST /farms → 403", r1.status_code == 403, r1.text)

        # ── 2. Origin 불일치 WS → 접속 거부 ─────────────────────────────────
        rejected, detail = await expect_ws_rejected(
            {"Cookie": f"fleet_session={admin_cookie}", "Origin": "http://evil.example"})
        check("2 Origin 불일치 WS → 접속 거부(의도: 4403)", rejected, detail)

        # ── 3. 세션 없는 WS → 거부 ───────────────────────────────────────────
        rejected, detail = await expect_ws_rejected({"Origin": BASE})   # Cookie 없음
        check("3 세션 없는 WS → 거부(의도: 4401)", rejected, detail)

        # ── 준비: 농장 2 · 로봇 · 사용자들 ───────────────────────────────────
        f1 = c.post("/api/v1/farms", json={"name": "농장1"}, headers=h_admin).json()
        f2 = c.post("/api/v1/farms", json={"name": "농장2"}, headers=h_admin).json()
        c.post("/api/v1/robots", headers=h_admin, json={
            "id": "scout01", "farm_id": f1["id"], "name": "스카웃1",
            "conn_kind": "legacy_ws",
            "config_json": {"ws_url": f"ws://127.0.0.1:{RPORT}/ws", "token": "RTOK"}})
        c.post("/api/v1/users", headers=h_admin, json={
            "login": "op1", "password": SECRETS["op1pw"], "role": "operator",
            "farm_ids": [f1["id"]]})
        c.post("/api/v1/users", headers=h_admin, json={
            "login": "op2", "password": SECRETS["op2pw"], "role": "operator",
            "farm_ids": [f2["id"]]})
        c.post("/api/v1/users", headers=h_admin, json={
            "login": "obs1", "password": SECRETS["obs1pw"], "role": "observer",
            "farm_ids": [f1["id"]]})
        r_dis = c.post("/api/v1/users", headers=h_admin, json={
            "login": "dis1", "password": SECRETS["dis1pw"], "role": "observer",
            "farm_ids": [f1["id"]]})
        dis1_id = r_dis.json()["id"]
        r_patch = c.patch(f"/api/v1/users/{dis1_id}", headers=h_admin, json={"disabled": True})
        check("사전조건: 로봇·사용자 생성 + 계정 정지",
              r_patch.status_code == 200, r_patch.text)

        online = await poll_until(
            lambda: c.get("/api/v1/robots/scout01/status", headers=h_admin).json().get("online"),
            timeout=15, interval=0.3)
        check("사전조건: 로봇 온라인", bool(online))

        cop1 = httpx.Client(base_url=BASE, timeout=10)
        r = cop1.post("/api/v1/auth/login", json={"login": "op1", "password": SECRETS["op1pw"]})
        op1_csrf = r.json()["csrf"]

        cop2 = httpx.Client(base_url=BASE, timeout=10)
        r = cop2.post("/api/v1/auth/login", json={"login": "op2", "password": SECRETS["op2pw"]})
        op2_csrf = r.json()["csrf"]

        cobs1 = httpx.Client(base_url=BASE, timeout=10)
        r = cobs1.post("/api/v1/auth/login", json={"login": "obs1", "password": SECRETS["obs1pw"]})
        obs1_cookie = cobs1.cookies["fleet_session"]

        # ── 4. observer 텔레옵 → denied + 가짜 로봇 teleop_count == 0 ───────
        ws_obs1 = await ws_connect(obs1_cookie)
        ws_conns.append(ws_obs1)
        await wait_frame(ws_obs1, lambda o: True, timeout=5)          # ready
        await ws_send(ws_obs1, {"type": "teleop", "robot": "scout01",
                                "payload": {"vx": 0.3, "wz": 0.0}})
        r4 = await wait_frame(ws_obs1, lambda o: o.get("type") in ("denied", "cmd_result"), timeout=5)
        await asyncio.sleep(0.3)          # 혹시 전달됐다면 가짜 로봇에 반영될 시간
        check("4 observer 텔레옵 → denied", bool(r4) and r4.get("type") == "denied", r4)
        check("4 가짜 로봇 teleop_count == 0", robot.teleop_count == 0, str(robot.teleop_count))

        # ── 5. observer estop → 허용 (D9 회귀) ──────────────────────────────
        await ws_send(ws_obs1, {"type": "cmd", "action": "estop", "robot": "scout01", "cmd_id": "s5"})
        r5 = await wait_frame(ws_obs1, lambda o: o.get("type") == "cmd_result" and o.get("cmd_id") == "s5")
        check("5 observer estop → 허용(D9)", bool(r5) and r5.get("result") == "sent", r5)
        got_estop = await poll_until(lambda: robot.estop is True, timeout=5)
        check("5 가짜 로봇이 estop 을 수신", bool(got_estop))

        # ── 6. operator 의 clear_estop → denied (admin 전용) ────────────────
        ws_op1 = await ws_raw_connect({"Cookie": f"fleet_session={cop1.cookies['fleet_session']}",
                                       "Origin": BASE})
        ws_conns.append(ws_op1)
        await wait_frame(ws_op1, lambda o: True, timeout=5)           # ready
        await ws_send(ws_op1, {"type": "cmd", "action": "clear_estop", "robot": "scout01", "cmd_id": "c6"})
        r6 = await wait_frame(ws_op1, lambda o: o.get("type") in ("denied", "cmd_result"), timeout=5)
        check("6 operator 의 clear_estop → denied(admin 전용)",
              bool(r6) and r6.get("type") == "denied", r6)
        check("6 estop 래치 유지(무효화 안 됨)", robot.estop is True, str(robot.estop))

        # ── 7. 타 농장 operator 임무 → 403 ──────────────────────────────────
        r7 = cop2.post("/api/v1/missions", headers={"X-CSRF": op2_csrf},
                       json={"robot_id": "scout01", "alleys": [0]})
        check("7 타 농장 operator 임무 → 403", r7.status_code == 403, r7.text)

        # ── 8. 정지(disabled) 계정 로그인 → 401 ──────────────────────────────
        r8 = httpx.post(BASE + "/api/v1/auth/login",
                        json={"login": "dis1", "password": SECRETS["dis1pw"]}, timeout=10)
        check("8 정지 계정 로그인 → 401", r8.status_code == 401, r8.text)

        # ── 9. 감사 무결성 ───────────────────────────────────────────────────
        audit_rows = c.get("/api/v1/audit", headers=h_admin, params={"limit": 500}).json()

        def has(action, result, target_sub=None, detail_sub=None):
            for a in audit_rows:
                if a.get("action") != action or a.get("result") != result:
                    continue
                if target_sub and target_sub not in (a.get("target") or ""):
                    continue
                if detail_sub and detail_sub not in (a.get("detail") or ""):
                    continue
                return True
            return False

        check("9-1 텔레옵 거부가 감사에 존재", has("teleop", "rejected", target_sub="scout01"))
        check("9-2 clear_estop 거부가 감사에 존재", has("clear_estop", "rejected"))
        check("9-3 교차농장 임무거부가 감사에 존재",
              has("mission_start", "rejected", detail_sub="농장 권한 없음"))
        check("9-4 정지계정 로그인거부가 감사에 존재", has("login", "rejected", target_sub="dis1"))

        blob = json.dumps(audit_rows, ensure_ascii=False)
        leaked = [v for v in SECRETS.values() if v in blob]
        check("9-5 감사 응답에 비밀번호 원문 없음", not leaked, str(leaked))
        check("9-6 감사 응답에 세션·CSRF 토큰 원문 없음",
              admin_cookie not in blob and admin_csrf not in blob and op1_csrf not in blob)

        users_resp = c.get("/api/v1/users", headers=h_admin).json()
        users_blob = json.dumps(users_resp, ensure_ascii=False)
        check("9-7 /users 응답에 비밀번호 필드 없음",
              "password" not in users_blob and "pw_hash" not in users_blob
              and not any(v in users_blob for v in SECRETS.values()))

        # DB 원본 파일까지 직접 열어 audit_log 테이블에서 원문 부재를 재확인
        audit_dump = dump_table_text(dbfile, "audit_log")
        leaked_db = [v for v in SECRETS.values() if v in audit_dump]
        check("9-8 audit_log DB 덤프에 비밀번호 원문 없음", not leaked_db, str(leaked_db))
        check("9-9 audit_log DB 덤프에 세션·CSRF 토큰 원문 없음",
              admin_cookie not in audit_dump and admin_csrf not in audit_dump)

        # ── 보너스: 로그아웃 후 /me 401 (세션 소멸 확인 — 세션만료 TTL 검증의 대체) ─
        r_logout = c.post("/api/v1/auth/logout", headers=h_admin)
        r_me = c.get("/api/v1/auth/me")
        check("보너스 로그아웃 후 /me → 401 (세션 무효화)",
              r_logout.status_code == 200 and r_me.status_code == 401,
              f"logout={r_logout.status_code} me={r_me.status_code}")

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
