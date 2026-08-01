"""
의존성 없는 WebSocket + 정적파일 서버 (RFC 6455)

이 머신은 pip 가 PEP 668 로 잠겨 있고 sudo 도 없어서 fastapi/uvicorn/websockets 를
설치할 수 없다. venv 도 ensurepip 가 없어 실패한다. 그래서 표준 라이브러리만으로
짰다. 결과적으로 배포가 단순해지는 이점도 있다 — 로봇에 올릴 때 파이썬 표준
라이브러리 말고는 아무것도 필요 없다.

지원 범위는 우리가 실제로 쓰는 것까지만이다: 텍스트 프레임, ping/pong, close,
클라이언트 마스킹. 확장(permessage-deflate)·바이너리·조각난 프레임은 다루지 않는다.
프레임 코덱을 따로 떼어둔 이유는, 나중에 로봇이 관제로 **걸어 나가는**(아웃바운드)
연결이 필요해지면 같은 코덱으로 클라이언트를 붙이기 위해서다.
"""
from __future__ import annotations

import base64
import hashlib
import hmac
import json
import os
import socket
import ssl
import struct
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from orchard_sim.link import protocol as P

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

OP_CONT, OP_TEXT, OP_BIN, OP_CLOSE, OP_PING, OP_PONG = 0x0, 0x1, 0x2, 0x8, 0x9, 0xA

MIME = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
        ".css": "text/css; charset=utf-8", ".json": "application/json",
        ".svg": "image/svg+xml", ".png": "image/png", ".ico": "image/x-icon"}


# ═══════════════════════════════════════════════════════════════════════════
# 프레임 코덱
# ═══════════════════════════════════════════════════════════════════════════
def encode_frame(payload: bytes, opcode: int = OP_TEXT, mask: bool = False) -> bytes:
    """서버→클라이언트는 마스킹하지 않는다 (RFC 6455 §5.1)."""
    n = len(payload)
    head = bytearray([0x80 | opcode])
    mbit = 0x80 if mask else 0x00
    if n < 126:
        head.append(mbit | n)
    elif n < (1 << 16):
        head.append(mbit | 126)
        head += struct.pack("!H", n)
    else:
        head.append(mbit | 127)
        head += struct.pack("!Q", n)
    if mask:
        key = os.urandom(4)
        head += key
        payload = bytes(b ^ key[i % 4] for i, b in enumerate(payload))
    return bytes(head) + payload


def _recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("연결 종료")
        buf += chunk
    return buf


def decode_frame(sock):
    """한 프레임을 읽어 (opcode, payload) 로 돌려준다."""
    b0, b1 = _recv_exact(sock, 2)
    opcode = b0 & 0x0F
    masked = bool(b1 & 0x80)
    n = b1 & 0x7F
    if n == 126:
        n = struct.unpack("!H", _recv_exact(sock, 2))[0]
    elif n == 127:
        n = struct.unpack("!Q", _recv_exact(sock, 8))[0]
    if n > 8 << 20:                       # 8 MB 초과는 우리 규약에 없다 — 끊는다
        raise ConnectionError(f"프레임이 너무 크다: {n}")
    key = _recv_exact(sock, 4) if masked else None
    data = _recv_exact(sock, n) if n else b""
    if masked:
        data = bytes(b ^ key[i % 4] for i, b in enumerate(data))
    return opcode, data


# ═══════════════════════════════════════════════════════════════════════════
# 연결
# ═══════════════════════════════════════════════════════════════════════════
class Conn:
    """WebSocket 연결 하나. 송신은 락으로 직렬화한다.

    role 은 업그레이드 때 제시한 토큰으로 정해지고 연결이 사는 동안 바뀌지
    않는다. 승격하려면 다른 토큰으로 다시 붙어야 한다 — 세션 중 권한이 오르는
    경로를 아예 두지 않는 편이 검증하기 쉽다.

    명령을 실제로 막는 것은 이 파일이 아니라 control_agent 다. 여기서는
    역할을 붙이기만 한다 (판정은 protocol.authorize).
    """

    # 기본값이 최소 권한인 이유: 역할을 넘기는 것을 잊은 호출 경로가 생기면
    # 그 연결은 조용히 전권을 갖게 된다. 잊으면 닫히는 쪽이 기본이어야 한다.
    # (개방 모드에서는 _role_for 가 admin 을 **명시적으로** 넘긴다)
    def __init__(self, sock, addr, role=P.ROLE_FALLBACK):
        self.sock = sock
        self.addr = addr
        self.role = P.normalize_role(role)
        self.lock = threading.Lock()
        self.alive = True
        self.opened = time.time()
        self.sent = 0
        self.dropped = 0

    def send_text(self, s: str) -> bool:
        if not self.alive:
            return False
        try:
            with self.lock:
                self.sock.sendall(encode_frame(s.encode("utf-8"), OP_TEXT))
            self.sent += 1
            return True
        except Exception:
            self.alive = False
            return False

    def send_json(self, obj) -> bool:
        return self.send_text(json.dumps(obj, separators=(",", ":")))

    def ping(self):
        try:
            with self.lock:
                self.sock.sendall(encode_frame(b"", OP_PING))
        except Exception:
            self.alive = False

    def close(self):
        self.alive = False
        try:
            with self.lock:
                self.sock.sendall(encode_frame(b"", OP_CLOSE))
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════════════════
# 토큰 → 역할
# ═══════════════════════════════════════════════════════════════════════════
def _token_roles(auth_token):
    """auth_token 파라미터를 ({토큰: 역할}, 경고목록, 잠금여부) 로 정규화한다.

        None / ""          → ({}, [], False)   개방 모드 (인증 없음)
        "abc..."           → ({"abc...": admin}, [], False)
        {"abc":"operator"} → 그대로 (역할 이름은 protocol 이 검증)
        {}  또는 전부 무효 → ({}, [경고], True)  **전면 거부**

    **단일 문자열이 admin 인 이유는 하위호환이다.** 지금까지 그 토큰 하나가
    전권이었고, 이미 배포된 launch 파일·스크립트·북마크 URL 이 전부 그 토큰을
    쓴다. 여기서 조용히 권한을 낮추면 현장에서 비상정지 해제가 안 되는 형태로
    터진다 — 권한 축소는 설정을 dict 로 바꾸는 **명시적 행동**이어야 한다.

    역할 이름이 이상하면 예외를 던지지 않고 observer 로 떨군다. 관제 서버가
    설정 오타 때문에 아예 안 뜨는 것보다, 뜨되 아무 명령도 안 먹는 쪽이 낫다
    (붙어서 로그를 볼 수는 있어야 원인을 찾는다). 다만 **조용히** 떨구지는
    않는다 — 경고 목록으로 돌려 준다.

    잠금(세 번째 값)이 있는 이유. 예전에는 사전의 항목이 전부 걸러져 {} 가
    되면 `not self.token_roles` 가 참이 되어 **개방 모드**로 열렸다. 즉 토큰을
    걸려고 쓴 설정이 오타 하나로 문을 활짝 여는 방향으로 실패했다. 사전을
    준 것 자체가 "인증을 걸겠다"는 의사표시이므로, 쓸 수 있는 토큰이 0개면
    개방이 아니라 전면 거부다. 설정 실수로 문이 열리는 것보다 안 열리는 편이
    낫다 — 안 열리면 즉시 알아채고 고치지만, 열린 것은 아무도 모른다.
    """
    if auth_token is None or auth_token == "":
        return {}, [], False
    if isinstance(auth_token, str):
        return {auth_token: P.ROLE_ADMIN}, [], False
    if isinstance(auth_token, dict):
        out, notes = {}, []
        for tok, role in auth_token.items():
            if not isinstance(tok, str) or not tok:
                notes.append(f"토큰 항목을 버렸다 — 열쇠가 비어 있거나 "
                             f"문자열이 아니다: {type(tok).__name__}")
                continue
            r = P.normalize_role(role)
            if not P.is_role(role):
                # 토큰 값은 절대 로그에 싣지 않는다. 역할만 말해도 고칠 수 있다.
                notes.append(f"토큰 하나의 역할 {role!r} 을 알 수 없어 "
                             f"{r} 로 떨궜다 (쓸 수 있는 이름: "
                             f"{', '.join(P.ROLES)})")
            out[tok] = r
        if not out:
            notes.append("유효한 토큰이 하나도 없다 — 모든 접속을 거부한다 "
                         "(개방 모드로 열지 않는다)")
            return {}, notes, True
        return out, notes, False
    # 예외를 던지면 노드가 아예 안 뜬다. 그러면 로그를 볼 수도 없으니
    # 원인을 못 찾는다. 뜨되 문은 닫아 두고 사유를 남긴다.
    return {}, [f"auth_token 이 문자열도 사전도 아니다 "
                f"({type(auth_token).__name__}) — 모든 접속을 거부한다"], True


# ═══════════════════════════════════════════════════════════════════════════
# 서버
# ═══════════════════════════════════════════════════════════════════════════
class ControlServer:
    """정적 대시보드(HTTP) + 텔레메트리/명령(WebSocket) 을 한 포트에서 낸다.

    on_message(conn, text) 는 관제에서 온 메시지마다 호출된다. 워커 스레드에서
    불리므로, 호출 측에서 ROS 노드 상태를 만질 때 락을 잡아야 한다.

    conn.role 에 인증 때 정해진 역할이 들어 있다(observer/operator/admin).
    이 서버는 역할을 **붙이기만** 하고 명령을 막지는 않는다 — 판정은
    protocol.authorize(conn.role, 명령), 거부는 on_message 쪽(control_agent)
    담당이다. 전송 계층이 인가 정책을 알면 MQTT 로 갈아탈 때 정책이 따라
    흩어진다.
    """

    def __init__(self, static_dir, port=8080, host="0.0.0.0", ws_path="/ws",
                 on_message=None, on_open=None, on_close=None, logger=None,
                 auth_token=None, tls_cert=None, tls_key=None):
        self.static_dir = os.path.abspath(static_dir)
        self.port, self.host, self.ws_path = port, host, ws_path
        self.on_message = on_message
        self.on_open = on_open
        self.on_close = on_close
        self.log = logger or (lambda m: None)
        # 인증: 브라우저 WebSocket API 는 커스텀 헤더를 못 붙이므로 질의문자열로
        # 받는다 (?token=...). 헤더(Authorization: Bearer)도 함께 받아 도구·스크립트
        # 접속을 지원한다. 비교는 hmac.compare_digest 로 — 타이밍 공격 방지.
        #
        # auth_token 은 두 형태를 받는다 (아래 _token_roles 참조).
        #   "abc..."                    단일 토큰 — 그 토큰이 admin (하위호환)
        #   {"abc":"admin","def":"observer"}   토큰별 역할
        # 원래 값을 그대로 들고 있는 이유는 기존 호출부·로그가 참조하기 때문이다.
        self.auth_token = auth_token or None
        # auth_locked = 인증을 걸겠다고 했는데 쓸 수 있는 토큰이 0개인 상태.
        # 개방 모드(token_roles 가 비었고 잠금도 아님)와 반드시 구분해야 한다 —
        # 둘 다 사전은 비어 있지만 하나는 전원 admin, 하나는 전원 거부다.
        self.token_roles, self.auth_notes, self.auth_locked = _token_roles(auth_token)
        self.tls_cert, self.tls_key = tls_cert, tls_key
        self.conns = []
        self.clock = threading.Lock()
        self.rejected = 0
        self._httpd = None
        self._thread = None

    @staticmethod
    def _presented(path, headers):
        """요청에서 토큰 문자열을 뽑는다. 없으면 None."""
        q = urllib.parse.urlsplit(path).query
        vals = urllib.parse.parse_qs(q).get("token")
        if vals:
            return vals[0]
        hdr = headers.get("Authorization", "")
        if hdr.startswith("Bearer "):
            return hdr[7:].strip()
        return None

    def _role_for(self, path, headers):
        """(허용 여부, 역할) 을 돌려준다. 거부면 (False, None).

        토큰이 여러 개여도 **중간에 빠져나오지 않는다.** compare_digest 가
        한 건의 비교 시간을 감춰도, 몇 번째에서 맞았는지가 응답 시간으로
        새면 토큰을 하나씩 확인해 볼 수 있게 된다.
        """
        if self.auth_locked:
            # 설정이 깨졌다. 여기서 개방 모드로 넘어가면 "토큰을 걸었다고
            # 믿는 채로 문이 열려 있는" 최악의 상태가 된다.
            return False, None
        if not self.token_roles:
            # 개방 모드(토큰 미설정). 여기서 권한을 낮추면 개발·검증 흐름이
            # 통째로 막힌다. 토큰을 안 걸었다는 것은 신뢰 경계를 이 서버 밖
            # (사내망·VPN)에 뒀다는 뜻이므로 붙은 사람을 admin 으로 본다.
            # 이 상태의 위험은 control_agent 가 기동 로그로 경고한다.
            return True, P.ROLE_ADMIN
        tok = self._presented(path, headers)
        if tok is None:
            return False, None
        # bytes 로 맞춰 비교한다 — compare_digest 는 비ASCII str 을 받으면
        # TypeError 를 던지고, 그 예외가 인증 경로를 500 으로 만든다.
        try:
            given = tok.encode("utf-8")
        except Exception:
            return False, None
        role = None
        for known, r in self.token_roles.items():
            if hmac.compare_digest(given, known.encode("utf-8")):
                role = r
        return (role is not None), role

    def _authorized(self, path, headers):
        """예전 시그니처 유지 — 불리언만 필요한 호출부용."""
        return self._role_for(path, headers)[0]

    def _roles_note(self) -> str:
        """기동 로그에 실을 역할 구성 요약.

        토큰 값은 절대 찍지 않는다(로그가 자격증명 저장소가 되면 안 된다).
        역할별 개수만 보여도 "observer 로 줄 걸 admin 으로 줬다" 같은 설정
        실수는 눈에 띈다. 개방 모드면 붙는 사람이 전부 admin 임을 못 박는다.
        """
        if self.auth_locked:
            return " (잠금 — 유효한 토큰이 0개라 모든 접속을 거부한다)"
        if not self.token_roles:
            return " (개방 모드 — 접속자 전원 admin)"
        n = {}
        for r in self.token_roles.values():
            n[r] = n.get(r, 0) + 1
        order = [r for r in P.ROLES if r in n]
        return " · 토큰 " + ", ".join(f"{r} {n[r]}개" for r in order)

    # ── 브로드캐스트 ────────────────────────────────────────────────────────
    def broadcast(self, obj):
        """살아 있는 모든 관제 화면에 보낸다. 끊긴 연결은 걷어낸다.

        직렬화 실패로 노드가 죽으면 안 된다 — 관제가 죽는 것보다 그 한 프레임을
        버리는 게 낫다 (2026-07-26: numpy int32 가 섞여 들어와 노드가 종료됨).
        """
        try:
            text = json.dumps(obj, separators=(",", ":"))
        except TypeError as e:
            self.log(f"직렬화 실패 — 프레임 폐기: {e}")
            return
        dead = []
        with self.clock:
            conns = list(self.conns)
        for c in conns:
            if not c.send_text(text):
                dead.append(c)
        if dead:
            with self.clock:
                for c in dead:
                    if c in self.conns:
                        self.conns.remove(c)
            for c in dead:
                if self.on_close:
                    self.on_close(c)

    def client_count(self):
        with self.clock:
            return sum(1 for c in self.conns if c.alive)

    # ── 기동 ────────────────────────────────────────────────────────────────
    def start(self):
        server = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, fmt, *a):
                pass                        # 기본 stderr 로깅은 시끄럽다

            def do_GET(self):
                if self.path.split("?")[0] == server.ws_path:
                    return self._upgrade()
                return self._static()

            # 대시보드(HTML)는 인증 없이 낸다 — 토큰을 입력할 화면 자체가 필요하다.
            # 실제 제어 권한은 WebSocket 업그레이드에서만 검사한다.

            # 정적 파일
            def _static(self):
                rel = self.path.split("?")[0].lstrip("/") or "index.html"
                path = os.path.normpath(os.path.join(server.static_dir, rel))
                if not path.startswith(server.static_dir) or not os.path.isfile(path):
                    self.send_error(404)
                    return
                with open(path, "rb") as f:
                    body = f.read()
                ext = os.path.splitext(path)[1].lower()
                self.send_response(200)
                self.send_header("Content-Type", MIME.get(ext, "application/octet-stream"))
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            # WebSocket 업그레이드
            def _upgrade(self):
                key = self.headers.get("Sec-WebSocket-Key")
                if not key or "websocket" not in (
                        self.headers.get("Upgrade", "").lower()):
                    # 사유(reason phrase)는 상태줄에 들어가고 latin-1 로만
                    # 인코딩된다 — 한글을 넣으면 UnicodeEncodeError 가 나서
                    # 응답이 통째로 안 나가고 연결이 그냥 끊긴다. 클라이언트는
                    # 그것을 '서버 죽음'과 구분할 수 없다. 한글은 본문(explain,
                    # UTF-8)으로 보낸다.
                    self.send_error(400, "Bad Request", "WebSocket 업그레이드 아님")
                    return
                ok, role = server._role_for(self.path, self.headers)
                if not ok:
                    with server.clock:
                        server.rejected += 1
                    server.log(f"인증 거부: {self.client_address[0]} "
                               f"(누적 {server.rejected})")
                    # 401 이 실제로 나가야 한다. 응답 없이 끊기면 화면에서
                    # '토큰이 틀렸다'와 '로봇이 죽었다'가 똑같아 보인다.
                    self.send_error(401, "Unauthorized", "인증 토큰 불일치")
                    return
                accept = base64.b64encode(
                    hashlib.sha1((key + GUID).encode()).digest()).decode()
                self.send_response(101, "Switching Protocols")
                self.send_header("Upgrade", "websocket")
                self.send_header("Connection", "Upgrade")
                self.send_header("Sec-WebSocket-Accept", accept)
                self.end_headers()
                try:
                    self.wfile.flush()
                except Exception:
                    return

                sock = self.connection
                sock.settimeout(None)
                conn = Conn(sock, self.client_address, role=role)
                with server.clock:
                    server.conns.append(conn)
                # 역할을 접속 로그에 남긴다 — 나중에 "누가 무엇을 했나"를
                # 되짚을 때 명령 이벤트만으로는 부족하다.
                server.log(f"관제 접속: {conn.addr[0]}:{conn.addr[1]} "
                           f"[{conn.role}] (총 {server.client_count()})")
                if server.on_open:
                    server.on_open(conn)
                try:
                    while conn.alive:
                        op, data = decode_frame(sock)
                        if op == OP_CLOSE:
                            break
                        if op == OP_PING:
                            with conn.lock:
                                sock.sendall(encode_frame(data, OP_PONG))
                            continue
                        if op == OP_PONG:
                            continue
                        if op == OP_TEXT and server.on_message:
                            try:
                                server.on_message(conn, data.decode("utf-8"))
                            except Exception as e:
                                server.log(f"메시지 처리 실패: {e}")
                except Exception:
                    pass
                finally:
                    conn.alive = False
                    with server.clock:
                        if conn in server.conns:
                            server.conns.remove(conn)
                    server.log(f"관제 해제: {conn.addr[0]}:{conn.addr[1]} "
                               f"(총 {server.client_count()})")
                    if server.on_close:
                        server.on_close(conn)
                    try:
                        sock.close()
                    except Exception:
                        pass

        class TS(ThreadingHTTPServer):
            daemon_threads = True
            allow_reuse_address = True

        self._httpd = TS((self.host, self.port), Handler)
        if self.tls_cert and self.tls_key:
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.load_cert_chain(self.tls_cert, self.tls_key)
            ctx.minimum_version = ssl.TLSVersion.TLSv1_2
            self._httpd.socket = ctx.wrap_socket(self._httpd.socket, server_side=True)
        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        kwargs=dict(poll_interval=0.2), daemon=True)
        self._thread.start()
        ips = _local_ips()
        sch = "https" if (self.tls_cert and self.tls_key) else "http"
        self.log(f"관제 서버 시작 — {sch}://{ips[0]}:{self.port}/  "
                 f"(WebSocket {self.ws_path})")
        if len(ips) > 1:
            self.log(f"  다른 PC 에서: " + ", ".join(f"{sch}://{i}:{self.port}/"
                                                     for i in ips[1:]))
        # 설정 실수는 기동 로그에서 반드시 보여야 한다. 조용히 걸러 두면
        # "토큰을 걸었는데 왜 아무도 못 붙지" 를 현장에서 헤매게 된다.
        for note in self.auth_notes:
            self.log(f"  auth_token 경고: {note}")
        if self.auth_locked:
            auth = "잠금(설정 오류 — 전면 거부)"
        elif self.token_roles:
            auth = "토큰 필요"
        else:
            auth = "없음(개방)"
        self.log(f"  전송 {'TLS' if sch == 'https' else '평문(TLS 미설정)'} · "
                 f"인증 {auth}{self._roles_note()}")
        return self

    def stop(self):
        for c in list(self.conns):
            c.close()
        if self._httpd:
            self._httpd.shutdown()
            self._httpd.server_close()


def _local_ips():
    """접속 안내에 쓸 IP 목록. 별도 PC 에서 붙을 주소를 바로 알려주기 위한 것."""
    out = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))          # 실제로 보내지 않는다 — 경로만 조회
        out.append(s.getsockname()[0])
        s.close()
    except Exception:
        pass
    out.append("127.0.0.1")
    return out
