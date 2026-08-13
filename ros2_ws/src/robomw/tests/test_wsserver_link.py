"""관제 링크 회귀 — 느린 관제가 로봇의 **수신**을 멈추면 안 된다.

2026-08-13 실기 회귀. 정찰 임무 중 관제 어댑터 링크가 2~4초 주기로 플랩했다.

  서버(fleet_server): sent 1011 (internal error) keepalive ping timeout
  로봇(control_agent): 접속이 살아 있는 동안 link_lost — 1.5초간 수신 트래픽 없음

원인. Conn.send_text 가 conn.lock 을 쥔 채 **시한 없는** sendall 을 했고,
수신 루프는 WS PING 에 답하려고 **같은 락**을 잡았다. 관제가 잠깐 못 읽으면
(asyncio 루프 정체) 커널 송신 버퍼가 차고 브로드캐스트가 락을 쥔 채 멈춘다.
그러면 그 연결의 수신 스레드는 PING 을 받은 자리에서 같이 멈춰 뒤따르는
명령·하트비트를 못 읽는다 → note_client 가 끊겨 로봇이 **살아 있는 링크**를
두절로 오판하고, PONG 도 못 나가 클라이언트 keepalive 가 터진다.

지문: **라이브러리 keepalive(PING)를 쓰는 클라이언트만** 죽었다. TEXT 경로는
락을 잡지 않아 단순 클라이언트는 멀쩡했다. 그 비대칭을 대조군으로 함께 고정한다.
"""
from __future__ import annotations

import base64
import os
import socket
import threading
import time

import pytest

from robomw.link import protocol as P
from robomw.link import wsserver
from robomw.link.wsserver import (OP_PING, OP_TEXT, ControlServer,
                                  encode_frame)

BIG = "x" * 60000            # 실기 map 프레임 급(58 KB) — 소켓 버퍼를 채우는 데 쓴다
HEARTBEAT = '{"v":1,"topic":"orchard/x/cmd","payload":{"cmd":"ping"}}'


def _handshake(sock, port):
    key = base64.b64encode(os.urandom(16)).decode()
    sock.sendall((f"GET /ws HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\n"
                  "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                  f"Sec-WebSocket-Key: {key}\r\n"
                  "Sec-WebSocket-Version: 13\r\n\r\n").encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        buf += sock.recv(4096)


def _slow_client(srv):
    """붙되 **읽지 않는** 관제. 양쪽 커널 버퍼를 줄여 놓는다.

    관제의 asyncio 루프가 정체돼 소켓을 안 읽는 상황을 **결정적으로** 만들기
    위한 것이다. 기본 버퍼(루프백 합계 수 MB)를 채우려면 수 MB 를 밀어야 해서
    시험이 느리고 머신 상태를 탄다. 양쪽을 몇 KB 로 줄이면 60 KB 프레임 하나로
    로봇의 송신이 막힌다 — 재현하려는 조건(송신 블록)은 똑같고 도달만 빠르다.

    돌려주는 값: (클라이언트 소켓, 서버쪽 Conn)
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2048)
    sock.settimeout(5)
    sock.connect(("127.0.0.1", srv.port))
    _handshake(sock, srv.port)
    for _ in range(200):
        if srv.conns:
            break
        time.sleep(0.02)
    assert srv.conns, "연결이 서버에 등록되지 않았다"
    conn = srv.conns[0]
    conn.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2048)
    return sock, conn


@pytest.fixture()
def server(tmp_path):
    seen: list[float] = []
    srv = ControlServer(static_dir=str(tmp_path), port=0,
                        on_message=lambda c, t: seen.append(time.monotonic()),
                        logger=lambda m: None)
    srv.start()
    srv.port = srv._httpd.server_address[1]
    try:
        yield srv, seen
    finally:
        # 정리도 시한을 둔다. 결함이 살아 있으면 conn.close() 가 송신 락을
        # 무기한 기다려 **시험이 끝나지 않는다** — 실패는 매달리는 것이 아니라
        # 떨어지는 것이어야 원인을 본다.
        done = threading.Thread(target=srv.stop, daemon=True)
        done.start()
        done.join(timeout=10)


def _receive_gap_while_send_blocked(srv, seen, *, use_ping, stall_s=3.0):
    """송신이 막혀 있는 동안 하트비트가 계속 처리되는가를 잰다.

    실기 어댑터가 정확히 이 모양이다: asyncio 루프가 정체되면 소켓을 안
    읽지만, 하트비트 태스크와 라이브러리 keepalive PING 은 계속 나간다.

    돌려주는 값: (하트비트 처리 최대 공백[초], 처리 건수).
    공백이 LINK_LOSS_STOP_MS 를 넘으면 실기에서 link_lost 가 뜬다.
    """
    sock, _conn = _slow_client(srv)
    time.sleep(0.2)

    stop = threading.Event()

    def broadcaster():                      # ROS 텔레메트리 타이머 역할 (5 Hz)
        while not stop.is_set():
            srv.broadcast({"topic": "orchard/x/map", "payload": BIG})
            time.sleep(0.2)

    bt = threading.Thread(target=broadcaster, daemon=True)
    bt.start()
    time.sleep(0.6)                         # 송신이 확실히 막히도록 둔다

    del seen[:]
    t0 = time.monotonic()
    end = t0 + stall_s
    while time.monotonic() < end:
        if use_ping:
            sock.sendall(encode_frame(b"", OP_PING, mask=True))
        sock.sendall(encode_frame(HEARTBEAT.encode(), OP_TEXT, mask=True))
        time.sleep(0.5)
    time.sleep(0.5)
    # 측정 창을 **여기서 닫는다.** 정리(stop/join)는 결함이 있을 때 몇 초씩
    # 걸리는데, 그 시간이 공백에 섞이면 대조군까지 실패해 신호가 가려진다.
    t_end = time.monotonic()
    got = sorted(t for t in seen if t0 <= t <= t_end)

    stop.set()
    bt.join(timeout=5)
    sock.close()

    marks = [t0] + got + [t_end]
    return max(b - a for a, b in zip(marks, marks[1:])), len(got)


@pytest.fixture()
def patient_send(monkeypatch):
    """송신 시한을 넉넉히 — 측정 창 동안 링크가 끊기지 않게.

    이 시험이 보려는 것은 '송신이 막힌 동안 수신이 도는가' 하나다. 시한이
    짧으면 그 사이에 연결이 정리돼 버려 무엇을 쟀는지 흐려진다.
    """
    # raising=False — 결함이 살아 있는(상수가 없는) 옛 코드에서도 시험이
    # **행동**으로 실패해야 한다. AttributeError 로 죽으면 무엇이 틀렸는지 안 보인다.
    monkeypatch.setattr(wsserver, "SEND_TIMEOUT_S", 30.0, raising=False)


def test_slow_client_without_ping_keeps_being_received(server, patient_send):
    """대조군 — PING 을 안 쓰는 단순 클라이언트는 원래도 멀쩡했다."""
    srv, seen = server
    gap, n = _receive_gap_while_send_blocked(srv, seen, use_ping=False)
    assert n > 0, "하트비트가 하나도 처리되지 않았다"
    assert gap * 1000 < P.LINK_LOSS_STOP_MS, f"수신 공백 {gap:.2f}s"


def test_slow_client_with_ping_does_not_stall_receive_loop(server, patient_send):
    """회귀 본체 — 라이브러리 keepalive(PING)를 쓰는 관제.

    고치기 전에는 첫 PING 에서 수신 루프가 송신 락을 무기한 기다리며 멈춰
    하트비트가 **한 건도** 처리되지 않았다.
    """
    srv, seen = server
    gap, n = _receive_gap_while_send_blocked(srv, seen, use_ping=True)
    assert n > 0, "PING 을 쓰자 하트비트가 한 건도 처리되지 않았다 — 수신 루프 정지"
    assert gap * 1000 < P.LINK_LOSS_STOP_MS, (
        f"수신 공백 {gap:.2f}s — 로봇이 살아 있는 링크를 두절로 오판한다")


def test_broadcast_gives_up_on_wedged_client(server, monkeypatch):
    """브로드캐스트는 **반드시 돌아온다.**

    브로드캐스트는 ROS 실행기 스레드에서 돈다. 안 읽는 관제 하나 때문에
    여기서 무기한 막히면 텔레메트리도 제어 틱도 통째로 선다. 시한을 넘긴
    관제는 끊고(재접속하게) 브로드캐스트는 진행해야 한다.
    """
    srv, _ = server
    monkeypatch.setattr(wsserver, "SEND_TIMEOUT_S", 0.5, raising=False)
    sock, conn = _slow_client(srv)
    time.sleep(0.3)

    # 별도 스레드에서 돌린다 — 결함이 살아 있으면 브로드캐스트가 **영영**
    # 돌아오지 않으므로, 본 스레드에서 부르면 시험이 실패하는 대신 매달린다.
    done = threading.Event()

    def push():
        for _ in range(3):
            srv.broadcast({"topic": "orchard/x/map", "payload": BIG})
        done.set()

    th = threading.Thread(target=push, daemon=True)
    t0 = time.monotonic()
    th.start()
    finished = done.wait(timeout=5.0)
    elapsed = time.monotonic() - t0

    assert finished, (f"브로드캐스트가 {elapsed:.1f}초 넘게 안 돌아왔다 — "
                      "안 읽는 관제 하나가 ROS 실행기를 세운다")
    assert not conn.alive, "시한을 넘긴 관제가 아직 살아 있다"
    assert conn not in srv.conns, "죽은 연결이 목록에 남았다"
    assert conn.dropped > 0
    sock.close()
