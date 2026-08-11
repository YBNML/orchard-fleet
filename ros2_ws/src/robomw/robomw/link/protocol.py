"""
관제 ↔ 무인이동체 통신 규약

지금은 관제와 시뮬레이터가 같은 PC 에서 돌지만, 현장에서는 별도 PC(그리고 대개
셀룰러 회선 너머)로 갈라진다. 그래서 처음부터 **DDS 를 네트워크 밖으로 끌지 않는
전제**로 설계한다. ROS 2 DDS 는 로봇 안에서만 쓰고, 로봇 밖으로는 이 규약만 나간다.

DDS 를 WAN 으로 늘리지 않는 이유
    · 멀티캐스트 디스커버리가 NAT/방화벽을 못 넘는다
    · 유니캐스트 피어를 손으로 박아도 회선이 끊기면 복구가 지저분하다
    · 기본값에 인증·암호화가 없다 (SROS2 를 붙여도 NAT 문제는 그대로다)

토픽 이름을 MQTT 계층 구조로 잡아둔 이유
    지금 전송은 WebSocket 이지만, 현장 전개에서는 MQTTS 브로커로 갈아탈 가능성이
    높다 (아래 '왜 이 형태인가' 참조). 토픽 문자열과 페이로드를 그대로 두면
    전송 계층만 바꿔 끼울 수 있다. 그래서 payload 에 전송 고유 필드를 넣지 않는다.

왜 이 형태인가 (2026-07 조사)
    · VDA 5050 — 창고 AGV/AMR 플릿 관리의 사실상 표준 인터페이스가 MQTT + JSON 이다.
    · OpenRobOps 1.0 (InOrbit, Apache-2.0, 2026-02) — 수천 대 운용 경험을 오픈소스화한
      운영 계층인데, 로봇 연결을 **아웃바운드 전용 MQTTS 또는 WebSocket** 으로 하고
      "VPN 이 필요 없다"를 전면에 내세운다. 우리가 여기서 하려는 것과 같은 형태다.
    · 아웃바운드 전용이 핵심이다. 로봇이 관제로 **걸어 나가는** 방향이면 로봇 쪽에
      공인 IP 도 포트 개방도 필요 없다. 셀룰러 CGNAT 뒤에서도 그대로 된다.

메시지 봉투
    {"v":1, "topic":"...", "ts":<로봇 시각 ns>, "seq":<n>, "payload":{...}}

    ts 는 로봇 기준 시각이다. 관제는 자기 시각과 비교해 편도 지연을 추정하는 데만
    쓰고, 절대 시각으로 신뢰하지 않는다 (별도 PC 라 시계가 다를 수 있다).
"""
from __future__ import annotations

PROTOCOL_VERSION = 1


def topic(site_id: str = "orchard", robot_id: str = "", kind: str = "") -> str:
    """토픽 문자열 조립: "<site_id>/<robot_id>/<kind>".

    기본 site_id 는 "orchard" — 지금까지는 현장이 하나(과수원)뿐이었어서
    아래 T_* 상수들은 이 함수를 그 기본값으로 호출해 만든다(결과는 이전
    하드코딩 문자열과 바이트 단위로 같다). 현장이 여러 곳이거나 기체 종류가
    달라져도(예: factory7/biped01) 조립 지점은 이 함수 하나로 유지된다.
    kind 자리에 "cmd"/"state"/"teleop" 등을 넣으면 cmd_name() 이 뽑아내는
    마지막 구간과 그대로 맞아떨어진다 — 그 구조를 이 함수가 깨지 않는다.
    """
    return f"{site_id}/{robot_id}/{kind}"


# ── 로봇 → 관제 ─────────────────────────────────────────────────────────────
T_STATE = topic("orchard", "{robot}", "state")      # 5 Hz  포즈·속도·모드·배터리
T_HEALTH = topic("orchard", "{robot}", "health")    # 1 Hz  센서 Hz, SLAM 표류, 경고
T_MISSION = topic("orchard", "{robot}", "mission")  # 변경 시  임무 진행 상황
T_MAP = topic("orchard", "{robot}", "map")          # 저빈도  누적 맵 요약 (점 다운샘플)
T_EVENT = topic("orchard", "{robot}", "event")      # 발생 시  비상정지·전복·통신두절 등
T_HELLO = topic("orchard", "{robot}", "hello")      # 접속 직후 1회  기체 정보·기하

# hello v2 페이로드 키(스펙 ① 확장) — 다현장·다기종 대비. 기존 hello 페이로드
# 필드는 그대로 두고 이 세 키를 additive 로 얹는다.
HELLO_SITE = "site"                    # 이 로봇이 속한 현장 id (topic() 의 site_id 와 같은 값)
HELLO_CAPABILITIES = "capabilities"    # 이 기체가 지원하는 기능군 목록 (CAPABILITY_FAMILIES 의 부분집합)
HELLO_MIDDLEWARE = "middleware"        # 미들웨어 이름·버전 문자열 (진단용)

# capabilities 배열에 담기는 값의 어휘. drive/work/diag 는 지금 실제로 쓴다.
# legged·manipulation 은 예약이다 — 다리형 기체·매니퓰레이터 지원은 아직
# 없고 이 프로젝트 범위 밖이다. 미리 이름을 박아두는 이유는 hello 를 보내는
# 쪽·받는 쪽이 나중에 같은 어휘를 쓰게 하기 위해서다(오타로 갈라지는 것을 막는다).
CAPABILITY_FAMILIES = ("drive", "work", "diag", "legged", "manipulation")

# ── 관제 → 로봇 ─────────────────────────────────────────────────────────────
T_CMD = topic("orchard", "{robot}", "cmd")          # 명령 (아래 CMD_*)
T_TELEOP = topic("orchard", "{robot}", "teleop")    # 수동 조종 (데드맨 — 아래 참조)

CMD_ESTOP = "estop"                        # 즉시 정지 + 래치
# 래치 해제는 2단계다 — 관제 승인과 현장 확인이 모두 있어야 풀린다.
# 근거: ISO 13849-1 §5.2.2(리셋은 위험구역 밖 + 시야 확보 위치에서, 시야가
# 불완전하면 특별 리셋 절차). 관제실은 시야가 없으므로 단독 해제는 없앴다.
CMD_CLEAR_ESTOP_REQUEST = "clear_estop_request"   # ① 관제 승인 (단독으로는 안 풀림)
CMD_CLEAR_ESTOP_CANCEL = "clear_estop_cancel"     # 해제 절차 취소
# ② 현장 확인은 명령이 아니라 로봇의 로컬 입력이다 — 링크로 오지 않는다.
#    (실기: 기체의 물리 리셋 버튼 / 시뮬: ~/local_reset 토픽)
#    아래 이름은 '링크로 오면 거부한다'는 것을 명시하기 위해서만 존재한다.
CMD_LOCAL_RESET = "local_reset"
# 정비·시운전 — 사람이 기체 곁에 있는 시간대를 관제가 알아야 한다.
# 사망 사고는 자율주행 중이 아니라 점검·시운전 중에 일어난다.
CMD_SET_SERVICE_MODE = "set_service_mode"
CMD_MISSION_START = "mission_start"        # {"alleys":[0,1,2], "mode":"mapping"}
CMD_MISSION_PAUSE = "mission_pause"
CMD_MISSION_RESUME = "mission_resume"
CMD_MISSION_CANCEL = "mission_cancel"
CMD_SET_MODE = "set_mode"                  # {"mode":"idle"|"mission"|"teleop"}
CMD_PING = "ping"

# v1 확장(스펙 ①) — self_test 결과 보고, GNSS/랜드마크 재정위, 사고 후 원인
# 분석용 블랙박스 덤프, work(작업기) 단독 정지. 넷 다 mission_* 과 별개다 —
# 임무 진행 중이 아니어도(예: 점검 중 self_test) 부를 수 있어야 한다.
CMD_SELF_TEST = "self_test"
CMD_RELOCALIZE = "relocalize"          # 위치 추정을 리셋한다 — 오적용 시 임무 궤적이 깨져 admin 전용
CMD_BLACKBOX_DUMP = "blackbox_dump"
CMD_WORK_STOP = "work_stop"

# ── 역할 기반 권한 ──────────────────────────────────────────────────────────
# 지금까지는 토큰 하나가 곧 전권이었다. 붙기만 하면 누구나 로봇을 몰 수 있다는
# 뜻인데, 현장에는 화면만 보는 사람(농장주·참관자·연구원)과 실제로 모는 사람이
# 섞인다. 그래서 토큰마다 역할을 붙이고, 명령마다 필요한 역할을 여기 표로 둔다.
#
# 역할을 **한 줄로 세운 이유**(observer < operator < admin). 능력을 조합으로
# 주는 방식(capability set)도 생각했지만, 조합이 늘면 "저 사람이 지금 비상정지를
# 걸 수 있나?"를 화면 보고 즉시 판단할 수 없게 된다. 안전 장치가 걸린 계통에서는
# 권한이 눈으로 확인되는 편이 낫다. 세밀함보다 즉답 가능성을 택했다.
ROLE_OBSERVER = "observer"      # 텔레메트리 구독만. 모든 명령 거부
ROLE_OPERATOR = "operator"      # 임무 지시·원격조종·비상정지
ROLE_ADMIN = "admin"            # 위 전부 + 비상정지 해제 + 모드 변경

ROLES = (ROLE_OBSERVER, ROLE_OPERATOR, ROLE_ADMIN)
ROLE_RANK = {ROLE_OBSERVER: 0, ROLE_OPERATOR: 1, ROLE_ADMIN: 2}

# 모르는 역할 이름은 가장 약한 쪽으로 떨군다. 설정 오타가 권한 상승이 되면 안 된다.
ROLE_FALLBACK = ROLE_OBSERVER

# 표에 없는 명령에 필요한 역할. 기능(플러그인)이 새 명령을 들고 와도 표에
# 등록되기 전에는 admin 만 쓸 수 있다 — 권한 표에서 빠진 명령이 조용히
# "아무나 쓸 수 있는 명령"이 되는 쪽이 훨씬 위험하다. 새 기능은
# register_command_role() 로 자기 명령의 필요 역할을 선언하면 된다.
ROLE_REQUIRED_DEFAULT = ROLE_ADMIN

# T_TELEOP 은 cmd 가 아니라 별도 토픽으로 오지만 권한 판정은 같은 표를 쓴다.
# cmd_name(T_TELEOP) 의 결과와 같은 문자열이어서 호출부가 분기할 필요가 없다.
ACT_TELEOP = "teleop"

ROLE_REQUIRED = {
    # ── 비상정지: 거는 것과 푸는 것의 문턱을 일부러 다르게 잡았다 ──────────
    # 위험을 본 사람이 권한 때문에 못 멈추는 것이 최악이다. 그래서 거는 쪽은
    # 가장 낮은 문턱에 둔다.
    #
    # 논점 — observer 도 걸 수 있게 할 것인가.
    #   찬성: 물리적 위험은 역할을 가리지 않는다. 옆에서 보던 사람이 사고를
    #         먼저 본다. 실제 산업 설비의 물리 비상정지 버튼에는 권한이 없다.
    #   반대: 링크 너머의 observer 는 '옆에 있는 사람'이 아니다. 토큰만 있으면
    #         원격에서 임무를 무한정 방해할 수 있고(가용성 공격), 관측 전용
    #         토큰은 화면 공유용으로 헤프게 뿌려지기 쉽다.
    #   결정: 기본은 operator 이상. 사람이 실제로 로봇 옆에 서는 배치라면
    #         이 한 줄을 ROLE_OBSERVER 로 바꾸는 것으로 정책이 바뀌게 해뒀다.
    #         (물리 비상정지 버튼은 이 경로와 무관하게 항상 우선한다)
    CMD_ESTOP: ROLE_OPERATOR,
    # 푸는 것은 거는 것보다 훨씬 위험하다. 멈춘 이유가 사라졌는지 판단할 수
    # 있는 사람만 풀어야 하므로 admin 전용이다. 그리고 이 승인만으로는 절대
    # 안 풀린다 — 현장 확인이 따로 있어야 한다(safety.request_clear 참조).
    # 해제해도 자동 재개는 없다 — 해제와 재개는 별개 결정이다.
    CMD_CLEAR_ESTOP_REQUEST: ROLE_ADMIN,
    CMD_CLEAR_ESTOP_CANCEL: ROLE_ADMIN,

    # ── 임무 ────────────────────────────────────────────────────────────────
    CMD_MISSION_START: ROLE_OPERATOR,
    CMD_MISSION_PAUSE: ROLE_OPERATOR,
    CMD_MISSION_RESUME: ROLE_OPERATOR,
    CMD_MISSION_CANCEL: ROLE_OPERATOR,

    # ── 모드 ────────────────────────────────────────────────────────────────
    # 모드 변경은 로봇의 거동 자체를 바꾼다(임무 ↔ 조종). 운전 중인 다른
    # 사람의 발밑을 빼는 격이라 admin 으로 둔다.
    CMD_SET_MODE: ROLE_ADMIN,
    CMD_SET_SERVICE_MODE: ROLE_ADMIN,

    # ── 조종 ────────────────────────────────────────────────────────────────
    # 원격조종은 데드맨(400 ms)이 걸려 있어 손을 떼면 멎지만, 그렇다고
    # 관측자에게 조종간을 줄 이유는 없다.
    ACT_TELEOP: ROLE_OPERATOR,

    # ── v1 확장 명령 ────────────────────────────────────────────────────────
    # self_test·블랙박스 덤프·work 정지는 조종·임무와 같은 급(operator).
    # relocalize 는 위치 추정 자체를 리셋해 임무 궤적을 깨뜨릴 수 있어
    # admin 전용으로 둔다(모드 변경과 같은 급).
    CMD_SELF_TEST: ROLE_OPERATOR,
    CMD_BLACKBOX_DUMP: ROLE_OPERATOR,
    CMD_WORK_STOP: ROLE_OPERATOR,
    CMD_RELOCALIZE: ROLE_ADMIN,

    # ── 무해한 것 ───────────────────────────────────────────────────────────
    # ping 은 링크 확인용이라 관측자도 쓸 수 있어야 한다. 오히려 관측자가
    # 링크 상태를 확인할 수단이 없으면 '멈춘 화면'과 '멈춘 로봇'을 구분 못 한다.
    CMD_PING: ROLE_OBSERVER,
}

# ── 안전 관련 상수 ──────────────────────────────────────────────────────────
# 데드맨: 원격 조종 명령은 이 시간 안에 갱신되지 않으면 무효가 된다.
# 회선이 끊기면 마지막 속도 명령이 그대로 살아 있는 것이 가장 위험한 실패 모드다.
TELEOP_DEADMAN_MS = 400

# 관제 링크가 이 시간 이상 끊기면 로봇은 스스로 정지한다.
# (임무 자체는 유지한다 — 통신이 돌아오면 재개할 수 있어야 한다)
LINK_LOSS_STOP_MS = 1500

# 로봇이 이 주기로 신호를 보낸다. 관제는 3회 놓치면 '연결 끊김'으로 표시한다.
HEARTBEAT_MS = 1000

MODE_IDLE = "idle"
MODE_MISSION = "mission"
MODE_TELEOP = "teleop"
MODE_ESTOP = "estop"

# ── work(작업기) 스키마 ─────────────────────────────────────────────────────
# mission_start 는 "어디를(alleys) 어떤 모드로(mapping 등)" 다루고, work 는
# "무엇을 하는 기체인가"를 다룬다 — 방제(spray)·예초(mow)·운반(transport) 등
# 부착 작업기가 있는 기체로 확장하려고 additive 로 얹는다. 기존 mission_*
# 명령·페이로드는 이 스키마와 무관하게 그대로 쓴다.
WORK_TYPES = ("scout", "spray", "mow", "transport")


def validate_work(payload) -> tuple:
    """work 시작 페이로드 검사: {"type": WORK_TYPES 중 하나, "params": dict(선택)}.

    (허용 여부, 사유) 를 돌려준다 — authorize() 와 같은 관례다. 실패는
    예외가 아니라 정상적인 거부이므로, 호출부가 거부 사유를 그대로 이벤트에
    실어 보낼 수 있게 문자열로 준다.

    params.speed_scale 은 선택 필드다(없으면 기본 속도). 0.1~1.0 범위로
    닫아 둔 이유: 0 이하는 사실상 정지 명령을 work 스키마로 우회하는 것이고,
    1.0 초과는 정격 속도를 넘어서는 값이라 이 계약 층에서 미리 막는다.
    """
    if not isinstance(payload, dict):
        return False, "payload 가 객체가 아니다"
    t = payload.get("type")
    if t not in WORK_TYPES:
        return False, f"알 수 없는 work 종류: {t!r} (허용: {', '.join(WORK_TYPES)})"
    params = payload.get("params", {})
    if params is None:
        params = {}
    if not isinstance(params, dict):
        return False, "params 가 객체가 아니다"
    if "speed_scale" in params:
        s = params["speed_scale"]
        if isinstance(s, bool) or not isinstance(s, (int, float)):
            return False, "speed_scale 은 숫자여야 한다"
        if not (0.1 <= s <= 1.0):
            return False, f"speed_scale 범위 초과: {s} (허용 0.1~1.0)"
    return True, ""


def envelope(topic_str: str, payload: dict, ts_ns: int, seq: int) -> dict:
    return {"v": PROTOCOL_VERSION, "topic": topic_str, "ts": int(ts_ns),
            "seq": int(seq), "payload": payload}


def parse(msg: dict):
    """(topic, payload, ts, seq) 로 분해. 형식이 아니면 ValueError."""
    if not isinstance(msg, dict):
        raise ValueError("메시지가 객체가 아니다")
    if msg.get("v") != PROTOCOL_VERSION:
        raise ValueError(f"프로토콜 버전 불일치: {msg.get('v')} != {PROTOCOL_VERSION}")
    t = msg.get("topic")
    if not isinstance(t, str):
        raise ValueError("topic 없음")
    p = msg.get("payload")
    if not isinstance(p, dict):
        raise ValueError("payload 없음")
    return t, p, int(msg.get("ts", 0)), int(msg.get("seq", 0))


def cmd_name(topic_str: str) -> str:
    """orchard/<robot>/cmd → 'cmd' 처럼 마지막 구간만 뽑는다."""
    return topic_str.rsplit("/", 1)[-1]


# ── 명령 실행 결과 (cmd_result) ─────────────────────────────────────────────
# 명령은 즉시 끝나지 않는 것이 많다(mission_start 는 임무가 끝나야 결과가
# 나온다). cmd_id 로 요청-결과를 상관시켜 관제가 "그 명령이 어떻게 됐는지"를
# 추적할 수 있게 한다. status·code 모두 닫힌 집합이다 — ROLE_REQUIRED 와
# 같은 이유로, 오타가 그대로 새 프로토콜 값이 되는 것을 막는다(fail-closed).
_CMD_RESULT_STATUSES = ("accepted", "rejected", "in_progress", "completed", "failed")

RESULT_CODES = ("OK", "DENIED", "BAD_PARAM", "BUSY", "ESTOPPED", "UNSUPPORTED",
                 "TIMEOUT", "INTERNAL")

# mission_* 결과 payload(data) 에 실리는 키 어휘. 값의 의미까지 이 계약이
# 정하지는 않는다(그건 mission 기능의 몫) — 어떤 키가 있어야 관제 UI가
# 고정된 필드를 그릴 수 있는지만 여기서 못박는다.
MISSION_REPORT_KEYS = ("alleys_done", "distance_m", "duration_s", "interventions", "coverage")


def make_cmd_result(cmd_id, cmd, status, code="OK", data=None) -> dict:
    """명령 실행 결과를 이벤트 payload 로 만든다(kind="cmd_result").

    status 나 code 가 닫힌 집합 밖이면 ValueError — 여기서 막지 않으면
    잘못된 값이 그대로 관제 화면까지 흘러가 새 상태값처럼 보이게 된다.
    """
    if status not in _CMD_RESULT_STATUSES:
        raise ValueError(f"알 수 없는 cmd_result 상태: {status!r} "
                          f"(허용: {', '.join(_CMD_RESULT_STATUSES)})")
    if code not in RESULT_CODES:
        raise ValueError(f"알 수 없는 결과 코드: {code!r} (허용: {', '.join(RESULT_CODES)})")
    return {"kind": "cmd_result", "cmd_id": cmd_id, "cmd": cmd, "status": status,
            "code": code, "data": data if data is not None else {}}


# ═══════════════════════════════════════════════════════════════════════════
# 권한 판정 — 호출부(control_agent) 인터페이스
# ═══════════════════════════════════════════════════════════════════════════
# 이 모듈은 **판정만** 한다. 연결에 역할을 붙이는 것은 link/wsserver.py 가,
# 명령을 막는 것은 control_agent 가 한다. 셋을 나눈 이유: 전송을 MQTT 로
# 바꿔도 표와 판정은 그대로 쓰고, 정책을 바꿀 때 고칠 곳이 이 파일 하나이기
# 위해서다.
#
# 호출부에서 쓰는 모양 (control_agent._on_ws_message):
#
#     role = getattr(conn, "role", ROLE_FALLBACK)   # 연결에 없으면 최소 권한
#     ok, why = authorize(role, payload.get("cmd"))
#     if not ok:
#         self.event("denied", why, "warn")         # 감사 흔적을 남긴다
#         return
#
# 조종(T_TELEOP)도 같은 함수를 쓴다 — cmd_name() 이 돌려주는 'teleop' 이
# 그대로 표의 열쇠다:
#
#     ok, why = authorize(role, P.cmd_name(t))      # t 가 .../teleop 일 때
#
# **판정은 메시지가 들어오는 길목에서 한다.** control_agent 는 명령을 큐에
# 넣었다가 나중에 소비하지만 조종은 지연 때문에 큐를 건너뛰고 즉시 실행된다.
# 소비 시점에서만 막으면 조종은 그대로 통과한다 — 하필 가장 막아야 할 것이다.
#
# 주의: 거부는 **조용히 버리지 말 것.** 권한이 없어 안 먹은 것인지 링크가
# 끊긴 것인지 화면에서 구분되지 않으면, 운전자가 로봇이 멈춘 줄 알고 접근한다.


def is_role(role) -> bool:
    """아는 역할 이름인가.

    `role in ROLE_RANK` 를 그대로 쓰면 해시 불가 값(list/dict)에 TypeError 가
    난다. 그 예외가 인증 경로 한가운데서 터지면 판정이 아예 건너뛰어지므로,
    타입 검사를 먼저 해서 '모르는 역할'로 조용히 흡수한다.
    """
    return isinstance(role, str) and role in ROLE_RANK


def normalize_role(role) -> str:
    """아는 역할이면 그대로, 아니면 최소 권한으로 떨군다.

    설정 파일 오타·구버전 클라이언트가 권한 상승이 되면 안 된다.
    **이것은 '사람의 권한'을 정할 때만 안전한 방향이다.** '명령의 문턱'을
    정할 때 같은 방향으로 떨구면 오타 하나가 명령을 열어버린다
    (register_command_role 참조 — 거기서는 반대 방향으로 올린다).
    """
    return role if is_role(role) else ROLE_FALLBACK


def role_rank(role) -> int:
    """역할의 서열. 모르는 이름은 최소 권한 서열."""
    return ROLE_RANK[normalize_role(role)]


def role_allows(role, required) -> bool:
    """role 이 required 이상인가."""
    return role_rank(role) >= role_rank(required)


def required_role(action: str) -> str:
    """명령(또는 'teleop')에 필요한 역할. 표에 없으면 admin (fail-closed)."""
    return ROLE_REQUIRED.get(action, ROLE_REQUIRED_DEFAULT)


def authorize(role, action) -> tuple:
    """(허용 여부, 사유) 를 돌려준다. 사유는 그대로 이벤트에 실어도 되는 한글.

    허용일 때 사유는 빈 문자열이다. 판정 실패를 예외로 던지지 않는 이유:
    권한 없는 명령은 '오류'가 아니라 정상적인 거부이고, 예외로 만들면
    호출부가 try 로 감싸다가 실수로 삼키기 쉽다.
    """
    if not isinstance(action, str) or not action:
        return False, "명령 이름이 없다"
    need = required_role(action)
    have = normalize_role(role)
    if role_allows(have, need):
        return True, ""
    known = " (등록되지 않은 명령)" if action not in ROLE_REQUIRED else ""
    return False, (f"권한 부족: '{action}' 은 {need} 이상이 필요하다 "
                   f"(현재 {have}){known}")


# 역할 등록에서 삼킨 설정 실수를 모아 둔다. 이 모듈은 로거를 모르므로
# (전송·프레임워크에 의존하지 않는 것이 이 파일의 목적이다) 호출부가
# take_role_warnings() 로 꺼내 자기 로거로 찍는다. 기능 적재가 끝난 뒤
# control_agent 가 한 번 비운다.
_ROLE_WARNINGS = []
_ROLE_WARNINGS_MAX = 20         # 기능이 반복 호출해도 메모리를 먹지 않게


def _warn_role(msg: str) -> None:
    if len(_ROLE_WARNINGS) < _ROLE_WARNINGS_MAX:
        _ROLE_WARNINGS.append(msg)


def take_role_warnings() -> list:
    """쌓인 역할 등록 경고를 꺼내고 비운다."""
    out = list(_ROLE_WARNINGS)
    _ROLE_WARNINGS.clear()
    return out


def register_command_role(action: str, role: str):
    """기능(플러그인)이 자기 명령의 필요 역할을 선언한다.

    새 기능을 붙일 때 이 파일(코어)을 고치지 않게 하려고 열어둔 문이다.
    features/ 모듈의 setup() 에서 한 번 부르면 된다:

        P.register_command_role("capture_start", P.ROLE_OPERATOR)

    선언하지 않으면 ROLE_REQUIRED_DEFAULT(admin)가 적용된다 — 즉 안 부르면
    권한이 열리는 게 아니라 닫힌다. 이미 등록된 명령의 역할은 덮어쓰지
    않는다. 기능이 코어 정책을(특히 clear_estop 을) 완화할 수 없어야 한다.

    **모르는 역할 이름은 최소 권한이 아니라 기본값(admin)으로 올린다.**
    예전에는 normalize_role 을 그대로 태워서 오타 하나가
        register_command_role("capture_start", "Operator")   # 대문자 O
    를 observer 로 등록했다. 즉 조심하려고 적은 한 줄이 그 명령을 **아무나
    쓸 수 있는 명령**으로 만들었다. 사람의 역할은 모를수록 낮춰야 안전하고
    (normalize_role), 명령의 문턱은 모를수록 높여야 안전하다 — 두 방향이
    반대라서 같은 함수를 쓰면 한쪽이 반드시 틀린다.

    돌려주는 값은 **실제로 등록된 역할**이다. 요청과 다를 수 있으니 확인이
    필요하면 비교하면 된다. 이미 등록돼 있었으면 그 역할을 그대로 돌려준다.
    """
    if not isinstance(action, str) or not action:
        _warn_role(f"명령 이름이 올바르지 않아 역할 등록을 거부했다: {action!r}")
        return None
    if action in ROLE_REQUIRED:
        return ROLE_REQUIRED[action]
    if not is_role(role):
        ROLE_REQUIRED[action] = ROLE_REQUIRED_DEFAULT
        _warn_role(f"명령 '{action}' 의 역할 이름 {role!r} 을 알 수 없다 — "
                   f"{ROLE_REQUIRED_DEFAULT} 로 올려 등록했다 (오타 확인 필요). "
                   f"쓸 수 있는 이름: {', '.join(ROLES)}")
        return ROLE_REQUIRED[action]
    ROLE_REQUIRED[action] = role
    return role
