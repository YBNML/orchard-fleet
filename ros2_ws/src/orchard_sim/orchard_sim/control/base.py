"""
관제 기능 확장 계약

기능(촬영·병충해 검출·수확량 추정·구역 순찰 …)은 늘고 바뀌고 없어진다. 그래서
기능을 **코어를 건드리지 않고** 붙이고 뗄 수 있게 나눈다.

    코어 (뺄 수 없음)          기능 (플러그인)
    ─────────────────         ──────────────
    명령 버스                  텔레메트리 생산자 (state / health / map …)
    텔레메트리 싱크            주행 거동 (임무 / 원격조종 / 순찰 …)
    안전 조정자                분석·수집 (촬영 / 검출 …)
    속도 조정자

**안전을 플러그인으로 두지 않은 이유.** "모든 것이 플러그인" 구조면 누군가
설정 한 줄로 비상정지를 끌 수 있다. 비상정지 래치·데드맨·링크두절 정지·전복
감지는 코어에 두고, 기능은 이를 우회할 수단을 갖지 못한다. 기능은 속도를
**요청**할 뿐이고, 최종 출력은 항상 안전 조정자를 통과한다.

기능 하나 추가하는 절차
    1. control/features/ 에 모듈 하나 만들고 Feature 를 상속한다
    2. 파라미터 features 목록에 모듈명을 넣는다
    3. 끝. 코어 파일은 고치지 않는다

기능끼리 필요로 하는 관계는 `requires` 로 **선언**한다 (촬영은 임무 주행이,
병충해 검출은 촬영이 있어야 뜻이 있다). 선언해 두면 레지스트리가 의존 기능을
먼저 setup 하고, 필수 의존이 빠졌으면 그 기능을 아예 적재하지 않는다. 없는
기능을 부르며 조용히 이상 동작하는 것보다 안 뜨고 사유가 남는 편이 낫다.

기능 하나 제거하는 절차
    features 목록에서 뺀다. 대시보드는 hello 의 기능 목록을 보고 해당 패널을
    감추므로 화면도 자동으로 맞춰진다.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass


@dataclass
class VelocityRequest:
    """기능이 내는 속도 **요청**. 최종 출력은 안전 조정자가 정한다.

    priority 가 큰 요청이 이긴다. 같으면 나중에 등록된 기능이 이긴다.
    관례:
        10  원격 조종 (사람이 직접 잡고 있는 것이 우선)
         5  임무 주행
         1  배경 거동 (자율 복귀 등)
    """
    v: float
    w: float
    priority: int = 5
    reason: str = ""


class Blackboard:
    """기능들이 공유하는 상태. 락으로 감싼 속성 보따리.

    센서 수신은 기능마다 중복해서 구독할 필요가 없다. 포즈·기울기·센서 주기처럼
    여러 기능이 함께 보는 값은 여기에 둔다. 기능 고유 상태는 기능 안에 둔다.
    """

    def __init__(self):
        self._lock = threading.RLock()
        self.pose = None            # (x, y, yaw) — map 프레임
        self.tilt_deg = 0.0
        self.lio_pose = None
        self.rates = {}             # 토픽명 → Hz
        self.extra = {}             # 기능이 임의로 쓰는 칸

    @property
    def lock(self):
        return self._lock

    def set(self, **kw):
        with self._lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def snapshot(self, *names):
        with self._lock:
            return {n: getattr(self, n, None) for n in names}


class Context:
    """기능이 코어에 접근하는 유일한 창구.

    기능은 node 를 직접 만지지 말고 되도록 이 창구를 쓴다. 그래야 나중에
    전송을 MQTT 로 바꾸거나 텔레메트리를 샘플링해도 기능을 안 고친다.
    """

    def __init__(self, node, bb, emit, event, params, safety):
        self.node = node
        self.bb = bb
        self._emit = emit
        self._event = event
        self._params = params
        self.safety = safety        # 읽기 전용 조회용 (estop 여부 등)
        self._lookup = None         # 레지스트리가 적재할 때 꽂아준다

    def bind_features(self, lookup) -> None:
        """레지스트리가 자기 조회 함수를 꽂는다.

        코어 호스트(control_agent)는 이 배선을 몰라도 된다 — Context 를 만들
        때는 아직 레지스트리가 없기 때문에, 레지스트리 쪽에서 스스로 꽂는다.
        """
        self._lookup = lookup

    def feature(self, name: str):
        """다른 기능 인스턴스를 이름으로 얻는다. 없으면 None.

        의존 기능의 공개 메서드를 직접 부를 때 쓴다.

        **되도록 블랙보드(bb)를 통한 느슨한 결합을 쓸 것.** 값을 흘려두고 읽는
        쪽은 상대가 없어도 그냥 값이 안 올 뿐이지만, 인스턴스를 직접 잡으면
        상대의 메서드 이름·시그니처에 묶인다. 게다가 반복 실패로 격리되어
        떼어진 기능을 붙잡고 있으면 죽은 객체를 계속 부르게 된다. 그래서
        여기서 얻은 것을 **보관하지 말고** 쓸 때마다 조회하고, None 인 경우를
        항상 감당해야 한다.
        """
        if self._lookup is None:
            return None
        try:
            return self._lookup(name)
        except Exception:
            return None

    def emit(self, topic_kind: str, payload: dict):
        """텔레메트리 발행. topic_kind 는 'state' / 'health' 같은 마지막 구간."""
        self._emit(topic_kind, payload)

    def event(self, kind: str, msg: str, level: str = "info"):
        self._event(kind, msg, level)

    def param(self, name, default=None):
        return self._params(name, default)

    def log(self, msg):
        self.node.get_logger().info(msg)

    def warn(self, msg):
        self.node.get_logger().warn(msg)


class Feature:
    """확장 기능 기반 클래스.

    수명주기: setup() → (tick / telemetry / on_command 반복) → teardown()
    모든 훅은 예외를 던져도 코어가 죽지 않는다 (레지스트리가 잡아 격리한다).
    """

    name = "unnamed"
    version = "1.0"
    summary = ""
    commands: tuple = ()        # 처리하는 명령 이름들 (protocol.CMD_*)
    topics: tuple = ()          # 내는 텔레메트리 종류들

    # 이 기능이 **없으면 안 되는** 다른 기능들의 name. 하나라도 목록에 없으면
    # 레지스트리는 이 기능을 적재하지 않고 사유를 남긴다. 선언해 둔 것은 항상
    # 먼저 setup 되므로, setup 안에서 ctx.feature(...) 로 이미 찾을 수 있다.
    requires: tuple = ()

    # 있으면 쓰고 없으면 마는 의존. 적재 순서에만 영향을 준다(있으면 먼저
    # setup 된다). 없어도 적재되므로 ctx.feature(...) 가 None 인 경우를 기능
    # 스스로 감당해야 한다.
    optional_requires: tuple = ()

    def setup(self, ctx: Context) -> None:
        self.ctx = ctx

    def on_command(self, cmd: str, payload: dict) -> bool:
        """처리했으면 True. False 면 다음 기능에게 넘어간다."""
        return False

    def tick(self, now: float):
        """20 Hz 로 불린다. 속도를 원하면 VelocityRequest 를 돌려준다."""
        return None

    def telemetry(self, now: float):
        """발행할 것이 있으면 [(종류, payload), ...] 를 돌려준다.
        주기 관리는 기능이 스스로 한다 (코어가 강제하지 않는다)."""
        return ()

    def describe(self) -> dict:
        """hello 에 실릴 자기 소개. 대시보드가 이걸 보고 패널을 켜고 끈다.

        의존 관계도 실어 보낸다 — 어떤 패널이 어떤 기능에 얹혀 있는지 화면에서
        바로 보이게 하려는 것이다.
        """
        return dict(name=self.name, version=self.version, summary=self.summary,
                    commands=list(self.commands), topics=list(self.topics),
                    requires=list(self.requires),
                    optional_requires=list(self.optional_requires))

    def teardown(self) -> None:
        pass
