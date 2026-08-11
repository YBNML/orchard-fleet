"""
기능 레지스트리 — 목록에 적힌 모듈만 불러 붙인다.

파라미터 예:
    features: ["telemetry_state", "telemetry_health", "telemetry_map",
               "teleop", "robomw.profiles.orchard.mission"]

이름은 robomw.features 아래 모듈명이다. 점이 들어간 이름은 완전 경로로 본다 —
현장 프로파일(robomw.profiles.*)이나 사내 다른 패키지의 기능도 그렇게 붙인다.

**기능 하나가 터져도 관제가 죽지 않는다.** 훅마다 예외를 잡아 그 기능만
격리하고 이벤트로 올린다. 관제가 죽는 것이 기능 하나 죽는 것보다 훨씬 나쁘다 —
비상정지를 못 누르게 되기 때문이다.

**의존 관계.** 기능은 `requires` 로 필요한 기능을 선언한다. 레지스트리는 목록
전체를 먼저 임포트해 의존 그래프를 그리고 위상 정렬한 순서로 setup 한다.
그래서 setup 안에서 이미 ctx.feature(...) 로 의존 기능을 찾을 수 있다.
필수 의존이 빠졌으면 그 기능(과 그것에 기대던 기능들)을 **적재하지 않고**
사유를 failed 에 남긴다. 관제 자체는 그래도 뜬다 — 기능 하나 없는 것보다
비상정지를 못 누르는 쪽이 나쁘기 때문이다. 순환 의존은 적재 전에 잡아
경로를 찍어 거부한다 (무한 루프 없음).
"""
from __future__ import annotations

import importlib
import inspect
import time

from robomw.core.base import Feature

PKG = "robomw.features"


class _Node:
    """적재 계획 한 칸 — 목록에 적힌 이름, 찾은 클래스, 의존 간선."""

    __slots__ = ("listed", "cls", "hard", "soft", "dead")

    def __init__(self, listed, cls):
        self.listed = listed
        self.cls = cls
        self.hard = set()       # 필수 의존 노드 번호 (requires)
        self.soft = set()       # 선택 의존 노드 번호 (optional_requires)
        self.dead = None        # 탈락 사유. None 이면 살아 있다


def _names(v):
    """의존 선언을 이름 목록으로 정규화한다.

    `requires = "drive_mission"` 처럼 튜플 대신 문자열을 적는 실수가 잦다.
    그대로 순회하면 한 글자씩 도는 엉뚱한 사유가 남으므로 여기서 바로잡는다.
    """
    if not v:
        return ()
    if isinstance(v, str):
        return (v,)
    return tuple(str(x) for x in v)


def _find_cycle(remain, wait):
    """남은 노드들 안에서 실제 순환 하나를 찾아 [a, b, …, a] 로 돌려준다.

    사람이 읽을 메시지를 만들려는 것이다. "순환이 있다"만으로는 어디를 고쳐야
    할지 알 수 없다. 방문 중(0)인 노드로 되돌아오면 그 지점부터가 순환이다.
    """
    state = {}
    stack = []

    def walk(i):
        state[i] = 0
        stack.append(i)
        for j in sorted(wait[i] & remain):
            if state.get(j) == 0:
                return stack[stack.index(j):] + [j]
            if j not in state:
                got = walk(j)
                if got:
                    return got
        stack.pop()
        state[i] = 1
        return None

    for i in sorted(remain):
        if i not in state:
            got = walk(i)
            if got:
                return got
    return sorted(remain)


class Registry:

    def __init__(self, ctx, on_event=None):
        self.ctx = ctx
        self._on_event = on_event or (lambda *a, **k: None)
        self.features = []          # [(Feature 인스턴스, 마지막 속도요청, 시각)]
        self.failed = {}            # 이름 → 사유
        self._vel = {}              # 기능 이름 → (VelocityRequest, 시각)
        self._alias = {}            # 목록에 적힌 이름 → 실제 기능 name
        # 기능끼리 서로를 찾을 수 있게 조회 창구를 Context 에 꽂는다. 코어
        # 호스트는 이 배선을 몰라도 되게 여기서 스스로 한다.
        bind = getattr(ctx, "bind_features", None)
        if callable(bind):
            bind(self.get)

    # ── 로딩 ────────────────────────────────────────────────────────────────
    def load(self, names):
        """목록을 적재한다. 임포트 → 의존 정렬 → setup 의 세 단계.

        단계를 나눈 이유: requires 는 클래스 속성이라 임포트하기 전에는 알 수
        없다. 그래서 전부 임포트해 놓고 그래프를 그린 뒤에야 순서를 정할 수
        있다. 의존이 하나도 없으면 순서는 목록 그대로다 (기존 동작 보존).
        """
        pend = []
        for nm in names:
            nm = str(nm).strip()
            if not nm:
                continue
            cls = self._resolve(nm)
            if cls is not None:
                pend.append(_Node(nm, cls))

        self._link(pend)
        order = self._plan(pend)

        for i in order:
            nd = pend[i]
            if nd.dead:                 # 앞 노드 실패가 여기까지 번졌다
                continue
            try:
                f = nd.cls()
                f.setup(self.ctx)
            except Exception as e:
                self.failed[nd.listed] = f"setup 실패: {e}"
                self.ctx.warn(f"기능 '{nd.listed}' setup 실패 — 건너뜀: {e}")
                nd.dead = f"setup 실패: {e}"
                # setup 이 깨진 기능에 기대던 기능도 함께 뺀다. 반쪽짜리
                # 의존을 안고 도는 것이 안 뜨는 것보다 위험하다.
                self._cascade(pend, i, f"의존 기능 '{nd.cls.name}' setup 실패")
                continue
            self._alias[nd.listed] = f.name
            self.features.append(f)
            self.ctx.log(f"기능 적재 — {f.name} v{f.version}"
                         + (f" · 명령 {list(f.commands)}" if f.commands else "")
                         + (f" · 토픽 {list(f.topics)}" if f.topics else "")
                         + (f" · 의존 {list(f.requires)}" if f.requires else ""))
        return self

    def _resolve(self, nm):
        """모듈을 임포트하고 Feature 하위 클래스를 찾는다. 실패하면 None."""
        try:
            mod = importlib.import_module(nm if "." in nm else f"{PKG}.{nm}")
        except Exception as e:
            self.failed[nm] = f"임포트 실패: {e}"
            self.ctx.warn(f"기능 '{nm}' 임포트 실패 — 건너뜀: {e}")
            return None
        for _, obj in inspect.getmembers(mod, inspect.isclass):
            if issubclass(obj, Feature) and obj is not Feature \
                    and obj.__module__ == mod.__name__:
                return obj
        self.failed[nm] = "Feature 하위 클래스를 찾지 못함"
        self.ctx.warn(f"기능 '{nm}' — Feature 하위 클래스가 없다")
        return None

    def _link(self, pend):
        """requires 선언을 노드 번호 간선으로 바꾼다. 못 찾으면 탈락시킨다."""
        index = {}
        for i, nd in enumerate(pend):
            index.setdefault(nd.cls.name, i)
        for i, nd in enumerate(pend):
            # 목록에는 모듈명을, requires 에는 기능명을 적는 일이 흔하다. 둘이
            # 다를 때를 위해 모듈명(마지막 구간)도 별칭으로 받아준다. 진짜
            # 이름이 항상 우선이라 별칭은 setdefault 로만 넣는다.
            index.setdefault(nd.listed.rsplit(".", 1)[-1], i)

        missing = []
        for i, nd in enumerate(pend):
            for want in _names(getattr(nd.cls, "requires", ())):
                j = index.get(want)
                if j is None:
                    missing.append(
                        (i, f"필수 의존 '{want}' 없음 — 기능 목록에 없거나 "
                            f"그 기능이 적재에 실패했다"))
                elif j == i:
                    missing.append((i, "requires 에 자기 자신이 들어 있다"))
                else:
                    nd.hard.add(j)
            for want in _names(getattr(nd.cls, "optional_requires", ())):
                j = index.get(want)
                if j is not None and j != i:
                    nd.soft.add(j)
        # 간선을 다 이은 뒤에 탈락시킨다 — 그래야 연쇄 탈락이 끝까지 퍼진다.
        for i, why in missing:
            if pend[i].dead is None:
                self._reject(pend, i, why)

    def _plan(self, pend):
        """위상 정렬(칸 알고리즘). 남은 것이 있으면 순환이므로 거부한다.

        고를 수 있는 후보가 여럿이면 **목록에서 앞선 것**을 고른다. 그래서
        의존 선언이 하나도 없으면 결과가 원래 목록 순서와 정확히 같다.
        """
        remain = {i for i, nd in enumerate(pend) if nd.dead is None}
        wait = {i: (pend[i].hard | pend[i].soft) for i in remain}
        order = []
        while remain:
            nxt = next((i for i in sorted(remain)
                        if not (wait[i] & remain)), None)
            if nxt is None:
                break               # 전부 서로를 기다린다 = 순환
            order.append(nxt)
            remain.discard(nxt)

        if remain:
            cyc = _find_cycle(remain, wait)
            path = " → ".join(pend[i].cls.name for i in cyc)
            for i in sorted(remain):
                if pend[i].dead is None:
                    self._reject(
                        pend, i,
                        f"순환 의존 — {path}" if i in cyc
                        else f"순환 의존({path})에 물려 순서를 정할 수 없다")
        return order

    def _reject(self, pend, i, why):
        """노드 하나를 적재 대상에서 뺀다. 사유를 남기고 연쇄까지 처리한다."""
        nd = pend[i]
        nd.dead = why
        self.failed[nd.listed] = why
        self.ctx.warn(f"기능 '{nd.listed}' 적재 안 함 — {why}")
        self._cascade(pend, i)

    def _cascade(self, pend, i, why=None):
        """i 를 **필수로** 기다리던 기능들도 함께 탈락시킨다.

        이미 탈락한 노드는 건드리지 않으므로 서로 물려 있어도 멈춘다.
        선택 의존(soft)만 걸린 기능은 살려 둔다 — 없어도 되는 관계다.
        """
        why = why or f"의존 기능 '{pend[i].cls.name}' 이(가) 빠졌다"
        for k, nk in enumerate(pend):
            if nk.dead is None and i in nk.hard:
                self._reject(pend, k, why)

    def get(self, name):
        """적재된 기능을 이름으로 찾는다. 없으면 None (ctx.feature 의 실체)."""
        name = str(name)
        want = self._alias.get(name, name)
        for f in self.features:
            if f.name == name or f.name == want:
                return f
        return None

    def describe(self):
        out = []
        for f in self.features:
            try:
                out.append(f.describe())
            except Exception:
                out.append(dict(name=f.name, version=f.version))
        return out

    # ── 라우팅 ──────────────────────────────────────────────────────────────
    def dispatch(self, cmd, payload) -> bool:
        """명령을 선언한 기능에게 먼저 주고, 아무도 안 받으면 전체에게 준다."""
        ordered = ([f for f in self.features if cmd in f.commands]
                   + [f for f in self.features if cmd not in f.commands])
        for f in ordered:
            try:
                if f.on_command(cmd, payload):
                    return True
            except Exception as e:
                self._isolate(f, "on_command", e)
        return False

    # ── 주기 훅 ─────────────────────────────────────────────────────────────
    def tick(self, now):
        """모든 기능을 돌리고 속도 요청을 모아 돌려준다."""
        for f in list(self.features):
            try:
                req = f.tick(now)
            except Exception as e:
                self._isolate(f, "tick", e)
                continue
            if req is not None:
                self._vel[f.name] = (req, now)
            else:
                self._vel.pop(f.name, None)
        return list(self._vel.values())

    def telemetry(self, now):
        out = []
        for f in list(self.features):
            try:
                for kind, payload in (f.telemetry(now) or ()):
                    out.append((kind, payload))
            except Exception as e:
                self._isolate(f, "telemetry", e)
        return out

    def teardown(self):
        # 적재의 역순으로 내린다 — 기대는 쪽을 먼저 내려야 내려간 기능을
        # 붙잡고 있는 순간이 생기지 않는다.
        for f in reversed(self.features):
            try:
                f.teardown()
            except Exception:
                pass
        self.features.clear()
        self._alias.clear()

    # ── 격리 ────────────────────────────────────────────────────────────────
    def _isolate(self, f, hook, err):
        """기능 하나의 예외로 관제가 죽지 않게 분리한다. 반복되면 떼어낸다."""
        key = f"{f.name}:{hook}"
        n = self.failed.get(key, 0)
        n = (n if isinstance(n, int) else 0) + 1
        self.failed[key] = n
        if n <= 3:
            self.ctx.warn(f"기능 '{f.name}' {hook} 예외 ({n}/3): {err}")
            self._on_event("feature_error", f"{f.name} {hook} 오류: {err}", "warn")
        if n == 3:
            self.ctx.warn(f"기능 '{f.name}' 반복 실패 — 떼어낸다")
            self._on_event("feature_disabled",
                           f"{f.name} 기능을 반복 실패로 비활성화했다", "warn")
            try:
                f.teardown()
            except Exception:
                pass
            if f in self.features:
                self.features.remove(f)
            self._vel.pop(f.name, None)
