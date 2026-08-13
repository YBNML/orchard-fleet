"""프리셋 3종 (스펙 ③ §3) — 사람이 버튼 하나로 고르는 임무 큐 모양.

**분담 분할의 기본값은 split_k=5 (A=[0..4] / B=[6,7,8]) 다.** 스펙 초안의
split_k=4 는 B=[5,6,7,8] 을 만드는데, 그 목록은 T3 실측에서 물리적으로 완주가
불가능했다: 이 밭의 선회 평지 패드는 부스트로피돈 파리티에 고정돼 있고
(`gen_heightmap.turn_pad_weights` — 쌍 (k,k+1) 은 짝수 k → 북단, 홀수 k → 남단),
계획기의 진행 방향은 통로 번호가 아니라 **요청 목록 안의 순번**에서 나온다
(`up = i % 2 == 0`). 둘은 오름차순 목록의 첫 통로가 짝수일 때만 맞는다.
[5,6,7] 로 낸 임무는 두 횡단이 모두 패드 반대편 램프(최대 91% 경사)로 가
로봇이 코를 박았다 — T3 리포트 §6.3 단차표.

그래서 프리셋은 자기가 만든 통로 목록의 파리티를 스스로 검증하고, 위반이면
거부한다(API 400). 이 규칙은 프리셋 층에만 있다 — REST 로 직접 내는 임무는
T4 그대로 검증하지 않는다(사람이 지형을 알고 내는 예외까지 막지는 않는다).
"""
from __future__ import annotations

from dataclasses import dataclass

from .nodes import Action, Condition, Node, Retry, Sequence

N_ALLEYS = 9                                   # 과수원 통로 0..8 (maps/orchard_v1)


class PresetError(ValueError):
    """프리셋 이름·파라미터·파리티 위반 — API 는 400 으로 번역한다."""


@dataclass
class Plan:
    """인스턴스 1개분 — 주 로봇과 그 로봇이 돌릴 트리."""
    robot_id: str
    tree: Node


def parity_safe(alleys: list[int]) -> bool:
    """선회 평지 패드와 목록 순번이 맞는지 — 연속 종주 목록만 판정한다.

    오름차순(+1 씩)은 첫 통로가 **짝수**일 때, 내림차순(−1 씩)은 첫 통로가
    **홀수**일 때 모든 횡단이 패드 위다(T3 §6.4 의 [7,6,5] 가 후자 —
    게이트를 통과한 실측 사례다). 비연속 목록은 헤드랜드 패드가 잠금 계산에서
    빠지므로(T4 M11) 애초에 발진 가능한 임무가 아니다 — 여기서도 거부한다.
    """
    if not alleys:
        return False
    if len(alleys) == 1:
        return True                            # 횡단이 없다 — 파리티 무관
    steps = {b - a for a, b in zip(alleys, alleys[1:])}
    if steps == {1}:
        return alleys[0] % 2 == 0
    if steps == {-1}:
        return alleys[0] % 2 == 1
    return False


def _check_parity(alleys: list[int], who: str) -> None:
    if not parity_safe(alleys):
        raise PresetError(
            f"{who} 통로 목록 {alleys} 은 선회 패드 파리티에 어긋납니다 "
            f"(오름차순은 첫 통로가 짝수, 내림차순은 홀수여야 합니다 — T3 §6.3)")


def _int(name: str, value) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        raise PresetError(f"{name} 은 정수여야 합니다: {value!r}") from None


def _patrol_tree(robot: str, alleys: list[int]) -> Node:
    """게이트 3종 뒤에 임무 하나 — 로봇이 켜져 있고, 놀고 있고, 통로가 비었을 때만
    발진한다. 조건 불충족은 실패가 아니라 대기다(Condition 의미론)."""
    return Sequence([Condition("robot_online", robot),
                     Condition("robot_idle", robot),
                     Condition("alley_free", list(alleys)),
                     Action({"robot": robot, "alleys": list(alleys)})])


def full_split_patrol(robot_a: str, robot_b: str, split_k: int = 5,
                      n_alleys: int = N_ALLEYS) -> list[Plan]:
    """분담 전체 정찰 — A=[0..split_k-1], B=[split_k+1..n-1] (split_k 는 버퍼로 비움).

    버퍼 통로 하나를 비우는 이유는 AlleyLock 이다: 인접한 두 목록은 사이의
    헤드랜드 패드를 공유해 충돌한다(traffic.pads). 기본 5 는 파리티까지
    맞는 유일한 근처 값이다(4 → B=[5..8] 은 첫 통로가 홀수).
    """
    k = _int("split_k", split_k)
    n = _int("n_alleys", n_alleys)
    if robot_a == robot_b:
        raise PresetError("분담 정찰은 서로 다른 로봇 두 대가 필요합니다")
    if not 0 < k < n - 1:
        raise PresetError(f"split_k 는 1..{n - 2} 범위여야 합니다: {k}")
    a, b = list(range(0, k)), list(range(k + 1, n))
    _check_parity(a, "A")
    _check_parity(b, "B")
    return [Plan(robot_a, _patrol_tree(robot_a, a)),
            Plan(robot_b, _patrol_tree(robot_b, b))]


def sequential_retry(robot: str, alleys: list[int], n: int = 2) -> list[Plan]:
    """통로 목록 하나를 최대 n회 시도 — 실패해도 사람 없이 한 번 더 간다."""
    alleys = [_int("alleys", x) for x in (alleys or [])]
    _check_parity(alleys, "통로")
    tree = Sequence([Retry(_int("n", n), Action({"robot": robot, "alleys": alleys}))])
    return [Plan(robot, tree)]


def single_alley_loop(robot: str, alley: int, n: int = 1) -> list[Plan]:
    """통로 한 줄 — 실기 확인용 최소 임무(브리프: Retry(n, Action([alley])))."""
    a = _int("alley", alley)
    if not 0 <= a < N_ALLEYS:
        raise PresetError(f"통로 번호는 0..{N_ALLEYS - 1} 여야 합니다: {a}")
    return [Plan(robot, Retry(_int("n", n), Action({"robot": robot, "alleys": [a]})))]


PRESETS = {"full_split_patrol": full_split_patrol,
           "sequential_retry": sequential_retry,
           "single_alley_loop": single_alley_loop}


def build(preset: str, params: dict | None = None) -> list[Plan]:
    """프리셋 이름+파라미터 → 인스턴스 계획 목록. 이름·인자 오류는 PresetError."""
    fn = PRESETS.get(preset)
    if fn is None:
        raise PresetError(f"알 수 없는 프리셋: {preset} "
                          f"(가능: {', '.join(sorted(PRESETS))})")
    try:
        return fn(**(params or {}))
    except TypeError as e:                     # 인자 이름·개수 불일치
        raise PresetError(f"프리셋 파라미터가 맞지 않습니다: {e}") from None
