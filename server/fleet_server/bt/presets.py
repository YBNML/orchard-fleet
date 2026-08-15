"""프리셋 3종 (스펙 ③ §3) — 사람이 버튼 하나로 고르는 임무 큐 모양.

**분담 분할의 기본값은 데이터 주도다(Task 5).** farm 매니페스트가 있으면
최대 폭 통로(자연 버퍼 — 그 열 간격이 유독 넓다는 것은 대개 나무 통로가
아니라 농로·헤드랜드라는 뜻이다)를 기본 분할점으로 쓴다. farm 이 없으면
(레거시 시뮬레이션 orchard_v1, 통로 9개) 기존 상수 split_k=5 를 그대로 쓴다
— 그 밭의 선회 평지 패드는 부스트로피돈 파리티에 고정돼 있고
(`gen_heightmap.turn_pad_weights` — 쌍 (k,k+1) 은 짝수 k → 북단, 홀수 k →
남단), 계획기의 진행 방향은 통로 번호가 아니라 **요청 목록 안의 순번**에서
나온다(`up = i % 2 == 0`). 둘은 오름차순 목록의 첫 통로가 짝수일 때만 맞는다.
split_k=4 의 B=[5,6,7,8] 로 낸 임무는 두 횡단이 모두 패드 반대편 램프(최대
91% 경사)로 가 로봇이 코를 박았다 — T3 리포트 §6.3 단차표.

**terrain=="terraced" 인 farm(또는 farm 이 아예 없는 레거시 호출)만 이
파리티 게이트를 적용한다.** 실사 과수원(terrain=="flat", maps/orchard_real)
에는 선회 평지 패드도 부스트로피돈 파리티도 없다 — 대신 T7 인계가 확정한
두 제약을 검증한다(T7 §인계):
① 최대 폭 통로(자연 버퍼, 실사에선 통로 20 — 폭 15.25m 농로)는 측위 보정이
   전무한 구간이라 어떤 임무 통로 목록에도 들어가면 안 된다.
② 임무 통로 목록의 연속 전이는 인접(±1) 또는 한 칸 건너(±2)까지만 허용한다.

프리셋은 자기가 만든 통로 목록을 스스로 검증하고, 위반이면 거부한다(API
400). 이 규칙은 프리셋 층에만 있다 — REST 로 직접 내는 임무는 T4 그대로
검증하지 않는다(사람이 지형을 알고 내는 예외까지 막지는 않는다).

farm 은 항상 서버 쪽(app.state.farm, Task 5 farm_routes)에서만 주입한다 —
클라이언트가 보내는 BT 생성 params 로는 절대 들어오지 않는다.
"""
from __future__ import annotations

from dataclasses import dataclass

from .nodes import Action, Condition, Node, Retry, Sequence

N_ALLEYS = 9                    # farm 이 없을 때의 레거시 기본값(maps/orchard_v1, 통로 0..8)
_LEGACY_SPLIT_K = 5             # farm 이 없을 때(orchard_v1)의 파리티 안전 기본 분할
_LEGACY_TERRAIN = "terraced"    # farm 이 없을 때의 기본 지형(레거시 시뮬레이션은 계단식)


class PresetError(ValueError):
    """프리셋 이름·파라미터·통로 목록 위반 — API 는 400 으로 번역한다."""


@dataclass
class Plan:
    """인스턴스 1개분 — 주 로봇과 그 로봇이 돌릴 트리."""
    robot_id: str
    tree: Node


def n_alleys_of(farm: dict | None) -> int:
    """farm.json 의 rows(통로는 rows-1 개) → farm 이 없으면 레거시 상수."""
    if farm is None:
        return N_ALLEYS
    return int(farm["rows"]) - 1


def terrain_of(farm: dict | None) -> str:
    """farm.json 의 terrain → farm 이 없으면 레거시 기본(terraced)."""
    if farm is None:
        return _LEGACY_TERRAIN
    return farm.get("terrain", _LEGACY_TERRAIN)


def alley_widths(farm: dict) -> list[float]:
    """행 원점(row_origins)의 cross-row 성분 차분으로 통로 폭을 구한다.

    farm.json 의 axes_note: 각 열은 원점에서 world +y 로 row_length_m 만큼
    뻗는다 — 즉 cross-row(통로 폭) 축은 world x 다. 연속한 두 열의 origin_x
    차이가 그 사이 통로의 폭이다(실사: 4.75~15.25m, 통로 20 이 최댓값)."""
    origins = farm.get("row_origins") or []
    if len(origins) < 2:
        return []
    return [origins[i + 1][0] - origins[i][0] for i in range(len(origins) - 1)]


def widest_alley(farm: dict | None) -> int | None:
    """최대 폭 통로(자연 버퍼) 인덱스 — farm 이 없거나 폭 정보가 없으면 None."""
    if farm is None:
        return None
    widths = alley_widths(farm)
    if not widths:
        return None
    return max(range(len(widths)), key=widths.__getitem__)


def parity_safe(alleys: list[int]) -> bool:
    """선회 평지 패드와 목록 순번이 맞는지 — 연속 종주 목록만 판정한다(terraced 전용).

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


def flat_terrain_safe(alleys: list[int], farm: dict | None) -> tuple[bool, str]:
    """T7 인계 ①② — flat 지형의 통로 목록 검증. (안전 여부, 위반 사유) 를 낸다."""
    if not alleys:
        return False, "통로 목록이 비었습니다"
    buffer_alley = widest_alley(farm)
    if buffer_alley is not None and buffer_alley in alleys:
        return False, (f"통로 {buffer_alley} 은 최대 폭 통로(자연 버퍼 — 측위 보정 전무 "
                       f"구간)라 임무에 넣을 수 없습니다 (T7 인계 ①)")
    for a, b in zip(alleys, alleys[1:]):
        if abs(b - a) not in (1, 2):
            return False, (f"통로 전이 {a}->{b} 는 인접(±1) 또는 한 칸 건너(±2) 까지만 "
                           f"허용됩니다 (T7 인계 ②)")
    return True, ""


def _check_alleys(alleys: list[int], who: str, farm: dict | None) -> None:
    """terrain 에 따라 파리티(terraced) 또는 T7 제약(flat) 을 검증한다."""
    if terrain_of(farm) == "terraced":
        if not parity_safe(alleys):
            raise PresetError(
                f"{who} 통로 목록 {alleys} 은 선회 패드 파리티에 어긋납니다 "
                f"(오름차순은 첫 통로가 짝수, 내림차순은 홀수여야 합니다 — T3 §6.3)")
    else:
        ok, why = flat_terrain_safe(alleys, farm)
        if not ok:
            raise PresetError(f"{who} 통로 목록 {alleys}: {why}")


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


def full_split_patrol(robot_a: str, robot_b: str, split_k: int | None = None,
                      n_alleys: int | None = None, farm: dict | None = None) -> list[Plan]:
    """분담 전체 정찰 — A=[0..split_k-1], B=[split_k+1..n-1] (split_k 는 버퍼로 비움).

    버퍼 통로 하나를 비우는 이유는 AlleyLock 이다: 인접한 두 목록은 사이의
    헤드랜드 패드를 공유해 충돌한다(traffic.pads). split_k 기본값은 데이터
    주도다 — farm 이 있으면 최대 폭 통로(자연 버퍼)를 쓴다(실사에선 통로 20
    이 그 자체로 T7 규칙 ① 이 요구하는 제외 대상이라 버퍼로도 맞아떨어진다).
    farm 이 없으면(레거시 시뮬레이션) 파리티까지 맞는 상수 5 를 쓴다."""
    if robot_a == robot_b:
        raise PresetError("분담 정찰은 서로 다른 로봇 두 대가 필요합니다")
    n = _int("n_alleys", n_alleys if n_alleys is not None else n_alleys_of(farm))
    if split_k is None:
        buffer_alley = widest_alley(farm)
        split_k = buffer_alley if buffer_alley is not None else _LEGACY_SPLIT_K
    k = _int("split_k", split_k)
    if not 0 < k < n - 1:
        raise PresetError(f"split_k 는 1..{n - 2} 범위여야 합니다: {k}")
    a, b = list(range(0, k)), list(range(k + 1, n))
    _check_alleys(a, "A", farm)
    _check_alleys(b, "B", farm)
    return [Plan(robot_a, _patrol_tree(robot_a, a)),
            Plan(robot_b, _patrol_tree(robot_b, b))]


def sequential_retry(robot: str, alleys: list[int], n: int = 2,
                     farm: dict | None = None) -> list[Plan]:
    """통로 목록 하나를 최대 n회 시도 — 실패해도 사람 없이 한 번 더 간다."""
    alleys = [_int("alleys", x) for x in (alleys or [])]
    _check_alleys(alleys, "통로", farm)
    tree = Sequence([Retry(_int("n", n), Action({"robot": robot, "alleys": alleys}))])
    return [Plan(robot, tree)]


def single_alley_loop(robot: str, alley: int, n: int = 1,
                      farm: dict | None = None) -> list[Plan]:
    """통로 한 줄 — 실기 확인용 최소 임무(브리프: Retry(n, Action([alley])))."""
    a = _int("alley", alley)
    na = n_alleys_of(farm)
    if not 0 <= a < na:
        raise PresetError(f"통로 번호는 0..{na - 1} 여야 합니다: {a}")
    _check_alleys([a], "통로", farm)
    return [Plan(robot, Retry(_int("n", n), Action({"robot": robot, "alleys": [a]})))]


PRESETS = {"full_split_patrol": full_split_patrol,
           "sequential_retry": sequential_retry,
           "single_alley_loop": single_alley_loop}


def build(preset: str, params: dict | None = None, farm: dict | None = None) -> list[Plan]:
    """프리셋 이름+파라미터 → 인스턴스 계획 목록. 이름·인자 오류는 PresetError.

    farm 은 항상 이 함수의 별도 인자로만 온다 — params(클라이언트 HTTP body)
    가 "farm" 키를 담아 보내도 fn(**params, farm=farm) 이 TypeError(중복
    키워드)로 거부한다(400) — 서버가 결정한 지형 규칙을 클라이언트가 덮어쓸
    수 없다."""
    fn = PRESETS.get(preset)
    if fn is None:
        raise PresetError(f"알 수 없는 프리셋: {preset} "
                          f"(가능: {', '.join(sorted(PRESETS))})")
    try:
        return fn(**(params or {}), farm=farm)
    except TypeError as e:                     # 인자 이름·개수 불일치
        raise PresetError(f"프리셋 파라미터가 맞지 않습니다: {e}") from None
