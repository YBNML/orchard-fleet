"""프리셋 3종 (스펙 ③ §3) — 사람이 버튼 하나로 고르는 임무 큐 모양.

**Task5 수정 라운드1 컨트롤러 룰링 반영.** 초판은 최대 폭 통로를
`row_origins` 로부터 산술로 추정(`widest_alley` argmax)해 회피 대상으로
삼았는데, 리뷰가 이를 기각했다 — 회피 사유(농로·측위 공백 vs 광폭 이상)는
기하만으로 판별할 수 없고, 사람이 T4 실측(§4.2·§1.3)을 근거로 큐레이션해야
한다(단일 출처 원칙, C2). 그래서 회피 대상은 이제 **farm.json 의
`no_go_alleys`**(additive 수동 필드) 다. `widest_alley`/`alley_widths` 는
진단용으로 남지만 어떤 검증에도 더는 쓰이지 않는다.

**전이 규칙도 정정됐다(C1).** 초판은 T7 로봇 엔진의 k±2 여유를 "서버가
허용할 규칙"으로 착각했다 — 실제로는 REST 직접 생성 경로
(`mission_ops.alleys_sequence_valid`)가 이미 인접(±1)만 허용하고 있어서,
프리셋이 ±2 를 승인해도 그 계획이 실제로 발진할 때(BT Action → 같은
`mission_ops` 공용 경로) 다시 거부돼 "200 수락 후 조용한 FAILED" 가 났다.
지금은 프리셋도 ±1 만 승인한다 — 두 검증이 같은 규칙을 본다.

**분담 분할 재설계(C3).** farm 이 있으면(terrain 무관하게 no_go 인지) 기본
분할점은 "no_go_alleys 를 제외한 가장 큰 연속 주행 블록"의 중앙 통로다 —
그 블록만 A/B 로 나눈다(블록 밖 통로는 이 프리셋의 기본값 범위 밖이다,
T7 이 별도 임무로 다뤄야 한다). farm 이 없으면(레거시 시뮬레이션
orchard_v1) 예전 상수 5 를 그대로 쓴다 — 그 밭의 선회 평지 패드는
부스트로피돈 파리티에 고정돼 있고(`gen_heightmap.turn_pad_weights` — 쌍
(k,k+1) 은 짝수 k → 북단, 홀수 k → 남단), 계획기의 진행 방향은 통로
번호가 아니라 **요청 목록 안의 순번**에서 나온다(`up = i % 2 == 0`). 둘은
오름차순 목록의 첫 통로가 짝수일 때만 맞는다 — split_k=4 의 B=[5,6,7,8]
로 낸 임무는 두 횡단이 모두 패드 반대편 램프(최대 91% 경사)로 가 로봇이
코를 박았다(T3 리포트 §6.3 단차표).

**terrain=="terraced" 인 farm(또는 farm 이 아예 없는 레거시 호출)만** 이
파리티 게이트를 적용한다. 그 외(`flat` 등)엔 no_go 교차 금지 + 인접(±1)
전이만 본다.

프리셋은 자기가 만든 통로 목록을 스스로 검증하고, 위반이면 거부한다(API
400). REST 로 직접 내는 임무도 이제 `mission_ops`(단일 출처, I1)가
같은 no_go 규칙을 본다 — 프리셋 우회로 인한 사각지대가 없다.

farm 은 항상 서버 쪽(app.state.farm, farm_routes)에서만 주입한다 —
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
    """farm.json 의 rows(통로는 rows-1 개) → farm 이 없거나 rows 가 없으면 레거시 상수."""
    if farm is None or "rows" not in farm:
        return N_ALLEYS
    return int(farm["rows"]) - 1


def terrain_of(farm: dict | None) -> str:
    """farm.json 의 terrain → farm 이 없으면 레거시 기본(terraced)."""
    if farm is None:
        return _LEGACY_TERRAIN
    return farm.get("terrain", _LEGACY_TERRAIN)


def no_go_alleys_of(farm: dict | None) -> list[int]:
    """farm.json 의 `no_go_alleys`(수동 큐레이션 필드, C2) — 없으면 빈 목록.

    51_extract_farm_geometry.py 는 이 키를 만들지 않는다(사람이 T4 류 실측
    근거로 직접 커밋한다) — 재실행돼도 51 의 병합 로직이 보존한다."""
    if farm is None:
        return []
    return [int(a) for a in (farm.get("no_go_alleys") or [])]


def alley_widths(farm: dict) -> list[float]:
    """행 원점(row_origins)의 cross-row 성분 차분으로 통로 폭을 구한다(진단용).

    farm.json 의 axes_note: 각 열은 원점에서 world +y 로 row_length_m 만큼
    뻗는다 — 즉 cross-row(통로 폭) 축은 world x 다. 연속한 두 열의 origin_x
    차이(절댓값 — M2: 행 순서가 뒤집힌 farm 이 와도 폭은 항상 양수)가 그
    사이 통로의 폭이다."""
    origins = farm.get("row_origins") or []
    if len(origins) < 2:
        return []
    return [abs(origins[i + 1][0] - origins[i][0]) for i in range(len(origins) - 1)]


def widest_alley(farm: dict | None) -> int | None:
    """최대 폭 통로 인덱스 — 진단·회귀 확인용(더는 no_go 판정에 쓰이지 않는다, C2).

    farm 이 없거나 폭 정보가 없으면 None."""
    if farm is None:
        return None
    widths = alley_widths(farm)
    if not widths:
        return None
    return max(range(len(widths)), key=widths.__getitem__)


def drivable_blocks(farm: dict | None) -> list[range]:
    """no_go_alleys 를 뺀 연속 주행 가능 블록들(오름차순). no_go 가 없으면
    전체 범위 [0, n_alleys) 하나가 블록이다."""
    n = n_alleys_of(farm)
    no_go = set(no_go_alleys_of(farm))
    blocks: list[range] = []
    start = None
    for i in range(n):
        if i in no_go:
            if start is not None:
                blocks.append(range(start, i))
            start = None
        elif start is None:
            start = i
    if start is not None:
        blocks.append(range(start, n))
    return blocks


def largest_drivable_block(farm: dict | None) -> range | None:
    blocks = drivable_blocks(farm)
    if not blocks:
        return None
    return max(blocks, key=len)


def _block_containing(farm: dict | None, k: int) -> range | None:
    for b in drivable_blocks(farm):
        if b.start <= k < b.stop:
            return b
    return None


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
    """C2·C1 — no_go_alleys 교차 금지 + 인접(±1) 전이만 허용(terraced 외 전 지형).

    ±2("한 칸 건너")는 수정 라운드1 에서 철회됐다 — REST 직접 생성 경로의
    `mission_ops.alleys_sequence_valid` 가 이미 ±1 만 허용하므로, 프리셋이
    더 느슨한 규칙을 승인하면 그 계획이 실제 발진(BT Action → 같은 공용
    경로)에서 다시 거부된다. 두 검증은 반드시 같은 규칙을 봐야 한다."""
    if not alleys:
        return False, "통로 목록이 비었습니다"
    no_go = set(no_go_alleys_of(farm))
    hit = sorted(no_go & set(alleys))
    if hit:
        note = farm.get("no_go_note") if farm else None
        return False, (f"통로 {hit} 은 진입 금지 구간(no_go_alleys)입니다"
                       + (f" — {note}" if note else ""))
    for a, b in zip(alleys, alleys[1:]):
        if abs(b - a) != 1:
            return False, (f"통로 전이 {a}->{b} 는 인접(±1) 만 허용됩니다 "
                           f"(REST 생성 경로와 같은 규칙 — mission_ops."
                           f"alleys_sequence_valid)")
    return True, ""


def _check_alleys(alleys: list[int], who: str, farm: dict | None) -> None:
    """terrain 에 따라 파리티(terraced) 또는 no_go+인접(그 외) 을 검증한다.

    N1(수정 라운드2) — no_go 교집합은 terrain 과 무관하게 **항상 먼저** 본다.
    terraced farm 이 no_go_alleys 도 함께 들고 있는 조합은 이 저장소에 실물이
    없어 지금은 도달 불가능하지만, "두 지형 분기가 몰래 다른 규칙을 본다"는
    C1 과 같은 종류의 발산을 애초에 봉인해 둔다."""
    hit = sorted(set(no_go_alleys_of(farm)) & set(alleys))
    if hit:
        note = farm.get("no_go_note") if farm else None
        raise PresetError(f"{who} 통로 목록 {alleys}: 통로 {hit} 은 진입 금지 구간"
                          f"(no_go_alleys)입니다" + (f" — {note}" if note else ""))
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
    """분담 전체 정찰 — A=[블록/구간 시작..k-1], B=[k+1..블록/구간 끝](k 는 버퍼).

    **terrain=="terraced"(farm 없음 포함)**: 예전 그대로 — A=[0..k-1],
    B=[k+1..n-1], 기본 k=5(파리티 안전 상수).

    **그 외(no_go 인지)**: farm 의 `no_go_alleys` 를 뺀 **가장 큰 연속 주행
    블록**만 대상으로 삼는다 — split_k 미지정 시 그 블록의 중앙 통로가
    버퍼다. 다른 블록(예: 실사에서 통로 20·23 이 갈라놓은 [21,22]·[24,25])
    은 이 기본값의 커버리지 밖이다(T7 이 별도 임무로 다뤄야 한다).
    명시적 split_k 는 그 블록(no_go 아닌 통로) 안에 있어야 하며 위반 시
    400 이다."""
    if robot_a == robot_b:
        raise PresetError("분담 정찰은 서로 다른 로봇 두 대가 필요합니다")
    n = _int("n_alleys", n_alleys if n_alleys is not None else n_alleys_of(farm))

    if terrain_of(farm) == "terraced":
        k = _int("split_k", split_k if split_k is not None else _LEGACY_SPLIT_K)
        if not 0 < k < n - 1:
            raise PresetError(f"split_k 는 1..{n - 2} 범위여야 합니다: {k}")
        a, b = list(range(0, k)), list(range(k + 1, n))
    else:
        # N2(수정 라운드2) — 블록 계산은 항상 farm 유도값(n_alleys_of)을 쓴다
        # (drivable_blocks 가 그렇게 만들어져 있다). 그래서 호출자가 명시
        # n_alleys 를 farm 과 다르게 주면 그 값이 조용히 무시되던 것이 예전
        # 버그였다 — 지금은 불일치를 400 으로 명시한다(무음 무시 금지).
        farm_n = n_alleys_of(farm)
        if n_alleys is not None and _int("n_alleys", n_alleys) != farm_n:
            raise PresetError(f"n_alleys 는 farm 기하와 불일치: 요청 {n_alleys}, "
                              f"farm {farm_n}")
        no_go = set(no_go_alleys_of(farm))
        if split_k is None:
            block = largest_drivable_block(farm)
            if block is None:
                raise PresetError("이 farm 에는 분담 정찰이 가능한 주행 블록이 없습니다"
                                  f"(no_go_alleys={sorted(no_go)} 가 전 통로를 덮습니다)")
            k = block.start + len(block) // 2
        else:
            k = _int("split_k", split_k)
            if k in no_go:
                raise PresetError(f"split_k={k} 는 진입 금지 통로(no_go_alleys)입니다 — "
                                  f"버퍼로 쓸 수 없습니다")
            block = _block_containing(farm, k)
            if block is None:
                raise PresetError(f"split_k={k} 는 주행 가능한 통로 범위 밖입니다 "
                                  f"(0..{n - 1}, no_go_alleys={sorted(no_go)} 제외)")
        if not (block.start < k < block.stop - 1):
            raise PresetError(
                f"split_k 는 블록[{block.start}..{block.stop - 1}] 안쪽 "
                f"{block.start + 1}..{block.stop - 2} 범위여야 합니다: {k}")
        a, b = list(range(block.start, k)), list(range(k + 1, block.stop))

    _check_alleys(a, "A", farm)
    _check_alleys(b, "B", farm)
    return [Plan(robot_a, _patrol_tree(robot_a, a)),
            Plan(robot_b, _patrol_tree(robot_b, b))]


def sequential_retry(robot: str, alleys: list[int], n: int = 2,
                     farm: dict | None = None) -> list[Plan]:
    """통로 목록 하나를 최대 n회 시도 — 실패해도 사람 없이 한 번 더 간다."""
    alleys = [_int("alleys", x) for x in (alleys or [])]
    na = n_alleys_of(farm)
    for a in alleys:                            # I3 — 범위 검사(예전엔 없었다)
        if not 0 <= a < na:
            raise PresetError(f"통로 번호는 0..{na - 1} 여야 합니다: {a}")
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
    키워드)로 거부한다(400) — 서버가 정한 지형 규칙을 클라이언트가 우회할
    수 없다."""
    fn = PRESETS.get(preset)
    if fn is None:
        raise PresetError(f"알 수 없는 프리셋: {preset} "
                          f"(가능: {', '.join(sorted(PRESETS))})")
    try:
        return fn(**(params or {}), farm=farm)
    except TypeError as e:                     # 인자 이름·개수 불일치
        raise PresetError(f"프리셋 파라미터가 맞지 않습니다: {e}") from None
