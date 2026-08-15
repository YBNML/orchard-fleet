"""현장 기하(`site_geom`) 해석 — 통로 번호 → 좌표의 **단일 출처**.

호스트가 `bb.extra["site_geom"]` 에 얹은 사전을 읽어 통로 중심 x·통로 y 구간·
횡단선 y 를 돌려준다. 같은 사전이 `hello` 의 `site.geometry` 로도 나가므로,
관제 화면이 말한 통로와 로봇이 계산한 통로가 갈라지지 않는다.

**왜 모듈로 뺐나.** 이 해석을 쓰는 기능이 둘이다 — `maintenance`(재정위
스탠드포인트)와 `mission`(주행 웨이포인트). 각자 한 벌씩 들고 있으면 언젠가
한쪽만 고쳐져 갈라지고, 그 순간 "관제가 보낸 통로"와 "로봇이 훑은 통로"가
다른 곳이 된다. 절차는 한 벌이어야 한다.

## 계약 (robomw/README.md §2 와 같은 표)

필수: `alleys`. 폴백 계산용: `x0`·`row_spacing`·`col_len`.
선택(있으면 우선):

    alley_centers_x : [x, …]                        길이 = alleys
    row_span_y      : [y_남단, y_북단]              (전 통로 공통) 또는
                      [[y_남단, y_북단], …]         (통로별, 길이 = alleys)
    alley_cross_y   : row_span_y 와 같은 두 형태    통로 진출입 **횡단선** y

**배열의 원소 순서가 곧 남단/북단의 정의다** (0번 south, 1번 north). 부호
규약이 현장마다 반대이므로(계단식 = 남단이 y 최솟값 / 실사 농장 = world +y 가
지리적 남이라 남단이 y 최댓값) 크기 비교로 추론하지 않는다.

### `alley_cross_y` 가 `row_span_y` 로 대체되지 않는 이유

`row_span_y` 는 **통로의 짧은 쪽**(이웃 두 열 중 안쪽 끝) 규약이다 — 그
자리까지 가면 통로 안이라는 뜻이라 정지·재정위에는 그것이 맞다. 그러나 통로
사이를 **가로지르는** 선은 그 사이의 열을 넘어가야 하고, 그 열의 끝 나무는
안쪽 끝보다 최대 2.7 m 바깥에 있다(실사 농장 실측). 안쪽 끝 기준으로 횡단선을
잡으면 열 끝 나무를 정면으로 들이받는다. 바깥 끝 정보는 `row_span_y` 안에
없으므로(두 통로 모두에서 '바깥'이라 max/min 어느 쪽으로도 안 나온다) 호스트가
따로 줘야 한다. 없으면 균일 격자 폴백(`±(col_len/2 + headland·0.6667)`)이다.
"""
from __future__ import annotations


def _pairs(value, k, n, key):
    """`[a, b]`(공통) 또는 `[[a, b], …]`(통로별) → (a, b). 판별은 **첫 원소가
    배열인가** 로 한다 — 길이로 판별하면 alleys==2 에서 두 형태가 겹친다."""
    try:
        if len(value) == 2 and not hasattr(value[0], "__len__"):
            return (float(value[0]), float(value[1])), None
        if len(value) != n:
            return None, f"{key} 길이 불일치 ({len(value)} vs {n})"
        pair = value[k]
        return (float(pair[0]), float(pair[1])), None
    except (TypeError, ValueError, IndexError):
        return None, f"{key} 형식 오류"


def alley_center_x(geom, k, n):
    """통로 k 의 중심 x. `alley_centers_x` 우선, 없으면 균일 격자 폴백."""
    cxs = geom.get("alley_centers_x")
    if cxs is not None:
        try:
            cxs = [float(v) for v in cxs]
        except (TypeError, ValueError):
            return None, "alley_centers_x 형식 오류"
        if len(cxs) != n:
            return None, f"alley_centers_x 길이 불일치 ({len(cxs)} vs {n})"
        return cxs[k], None
    try:
        return float(geom["x0"]) + (k + 0.5) * float(geom["row_spacing"]), None
    except (KeyError, TypeError, ValueError):
        return None, "현장 기하 형식 오류"


def alley_span_y(geom, k, n):
    """통로 k 의 (y_남단, y_북단). `row_span_y` 우선, 없으면 ±col_len/2 폴백."""
    span = geom.get("row_span_y")
    if span is not None:
        return _pairs(span, k, n, "row_span_y")
    try:
        half = float(geom["col_len"]) / 2.0
    except (KeyError, TypeError, ValueError):
        return None, "현장 기하 형식 오류"
    return (-half, half), None      # 계단식 규약: 남단 = y 최솟값


def alley_cross_y(geom, k, n, headland_factor=0.6667):
    """통로 k 로 드나드는 **횡단선** (y_남단쪽, y_북단쪽).

    `alley_cross_y` 우선. 없으면 균일 격자 폴백 `±(col_len/2 + headland·0.6667)`
    — 계단식 월드의 `DriveMission.cross_y` 와 정확히 같은 값이다.
    """
    cy = geom.get("alley_cross_y")
    if cy is not None:
        return _pairs(cy, k, n, "alley_cross_y")
    try:
        half = float(geom["col_len"]) / 2.0
        hl = float(geom.get("headland", 0.0))
    except (KeyError, TypeError, ValueError):
        return None, "현장 기하 형식 오류"
    off = half + hl * headland_factor
    return (-off, off), None
