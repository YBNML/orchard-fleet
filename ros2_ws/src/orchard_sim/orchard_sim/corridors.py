"""
자유공간 → 통로 중심선 → 경로 그래프

사용자의 실제 과수원 문제: "FAST-LIO2 로 맵은 쉽게 나오는데, 거기서 주행 가능한
통로를 뽑고 그 중앙을 지나가게 하는 게 안 됐다."

여기서 쓰는 방법은 **거리변환 + 스켈레톤**이다. 통로 중앙은 정의상 양쪽 벽에서
가장 먼 곳이므로, 자유공간의 거리변환 능선이 곧 중심선이다. 규칙을 넣지 않아도
기하에서 나온다.

그리고 중요한 성질: 단차 둑이 이미 주행 불가로 찍혀 있으면 스켈레톤은 **사다리(빗)
모양**이 된다 — 통로들이 오직 양끝 선회 구간에서만 연결된다. 그래서 통로 간 이동은
경유점을 손으로 넣지 않아도 저절로 선회 구간을 지난다.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

try:
    from skimage.morphology import skeletonize
    HAVE_SKIMAGE = True
except ImportError:
    HAVE_SKIMAGE = False


def distance_field(free, cell=0.10):
    """자유공간의 거리변환 [m]. 값이 곧 '가장 가까운 장애물까지의 거리'."""
    return ndimage.distance_transform_edt(free) * cell


def skeleton(free, min_branch_px=8):
    """자유공간 → 1픽셀 두께 스켈레톤, 짧은 가지는 쳐낸다.

    스켈레톤 가지(spur)는 통로 폭이 국소적으로 비대칭일 때 생긴다. 결주 때문에
    생긴 틈이나 잡초 하나가 만든 돌기가 대표적이다. 짧은 가지를 쳐내지 않으면
    경로 그래프에 가짜 분기가 잔뜩 생긴다.
    """
    if not HAVE_SKIMAGE:
        raise RuntimeError("scikit-image 가 필요합니다: pip install scikit-image")
    sk = skeletonize(free.astype(bool))
    for _ in range(3):
        sk = _prune_spurs(sk, min_branch_px)
    return sk


_NB = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]], np.uint8)


def _neighbor_count(sk):
    return ndimage.convolve(sk.astype(np.uint8), _NB, mode="constant")


def _prune_spurs(sk, min_len):
    """끝점에서 시작해 분기점까지의 길이가 min_len 미만인 가지를 제거."""
    sk = sk.copy()
    for _ in range(min_len):
        nc = _neighbor_count(sk)
        ends = sk & (nc == 1)
        if not ends.any():
            break
        sk[ends] = False
    return sk


def endpoints_and_junctions(sk):
    nc = _neighbor_count(sk)
    return (sk & (nc == 1)), (sk & (nc >= 3))


def trace_segments(sk):
    """스켈레톤을 '노드 사이의 경로' 단위로 쪼갠다.

    스켈레톤의 분기점은 한 픽셀이 아니라 **2~4 픽셀 군집**으로 나온다
    (2026-07-26 실측: 분기점 46 픽셀이 군집 16개, 군집당 평균 2.9 픽셀).
    노드를 (r,c) 정확 일치로 잡으면 같은 교차점이 여러 노드로 쪼개져
    그래프가 조각난다 — 실제로 연결 컴포넌트가 16개가 됐다.

    그래서 **분기점/끝점 군집을 하나의 노드로 묶고**, 노드를 제거한 뒤 남은
    체인들을 엣지로 잡는다.

    반환: [(경로 픽셀 배열, 시작 노드 id, 끝 노드 id), ...]
    """
    ends, junc = endpoints_and_junctions(sk)
    nodes_mask = ends | junc
    conn = np.ones((3, 3), np.uint8)
    node_lab, n_nodes = ndimage.label(nodes_mask, conn)

    # 노드를 뺀 나머지 = 엣지 체인
    chains = sk & ~nodes_mask
    chain_lab, n_chains = ndimage.label(chains, conn)

    H, W = sk.shape
    offs = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    def touching_nodes(mask_idx):
        """이 체인이 닿는 노드 id 집합."""
        rs, cs = mask_idx
        out = set()
        for dr, dc in offs:
            rr, cc = rs + dr, cs + dc
            ok = (rr >= 0) & (rr < H) & (cc >= 0) & (cc < W)
            lab = node_lab[rr[ok], cc[ok]]
            out.update(int(v) for v in np.unique(lab) if v > 0)
        return out

    def order_chain(rs, cs):
        """체인 픽셀을 한 끝에서부터 순서대로 정렬한다."""
        pts = list(zip(rs.tolist(), cs.tolist()))
        if len(pts) <= 2:
            return np.array(pts)
        pset = set(pts)
        deg = {p: sum(((p[0] + dr, p[1] + dc) in pset) for dr, dc in offs) for p in pts}
        starts = [p for p in pts if deg[p] <= 1] or [pts[0]]
        cur, prev, path = starts[0], None, []
        while cur is not None:
            path.append(cur)
            nxt = None
            for dr, dc in offs:
                cand = (cur[0] + dr, cur[1] + dc)
                if cand in pset and cand != prev and cand not in path:
                    nxt = cand
                    break
            prev, cur = cur, nxt
        return np.array(path)

    segs = []
    objs = ndimage.find_objects(chain_lab)
    for i, sl in enumerate(objs, start=1):
        if sl is None:
            continue
        sub = chain_lab[sl] == i
        rs, cs = np.nonzero(sub)
        rs = rs + sl[0].start
        cs = cs + sl[1].start
        if rs.size < 2:
            continue
        touch = sorted(touching_nodes((rs, cs)))
        if len(touch) < 2:
            # 노드 하나에만 닿는 막다른 체인 — 가지치기에서 남은 잔여물
            continue
        path = order_chain(rs, cs)
        segs.append((path, touch[0], touch[-1]))

    # 노드 군집끼리 직접 붙어 있는 경우(체인 길이 0)도 엣지로 잇는다
    for i in range(1, n_nodes + 1):
        rs, cs = np.nonzero(node_lab == i)
        for dr, dc in offs:
            rr, cc = rs + dr, cs + dc
            ok = (rr >= 0) & (rr < H) & (cc >= 0) & (cc < W)
            other = np.unique(node_lab[rr[ok], cc[ok]])
            for j in other:
                if j > i:
                    mid = np.array([[int(rs.mean()), int(cs.mean())]])
                    segs.append((mid, i, int(j)))

    return segs


# ═══════════════════════════════════════════════════════════════════════════
# 경로 그래프
# ═══════════════════════════════════════════════════════════════════════════
class RouteGraph:
    """세그먼트를 노드-엣지 그래프로 정리한다.

    엣지 유형은 **기하에서 판정한다** — 손으로 라벨링하지 않는다.
    통로(alley)는 길고 곧으며 주 방향이 열 방향(y)이고, 선회(headland)는
    그것들을 잇는 나머지다.
    """

    def __init__(self, grid, segments, dist, row_axis=1):
        self.g = grid
        self.dist = dist
        self.nodes = []          # 노드 대표 (r, c)
        self._nidx = {}          # 스켈레톤 노드 라벨 → 그래프 노드 id
        self.edges = []          # dict(a, b, pts, length, width, kind)
        for seg, la, lb in segments:
            a = self._node_from_label(la, seg[0])
            b = self._node_from_label(lb, seg[-1])
            if a == b:
                continue         # 자기 자신으로 돌아오는 고리는 버린다
            xy = np.stack(self.g.to_world(seg[:, 0], seg[:, 1]), axis=1)
            length = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
            width = float(np.median(dist[seg[:, 0], seg[:, 1]]) * 2.0)
            d = xy[-1] - xy[0]
            straight = float(np.linalg.norm(d)) / max(length, 1e-6)
            along_row = abs(d[row_axis]) / max(np.linalg.norm(d), 1e-6)
            kind = ("alley" if (length > 8.0 and straight > 0.9 and along_row > 0.9)
                    else "headland")
            self.edges.append(dict(a=a, b=b, pts=xy, rc=seg, length=length,
                                   width=width, kind=kind))

    def _node_from_label(self, label, rc_hint):
        """스켈레톤 노드 **군집 라벨**로 그래프 노드를 잡는다.
        픽셀 좌표로 잡으면 같은 교차점이 쪼개진다 (위 trace_segments 주석 참조)."""
        if label not in self._nidx:
            self._nidx[label] = len(self.nodes)
            self.nodes.append(tuple(int(v) for v in rc_hint))
        return self._nidx[label]

    def alleys(self):
        return [e for e in self.edges if e["kind"] == "alley"]

    def adjacency(self):
        adj = {i: [] for i in range(len(self.nodes))}
        for k, e in enumerate(self.edges):
            adj[e["a"]].append((e["b"], k))
            adj[e["b"]].append((e["a"], k))
        return adj

    def shortest_path(self, n_from, n_to):
        """엣지 길이 기준 다익스트라. 통로 간 이동이 선회 구간을 경유하는지
        확인하는 데 쓴다 — 규칙이 아니라 그래프 위상에서 나오는지 보려는 것."""
        import heapq
        adj = self.adjacency()
        dist = {n_from: 0.0}
        prev = {}
        pq = [(0.0, n_from)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == n_to:
                break
            if d > dist.get(u, np.inf):
                continue
            for v, ek in adj[u]:
                nd = d + self.edges[ek]["length"]
                if nd < dist.get(v, np.inf):
                    dist[v] = nd
                    prev[v] = (u, ek)
                    heapq.heappush(pq, (nd, v))
        if n_to not in dist:
            return None, None
        path_e, cur = [], n_to
        while cur != n_from:
            u, ek = prev[cur]
            path_e.append(ek)
            cur = u
        return list(reversed(path_e)), dist[n_to]

    def node_nearest(self, x, y):
        rc = np.array(self.nodes)
        wx, wy = self.g.to_world(rc[:, 0], rc[:, 1])
        return int(np.argmin((wx - x) ** 2 + (wy - y) ** 2))


def alley_polylines(graph, min_length=10.0):
    """통로 엣지를 중심선 폴리라인으로 정리 (y 순 정렬, 균일 리샘플)."""
    out = []
    for e in graph.alleys():
        if e["length"] < min_length:
            continue
        pts = e["pts"]
        if pts[0, 1] > pts[-1, 1]:
            pts = pts[::-1]
        out.append(dict(pts=pts, length=e["length"], width=e["width"],
                        x_mean=float(np.mean(pts[:, 0]))))
    out.sort(key=lambda d: d["x_mean"])
    return out
