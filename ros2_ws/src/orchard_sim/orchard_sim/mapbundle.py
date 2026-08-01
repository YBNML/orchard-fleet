"""맵 번들 — 사전 맵 기반 자율주행이 소비하는 오프라인 산출물.

왜 번들인가
    주행 중 맵을 만드는 방식(LIO)은 MID-70 의 전방 시야(±35.2°) 때문에 선회부
    에서 측방·요 제약이 사라져 오차가 **누적**된다. 8가지 대책을 재현 실험까지
    해서 전부 실패했다. 사전 맵은 절대 기준을 주므로 오차가 누적되지 않는다 —
    퇴화 구간은 잠시 관성으로 버티고, 구조물이 보이면 다시 정착한다.

    그래서 '맵을 만드는 일'과 '맵을 쓰는 일'을 파일 경계로 갈랐다. 번들은
    한 번 만들고 서명해 두고, 주행은 그것을 읽기만 한다.

구성
    cloud.npz    정합 기준 점군 (다운샘플). 로컬리제이션이 쓴다
    trav.npz     주행가능도 격자 + 원점·해상도. 주행경계(S9)의 기준
    graph.json   통로 중심선·연결 그래프. 경로 생성이 쓴다
    meta.yaml    원점·해상도·기하·**버전 해시**·생성 이력

버전 해시가 핵심이다. 서버가 v2 그래프로 만든 임무를 로봇이 v1 지형에서
실행하면 계단식 밭에서는 그대로 사고가 된다 — 로봇은 해시가 다르면 임무를
거부한다.
"""
from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path

import numpy as np

BUNDLE_VERSION = 1
FILES = ("cloud.npz", "trav.npz", "graph.json", "meta.yaml")


# ── 해시 ────────────────────────────────────────────────────────────────────
def _digest_files(d: Path) -> str:
    """meta 를 제외한 산출물의 내용 해시. 같은 입력 → 같은 해시."""
    h = hashlib.sha256()
    for name in ("cloud.npz", "trav.npz", "graph.json"):
        p = d / name
        if p.exists():
            h.update(name.encode())
            h.update(p.read_bytes())
    return h.hexdigest()[:16]


# ── 저장 ────────────────────────────────────────────────────────────────────
def save(out_dir, *, cloud, trav, origin, cell, alleys, geom,
         source: str = "", notes: str = "") -> dict:
    """번들을 디렉토리에 쓴다.

    cloud   (N,3) float32 — 정합 기준 점군
    trav    (H,W) bool    — 주행가능 격자
    origin  (xmin, ymin)  — trav[0,0] 셀의 좌하단 월드 좌표
    alleys  [{"pts": (M,2) float, "length": m, "width": m}] — 통로 중심선
    geom    {"rows","alleys","row_spacing","x0","col_len","headland", ...}
    """
    d = Path(out_dir)
    d.mkdir(parents=True, exist_ok=True)

    np.savez_compressed(d / "cloud.npz",
                        points=np.asarray(cloud, dtype=np.float32))
    np.savez_compressed(d / "trav.npz",
                        trav=np.asarray(trav, dtype=bool),
                        origin=np.asarray(origin, dtype=np.float64),
                        cell=np.float64(cell))

    graph = dict(
        cell=float(cell),
        alleys=[dict(index=i,
                     x_mean=float(np.mean(a["pts"][:, 0])),
                     length=float(a["length"]), width=float(a["width"]),
                     pts=[[round(float(x), 3), round(float(y), 3)]
                          for x, y in a["pts"]])
                for i, a in enumerate(alleys)],
    )
    (d / "graph.json").write_text(
        json.dumps(graph, ensure_ascii=False, indent=1), encoding="utf-8")

    digest = _digest_files(d)
    meta = dict(bundle_version=BUNDLE_VERSION, hash=digest,
                created_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
                source=source, notes=notes,
                cell=float(cell),
                origin=[float(origin[0]), float(origin[1])],
                shape=[int(trav.shape[0]), int(trav.shape[1])],
                n_points=int(len(cloud)), n_alleys=len(alleys),
                geom={k: (float(v) if isinstance(v, (int, float, np.floating))
                          else v) for k, v in geom.items()})
    # YAML 을 직접 쓴다 — 의존성을 늘리지 않으려고. 구조가 평평해서 충분하다.
    lines = []
    for k, v in meta.items():
        if isinstance(v, dict):
            lines.append(f"{k}:")
            for kk, vv in v.items():
                lines.append(f"  {kk}: {json.dumps(vv, ensure_ascii=False)}")
        else:
            lines.append(f"{k}: {json.dumps(v, ensure_ascii=False)}")
    (d / "meta.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return meta


# ── 적재 ────────────────────────────────────────────────────────────────────
class Bundle:
    """읽기 전용 맵 번들. 주행 스택이 쓰는 쪽."""

    def __init__(self, d):
        self.dir = Path(d)
        missing = [f for f in FILES if not (self.dir / f).exists()]
        if missing:
            raise FileNotFoundError(f"번들에 파일이 없다: {missing}")
        self.meta = _read_meta(self.dir / "meta.yaml")

        z = np.load(self.dir / "cloud.npz")
        self.cloud = z["points"]
        z = np.load(self.dir / "trav.npz")
        self.trav = z["trav"]
        self.origin = tuple(float(v) for v in z["origin"])
        self.cell = float(z["cell"])
        self.graph = json.loads((self.dir / "graph.json").read_text(encoding="utf-8"))
        self.alleys = [np.asarray(a["pts"], dtype=float) for a in self.graph["alleys"]]

    # 임무 계약 — 로봇은 해시가 다르면 임무를 거부한다
    @property
    def hash(self) -> str:
        return str(self.meta.get("hash", ""))

    def verify(self) -> bool:
        """파일 내용이 meta 의 해시와 맞는가 (전송 중 손상·부분 갱신 탐지)."""
        return _digest_files(self.dir) == self.hash

    # 좌표 변환 — trav 격자 ↔ 월드
    def to_idx(self, x, y):
        c = np.floor((np.asarray(x) - self.origin[0]) / self.cell).astype(int)
        r = np.floor((np.asarray(y) - self.origin[1]) / self.cell).astype(int)
        return r, c

    def is_drivable(self, x, y) -> bool:
        r, c = self.to_idx(x, y)
        if not (0 <= r < self.trav.shape[0] and 0 <= c < self.trav.shape[1]):
            return False
        return bool(self.trav[r, c])

    def alley_count(self) -> int:
        return len(self.alleys)


def _read_meta(path) -> dict:
    """save() 가 쓴 평평한 YAML 을 되읽는다 (외부 의존성 없이)."""
    meta, cur = {}, None
    for raw in Path(path).read_text(encoding="utf-8").splitlines():
        if not raw.strip():
            continue
        if raw.startswith("  ") and cur is not None:
            k, _, v = raw.strip().partition(": ")
            meta[cur][k] = json.loads(v) if v else None
            continue
        k, _, v = raw.partition(": ")
        if v == "":
            cur = k.rstrip(":")
            meta[cur] = {}
        else:
            cur = None
            meta[k] = json.loads(v)
    return meta
