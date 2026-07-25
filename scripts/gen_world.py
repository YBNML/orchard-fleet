#!/usr/bin/env python3
"""
과수원 월드 생성기 (설계서 §3 / §4)

    python3 scripts/gen_world.py --rows 4 --trees-per-row 20 --out sim/worlds/orchard.sdf

전제:
  · gen_tree.py 로 나무 모델들이 sim/models/ 에 이미 생성돼 있어야 한다
  · gen_heightmap.py 로 지형이 생성돼 있어야 한다

구조 (2026-07-25 실측 근거, docs/findings/...):
  · panoptic 인스턴스 분리는 최상위 non-nested 모델 단위로만 일어난다
  · 배경 행   : tree_full 모델 <include> 하나 (semantic 라벨만)
  · 계측 블록 : tree_body 모델 <include> + 과실마다 apple 모델을 최상위 <include>
               → 과실별 인스턴스 ID 확보. 단, 엔티티 수가 폭증하므로 소수 그루만
"""
import argparse
import json
import math
import os
import random

try:
    import numpy as np
except ImportError:
    np = None


# ── 경사면 지형 높이 샘플링 ─────────────────────────────────────────────────
class Terrain:
    """gen_heightmap.py 가 남긴 높이 필드를 읽어 (x,y) → z(m) 를 준다.

    나무·로봇·오브젝트를 경사면에 앉히기 위한 것. 평지 폴백도 지원한다.
    """
    def __init__(self, models_dir, name):
        self.ok = False
        base = os.path.join(models_dir, name)
        npy = os.path.join(base, "heightmap.npy")
        meta = os.path.join(base, "heightmap_meta.json")
        if np is None or not (os.path.exists(npy) and os.path.exists(meta)):
            return
        self.H = np.load(npy)
        with open(meta) as f:
            self.m = json.load(f)
        self.n = self.H.shape[0]
        self.ok = True

    def z(self, x, y, flip_x=False, flip_y=False):
        if not self.ok:
            return 0.0
        half = self.m["half"]; E = self.m["size_x"]; n = self.n
        fx = (x + half) / E
        fy = (y + half) / E
        if flip_x:
            fx = 1 - fx
        if flip_y:
            fy = 1 - fy
        col = min(max(fx * (n - 1), 0), n - 1)
        row = min(max(fy * (n - 1), 0), n - 1)
        c0, r0 = int(col), int(row)
        c1, r1 = min(c0 + 1, n - 1), min(r0 + 1, n - 1)
        dc, dr = col - c0, row - r0
        top = self.H[r0, c0] * (1 - dc) + self.H[r0, c1] * dc
        bot = self.H[r1, c0] * (1 - dc) + self.H[r1, c1] * dc
        return float(top * (1 - dr) + bot * dr)

# 세장방추형 기본 배치 (설계서 §4.1)
DEFAULTS = dict(
    row_spacing=3.50,        # 열간 m
    tree_spacing=1.50,       # 주간 m
    headland=6.0,            # 선회 공간 m (양단)
    understory_width=1.20,   # 수관하부 청경(나지) 폭 m
    post_spacing=10.0,       # 지주 간격 m
    terrain_size=120.0,      # 지형 한 변 m
    terrain_relief=1.5,      # 총기복 m
    pos_jitter=0.05,         # 나무 위치 지터 σ m
    yaw_jitter=0.20,         # 나무 방위 지터 σ rad
    missing_prob=0.03,       # 결주 확률
)


def discover_trees(models_dir):
    """sim/models 에서 apple_tree_* 모델과 그 severity 를 수집한다."""
    trees = []
    for name in sorted(os.listdir(models_dir)):
        if not name.startswith("apple_tree_"):
            continue
        gt_path = os.path.join(models_dir, name, "ground_truth.json")
        sev = None
        if os.path.exists(gt_path):
            with open(gt_path) as f:
                sev = json.load(f).get("severity")
        # model.sdf = full(배경목 include용), model_body.sdf = body(계측 블록용)
        has_full = os.path.exists(os.path.join(models_dir, name, "model.sdf"))
        has_body = os.path.exists(os.path.join(models_dir, name, "model_body.sdf"))
        trees.append(dict(name=name, severity=sev, full=has_full, body=has_body))
    return trees


# ── SDF 조각 ────────────────────────────────────────────────────────────────
def sdf_header(world_name, cfg, terrain_model, step_size=0.002,
               collision_detector="bullet"):
    phys_name = f"{step_size * 1000:.0f}ms"
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{world_name}">

    <plugin filename="gz-sim-physics-system"           name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"     name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"           name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"               name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system"            name="gz::sim::systems::NavSat"/>

    <physics name="{phys_name}" type="dart">
      <max_step_size>{step_size}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <!-- 하이트맵 충돌은 DART 기본(FCL) 에서 매우 비싸다.
           513x513 → 52만 삼각형을 매 스텝 처리하게 되어 RTF 를 잡아먹는다.
           설계서 §4.2 / gz-sim heightmap.sdf 권고에 따라 bullet 을 쓴다. -->
      <dart><collision_detector>{collision_detector}</collision_detector></dart>
    </physics>

    <!-- 국내 어느 과수원의 좌표를 걸어둔다 (GNSS/navsat 시뮬용) -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>36.57</latitude_deg>
      <longitude_deg>128.50</longitude_deg>
      <elevation>50.0</elevation>
    </spherical_coordinates>

    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.6 0.7 0.85</background>
      <sky/>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>1.0 0.98 0.95 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.85</direction>
    </light>

    <include>
      <uri>model://{terrain_model}</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
"""


def sdf_footer():
    return "\n  </world>\n</sdf>\n"


def bg_tree_include(name, model, x, y, z, yaw):
    """배경목: tree_full(과실 구워넣음) 모델을 통째로 <include>.

    model.config 의 기본 SDF 가 model.sdf(=full 버전)이므로 <include> 하나로
    과실까지 렌더링된다. 나무 전체가 하나의 semantic 라벨을 갖는다(인스턴스 불필요).
    z 는 경사면 지형 높이 (Terrain.z).
    """
    return f"""    <include>
      <name>{name}</name>
      <uri>model://{model}</uri>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
    </include>
"""


# ── 나무 영역 밖 환경 오브젝트 ──────────────────────────────────────────────
# 라벨 체계 확장 (설계서 §8.3 의 0~59 뒤를 잇는다)
LBL_GROUND = 10         # 지면 (초생)
LBL_WEED = 11           # 잡초
LBL_TRELLIS = 12        # 지주
LBL_WIRE = 13           # 지주 와이어
LBL_FRUIT_FALLEN = 43   # 낙과
LBL_STRUCTURE = 60      # 건물·탱크·울타리·전신주
LBL_MACHINERY = 61      # 농기계 (트랙터·스피드스프레이어)
LBL_CONTAINER = 62      # 수확 컨테이너
LBL_WINDBREAK = 63      # 방풍림
LBL_ROCK = 64           # 돌무더기 등 자연 장애물
LBL_ROAD = 65           # 진입로
LBL_SOIL = 66           # 수관하부 청경(나지) 대
LBL_IRRIGATION = 67     # 점적관수 호스


class Prop:
    """여러 visual/collision 을 **하나의 모델 엔티티**로 묶는다.

    2026-07-25 RTF 벤치마크에서 확인한 사실:
        엔티티 554개 → RTF 1.21 / 엔티티 1,754개 → RTF 0.30
    비용은 삼각형 수가 아니라 **모델 엔티티 개수**에 붙는다. 그러므로 디테일은
    "모델을 많이" 가 아니라 "한 모델 안에 visual 을 많이" 로 넣어야 한다.
    이 클래스가 그 규율을 강제한다.
    """

    def __init__(self, name, static=True):
        self.name = name
        self.static = static
        self.items = []
        self.cols = []
        self._n = 0

    def _mat(self, rgb, spec=None):
        r, g, b = rgb
        s = spec or (r * 0.3, g * 0.3, b * 0.3)
        return (f'<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>'
                f'<diffuse>{r * 1.1:.3f} {g * 1.1:.3f} {b * 1.1:.3f} 1</diffuse>'
                f'<specular>{s[0]:.3f} {s[1]:.3f} {s[2]:.3f} 1</specular></material>')

    def add(self, geom, pose, rgb, label, tag=None):
        self._n += 1
        nm = tag or f"v{self._n}"
        px, py, pz, rr, pp, yy = pose
        self.items.append(
            f'        <visual name="{nm}_{self._n}">\n'
            f'          <pose>{px:.3f} {py:.3f} {pz:.3f} {rr:.4f} {pp:.4f} {yy:.4f}</pose>\n'
            f'          <geometry>{geom}</geometry>\n'
            f'          {self._mat(rgb)}\n'
            + _lbl(label) +
            f'        </visual>\n')

    def add_collision(self, geom, pose):
        px, py, pz, rr, pp, yy = pose
        self.cols.append(
            f'        <collision name="c{len(self.cols)}">\n'
            f'          <pose>{px:.3f} {py:.3f} {pz:.3f} {rr:.4f} {pp:.4f} {yy:.4f}</pose>\n'
            f'          <geometry>{geom}</geometry>\n'
            f'        </collision>\n')

    def build(self):
        if not self.items:
            return ""
        st = "      <static>true</static>\n" if self.static else ""
        return (f'    <model name="{self.name}">\n{st}'
                f'      <pose>0 0 0 0 0 0</pose>\n'
                f'      <link name="link">\n'
                + "".join(self.items) + "".join(self.cols) +
                f'      </link>\n    </model>\n')

    def count(self):
        return self._n


def _lbl(n):
    return (f'          <plugin filename="gz-sim-label-system" '
            f'name="gz::sim::systems::Label"><label>{n}</label></plugin>\n')


# ── 지오메트리 축약 ─────────────────────────────────────────────────────────
def _box(x, y, z):
    return f"<box><size>{x:.4f} {y:.4f} {z:.4f}</size></box>"


def _cyl(r, l):
    return f"<cylinder><radius>{r:.4f}</radius><length>{l:.4f}</length></cylinder>"


def _ell(a, b, c):
    return f"<ellipsoid><radii>{a:.4f} {b:.4f} {c:.4f}</radii></ellipsoid>"


def _sph(r):
    return f"<sphere><radius>{r:.4f}</radius></sphere>"


# ═══════════════════════════════════════════════════════════════════════════
# 환경 구성
# ═══════════════════════════════════════════════════════════════════════════
def build_environment(cfg, terrain, orchard_x0, orchard_x1, orchard_y0, orchard_y1,
                      flip_x, flip_y, rng, detail=2):
    """나무 영역 밖 + 열 사이 디테일. detail: 0=없음 1=기본 2=풍부

    반환은 SDF 문자열 리스트. 각 요소가 **모델 하나** 이므로 리스트 길이가
    곧 추가 엔티티 수다 (RTF 예산의 실제 단위).
    """
    zf = lambda x, y: terrain.z(x, y, flip_x, flip_y)
    HL = cfg["headland"]
    S = cfg["row_spacing"]
    margin = 4.0
    fx0, fx1 = orchard_x0 - margin, orchard_x1 + margin
    fy0, fy1 = orchard_y0 - HL - 3.0, orchard_y1 + HL + 3.0
    out = []

    # ── 방풍림 (좌·우·상, 각 변을 모델 하나로) ───────────────────────
    for tag, wx in (("W", fx0 - 1.5), ("E", fx1 + 1.5)):
        p = Prop(f"windbreak_{tag}")
        y = fy0 + 1.0
        while y <= fy1 - 1.0:
            jx = wx + rng.gauss(0, 0.3)
            _windbreak_into(p, jx, y, zf(jx, y), rng.uniform(4.5, 6.5), rng)
            y += rng.uniform(2.8, 3.6)
        out.append(p.build())
    p = Prop("windbreak_N")
    for i in range(7):
        wx = fx0 + i * (fx1 - fx0) / 6.0
        wy = fy1 + 1.5 + rng.gauss(0, 0.4)
        _windbreak_into(p, wx, wy, zf(wx, wy), rng.uniform(4.5, 6.0), rng)
    out.append(p.build())

    # ── 울타리: 말뚝 + 철망 담장 + 상하 가로대 ───────────────────────
    gate = (fx0 + fx1) / 2
    runs = [("fence_W", [(fx0, fy0), (fx0, fy1)]),
            ("fence_E", [(fx1, fy0), (fx1, fy1)]),
            ("fence_N", [(fx0, fy1), (fx1, fy1)]),
            ("fence_S", [(fx0, fy0), (gate - 2.5, fy0)]),
            ("fence_S2", [(gate + 2.5, fy0), (fx1, fy0)])]
    for nm, pts in runs:
        out.append(_fence_run(nm, pts, zf, detail))

    # ── 진입로 (모델 하나에 분절 visual) ─────────────────────────────
    road_y = orchard_y0 - HL * 0.55
    out.append(_access_road("access_road", fx0 + 0.5, fx1 - 0.5, road_y, zf))

    # ── 건물·설비 ────────────────────────────────────────────────────
    sx, sy = fx1 - 2.5, fy0 + 2.5
    out.append(_farm_yard("farm_yard", sx, sy, zf, rng))

    # ── 농기계 (트랙터 + 스피드스프레이어, 모델 하나) ────────────────
    trx, try_ = fx0 + 3.0, fy0 + 2.0
    out.append(_machinery("machinery", trx, try_, zf, rng))

    # ── 수확 컨테이너 (전부 모델 하나) ───────────────────────────────
    p = Prop("harvest_bins")
    for bx, by in [(orchard_x0 + 2.0, orchard_y0 - HL * 0.45),
                   (orchard_x0 + 9.0, orchard_y1 + HL * 0.45),
                   (orchard_x1 - 3.0, orchard_y1 + HL * 0.5)]:
        _bins_into(p, bx, by, zf(bx, by), rng.uniform(0, 3.14),
                   rng.randint(1, 3), rng.randint(2, 4))
    out.append(p.build())

    # ── 전신주 (모델 하나) ───────────────────────────────────────────
    p = Prop("utility_poles")
    pole_xy = []
    for i in range(4):
        px = fx1 + 2.5
        py = fy0 + (fy1 - fy0) * (i + 0.5) / 4
        pz = zf(px, py)
        pole_xy.append((px, py, pz))
        p.add(_cyl(0.13, 8.0), (px, py, pz + 4.0, 0, 0, 0), (0.55, 0.55, 0.52), LBL_STRUCTURE, "pole")
        p.add(_box(0.10, 1.8, 0.10), (px, py, pz + 7.2, 0, 0, 0), (0.35, 0.28, 0.2), LBL_STRUCTURE, "arm")
        p.add_collision(_cyl(0.15, 8.0), (px, py, pz + 4.0, 0, 0, 0))
    # 전선 — 전신주 사이를 잇는다 (처짐은 생략, 직선 근사)
    for (ax, ay, az), (bx, by, bz) in zip(pole_xy[:-1], pole_xy[1:]):
        for off in (-0.7, 0.0, 0.7):
            L = math.hypot(bx - ax, by - ay)
            mx, my = (ax + bx) / 2, (ay + by) / 2
            mz = (az + bz) / 2 + 7.2
            yaw = math.atan2(by - ay, bx - ax)
            p.add(_box(L, 0.03, 0.03), (mx, my + off, mz, 0, 0, yaw),
                  (0.2, 0.2, 0.2), LBL_STRUCTURE, "wire")
    out.append(p.build())

    # ── 돌무더기 (모델 하나) ─────────────────────────────────────────
    p = Prop("rock_piles")
    placed = 0
    for i in range(8):
        rx = rng.choice([fx0 - 0.8, fx1 + 0.8, rng.uniform(fx0, fx1)])
        ry = rng.uniform(fy0 + 1.0, fy1 - 1.0)
        if orchard_x0 - 1 < rx < orchard_x1 + 1 and orchard_y0 - 1 < ry < orchard_y1 + 1:
            continue
        z = zf(rx, ry)
        for _ in range(rng.randint(4, 8)):
            r = rng.uniform(0.16, 0.34)
            g = rng.uniform(0.38, 0.55)
            p.add(_ell(r, r * 0.85, r * 0.7),
                  (rx + rng.gauss(0, 0.45), ry + rng.gauss(0, 0.45), z + r * 0.8, 0, 0, 0),
                  (g, g, g * 0.95), LBL_ROCK, "rock")
        p.add_collision(_box(1.4, 1.4, 0.5), (rx, ry, z + 0.25, 0, 0, 0))
        placed += 1
    out.append(p.build())

    return [o for o in out if o]


def _windbreak_into(p, x, y, z, h, rng):
    r = rng.uniform(0.55, 0.85)
    crown_h = h * 0.78
    p.add(_cyl(0.12, h * 0.22), (x, y, z + h * 0.11, 0, 0, 0), (0.28, 0.2, 0.13),
          LBL_WINDBREAK, "wb_trunk")
    p.add(_ell(r, r, crown_h / 2), (x, y, z + h * 0.22 + crown_h / 2, 0, 0, 0),
          (0.12, 0.30, 0.14), LBL_WINDBREAK, "wb_crown")
    p.add_collision(_cyl(0.15, h), (x, y, z + h / 2, 0, 0, 0))


def _fence_run(name, pts, zf, detail, post_gap=2.5):
    """말뚝 + 철망 담장 + 상하 가로대. 전체가 모델 하나."""
    p = Prop(name)
    for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
        seg = math.hypot(bx - ax, by - ay)
        n = max(int(seg / post_gap), 1)
        yaw = math.atan2(by - ay, bx - ax)
        for i in range(n + 1):
            t = i / n
            px, py = ax + (bx - ax) * t, ay + (by - ay) * t
            pz = zf(px, py)
            p.add(_cyl(0.05, 1.35), (px, py, pz + 0.675, 0, 0, 0),
                  (0.42, 0.34, 0.24), LBL_STRUCTURE, "post")
        # 패널 — 말뚝 사이 구간마다 철망 담장 + 가로대
        for i in range(n):
            t0, t1 = i / n, (i + 1) / n
            xa, ya = ax + (bx - ax) * t0, ay + (by - ay) * t0
            xb, yb = ax + (bx - ax) * t1, ay + (by - ay) * t1
            xm, ym = (xa + xb) / 2, (ya + yb) / 2
            za, zb = zf(xa, ya), zf(xb, yb)
            zm = (za + zb) / 2
            L = math.hypot(xb - xa, yb - ya)
            pitch = -math.atan2(zb - za, L) if L > 1e-6 else 0.0
            # 철망 담장 (하부 0.1~1.0 m). 알파 텍스처 대신 얇은 판 — 알파는
            # 비싸고 LiDAR 깊이 패스를 교란한다 (설계서 §9-3)
            p.add(_box(L * 1.02, 0.02, 0.90), (xm, ym, zm + 0.55, 0, pitch, yaw),
                  (0.46, 0.47, 0.44), LBL_STRUCTURE, "mesh")
            # 상·하 가로대
            for hz in (0.15, 1.28):
                p.add(_box(L * 1.02, 0.035, 0.035), (xm, ym, zm + hz, 0, pitch, yaw),
                      (0.38, 0.31, 0.22), LBL_STRUCTURE, "rail")
            if detail >= 2:
                # 상단 철선 2가닥
                for hz in (1.05, 1.19):
                    p.add(_box(L * 1.02, 0.012, 0.012), (xm, ym, zm + hz, 0, pitch, yaw),
                          (0.30, 0.30, 0.32), LBL_STRUCTURE, "wire")
            p.add_collision(_box(L, 0.06, 1.35), (xm, ym, zm + 0.675, 0, pitch, yaw))
    return p.build()


def _access_road(name, x_start, x_end, y, zf, width=3.0, segments=26):
    p = Prop(name)
    for i in range(segments):
        t0, t1 = i / segments, (i + 1) / segments
        xa = x_start + (x_end - x_start) * t0
        xb = x_start + (x_end - x_start) * t1
        xm = (xa + xb) / 2
        za, zb = zf(xa, y), zf(xb, y)
        zm = (za + zb) / 2
        L = abs(xb - xa)
        pitch = -math.atan2(zb - za, L) if L > 1e-6 else 0.0
        p.add(_box(L * 1.06, width, 0.04), (xm, y, zm + 0.03, 0, pitch, 0),
              (0.44, 0.38, 0.30), LBL_ROAD, "road")
        # 바퀴자국 두 줄
        for wy in (-0.75, 0.75):
            p.add(_box(L * 1.06, 0.35, 0.012), (xm, y + wy, zm + 0.052, 0, pitch, 0),
                  (0.36, 0.30, 0.24), LBL_ROAD, "rut")
    return p.build()


def _farm_yard(name, sx, sy, zf, rng):
    """창고 + 물탱크 + 호스릴 + 비료 포대 — 한 모델."""
    p = Prop(name)
    z = zf(sx, sy)
    yaw = rng.uniform(-0.25, 0.25)
    # 창고
    p.add(_box(4.0, 3.0, 2.5), (sx, sy, z + 1.25, 0, 0, yaw), (0.70, 0.68, 0.60),
          LBL_STRUCTURE, "shed_wall")
    p.add(_box(4.2, 2.1, 0.12), (sx, sy - 0.85, z + 2.9, 0.5, 0, yaw), (0.35, 0.15, 0.12),
          LBL_STRUCTURE, "roof_l")
    p.add(_box(4.2, 2.1, 0.12), (sx, sy + 0.85, z + 2.9, -0.5, 0, yaw), (0.35, 0.15, 0.12),
          LBL_STRUCTURE, "roof_r")
    p.add(_box(0.06, 1.0, 2.0), (sx - 2.0, sy, z + 1.0, 0, 0, yaw), (0.30, 0.26, 0.22),
          LBL_STRUCTURE, "door")
    p.add_collision(_box(4.0, 3.0, 2.5), (sx, sy, z + 1.25, 0, 0, yaw))
    # 물탱크
    tx, ty = sx - 5.5, sy + 0.5
    tz = zf(tx, ty)
    p.add(_cyl(1.1, 2.5), (tx, ty, tz + 1.25, 0, 0, 0), (0.55, 0.57, 0.60),
          LBL_STRUCTURE, "tank")
    p.add(_cyl(1.15, 0.12), (tx, ty, tz + 2.56, 0, 0, 0), (0.42, 0.44, 0.47),
          LBL_STRUCTURE, "tank_lid")
    p.add_collision(_cyl(1.1, 2.5), (tx, ty, tz + 1.25, 0, 0, 0))
    # 호스릴
    hx, hy = sx - 3.0, sy - 1.6
    hz = zf(hx, hy)
    p.add(_box(1.1, 0.8, 0.35), (hx, hy, hz + 0.18, 0, 0, 0), (0.40, 0.15, 0.12),
          LBL_STRUCTURE, "reel_frame")
    p.add(_cyl(0.42, 0.55), (hx, hy, hz + 0.62, 1.5708, 0, 0), (0.15, 0.18, 0.35),
          LBL_STRUCTURE, "reel_drum")
    p.add_collision(_box(1.1, 0.9, 1.0), (hx, hy, hz + 0.5, 0, 0, 0))
    # 비료 포대 더미
    bx, by = sx + 1.2, sy - 2.2
    bz = zf(bx, by)
    for i in range(6):
        r_, c_ = divmod(i, 3)
        p.add(_box(0.6, 0.35, 0.22),
              (bx + c_ * 0.38, by + rng.gauss(0, 0.05), bz + 0.11 + r_ * 0.23,
               0, 0, rng.gauss(0, 0.15)),
              (0.78, 0.76, 0.70), LBL_STRUCTURE, "sack")
    return p.build()


def _machinery(name, trx, try_, zf, rng):
    """트랙터 + 스피드스프레이어 — 한 모델."""
    p = Prop(name)
    # ── 트랙터 ──
    z = zf(trx, try_)
    yaw = rng.uniform(-0.4, 0.4)
    p.add(_box(2.9, 1.3, 0.75), (trx, try_, z + 0.72, 0, 0, yaw), (0.15, 0.35, 0.15),
          LBL_MACHINERY, "tr_body")
    p.add(_box(1.15, 1.2, 0.9), (trx - 0.45, try_, z + 1.55, 0, 0, yaw), (0.20, 0.22, 0.25),
          LBL_MACHINERY, "tr_cab")
    p.add(_cyl(0.09, 0.9), (trx - 0.45, try_, z + 2.35, 0, 0, 0), (0.25, 0.25, 0.28),
          LBL_MACHINERY, "tr_exh")
    for dx, dy, r_, w_ in ((1.0, 0.72, 0.42, 0.28), (1.0, -0.72, 0.42, 0.28),
                           (-0.95, 0.75, 0.62, 0.36), (-0.95, -0.75, 0.62, 0.36)):
        p.add(_cyl(r_, w_), (trx + dx, try_ + dy, z + r_, -1.5708, 0, 0),
              (0.08, 0.08, 0.08), LBL_MACHINERY, "tr_wheel")
    p.add_collision(_box(3.0, 1.6, 1.8), (trx, try_, z + 0.9, 0, 0, yaw))
    # ── 스피드스프레이어 ──
    sx, sy = trx + 4.8, try_ + 0.3
    z = zf(sx, sy)
    yaw = rng.uniform(-0.4, 0.4)
    p.add(_cyl(0.62, 2.0), (sx, sy, z + 1.0, 0, 1.5708, yaw), (0.75, 0.72, 0.15),
          LBL_MACHINERY, "sp_tank")
    p.add(_cyl(0.50, 0.35), (sx - 1.15, sy, z + 0.95, 0, 1.5708, yaw), (0.30, 0.30, 0.32),
          LBL_MACHINERY, "sp_fan")
    p.add(_box(2.4, 1.15, 0.35), (sx, sy, z + 0.42, 0, 0, yaw), (0.25, 0.25, 0.28),
          LBL_MACHINERY, "sp_chassis")
    for dy in (0.62, -0.62):
        p.add(_cyl(0.32, 0.22), (sx + 0.7, sy + dy, z + 0.32, -1.5708, 0, 0),
              (0.08, 0.08, 0.08), LBL_MACHINERY, "sp_wheel")
    p.add_collision(_box(2.6, 1.3, 1.6), (sx, sy, z + 0.9, 0, 0, yaw))
    return p.build()


def _bins_into(p, x, y, z, yaw, stacks, per_stack):
    W, D, Hh = 1.15, 1.15, 0.75
    c, s = math.cos(yaw), math.sin(yaw)
    for st in range(stacks):
        for i in range(per_stack):
            ox = st * (W + 0.12)
            wx = x + ox * c
            wy = y + ox * s
            p.add(_box(W, D, Hh), (wx, wy, z + Hh / 2 + i * Hh, 0, 0, yaw),
                  (0.52, 0.36, 0.20), LBL_CONTAINER, "bin")
            # 측면 살대 — 목재 컨테이너 느낌
            p.add(_box(W * 1.01, D * 1.01, 0.06), (wx, wy, z + i * Hh + Hh - 0.06, 0, 0, yaw),
                  (0.40, 0.27, 0.15), LBL_CONTAINER, "bin_rim")
    p.add_collision(_box(stacks * (W + 0.12), D, per_stack * Hh),
                    (x, y, z + per_stack * Hh / 2, 0, 0, yaw))


# ═══════════════════════════════════════════════════════════════════════════
# 열 사이 디테일 — 지주/와이어, 청경대, 관수호스, 잡초, 낙과
# ═══════════════════════════════════════════════════════════════════════════
def build_row_details(cfg, terrain, x0, y0, R, T, flip_x, flip_y, rng, detail=2):
    """수목열·통로에 붙는 디테일. 열당 모델 하나로 묶는다."""
    zf = lambda x, y: terrain.z(x, y, flip_x, flip_y)
    S = cfg["row_spacing"]
    col_l = (T - 1) * cfg["tree_spacing"]
    y_lo, y_hi = y0 - 1.0, y0 + col_l + 1.0
    out = []

    # ── 지주 + 3단 와이어 (설계서 §4.1: 1.00 / 2.00 / 2.90 m) ────────
    for r in range(R):
        rx = x0 + r * S
        p = Prop(f"trellis_r{r}")
        y = y_lo
        while y <= y_hi:
            z = zf(rx, y)
            p.add(_cyl(0.0375, 2.96), (rx, y, z + 1.48, 0, 0, 0), (0.35, 0.28, 0.20),
                  LBL_TRELLIS, "post")
            y += cfg["post_spacing"]
        # 말단 앵커 (외측 30° 경사)
        for ye, sgn in ((y_lo, -1), (y_hi, 1)):
            z = zf(rx, ye)
            p.add(_cyl(0.045, 1.83), (rx, ye + sgn * 0.5, z + 0.79, sgn * 0.52, 0, 0),
                  (0.35, 0.28, 0.20), LBL_TRELLIS, "anchor")
        # 와이어 3단 — 지형을 따라 분절
        nseg = 12
        for hz in (1.00, 2.00, 2.90):
            for i in range(nseg):
                ya = y_lo + (y_hi - y_lo) * i / nseg
                yb = y_lo + (y_hi - y_lo) * (i + 1) / nseg
                ym = (ya + yb) / 2
                za, zb = zf(rx, ya), zf(rx, yb)
                L = yb - ya
                roll = math.atan2(zb - za, L)
                p.add(_box(0.006, L * 1.02, 0.006),
                      (rx, ym, (za + zb) / 2 + hz, roll, 0, 0),
                      (0.55, 0.55, 0.58), LBL_WIRE, "wire")
        out.append(p.build())

    # ── 수관하부 청경(나지) 대 — 설계서 §4.2 의 2존 지면 ─────────────
    UW = cfg["understory_width"]
    for r in range(R):
        rx = x0 + r * S
        p = Prop(f"understory_r{r}")
        nseg = 20
        for i in range(nseg):
            ya = y_lo + (y_hi - y_lo) * i / nseg
            yb = y_lo + (y_hi - y_lo) * (i + 1) / nseg
            ym = (ya + yb) / 2
            za, zb = zf(rx, ya), zf(rx, yb)
            L = yb - ya
            roll = math.atan2(zb - za, L)
            p.add(_box(UW, L * 1.04, 0.03), (rx, ym, (za + zb) / 2 + 0.02, roll, 0, 0),
                  (0.40, 0.33, 0.25), LBL_SOIL, "soil")
        if detail >= 2:
            # 점적관수 호스 — 청경대 위를 따라간다
            for i in range(nseg):
                ya = y_lo + (y_hi - y_lo) * i / nseg
                yb = y_lo + (y_hi - y_lo) * (i + 1) / nseg
                ym = (ya + yb) / 2
                za, zb = zf(rx, ya), zf(rx, yb)
                L = yb - ya
                roll = math.atan2(zb - za, L)
                p.add(_cyl(0.011, L * 1.02), (rx + 0.28, ym, (za + zb) / 2 + 0.05, roll + 1.5708, 0, 0),
                      (0.10, 0.10, 0.11), LBL_IRRIGATION, "drip")
        out.append(p.build())

    if detail < 2:
        return [o for o in out if o]

    # ── 통로 잡초 포기 (통로당 모델 하나) ────────────────────────────
    for k in range(R - 1):
        cx = x0 + (k + 0.5) * S
        p = Prop(f"weeds_a{k}")
        for _ in range(70):
            wx = cx + rng.gauss(0, 0.75)
            wy = rng.uniform(y_lo, y_hi)
            wz = zf(wx, wy)
            h = rng.uniform(0.10, 0.26)
            g = rng.uniform(0.22, 0.42)
            p.add(_ell(rng.uniform(0.07, 0.16), rng.uniform(0.07, 0.16), h / 2),
                  (wx, wy, wz + h / 2, 0, 0, 0), (g * 0.5, g, g * 0.35), LBL_WEED, "weed")
        out.append(p.build())

    # ── 낙과 (계측 블록 주변, 모델 하나) ─────────────────────────────
    p = Prop("fallen_fruit")
    for _ in range(90):
        r = rng.randrange(R)
        fx = x0 + r * S + rng.gauss(0, 0.55)
        fy = rng.uniform(y_lo, y_hi)
        fz = zf(fx, fy)
        p.add(_sph(0.0375), (fx, fy, fz + 0.035, 0, 0, 0),
              (rng.uniform(0.45, 0.72), rng.uniform(0.08, 0.22), 0.08),
              LBL_FRUIT_FALLEN, "fallen")
    out.append(p.build())

    return [o for o in out if o]


def instrumented_tree(inst_name, model, x, y, z, yaw, body_parts, body_mesh, cfg_height,
                      trunk_r, apples):
    """계측 블록: 나무 몸체(인라인) + 과실마다 최상위 apple <include>.

    배경목과 달리 body 메시(과실 없음)를 써야 과실이 이중으로 생기지 않는다.
    <include> 는 model.config 기본 SDF(full)만 로드하므로, body 를 쓰려면
    tree_body.glb 서브메시를 참조하는 <model> 을 인라인으로 조립한다.
    과실은 각각 최상위 <include> 여야 panoptic 인스턴스가 분리된다(2026-07-25 실측).
    z 는 경사면 지형 높이. 과실도 z 만큼 올려 지면 위에 얹는다.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    visuals = "".join(f"""        <visual name="{bp['part']}">
          <geometry><mesh>
            <uri>model://{model}/meshes/{body_mesh}</uri>
            <submesh><name>{bp['part']}</name></submesh>
          </mesh></geometry>
          <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
            <label>{bp['label']}</label>
          </plugin>
        </visual>
""" for bp in body_parts)

    parts = [f"""    <model name="{inst_name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">
{visuals}        <collision name="trunk_collision">
          <pose>0 0 {cfg_height / 2:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>{trunk_r:.4f}</radius><length>{cfg_height:.3f}</length></cylinder></geometry>
        </collision>
      </link>
    </model>
"""]
    for a in apples:
        lx, ly, lz = a["local_xyz"]
        wx = x + (lx * c - ly * s)
        wy = y + (lx * s + ly * c)
        label = a["label_id"]        # 40 healthy / 41 diseased
        parts.append(f"""    <include>
      <name>{inst_name}__{a['apple_id']}</name>
      <uri>model://apple</uri>
      <pose>{wx:.3f} {wy:.3f} {z + lz:.3f} 0 0 0</pose>
      <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
        <label>{label}</label>
      </plugin>
    </include>
""")
    return "".join(parts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rows", type=int, default=4)
    ap.add_argument("--trees-per-row", type=int, default=20)
    ap.add_argument("--instrumented-rows", type=int, default=2,
                    help="가운데 몇 개 행을 계측 블록으로 할지")
    ap.add_argument("--instrumented-trees", type=int, default=10,
                    help="계측 행에서 몇 그루에 과실별 인스턴스를 붙일지")
    ap.add_argument("--models-dir", default="sim/models")
    ap.add_argument("--terrain-model", default="orchard_terrain")
    ap.add_argument("--out", default="sim/worlds/orchard.sdf")
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--detail", type=int, default=2, choices=[0, 1, 2],
                    help="행 디테일 수준. 0=없음 1=지주·와이어·청경대 2=+관수호스·잡초·낙과")
    ap.add_argument("--posts", action="store_true",
                    help="(구버전 호환) --detail 1 이상과 동일")
    ap.add_argument("--robot", default=None,
                    help="스폰할 로봇 모델명 (예: scout_mini_mid70). 통로 시작점에 배치")
    ap.add_argument("--step-size", type=float, default=0.002,
                    help="물리 스텝 s. 4ms 를 넘기면 휠 접촉이 불안정해진다")
    ap.add_argument("--collision-detector", default="bullet",
                    choices=["bullet", "ode", "dart", "fcl"],
                    help="하이트맵에는 bullet 권장 (설계서 §4.2)")
    ap.add_argument("--environment", action="store_true",
                    help="나무 영역 밖 방풍림·창고·물탱크 배치")
    ap.add_argument("--flip-x", action="store_true", help="지형 x 샘플링 뒤집기 (경사 방향 안 맞을 때)")
    ap.add_argument("--flip-y", action="store_true", help="지형 y 샘플링 뒤집기")
    for k, v in DEFAULTS.items():
        ap.add_argument(f"--{k.replace('_', '-')}", type=type(v), default=v)
    args = ap.parse_args()

    cfg = {k: getattr(args, k) for k in DEFAULTS}
    rng = random.Random(args.seed)
    models_dir = os.path.abspath(args.models_dir)

    trees = discover_trees(models_dir)
    if not trees:
        raise SystemExit(f"{models_dir} 에 apple_tree_* 모델이 없습니다. gen_tree.py 를 먼저 실행하세요.")
    bg_pool = [t for t in trees if t["full"]]
    body_pool = [t for t in trees if t["body"]]
    if not bg_pool or not body_pool:
        raise SystemExit("배경용(model_full.sdf) 또는 계측용(model.sdf) 모델이 부족합니다.")

    # ── 경사면 지형 로드 ────────────────────────────────────────────────
    terrain = Terrain(models_dir, args.terrain_model)
    zf = lambda x, y: terrain.z(x, y, args.flip_x, args.flip_y)
    if not terrain.ok:
        print("[gen_world]   ⚠ 경사면 높이필드 없음 → 평지(z=0)로 배치. "
              "gen_heightmap.py 를 먼저 실행하면 경사면에 앉습니다.")
    else:
        # 계단식 지형은 격자에 맞춰 만들어졌다. 격자가 어긋나면 테라스 경계와
        # 수목열이 안 맞아 나무가 법면 한가운데 서게 된다 — 조용히 잘못되므로 막는다.
        mism = [(k, terrain.m.get(k), getattr(args, k, None))
                for k in ("rows", "trees_per_row", "row_spacing", "tree_spacing")
                if terrain.m.get(k) is not None
                and abs(float(terrain.m[k]) - float(getattr(args, k))) > 1e-6]
        if mism:
            det = ", ".join(f"{k}: 지형 {a} vs 월드 {b}" for k, a, b in mism)
            raise SystemExit(
                f"[gen_world] ✘ 지형과 월드의 격자가 다릅니다 ({det}).\n"
                f"    같은 인자로 gen_heightmap.py 를 다시 실행하세요:\n"
                f"    python3 scripts/gen_heightmap.py --rows {args.rows} "
                f"--trees-per-row {args.trees_per_row} "
                f"--row-spacing {args.row_spacing} --tree-spacing {args.tree_spacing}")

    # ── 배치 격자 ──────────────────────────────────────────────────────
    R, T = args.rows, args.trees_per_row
    row_w = (R - 1) * cfg["row_spacing"]
    col_l = (T - 1) * cfg["tree_spacing"]
    x0 = -row_w / 2
    y0 = -col_l / 2

    inst_center_rows = set()
    if args.instrumented_rows > 0:
        mid = R // 2
        half = args.instrumented_rows // 2
        inst_center_rows = set(range(max(0, mid - half),
                                     min(R, mid - half + args.instrumented_rows)))

    body = []
    stats = dict(bg=0, instrumented=0, apples=0, missing=0, posts=0)

    for r in range(R):
        rx = x0 + r * cfg["row_spacing"]
        is_inst_row = r in inst_center_rows
        for t in range(T):
            ty = y0 + t * cfg["tree_spacing"]
            if rng.random() < cfg["missing_prob"]:
                stats["missing"] += 1
                continue
            jx = rx + rng.gauss(0, cfg["pos_jitter"])
            jy = ty + rng.gauss(0, cfg["pos_jitter"])
            yaw = rng.gauss(0, cfg["yaw_jitter"])

            tz = zf(jx, jy)
            if is_inst_row and t < args.instrumented_trees:
                pick = rng.choice(body_pool)
                gt = json.load(open(os.path.join(models_dir, pick["name"], "ground_truth.json")))
                inst = f"{pick['name']}__r{r}t{t}"
                trunk_r = gt["geometry"]["trunk_base_dia"] / 2
                body.append(instrumented_tree(
                    inst, pick["name"], jx, jy, tz, yaw,
                    gt["body_parts"], gt.get("body_mesh", "tree_body.glb"),
                    gt["geometry"]["height"], trunk_r, gt["apples"]))
                stats["instrumented"] += 1
                stats["apples"] += len(gt["apples"])
            else:
                pick = rng.choice(bg_pool)
                name = f"{pick['name']}__r{r}t{t}"
                # 배경목: model.config 기본 SDF(full, 과실 구워넣음)를 <include>
                body.append(bg_tree_include(name, pick["name"], jx, jy, tz, yaw))
                stats["bg"] += 1


    # 행 디테일 — 지주/와이어, 청경대, 관수호스, 잡초, 낙과
    if args.detail > 0:
        rd = build_row_details(cfg, terrain, x0, y0, R, T,
                               args.flip_x, args.flip_y, rng, args.detail)
        body.append("\n    <!-- 행 디테일 (지주·와이어·청경대·관수·잡초·낙과) -->\n")
        body.extend(rd)
        stats["rowdetail"] = len(rd)

    # 환경 오브젝트 (나무 영역 밖)
    if args.environment:
        env = build_environment(cfg, terrain, x0, x0 + row_w, y0, y0 + col_l,
                                args.flip_x, args.flip_y, rng, args.detail)
        body.append("\n    <!-- 환경 오브젝트 -->\n")
        body.extend(env)
        stats["env"] = len(env)

    # 로봇 스폰 — 첫 통로(0열과 1열 사이) 시작점, 선회 구간에서 진입
    robot_block = ""
    if args.robot:
        spawn_x = x0 + cfg["row_spacing"] / 2          # 0열과 1열 사이 통로 중앙
        spawn_y = y0 - cfg["headland"] / 2             # 선회 구간
        spawn_z = zf(spawn_x, spawn_y) + 0.20          # 경사면 위 + 여유
        robot_block = f"""    <include>
      <name>{args.robot}</name>
      <uri>model://{args.robot}</uri>
      <pose>{spawn_x:.3f} {spawn_y:.3f} {spawn_z:.3f} 0 0 1.5708</pose>
    </include>
"""

    world_name = f"orchard_{R}x{T}"
    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        f.write(sdf_header(world_name, cfg, args.terrain_model,
                           step_size=args.step_size,
                           collision_detector=args.collision_detector))
        f.write(f"\n    <!-- 나무 {stats['bg']} 배경 + {stats['instrumented']} 계측"
                f" / 결주 {stats['missing']} / 과실 인스턴스 {stats['apples']} -->\n")
        f.writelines(body)
        if robot_block:
            f.write("\n    <!-- 로봇 -->\n")
            f.write(robot_block)
        f.write(sdf_footer())

    total_entities = (stats["bg"] + stats["instrumented"] + stats["apples"]
                      + stats.get("env", 0) + stats.get("rowdetail", 0) + 1)
    print(f"[gen_world] {out}")
    print(f"[gen_world]   {R}행 x {T}주  (열간 {cfg['row_spacing']} / 주간 {cfg['tree_spacing']} m)")
    print(f"[gen_world]   배경목 {stats['bg']} / 계측목 {stats['instrumented']}"
          f" / 결주 {stats['missing']}")
    print(f"[gen_world]   과실 인스턴스 {stats['apples']:,} (계측 블록만)")
    if stats.get("rowdetail"):
        print(f"[gen_world]   행 디테일 모델 {stats['rowdetail']} "
              f"(지주·3단와이어·청경대·점적관수·잡초·낙과)")
    if stats.get("env"):
        print(f"[gen_world]   환경 오브젝트 모델 {stats['env']} "
              f"(방풍림·울타리·창고·물탱크·농기계·컨테이너·전신주·돌무더기·진입로)")
    if terrain.ok:
        # 통로(테라스) 중심의 지면 높이 — 계단이 실제로 생겼는지 바로 보인다
        centers = [zf(x0 + (k + 0.5) * cfg["row_spacing"], 0.0) for k in range(R - 1)]
        steps = [centers[i + 1] - centers[i] for i in range(len(centers) - 1)]
        if terrain.m.get("profile", "").startswith("terraced"):
            print(f"[gen_world]   계단식 지형: 통로 {len(centers)}단, "
                  f"지면 {centers[0]:.2f} → {centers[-1]:.2f} m")
            print(f"[gen_world]   통로간 단차: "
                  + " ".join(f"{s * 100:.0f}" for s in steps) + " cm")
        else:
            print(f"[gen_world]   지형: 지면 {centers[0]:.2f} → {centers[-1]:.2f} m")
    print(f"[gen_world]   대략 총 엔티티 {total_entities:,}")
    print(f"[gen_world]   과수원 {row_w:.1f} x {col_l:.1f} m + 선회 {cfg['headland']} m")


if __name__ == "__main__":
    main()
