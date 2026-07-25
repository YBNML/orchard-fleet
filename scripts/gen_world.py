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
def sdf_header(world_name, cfg, terrain_model):
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

    <physics name="2ms" type="dart">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
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


def bg_tree_include(name, model, x, y, yaw):
    """배경목: tree_full(과실 구워넣음) 모델을 통째로 <include>.

    model.config 의 기본 SDF 가 model.sdf(=full 버전)이므로 <include> 하나로
    과실까지 렌더링된다. 나무 전체가 하나의 semantic 라벨을 갖는다(인스턴스 불필요).
    """
    return f"""    <include>
      <name>{name}</name>
      <uri>model://{model}</uri>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.4f}</pose>
    </include>
"""


def instrumented_tree(inst_name, model, x, y, yaw, body_parts, body_mesh, cfg_height,
                      trunk_r, apples):
    """계측 블록: 나무 몸체(인라인) + 과실마다 최상위 apple <include>.

    배경목과 달리 body 메시(과실 없음)를 써야 과실이 이중으로 생기지 않는다.
    <include> 는 model.config 기본 SDF(full)만 로드하므로, body 를 쓰려면
    tree_body.glb 서브메시를 참조하는 <model> 을 인라인으로 조립한다.
    과실은 각각 최상위 <include> 여야 panoptic 인스턴스가 분리된다(2026-07-25 실측).
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
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.4f}</pose>
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
      <pose>{wx:.3f} {wy:.3f} {lz:.3f} 0 0 0</pose>
      <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
        <label>{label}</label>
      </plugin>
    </include>
""")
    return "".join(parts)


def trellis_posts(models_dir, cfg, row_x, y0, y1):
    """열을 따라 일정 간격으로 지주를 세운다. collision 없는 얇은 원기둥 visual."""
    posts = []
    y = y0
    idx = 0
    while y <= y1:
        posts.append(f"""    <model name="post_r{row_x:.1f}_{idx}">
      <static>true</static>
      <pose>{row_x:.3f} {y:.3f} 1.48 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.0375</radius><length>2.96</length></cylinder></geometry>
          <material><ambient>0.35 0.28 0.2 1</ambient><diffuse>0.35 0.28 0.2 1</diffuse></material>
          <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>12</label></plugin>
        </visual>
      </link>
    </model>
""")
        y += cfg["post_spacing"]
        idx += 1
    return "".join(posts)


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
    ap.add_argument("--posts", action="store_true", help="지주 생성")
    ap.add_argument("--robot", default=None,
                    help="스폰할 로봇 모델명 (예: scout_mini_mid70). 통로 시작점에 배치")
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

            if is_inst_row and t < args.instrumented_trees:
                pick = rng.choice(body_pool)
                gt = json.load(open(os.path.join(models_dir, pick["name"], "ground_truth.json")))
                inst = f"{pick['name']}__r{r}t{t}"
                trunk_r = gt["geometry"]["trunk_base_dia"] / 2
                body.append(instrumented_tree(
                    inst, pick["name"], jx, jy, yaw,
                    gt["body_parts"], gt.get("body_mesh", "tree_body.glb"),
                    gt["geometry"]["height"], trunk_r, gt["apples"]))
                stats["instrumented"] += 1
                stats["apples"] += len(gt["apples"])
            else:
                pick = rng.choice(bg_pool)
                name = f"{pick['name']}__r{r}t{t}"
                # 배경목: model.config 기본 SDF(full, 과실 구워넣음)를 <include>
                body.append(bg_tree_include(name, pick["name"], jx, jy, yaw))
                stats["bg"] += 1

        if args.posts:
            body.append(trellis_posts(models_dir, cfg,
                                      rx, y0 - cfg["tree_spacing"], y0 + col_l + cfg["tree_spacing"]))
            stats["posts"] += 1

    # 로봇 스폰 — 첫 통로(0열과 1열 사이) 시작점, 선회 구간에서 진입
    robot_block = ""
    if args.robot:
        spawn_x = x0 + cfg["row_spacing"] / 2          # 0열과 1열 사이 통로 중앙
        spawn_y = y0 - cfg["headland"] / 2             # 선회 구간
        robot_block = f"""    <include>
      <name>{args.robot}</name>
      <uri>model://{args.robot}</uri>
      <pose>{spawn_x:.3f} {spawn_y:.3f} 0.20 0 0 1.5708</pose>
    </include>
"""

    world_name = f"orchard_{R}x{T}"
    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        f.write(sdf_header(world_name, cfg, args.terrain_model))
        f.write(f"\n    <!-- 나무 {stats['bg']} 배경 + {stats['instrumented']} 계측"
                f" / 결주 {stats['missing']} / 과실 인스턴스 {stats['apples']} -->\n")
        f.writelines(body)
        if robot_block:
            f.write("\n    <!-- 로봇 -->\n")
            f.write(robot_block)
        f.write(sdf_footer())

    total_entities = (stats["bg"] + stats["instrumented"] + stats["apples"]
                      + stats["posts"] * ((int(col_l / cfg["post_spacing"]) + 2)))
    print(f"[gen_world] {out}")
    print(f"[gen_world]   {R}행 x {T}주  (열간 {cfg['row_spacing']} / 주간 {cfg['tree_spacing']} m)")
    print(f"[gen_world]   배경목 {stats['bg']} / 계측목 {stats['instrumented']}"
          f" / 결주 {stats['missing']}")
    print(f"[gen_world]   과실 인스턴스 {stats['apples']:,} (계측 블록만)")
    print(f"[gen_world]   대략 총 엔티티 {total_entities:,}")
    print(f"[gen_world]   과수원 {row_w:.1f} x {col_l:.1f} m + 선회 {cfg['headland']} m")


if __name__ == "__main__":
    main()
