#!/usr/bin/env python3
"""
세장방추형 사과나무 절차적 생성기 (Blender headless)

    blender --background --python scripts/gen_tree.py -- --seed 42 --severity 7 --out sim/models

설계서 §4.3 / §8.8 근거. 다운로드 가능한 과수 에셋이 존재하지 않으므로 직접 생성한다.
(Gazebo Fuel 을 13개 검색어로 조회한 결과 apple/orchard/vineyard/crop 모델 0개)

출력 (나무 1그루당):
    <out>/apple_tree_<id>/meshes/tree_body.glb   주간 + 측지 + 잎카드        (과실 없음)
    <out>/apple_tree_<id>/meshes/tree_full.glb   위 + 과실을 메시에 구워넣음  (배경 행용)
    <out>/apple_tree_<id>/ground_truth.json      severity·과실 위치 정답
    <out>/apple/meshes/apple.glb                 공유 과실 메시 1개 (80 tris)

왜 두 가지 메시인가 — 2026-07-25 실측(docs/findings/2026-07-25-label-instance-separation.md):
panoptic 인스턴스 분리는 **최상위 non-nested 모델 단위로만** 일어난다. 따라서
  · 계측 블록 : tree_body.glb + 과실마다 최상위 <include> → 과실별 인스턴스 ID 확보
  · 배경 행   : tree_full.glb 한 개              → 엔티티 1개, semantic 라벨만
"""
import json
import math
import os
import random
import sys

import bpy
import bmesh
from mathutils import Vector, Matrix

# ── 세장방추형 기본 제원 (설계서 §4.1) ──────────────────────────────────────
DEFAULTS = dict(
    height=3.20,            # 수고 m
    canopy_width=1.20,      # 수관 폭 m
    trunk_base_dia=0.070,   # 주간 기부 직경 m
    trunk_top_dia=0.020,    # 주간 선단 직경 m
    trunk_noise=0.010,      # 축방향 노이즈 σ m
    n_feathers=28,          # 측지 개수
    feather_z_min=0.70,     # 수관 하단고 m
    feather_z_max=3.00,
    feather_len_bottom=0.60,  # 하단 측지 길이 (수관 반경을 결정)
    feather_len_top=0.30,     # 상단 측지 길이
    feather_pitch_min=-30.0,  # 하수각 deg
    feather_pitch_max=-10.0,
    golden_angle=137.5,     # 측지 방위각 층화
    n_apples=60,            # 주당 과실 수
    apple_dia=0.075,        # 과실 직경 m
    apple_z_min=0.80,
    apple_z_max=3.00,
    apple_r_min=0.15,       # 주간축에서의 반경 m
    apple_r_max=0.55,
    leaf_cards_per_feather=9,   # 측지당 잎-클러스터 카드
    leaf_card_w=0.16,           # 잎 클러스터 카드 크기 m
    leaf_card_h=0.13,
    severity=0,             # 0~10
    severity_threshold=6,   # §8.8 D13: 0~5 정상 / 6~10 병충해
)

# ── 라벨 ID (설계서 §8.3) ───────────────────────────────────────────────────
LABEL_TRUNK = 20
LABEL_BRANCH = 21
LABEL_LEAF_HEALTHY = 30
LABEL_LEAF_DISEASED = 31          # 갈색무늬병 대표
LABEL_FRUIT_HEALTHY = 40
LABEL_FRUIT_DISEASED = 41         # 탄저병 대표


# ═══════════════════════════════════════════════════════════════════════════
# severity → 외형 매핑.  이 함수가 정답 생성의 핵심이므로 명시적으로 문서화한다.
# ═══════════════════════════════════════════════════════════════════════════
def severity_to_appearance(severity: int) -> dict:
    """severity 0~10 을 잎/과실 병징 비율로 환산한다.

    근거: 기존 파이프라인(§8.8)의 VLM 채점 정의 —
        1  = 매우 경미·미용상 결함
        5  = 식물 일부에 영향을 주는 중간 결함
        10 = 심각·광범위·치명적 손상
    잎이 과실보다 먼저·넓게 발현하므로 과실 발현은 severity 3 이후로 지연시킨다.
    """
    s = max(0, min(10, int(severity)))
    return {
        "diseased_leaf_fraction": round(min(1.0, s / 10.0) * 0.60, 4),
        "diseased_fruit_fraction": round(max(0.0, (s - 3) / 7.0) * 0.40, 4),
    }


# ═══════════════════════════════════════════════════════════════════════════
# 기하 헬퍼 — Blender op 을 쓰지 않고 정점/면을 직접 만든다 (결정론·속도)
# ═══════════════════════════════════════════════════════════════════════════
def _frame(axis: Vector):
    """axis 를 z 로 하는 정규직교 기저를 만든다."""
    z = axis.normalized()
    ref = Vector((0, 0, 1)) if abs(z.z) < 0.9 else Vector((1, 0, 0))
    x = ref.cross(z).normalized()
    return x, z.cross(x).normalized(), z


def tapered_cylinder(p0, p1, r0, r1, segments, rings=1, jitter=0.0, rng=None):
    """p0→p1 을 잇는 테이퍼 원기둥. (verts, faces) 반환, 양 끝은 닫는다."""
    p0, p1 = Vector(p0), Vector(p1)
    ex, ey, ez = _frame(p1 - p0)
    verts, faces = [], []

    for i in range(rings + 1):
        t = i / rings
        c = p0.lerp(p1, t)
        if jitter and rng and 0 < i < rings:
            c = c + ex * rng.gauss(0, jitter) + ey * rng.gauss(0, jitter)
        r = r0 + (r1 - r0) * t
        for k in range(segments):
            a = 2 * math.pi * k / segments
            verts.append(tuple(c + ex * (r * math.cos(a)) + ey * (r * math.sin(a))))

    for i in range(rings):
        for k in range(segments):
            a = i * segments + k
            b = i * segments + (k + 1) % segments
            faces.append((a, b, b + segments, a + segments))

    base = len(verts)
    verts.append(tuple(p0))
    verts.append(tuple(p1))
    for k in range(segments):
        faces.append((k, (k + 1) % segments, base))
        top = rings * segments
        faces.append((top + (k + 1) % segments, top + k, base + 1))
    return verts, faces


def quad_card(center, normal, w, h, roll=0.0):
    """알파 테스트 잎-클러스터 카드 (쿼드 1장 = 2 tris)."""
    ex, ey, _ = _frame(Vector(normal))
    if roll:
        c, s = math.cos(roll), math.sin(roll)
        ex, ey = ex * c + ey * s, -ex * s + ey * c
    c = Vector(center)
    return [tuple(c + ex * (sx * w / 2) + ey * (sy * h / 2))
            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))], [(0, 1, 2, 3)]


def icosphere(radius, subdivisions=2):
    """정이십면체 세분 구. subdivisions=2 → 80 tris (설계서 예산과 일치)."""
    t = (1 + 5 ** 0.5) / 2
    verts = [Vector(v) for v in (
        (-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0),
        (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
        (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1))]
    faces = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
             (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
             (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
             (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]

    for _ in range(subdivisions - 1):
        cache, new_faces = {}, []

        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in cache:
                verts.append((verts[a] + verts[b]).normalized())
                cache[key] = len(verts) - 1
            return cache[key]

        for a, b, c in faces:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            new_faces += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        faces = new_faces

    return [tuple(v.normalized() * radius) for v in verts], faces


# ═══════════════════════════════════════════════════════════════════════════
# Blender 오브젝트 조립
# ═══════════════════════════════════════════════════════════════════════════
def clear_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def make_material(name, rgba, alpha_blend=False):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs["Base Color"].default_value = rgba
        if "Roughness" in bsdf.inputs:
            bsdf.inputs["Roughness"].default_value = 0.85
    mat.diffuse_color = rgba
    if alpha_blend:
        mat.blend_method = "CLIP"
    return mat


def add_mesh(name, verts, faces, material):
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], faces)
    mesh.validate()
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    obj.data.materials.append(material)
    bpy.context.collection.objects.link(obj)
    return obj


def join_into(name, pieces, material):
    """(verts, faces) 조각들을 하나의 오브젝트로 합친다."""
    all_v, all_f, off = [], [], 0
    for v, f in pieces:
        all_v.extend(v)
        all_f.extend([tuple(i + off for i in face) for face in f])
        off += len(v)
    return add_mesh(name, all_v, all_f, material)


def tri_count(obj):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    n = len(bm.faces)
    bm.free()
    return n


# ═══════════════════════════════════════════════════════════════════════════
# 나무 생성
# ═══════════════════════════════════════════════════════════════════════════
def feather_length(p, cfg):
    """수관 상단으로 갈수록 측지가 짧아진다 → 세장방추(원뿔) 수형."""
    return cfg["feather_len_bottom"] + (cfg["feather_len_top"] - cfg["feather_len_bottom"]) * p


def build_tree(cfg, rng):
    app = severity_to_appearance(cfg["severity"])

    mat_bark = make_material("bark", (0.32, 0.24, 0.17, 1))
    mat_leaf_h = make_material("leaf_healthy", (0.18, 0.42, 0.14, 1), alpha_blend=True)
    mat_leaf_d = make_material("leaf_diseased", (0.45, 0.34, 0.12, 1), alpha_blend=True)
    mat_fruit_h = make_material("fruit_healthy", (0.72, 0.10, 0.09, 1))
    mat_fruit_d = make_material("fruit_diseased", (0.38, 0.22, 0.14, 1))

    # ── 주간 ────────────────────────────────────────────────────────────
    trunk = tapered_cylinder(
        (0, 0, 0), (0, 0, cfg["height"]),
        cfg["trunk_base_dia"] / 2, cfg["trunk_top_dia"] / 2,
        segments=10, rings=8, jitter=cfg["trunk_noise"], rng=rng)
    obj_trunk = join_into("trunk", [trunk], mat_bark)

    # ── 측지 (황금각 층화 배치) ─────────────────────────────────────────
    feathers, feather_axes = [], []
    for i in range(cfg["n_feathers"]):
        p = i / max(1, cfg["n_feathers"] - 1)
        z = cfg["feather_z_min"] + (cfg["feather_z_max"] - cfg["feather_z_min"]) * p
        az = math.radians(cfg["golden_angle"] * i)
        pitch = math.radians(rng.uniform(cfg["feather_pitch_min"], cfg["feather_pitch_max"]))
        ln = feather_length(p, cfg) * rng.uniform(0.85, 1.15)

        base_r = (cfg["trunk_base_dia"] / 2) * (1 - p) + (cfg["trunk_top_dia"] / 2) * p
        p0 = Vector((base_r * math.cos(az), base_r * math.sin(az), z))
        d = Vector((math.cos(az) * math.cos(pitch),
                    math.sin(az) * math.cos(pitch),
                    math.sin(pitch)))
        p1 = p0 + d * ln
        feathers.append(tapered_cylinder(p0, p1, 0.012, 0.004, segments=6, rings=2))
        feather_axes.append((p0, p1, ln))
    obj_feathers = join_into("feathers", feathers, mat_bark)

    # ── 잎-클러스터 카드 ────────────────────────────────────────────────
    healthy_cards, diseased_cards = [], []
    for (p0, p1, ln) in feather_axes:
        for j in range(cfg["leaf_cards_per_feather"]):
            t = rng.uniform(0.25, 1.0)
            c = p0.lerp(p1, t) + Vector((rng.gauss(0, 0.05),
                                         rng.gauss(0, 0.05),
                                         rng.gauss(0, 0.04)))
            nrm = Vector((rng.gauss(0, 1), rng.gauss(0, 1), rng.gauss(0, 1)))
            if nrm.length < 1e-6:
                nrm = Vector((0, 0, 1))
            card = quad_card(c, nrm, cfg["leaf_card_w"], cfg["leaf_card_h"],
                             roll=rng.uniform(0, math.pi))
            (diseased_cards if rng.random() < app["diseased_leaf_fraction"]
             else healthy_cards).append(card)

    obj_leaf_h = join_into("leaf_healthy", healthy_cards, mat_leaf_h) if healthy_cards else None
    obj_leaf_d = join_into("leaf_diseased", diseased_cards, mat_leaf_d) if diseased_cards else None

    # ── 과실 배치 (엽군 내부 기각 샘플링) ───────────────────────────────
    apples = _place_apples(cfg, rng, app)

    return dict(
        trunk=obj_trunk, feathers=obj_feathers,
        leaf_healthy=obj_leaf_h, leaf_diseased=obj_leaf_d,
        apples=apples, appearance=app,
        materials=dict(fruit_h=mat_fruit_h, fruit_d=mat_fruit_d),
    )


def _place_apples(cfg, rng, app):
    """과실을 수관 내부에 배치한다.

    표면에 얹지 않고 **엽군 내부**에 넣는 것이 핵심이다. 단면 가시율 목표
    55~65%(설계서 §4.1)를 맞추기 위한 것으로, 표면 배치는 가시율이 90%를 넘어
    실제 과수원(40.85~79.83%)과 동떨어진 데이터를 만든다.
    """
    apples, cluster_id = [], 0
    while len(apples) < cfg["n_apples"]:
        # 착과 군집: 단일 75.3%, 나머지 2~4개 (arXiv:1808.04336)
        size = 1 if rng.random() < 0.753 else rng.randint(2, 4)
        size = min(size, cfg["n_apples"] - len(apples))

        z0 = rng.uniform(cfg["apple_z_min"], cfg["apple_z_max"])
        p = (z0 - cfg["feather_z_min"]) / (cfg["feather_z_max"] - cfg["feather_z_min"])
        r_max = min(cfg["apple_r_max"], feather_length(max(0.0, min(1.0, p)), cfg) * 0.92)
        r0 = rng.uniform(cfg["apple_r_min"], max(cfg["apple_r_min"] + 1e-3, r_max))
        az0 = rng.uniform(0, 2 * math.pi)

        for _ in range(size):
            az = az0 + rng.gauss(0, 0.12)
            r = max(cfg["apple_r_min"], r0 + rng.gauss(0, 0.04))
            z = z0 + rng.gauss(0, 0.06)
            apples.append(dict(
                apple_id=f"a{len(apples):03d}",
                local_xyz=[round(r * math.cos(az), 5),
                           round(r * math.sin(az), 5),
                           round(z, 5)],
                diameter=cfg["apple_dia"],
                cluster_id=cluster_id,
                diseased=rng.random() < app["diseased_fruit_fraction"],
            ))
        cluster_id += 1

    for a in apples:
        a["label_id"] = LABEL_FRUIT_DISEASED if a["diseased"] else LABEL_FRUIT_HEALTHY
    return apples


# ═══════════════════════════════════════════════════════════════════════════
# 익스포트
# ═══════════════════════════════════════════════════════════════════════════
def export_mesh(objs, filepath):
    """선택한 오브젝트만 glTF 바이너리(.glb)로 내보낸다.

    Gazebo 문서는 COLLADA 를 권장하지만 **Ubuntu 24.04 의 Blender 4.0.2 빌드에는
    COLLADA 익스포터가 없다** (`bpy.ops.wm.collada_export` 부재, WITH_OPENCOLLADA 미포함).
    사용 가능한 것은 obj / ply / fbx / gltf / x3d / stl 뿐이다.

    glTF 를 고른 이유: gz-common5-graphics 가 libassimp.so.5 (5.3.1) 에 링크돼 있고
    assimp 가 glTF 를 지원한다. 또한 .glb 는 단일 바이너리라 텍스처 동봉이 쉽고
    PBR 머티리얼이 그대로 넘어간다 — 병징 알베도 맵(§8.8)에 필요하다.
    """
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        if o:
            o.select_set(True)
    bpy.context.view_layer.objects.active = next(o for o in objs if o)
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=filepath,
        export_format="GLB",
        use_selection=True,
        export_apply=True,
        export_yup=False,          # Gazebo 는 Z-up. 축 변환을 하지 않는다
        export_materials="EXPORT",
    )
    return filepath


def bake_apples(tree, cfg):
    """과실 60개를 하나의 메시로 구워넣는다 (배경 행용)."""
    sphere_v, sphere_f = icosphere(cfg["apple_dia"] / 2, subdivisions=2)
    healthy, diseased = [], []
    for a in tree["apples"]:
        c = Vector(a["local_xyz"])
        piece = ([tuple(Vector(v) + c) for v in sphere_v], sphere_f)
        (diseased if a["diseased"] else healthy).append(piece)
    objs = []
    if healthy:
        objs.append(join_into("fruit_healthy", healthy, tree["materials"]["fruit_h"]))
    if diseased:
        objs.append(join_into("fruit_diseased", diseased, tree["materials"]["fruit_d"]))
    return objs


# ═══════════════════════════════════════════════════════════════════════════
# SDF 모델 패키지 생성 — 바로 <include> 할 수 있는 형태로 내보낸다
# ═══════════════════════════════════════════════════════════════════════════
MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{desc}</description>
</model>
"""

# 부위별 semantic 라벨을 붙이려면 glb 안의 오브젝트를 <submesh> 로 골라야 한다.
# 2026-07-25 실측으로 동작 확인함 (trunk/feathers/leaf_healthy/leaf_diseased 4부위 분리).
_VISUAL = """      <visual name="{part}">
        <geometry><mesh>
          <uri>model://{model}/meshes/{mesh}</uri>
          <submesh><name>{part}</name></submesh>
        </mesh></geometry>
        <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
          <label>{label}</label>
        </plugin>
      </visual>
"""


def write_tree_model_sdf(tree_dir, name, cfg, parts, variant):
    """variant='body' → 과실 없음 (계측 블록용) / 'full' → 과실 포함 (배경 행용)."""
    mesh = "tree_body.glb" if variant == "body" else "tree_full.glb"
    visuals = "".join(
        _VISUAL.format(part=p, label=l, model=name, mesh=mesh) for p, l in parts)

    # 충돌은 주간 원기둥 하나뿐. 잎·과실에는 collision 을 두지 않는다.
    # Baylands 사례: 시각 메시를 충돌로 쓰면 RTF 5%, 프리미티브로 바꾸면 90% (설계서 §9-1)
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
{visuals}      <collision name="trunk_collision">
        <pose>0 0 {cfg['height'] / 2:.3f} 0 0 0</pose>
        <geometry><cylinder>
          <radius>{cfg['trunk_base_dia'] / 2:.4f}</radius>
          <length>{cfg['height']:.3f}</length>
        </cylinder></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
    suffix = "" if variant == "body" else "_full"
    with open(os.path.join(tree_dir, f"model{suffix}.sdf"), "w") as f:
        f.write(sdf)


def write_apple_model(out_root, cfg):
    """공유 과실 모델. 계측 블록에서 **최상위 <include>** 로 참조된다.

    최상위여야 하는 이유는 2026-07-25 실측 결과다 — panoptic 인스턴스 분리는
    최상위 non-nested 모델 단위로만 일어나므로, 나무 안에 넣으면 과실이 전부
    인스턴스 1개로 뭉개진다.
    """
    d = os.path.join(out_root, "apple")
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "model.config"), "w") as f:
        f.write(MODEL_CONFIG.format(name="apple",
                                    desc=f"공유 사과 메시 ⌀{cfg['apple_dia']} m, 80 tris"))
    with open(os.path.join(d, "model.sdf"), "w") as f:
        f.write(f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="apple">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><mesh><uri>model://apple/meshes/apple.glb</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
""")


# ═══════════════════════════════════════════════════════════════════════════
def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    import argparse
    ap = argparse.ArgumentParser(prog="gen_tree.py")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--severity", type=int, default=0, choices=range(0, 11),
                    help="0~10. 0~5 정상 / 6~10 병충해 (설계서 D13)")
    ap.add_argument("--out", default="sim/models")
    ap.add_argument("--tree-id", default=None)
    for k, v in DEFAULTS.items():
        if k in ("severity", "severity_threshold"):
            continue
        ap.add_argument(f"--{k.replace('_', '-')}", type=type(v), default=v)
    return ap.parse_args(argv)


def main():
    args = parse_args()
    cfg = dict(DEFAULTS)
    for k in DEFAULTS:
        if hasattr(args, k):
            cfg[k] = getattr(args, k)
    cfg["severity"] = args.severity

    tree_id = args.tree_id or f"apple_tree_s{args.severity:02d}_{args.seed:04d}"
    out_root = os.path.abspath(args.out)
    tree_dir = os.path.join(out_root, tree_id)
    mesh_dir = os.path.join(tree_dir, "meshes")

    rng = random.Random(args.seed)
    clear_scene()
    tree = build_tree(cfg, rng)

    body_objs = [tree["trunk"], tree["feathers"],
                 tree["leaf_healthy"], tree["leaf_diseased"]]
    body_objs = [o for o in body_objs if o]
    export_mesh(body_objs, os.path.join(mesh_dir, "tree_body.glb"))

    fruit_objs = bake_apples(tree, cfg)
    export_mesh(body_objs + fruit_objs, os.path.join(mesh_dir, "tree_full.glb"))

    # 성능 예산 추적 — 씬을 비우기 전에 센다 (설계서 §9: 그루당 2~5k tris 목표)
    tris = {o.name: tri_count(o) for o in body_objs + fruit_objs}
    tris["_body_total"] = sum(tri_count(o) for o in body_objs)
    tris["_full_total"] = sum(tris[o.name] for o in body_objs + fruit_objs)

    # clear_scene() 이후 Blender 오브젝트 참조는 무효화된다(StructRNA removed).
    # SDF 를 쓸 때 필요한 것은 이름뿐이므로 지금 순수 파이썬 값으로 뽑아둔다.
    body_part_names = [o.name for o in body_objs]
    fruit_part_names = [o.name for o in fruit_objs]

    # 공유 과실 메시 (계측 블록에서 최상위 <include> 로 60번 참조된다)
    clear_scene()
    mat_a = make_material("fruit_healthy", (0.72, 0.10, 0.09, 1))
    sv, sf = icosphere(cfg["apple_dia"] / 2, subdivisions=2)
    apple_obj = add_mesh("apple", sv, sf, mat_a)
    apple_dir = os.path.join(out_root, "apple", "meshes")
    export_mesh([apple_obj], os.path.join(apple_dir, "apple.glb"))

    tris["apple_shared"] = tri_count(apple_obj)

    # ── SDF 모델 패키지 ────────────────────────────────────────────────
    ALL_LABELS = {
        "trunk": LABEL_TRUNK,
        "feathers": LABEL_BRANCH,
        "leaf_healthy": LABEL_LEAF_HEALTHY,
        "leaf_diseased": LABEL_LEAF_DISEASED,
        "fruit_healthy": LABEL_FRUIT_HEALTHY,
        "fruit_diseased": LABEL_FRUIT_DISEASED,
    }
    # severity 에 따라 병징 부위가 없을 수도 있으므로 실제 생성된 것만 넣는다
    body_parts = [(p, ALL_LABELS[p]) for p in body_part_names]
    full_parts = [(p, ALL_LABELS[p]) for p in body_part_names + fruit_part_names]

    with open(os.path.join(tree_dir, "model.config"), "w") as f:
        f.write(MODEL_CONFIG.format(
            name=tree_id,
            desc=f"세장방추형 사과나무 seed={args.seed} severity={cfg['severity']}"))
    write_tree_model_sdf(tree_dir, tree_id, cfg, body_parts, "body")
    write_tree_model_sdf(tree_dir, tree_id, cfg, full_parts, "full")
    write_apple_model(out_root, cfg)

    gt = dict(
        tree_id=tree_id,
        seed=args.seed,
        severity=cfg["severity"],
        severity_threshold=cfg["severity_threshold"],
        disease_label=("DEFECT" if cfg["severity"] >= cfg["severity_threshold"]
                       else "NORMAL"),
        appearance=tree["appearance"],
        geometry=dict(height=cfg["height"], canopy_width=cfg["canopy_width"],
                      trunk_base_dia=cfg["trunk_base_dia"],
                      n_feathers=cfg["n_feathers"]),
        label_ids=dict(trunk=LABEL_TRUNK, branch=LABEL_BRANCH,
                       leaf_healthy=LABEL_LEAF_HEALTHY,
                       leaf_diseased=LABEL_LEAF_DISEASED,
                       fruit_healthy=LABEL_FRUIT_HEALTHY,
                       fruit_diseased=LABEL_FRUIT_DISEASED),
        n_apples=len(tree["apples"]),
        n_apples_diseased=sum(1 for a in tree["apples"] if a["diseased"]),
        tri_counts=tris,
        apples=tree["apples"],
    )
    with open(os.path.join(tree_dir, "ground_truth.json"), "w") as f:
        json.dump(gt, f, indent=2, ensure_ascii=False)

    print(f"[gen_tree] {tree_id}")
    print(f"[gen_tree]   severity {cfg['severity']} → {gt['disease_label']}"
          f"  (임계 {cfg['severity_threshold']})")
    print(f"[gen_tree]   잎 병징 비율 {tree['appearance']['diseased_leaf_fraction']:.2%}"
          f" / 과실 병징 비율 {tree['appearance']['diseased_fruit_fraction']:.2%}")
    print(f"[gen_tree]   과실 {gt['n_apples']}개 (병징 {gt['n_apples_diseased']}개)")
    print(f"[gen_tree]   삼각형  body {tris['_body_total']:,} / full {tris['_full_total']:,}"
          f"  (공유 과실 {tris['apple_shared']})")
    print(f"[gen_tree]   → {tree_dir}")


if __name__ == "__main__":
    main()
