#!/usr/bin/env python3
"""앵커 벽 캘리브레이션 — 통로×단별 '겉보기 벽' 위치를 실측해 테이블로

    python3 scripts/47_calibrate_anchor_walls.py   # → maps/orchard_v1/anchor_walls.json
    # 실사(평탄) 월드 — 기하를 farm.json 에서 읽고 통로 캐노피 끝에서 잰다
    python3 scripts/47_calibrate_anchor_walls.py --world orchard_real \
        --farm maps/orchard_real/farm.json --terrain sim/models/orchard_terrain_real \
        --out maps/orchard_real/anchor_walls.json

왜 필요한가 (08-03 실측)
    헤드랜드 접근 중 라이다 원뿔이 보는 '벽'은 주행불가 경계가 아니라
    계단식 램프의 상승면이다. 그 겉보기 벽의 y 는 통로마다 최대 1.9 m
    다른 상수(산포 ≤3 cm)로 실측된다 — 보편 벽(레이캐스트+고정 법면)
    가정이 앵커 만성 편향의 정체였다. 참자세로 로봇을 세워 두 지점씩
    재고 평균을 저장한다. 지형·모델이 바뀌면 다시 돌려야 한다.

**실사(평탄) 월드에서는 '벽이 없는 것'이 정상 결과다.** 계단식의 둑도
울타리도 없고 지면은 정사영상 평지다 — 원뿔 z 대역이 지상 0.3~1.5 m 라
평지 자체는 벽으로 안 잡힌다. 열 말단 앵커 조립체가 잡히면 그것이 이
월드의 '벽'이고, 안 잡히면 그 통로·단의 키를 **아예 쓰지 않는다**.
map_localizer 는 anchor_walls.json 이 없거나 키가 없으면 앵커를 건너뛴다
(기존 설계) — 없는 벽을 억지로 채우는 것보다 건너뛰는 쪽이 안전하다.

전제: Gazebo + livox 브리지 가동, control_agent 는 꺼도 됨.
"""
from __future__ import annotations

import json
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2

sys.path.insert(0, "ros2_ws/src/orchard_sim")
from orchard_sim import transforms as tfu                          # noqa: E402
from orchard_sim.map_localizer import read_xyz                     # noqa: E402


def load_terrain(path):
    """지형 하이트맵 + 메타. 텔레포트 z 를 정하는 데만 쓴다."""
    m = json.load(open(f"{path}/heightmap_meta.json"))
    h = np.load(f"{path}/heightmap.npy")
    return m, h, h.shape[0], m["half"], m["size_x"]


def make_gz_at(H, N_, HALF_, E_):
    def gz_at(x, y):
        fc = np.clip((x + HALF_) / E_ * (N_ - 1), 0, N_ - 1)
        fr = np.clip((y + HALF_) / E_ * (N_ - 1), 0, N_ - 1)
        c0, r0 = int(fc), int(fr)
        c1, r1 = min(c0 + 1, N_ - 1), min(r0 + 1, N_ - 1)
        dc, dr = fc - c0, fr - r0
        return float((H[r0, c0] * (1 - dc) + H[r0, c1] * dc) * (1 - dr)
                     + (H[r1, c0] * (1 - dc) + H[r1, c1] * dc) * dr)
    return gz_at


def farm_probes(farm_path):
    """farm.json → [(k, end, cx, probe_y, yaw), …]  (실사 농장용)

    **부호 규약**(farm.json axes_note): world +y 가 지리적 남이다. 따라서
    '남단' = y 최대 쪽이고 그 단을 향할 때 yaw = +90°, '북단' = y 최소 쪽에
    yaw = −90° 다 — 계단식 월드와 정반대이므로 여기서 한 번에 못박는다.

    관측점은 통로 캐노피 끝에서 **안쪽으로** 2.0·4.0 m 다. 계단식의 31.5·33.5
    (램프 위·패드 위)에 해당하는 자리가 이 월드에는 없다 — 헤드랜드가 2.04 m
    뿐이라 캐노피 밖은 곧 밭 밖이고, 임무가 앵커를 실제로 쓰는 순간은 통로
    끝 접근 중이다.
    """
    f = json.load(open(farm_path))
    n = int(f["rows"])
    hl = float(f["headland_m"])
    x = [float(p[0]) for p in f["row_origins"]]
    y0 = [float(p[1]) + hl for p in f["row_origins"]]                     # 북단(y 최소)
    y1 = [float(p[1]) + float(L) - hl for p, L in zip(f["row_origins"],
                                                     f["row_lengths_m"])]  # 남단(y 최대)
    out = []
    for k in range(n - 1):
        cx = (x[k] + x[k + 1]) / 2.0
        south = min(y1[k], y1[k + 1])       # 이웃 두 열 중 안쪽(짧은 쪽)
        north = max(y0[k], y0[k + 1])
        for d in (2.0, 4.0):
            out.append((k, "south", cx, south - d, +math.pi / 2))
            out.append((k, "north", cx, north + d, -math.pi / 2))
    return out


def main():
    import argparse
    # 로봇 인스턴스별 네임스페이스 (기본 1호기 — 옛 호출 형태 유지)
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="scout01")
    ap.add_argument("--world", default="orchard_10x41")
    ap.add_argument("--farm", default="",
                    help="농장 매니페스트. 주면 통로 중심·관측점을 여기서 만든다")
    ap.add_argument("--terrain", default="sim/models/orchard_terrain")
    ap.add_argument("--out", default="maps/orchard_v1/anchor_walls.json")
    aa = ap.parse_known_args()[0]
    M, H, N_, HALF_, E_ = load_terrain(aa.terrain)
    gz_at = make_gz_at(H, N_, HALF_, E_)
    rclpy.init()
    n = Node(f"anchor_cal_{aa.robot}")
    got, imu_r = [], [None]
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=2)
    n.create_subscription(PointCloud2, f"/{aa.robot}/livox/lidar",
                          lambda m: got.append(read_xyz(m)), qos)

    def cbi(m):
        q = m.orientation
        if q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.5:
            imu_r[0] = tfu.quat_to_matrix(q.x, q.y, q.z, q.w)

    n.create_subscription(Imu, f"/{aa.robot}/imu", cbi, qos)

    def tp(x, y, yaw):
        z = gz_at(x, y) + 0.30
        req = (f'name: "{aa.robot}", '
               f'position: {{x: {x}, y: {y}, z: {z:.2f}}}, '
               f'orientation: {{x: 0, y: 0, z: {math.sin(yaw/2):.6f}, '
               f'w: {math.cos(yaw/2):.6f}}}')
        subprocess.run(["gz", "service", "-s",
                        f"/world/{aa.world}/set_pose",
                        "--reqtype", "gz.msgs.Pose",
                        "--reptype", "gz.msgs.Boolean",
                        "--timeout", "3000", "--req", req],
                       capture_output=True)
        got.clear()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.5:
            rclpy.spin_once(n, timeout_sec=0.05)

    def wall_range():
        if not got:
            return None
        pts = got[-1].copy()
        pts[:, 0] += 0.275                      # 센서→base 전방 오프셋
        R = imu_r[0]
        yaw_b = math.atan2(R[1, 0], R[0, 0])
        c_, s_ = math.cos(-yaw_b), math.sin(-yaw_b)
        Rz = np.array([[c_, -s_, 0], [s_, c_, 0], [0, 0, 1]])
        lvl = pts @ (Rz @ R).T                  # 롤·피치 수평화
        r = np.hypot(lvl[:, 0], lvl[:, 1])
        ang = np.abs(np.arctan2(lvl[:, 1], lvl[:, 0]))
        cone = (ang < math.radians(8.0)) & (r > 2.0) \
            & (lvl[:, 2] > -0.50) & (lvl[:, 2] < 0.75)  # 수관·자기반사 제외
        # 센서 기준 대역 = 지상 0.3~1.5 m. 라이다 0.645→0.80 m (스펙 ④ §3) 재도출.
        if cone.sum() < 40:
            return None
        return float(np.percentile(r[cone], 10))

    # 관측 계획 — 계단식은 지형 메타의 균일 격자, 실사는 farm.json.
    plan = {}                       # (k, end) → [(cx, ty, yaw), …]
    if aa.farm:
        for k, end, cx, ty, yaw in farm_probes(aa.farm):
            plan.setdefault((k, end), []).append((cx, ty, yaw))
    else:
        rows = int(M["rows"])
        x0 = M["x0"]
        n_alleys = rows - 1
        for k in range(n_alleys):
            cx = x0 + (k + 0.5) * 3.5
            for end, sgn in (("north", 1), ("south", -1)):
                # 08-11: 패드 단은 임무가 실제로 앵커를 쓰는 관측점(램프 위 31.5 ·
                # 패드 위 33.5)에서 잰다. 통로 바닥(28·30)에서 재면 진입램프 면이
                # 겉보기 벽으로 잡히거나(5S 산포 1.8 m) 임무 관측과 계통 편차가
                # 생긴다. 패드 규칙: 북단은 k<8, 남단은 k>0 (부스트로피돈 파리티).
                padded = (k < n_alleys - 1) if sgn > 0 else (k > 0)
                for ty in ((31.5, 33.5) if padded else (28.0, 30.0)):
                    plan.setdefault((k, end), []).append(
                        (cx, sgn * ty, sgn * math.pi / 2))
    table, blank = {}, []
    for (k, end) in sorted(plan, key=lambda t: (t[0], t[1])):
        ws = []
        for cx, ty, yaw in plan[(k, end)]:
            tp(cx, ty, yaw)
            w = wall_range()
            if w is not None:
                # 진행 방향(±y)으로 w 만큼 간 자리가 겉보기 벽의 y
                ws.append(ty + math.copysign(w, math.sin(yaw)))
        if ws:
            table[f"{k}:{end}"] = round(float(np.mean(ws)), 2)
            print(f"통로{k} {end}: 벽 y={table[f'{k}:{end}']:+.2f} "
                  f"(산포 {max(ws)-min(ws):.2f})")
        else:
            blank.append(f"{k}:{end}")
    if blank:
        print(f"\n벽 미검출 {len(blank)}/{len(plan)} — 키를 쓰지 않는다 "
              f"(로컬라이저가 그 통로·단에서 앵커를 건너뛴다)")
        print("   " + " ".join(blank))
    json.dump(table, open(aa.out, "w"), indent=1)
    print(f"저장: {aa.out}  (키 {len(table)}개)")


if __name__ == "__main__":
    main()
