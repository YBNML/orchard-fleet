# ADDENDUM DECISION BRIEF
## AgileX SCOUT MINI + Livox MID-70, Korean high-density apple orchard (세장방추형), ROS 2 Jazzy + Gazebo Harmonic

**Standing rule applied throughout: where a verification verdict refutes or corrects a research finding, the VERDICT WINS.** Every place that happened is called out explicitly with a `VERDICT OVERRIDE` tag.

---

## 0. The six verdict overrides that changed the design

| # | Research said | Verdict says | Design consequence |
|---|---|---|---|
| V1 | The 70.4° FOV is what breaks AMCL | AMCL has **no** 360° requirement (`nav2_amcl/src/amcl_node.cpp` computes `angle_min + i*angle_increment`, no full-circle check). The **hard** blockers are: AMCL subscribes to `sensor_msgs/msg/LaserScan` **only**; the non-repetitive rosette does not flatten into a stable planar scan; AMCL also needs a pre-built static occupancy grid, awkward in 노지 | AMCL is **off the critical path entirely**. Not "degraded" — structurally wrong tool. See §2. |
| V2 | Nav2 costmaps don't persist outside FOV; add STVL for persistence | `nav2_costmap_2d/plugins/voxel_layer.cpp`: marking writes LETHAL, clearing happens **only** via `raytraceFreespace()` inside the FOV. Persistence is **already the default**. The failure mode is **stale, un-clearable phantom obstacles**, not missing ones | STVL's role **inverts**: `decay_time` is there to **expire** stale marks, not to retain them. Decay set to **8–15 s**, not the 15–60 s originally proposed. |
| V3 | Use SmacPlannerHybrid (min turning radius 0.4–0.6 m) at headlands | Nav2's own selection table: Hybrid-A* → "Non-circular or Circular **Ackermann**, ... **Legged**". SmacPlanner2D → **circular** differential only (no SE2 footprint check). Only **SmacPlannerLattice** covers "Non-circular Differential" — and its shipped control sets "were generated using the 5cm **Ackermann** files" | **Do not use Hybrid-A***. Headland default = NavFn + RotationShim + MPPI. Upgrade path = SmacPlannerLattice with a **custom-generated differential control set** including rotate-in-place primitives (`rotation_penalty`). |
| V4 | MPPI is the modern choice vs "legacy DWB" | DWB is **not** deprecated (`dwb_core` released at Jazzy 1.3.12). And **MPPI is already the Jazzy default** — `nav2_bringup/params/nav2_params.yaml` has shipped `nav2_mppi_controller::MPPIController` since tag 1.3.0. Default planner is **NavFn**, not Smac. MPPI's own README warns of **jitter in narrow corridors** if `repulsion_weight` is high relative to `inflation_radius` | MPPI is not a "choice", it's the default. The real work is the narrow-corridor tuning, and §2 gives the numbers. |
| V5 | "Rendered lesion texture lacks fine-scale statistics, so sim disease training won't transfer" | **PARTIALLY_TRUE / mechanism refuted.** Klein et al., *Front. Plant Sci.* 15 (2024), doi:10.3389/fpls.2024.1360113 trained **exclusively on Blender-rendered images** and hit **89.6%** on real greenhouse tomato photos; the authors state "photorealism ... is **not the main quality driver**". Training on real PlantVillage photos was "barely better than random guessing" on their real test set. SDFormat 1.11 `<pbr><metal><albedo_map>` takes a **user-supplied photo file** — lesion texture statistics are authorable, not shader-synthesized | The justification for not sim-training the deployed disease model **changes**. It is no longer "renders look fake". It is (a) GSD physics, (b) biological species-ID ceiling, (c) domain shift that real data doesn't escape either. And synthetic-primary training for coarse **symptom** detection becomes defensible if you invest in **content diversity + iterative refinement**, not photorealism. See §5. |
| V6 | Alley free width ~2.3–2.6 m; keep inflation below 0.7 m; 1.475 m clearance per side | Canopy width is **0.9–1.5 m** (Robinson: tree diameter 0.9–1.2 m; 한국사과협회: 나무 폭 1.5 m 내외). At 3.0–3.5 m rows the free alley is **1.6–2.4 m** | Worst case free alley = **2.0 m**, giving **0.725 m** per side, not 1.475 m. `inflation_radius` drops to **0.40 m** (hard cap 0.55 m). This is a materially tighter corridor than the original brief assumed. |

Two further verdict facts that are project-blocking and were not in any finding's headline:

- **The MID-70 has no ROS 2 Jazzy driver path.** `livox_ros_driver2` README "Supported LiDAR list" = **HAP and Mid360 only**. `livox_ros_driver` (v1) = ROS 1, Ubuntu 14/16/18. `livox_ros2_driver` = v0.0.1-beta, Dashing/Foxy/Humble. MID-70 appears in none of them for Jazzy.
- **The MID-70 has no built-in IMU** (livoxtech.com/mid-70/specs has no IMU row). Every LIO package (FAST-LIO2, Point-LIO) mandates IMU input. Point-LIO furthermore has **no ROS 2 branch at all** (`hku-mars/Point-LIO` branches: `master`, `point-lio-with-grid-map`; README "tested on Ubuntu20.04 with noetic").

---

## 1. Sensor suite verdict

**Statement of record: the Livox MID-70 alone cannot drive full Nav2.** Not primarily because of the 70.4° aperture (V1), but because it emits point clouds only, its non-repetitive rosette does not flatten to a stable `sensor_msgs/LaserScan`, it has no IMU, it has no ROS 2 Jazzy driver, and a forward cone can never raytrace-clear the lateral/rear costmap cells it marks (V2). **The MID-70 is reclassified as a perception sensor.** Navigation runs on a separate sensor set.

Poses are `x y z roll pitch yaw` in metres/radians relative to `base_link`, **with `base_link` at the ground-projected chassis centre, z = 0 at the ground plane**. If your Scout Mini URDF puts `base_link` at wheel-axle height, subtract 0.0875 m from every z.

| # | Sensor (SDF `<sensor>` name / type) | Pose | Rate / config | Role | Why it exists |
|---|---|---|---|---|---|
| 1 | `nav_lidar_360` — `gpu_lidar` | `0.15 0 0.55 0 0 0` | 360 h-samples × 16 v-samples, h-FOV ±π, v-FOV ±0.26 rad (±15°), 0.15–30 m, **10 Hz** | Nav2 costmap marking+clearing; scan-matching; end-of-row detection backup | **Fills the entire Nav2 gap.** Restores lateral/rear raytrace clearing so stale marks can expire (V2). ~5,760 rays/frame — deliberately cheap. HW analogue: **Livox MID-360** (~USD 650–900), the one Livox unit `livox_ros_driver2` actually supports, and that driver **does** list Jazzy. Budget fallback: LD19 / RPLIDAR S2 2D. |
| 2 | `mid70_perception` — `gpu_lidar` | `0.10 0.24 0.70 0 -0.175 1.571` (left-facing, pitched **up** 10°) | **100 × 100** samples, h/v FOV **±0.61436 rad**, 0.05–90 m, **10 Hz** = 10,000 pts/frame | Canopy structure, trunk detection, fruit 3D localization, **camera standoff control** | Faithful 100 kpts/s budget. **Side-facing, not forward**, because (a) row centreline comes free from sensor 1, (b) the two-sided imaging requirement (§5) already dictates a canopy-facing rig, (c) it resolves the mount conflict between nav-optimal (low/level/forward) and perception-optimal (high/tilted/lateral) *before* the URDF is written. |
| 3 | `imu_link` — `imu` | `0 0 0.10 0 0 0` | 200 Hz, gyro σ 2e-4, accel σ 1.7e-2 | `robot_localization` local + global EKF | **Mandatory.** MID-70 has no IMU; both EKFs and any future LIO need one. On HW this is a separate part with its own extrinsic calibration and time-sync burden. |
| 4 | wheel odometry — `gz::sim::systems::DiffDrive` | n/a | 50 Hz, `<publish_tf>false</publish_tf>` | Twist source for local EKF; the **only** along-row (X) constraint in-row | Publish odom from **exactly one** source. Do not also run `gz_ros2_control` + `diff_drive_controller`. |
| 5 | `navsat` — `navsat` (+ `gz::sim::systems::NavSat`, world `<spherical_coordinates>`) | `-0.20 0 0.60 0 0 0` | 5 Hz | Global EKF input **in headlands only** | Gated hard in-row (§2). World must declare the NavSat system plugin. |
| 6 | `cam_canopy_left` — `camera` | `0.05 0.26 1.20 0 -0.209 1.571` (up 12°) | **1920×1080**, h-FOV 1.047 rad (60°), 10 Hz, near 0.1 / far 30 | Primary imaging: fruit counting + disease symptoms | At **1.0 m standoff → 0.60 mm/px**. Two-sided coverage is non-negotiable (single-side visibility 40.85–79.83%, Roy/Isler arXiv:1808.04336). |
| 7 | `cam_canopy_right` — `camera` | `0.05 -0.26 1.20 0 -0.209 -1.571` | identical | Mirror of #6 | With L+R cameras, **one boustrophedon pass over all M+1 alleys images both sides of all M rows**. Halves mission time vs a single-sided rig. |
| 8 | `cam_forward_rgbd` — `rgbd_camera` | `0.30 0 0.45 0 0 0` | 640×480, h-FOV 1.204 rad (69°), 10 Hz, 0.2–10 m | End-of-row detection, headland trigger, tall-grass/vegetation traversability classification | Reproduces the GNSS-free headland approach (*Machines* 11(1):84 — 100% row-end detection success, 0.54 m mean error, within 7 m). Also the first-line answer to arXiv:2407.18535 (grass marked as lethal). |
| 9–12 | `semseg` / `panoptic` (`segmentation`), `boxes_visible_2d` / `boxes_full_2d` (`boundingbox_camera`) | **Co-located with #6**, identical pose and intrinsics | 1920×1080, **1–2 Hz or triggered** | Training-label ground truth | See §6. Must share extrinsics **and** `frame_id` with #6 or masks will not align. |

**Sensors deliberately NOT included:** no second MID-70; no thermal (no use case identified); no multispectral (arXiv:2302.08818 reports operational failure from open-field exposure balancing — and Gazebo has no auto-exposure to reproduce it anyway).

---

## 2. Localization & navigation architecture

### 2.1 The core localization decision

The row is **deliberately self-similar along X**. No sensor recovers along-row position from a mid-row observation — this degrades AMCL, slam_toolbox loop closure, and FAST-LIO2 pose-graph optimization *identically*. FAST-LIO's own authors say it: "the LiDAR-based solution easily degenerates ... This problem is more obvious when the LiDAR has a small FoV" (arXiv:2010.08196 §I).

**So stop trying to observe X continuously.**

| Axis | In-row source | Headland source |
|---|---|---|
| **Y (lateral), yaw** | Continuous, from row-centreline fit to trunk points (`nav_lidar_360` + `mid70_perception`) | GNSS + EKF |
| **X (along-row)** | **Dead-reckoned** from wheel odom + IMU; **discretely reset** at row-entry and row-exit events detected by `cam_forward_rgbd` | GNSS + EKF |
| **map→odom** | `robot_localization` global EKF, GNSS covariance inflated ×100 or gated off on FLOAT/DGPS status | `robot_localization` global EKF, GNSS trusted |

X drift over a 45 m row at 1–2% wheel-odom error = 0.45–0.9 m, corrected at the far end by row-exit detection. That is acceptable because **X error does not cause collisions in a row** — Y error does, and Y is well observed.

### 2.2 Pipeline, sensors → `/cmd_vel`

```
gz-sim Harmonic (ogre2)
  └─ ros_gz_bridge (parameter_bridge, YAML config)   ros-jazzy-ros-gz 
       ├─ /clock                                      → use_sim_time:=true on EVERY node
       ├─ /scan_360 (PointCloud2)
       ├─ /mid70/points (PointCloud2)
       ├─ /imu, /odom, /navsat/fix, /cam_*/image_raw
       │
  pointcloud_to_laserscan_node   (pkg: pointcloud_to_laserscan)
       inf_is_valid: true   min_height: 0.20   max_height: 1.00
       → /scan                                        [360-only; NEVER the MID-70]
       │
  robot_localization  ekf_node "ekf_odom"   → odom→base_link   (wheel twist + IMU)
  robot_localization  navsat_transform_node → /odometry/gps
  robot_localization  ekf_node "ekf_map"    → map→odom         (+ /odometry/gps, gated)
  row_centerline_node  (custom, ~200 LOC)   → geometry_msgs/PoseWithCovarianceStamped
                                               (Y+yaw only, huge X covariance) → ekf_map
  row_event_node       (custom, ~150 LOC)   → discrete X reset at row entry/exit
       │
  nav2_route (Route Server, Jazzy)  — topological graph, typed edges
       │
  nav2_planner        ├─ "InRow"    : RowCenterlinePlanner   (custom nav2_core plugin, ~300 LOC)
                      └─ "Headland" : nav2_navfn_planner::NavfnPlanner
  nav2_controller     RotationShimController → nav2_mppi_controller::MPPIController
  nav2_costmap_2d     StaticLayer? no · STVL · InflationLayer
  nav2_bt_navigator   BT selects planner_id + goal_checker per route edge type
       │
       └─→ /cmd_vel  →  gz DiffDrive
```

**Stage-0 scaffold (do this first, it is not optional):** a ~50-line node that reads gz model pose and publishes a perfect `map→odom`. Tune MPPI, inflation, route graph and row geometry against a perfect localizer, so that when navigation misbehaves you know which layer is broken. Swap in the real stack at step 6.

### 2.3 Key config values

```yaml
# --- footprint (polygon, NOT radius) --------------------------------
footprint: "[[0.34,0.30],[0.34,-0.30],[-0.34,-0.30],[-0.34,0.30]]"
#   Scout Mini 0.627 x 0.550 -> circumscribed 0.417, inscribed 0.275
#   circular 0.417 would throw away 0.142 m/side out of only 0.725 m  [V6]

# --- local costmap --------------------------------------------------
resolution: 0.05 ; rolling_window: true ; width/height: 6.0
plugins: ["stvl_layer", "inflation_layer"]
stvl_layer:                            # spatio_temporal_voxel_layer 2.5.5 (Jazzy)
  plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
  voxel_decay: 12.0                    # SECONDS. 8-15. Purpose = EXPIRE stale
                                       # un-clearable marks, NOT retain them.  [V2]
  decay_model: 0                       # linear
  voxel_size: 0.05 ; mark_threshold: 0
  observation_sources: scan360 mid70_ground_guard
  scan360: {min_obstacle_height: 0.20, max_obstacle_height: 1.00,
            clearing: true, marking: true, obstacle_range: 10.0}
#   min 0.20 rejects mown alley grass (arXiv:2407.18535)
#   max 1.00 rejects overhanging canopy the mast can brush through
inflation_layer:
  inflation_radius: 0.40               # HARD CAP 0.55. Free alley worst case
                                       # 2.0 m -> 0.725 m/side.  [V6]
  cost_scaling_factor: 3.0

# --- global costmap -------------------------------------------------
resolution: 0.10 ; rolling_window: true ; width/height: 40.0
#   0.05 m over a multi-hectare orchard is a memory problem

# --- controller_server ----------------------------------------------
FollowPath:
  plugin: "nav2_rotation_shim_controller::RotationShimController"
  primary_controller: "nav2_mppi_controller::MPPIController"
  angular_dist_threshold: 0.785 ; forward_sampling_distance: 0.5
  rotate_to_heading_angular_vel: 0.6
  # MPPI:
  motion_model: "DiffDrive"            # correct for 4WD skid-steer
  batch_size: 2000 ; time_steps: 56 ; model_dt: 0.05
  vx_max: 0.6                          # imaging speed limit, see below
  vx_min: -0.20 ; wz_max: 1.0
  ObstaclesCritic:
    repulsion_weight: 1.0              # LOWERED from default 1.5: MPPI README
                                       # warns of "jitter in narrow corridors" [V4]
    critical_weight: 20.0
    inflation_radius: 0.40             # MUST mirror the costmap layer exactly
    cost_scaling_factor: 3.0
  PathAlignCritic: {weight: 16.0}      # raised: stay on the centreline

# --- planner_server (two plugins configured simultaneously) ----------
planner_plugins: ["InRow", "Headland"]
InRow:    {plugin: "orchard_nav::RowCenterlinePlanner"}
Headland: {plugin: "nav2_navfn_planner::NavfnPlanner", tolerance: 0.25}
#   NOT SmacPlannerHybrid: Nav2's own table lists it Ackermann/Legged only. [V3]
#   Upgrade path if wheel scrub proves to matter: SmacPlannerLattice with a
#   custom-GENERATED differential control set incl. rotate-in-place primitives.

# --- behavior_server -------------------------------------------------
#   Keep Spin (sweeps sensors, cheapest way to repopulate the costmap).
#   Keep BackUp with backup_dist <= 0.15 — the "delete BackUp" advice was
#   conditioned on zero rear coverage, which sensor #1 removes.
```

**In-row speed cap 0.6 m/s** is an *imaging* constraint, not a navigation one: at 0.6 mm/px and a 1/1000 s global shutter, 0.6 m/s gives 0.6 mm of smear = 1 px. Headland 1.0 m/s.

**Verify before designing around it:** `apt list ros-jazzy-nav2-route` on the target machine. Jazzy Nav2 is at **1.3.12**. If `nav2_route` is absent from binaries, fall back to a plain `NavigateThroughPoses` BT over waypoints — the route graph is a convenience, not a dependency.

**FAST-LIO2 / Point-LIO: research spur only, never plan of record.** Point-LIO has no ROS 2 branch. `FAST_LIO` ROS2 branch says "ROS >= Foxy (Recommend ROS-Humble)" and depends on `livox_ros_driver2`, which does not support the MID-70 at all. Both require Livox `CustomMsg` per-point timestamps, which a stock `gpu_lidar` `PointCloud2` does not provide — you would have to synthesise `CustomMsg` in sim yourself. And the MID-70 has no IMU. Four independent blockers.

---

## 3. Orchard geometry spec

Drive the generator from this table. Everything is a parameter; the "Nominal" column is the default profile (`sejang_bangchuhyeong`).

| Parameter | Nominal | Range / variant | Source |
|---|---|---|---|
| Row spacing 열간 | **3.50 m** | 2.80–3.90 (키큰방추형 3.30) | RDA 농사로 cntntsNo=30663; 충북 ARES 세장방추형 2.8–3.5 |
| In-row spacing 주간 | **1.50 m** | 1.00–1.80 (키큰방추형 1.00) | RDA: 3.5×1.5 → 190주/10a |
| Density | **1,900 trees/ha** (190주/10a) | 1,460–3,570/ha | arithmetic check: 1000/5.25 = 190 ✓ |
| Tree height 수고 | **3.20 m** | 3.0–3.5 (키큰방추형 4.0–4.5) | 세장방추형 standard |
| Canopy width | **1.20 m** | 0.90–1.50 | Robinson 0.9–1.2; 한국사과협회 1.5 |
| **Free alley width** | **2.30 m** | **2.00 m worst case** (1.5 m canopy) | **V6 — this drove `inflation_radius`** |
| Canopy bottom height | 0.70 m | 0.60–0.80 | lowest feathers ~0.30 m at nursery |
| Trunk base diameter | 0.070 m | 0.060–0.080 | Cornell min caliper 16 mm at planting |
| Trunk top diameter | 0.020 m | at 3.2 m | linear taper + axial noise σ 0.01 m |
| Feathers (lateral branches) | **28** | 20–40 | pendant, **−10° to −30°** from horizontal |
| Feather length | 0.45 m | 0.30–0.60 (키큰 0.50) | defining tall-spindle trait |
| Feather azimuth | golden angle **137.5°** stratified | | |
| Feather z-range | 0.70 → 3.00 m | | |
| Inline trellis post | **3.66 m** (12 ft), 0.70 m buried → **2.96 m** exposed, ⌀0.075 m | every **10.0 m** | Cornell cost table: 110 poles/acre ÷ 10 rows × 400 ft |
| End anchor post | 1.83 m (6 ft), angled 30° outward | 2 per row | Cornell: 20 anchor poles/acre |
| Trellis wires | **3**, at **1.00 / 2.00 / 2.90 m**, ⌀2.5 mm | **LOW CONFIDENCE — inferred, not sourced** | visual only, no collision; 2.5 mm is below sim ray density anyway |
| Rows × trees — **dev plot** | **4 × 20** | build this first, measure RTF | |
| Rows × trees — full plot | 6 × 30 | | |
| Orchard footprint (full) | **21.0 m × 55.5 m ≈ 0.117 ha** | 5 gaps ×3.5 + 2 outer alleys; 30×1.5 + 2×6.0 headland | |
| **Headland width 선회 공간** | **6.0 m** | ≥5.0 m hard minimum | *Machines* 11(1):84 — maneuvering failed below 5 m with aligned rows |
| Apples per tree (mature) | **60** | 50–100; year-2 15–20, year-3 50–60 | Cornell training table; 771 bu/ac ÷ 1320 trees ÷ 0.19 kg ≈ 60 |
| Apple diameter | **0.075 m** | 0.060–0.085 | Korean Fuji avg 75 mm |
| Cluster size distribution | 75.3% singles, rest 2–4 | | arXiv:1808.04336 Fig 21(b) |
| Apple radial placement | 0.15–0.55 m from trunk axis, z 0.80–3.00 m | **inside** the leaf volume | see §4 occlusion target |
| **Target single-side visible fraction** | **55–65%** | must land in **40.85–79.83%** | arXiv:1808.04336 — validate with `full_2d` vs `visible_2d` boxes |
| Ground: alley | mown grass, textured plane, no per-blade collision | ambientCG Grass004 (CC0) | |
| Ground: under-row strip | **1.20 m** bare soil (청경), centred on row | 0.90–1.50; 부초 mulch variant on slope | 부분초생재배: 열간 초생 / 수관하부 청경 |
| Slope variant 경사지 | 0% flat default | 0–10% option → forces 부초 mulch | many Korean apple orchards are on slopes |
| Scene budget | ~11k tris/tree → **~2.0 M tris** at 180 trees | apples at 80 tris (icosphere subdiv 1), **not** 320 | validate empirically, this is a planning estimate |

---

## 4. Tree asset pipeline

**Decision: procedural generation. There is no download that shortcuts this.** Confirmed by direct Fuel REST API query (`fuel.gazebosim.org/1.0/models`) across 13 terms: Fuel contains **zero** apple, orchard, vineyard or crop models. Only `OpenRobotics/Oak tree` (CC0), `OpenRobotics/Pine Tree` (CC0), `shrijitsingh99/Juniper Tree` (CC0), `facugu123/Tree` (CC BY 4.0) — generic forest species, baked-in foliage, no fruit. Usable only as headland windbreak filler. Every Sketchfab/BlenderKit apple tree is an ornamental round-canopy shade tree — wrong architecture, no per-fruit hierarchy.

### 4.1 The critical structural decision

Fruit must be individually addressable for panoptic instance IDs. **But do not make each apple a `<model>` or a nested `<include>`** — gz-sim issue **#1579** (open since 2022-07-07) breaks panoptic mode with nested labeled includes, returning zeros in `labels_map` and duplicate colours for distinct copies. And runtime-spawned models silently lose their labels.

**Therefore:**
- One tree = **one flat, top-level `<include>`** in the world SDF.
- Inside that model: 1 tree-body `<visual>` (trunk + feathers + leaf cards, one merged `.dae`) + **N apple `<visual>`s**, each referencing the **same** 80-tri `apple.dae` mesh (shared via `gz::common::MeshManager` → VRAM stays flat) at its own `<pose>`, each carrying its **own** `<plugin filename="gz-sim-label-system">`.
- Instance IDs are assigned per-visual automatically. 60 apples × 180 trees = **10,800 visuals**. VRAM flat, draw calls linear — this is the performance risk, see §9.
- **LOD:** only the rows you are actively imaging need per-apple visuals. Background rows get a single merged fruit submesh. Make this a generator flag.

### 4.2 Blender 4.x headless `bpy` generator

```
gen_tree.py   --height 3.2 --canopy_w 1.2 --n_feathers 28 --n_apples 60 --seed N
  1. trunk    : tapered cylinder, 10 radial segs, ⌀0.070→0.020, axial noise σ0.01
  2. feathers : 28 cylinders, 6 radial segs, len 0.30–0.60,
                pitch −10..−30°, azimuth k·137.5°, z 0.70→3.00
  3. foliage  : alpha-tested leaf-CLUSTER cards (5–20 leaves per quad in a
                2048² atlas), 150–400 cards/tree @ 2 tris.  NOT per-leaf cards
                (~4,000 tris/tree vs ~600 for equivalent visual density)
  4. apples   : icosphere subdiv 1 (80 tris), ⌀0.075, placed by rejection
                sampling INSIDE the leaf volume to hit the 55–65% single-side
                visible target — NOT conveniently on the canopy surface
  5. export   : COLLADA .dae  (gazebosim.org/api/sim/9/model_and_optimize_meshes.html
                explicitly recommends COLLADA for visual geometry; glTF/FBX
                are not mentioned)
  6. emit     : ground-truth JSON  {tree_id, apple_id, world_xyz, diameter,
                cluster_id, label_id}
gen_world.py  → FLAT world SDF, one <include> per tree, Label plugins baked in
```

**Skip Sapling Tree Gen / Modular Tree.** They produce naturalistic branching; a trained spindle is a deliberately artificial shape, easier to script directly than to coax out of Sapling's parameters.

### 4.3 Named assets and licences

| Asset | Licence | Use |
|---|---|---|
| **ambientCG** (ambientcg.com) — Bark012, Grass004, Ground037 | **CC0**, whole library, no attribution | bark, alley grass, bare strip soil |
| **Poly Haven** (polyhaven.com) | **CC0**, whole library | HDRIs, additional ground |
| `FieldRobotEvent/virtual_maize_field`, branch `ros2-gz` | **GPL-3.0** ⚠️ | **Read for architecture ONLY.** Solves the identical structural problem (procedural rows → SDF, Jazzy + gz, with CI). Copying source makes your generator GPL-3.0. |
| `tduboudi/IAMPS2019-Procedural-Fruit-Tree-Rendering-Framework` | **MIT** ✅ | Safe starting script. Caveat: scatters fruit as random particles (authors flag it as unnatural) and outputs images, not meshes. |
| OrchardBench, arXiv:2607.06337 | CC BY 4.0 (paper) | Mine the L-system + fruit-on-stem-tether methodology. Runs on Newton/MuJoCo-Warp, **not Gazebo**; asset pack release unconfirmed. |
| Fuel `OpenRobotics/Oak tree`, `Pine Tree`, `Juniper Tree` | CC0 | headland windbreak filler only |

### 4.4 Effort

| Stage | Days |
|---|---|
| v0 crude: trunk + feather cylinders + apple spheres, no leaves, flat SDF emitter, GT JSON | **1–2** |
| Leaf-cluster cards + 2048² atlas + bark/leaf/soil textures | 2–3 |
| Trellis posts, wires, anchors, two-zone ground | 1 |
| Per-apple `<visual>` emission + Label IDs + occlusion-targeted placement + visible-fraction validation loop | 2–3 |
| LOD, perf tuning, RTF measurement, parameter profiles (세장방추형 / 키큰방추형) | 2–3 |
| **Total** | **8–12 days** |

The original 1-week estimate was self-flagged low-confidence and is **optimistic**. Assume 8–12 days for a competent `bpy` user; 3× that for someone learning `bpy`. This is the single largest asset work item and everything downstream is blocked on it.

---

## 5. Disease imaging plan

### 5.1 What the verdict changed (V5)

The original justification — "rendered lesion texture lacks fine-scale appearance statistics" — is **refuted as a mechanism** and must not appear in the writeup:

- Klein et al. 2024 trained **exclusively on Blender renders** → **89.6%** on real greenhouse tomato photos. Purely-rendered training demonstrably *can* transfer. (Caveat: ~29-image real test set, post-adjustment figure; an existence proof against "will not", not proof it is easy.)
- The same authors: "photorealism (which is expensive to achieve) is **not the main quality driver**". What fixed transfer was **content** — disease-pattern diversity, cluttered backgrounds, defocused branches, lighting variation.
- Training on **real** PlantVillage photos scored "barely better than random guessing" on their real test set. Real texture statistics do not rescue transfer.
- Noyan, arXiv:2206.04374: a model trained on **8 background pixels** of PlantVillage hit 49.0% vs 2.6% chance. Classifiers largely are not using lesion texture at all.
- SDFormat 1.11 `<material><pbr><metal><albedo_map>` takes a **user-supplied image file**. Texture a leaf with a macro photo of real 갈색무늬병 and the fine-scale statistics *are* real photographic statistics. The alleged deficiency is an asset-authoring choice, not a Gazebo limit.

**The genuine Gazebo-specific gaps are different:** no subsurface scattering for leaf translucency, rasterization not path tracing, no auto-exposure, no canopy sun-fleck dappling, no rolling shutter, no motion blur from skid-steer yaw jitter.

### 5.2 What we will actually build

**Split the two analytics targets — they have opposite sim-to-real profiles.**

**(A) Fruit counting / yield — TRAIN IN SIM. This works.** Deep Count (Rahnemoonfar & Sheppard, *Sensors* 2017) trained on literally drawn coloured circles and hit **91.03%** on real tomato photos. Counting is low-spatial-frequency: blob count, colour, scale statistics. Use panoptic segmentation + 3D boxes for per-fruit instance IDs, and use sim to solve the actually-hard part — **cross-frame association / double-count elimination** — with perfect ground-truth tracks you can never obtain in the field.

**(B) Disease — sim for pipeline, viewpoint planning and coverage; real data for the deployed model.** Not because renders look fake (V5), but because:

1. **GSD physics.** 1920×1080 @ 60° h-FOV: frame width = 2·d·tan(30°).

| Standoff | 1080p | 4K (3840×2160) | Min resolvable (≈3 px) @1080p |
|---|---|---|---|
| 1.0 m | **0.60 mm/px** | 0.30 mm/px | 1.8 mm |
| 1.2 m | 0.72 mm/px | 0.36 mm/px | 2.2 mm |
| 1.5 m | 0.90 mm/px | 0.45 mm/px | 2.7 mm |

Marssonina lesions 5–10 mm = **8–17 px** ✅. Rust spots several mm ✅. Anthracnose fruit lesions 3–5 mm = 5–8 px ✅. **Acervuli (분생포자층) 0.1–0.2 mm = 0.17–0.33 px** ❌ — sub-pixel by ~5×, and 4K does not fix it. Yet Korean extension guidance names exactly this as the definitive marker: "갈색무늬병을 구분할 수 있는 가장 뚜렷한 특징은 병반의 흑색돌기이며 ... 만져보면 거칠거칠합니다". Resolving it needs ~0.05 mm/px ≈ 0.15 m standoff — an **arm / active-perception** problem, not a drive-by problem.

2. **Biological ceiling.** Korean phytopathology states apple-blotch-like symptoms cannot be reliably separated from true *Marssonina coronaria* by eye; confirmation needs stereomicroscopy, culture, cross-section, PCR. No model removes this.

3. **Domain shift binds regardless of data source.** Shibuya et al. (via arXiv:2510.12909): same-field macro F1 98.2–99.5% vs **cross-field 49.6–87.6%**, on all-real imagery, with background removal having "only a limited impact."

### 5.3 Scope tiers — say this in the proposal

| Tier | Targets | Verdict |
|---|---|---|
| **1 — deliverable from the drive-by camera** | 붉은별무늬병 (*G. yamadae*, saturated orange-red, macro-scale, best signal-to-renderer ratio); 갈색무늬병 at **mid/late** severity (5–10 mm lesions + chlorosis + 조기낙엽); 점무늬낙엽병; 응애 damage as canopy-scale 황갈색 변색/조기낙엽; 사과혹진딧물 leaf curl **modelled as geometry deformation, not texture** | Build it |
| **2 — needs stop-and-stare or a dedicated fruit camera** | 탄저병, 겹무늬썩음병 on fruit — only on unoccluded row-facing fruit | Stretch |
| **3 — OUT OF SCOPE, and say why** | 부란병 (trunk cankers: different camera geometry entirely — low, downward, near-field, behind trellis and trunk guards); any early-stage diagnosis; anything hinging on acervuli texture | Cite the GSD table as justification. This is a strength of the writeup. |

### 5.4 Rendering approach

- **(a) Material swap — base layer, whole block.** Author ~6 leaf PBR sets (healthy, marssonina_mid, marssonina_late, alternaria, rust, mite_chlorosis) + 3 fruit sets, using **real macro photographs** as albedo maps (V5: this is legitimate, not a cheat). Assign per-tree from the generator. **Cost: 2–3 days. Yields tree-level labels only** — no per-lesion masks (see §6).
- **(b) Lesion-as-geometry — 20–50 designated "inspection trees" only.** Each lesion = a small textured disc offset ~1 mm along the leaf normal, with its own Label plugin. **This is the only way to get per-lesion masks and boxes out of gz-sim.** Placement: Poisson-disc on leaf UV, size from the measured 3–10 mm distribution, vein-bounded for Marssonina. **Cost: 3–4 days.**
- **(c) Diffusion style transfer — offline, optional.** ControlNet + LoRA structural conditioning (the VitiForge / 2026 mushroom-pipeline recipe). **Never bare CycleGAN** — documented failure: "applying disease outside the leaf region into the background", which silently invalidates your pixel-perfect masks. LeafGAN exists specifically to fix this with a segmentation constraint.
- **Iterative refinement is mandatory, and refine CONTENT not photorealism** (V5). Klein et al.'s iteration 1 "classified nearly all images as healthy"; six refinement rounds were needed. Budget for that loop.

### 5.5 Real-data anchor — start the applications today

| Dataset | Content | Gate |
|---|---|---|
| **AI Hub 과수화상병 촬영 이미지** (`dataSetSn=146`) | ~145,938 apple images with bboxes, disease code, part, growth stage, **severity 0–3**, 진전도; covers 갈색무늬병 / 흑성병 / 점무늬낙엽병 / 탄저병 | **Korean nationals only, approval required** |
| **AI Hub 과수원 내 로봇 주행 데이터 (사과, 배)** (`dataSetSn=71700`) | 253,851 apple orchard frames @1920×1080 + 138,504 `.pcd`, 2D bbox + semantic seg + 3D cuboid, collected 농로/노지 | Same gate. **Exactly your robot-POV geometry — this is the correct benchmark for the whole project, not just disease.** |
| Plant Pathology 2021-FGVC8 (Kaggle) | ~18.6–23k in-field apple leaves, 6 classes | Open. US cultivars, **no 갈색무늬병 class** |
| PlantDoc (CODS-COMAD 2020) | 2,598 in-field images | Open. **OOD test set only, never training** |
| PlantVillage apple | 3,171 lab-isolated leaves | **Actively harmful.** 99% → 31% collapse cross-dataset |

### 5.6 The honest statement, verbatim for the report

> Simulation is used to develop and validate the autonomous data-collection pipeline, to optimize camera configuration, standoff and coverage, and to train and validate fruit counting and cross-frame fruit association. The disease model is trained on real field imagery (AI Hub 과수화상병 + Plant Pathology 2021 FGVC8) and evaluated on real Korean orchard robot-POV imagery (AI Hub 71700). Simulation-rendered disease imagery is used for pipeline verification, viewpoint-planning studies, and as auxiliary training data under iterative content refinement — not as the sole training source for the deployed model. The system output is framed as **symptom detection and severity mapping**, not species-level diagnosis: Korean phytopathology literature establishes that apple-blotch-like symptoms require microscopy/culture/PCR to separate from *Marssonina coronaria*, and the definitive visual marker (분생포자층, 0.1–0.2 mm) is sub-pixel at any standoff a ground robot can drive at (0.60 mm/px at 1.0 m, 1080p, 60° HFOV).

Never report a sim-test disease accuracy without the real-data number beside it.

---

## 6. Ground-truth export

**Four sensors, one link, identical pose and intrinsics, identical bridged `frame_id`.**

```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>

<link name="cam_canopy_left_link">
  <sensor name="rgb" type="camera">
    <topic>orchard/rgb</topic><update_rate>10</update_rate><always_on>1</always_on>
    <camera><horizontal_fov>1.047</horizontal_fov>
      <image><width>1920</width><height>1080</height><format>R8G8B8</format></image>
      <clip><near>0.1</near><far>30</far></clip></camera>
  </sensor>

  <sensor name="semseg" type="segmentation">
    <topic>orchard/semantic</topic><update_rate>2</update_rate><always_on>1</always_on>
    <camera><segmentation_type>semantic</segmentation_type>
      <horizontal_fov>1.047</horizontal_fov>
      <image><width>1920</width><height>1080</height></image>
      <clip><near>0.1</near><far>30</far></clip></camera>
  </sensor>

  <!-- VERDICT: "panoptic" and "instance" are ALIASES for ONE mode, not two.
       gz-sim8 examples/worlds/segmentation_camera.sdf uses <segmentation_type>instance
       for the sensor it names "panoptic". Do not budget for three modes. -->
  <sensor name="panoptic" type="segmentation">
    <topic>orchard/panoptic</topic><update_rate>2</update_rate><always_on>1</always_on>
    <camera><segmentation_type>panoptic</segmentation_type>
      <horizontal_fov>1.047</horizontal_fov>
      <image><width>1920</width><height>1080</height></image>
      <clip><near>0.1</near><far>30</far></clip></camera>
  </sensor>

  <sensor name="boxes_vis" type="boundingbox_camera">
    <topic>orchard/boxes_visible_2d</topic><update_rate>2</update_rate><always_on>1</always_on>
    <camera><box_type>visible_2d</box_type>   <!-- occlusion-aware -->
      <horizontal_fov>1.047</horizontal_fov>
      <image><width>1920</width><height>1080</height></image>
      <clip><near>0.1</near><far>30</far></clip></camera>
  </sensor>

  <sensor name="boxes_full" type="boundingbox_camera">
    <topic>orchard/boxes_full_2d</topic><update_rate>2</update_rate><always_on>1</always_on>
    <camera><box_type>full_2d</box_type>      <!-- amodal -->
      <horizontal_fov>1.047</horizontal_fov>
      <image><width>1920</width><height>1080</height></image>
      <clip><near>0.1</near><far>30</far></clip></camera>
  </sensor>
</link>
```

`visible_2d ÷ full_2d` per fruit **is** the occlusion measurement. That single ratio is how you verify your generated orchard lands in the published 40.85–79.83% band. Do not skip the second bounding-box sensor.

**Labels — opt-in, per-visual, baked into the world SDF:**

```xml
<include>
  <name>tree_r03_t12</name>
  <uri>model://apple_tallspindle_marssonina</uri>
  <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
    <label>21</label>
  </plugin>
</include>
```
…and inside the model, each apple `<visual>` and each lesion decal `<visual>` carries its own Label plugin.

**Label ID plan (hard limit 0–255 semantic; `labelColor = label/255.0f` in `Ogre2SegmentationMaterialSwitcher.cc`):**

`0` background · `10` ground · `11` weed · `12` trellis_post · `13` wire · `20` trunk · `21` branch · `30` leaf_healthy · `31` leaf_marssonina · `32` leaf_alternaria · `33` leaf_rust · `34` leaf_mite_chlorosis · `35` leaf_aphid_curl · `40` fruit_healthy · `41` fruit_anthracnose · `42` fruit_whiterot · `50–59` lesion_* decals

**Resulting gz topics:** `orchard/semantic/labels_map`, `.../colored_map`, `.../camera_info`; `orchard/panoptic/labels_map`, `.../colored_map`; `orchard/boxes_visible_2d` (`AnnotatedAxisAligned2DBox_V`), `orchard/boxes_visible_2d_image`, `.../camera_info`.

**`ros_gz_bridge` YAML** (ros_gz jazzy 1.0.23; `vision_msgs` mappings landed in 1.0.0, 2024-04-24, predating Jazzy's 2024-05-23 release):

```yaml
- {ros_topic_name: /orchard/rgb, gz_topic_name: /orchard/rgb,
   ros_type_name: sensor_msgs/msg/Image, gz_type_name: gz.msgs.Image, direction: GZ_TO_ROS}
- {ros_topic_name: /orchard/semantic/labels_map, gz_topic_name: /orchard/semantic/labels_map,
   ros_type_name: sensor_msgs/msg/Image, gz_type_name: gz.msgs.Image, direction: GZ_TO_ROS}
- {ros_topic_name: /orchard/panoptic/labels_map, gz_topic_name: /orchard/panoptic/labels_map,
   ros_type_name: sensor_msgs/msg/Image, gz_type_name: gz.msgs.Image, direction: GZ_TO_ROS}
- {ros_topic_name: /orchard/boxes_visible_2d, gz_topic_name: /orchard/boxes_visible_2d,
   ros_type_name: vision_msgs/msg/Detection2DArray,
   gz_type_name: gz.msgs.AnnotatedAxisAligned2DBox_V, direction: GZ_TO_ROS}
- {ros_topic_name: /orchard/boxes_full_2d, gz_topic_name: /orchard/boxes_full_2d,
   ros_type_name: vision_msgs/msg/Detection2DArray,
   gz_type_name: gz.msgs.AnnotatedAxisAligned2DBox_V, direction: GZ_TO_ROS}
```
Use `override_frame_id` so all five share the RGB optical frame.

**Six non-negotiable rules, all verdict-sourced:**

1. **Texture-painted lesions produce ZERO masks and ZERO boxes.** Segmentation flat-fills an entire *visual* with one label colour; there is no UV/texture-space label channel anywhere in the pipeline. **Prototype lesion-as-separate-visual on ONE tree in week 1**, before building the orchard.
2. **Emit a FLAT world.** gz-sim issue **#1579** (open, filed 2022-07-07): nested labeled `<include>`s make panoptic `labels_map` return zeros and give identical colours to distinct copies. Never wrap rows in parent models containing labeled includes.
3. **Never spawn labeled models at runtime** — they silently lose their labels. Bake all variation into pre-generated world files.
4. **`labels_map` is `RGB_INT8`.** Any `image_transport` compressed republish, lossy rosbag2 compression, or JPEG write corrupts integer labels into plausible-but-wrong classes. Raw PNG only.
5. **Instance IDs are 16-bit (65,535) per label.** Fine for per-fruit within a block; will overflow if you try per-leaf across the orchard. Restrict per-instance labeling to fruit and to inspection-tree lesions.
6. **Timestamp alignment.** RGB at 10 Hz and GT at 2 Hz will not share timestamps. Prefer `<triggered>`/`<trigger_topic>` so all five capture the same instant — **verify per-sensor-type trigger support in gz-sim 8 first**; if unsupported for `segmentation`/`boundingbox_camera`, match on exact sim time and **assert timestamp equality in the dataset writer**. Silently mismatched RGB/mask pairs are the classic way a synthetic dataset becomes quietly worthless.

Docs note: `https://gazebosim.org/api/sensors/8/boundingbox_camera.html` **returns HTTP 404**. The sensor is present in Harmonic (`BoundingBoxCameraSensor` in gz-sensors 8.2.2; `gz-sim8/examples/worlds/boundingbox_camera.sdf`), but you must read the **v7 or v9** tutorial for the v8 API.

---

## 7. What this adds to build effort vs teleop-only

**Teleop-only baseline (drive with a joystick, record images):**

| Item | Days |
|---|---|
| Secure Boot / MOK / DKMS fix + GL vendor verification | 0.5–1 |
| Scout Mini URDF + gz Harmonic bringup + `/clock` + TF | 2–3 |
| Crude orchard world (v0 generator, no leaves, no labels) | 1–2 |
| Teleop + naive image capture | 1 |
| **Baseline total** | **≈ 5–7 days** |

**Incremental cost of the full autonomous + labeled-analytics plan:**

| Work item | Days |
|---|---|
| 360° nav LiDAR SDF + `pointcloud_to_laserscan` + STVL + costmap tuning | 2 |
| `robot_localization` dual-EKF + `navsat_transform` + **custom canopy-GNSS degradation node** (fix→float transitions, correlated multipath — without this you validate nothing) | 3 |
| Ground-truth `map→odom` scaffold node | 0.5 |
| Nav2 bringup, BT wiring, MPPI + RotationShim tuning in a 2.0 m corridor | 4 |
| `nav2_route` graph, typed row/headland edges, per-edge planner+controller selection | 3 |
| `RowCenterlinePlanner` `nav2_core::GlobalPlanner` plugin (~300 LOC C++) | 4 |
| Row-centreline Y/yaw estimator + end-of-row detector + discrete X reset | 3 |
| **Full procedural orchard generator** (leaves, trellis, per-apple visuals, Label IDs, occlusion-targeted placement, LOD) | **8–12** |
| Disease PBR material sets (6 leaf + 3 fruit) + lesion-decal geometry on inspection trees | 5 |
| GT sensor rig + bridge + dataset writer + timestamp assertions + label-integrity tests | 3 |
| Occlusion budget + camera-placement sweep experiments | 3 |
| Performance/LOD/RTF work | 3 |
| AI Hub application, download, ingestion, real-data training/eval harness | 4 |
| **Incremental total** | **≈ 45–49 days** |

**Grand total ≈ 50–56 person-days (10–11 weeks solo) vs 5–7 days teleop — roughly 8×.**

Composition of the delta: **~30% orchard asset + labeling pipeline**, ~30% Nav2 + custom plugins, ~15% localization, ~15% ground-truth/dataset infrastructure, ~10% performance. **The single largest line item is the asset pipeline, not Nav2.** If schedule pressure hits, the correct cut is *not* Nav2 — it is scene scale (4×20 instead of 6×30) and lesion-geometry coverage (10 inspection trees instead of 50).

---

## 8. Open questions for the user (ranked)

**Q1 — Is there a real-hardware deliverable, or is this sim-only? (blocks the BOM and ~5 days)**
The MID-70 has **no ROS 2 Jazzy driver** (`livox_ros_driver2` supports HAP + Mid360 only; `livox_ros2_driver` is v0.0.1-beta on Dashing/Foxy/Humble) **and no built-in IMU**. Options: **(a) sim-only** — the gap is irrelevant, `gpu_lidar` publishes `PointCloud2` natively; **(b)** swap to a MID-360 on hardware, keeping MID-70 in sim only; **(c)** port `livox_ros2_driver` to Jazzy (~3–5 days, then you own it forever).
**→ Default: (a). Declare sim-only now; revisit at hardware procurement.**

**Q2 — Is a second, 360° LiDAR acceptable in the sensor set?**
This is the load-bearing question of §1. Options: **(a)** virtual 360° `gpu_lidar` in sim + **Livox MID-360** on hardware (~USD 650–900; same vendor, one driver, Jazzy-supported); **(b)** virtual 360° in sim + a cheap 2D scanner (LD19 ~USD 100 / RPLIDAR S2) on hardware; **(c)** MID-70 only — accept no lateral/rear costmap coverage, no raytrace clearing, and phantom-obstacle traps during in-place rotation.
**→ Default: (a).** (c) is a research question, not a plan; if the user wants it, build (a) first as the regression baseline and *then* measure what (c) loses.

**Q3 — Will you apply for AI Hub datasets 146 and 71700 now?**
Both are **Korean-nationals-only with an approval step**, and both are on the critical path for the disease target with no substitute. Options: **(a)** apply this week for both; **(b)** apply for 146 only; **(c)** fall back to Plant Pathology 2021-FGVC8 + PlantDoc (US cultivars, **no 갈색무늬병 class**, 2,598 OOD images).
**→ Default: (a), today.** Approval latency is schedule risk that costs nothing to start eliminating. 71700 in particular is the correct benchmark for the *entire* project, not just disease.

**Q4 — Per-apple instance labeling: full block or targeted?**
10,800 apple `<visual>`s is the main draw-call risk on the iGPU. Options: **(a)** per-apple visuals on all 180 trees; **(b)** per-apple visuals on the 2 central rows + 20–50 inspection trees, background rows get one merged fruit submesh; **(c)** per-apple only on a single 20-tree row.
**→ Default: (b).** Measure RTF on one row before committing either way — and treat the measurement as a gate, not a formality.

---

## 9. Risks, ranked by what bites first

1. **Broken NVIDIA driver — correctness, not just performance.** Gazebo's own troubleshooting page states that defaulting to Intel graphics on a hybrid machine produces **"incorrect shadows or laser scans."** Both your LiDARs are `gpu_lidar` rendering sensors. You could burn weeks debugging a Nav2 failure that is a rendering-backend artifact. Fix via `mokutil --import` (enroll the DKMS key; do not disable Secure Boot), verify with `glxinfo | grep vendor`, and set `QT_QPA_PLATFORM=xcb` for the Wayland session (Ogre+Qt have no native Wayland support). Also watch for `llvmpipe` fallback → black 3D areas (gz-sim #1116). **Do this before anything else.**
2. **Segmentation flat-fills visuals.** Texture-painted lesions yield zero masks and zero boxes. If the whole disease pipeline is designed around material swapping and this is discovered after the orchard is built, the entire ground-truth story is lost. **Prototype lesion-as-visual on one tree in week 1.**
3. **Draw-call explosion.** 10,800 apple visuals + alpha-tested leaf cards (which defeat early-Z) + five simultaneous rendering sensors, on an iGPU. Sub-1.0 RTF silently decouples Nav2's control loop from simulated sensor rates, producing controller tuning that is invalid on hardware. Mitigations: `<shadows>0</shadows>`, one directional light, primitive collision shapes only (skip collision on apples and leaves entirely), 80-tri apples not 320, `Ogre2Visual` static flag, GT cameras at 1–2 Hz or triggered. **Measure one row before committing.**
4. **Asset pipeline on the critical path.** 8–12 days, everything downstream blocked, no download shortcuts it, and the estimate has low confidence.
5. **Along-row (X) unobservability.** Affects AMCL, slam_toolbox loop closure and FAST-LIO2 PGO **identically** — swapping localizers does not make it go away. Mitigated by design (dead-reckon X, reset at row-end events), but if end-of-row detection is unreliable the mitigation collapses.
6. **gz-sim #1579 + runtime-spawn label loss.** An orchard naturally wants row→tree→fruit nesting; that is precisely the pattern that silently produces garbage instance IDs. Flat world SDF, no runtime spawning.
7. **Tighter corridor than originally planned (V6).** Free alley 2.0–2.3 m, not 2.95 m. `inflation_radius` above ~0.55 m makes the two canopy walls' inflation fields meet and renders the whole row high-cost. Symptom presents as "the planner refuses to enter rows" or "the controller oscillates" — reads as a planner bug, is a costmap parameter. **Visually inspect the costmap in RViz before debugging anything upstream.** Compounded by MPPI's documented narrow-corridor jitter if `repulsion_weight` is high relative to `inflation_radius`.
8. **Simulated GNSS lies.** gz NavSat models position + optional Gaussian noise. No multipath, no canopy attenuation, no fix→float state machine, no correlated bias. Real leaf-on dual-band RTK degrades to 0.17–0.18 m (vs 0.07 leaf-off); single-band to 1.5–3.0 m with >5 m maxima. Your analytics window is **leaf-on** — the worst case. A GNSS-dependent design validated in Gazebo is not validated.
9. **Simulated MID-70 lies in the opposite direction.** `livox_laser_simulation` is Gazebo Classic only; Harmonic forces a uniform ray grid, removing exactly the angular sparsity and temporal irregularity that make a rosette hard to consume. Match the 10,000 pts/frame budget and consider decimating/time-jittering rays.
10. **Partial `use_sim_time`.** The single most common Nav2-in-sim failure, and it presents as erratic controller behaviour, not as a clock error. Every Nav2 lifecycle node, `robot_state_publisher`, both EKFs and every custom node.
11. **AI Hub approval latency or denial.** No substitute exists for Korean apple disease + robot-POV data. Fallback (FGVC8 + PlantDoc) has no 갈색무늬병 class.
12. **Over-claiming.** It is trivially easy to report 95%+ disease accuracy on a held-out *simulated* test set. That number measures how well the model memorised your own texture set. Never report it without the real-data number beside it.
13. **Non-rigid foliage.** Wind-moved leaves are pervasive dynamic outliers to both scan matching and the costmap. STVL `voxel_decay` becomes a trade-off between forgetting real trunks and accumulating leaf noise — and Gazebo trees do not move in wind unless you make them, so sim will not predict the tuning.
14. **`labels_map` corruption via any lossy path**, and RGB/mask timestamp mismatch. Both fail silently and both poison the dataset rather than crashing.

---

### Key references
Livox MID-70 spec: livoxtech.com/mid-70/specs · `Livox Mid-70 User Manual EN v1.2` · Nav2: index.ros.org/p/nav2_mppi_controller (Jazzy 1.3.12), `navigation2/jazzy/nav2_smac_planner/README.md`, `nav2_mppi_controller/README.md`, docs.nav2.org/tuning · STVL: index.ros.org/p/spatio_temporal_voxel_layer (Jazzy 2.5.5) · gz-sim8 `examples/worlds/segmentation_camera.sdf`, `boundingbox_camera.sdf`; `Ogre2SegmentationMaterialSwitcher.cc`; gz-sim issue #1579, #1116 · gazebosim.org/docs/latest/troubleshooting · gazebosim.org/api/sim/9/model_and_optimize_meshes.html · ros_gz jazzy `ros_gz_bridge/README.md` · RDA 농사로 cntntsNo=30663 · Robinson et al., *NY Fruit Quarterly* 14(2), 2006 · Roy/Isler arXiv:1808.04336 · *Machines* 11(1):84 · Klein et al. doi:10.3389/fpls.2024.1360113 · Noyan arXiv:2206.04374 · Fei & Vougioukas arXiv:2107.01321 · arXiv:2407.18535 · arXiv:2603.23112 · github.com/FieldRobotEvent/virtual_maize_field (GPL-3.0) · github.com/tduboudi/IAMPS2019-Procedural-Fruit-Tree-Rendering-Framework (MIT) · ambientcg.com, polyhaven.com (CC0) · aihub.or.kr `dataSetSn=146`, `dataSetSn=71700`