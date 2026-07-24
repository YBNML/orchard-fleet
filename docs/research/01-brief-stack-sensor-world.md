# ORCHARD SIMULATION — DECISION BRIEF
**Target:** AgileX SCOUT MINI + Livox MID-70, orchard (노지) data collection UGV, image-based ag analytics
**Host:** Ubuntu 24.04.4 noble, i5-13500, 31 GiB, GTX 1660 SUPER + Intel AlderLake-S iGPU, Secure Boot on, Wayland
**Date:** 2026-07-24

---

## 0. WHERE VERIFICATION OVERRODE RESEARCH (read first)

Four corrections. Verdicts win; the research findings below are amended accordingly.

| # | Research said | Verdict says (WINS) | Consequence |
|---|---|---|---|
| 1 | **MOK enrollment at the blue MokManager screen is mandatory and cannot be automated — needs a physical keyboard.** | **FALSE as a blocker.** Canonical ships pre-built, *Canonical-signed* NVIDIA modules that chain to the **Canonical Master CA already enrolled on this box**. Verified available for the exact running kernel: `linux-modules-nvidia-580-generic`, `linux-modules-nvidia-580-6.8.0-136-generic`, `linux-objects-nvidia-580-6.8.0-136-generic`, `linux-signatures-nvidia-6.8.0-136-generic`, all `6.8.0-136.136+1` from noble-updates/restricted. `apt-get -s install` resolves clean (4 new, 0 removed). **No MOK, no console, no BIOS.** | Section 8 shrinks dramatically. The GPU fix is an SSH job. This is the single most schedule-relevant correction in the whole brief. |
| 2 | Lyrical is non-LTS and only exists for Ubuntu 26.04. | Lyrical Luth (May 2026) **is the current LTS** (5-yr support), Tier 1 on Ubuntu 26.04 Resolute, **Tier 3 (source-only) on Noble**. Kilted is *also* Tier 1 on Noble. Jazzy is *"the only LTS with Noble at Tier 1"*, not *"the tier-1 distro"*. | Decision unchanged (Jazzy), reasoning tightened. It also means a future OS jump to 26.04 + Lyrical is the natural re-platform, not Kilted. |
| 3 | Isaac Sim rejected because TU116 is an unsupported architecture. | **Turing IS listed as supported** in the Omniverse RTX Renderer architecture table. The correct grounds are: NVIDIA's verbatim *"GPUs without RT Cores … are not supported"*, "no support guarantees" for non-RTX GPUs, and **6 GB VRAM vs a 16 GB minimum**. | Same answer (Isaac Sim is out), but do not repeat the architecture argument — it is refutable and will cost credibility. |
| 4 | `box_type=3d` bounding boxes are usable per-fruit 6-DoF ground truth. | **gz-sensors issue #428 is OPEN** (filed 2024-05-06 against a Harmonic binary): 3D detections come out rotated ~180° with negative z. | Do **not** build the LiDAR-camera fusion evaluator on `box_type=3d` until validated. Derive per-fruit 3D truth from `pose_publisher` / `/world/<name>/pose/info` instead. Also confirmed: these sensors are **ogre2-only**, and **no ogre1 fallback exists**. |

Also confirmed unchanged: Gazebo Classic 11 is EOL **2025-01-29** with zero noble binaries anywhere (OSRF repo → focal only; Ubuntu archive → jammy universe only). No Livox non-repetitive plugin exists for gz-sim 8, and SDF 1.11 `lidar.sdf` structurally cannot express one (`<scan>` grammar is uniform-grid only: samples/resolution/min_angle/max_angle).

---

## 1. STACK DECISION

**ROS 2 Jazzy Jalisco + Gazebo Harmonic (gz-sim 8.11.0), native on the host. No Docker, no Gazebo Classic.**

- Jazzy: May 2024 → **May 2029**, Tier 1 [d][a][s] on Noble amd64/arm64 (REP-2000 line 1088).
- Harmonic: LTS, Sep 2023 → **May 2029**. REP-2000 Jazzy dependency table line 1150 pins Gazebo = Harmonic. `ros_gz` ships Jazzy+Harmonic on the `jazzy` branch with binaries from packages.ros.org.
- Verified from package metadata, not docs: `ros-jazzy-gz-sim-vendor 0.0.10` → *"Vendor package for: gz-sim8 8.11.0"*. `ros_gz` for Jazzy/noble is **1.0.22**.
- Six-year runway with a single EOL date, zero re-platforming.

**Runner-up: ROS 2 Kilted Kaiju + Gazebo Ionic (gz-sim9 9.5.0).** Genuinely Tier 1 on Noble, genuinely newer. **Lost because it EOLs November 2026 — ~4 months from today**, and Ionic EOLs December 2026. Anyone reading only the REP-2000 platform table will pick it. Put "we are on Jazzy, not Kilted" in the repo README.

**Rejected outright:** Rolling on noble (stalest build stamp 20260428 vs Jazzy 20260618 — Rolling has moved off noble); Lyrical (Tier 3 source-only on noble); Humble + Gazebo Classic 11 in Docker (targets an EOL simulator, pins you to a distro that dies May 2027, and exists only to run Livox Classic plugins); Isaac Sim (see §0.3).

### The HTTPS cert problem is a non-problem

`packages.ros.org` presents a `*.osuosl.org` cert (it is an OSU Open Source Lab mirror); HTTPS fails, HTTP returns 200. **The official `ros2-apt-source_1.2.0.noble_all.deb` configures `URIs: http://packages.ros.org/ros2/ubuntu` itself** — plain HTTP is the shipped configuration. Integrity comes from the GPG-signed InRelease plus an inline `Signed-By:` key (`C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`, rsa4096, expires 2030-06-01). No workaround needed.

**Standing rule for the team:** never hand-write a `https://packages.ros.org` sources line, and audit any Dockerfile / CI script / blog-post snippet for one — it will fail on this machine.

```bash
# 1. locale + universe
sudo apt update && sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository universe

# 2. documented noble gotcha — must read "Suites: noble noble-updates noble-backports"
grep Suites /etc/apt/sources.list.d/ubuntu.sources

# 3. official apt source package (writes http:// + inline GPG key; no postinst)
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# 4. stack — Gazebo arrives via vendor packages
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
                    ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
                    ros-jazzy-ros2-socketcan ros-jazzy-pointcloud-to-laserscan
sudo apt install -y mesa-utils   # not currently installed; needed to verify GL

# 5. environment — Wayland workaround is MANDATORY for the gz GUI
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'export QT_QPA_PLATFORM=xcb'       >> ~/.bashrc
```

**Do NOT add `packages.osrfoundation.org`.** Since Jazzy, Gazebo ships as ROS vendor packages (`gz-sim-vendor`, `gz-common-vendor`, `gz-msgs-vendor`, `gz-transport-vendor`, `sdformat-vendor`). Adding the OSRF repo and installing `gz-harmonic` gives two parallel Gazebo installs and produces plugin-path and version-match failures that are miserable to diagnose.

Wayland is confirmed active (`XDG_SESSION_TYPE=wayland`). Gazebo's own troubleshooting page: *"There's an issue with the interaction of Ogre and Qt in Gazebo that prevents wayland from working properly."* `QT_QPA_PLATFORM=xcb` is required today, not hypothetically. **Strong suggestion:** log out and select "Ubuntu on Xorg" at GDM for the sim workstation — it removes the XWayland frame copy (10–30% of GUI fps under PRIME) and is the lowest-risk config for gz + rviz2 + NVIDIA.

---

## 2. LIVOX MID-70 STRATEGY

### Real sensor spec (Mid-70 User Manual v1.2, 2021.02)

| Parameter | Value |
|---|---|
| FOV | **70.4° CIRCULAR** (not rectangular) |
| Point rate | 100,000 pts/s (200,000 dual-return) |
| Range @100 klx | 90 m @10% reflectivity, 130 m @20%, 260 m @80% |
| Close-proximity blind zone | 0.05 m (0.05–0.2 m detectable, precision not guaranteed) |
| Range precision | 1σ ≤ 2 cm @20 m; 1σ ≤ 3 cm @0.2–1 m |
| Angular precision | 1σ < 0.1° |
| Beam divergence | 0.28° (V) × 0.03° (H) |
| Laser | 905 nm, Class 1 |
| Lines | **1 (single laser)** → `line` field always 0 |
| Frame rate | **None in spec** — driver choice (5/10/20/50 Hz) |
| IMU | **NONE** |
| Physical | 97 × 62.7 × 64 mm, 580 g, IP67, 8 W avg / 35 W peak |

Two properties a uniform grid cannot reproduce: **non-repetitive accumulation** (≈32-line-equivalent coverage at 0.2 s integration, approaching 86% at 1.5 s) and **range that shortens toward the FOV edge** (the 90 m figure is a centre-of-FOV number).

### RECOMMENDED: three-layer plan, uniform `gpu_lidar` + a bridge node that owns the contract

**Layer 1 — freeze the interface contract before any simulator work (~1 day).** This is the highest-leverage decision in the whole project because it decouples every downstream consumer from the simulator choice *and* from the real-hardware driver gap.

```
Topic:     /livox/lidar
frame_id:  livox_frame
Type:      sensor_msgs/msg/PointCloud2, Livox PointXYZRTLT field layout:
             x, y, z          float32   (m)
             intensity        float32   (0.0–255.0)
             tag              uint8
             line             uint8      (always 0 for MID-70)
             timestamp        float64
Rate:      10 Hz (publish_freq analogue)
Optional:  /livox/lidar as livox_ros_driver2/msg/CustomMsg — ONLY if you commit to FAST-LIO2.
             CustomMsg:   header, uint64 timebase, uint32 point_num, uint8 lidar_id,
                          uint8[3] rsvd, CustomPoint[] points
             CustomPoint: uint32 offset_time (ns rel. timebase), float32 x,y,z,
                          uint8 reflectivity, uint8 tag, uint8 line
```
**Field-mismatch trap:** CustomMsg uses `uint8 reflectivity`; PointCloud2 PointXYZRTLT uses `float32 intensity` 0–255. Code written against one and ported to the other silently truncates. Pin both in the bridge node and unit-test both paths.

Implement `livox_sim_bridge` (a ROS 2 node), which: subscribes the `ros_gz_bridge` PointCloud2, applies the **circular FOV mask** (drop points where `sqrt(az² + el²) > 35.2°`, removing ~21% of a square grid, concentrated at the corners where an orchard robot looks for trunks and ground), rewrites fields into PointXYZRTLT, and republishes. Downstream perception/SLAM never learns which backend produced the cloud.

**Layer 2 — Phase 1 sim: Gazebo Harmonic built-in `gpu_lidar`.** SDF:

```xml
<sensor name="livox_mid70" type="gpu_lidar">
  <update_rate>10</update_rate>
  <topic>livox/points_raw</topic>
  <lidar>
    <scan>
      <horizontal><samples>113</samples><min_angle>-0.6143</min_angle><max_angle>0.6143</max_angle></horizontal>
      <vertical>  <samples>113</samples><min_angle>-0.6143</min_angle><max_angle>0.6143</max_angle></vertical>
    </scan>
    <range><min>0.05</min><max>90.0</max><resolution>0.01</resolution></range>
    <noise><type>gaussian</type><mean>0.0</mean><stddev>0.02</stddev></noise>
  </lidar>
</sensor>
```
113 × 113 × 10 Hz = 127.7 k rays/s; × 0.785 (circular mask) ≈ **100.3 k pts/s delivered**, matching the real point rate after masking. ±0.6143 rad = ±35.2°. `max` = 90 m (the 10%-reflectivity figure) — orchard bark and foliage sit at ~15–30% reflectivity; 260 m would be fantasy. stddev 0.02 matches the ≤2 cm 1σ spec.

Cost is cheap: gz's GpuLidar is multi-pass cubemap rendering, so cost ≈ (cubemap faces covered) × (scene draw cost) × update_rate — **70.4° fits inside a single 90° face**, one extra scene render per tick. Ray count is nearly free. Budget: 1–2 days including URDF.

**Known deviations to document in the repo, not paper over:** (a) no non-repetitive accumulation — every frame samples identical directions, capping any coverage-vs-dwell-time experiment; (b) constant `max` overestimates edge-of-FOV performance; (c) gz-sim issue #2743 — ogre2 GpuLidar has a documented V-shaped range error growing with distance and is more approximate than Classic at shallow incidence angles, which is exactly the geometry of a beam grazing leaves and bark.

**Layer 3 — FALLBACK / escalation, only on evidence.** A custom `gz::sim::System` plugin. Justified **only** if a downstream algorithm provably depends on the rosette (FAST-LIO degeneracy in repetitive tree rows; canopy gap-fraction / LAI where coverage-vs-integration-time *is* the physical quantity). For row-following, Nav2 costmaps, and negative-obstacle detection, the uniform grid is adequate.

If you do it: start from `gz-sim8/examples/plugin/custom_sensor_system`; reuse `scan_mode/mid70.csv` unchanged from `Livox-SDK/livox_laser_simulation` (13.27 MB, 400,001 rows, `Time/s,Azimuth/deg,Zenith/deg`, 1e-5 s step = exactly 100 kHz = **4.0 s before the pattern loops**); instantiate a dense uniform `gpu_lidar` (0.1° over 70.4° → 704×704 ≈ 496 k rays/frame); precompute at `Configure()` a lookup mapping each CSV row to the nearest grid cell (never linear-search per update); advance a time-indexed sliding window by sim time. Architectural reference: `DHA-Tappuri/livox_ignition` (Fortress), which states the approach explicitly. **Cost: realistically 2–3 weeks for a competent C++/Gazebo dev, not a weekend.** It is intrinsically lossy — pattern-shaped sampling of a uniform grid, because `gz::rendering::GpuRays` (gz-rendering8) exposes only `SetAngleMin/Max`, `SetRayCount`, `SetVerticalRayCount`, `SetHorizontal/VerticalResolution` — **there is no per-ray direction API**, and SDF 1.11 `lidar.sdf` has no custom-pattern element. **496 k rays/frame at 10 Hz will not run on the Intel iGPU — fix the GPU before committing.**

A second fallback exists and is worth exactly one scoped experiment (~1–2 days, not a main line): **Gazebo Classic 11 in a jammy/Humble container** with `LCAS/livox_laser_simulation_ros2` — the only way to get genuine per-ray non-repetitive scanning today. Build it from `ros:humble` + `gazebo11` from **jammy universe** (OSRF removed gazebo11 for jammy and noble; only focal survives). LCAS = Lincoln Centre for Autonomous Systems, an agri-robotics group, so it is the most domain-aligned fork. Note the dependency is outside OSRF's control and could vanish.

A third option unlocks only after the GPU fix: **RobotecAI/RGLGazeboPlugin** supports Harmonic and ships a built-in **Livox Mid70 preset**, publishing PointCloudPacked at ~4× gz `gpu_lidar` throughput — but it requires CUDA + OptiX, i.e. a working `nvidia.ko`. Re-evaluate this the day `nvidia-smi` works; it may make Layer 3 unnecessary.

### BLACKLIST

**`Jerry1962325/livox_laser_simulation_jazzy` is fabricated.** It advertises exactly what this project wants ("migrated from Gazebo Classic to Gazebo Sim for ROS 2 Jazzy", "ECS architecture", "PointCloud2 + CustomMsg") and will be the top search hit. Its `Update()` is `// Update sensor data (placeholder for now)` and `PostUpdate()` emits hardcoded fake geometry — a "square room" with `wall_distance = 15.0` and a test obstacle at (5,0,1). **It never raycasts the scene.** It produces plausible-looking clouds in RViz with zero relationship to world geometry, a failure mode that can survive weeks. Put this in the team notes.

### Real-hardware driver gap — flag NOW, not at integration

`livox_ros_driver2` (Livox-SDK2) supports **only HAP and Mid-360**. The MID-70 is an SDK1 device served by `livox_ros2_driver` — **v0.0.1-beta, dashing-era, last touched 2024-12-17, build notes for Foxy/Humble, no Jazzy support, no `ros-jazzy-livox-*` binary on packages.ros.org**. The simulation can be made to look perfect while the real sensor has no supported driver on the chosen distro. Three paths: (a) port `livox_ros2_driver` to Jazzy (rclcpp API drift, ament changes); (b) run the driver in a Humble container bridged to Jazzy; (c) reconsider the sensor. See Open Question 2.

### Mounting

`base_link → livox_mast_link → livox_base` (97 × 62.7 × 64 mm body) `→ livox_frame` (optical origin; name matches the real driver's default `frame_id`, and the sensor plugin lives here). SCOUT MINI deck is only 245 mm tall, so a **~0.55 m mast on the top slide rails** puts the sensor at **0.8 m above ground**. Put the pitch on the mast→base joint as a xacro parameter so tilt retunes without touching the sensor definition. **Default 25° down**; ship a **40° preset** — published orchard-UGV work with this exact sensor at 0.8 m / 40° cut the ground blind spot from 3 m to 0.21 m, raised ground point density by an order of magnitude, and hit 92.7% negative-obstacle (ditch/pothole) detection (Wang et al., *Sensors* 2024, 24(24):7929).

---

## 3. SCOUT MINI MODEL STRATEGY

**Author your own `scout_mini_description` package. There is nothing worth porting.** `git grep -lE 'gz-sim|ros_gz|gz_ros2_control|gz::sim'` across `agilexrobotics/ugv_gazebo_sim@humble` returns **zero files**. No official gz-sim SCOUT MINI simulation exists anywhere.

### Where the geometry comes from

**`westonrobot/scout_ros2`, `urdf/scout_mini/*.xacro` + `meshes/scout_mini/{mini_base_link.dae, mini_wheel.dae}`** (BSD, Clearpath/Weston Robot headers — attribution required; last commit 2024-04-26). It is the **only** ROS 2 repo with a genuine Scout Mini description, and its numbers match the official spec sheet exactly.

- `agilexrobotics/scout_ros2` (branches `humble`, `main`, `origin/jazzy`) ships **no Scout Mini URDF at all** — only `scout_v2.xacro`. Its value is the **driver** (`scout_base`, `scout_msgs`, `scout_mini_base.launch.py`).
- `agilexrobotics/ugv_gazebo_sim@humble` *does* contain `scout_mini.xacro`/`.urdf`/`.gazebo` + meshes, but **every AgileX-authored number is wrong**: `wheel_radius=0.16` (the 160 mm *diameter* used as radius → wheels 2× oversize), base mass **132.39 kg** (real robot ~23–26 kg), `wheelbase=0.3133`/`track=0.4564`. Its Classic `diff_drive` block uses `wheel_separation=0.451` (that is the **wheelbase**, not the 0.490 track) and `wheel_diameter=0.08` (should be 0.175). Commented-out blocks contain `wheel_separation=4`. Odometry from any of these is badly scaled. Its README also targets Ubuntu 22.04 + `ros-humble-gazebo-*` + `source /usr/share/gazebo-11/setup.bash` — **none of which is installable on noble**.

**Keep (verified against the spec sheet):** `base_x=0.595`, `base_y=0.395`, `base_z=0.130`, **wheelbase 0.452**, **track 0.490**, **wheel_radius 0.0875** (175 mm dia), **wheel_length 0.0852**. Back-computes to 121 mm ground clearance vs the official 115 mm — independent confirmation. Official spec: 612×580×245 mm, 26 kg, 10 kg payload, 2.7 m/s, 0 m turning radius, 115 mm clearance, 30° gradeability, 4 × 150 W brushless, 24 V/15 Ah, CAN.

**Vendor the fork and pin commits.** Upstream maintenance is thin on both repos, and the Mini description lives in a third-party fork, not the vendor repo.

### What must be fixed (highest-leverage change in this section)

**Inertias.** Even the good westonrobot file has Husky-inherited tensors **40–285× too large** (wheel: mass 1 kg with `ixx=izz=0.7171`, `iyy=0.1361`). Oversized wheel inertia dominates skid-steer yaw response and makes sim-tuned controller gains useless on hardware. Recompute:

- **Wheel** m = 1.5 kg, r = 0.0875, l = 0.0852 → spin axis `0.5·m·r² = 0.00574`; transverse `(1/12)·m·(3r²+l²) = 0.00378` kg·m²
- **Chassis** m = 20 kg, box 0.595 × 0.395 × 0.130 → `Ixx = 0.288`, `Iyy = 0.618`, `Izz = 0.850` kg·m²

Total lands near the real 23–26 kg with Livox + mast + compute payload.

### Drive: gz-sim `DiffDrive`, **not** `gz_ros2_control`

The real `scout_base` is a standalone node, **not** a ros2_control `SystemInterface`. `gz_ros2_control` would give you a controller stack in sim with no counterpart on hardware — pure sim/real divergence for zero reuse. `DiffDrive` reproduces the real driver's interface exactly: `geometry_msgs/Twist` in on `/cmd_vel`, `nav_msgs/Odometry` + `odom→base_link` TF out. `<left_joint>`/`<right_joint>` "can appear multiple times", making it a direct 4WD skid-steer fit.

```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
  <left_joint>front_left_wheel</left_joint>   <left_joint>rear_left_wheel</left_joint>
  <right_joint>front_right_wheel</right_joint><right_joint>rear_right_wheel</right_joint>
  <wheel_separation>0.49</wheel_separation>   <!-- TRACK, not wheelbase -->
  <wheel_radius>0.0875</wheel_radius>
  <max_linear_velocity>2.7</max_linear_velocity>
  <max_linear_acceleration>3.0</max_linear_acceleration>
  <topic>/cmd_vel</topic><odom_topic>/odom</odom_topic><tf_topic>/tf</tf_topic>
  <frame_id>odom</frame_id><child_frame_id>base_link</child_frame_id>
  <odom_publish_frequency>50</odom_publish_frequency>
</plugin>
```

`TrackedVehicle` is explicitly wrong here — it models track *links* with a `TrackController` per track, not wheel joints.

### Classic → Harmonic port checklist (for anything salvaged)

- plugin `filename` loses the `lib*.so` form: `libgazebo_ros_diff_drive.so` → `gz-sim-diff-drive-system`
- delete all `<transmission>` / `hardware_interface/VelocityJointInterface` blocks (Classic ros_control only)
- `<sensor type="ray">` + `libgazebo_ros_laser.so` → `<sensor type="gpu_lidar">` with `<topic>`
- `libgazebo_ros_openni_kinect.so` → `<sensor type="rgbd_camera">`
- camelCase params → snake_case
- world must load `Physics`, `UserCommands`, `SceneBroadcaster`, `gz-sim-sensors-system` (`<render_engine>ogre2</render_engine>`), `gz-sim-imu-system`, `gz-sim-joint-state-publisher-system`
- every ROS topic needs an explicit `ros_gz_bridge` entry **including `/clock`**
- `package://` mesh URIs require `GZ_SIM_RESOURCE_PATH`

### Friction, sensors, validation

Anisotropic wheel friction: **mu ≈ 0.9–1.1 longitudinal, mu2 ≈ 0.3–0.5 lateral** — not the AgileX uniform `mu1=mu2=0.2` hack (a deliberate trick to let skid-steer rotate at all on a flat plane; it transfers to nothing). Add `WheelSlip` per wheel with `wheel_normal_force ≈ 60 N` (≈24 kg × 9.81 / 4). **Avoid `fdir1`** — it is interpreted in world coordinates rather than link-relative and has documented cross-engine breakage (gz-physics issue #258); it can silently do nothing.

The real driver publishes **no IMU and no joint_states**, and `/odom` is **dead-reckoned from CAN-reported body velocity**, not wheel encoders. On soft orchard soil skid-steer slip corrupts yaw directly. Add a `<sensor type="imu">` on a dedicated `imu_link` in sim, fuse `/odom` + `/imu/data` with `robot_localization` `ekf_node`, and disable the driver's own TF. **This implies a hardware IMU purchase** (see Open Question 5).

Message types: stay on plain `geometry_msgs/Twist`. Jazzy Nav2 still defaults `enable_stamped_cmd_vel=false`, matching `scout_base`. This **flips to TwistStamped in Kilted**, and `diff_drive_controller` is already moving — isolate the interface behind a single remap/twist_stamper point now.

**Validation gate before trusting anything:** command 1.0 m/s straight and pure in-place rotation; confirm ground-truth pose (Odometry Publisher / `/model/.../pose`) tracks `/odom` within a few percent. Systematic curvature error ⇒ `wheel_separation` or friction is wrong.

---

## 4. ORCHARD WORLD DESIGN

**There is no orchard world to download. This is an asset-authoring project, not an asset-integration project.** Direct Fuel REST API queries for `orchard`, `vineyard`, `apple`, `vine`, `citrus` return **zero models**; `worlds?q=farm|orchard|agriculture` return **zero**. The entire Fuel catalog has **4 tree models** (Oak, Pine, Juniper, generic Tree), **none fruit-bearing**. Every "orchard Gazebo" repo found is a 0–5 star unlicensed hobby project. Do not let any plan assume "we'll grab an orchard world from Fuel."

### Build a Python + Jinja2 generator, modeled on `virtual_maize_field`

`FieldRobotEvent/virtual_maize_field` (GPL-3.0 code; models/textures separately licensed; tested on Humble/Jazzy/Rolling) is the right architectural template: `field_2d_generator.py` renders a `field.world.template` and **emits the SDF and the procedural heightmap PNG in one pass**, including a trick worth stealing — `cv2.circle` flattens a disc under each plant footprint so nothing floats or sinks. Study it for architecture; do not vendor GPL-3.0 code into your tree unless you accept the licence. `Romea/cropcraft` (**Apache-2.0**, arXiv:2511.02417, ~10% mIoU sim-to-real gap reported) is the licence-safe reference and has `headland_width`; its plant library is row-crop and vine only — **no trees**.

Skip ERB (Gazebo Classic idiom) and skip xacro for worlds (works, not the ecosystem norm).

Parameters: `row_spacing`, `tree_spacing`, `n_rows`, `trees_per_row`, `headland_width`, position/tilt jitter, missing-tree probability, `random_seed`.

### Default preset: Korean tall-spindle apple (노지 사과원)

| Parameter | Value | Source |
|---|---|---|
| Row spacing | **3.5 m** | RDA 농사로, 중간지력 후지 3.5 × 1.5 m = 190 trees/10a |
| In-row spacing | **1.5 m** | same |
| Tree height | **3.0 m** | Tall Spindle (NY Fruit Quarterly 14(2), 2006); PSU rule "height ≤ 90% of cross-row spacing" → 3.15 m ceiling at 3.5 m rows |
| Canopy diameter | **1.0 m** | 0.9–1.2 m mature tall-spindle |
| Lowest branch / free trunk | **0.8 m** | 30–35 in above soil, raised so pendant fruiting branches clear the ground |
| Trunk radius | 0.05–0.06 m | OrchardBench uses 0.035 m for a compact trained tree |
| Rows × trees | **12 × 40 = 480 trees** | ≈ 42 × 60 m canopy area |
| Headland | **6 m each end** (design for 5–8) | virtual_maize_field default 2.0 m; CropCraft 4.0 m (vineyard 8.0 m) |
| Total world | **~42 × 72 m** inside a **120 × 120 m** terrain | |
| Row orientation | **N–S** | Korean extension recommendation |

SCOUT MINI (612 × 580 mm, 0 m turning radius) clears a 3.5 m alley with ~1.5 m each side and spins in place in the headland — but size the headland at 5–8 m so the world stays valid if you later test Ackermann or non-holonomic planners.

**Second preset (do not skip — it is a genuinely different navigation problem): Korean pear Y-trellis (배 Y자 수형), 6 m rows × 3 m in-row.** Wide alleys, overhead trellis canopy, sky occluded above the robot, GNSS degraded. RDA recommends a triangular support frame for dense Y-trellis. Building for one crop and switching later means regenerating assets, not just parameters — decide before generator work starts (Open Question 1).

### Terrain: heightmap, not a plane, not a mesh

A perfect plane makes SCOUT MINI odometry/IMU behaviour and LiDAR ground segmentation trivially easy and therefore misleading.

- **513 × 513, 8-bit grayscale PNG, no alpha** (Perlin/simplex, low-pass filtered). Square + `2^n+1` + 8-bit gray are **hard requirements** — a 512×512 or RGBA export fails or misbehaves.
- `<size>120 120 1.5</size>` → 120 × 120 m with 1.5 m total relief ≈ 1–2% gentle slope.
- **Identical `<heightmap>` block in both `<collision>` and `<visual>`.**
- 3 diffuse+specular textures with `<blend><min_height>/<fade_dist>` for soil/grass/weed transitions. Reuse the texture set from Fuel model **`chapulina/Heightmap Bowl` (CC-BY 4.0** — attribution required): 1024² grass/dirt/fungus `_diffusespecular.png` + `flat_normal.png`.
- `<physics><dart><collision_detector>bullet</collision_detector></dart></physics>` — gz-sim's own `heightmap.sdf` says heightmaps behave better with it.
- Keep the GeoTIFF/DEM path in reserve (`dem_volcano.sdf` pattern) for a real Korean orchard site later.
- Caveat: gz-sim issue #2743 area — GPU lidar has missed heightmap visuals in some versions. Verify ground returns exist before trusting ground segmentation.

### Tree assets: author your own, three parts, merged per row

Build **one** apple tree in Blender (Sapling Tree Gen, free Blender extension, for the skeleton):
1. **trunk + scaffold** solid mesh, 2–4 k tris
2. **alpha-mapped leaf cards**, 3–6 k tris
3. **60–120 apple instances as a separate mesh with a separate material** (so fruit can be colour-coded for segmentation)

Export DAE/glTF. Then: `<static>true</static>`; **collision = one cylinder (r ≈ 0.06, h ≈ 2.5) for the trunk only**. **Never let leaf/canopy geometry become collision geometry** — the documented Baylands case went from ~5% RTF to ~90% RTF on an RTX 3060 purely by replacing visual-mesh collisions with primitives.

**The trick that makes this safe:** gz-sim's `gpu_lidar` is a *rendering-based* sensor — it ray-casts **visual** geometry. Leaf cards and apples appear in the simulated Livox cloud even though collision is a bare cylinder.

**Use 3–5 distinct tree meshes reused at varying scale/yaw, not 480 unique files** — identical mesh URIs let OGRE share one loaded mesh + material. 480 × 3 k tris = 1.44 M tris is fine on a 1660 SUPER. Textures at **1024², shared**; 6 GB VRAM will thrash on 2048²/4096² sets.

**Merge each row into ONE static model with one merged mesh at generation time** (the CropCraft/Blender approach): 1 entity + 1 draw call per row instead of 40. This is the single biggest performance lever, because **gz-sim exposes no instancing and no LOD** at the SDF or gz-rendering API level (a full 1,482-file listing of gz-rendering8 matched nothing for lod/instanc/impostor/batch).

**Enable Levels from day one.** Wrap each row (or each 20 × 20 m tile) in a `<level>` with a `<buffer>`, declare the SCOUT MINI as the `<performer>`, launch with `gz sim orchard.sdf --levels`. Retrofitting means regenerating the whole world; emitting level tiles from the generator costs almost nothing now.

### The fruit-instancing conflict, and how to resolve it

Merging rows into one mesh is required for performance. But **per-object ground truth requires every fruit to be its own top-level model** (see §6). These are in direct tension at 480 trees × ~80 apples = 38,400 entities.

**Resolution — a two-tier world:**
- **Background rows (10 of 12):** fully merged per-row mesh, fruit baked in as visual geometry only. Zero per-fruit entities. Used for navigation, SLAM, row-following, LiDAR mapping.
- **Instrumented block (2 rows × 10 trees = 20 trees):** trunk/canopy still merged per tree, but **each fruit is a separate top-level model** with a `Label` plugin. At ~60 apples/tree that is **~1,200 fruit models**, which is tractable. Used exclusively for vision dataset generation and counting-pipeline evaluation.

Keep the fruit→tree association in a sidecar JSON emitted by the generator.

### Lighting / weather: set expectations now

You get `<light type="directional" name="sun"><cast_shadows>` and `<scene><sky></sky></scene>` (**ogre2 only**). You do **NOT** get fog — `<scene><fog>` parses in the SDF spec but has no effect; it was never ported from Classic. You do **NOT** get foliage sway — `WindEffects` applies rigid-body forces only, there is no vegetation vertex shader. **Do not promise "fog and wind-blown leaves."** Lighting/weather robustness for the ag-vision models must come from (a) sun `<direction>`/`<diffuse>`/colour-temperature sweeps across simulated times of day, and (b) offline augmentation.

### Licensing register (record at ingest)

| Asset | Licence | Note |
|---|---|---|
| `chapulina/Heightmap Bowl` textures | **CC-BY 4.0** | attribution required |
| `OpenRobotics/Grass Plane`, `hexarotor/grasspatch` | CC-BY 4.0 | attribution required |
| `OpenRobotics/Oak tree`, `Pine Tree`, `shrijitsingh99/Juniper Tree` | CC0 | usable as filler/windbreak, not fruit trees |
| `Gambit/Orange, Peach, Plum, Strawberry` | CC-BY 4.0 | loose fruit only |
| `Romea/cropcraft` | Apache-2.0 | safe to derive from |
| `FieldRobotEvent/virtual_maize_field` | GPL-3.0 | **reference the architecture; do not vendor the code** |
| `PlantSimulationLab/Helios` | **GPLv2** | **arm's length.** Ships `AppleFruit.obj`, `AppleBark.jpg`, `AppleLeaf.png`, a Weber-Penn "Apple" preset, and `samples/weberpenntree_orchard` (num_rows=4, row_spacing=2.5, num_trees_per_row=10, tree_spacing=1.5). Use as a *visual and parametric reference only* — shipping derived meshes risks infecting the project. |
| `westonrobot/scout_ros2` URDF/meshes | BSD (Clearpath/Weston Robot) | attribution required |
| Poly Haven | CC0 | no fruit tree; `tree_small_02` is 4.65 M polys with 8192² textures — unusable without aggressive decimation |
| `kubja/gazebo-vegetation` | **unknown** | meshes sourced from free3d.com, no stated licence — **do not use** in anything published |

`OrchardBench` (arXiv:2607.06337, Apache-2.0) is Newton/MuJoCo-Warp, not Gazebo, but its geometry is a good reference: tree height 2.4 m, trunk radius 0.035 m, terrain roughness 3 cm amplitude (20 cm max), pipe-model taper `r_parent^β = Σ r_child^β` at β = 2.2–2.3, apples biased to well-lit outer spurs.

Dead leads, do not chase: **agri-gaia** is a Gaia-X data-platform project (DataSpaceConnector, SEEREP) with no Gazebo world; **FieldSAFE** is a 2-hour tractor sensor *dataset*, not a simulator; **"AgriSim"** does not exist.

---

## 5. CAMERA SUITE

Converged from the orchard-robot literature (arXiv:2209.04278, arXiv:2507.01912, arXiv:2409.19786) and tied directly to the geometry-driven analytics in §6.

| # | Sensor | Res | FOV | Rate | Mount | Purpose |
|---|---|---|---|---|---|---|
| C1 | RGB-D (`rgbd_camera`) forward | 848 × 480 | 1.20 rad H (~69°) | 15 Hz | front, **z = 0.45 m, pitch −25°** | row following, obstacle avoidance, Nav2 costmap |
| C2 | RGB **left**, canopy-facing | 1280 × 720 | **1.57 rad H (90°)** | 10 Hz | mast **z = 0.60 m**, yaw ±90°, pitch 0° | **trunk detection / Tree-SLAM**, ground-level fruit |
| C3 | RGB **left**, canopy-facing upper | 1280 × 720 | 1.57 rad H | 10 Hz | mast **z = 1.40 m**, yaw ±90°, pitch +10° | mid-canopy fruit detection, sizing, canopy density |
| C4/C5 | Mirror of C2/C3 on the **right** side | | | | | both rows in one pass |
| S1..S3 | Segmentation + bbox GT cameras, **co-located with C3** | 1280 × 720 | identical intrinsics | **2–5 Hz** | | ground truth only (see §6) |

**Rationale for the numbers.** Canopy standoff in practice is short — the OrBot orchard robot held **2.5 ft (~0.76 m)** from the canopy with the view kept parallel to the row. At a 1.0 m standoff you need **~90–100° HFOV to span a 2 m canopy**, so **1.57 rad**, not the 1.047 rad in Gazebo's shipped bounding-box example. The lower camera sits at **trunk height** because trunk-based semantic SLAM is the GNSS-independent localisation fallback under canopy — Tree-SLAM reaches **18 cm geo-localisation error (<20% of planting distance)** across apple and pear orchards in multiple seasons (arXiv:2507.12093). 1280 × 720 **global shutter** because the robot images while moving. C1 at −25° is the literature-standard row-following pose.

**Mast and CG.** SCOUT MINI deck is 245 mm with 115 mm clearance and 10 kg sustained payload. C3 at 1.40 m plus the MID-70 at 0.80 m plus compute raises the CG on a small skid-steer base on uneven ground. **Model the mast geometry and masses in the URDF** — the same extrinsics you intend to build — or every calibration and fusion result transfers meaninglessly and the simulated driving dynamics will be optimistic.

**Illumination.** Plan for controlled lighting and randomise it. OrBot reported harvest success **88% (day) → 94% (night)** under controlled 5600 K LED at 10% intensity, purely by removing illumination variance. In sim, randomise sun angle, intensity and colour temperature per episode so the detector never learns a fixed illuminant.

**Do not run all sensors at once for normal operation.** The ground-truth cameras (S1–S3) are dataset-generation-only and run at 2–5 Hz; `<save>` writes from the sensor thread and will saturate I/O at 30 Hz across multiple cameras.

---

## 6. GROUND TRUTH

**Scope discipline first.** Split the 8 candidate ag-analytics tasks by whether simulation can serve them at all:

**Developable in sim (signal = shape, occlusion, 3D layout):** fruit detection / counting / yield, fruit sizing, canopy volume + LAI, trunk detection, row following, camera-LiDAR fusion for 3D fruit position, blossom density (geometrically).

**NOT developable in sim (signal = leaf micro-texture, sub-surface scattering, non-RGB bands):** disease/pest lesion detection, water stress, nutrient deficiency, ripeness grading. Symptoms appear in the **infrared days before the visible range**; the working spectrum is UV–SWIR with cameras covering **VIS-NIR 400–1300 nm**. **Gazebo has no multispectral or hyperspectral sensor and there is no plausible path to faking a reflectance cube from an Ogre2 PBR material.** If stakeholders expect "disease detection" from this phase, reset that expectation now, or the deliverable will be judged a failure. The sim supports these tasks only at the data-plumbing level: topic shapes, timing, storage, annotation schema.

### Mechanism (verified against gz-rendering8 / gz-sensors8 source)

Gazebo Harmonic genuinely ships both sensors — inherited from Fortress, not new in Harmonic — and gz-sim8 `Sensors.cc` constructs both on `sdf::SensorType::SEGMENTATION_CAMERA` and `BOUNDINGBOX_CAMERA`. Shipped example worlds: `gz-sim8/examples/worlds/segmentation_camera.sdf` and `boundingbox_camera.sdf`.

```xml
<sensor name="seg" type="segmentation">
  <camera><segmentation_type>instance</segmentation_type></camera>   <!-- NOT semantic -->
</sensor>
<sensor name="bbox_modal" type="boundingbox_camera">
  <camera><box_type>visible_2d</box_type></camera>
</sensor>
<sensor name="bbox_amodal" type="boundingbox_camera">
  <camera><box_type>full_2d</box_type></camera>
</sensor>
```

Each object of interest carries:
```xml
<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>10</label></plugin>
```
Taxonomy: **10 = fruit, 20 = trunk, 30 = branch, 40 = leaf/canopy, 50 = ground, 60 = trellis/post.** Labels are **uint8 (0–255 classes)**; instance count is 16-bit (65,535 per class per frame). Anything unlabelled is background and invisible to both sensors.

Labels map encoding (panoptic): channel 0 = class label, channels 1–2 = instance count as 16-bit big-endian → `labels_map[y,x,1]*256 + labels_map[y,x,2]`. `<topic>/colored_map` carries a human-viewable RGB version. Offline export via `<save enabled="true"><path>…</path></save>` inside `<camera>` → `images/`, `labels_maps/`, `colored_maps/`.

`ros_gz_bridge` on the **jazzy** branch maps `gz.msgs.AnnotatedAxisAligned2DBox_V → vision_msgs/msg/Detection2DArray` and `gz::msgs::AnnotatedOriented3DBox_V → Detection3DArray`. (The **humble** branch lists only the 3D pair — another reason to be on Jazzy.)

### The three traps, all of which fail silently

**TRAP 1 — FATAL IF MISSED. Both sensors group instances by top-level model.** `Ogre2SegmentationMaterialSwitcher.cc:121` does `TopLevelModelVisual(_visual)->Name()` and only increments the instance counter when that name changes; `Ogre2BoundingBoxMaterialSwitcher.cc:143` does the same and `Ogre2BoundingBoxCamera.cc:783-816` merges all ogreIds sharing a parent name into **one** box. **200 apples authored as `<visual>`s, `<link>`s, or nested `<include>`s inside one tree model yield exactly ONE instance and ONE box covering the whole tree.** This produces plausible-looking output that is completely wrong.
→ **Every fruit must be its own top-level model.** Verify on **day one** with a two-apple test world before generating any orchard. Related: gz-sim issue **#1579** ("Unexpected Behavior for Segmentation Camera in Panoptic Mode With Nested Includes" — nested includes return 0 in the label map with identical colours) has been **open since 2022-07-07 with no PR**. Keep the model hierarchy flat.

**TRAP 2 — panoptic instance IDs are NOT temporally stable.** `this->instancesCount.clear()` runs in `cameraPostRenderScene` **every frame** (`Ogre2SegmentationMaterialSwitcher.cc:386-389`); IDs are then re-assigned by scene-graph render order. **Instance 7 in frame N is not the same fruit as instance 7 in frame N+1.** They cannot serve as tracking / re-ID / multi-view-association ground truth.
→ Take identity from **model names** via `pose_publisher` / `/world/<name>/pose/info` and the `logical_camera` (names + poses inside a frustum; ignores occlusion, so it gives "fruit present in volume", not "fruit visible").

**TRAP 3 (verdict override) — `box_type=3d` is buggy in Harmonic.** gz-sensors **issue #428 is OPEN**: 3D detections rotated ~180° with negative z, i.e. objects reported behind the camera. `visible_2d` and `full_2d` are the safe paths. 3D mode also publishes no overlay image, so debugging needs custom RViz2 markers via the `Detection3DArray` bridge.
→ **Do not build the camera-LiDAR fusion evaluator on `box_type=3d`.** Derive per-fruit 3D truth from model poses. Re-test #428 before relying on it.

### What the pipeline emits per frame

```json
{"frame_id": …, "timestamp": …, "camera_pose": …,
 "objects": [{"fruit_model_name": "apple_r03_t12_f47",
              "world_pose": …, "camera_frame_pose": …, "diameter_m": 0.0721,
              "modal_bbox": [...], "amodal_bbox": [...],
              "visible_pixel_count": 812, "occlusion_ratio": 0.41}]}
```

**The free win:** running `visible_2d` and `full_2d` co-located gives **modal + amodal boxes for free** — exactly the annotation that `AmodalAppleSize_RGB-D` (3,925 RGB-D images, 15,335 apples) had to produce by hand. `occlusion_ratio = modal_area / amodal_area` is the single most valuable label you can generate that real datasets mostly lack.

**The thing simulation uniquely enables — build this early.** In a real orchard you cannot know a tree's true fruit count without destructively picking it. In sim you know it exactly. **Build the double-counting / multi-view-association evaluator against simulated ground truth.** Bar to beat: **96.9% counting accuracy over 1,790 apples, 1.1 cm mean fruit size error** (arXiv:2409.19786). This is the part of the "agricultural solution" that real data genuinely cannot validate.

### Training data: plan mixed synthetic + real from day one

Sim-only training is a schedule trap. The reference number: a strawberry digital-twin pipeline reached **F1 = 0.80 from synthetic data alone, rising to F1 = 0.93 when synthetic and real were mixed**. Adopt an annotation schema that is a **superset of MinneApple** (1,000 images, >41,000 instances, polygon masks + patch-based counting for clustered fruit) and **WGISD** (300 images, 4,432 grape clusters, boxes + binary masks, COCO conversion available) so synthetic output concatenates with public real data without a conversion layer. **ACFR** (1,120 apple + 1,964 mango + 620 almond, collected from the UGV "Shrimp") is the closest viewpoint analogue — note it annotates apples as **circles**, a hint that a circle/ellipse parameterisation beats a box for round fruit. `Fuji-SfM` / `KFuji RGB-DS` / `PFuji-Size` / `AmodalAppleSize_RGB-D` round out the apple set. **Check each licence before mixing into a commercialised training corpus.**

Accuracy bars to calibrate the simulated noise model against: fruit sizing MAE **1.1–4.2 mm** in the field (3.7 mm via MSAC sphere fitting); LiDAR-camera fusion σ = **0.245 / 0.227 / 0.275 cm at 0.5 / 1.2 / 1.8 m** (~⅕ the error of a D455 alone); canopy volume vs leaf count **R² = 0.959**; TLS LAI **R² = 0.85 / 0.84 / 0.86** for apple / pear / vine.

Domain randomisation is mandatory: background (uncorrelated images, BlenderProc practice), sun angle and colour temperature, leaf albedo, fruit colour variance, camera exposure.

---

## 7. PERFORMANCE PLAN

**Levers, in strict priority order.** If you are below 0.5 RTF headless, items 1–2 are wrong; go back before touching anything else.

1. **Collision geometry.** Trunk = `<cylinder>` primitive. Canopy/foliage = **no `<collision>` element at all**. **Never** a `<mesh>` inside `<collision>`. (Baylands: 5% → ~90% RTF on an RTX 3060 from this change alone.)
2. **`<static>true</static>` on every tree.** Removes them from the dynamics solver entirely.
3. **Mesh/texture budget.** 2–5 k tris and 1–2 shared 1024² textures per tree; **3–5 distinct meshes reused**, not 480 unique files. Avoid alpha-tested leaf cards where possible — alpha test is expensive *and* perturbs the lidar depth pass.
4. **Shadows off.** `<scene><shadows>0</shadows><grid>false</grid><origin_visual>false</origin_visual></scene>`; if you need some shadowing, keep it on the robot only and set `<cast_shadows>false</cast_shadows>` per canopy visual.
5. **Sensor throttling.** gpu_lidar 10 Hz; C2–C5 at 10 Hz / 1280×720; GT cameras 2–5 Hz; tighten `<clip><far>`; **bridge only what you consume**.
6. **Headless server + detachable GUI.** `gz sim -v4 -s -r --headless-rendering orchard.sdf` (EGL, **ogre2-only**), attach `gz sim -g` when needed. GUI restarts never disturb the server, and GUI rendering stops competing with sensor rendering.
7. **Physics.** `<physics name="2ms" type="dart"><max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor></physics>`. 2 ms (500 Hz) is plenty for ≤2 m/s skid-steer and 2× cheaper than the 1 ms default; do not exceed ~4 ms or wheel contacts destabilise. Set `real_time_factor` to 0 for batch dataset generation.
8. **Levels** (`--levels`), per-row or per-20 m-tile, robot as `<performer>`, keeping ~60–100 m resident.
9. **Data path.** Prefer gz-native recording (`gz sim --record`, `.mcap`) over pushing everything through `ros_gz_bridge`.

**The bridge is the real sink, not the sensor.** gz-sim/ros_gz issue **#368** documents RTF ~90% → **40–60%** purely from enabling gpu_lidar bridges, with topics arriving at ~60% of the requested rate, and **31–33% RTF with uncapped sensor rates**. `PointCloudPacked → PointCloud2` is a per-point CPU repack. **Measure RTF with and without bridges** and report both.

### RTF targets — 480-tree world (Levels on, ~120 trees resident), 1 gpu_lidar @10 Hz + 2 RGB @10 Hz

| Configuration | RTF | GUI fps | Verdict |
|---|---|---|---|
| **GPU BROKEN today** — Intel UHD 730 iGPU (24 EU, ~0.7 TFLOPS, no dedicated VRAM, DDR bandwidth contended by the 14-core CPU running DART), GUI on | **0.1–0.3** | 5–15 | Debug only. Verifies SDF parses and topics wire up. Useless for perception timing or dataset throughput. **Any RTF measurement taken now is meaningless.** |
| GPU broken, headless, sensors only | 0.3–0.5 | — | Marginal |
| **GPU FIXED** — GTX 1660 SUPER (1408 cores, 6 GB GDDR6, 336 GB/s, ~5.03 TFLOPS ≈ **7× the iGPU**, and it takes the render working set **off the CPU memory bus**), GUI on | **0.8–1.0** | 30–60 | **Design target** |
| GPU fixed, headless (`-s --headless-rendering`) | **1.0–2.0+** | — | Physics-bound on the strong i5-13500. Use for dataset generation. |
| GPU fixed, 496 k-ray dense-grid custom Livox plugin @10 Hz | ~0.3–0.6 (est.) | — | Feasible only with GPU fixed; will not run at all on the iGPU |

**These are engineering extrapolations from hardware ratios and published analogues, not measurements on this machine.** Do not commit schedule to them — benchmark a representative world the day the GPU works.

**Interim cap while the GPU is broken:** flat plane + ~30 low-poly trees, no GT cameras. Do not attempt the full world.

---

## 8. PREREQUISITE WORK THE USER MUST DO — much smaller than the research claimed

### The GPU fix is NOT a physical-console job (verdict override)

Root cause, verified on the box: Ubuntu's `shim-signed` postinst generated `/var/lib/shim-signed/mok/MOK.{der,priv}` on 2025-05-30 (subject `CN = ubuntu Secure Boot Module Signature key`). DKMS built nvidia **580.173.02** and signed it with that key (`modinfo` on `/lib/modules/6.8.0-136-generic/updates/dkms/nvidia.ko.zst` → `signer: ubuntu Secure Boot Module Signature key`). **The key was never enrolled** — `mokutil --list-enrolled` shows only the Canonical Master CA, `mokutil --list-new` is empty. `/sys/kernel/security/lockdown` = `[integrity]`, so the kernel refuses the module.

**PATH A (RECOMMENDED — fully headless, SSH-only, keeps Secure Boot):** install Canonical's pre-built, **Canonical-signed** modules, which chain to the Master CA **already enrolled** on this machine. Verified available for the exact running kernel at `6.8.0-136.136+1` from noble-updates/restricted and noble-security/restricted.

```bash
# 0. protect the already-correct 580 userspace from autoremove
sudo apt-mark manual nvidia-utils-580 nvidia-kernel-common-580 \
     libnvidia-gl-580 libnvidia-compute-580 libnvidia-encode-580 \
     libnvidia-decode-580 xserver-xorg-video-nvidia-580

# 1. DRY RUN, then install the Canonical-signed modules
sudo apt-get -s install linux-modules-nvidia-580-generic     # expect 4 new, 0 removed
sudo apt-get    install linux-modules-nvidia-580-generic
#   NOTE: use -generic, NOT -generic-hwe-24.04 (that tracks the 7.0 HWE kernel, wrong here)

# 2. CRITICAL: /etc/depmod.d/ubuntu.conf sets "search updates ubuntu built-in".
#    The unloadable DKMS module in updates/ SHADOWS the good one in kernel/nvidia-580/.
#    Both DKMS packages must go. This pulls nvidia-driver-580 (Depends: nvidia-dkms-580).
sudo apt-get -s purge nvidia-dkms-580 nvidia-dkms-550 nvidia-driver-580 nvidia-driver-550
#    READ THE DRY RUN. ABORT if it lists ubuntu-desktop or any libnvidia-* / nvidia-utils-580.
sudo apt-get    purge nvidia-dkms-580 nvidia-dkms-550 nvidia-driver-580 nvidia-driver-550
sudo apt-get -s autoremove   # dry run again before committing
sudo apt-get    autoremove

# 3. rebuild module deps + initramfs
sudo dkms status                      # expect empty
ls /lib/modules/6.8.0-136-generic/updates/dkms/   # expect no nvidia*
sudo depmod -a && sudo update-initramfs -u -k all
sudo reboot

# 4. verify
nvidia-smi
ls /dev/dri                           # expect a NEW renderD129 / card2
mokutil --sb-state                    # still "SecureBoot enabled" — as intended
```

**NEVER run `sudo apt purge 'nvidia-*'`.** It takes the working 580 stack with it and can drag out `ubuntu-desktop`, leaving no GL and possibly no graphical login. Always `-s` first and read the removal list. (`nvidia-settings` sitting at 510.47.03 is normal Ubuntu packaging — ignore it.)

Note: the 550 leftovers are inert, not an active conflict — only `nvidia-driver-550` + `nvidia-dkms-550` remain, and `/usr/src` has no `nvidia-kernel-source-550`, so 550 can never build.

### The three things that DO need a human

1. **PRIME mode (requires a reboot; the choice is yours).** `prime-select query` = `on-demand`, so **even after a successful fix, gz sim will silently keep using the Intel iGPU** unless you act. This is the easiest way to "fix" the GPU, see zero improvement, and misdiagnose it. For a sim workstation: `sudo prime-select nvidia && sudo reboot`. Otherwise per-process: `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia QT_QPA_PLATFORM=xcb gz sim -v4 orchard.sdf`.
2. **Session type (requires a logout).** Log out, click the gear at the GDM login screen, select **"Ubuntu on Xorg"**. Eliminates XWayland's extra frame copy and Gazebo's officially-acknowledged Ogre/Qt Wayland incompatibility. Optional but strongly recommended.
3. **Only if Path A somehow fails** — PATH B, the MOK route, which *does* need a physical keyboard and monitor: `sudo mokutil --import /var/lib/shim-signed/mok/MOK.der` (password prompt twice; **lowercase a-z and 0-9 only, 8–16 chars** — the UEFI prompt uses a **US keymap regardless of your OS layout**, so a Korean layout causes "password mismatch", and 3 failures aborts MokManager), reboot, then at the blue "Shim UEFI key management" screen with a **~10 s countdown**: press any key → Enroll MOK → Continue → Yes → password → Reboot. It is shim/MokManager EFI code running before the OS; it cannot be driven over SSH, VNC, or by an agent, and this desktop has no BMC/IPMI. If the countdown is missed it boots normally and the request stays queued. `mokutil --disable-validation` requires the same blue screen, so it has no advantage. Disabling Secure Boot in BIOS is the other physical option.

### Hardware decisions that are purchases, not configuration

- **An IMU.** The MID-70 has none and the SCOUT MINI driver exposes none; `/odom` is dead-reckoned from CAN body velocity. A 9-DoF IMU on the compute box is effectively mandatory for `robot_localization` and for any Livox-oriented SLAM (FAST-LIO2 / LIO-SAM assume a tightly-coupled IMU that Avia/Mid-360 provide internally — every livox_laser_simulation tutorial demonstrates FAST-LIO with those sensors, and copying that workflow with a MID-70 will fail).
- **Photorealistic synthetic imagery** (Isaac Sim / Omniverse) is an **RTX 4080/5080 16 GB-class purchase**, not a config change. Flag to whoever owns the roadmap now.

---

## 9. OPEN QUESTIONS — ranked

### Q1 (highest impact). Which crop and training system is the target orchard?
Changes assets, world geometry, sensor poses, and the navigation problem. Regenerating is not a parameter tweak.
- **(a) Korean tall-spindle apple, 3.5 × 1.5 m** — narrow alleys, open sky, LiDAR sees through canopy gaps. 190 trees/10a (RDA 중간지력 후지).
- (b) Korean pear Y-trellis, 6 × 3 m — wide alleys, **overhead trellis canopy occluding sky above the robot**, GNSS degraded. A genuinely different failure-mode set.
- (c) Korean planar / multi-leader apple (평면형 / 다축형), 3.0 × 1.5 m — 1,149.9 ha in 2025 with a government target of **5,000 ha by 2030**; explicitly designed to "arrange trees two-dimensionally and minimise work movement paths" (flower thinning: 200 workers slender-spindle vs 60–80 multi-leader).
- **Recommended default: (a) as the primary preset, with (b) built as a second preset in the same generator.** (a) is the harder navigation problem and matches 노지 사과원 practice; (b) exposes the GNSS/occlusion failure modes that (a) hides. (c) is the strategically growing system and is a cheap parameter variant of (a) — worth adding third.

### Q2. What is the plan for the real MID-70 driver on ROS 2 Jazzy?
This is the highest *project* risk and it is invisible from the simulator. `livox_ros_driver2` does not support the MID-70 at all; `livox_ros2_driver` is v0.0.1-beta, dashing-era, no Jazzy support, no apt package.
- (a) Port `livox_ros2_driver` + Livox-SDK v1 to Jazzy (rclcpp API drift, ament changes).
- (b) Run the SDK1 driver in a Humble container, bridge to the Jazzy stack.
- (c) Write a thin SDK-v1 → PointCloud2 node yourself against the frozen contract in §2.
- (d) **Switch sensor to Livox Mid-360** — first-class ROS 2 Jazzy driver support, built-in IMU, actively maintained SDK2 (pushed 2026-04-14), and a working Harmonic community sim port lineage. Costs the MID-70's superior 0.05 m blind zone and longer range.
- **Recommended default: spike (c) in week 1** (it is small, and the §2 contract makes it swappable), **and if the MID-70 has not yet been purchased, seriously evaluate (d)** — for an orchard UGV doing row-following, the built-in IMU alone may justify it.

### Q3. Is true non-repetitive scan fidelity a requirement?
Decides whether 2–3 weeks of C++ plugin work enters the plan.
- (a) **No** — uniform `gpu_lidar` + circular mask. Adequate for row following, Nav2 costmaps, trunk detection, negative-obstacle detection.
- (b) Yes — custom `gz::sim::System` (Layer 3, §2), or the RGL plugin once CUDA works, or the Classic/Humble container side-experiment.
- **Recommended default: (a), with an explicit escalation trigger.** Escalate only if a named downstream algorithm provably depends on the rosette — FAST-LIO degeneracy in repetitive rows, or canopy gap-fraction / LAI where coverage-vs-integration-time *is* the physical quantity. Note the honest caveat: with a uniform grid, **canopy volume / LAI results tuned in sim will not transfer reliably**, because they depend directly on point density accumulated by the non-repetitive pattern. Also note the 4.0 s CSV loop caps long-dwell accumulation even in path (b).

### Q4. Does phase 1 include fruit-level analytics, or navigation + mapping first?
Decides whether the instrumented-block ground-truth machinery (§4, §6) is built now or later.
- (a) Navigation + mapping + data collection only; ground truth deferred.
- (b) Both, in parallel.
- (c) Fruit analytics first.
- **Recommended default: (b), but asymmetric.** Full 480-tree world for navigation/SLAM, plus a **small 20-tree instrumented block (~1,200 fruit models)** for vision GT. The instrumented block is cheap and validates the §6 traps (especially TRAP 1) on day one, when they are recoverable, rather than after the world generator is written.

### Q5. IMU procurement, and GPU remediation path — confirm both.
- **IMU:** has one been budgeted? The MID-70 has none, the SCOUT MINI exposes none, and `/odom` yaw drift on soft orchard soil is a certainty. **Recommended default: spec a 9-DoF IMU on the compute box now**, model it in the URDF as `imu_link` with a `<sensor type="imu">`, and fuse via `robot_localization` `ekf_node`.
- **GPU:** **Recommended default: Path A** (Canonical-signed modules, headless, keeps Secure Boot). Confirm nobody objects to purging `nvidia-driver-580` (the metapackage; the userspace libs survive via `apt-mark manual`).

---

## 10. HONEST RISK LIST — most likely to go wrong, in order

1. **Real-hardware MID-70 driver gap.** The simulation can be made to look perfect while the real sensor has no supported driver on Jazzy. `livox_ros_driver2` = HAP + Mid-360 only; `livox_ros2_driver` = v0.0.1-beta, frozen 2024-12-17, never tested past Foxy/Galactic. **Validate in week 1. This is the single most likely schedule-killer.**
2. **The fabricated-repo trap.** `Jerry1962325/livox_laser_simulation_jazzy` advertises exactly what this project wants and emits a hardcoded fake room. Plausible-looking RViz output with zero relationship to world geometry — a failure that can survive weeks. Blacklist explicitly.
3. **The top-level-model instancing trap (§6 TRAP 1).** Authoring fruit as visuals or nested includes silently collapses everything into one instance and one box per tree. Fails silently, and gz-sim #1579 has been open since 2022. **Two-apple test world on day one, before any generator work.**
4. **AgileX URDF numbers.** Every AgileX-authored SCOUT MINI description is dimensionally wrong (wheel radius 0.16 m = 2×, mass 132 kg, track/wheelbase swapped, wheel_diameter 0.08). Copying any of them silently produces ~2×-scaled odometry. Use westonrobot geometry and nothing else — and **even that has inertias 40–285× too large**, which will make sim-tuned controller gains useless on hardware.
5. **The "GPU is fixed but nothing got faster" misdiagnosis.** `prime-select` is `on-demand`. After a successful module fix, gz sim keeps using the Intel iGPU unless you switch PRIME or set offload env vars.
6. **`ros_gz_bridge` RTF collapse.** 90% → 40–60% just from enabling gpu_lidar bridges (issue #368). An architecture that assumes everything flows through ROS topics at high rate will not hit real-time regardless of GPU. Design around gz-native recording; measure both ways.
7. **Zero downloadable orchard content.** Fuel has 4 non-fruiting trees and no agricultural worlds. This is an asset-authoring project. Budget real Blender time or the schedule is fiction.
8. **Sim-to-real gap on appearance.** Sim-only detector = F1 0.80 vs 0.93 mixed. Deferring real orchard data collection is a trap; it must run in parallel. And **any conclusion drawn in sim about SLAM robustness in repetitive orchard rows carries an unquantified optimism bias** — plan at least one real-sensor collection early to bound it.
9. **Stakeholder expectation on disease detection.** It needs 400–2500 nm bands that no gz sensor produces. If "disease detection" is in anyone's phase-1 expectation, reset it now. Ripeness/colour grading is borderline and should also be treated as real-data-only.
10. **The Kilted EOL trap.** Kilted is genuinely Tier 1 on noble and looks more modern. It dies November 2026. Someone will `apt install ros-kilted-*` and it will silently rot in ~4 months.
11. **`box_type=3d` correctness (gz-sensors #428, open).** Anyone who builds the fusion evaluator on it gets 180°-rotated, negative-z detections.
12. **6 GB VRAM ceiling.** 480 trees with 2048²/4096² textures will thrash OGRE and collapse RTF in a way that looks like a physics problem. 1024², shared meshes, 3–5 distinct trees.
13. **Skid-steer friction does not transfer.** Tuning that makes gz-sim rotate correctly on a flat plane can be actively misleading on grass/mud/ruts/slope. Validate turning on the real robot early. `fdir1` is a specific trap — world-coordinate interpretation, documented cross-engine breakage (gz-physics #258); it can silently do nothing.
14. **Twist → TwistStamped migration boundary.** Jazzy Nav2 defaults to Twist (matching `scout_base`); Kilted flips to TwistStamped and `diff_drive_controller` is already moving. Isolate `/cmd_vel` behind one remap point now.
15. **Mixing Gazebo sources.** Adding `packages.osrfoundation.org` alongside the `ros-jazzy-*-vendor` packages yields two Gazebo installs and hard-to-diagnose plugin-path failures.
16. **Isaac Sim forecloses permanently.** No RT cores, 6 GB < 16 GB. If photoreal synthetic data with domain randomisation later becomes a requirement, that is a hardware purchase. Flag to the roadmap owner now, not at integration.
17. **Upstream maintenance is thin.** `westonrobot/scout_ros2` last commit 2024-04-26; `agilexrobotics/scout_ros2`'s Jazzy work sits on a branch oddly named `origin/jazzy` (2024-10-21) that never reached main — a maintainer slip, not a curated release. **Vendor and pin commits; do not track branch heads.**
18. **The Gazebo Classic escape hatch is narrowing.** Classic EOL'd 2025-01-29; `gazebo11` is gone from OSRF for jammy and noble. The container fallback survives only because Ubuntu jammy universe still carries `gazebo`/`libgazebo-dev` — outside OSRF's control, and it could disappear. Never build the primary pipeline on it.