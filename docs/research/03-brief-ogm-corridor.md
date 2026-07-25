# Orchard Drivable-Corridor Pipeline — Plan of Record

**Scope:** accumulated point cloud → 2.5D DEM → traversability → drivable mask → alley centrelines → `nav2_route` GeoJSON + Nav2 costmaps, on Ubuntu 24.04 / ROS 2 Jazzy / gz-sim 8.11.0, with a single forward MID-70.

**Verdicts that overrule the research findings are marked `[VERDICT OVERRIDE]` and are stated where they change the design.**

---

## 0. Three prerequisite patches (do these before any pipeline code)

These are not optional; without (a) nothing in §5 is computable, and (b)/(c) make every result in this sim untransferable.

**(a) Ground-truth dump — `scripts/gen_world.py`.** The tree placement loop at `scripts/gen_world.py:781-808` computes `jx, jy, yaw, tz` with `rng.gauss` jitter and the `missing_prob` skip, then discards them; only SDF is written. Add ~20 lines writing `sim/worlds/<world>_gt.json`:

```json
{"row_spacing":3.5, "tree_spacing":1.5, "x0":-15.75, "y0":-30.0,
 "trees":[{"row":r,"idx":t,"x":jx,"y":jy,"z":tz,"yaw":yaw,"present":true,"model":"apple_tree_s00"}],
 "row_lines":[{"row":r,"x":x0+r*3.5,"terrace_level":levels[r]}],
 "alley_centrelines":[{"alley":a,"x":x0+(a+0.5)*3.5,"z":level_a,"y_range":[-30,30]}],
 "terrace_steps":[...], "weeds":[{"x":..,"y":..,"h":..}], "fallen_fruit":[...]}
```
Also dump the weed and fallen-fruit positions from `build_row_details` (`gen_world.py:620-641`) — they are the dominant false-lethal source (§7.2).

**(b) 16-bit heightmap — `scripts/gen_heightmap.py`.** `heightmap_meta.json` gives `size_z = 4.8188` m over an 8-bit PNG = **1.89 cm/level**. The project's measured "alley flatness 0.6–1.9 cm" *is that quantisation step*, not terrain. Every roughness threshold below is currently being tuned against a PNG artifact. Emit 16-bit (0.0074 cm/level) and re-measure. Also raise `size_px` from 513 (0.2344 m/px, so the 1.2 m bank is only 5.1 source pixels and gz-sim bilinearly smooths it) to 1025 (0.117 m/px).

**(c) Headland geometry — `gen_heightmap.py` args.** Measured slope by |y| band: 28–32 m → 29.9°, 32–34 → 20.8°, ≥34.5 → 8.1°. With `headland=6.0` (block ends at |y|=36) and `ramp_len=4.5`, the legal terrace crossing exists only in **|y| ∈ [34.5, 36.0] — a 1.5 m band** for a robot with a 0.834 m circumscribed diameter. Scout Mini turns in place so this is survivable but it is the wrong thing to debug. Regenerate with `--ramp-len 2.5 --headland 8.0` → pure ramp from |y|=32.5, block edge 38.0 → **5.5 m usable crossing band**.

---

## 1. Pipeline of record

Ten stages. Stages S1–S7 run **offline, once per world**, on the accumulated map. S8–S9 run online. This split is the single most important architectural decision and it is forced by geometry, not by convenience (§6).

### S0 — Accumulate the cloud
Source: `orchard_sim/mapping_run.py` driving the robot down all 9 alleys at 0.5 m/s, `livox_sim_bridge` output transformed to `map` by the Stage-0 GT localizer (`orchard_sim/gt_localizer.py`). Accumulate into a single float32 `(N,3)` array. At 10 Hz × ~7,300 returns × ~1200 s ≈ 8.8×10⁷ points ≈ 1.1 GB — decimate on the fly with a 0.02 m voxel grid (`numpy` lexsort on quantised indices, no PCL needed) to ~10⁷ points.

**Two runs required:** one each direction per alley. The occlusion shadow (§6) falls on the uphill side and is direction-dependent; a single-direction pass leaves a permanent no-data strip.

### S1 — 2.5D DEM, 0.10 m cells, gated-neighbourhood percentile
Bin points by `(ix, iy) = floor((x,y)/0.10)`. Per cell:
- `n[c]` = point count
- `zq[c]` = **20th percentile of z** (not min — min latches onto range noise; not 25th — CMU's `quantileZ=0.25` is tuned for a 360° lidar with far denser ground coverage than a 70.4° wedge gives at grazing incidence)
- `relief[c]` = p90(z) − p10(z)

Then the **height-gated 3×3 smoothing** — this is the whole terrace adaptation and the piece CMU's `terrainAnalysis.cpp` lacks:

```python
ground[c] = median({ zq[n] for n in N8(c) ∪ {c} : |zq[n] - zq[c]| <= 0.15 })
```

The 0.15 m gate is calibrated: over a 0.3 m (3×3) window the 8.1° headland ramp changes ground by 0.3 × 0.142 = **0.043 m** (passes), while a 58% bank changes it by **0.174 m** (rejected, so bank cells keep their own true height instead of being smeared into a false ramp). Cells with `n[c] < 5` → `UNKNOWN`.

> `[VERDICT OVERRIDE]` The research claim that "a per-cell 2.5D elevation grid is **required**" is **refuted**. What is required is that the ground model be *local and piecewise*; polar piecewise-planar models (Patchwork++ R-VPF, Himmelsbach line-fit) satisfy that too, and single-plane RANSAC restricted to one alley is actually close to a *best case* here — the MID-70's ground footprint stays within one terrace out to ≈2.0 m forward, and each terrace is flat to the quantisation floor. We choose the 2.5D raster anyway, for two reasons the verdict itself endorses: (i) it is the representation with a clear edge on **negative obstacles** — a lower neighbouring terrace is a 0.26–0.50 m drop-off that every "points above z are obstacles" rule labels free — and (ii) offline the reference frame is the map, not the sensor, which deletes the only advantage the polar methods have. Do not repeat the claim that RANSAC "cannot separate ground here." It is high-variance at terrace boundaries and on the headland ramp; that is the defensible statement.

### S2 — Feature layers
All from `ground` (float32, 0.10 m), computed with `scipy.ndimage`:

| layer | computation | window |
|---|---|---|
| `step` | `maximum_filter(ground,7) - minimum_filter(ground,7)` | 7×7 = 0.7 m ≈ footprint |
| `slope` | `hypot(*np.gradient(ground, 0.10))`, then `arctan` | 3×3 central difference |
| `rough` | `ground - uniform_filter(ground,7)`, then `generic_filter(std,7)` | 7×7 |
| `drop` | `ground[c] - minimum_filter(ground,7)[c]` (signed, negative-obstacle) | 7×7 |
| `relief` | p90−p10 of raw z, from S1 | per cell |
| `unknown` | `n[c] < 5` | per cell |

Use a **window**, not cell-to-cell differences, for `step`. At 0.10 m cells a single cell on the 58% bank holds only 5.8 cm — below any usable threshold. Over the 0.7 m window the same bank yields 0.58 × 0.7 = **0.41 m**; the mildest bank (30.8% peak grade) yields **0.216 m**. Alley cells yield 0.006–0.019 m. That is the discriminant, and its margin is ~11× on one side and ~2× on the other.

### S3 — AGL obstacle layers, two bands
For every point: `agl = z - ground[cell(x,y)]`. Two separate rasters:

- **`wall`** — count of points with `agl ∈ [0.30, 1.30] m`, occupied if count ≥ 3. This is the corridor boundary layer used for centreline extraction.
- **`clutter`** — count of points with `agl ∈ [0.12, 0.30] m`, occupied if count ≥ 3. Weeds (0.10–0.26 m, 70/alley, Gaussian σ=0.75 m about the alley **centreline**, `gen_world.py:620-630`) and fallen fruit (0.075 m, 90/alley) land here. Fed to Nav2 as **soft cost**, never lethal, and never used for centreline extraction.

> `[VERDICT OVERRIDE]` The premise that canopy overhang at 0.7–3.0 m would mark the alley fully blocked, and that a robot-height band is the fix, is **refuted on both halves**. (i) The tall spindle is **conical** — measured from the seven generated GLBs, max canopy radius is **0.861 m at z = 0.50 m**, giving a narrowest free alley of 3.50 − 2(0.861) = **1.78 m**, against a 0.834 m circumscribed robot. The alley is never blocked at any height. (ii) Because the canopy is *widest inside the robot band*, a 0–1.25 m AGL band gives min free width 1.78 m — **identical** to projecting every non-ground point. The height band buys **exactly 0.00 m of alley width.** Its real jobs here are the *low* cut (separating traversable vegetation from walls) and the fact that it is computed **above local ground** rather than in the global frame. `0.7–3.0 m` is not the canopy silhouette anyway — it is `feather_z_min`/`feather_z_max` in `scripts/gen_tree.py:39-40`, the trunk band where branches attach; the feathers pitch down −30°…−10° so real canopy starts at **z = 0.34–0.45 m**.

### S4 — Drivable mask
```
drivable = (step < 0.10) & (slope < 12°) & (rough < 0.05) & (|drop| < 0.10)
         & (relief < 0.08) & ~unknown & ~wall
```
Unknown handling, explicitly: `unknown → non-drivable`. Then fill **only** unknown connected components with area < 0.25 m² that are fully enclosed by drivable (`scipy.ndimage.label` + boundary test). Do **not** run a blanket morphological close — that would silently fill the systematic uphill occlusion shadow with invented ground.

### S5 — Row heading, globally, once
Radon-style: for θ ∈ [−10°, 10°] in 0.1° steps, rotate the `wall` raster (`cv2.warpAffine`, `INTER_NEAREST`), project onto the x-axis, score by kurtosis of the 1-D histogram. Take the argmax. Everything downstream depends on this; a 2° error destroys S6.

### S6 — Anisotropic closing along the row direction
In the rotated (row-axis-aligned) frame:
```python
k = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 13))   # 1.3 m ALONG the row
occ_closed = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, k)
```
Measured effect on this exact geometry: junctions **519 → 80**, false row crossings **105 px → 0**, lateral RMSE unchanged (5.53 → 5.43 cm). It welds the 0.30 m inter-tree gaps (1.2 m canopy on 1.5 m spacing) and the 1.80 m single-missing-tree gaps, while a genuine 3-tree gap (4.8 m) correctly stays open. **Never use an isotropic kernel** — that closes the alley. **Disable for |y| > 30 m** or the headland gets sealed.

Note the division of labour: on *this* terrain the S4 traversability mask already kills the missing-tree crossings, because the 1.2 m bank sits under the row line whether a tree is present or not. The closing is for **topology** (the ~400 medial-axis notches and 207 loops). In a flat real orchard the closing is the *only* defence, which is why it stays in the pipeline.

### S7 — C-space, centreline, graph
1. `free = (drivable) & ~occ_closed`; `edt = cv2.distanceTransform(free.astype(uint8), cv2.DIST_L2, 5) * 0.10`
2. `cfree = edt > 0.50` (0.417 m circumscribed radius + 0.083 m margin). Worst case 1.78 − 2(0.50) = **0.78 m** of corridor survives.
3. **Primary centreline (in-alley): constrained row-model fit.** Histogram-project `occ_closed` perpendicular to the row heading, peak-pick, fit a 1-D lattice with the known 3.5 m pitch by least squares. Alley centreline = midpoint of consecutive row lines, sampled every 0.25 m in y, with `ground` sampled for z. Measured to hold 10/10 rows at 3%, 10% **and 20%** missing trees.
4. **Secondary (headland + self-check): skeleton.** `cv2.ximgproc.thinning(cfree.astype(uint8)*255, cv2.ximgproc.THINNING_ZHANGSUEN)`.

> `[VERDICT OVERRIDE]` Three parts of the standard skeleton recipe are **refuted**. (i) The "distance-transform ridge" option is not a centreline method at all — a naive EDT local-maxima ridge on this grid gave 3,396 px in **1,159 disconnected components**. Delete it. (ii) `skeletonize` and `medial_axis` are **not** interchangeable: on the identical grid, 13 vs 394 endpoints, 156 vs 702 junction px, mid-alley p95 lateral deviation **5.0 cm vs 40.2 cm** clean and 15.0 vs 55.0 cm with boundary noise. Against 87.5 cm of usable half-clearance, `medial_axis` burns 47–63% of the margin. Use Zhang-Suen thinning only. (iii) Skeletonising the **free-space** grid, as normally described, gives 2,589 junction px and 0 endpoints — a dense mesh. The mandatory, usually-unstated precondition is to skeletonise the **configuration-space** free set (step 2 above). Note also `python3-skimage` on noble is 0.22.0, where the signature is `skeletonize(image, *, method=None)` and `medial_axis(..., *, rng=None)` — upstream docs showing `method='lee'` and `random_state=` do not apply.
5. Prune: iterative endpoint deletion, **15 iterations = 1.5 m** (one tree spacing). Then **collapse cycles**, do not prune them — contract any cycle enclosing < 2.25 m² (= tree_spacing²). Loops have no endpoints; the measured pruning sweep drove endpoints 90 → 0 while junctions stayed at 515. This is where the standard tutorial fails silently.
6. Degree map by `cv2.filter2D` with a 3×3 ones kernel; deg 1 = endpoint, deg ≥ 3 = junction. Split at junctions. Order each polyline by projecting pixels onto the row heading and **sorting** — never rely on pixel traversal order.

### S8 — Outputs
- `orchard_dem.npz` — `ground`, `step`, `slope`, `rough`, `unknown`, origin, resolution.
- `orchard_static.pgm` + `.yaml` at **0.05 m** (nearest-neighbour upsample of the 0.10 m mask), served by `nav2_map_server` into Nav2's `static_layer` with `map_subscribe_transient_local: true`. 630×1440 = 907k cells = 0.91 MB. The terrain and trees are static and known — bake them.
- `orchard_routes.geojson` for `nav2_route`. Verified schema from `/opt/ros/jazzy/share/nav2_route/graphs/sample_graph.geojson`: nodes are `Point` features with `properties {id, frame:"map"}`; edges are `MultiLineString` features with `properties {id, startid, endid}`. Emit 18 nodes (9 alleys × 2 ends) and 25 edges (9 alley + 8+8 headland connectors). Add `metadata.speed_limit: 0.3` on headland edges. Enable `CostmapScorer` so a blocked alley re-routes without regenerating the graph.
- `alley_centrelines.csv` — (alley, y, x, z) at 0.25 m spacing, for §5.

### S9 — Online: AGL filter node + STVL
New node `orchard_sim/agl_filter_node.py`: loads `orchard_dem.npz`, subscribes `/livox/points`, transforms to `map`, computes per-point `agl = z - ground[cell]`, republishes **two** clouds:
- `/perception/walls` — `agl ∈ [0.30, 1.30]` → STVL marking source (lethal)
- `/perception/clutter` — `agl ∈ [0.12, 0.30]` → separate Nav2 `obstacle_layer` source at reduced cost

STVL config — the height gate must be **wide open**, because the filtering has already happened upstream:
```yaml
stvl_layer:
  plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
  observation_sources: walls
  walls:
    topic: /perception/walls
    min_obstacle_height: -100.0    # DISABLED ON PURPOSE
    max_obstacle_height:  100.0    # DISABLED ON PURPOSE
    obstacle_range: 3.0            # usable annulus is 0.9-3.0 m; see §6
    voxel_decay: 10.0
    filter: "passthrough"          # anything else silently ignores min/max entirely
```

---

## 2. Why the naive approaches fail here — with this orchard's numbers

**2.1 A global z-threshold is topologically impossible, not merely inaccurate.**
`heightmap_meta.json` gives nine monotonic terrace steps (`levels = np.cumsum(steps)`, `gen_heightmap.py:95`): 0.3798, 0.3316, 0.3750, 0.2734, 0.4762, 0.4974, 0.2647, 0.3396, 0.4325 → alley floors at z = 0.00, 0.38, 0.71, 1.09, 1.36, 1.84, 2.33, 2.60, 2.94, **3.37 m**. Total relief **3.370 m > 3.2 m tree height (1.05×)**. The ground on alley 9 is above the *canopy top* of alley 0. No horizontal plane separates ground from canopy across more than two adjacent alleys. Errors accumulate in one direction because the staircase is monotonic, so they never average out.

**2.2 The STVL / Nav2 height band is applied in the costmap global frame — confirmed in source, and it fails silently in two opposite directions.**
`spatio_temporal_voxel_layer.cpp:77` sets `_global_frame = layered_costmap_->getGlobalFrameID()`; `measurement_buffer.cpp:146` calls `tf2::doTransform(cloud, *cld_global, tf_stamped)` and *only then* applies `pcl::PassThrough` on `"z"` at lines 167-172. Nav2's `observation_buffer.cpp` is identical (transform line 116, filter lines 143-144). With the draft config `min_obstacle_height: 0.20, max_obstacle_height: 1.00`:
- **Alley 0** (ground 0.00–0.52): the *ground itself* is inside the band → every ground return marked LETHAL → robot entombed, "stuck for no reason".
- **Alleys 3–9** (ground 1.09–3.85): the entire 0.80 m window is *below* that alley's ground → PassThrough discards **100% of points** → trees invisible, clean-looking empty costmap, robot drives into trunks.

These two opposite symptoms share one root cause. Debugging them separately will mislead. One step alone eats 33–62% of an 0.80 m window; two steps exceed it outright. There is **no** (min,max) pair that works. The upstream fix (STVL PR #304: `passthrough_relative`, `voxel_relative`, `z_reference_frame`) is still **open and unmerged**; `strings /opt/ros/jazzy/lib/libspatio_temporal_voxel_layer_core.so` contains none of those symbols.

**A trap you must not fall into:** in the current sim rig the *local* costmap (`global_frame: odom`) will appear to work anyway. gz-sim's `DiffDrive` uses `gz::math::DiffDriveOdometry`, which tracks only X, Y and Heading — **no z, no pitch** — so `odom→base_link` has z ≡ 0 and the band rides with the robot. That accident evaporates the moment odometry carries true z, which is exactly FAST-LIO2, i.e. the real rig. It also carries no pitch, so on the 8.1° headland ramp an un-levelled plane drifts ~0.70 m over a 5 m lookahead. And the global costmap runs on `map` and is unaffected. Do not let a working local costmap in sim convince you the problem is solved.

**2.3 Canopy overhang does *not* block the alley, and height banding does not help.**
Measured max canopy radius vs height across all seven GLB meshes (free alley = 3.50 − 2r):

| z (m) | 0.25 | 0.50 | 0.75 | 1.00 | 1.50 | 2.00 | 3.00 |
|---|---|---|---|---|---|---|---|
| max r (m) | 0.694 | **0.861** | 0.839 | 0.740 | 0.678 | 0.574 | 0.429 |
| free (m) | 2.11 | **1.78** | 1.82 | 2.02 | 2.15 | 2.35 | 2.64 |

Min over all heights = 1.78 m. Min over 0–1.25 m only = 1.78 m. **Identical.** The tall spindle is conical, widest inside the robot band. If your real-orchard intuition was "the canopy closes the corridor in the projected grid," that intuition is wrong for tall spindle in 3.5 m rows — Robinson (NYSHS) specs 0.9–1.2 m tree diameter in 3.0–3.9 m rows precisely so it cannot. It *is* right for Y-/V-trellis or netted systems, which this is not. Note that Nav2's `obstacle_layer` default `max_obstacle_height` is already 2.0 m, so the canopy top is discarded out of the box and the alley still does not close.

**2.4 What actually pinches your corridor is at ankle height, not overhead.**
`gen_world.py:620-630` scatters **70 weed clumps per alley**, 0.10–0.26 m tall, Gaussian σ = 0.75 m about the **alley centreline** (~1.1 per metre over a 62 m row), plus 90 fallen fruit at 0.075 m. With Nav2's default `min_obstacle_height: 0.0` every one is marked. One lethal cell at the centreline plus the 0.417 m inscribed radius removes ~0.83 m from a 1.78–2.10 m corridor. A "robot height" band keeps all of them. This is the single most likely thing to destroy your corridor in this sim, and it is invisible if you are looking at the canopy.

**2.5 The terrace step is not a wall, and the reason it is non-drivable is not clearance alone.**
The 0.2647 m step is a **1.2 m smoothstep bank**, peak grade **30.8% = 17.1°**, comfortably inside Scout Mini's 30° gradeability. The steepest bank in the world is the 0.4974 m step at **57.5% = 29.9°** (analytic max 62.2% = 31.9°), i.e. *at* the rating. So the "26 cm > 115 mm clearance" argument is the wrong criterion — clearance governs belly grounding, and AgileX's published *step-climb* figure is 70 mm anyway, lower than the clearance. The bank is non-drivable for three other reasons, all of which your S2 layers capture: rise within the 0.627×0.55 m footprint is **0.169–0.318 m** (1.5–2.8× the 0.115 m clearance → high-centres even on the mildest step); the drive wheel radius in `sim/models/scout_mini_mid70/model.sdf` is **0.0875 m**, and a rigid wheel practically manages ~0.5r = 0.044 m; and 30° gradeability is a *fall-line* rating while crossing a bank is a **lateral side-slope traverse at up to 29.9°**, for which AgileX publishes nothing. Practically it is moot mid-row — the bank is centred on the row line with trunks every 1.5 m. Terrace changes happen only at the headland ramp.

**2.6 The slope threshold window is narrow and it is the tightest constraint in the design.**
It must sit above the headland ramp (8.1–8.4°, the only legal crossing) and below the mildest step face (17.1°). That is a **2.0× window**. Any DEM smoothing over more than ~0.3 m closes it by turning the 1.2 m bank into a ramp. Do not use the 30° gradeability figure as the threshold — it leaves every step face traversable.

**2.7 Sophisticated ground segmenters fail for one shared structural reason.**
Patchwork++'s Concentric Zone Model bins in polar coordinates around the sensor; a single ring at ~3.5 m radius spans two terraces and the bank between them, and its A-GLE elevation criterion adapts to a *unimodal* ground height distribution — a terraced field is multi-modal by construction. `linefit_ground_segmentation`'s critical parameter is a single scalar `sensor_height`. CSF's cloth-rigidness is one global scalar and the remote-sensing literature documents exactly this failure ("at the edge of the slope, some distances will appear between the cloth and the ground"). `octomap_server`'s `filter_ground_plane` is a single RANSAC perpendicular-plane fit. `ros-jazzy-autoware-ground-filter` 1.8.0 — the only apt-installable ground segmenter on Jazzy — propagates ground height radially outward from the sensor with slope checks tuned for ~15 cm road curbs. All of them assume one connected ground surface anchored to the sensor or globally smooth. And their selling point is 40+ Hz per-scan throughput, which buys you nothing because you accumulate offline.

---

## 3. Parameters

| Parameter | Value | Justification |
|---|---|---|
| DEM / traversability cell | **0.10 m** | 23 cells across a 2.3 m alley, 18 across the 1.78 m worst case. Skeleton lateral RMSE 5.5 cm here vs 4.1 @0.05 (4× cost) and 9.4 @0.20 with a much dirtier skeleton (236 vs 90 endpoints — the 0.30 m inter-tree gap aliases at 0.20 m). Upper bound is 0.20 m: 0.58 × 0.20 = 0.116 m intra-cell relief already collides with the 0.10 m step threshold. |
| Output OGM / Nav2 costmap | **0.05 m** | Nav2 native; nearest-neighbour upsample of the 0.10 m mask. 630×1440 = 0.91 MB. Do **not** estimate elevation at 0.05 m. |
| Ground percentile per cell | **20th** | Min-z latches on range noise and on canopy-only cells; CMU uses `quantileZ=0.25` but for a denser 360° sensor. 20th is the compromise for a 70.4° wedge at grazing incidence. |
| Min points per ground cell | **5** | Below this → `UNKNOWN`. Guards against a cell containing only foliage inventing ground at 2–3 m. |
| Ground-smoothing neighbourhood | **3×3 (0.3 m)** | Larger straddles the 1.2 m bank. |
| Ground-smoothing height gate | **0.15 m** | Ramp changes ground 0.043 m over 0.3 m (passes); 58% bank changes it 0.174 m (rejected). This gate *is* the terrace adaptation. |
| Step-height window | **7×7 (0.7 m)** | ≈ robot footprint (0.627 × 0.55 m). |
| Step-height LETHAL | **0.10 m** | Alley cells 0.006–0.019 m (≈9× margin); mildest bank over this window 0.216 m (≈2.2× margin); below the 0.115 m datasheet clearance. Use 0.115 m, not the sim's 0.121 m collision-box value. |
| Step-height soft onset | **0.05 m** | |
| Slope LETHAL | **12°** | Window is (8.4°, 17.1°) — above the headland ramp, below the mildest bank. Midpoint-ish, biased low because DEM smoothing only ever flattens banks. |
| Slope soft onset | **9°** | Just above the 8.4° ramp. |
| Roughness (σ of plane residual, 7×7) | soft **0.02 m**, lethal **0.05 m** | Currently meaningless until prerequisite (b) — 1.89 cm quantisation is the entire measured signal. |
| Relief (p90−p10 per cell) | **< 0.08 m** for drivable | Secondary vegetation/canopy guard. |
| Negative obstacle (`drop`, 7×7) | **> 0.10 m → lethal** | Terrace drop-offs are 0.26–0.50 m. CMU's `considerDrop` defaults to **false** — that default would make driving off a 0.5 m terrace edge invisible. |
| Wall AGL band | **[0.30, 1.30] m** | Low cut above the 0.26 m max weed; ceiling above the 1.20 m camera mast. Buys 0.00 m of alley width (§2.3) — it is not a canopy filter. |
| Clutter AGL band | **[0.12, 0.30] m** | Weeds + fallen fruit → soft cost only. |
| Occupied threshold | **≥ 3 points** in band | Trivial after accumulation. |
| Row-heading search | ±10°, 0.1° steps | Rows are along +y by construction; estimate anyway. |
| Anisotropic closing kernel | **(1, 13) = 1.3 m along row** | Measured: junctions 519→80, false crossings 105→0, RMSE unchanged. ≈1× tree spacing: welds 0.30 m notches and 1.80 m single-gaps, leaves a genuine 4.8 m 3-tree gap open. Disable for \|y\| > 30 m. |
| C-space erosion | **0.50 m** | 0.417 m circumscribed radius + 0.083 m. Leaves 0.78 m of corridor at the 1.78 m worst case. |
| Thinning | `cv2.ximgproc.THINNING_ZHANGSUEN` | 63.9 ms on the full 1440×770 grid vs 192.2 ms for `medial_axis`, and 8× fewer spurs. |
| Skeleton spur prune | **15 iterations = 1.5 m** | One tree spacing. |
| Cycle collapse threshold | **enclosed area < 2.25 m²** | tree_spacing². Cycles, not spurs — pruning cannot touch loops. |
| Nav2 `inflation_radius` | **0.60 m** | Above ~0.80 m the alley closes entirely and you get an unsolvable planning problem that looks like a mapping bug. |
| STVL `obstacle_range` | **3.0 m** | Usable annulus is 0.9–3.0 m (§6). |
| STVL `voxel_decay` | **10.0 s** | The alley beside you is 2.8–3.3 s stale at 0.5 m/s; decay must exceed that. |
| STVL `min/max_obstacle_height` | **−100.0 / +100.0** | Deliberately disabled; filtering moved upstream to the AGL node. |

---

## 4. Implementation — what to install vs write

### Install: essentially nothing
`numpy 1.26.4`, `scipy 1.11.4` and `python3-opencv 4.6.0` are already present, and **`cv2.ximgproc.thinning` is compiled into the stock Ubuntu 24.04 opencv package** (verified: `hasattr(cv2,'ximgproc')` is True). The entire core pipeline runs today with zero installs. `apt install python3-networkx` (2.8.8) only for the evaluation script's graph-edit-distance metric — or hand-roll union-find in ~40 lines and skip it. Do not reach for `scikit-image` / `sklearn` / `shapely`; none are installed and none are needed.

Optional, visualisation only: `ros-jazzy-grid-map-rviz-plugin` + `ros-jazzy-grid-map-msgs`, purely to see the terraces in RViz.

*Note:* `/etc/apt/sources.list.d/ros2.sources` may still carry `Enabled: no`; the cached index makes `apt-get install -s` succeed and give a false sense of readiness. Re-enable before any real install. The URI is already `http://`, which is the existing workaround for the recorded certificate mismatch.

### Reject, decisively

| Package | Verdict |
|---|---|
| **`grid_map` 2.2.2** | **Do not install for this.** `[VERDICT OVERRIDE]` It ships **no traversability filter**. `grid_map_filters` exports exactly 16 generic plugins (BufferNormalizer, ColorBlending, ColorFill, ColorMap, Curvature, Deletion, Duplication, LightIntensity, MathExpression, MeanInRadius, MinInRadius, NormalColorMap, NormalVectors, SetBasicLayers, SlidingWindowMathExpression, Threshold). "Traversability" exists only as a **demo YAML recipe** in `grid_map_demos/config/filters_demo_filter_chain.yaml`, where it is a `MathExpressionFilter` with `0.5*(1.0-(slope/0.6)) + 0.5*(1.0-(roughness/0.1))`. That 0.6 rad = 34.4° knob would score our 29.9° bank at ~0.06 rather than rejecting it, and its `NormalVectorsFilter` radius of 0.05 m is far too small for ~7,300 returns/frame. `SlidingWindowMathExpressionFilter` evaluates an Eigen expression per cell — seconds, not milliseconds, on 907k cells. You would write the same 200 lines of kernel anyway and additionally pay for a C++ node, a custom message type, and a rosdep. Numpy + `nav_msgs/OccupancyGrid` + an `.npz` is strictly less machinery. |
| **CMU `terrain_analysis`** | Mine, do not adopt. Read `terrainAnalysis.cpp:528-600` for the quantile-ground algorithm and `noDataObstacle`; note `considerDrop=false` at line 43 is the most dangerous default in the space. But it is a robot-centred **rolling 51×51 @ 0.2 m = 10.2 m** window with 2 s decay — reactive per-scan, not the offline whole-map segmentation we want, and its 3×3 (0.6 m) neighbourhood straddles the 1.2 m bank. |
| **Patchwork++** | ROS 2 wrapper is real (`url-kaist/patchwork-plusplus`, `ros/package.xml` → package `patchworkpp` 0.1.0, `ament_cmake`) but is **Humble-badged, source-only, never released to any rosdistro**, with no Jazzy CI. Structurally wrong anyway (§2.7). |
| **`elevation_mapping`** | **Zero ROS 2 support and formally discontinued.** Trap: the repo *has* a branch named `ros2`, but it is catkin (`<buildtool_depend>catkin</buildtool_depend>`, v0.7.6, frozen 2022-02-05). Master's head commit is "Discontinue Elevation Mapping Maintenance". |
| **`elevation_mapping_cupy`** | ROS 2 exists only on non-default branches (`release/jazzy`, v2.1.0). Requires CUDA + `cupy-cuda12x` + a CUDA torch build. `[MEMORY CORRECTION]` The stored note "NVIDIA blocked by Secure Boot" is **stale** — `nvidia-smi` reports a working GTX 1660 SUPER on driver 580.173.02 / CUDA 13.0 with Secure Boot still enabled (MOK-enrolled). But there is **no CUDA toolkit** (`nvcc` absent, `/usr/local` empty) and no CuPy. Not a fallback. |
| `traversability_estimation`, `linefit_ground_segmentation`, `kiss-icp`, `octomap_server` | No ROS 2 branch at all / ROS 1 only / wrong problem / single-plane RANSAC ground fit. |

### Write ourselves (~700 lines total, all Python)

| File | Lines | Content |
|---|---|---|
| `scripts/gen_world.py` (patch) | ~25 | GT dump (§0a) |
| `scripts/gen_heightmap.py` (patch) | ~15 | 16-bit PNG, 1025 px, `--ramp-len 2.5 --headland 8.0` |
| `scripts/build_dem.py` | ~150 | S0–S2: voxel decimate, percentile DEM, gated smoothing, feature layers → `orchard_dem.npz` |
| `scripts/build_traversability.py` | ~120 | S3–S4: AGL bands, drivable mask, unknown handling → `.pgm`/`.yaml` |
| `scripts/extract_corridor.py` | ~250 | S5–S8: heading, closing, C-space, row-model fit, thinning, prune, cycle collapse, graph → GeoJSON + CSV |
| `ros2_ws/src/orchard_sim/orchard_sim/agl_filter_node.py` | ~120 | S9: online per-point AGL, two republished clouds |
| `scripts/eval_corridor.py` | ~200 | §5 metrics |
| `scripts/inject_pose_noise.py` | ~60 | §5.3 |

**Keep as-is:** `livox_sim_bridge`, `gt_localizer`, `sdf_static_tf`, Nav2 1.3.12, `nav2_route` 1.3.12, STVL 2.5.5 (reconfigured per S9).

---

## 5. Evaluation protocol

Everything here requires prerequisite §0(a). This is the highest-value thing this simulation can give you: it converts *"detecting the drivable corridor is hard"* from a judgement into a regression number you can hold against your real-orchard memory.

### 5.1 Core metrics

| # | Metric | Definition | Success | Notes |
|---|---|---|---|---|
| 1 | **Centreline lateral RMSE** | per-alley and pooled, extracted vs GT `alley_centrelines`, sampled every 0.25 m in y | RMSE ≤ **0.10 m**, p95 ≤ 0.15 m, max ≤ 0.25 m | Geometric floor on clean grids is **5.5 cm @ 0.10 m**. >15 cm indicts perception, not geometry. |
| 2 | **Corridor width bias** | 2 × EDT along the extracted centreline, vs GT free width from canopy footprints | \|bias\| ≤ **0.10 m**, RMSE ≤ 0.15 m | Signed. Negative = over-segmentation (timid). Positive = false openings (dangerous). Truth is **1.78–2.30 m**, not a flat 2.30. |
| 3 | **Drivable-cell precision / recall / IoU** | vs GT mask = alley polygons ∩ (\|x−row_line\| > 0.6) minus canopy/weed footprints | precision ≥ **0.98**, recall ≥ 0.85, IoU ≥ 0.90 | **Report separately.** Recall loss = timidity. Precision loss = false openings. Opposite consequences; a single IoU hides both. |
| 4 | **Alleys recovered end-to-end** | fraction with ≥95% y-coverage | **9/9** | |
| 5a | **False row crossings** | extracted graph edges crossing a GT `row_line` inside \|y\| < 30 | **0** — hard gate | Baseline without S6 closing: 105 px, 10/10 rows breached. |
| 5b | **Excess junctions** | measured vs ideal 18 | ≤ **20** | Raw medial axis gives ~515. |
| 5c | **Independent cycles in-alley** | Euler characteristic (components 8-conn vs background 4-conn) | **0** | Raw medial axis gives 207. |
| 5d | **Graph edit distance** | vs ideal 18-node / 25-edge graph | **0** | |
| 6 | **Step-face recall** | fraction of GT bank cells (\|x−row_line\| < 0.6) classified non-drivable | ≥ **0.95** | |
| 7 | **Negative-obstacle safety** | drivable cells within 0.5 m of a >0.20 m GT drop | **0** | Safety gate; failing this means a robot can be driven off a 0.5 m terrace edge with the map reporting clear. |
| 8 | **Ground-height MAE** | DEM `ground` vs `heightmap.npy`, per alley | ≤ **0.03 m** in-alley | Isolates S1 from everything downstream. |

### 5.2 Sweeps
- `missing_prob` ∈ {0.00, 0.03, 0.10, 0.20} — at 0.03, P(a row contains ≥1 gap) = 1−0.97⁴¹ = **71.3%**, ~12 of 410 trees absent per world. One missing tree opens a **1.80 m** lateral gap (2×1.5 − 1.2), over 3× robot width, i.e. a genuinely drivable false opening. Two consecutive: 3.30 m. Three: 4.80 m.
- `canopy_width` 1.2 → 2.8 m — alleys stay topologically open until ~2.8 m (0.70 m free). Note this world currently **cannot reproduce** the canopy-closure failure mode; the generator needs a lateral-growth parameter before that is studyable.
- Grid resolution 0.05 / 0.10 / 0.20 m.
- Anisotropic-closing length 0 / 0.4 / 0.8 / 1.2 / 1.6 m — the measured junction curve is 519/519/177/177/80/80.

### 5.3 The experiment that actually addresses the real-orchard question
`inject_pose_noise.py` corrupts `gt_localizer`'s output before accumulation: σ_pitch ∈ {0, 0.25, 0.5, 1.0}° and a z random walk of σ ∈ {0, 0.05, 0.10, 0.20} m over 60 m. Re-run S0–S7 and plot metrics 1, 3, 5a, 8 against noise.

Why this matters more than any threshold tuning: at grazing incidence the ground depression angle falls to **4.6° at 8 m and 3.1° at 12 m**, so 1° of pitch error becomes **14 cm of z error at 8 m and 21 cm at 12 m** — the latter exceeding the 0.20 m obstacle threshold on its own and fabricating phantom terraces. Your Stage-0 localizer makes that term exactly **zero**, so the whole problem is invisible in sim today. In the real orchard that term is supplied by FAST-LIO2, and z/pitch are its weakest axes for a forward-facing 70° FOV in a straight, self-similar, repeating-tree corridor — along-track translation is weakly observable, and plausible drift is the *same order* as the 26–50 cm steps you are trying to measure. **My working hypothesis is that your real-orchard failure was map corruption, not segmenter failure.** Fixing a segmenter cannot fix a map whose terrace heights are already wrong. This experiment costs a day and could change what you build. If metric 1 stays under 0.10 m at σ_pitch = 0.5° but blows past 0.30 m at 1.0°, you have your answer and your spec for the localizer.

---

## 6. The MID-70-only handicap

### The mechanism is not what it seems
The intuitive model — "the robot passes alongside the walls and maps them" — is **false**. `[VERDICT OVERRIDE]` Ray-casting the sensor exactly as configured in `sim/models/scout_mini_mid70/model.sdf` (113×113 grid, ±0.6143 rad h and v, circular mask → 9,845 rays/frame, matching the measured ~9,850) over a 20 m drive: of 1.08 M wall returns, **exactly 0** landed within ±0.5 m laterally of the robot. Minimum forward lead of *any* wall return is **1.42 m** (2.0 m alley) / **1.63 m** (2.3 m alley). Passing alongside a wall contributes literally nothing. Every wall point is acquired while **approaching head-on**; the wall then exits the 35.2° half-cone and is never seen again.

### Consequences, in order of importance

**(1) Accumulation is mandatory, not an optimisation.** The robot physically cannot observe the bank beside it. Per-scan traversability of the alley walls is *geometrically impossible*, not merely inaccurate. This is why S1–S7 are offline. It also means localisation drift propagates **directly** into the corridor estimate, with no independent check.

**(2) Everything you see, you see stale.** At 0.5 m/s the freshest observation of the wall you are currently driving between is **2.84 s (2.0 m alley) / 3.26 s (2.3 m alley)** old; at 1.0 m/s, 1.42/1.63 s. Lateral centring runs **open-loop over 1.42–1.63 m of travel, every time**. Set `voxel_decay ≥ 10 s` accordingly, and do not build any controller that assumes fresh lateral measurements.

**(3) Every wall measurement is grazing.** Median incidence off the wall normal is **70.9°** (5–95th pct 58.0–84.3°), median range 3.05 m. Footprints are elongated ~3× along the row. This caps achievable lateral precision of the extracted wall face, and it is why the 0.10 m target in §5.1 is realistic and 0.05 m is not.

**(4) Density collapses with height.** Best case (opaque flat wall, no self-occlusion), 0.10 m cells, 20 m drive at 0.5 m/s, 2.0 m alley — cells hit / points per cell: 0.0–0.4 m: 100% / 226; 0.4–0.8 m: 100% / 269; 1.2–1.6 m: 100% / 167; 2.0–2.4 m: 95.3% / 73; **2.8–3.2 m: 86.2% / 31** — a 9× collapse. A point at the 3.2 m canopy top only enters the cone from **3.89 m** ahead. With a realistic occluding hedge (r = 0.6 m cylinders at 1.5 m pitch on the row line), cell fill drops to **58.6%** at chest height with **41.4%** of along-row columns getting zero returns in the 0.6–1.4 m band. **This is a further argument for the [0.30, 1.30] m wall band**: it sits in the only well-sampled part of the wall.

**(5) The occlusion shadow is systematic and direction-dependent.** Because the staircase is monotonic, from the low terrace the bank always occludes the terrace above it — the *same side* of every alley, and which side depends on driving direction. Those cells receive no ground returns; if a 20th-percentile is applied to a cell holding only foliage it invents ground at 2–3 m. Hence explicit `UNKNOWN` handling and the two-direction accumulation in S0. This is confident, plausible, wrong ground — the worst failure class.

**(6) Off-centre observation is asymmetric.** At 0.3 m lateral offset in a 2.3 m alley the near wall yields **30% more points/frame** and first becomes visible at 1.21 m lead vs 2.06 m for the far wall. A naive "midpoint between the two point-cloud walls" estimator therefore has a built-in bias toward the near wall. The S7 row-model fit is immune to this because it fits a global lattice; the skeleton is not. Another reason the model fit is primary.

**(7) Sim flatters the sensor.** gz-sim's `gpu_lidar` is a **uniform** 113×113 grid. The real Mid-70 is a Risley-prism rosette whose "observation density within the FoV is nonhomogeneous with a peak at the center" (Brazeal et al., *Sensors* 21(14):4722). All wall returns land at the **FOV edge**, the thinnest part of the real pattern — and the Mid-70 manual states "the closer to the edge of the FOV, the shorter the effective detection range is." Real coverage reaches only 32-line-equivalent in 100 ms and needs ~1.5 s of integration for 86%. Discount the density numbers in (4) accordingly.

### The cost, and what does *not* buy it back
Usable marking annulus is **0.9–3.0 m** (0.914 m ground blind spot = 0.645/tan 35.2°, matching the measured 0.91 m; the ±1.75 m step face does not enter the FOV until 2.48 m ahead, leaving only ~0.5 m of margin against STVL's 3.0 m default `obstacle_range`). At the blind-spot boundary lateral coverage is only 2 × 0.64 = **1.28 m against a 2.0–2.3 m alley**.

**Tilting or yawing the MID-70 does not add lateral coverage** — it only moves the blind spot. This is pure FOV geometry given the locked mount at (0.275, 0, 0.645) with zero tilt. The only things that would change items (1)–(3) are a second sensor or a nodding mount, both of which are outside the locked decision. Everything in this plan is designed around the constraint rather than against it.

---

## 7. Honest failure list — ranked by what bites first in this sim

1. **STVL silently deletes the world on alleys 3–9.** Global-frame `min/max_obstacle_height` discards 100% of points where the whole band sits below that alley's ground. Symptom: clean empty costmap, smooth planning, robot drives into trunks. Simultaneously on alley 0 the ground is marked lethal and the robot looks "stuck for no reason." Same root cause, opposite symptoms. **Fix is S9 and it must be first.** Extra trap: the local costmap will look fine anyway because gz `DiffDriveOdometry` has z ≡ 0 — a coincidence that vanishes with FAST-LIO2.

2. **Weeds and fallen fruit pinch the corridor from the middle.** 70 clumps/alley at σ = 0.75 m about the centreline, 0.10–0.26 m tall, rigid in sim physics. With `min_obstacle_height: 0.0` every one is lethal; one centreline cell plus 0.417 m inscribed radius removes ~0.83 m from a 1.78–2.10 m corridor. The two-band split in S3 exists for exactly this. Will present as "the alley is blocked" and be misdiagnosed as canopy.

3. **The headland crossing fails and looks like a perception bug.** Legal crossing only at |y| ≥ 34.5 while the block ends at 36.0. Nav2 will fail at row change for reasons that have nothing to do with traversability. Prerequisite §0(c).

4. **Phantom ground in the uphill occlusion shadow.** Systematic, same side of every alley, direction-dependent. Produces confident, plausible, wrong DEM. Mitigated by `n < 5 → UNKNOWN` + two-direction accumulation, but the fill rule for enclosed unknowns is the exact place a subtle bug will hide.

5. **The row-heading estimate (S5) drifts and takes S6 with it.** A 2° heading error makes the anisotropic kernel cut *across* rows instead of along them, and the closing goes from the highest-value line in the pipeline to actively harmful. Add an assertion: after rotation, the row-projection histogram must show ≥ 8 peaks at 3.5 ± 0.15 m spacing.

6. **The 0.15 m gated-neighbourhood is the main piece of new, untested logic.** It has no upstream reference implementation. Unit-test it directly against the nine known `terrace_steps` before anything downstream: synthesise a DEM from `heightmap.npy`, run S1, assert per-alley ground MAE ≤ 0.03 m and that no smoothed cell straddles a step.

7. **The slope window is only 2.0× wide (8.4°–17.1°).** Any smoothing over >0.3 m, any DEM upsampling artefact, or the sim's own 0.2344 m/px bilinear heightmap interpolation narrows it further. Regenerating at 1025 px (prereq §0b) widens the margin; without that, the sim's banks are gentler and smoother than real terrace faces and slope-based rejection will look more reliable than it is.

8. **Roughness thresholds are currently measuring a PNG.** 1.89 cm/level quantisation *is* the measured 0.6–1.9 cm "flatness." Any threshold tuned to it is badly over-optimistic against a real orchard floor with ruts, grass tussocks, windfall fruit and irrigation line. The 9× step-height margin is **not** transferable.

9. **Missing-tree false openings if S4 is skipped or ordered wrong.** ~12 per world, 71% of rows breached, each a 1.80 m genuinely-drivable-in-2D opening. On *this* terrain the traversability mask catches them because the bank exists regardless of tree presence — which means the pipeline will look robust here for a reason that will not hold on flat real ground. Verify S6 alone also zeroes metric 5a, with S4 disabled, so you know both defences work independently.

10. **Circularity in the row-model fit.** It assumes straight, parallel, equally-spaced rows — which is *literally how `gen_world.py` places trees*. Evaluating it here is close to circular. Real orchards have curved rows, terrace-following rows and variable spacing. The skeleton path generalises; the model path is robust. Do not let good sim numbers on metric 1 create false confidence — the model-vs-skeleton disagreement detector (> 0.3 m ⇒ flag) in S7 is your only guard against this in the field.

11. **`nav2_route` coordinate convention.** The shipped sample graphs carry `crs: EPSG::3857` while the node `frame` property says `"map"`. Verify a hand-written two-node graph end-to-end before generating 9 alleys programmatically.

12. **Everything above is unmeasurable until `gen_world.py` dumps ground truth.** It currently writes only SDF. Reconstructing tree positions by re-running the RNG requires replicating the exact interleaved call order (missing check → jitter x → jitter y → yaw → model pick) — fragile, and getting it subtly wrong silently corrupts every metric in §5.

---

**Files referenced:** `/home/myhome/YBNML/scripts/gen_world.py`, `/home/myhome/YBNML/scripts/gen_heightmap.py`, `/home/myhome/YBNML/scripts/gen_tree.py`, `/home/myhome/YBNML/sim/models/orchard_terrain/heightmap_meta.json`, `/home/myhome/YBNML/sim/models/orchard_terrain/heightmap.npy`, `/home/myhome/YBNML/sim/models/scout_mini_mid70/model.sdf`, `/home/myhome/YBNML/ros2_ws/src/orchard_sim/orchard_sim/{gt_localizer.py,livox_sim_bridge.py,mapping_run.py}`, `/opt/ros/jazzy/share/nav2_route/graphs/sample_graph.geojson`.