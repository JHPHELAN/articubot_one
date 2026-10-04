# 2026-10-03 — Low-obstacle (hall-table shelf) detection

## Symptom
Stormy drives *under* the hall table and hangs on its low shelf (measured **0.136–0.156 m**
off the floor). The shelf never becomes a lethal obstacle in the costmap, so Nav2 plans and
drives into/under it. The table is open: a **shelf + 4 thin legs**, wall behind.

## Root cause (established live, not assumed)
- The OAK-D **sees** the shelf richly — thousands of in-band points at the correct height,
  continuously, even stationary.
- Those points transform into the **correct, in-grid local voxel** (iz=1, odom 0.05–0.10 m),
  **0 % dropped**.
- Nav2's VoxelLayer **never records a MARK** on them — the voxel stays `UNKNOWN` — while the
  taller **wall marks fine** at iz=2 (~0.19 m).
- Ruled out, each tested on the live robot:
  - **Clearing** — OAK clearing off, then scan clearing off (separately): no change.
  - **Height gate** — lowered global `min_obstacle_height` 0.10 → 0.06: cell went
    UNKNOWN→FREE but never MARKED.
  - **origin_z / z-reference** — shelf points land at odom z 0.062–0.094, 0 % below origin_z.
  - **Resolution** — points sit cleanly inside one voxel; finer voxels give the same cell.
  - **`mark_threshold`** — already 0 (max sensitive); irrelevant since there are zero hits.
  - **Marking-only + max-combination** ("see something, say something, keep it") — still
    UNKNOWN, so the mark genuinely never fires.
- **Common thread:** the shelf returns are **low, close, and just above the camera**
  (sensor odom z ≈ 0.03, shelf 0.062–0.077) → **near-horizontal** returns that Nav2's
  voxelization places in the right cell but does not register as obstacles. This is an
  **ingestion/geometry** property, below the costmap parameter surface. No software knob
  found reaches it.

## Side findings
- **OAK physically nose-down ~1.4°** (anti-roll mount screws), **unmodeled** → costmap saw a
  sloped floor and a band of **below-floor cruft** (~13 % of frontal returns): specular tile
  reflections + grazing-angle stereo noise.
- **Lidar height modeled ~8.6 mm low** vs measured.
- **Localization drift** (`map→odom` ~20 cm over a run) → the recurring **imperfect docking**
  (separate thread, not yet addressed).

## Changes deployed (this commit)
1. **OAK mount pitch 0 → 0.027 rad (~1.55° nose-down)** — `robot.urdf.xacro`.
   Corrects the screw-induced tilt. Floor plane went ~1.4–1.9° slope → **+0.17° residual**
   at 0.024 rad; added the 0.17° (→0.027) to zero it. The 0.17° is within measurement noise
   (~6 mm at 2 m); **re-verify on CLEAN floor** (no table ahead) and adjust if needed.
2. **Lidar stack-up fix → base_laser 0.1757 m** (measured 0.1755).
   `stingray_properties.xacro`: `chassis_height` 0.081 → 0.0882 (3" side panels + 2×6 mm
   plates, base_link on centerline). `ldlidar.xacro`: `ldlidar_chassis_height` 0.0228 →
   0.022 and scan-plane formula → 0.032. **Side effect:** e-stop/battery frames rise 3.6 mm
   (correct — they mount on the now-accurate chassis top).
3. **Keepout over the hall table** — `assets/maps/keepout_mask.pgm`, map rectangle
   **x[-1.12, 0.0] × y[1.50, 2.19]**. Deterministic net; verified planner refuses goals in
   it. NOTE: pinned to the current map frame — re-check if SLAM relocalizes.
4. **Global OAK `min_obstacle_height` 0.10 → 0.06** — `nav2_params.yaml`.
   Safe now that the pitch fix flattened the floor; catches lower obstacles the layer *does*
   ingest. Does **not** fix the shelf (that's the ingestion issue above).

## Recommended next steps
- **Up-tilt the OAK ~½ its vertical FOV (hardware).** Moves low returns out of the
  degenerate near-horizontal regime so marking fires, and cuts floor cruft. Re-model the
  pitch in the URDF, then re-run the voxel probe to confirm the shelf marks.
- **Add up-tilted sonar** for glass / clear / thin obstacles that lidar *and* stereo miss.
- **Reduce below-floor cruft:** enable OAK depth confidence/temporal filters
  (`oakd_nav_slim.yaml` has `i_enable_threshold_filter`), and try **headlights OFF on tile**
  (specular reflections make it worse).
- **Investigate docking imprecision** (localization drift at the dock; is the dock routine a
  plain nav-to-pose or a detection-based final approach?).

## Diagnostic method (live rclpy probes, run from /tmp during session)
- `verify.py` — floor-flatness + base_laser height + OAK pitch from TF.
- `diag.py` / `diag_layer.py` — cost vs forward distance, height-resolved; master vs
  obstacle/voxel layer.
- `voxdump.py` / `zmatch.py` — decode the raw `/local_costmap/voxel_grid`; correlate OAK
  cloud-z to the voxel it lands in and whether it's MARKED.
(Available to commit to a `scripts/diagnostics/` folder if we want them reusable for the
up-tilt re-verification.)
