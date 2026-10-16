# Autonomy (branch `autonomy`)

Full-autonomy stack for the 2027 Lunabotics rules (guidebook 2026-2027, sections 11, 13-17), built on the
`upgrade/lyrical` port ([MIGRATION.md](MIGRATION.md), [REVIEW.md](REVIEW.md)).

## Decisions made without asking

Each one is the call I made where the brief or the rulebook left a choice; change any of them by editing the named file.

| # | Decision | Why | Where |
|---|---|---|---|
| D1 | **Arena frame = rulebook frame**: origin at the starting-zone corner *measured between the ducts* (interior 6.88 × 5.0 m), X along the length toward the berm, Y across. | 11.2 gives interior dimensions between the ducts; the berm centre (5.38, 0.9) is "from the Arena Wall in the corner of the starting zone". If the judges' origin is the wall itself, the berm moves 17 cm; that is one measured offset in `arena_tags.yaml`. | `lunabot_config/params/arena/arena_tags.yaml` |
| D2 | **Column at (3.6, 2.65), 0.45 m square** in the sim. | Not dimensioned in the guidebook; read off Figure 9 (scaled drawing). Autonomy never uses it: the column is only detected online. | `lunabot_sim/scripts/generate_arena.py` |
| D3 | **Construction zone x ≥ 3.88, y ≤ 1.5** in the sim; obstacles are not placed there. | Figure 9 ("~1.5 m" along the y = 0 wall). 13.9.15 allows pushing obstacles there, so the sim leaves it clear like the figure. | generator |
| D4 | Sim craters are bowls 40–50 cm wide, **20–45 cm deep**; boulders 30–40 cm with **12–35 cm** exposed height. | 11.4: "not exceeding ~40–50 cm in either measurement"; "varying heights". | generator |
| D5 | **Compass names for the sim start heading**: east = +X (toward the berm), north = +Y. | The guidebook only says N/E/S/W are drawn at random; the mapping is arbitrary and only matters for reporting. | `generate_arena.py`, `gui_launch.py` (`arena_type:=artemis2027` only; the old arenas keep their mapping) |
| D6 | **AprilTag 36h11, 0.25 m black edge (0.3125 m printed incl. white border), IDs 0–3, 2 per side** of the starting-zone frame, centres 0.40 m above the regolith. | See [Tags](#tags). | `arena_tags.yaml`, `apriltag/fisheye_36h11.yaml` |
| D7 | **Localize from tag *centres* only** (2D rigid fit over ≥ 2 tags), not single-tag PnP orientation. | Convention-free and robust; with 4 tags on two perpendicular sides the fit is well conditioned. | `lunabot_nav/scripts/arena_localizer.py` |
| D8 | **Dump side = rear.** The bucket pivots at x = −0.204 and tips back over the rear; the real `auto_depositing_server` reverses before dumping. Dump poses put the rear edge (0.8 m) parallel to the berm box's long axis. | URDF + server code. | `mission_executive.py` |
| D9 | **MCC link = `foxglove_bridge` with allow-lists**, not `rmw_zenoh`. | See [MCC link](#mcc-link-and-bandwidth). | `lunabot_config/params/mcc/foxglove_bridge.yaml` |
| D10 | **No a-priori wall/column geometry anywhere in localization or the costmaps.** Walls, ducts and the column reach the planner only through online sensing (RTAB-Map `/map` and the terrain layer). Goal *positions* (berm, zones) are rulebook coordinates in the tag-defined arena frame. | 16.3.2 and 11.2.3. A geofence from the arena dimensions would be easy, but it is exactly the kind of a-priori wall use the autonomy judges are told to look for. | costmap params |
| D11 | **Our own pits are drivable.** The terrain mapper judges height against the local ground, and craters are lethal only in the obstacle zone. In the excavation and construction zones a depression is not an obstacle, and rock-sized blobs (often our own dump piles) stay lethal but get no hard margin. That margin is there for the obstacle-zone contact penalty, and beside a pile it blocked the next dump slot (0.7 m apart). | Last season the robot dug a pit it then had to drive into. 17.1 item 8: the crater/rock penalty applies only in the obstacle zone; filling craters is allowed in the excavation and construction zones. Zone bounds are rulebook coordinates in the tag frame, like the goals (D10). Before the arena frame is known, everything is treated as obstacle zone. | `terrain_mapper.py` (`dig_zones`, `ground_window`) |
| D12 | **Sim digs pits and builds piles** (test harness). Each "excavation complete" cuts the litres-per-load out of the ground under the pass (0.6 × 0.35 m, 35° sides); each "dump complete" adds a 35° cone behind the robot. The regolith model is swapped for the edited mesh. | The sim has no granular regolith; without this, pits, pile growth and dump-slot blocking are never exercised. Payload mass and real digging still aren't simulated. | `tools/run_eval.py` (`Ground`), `--no-ground-edits` |

## Rulebook constraints and how each is met

| Rule | Implementation |
|---|---|
| No GPS/compass (13.7.1) | None used. Heading comes from Point-LIO (IMU + lidar) and the tags. |
| Fiducials only on the starting-zone frame, 2 sides, no holes (13.7.2) | 4 passive printed tags on those two sides, taped/clamped. Mass: paper + clamps, < 0.2 kg against the 80 kg limit (13.4.4). They need no power. |
| No a-priori wall/column for localization, no wall following (16.3.2) | D10. Walls and the column are features RTAB-Map learns online. |
| No point-and-traverse / bread crumbs (16.3.1, 16.3.3) | Every goal is computed on the robot from the map (lane and dump-slot selection by planned cost-to-go). Nothing is sent from the MCC during autonomy. |
| Live MCC visualization of obstacle detection, map and planned path (16, 16.3.3) | `/mcc/map`, `/mcc/obstacles`, `/mcc/plan`, `/mcc/pose` via the bridge, shown in `lunabot_gui` (and any Foxglove client). |
| Hands-free from the starting zone (16.5) | `mission_executive` starts in the starting zone; localization can include in-zone spins (16.5.1 allows remote moves there; we do it autonomously). |
| Start a cycle within 5 min (15.3.1) | Executive excavates in place by 4:00 if the tags have not fixed the arena frame yet. |
| Inhibit at the end of the timer (15.4.1) | Executive cancels everything and zeroes `/cmd_vel` when its run clock expires, and on `/mission/inhibit`. Dumping already in progress may finish (15.4.1); excavation may not. |
| Don't excavate in the obstacle/construction zones (13.9.14) | Excavation lanes are restricted so the whole swept footprint stays at x < 2.45. |
| Obstacle penalties (17.1.8) | Rocks/craters get hard margins (rock 0.15 m, crater 0.20 m) plus a soft inflation cost. See [Cost model](#cost-model). |
| No wall contact (13.9.4b, 13.9.6) | Walls/ducts are detected online and treated as lethal + inflation. Contacts measured by the harness. |
| Bandwidth ≤ 4 Mbps average, scored (14.1.19, 17.1.4) | MCC link target < 0.1 Mbps, no NASA cameras. |

## Tags

*(Selection, placement procedure and detection-range results: filled in below as measured.)*

**Why the fisheye.** The D456s are mounted pitched 48° down (URDF `base_camera_front_joint` / `base_camera_back_joint`):
the top of their 65° vertical FOV is 15.5° below horizontal, so a tag on the frame at 0.25–0.55 m is fully visible only within
about 0.1–0.5 m. The fisheye sits at 0.58 m, pitched 45° down with a 120° × 104° FOV and a pan joint. It sees from 7° above the
horizon down, which covers a tag at 0.4 m at any distance in the starting zone, and panning ±90° (plus the robot's own spin if needed)
covers all four headings. `arena_localizer` sweeps the pan while it has no fix.

**Size.** At 0.25°/pixel (480 px over 120°), a 0.25 m tag is ~19 px across at 3 m and ~38 px at 1.5 m. 36h11 decodes reliably
above ~15 px. The farthest a robot centre can be from both tag sides inside the 2 × 2 m starting zone is ~1.5 m, and re-observation
from the excavation zone happens at 1.5–3 m.

**Family.** 36h11: the largest Hamming distance of the common families (lowest false-positive rate), the `apriltag_ros` default, and
the family the team's RTAB-Map marker config already uses (`Marker/Dictionary: 20`). `max_hamming: 0`, because a mis-decoded ID
would corrupt the arena frame.

## Cost model

Every number has a physical reason; nothing is tuned to a sim artifact.

**Perception → costmap** (`lunabot_nav/scripts/terrain_mapper.py`, `/terrain/grid`, used by both costmaps):

| Quantity | Value | Reason |
|---|---|---|
| Ground reference | local: mean of each cell's lowest return within 1.0 m (two passes, rocks/craters excluded), in the gravity-aligned frame | D11. A robot in its own pit would see the ground around it as rock against its own height. Independent of odom z drift. 1.0 m is several crater widths, so a crater can't be its own reference. |
| Rock threshold | > 0.10 m above ground, ≥ 2 scans, more high than ground evidence | Boulders are 30–40 cm across with varying heights; lidar noise ~2 cm, raked undulation ~±2 cm. 10 cm is under the robot's ~0.1 m wheel-radius clearance. |
| Crater threshold | > 0.10 m below ground, same evidence rule, **obstacle zone only** | Craters are 40–50 cm wide and deep. Grazing rays only see the far wall: at ~2 m and beyond, a crater shows as unknown (shadow) before it shows as deep. In the excavation/construction zones depressions are our own pits (D11). |
| Unknown cells | value 25 → Nav2 cost ~63 (non-trinary) | "Driving near unknown space" costs extra, so plans go through observed ground where they can. |
| Rock hard margin | 0.15 m, painted lethal, obstacle zone only | −50 for the first contact (obstacle zone only, 17.1 item 8). Pose error (≤ 0.10 m measured, LIO 1–10 cm) + tracking error. |
| Crater hard margin | 0.20 m, painted lethal | A wheel in a crater is a penalty and can end the run (no recovery from a 0.45 m hole), so the margin is larger. |
| Walls / ducts / column | lethal from online sensing (terrain grid), then inflated like any obstacle | D10. Wall contact is a safety DQ, so the soft inflation below applies to them too. |
| Soft inflation | radius 0.8 m, cost_scaling 5.0 | 0.8 ≥ the footprint's circumscribed radius (0.72 m), which Smac needs for exact footprint checks (the first benchmark showed paths crossing walls with 0.7). Cost decays to ~35 % of inscribed at 0.2 m beyond the inscribed radius. |
| Footprint | `[0.6, ±0.41] … [−0.45, ±0.41]` about base_link (sim and real params) | The real robot's outline. Planner collision checks use it exactly (Smac Hybrid/Lattice), not a circle. |

**Planner** (see benchmark below): Reeds-Shepp Hybrid-A* with `reverse_penalty 3.0` (rear sensing is limited: reversing costs
4× a forward metre), `change_penalty 0.5` (each direction change costs half a metre), `non_straight_penalty 1.2`
(arcs cost 20 % more than straight: turning scrubs a skid steer, which only reached 61 % of the commanded yaw before the
track fix), `cost_penalty 2.0` (stay in low-cost space, away from obstacles), minimum turning radius 0.5 m.

**Goal choice by cost-to-go** (`mission_executive.py`): every candidate goal is scored by Nav2's own plan for the leg(s) it
leads to: `length + 0.5 m/rad × heading change + 3.0 × reversed length`. Then:
- *Excavation lane*: candidate passes (7 lateral positions × headings 0, ±45°), as close to the obstacle field as the
  excavation zone allows (the whole swept footprint stays at x < 2.45 m), checked clear in the terrain grid and not
  already dug. Ranked by a heuristic, and the best 4 are scored by planned cost **current pose → lane start** plus
  **lane end → next dump approach**. So the loaded robot ends its dig already pointed along a cheap, low-turn route.
- *Dump pose*: rear edge (dump side) toward the berm, parallel to the 2.2 m long axis. Slots at x = 4.68 / 5.38 / 6.08
  (0.7 m apart, the robot's 0.8 m rear edge overlapping slightly) are used least-used-first. That spreads the material along
  the box (piles that spill over the box edges don't count) and puts the first 25 L into the box early. For each slot the
  approach is a straight reverse of 0.6, 0.3 or 0 m (Nav2 BackUp), whichever plans cheapest, because random rocks can block
  the long approach (seed 2 in the benchmark puts boulders on two of them).
- *Exits*: the cost-to-go weights turns and reversing, and the planner itself penalizes direction changes. So 180° turns in
  tight spaces lose to arcs through open space whenever one exists.

## Planner choice and benchmark

*(Filled in below.)*

## MCC link and bandwidth

*(Filled in below.)*

## Batch-run results

*(Filled in below.)*

## Changes that affect the real robot (needs hardware test)

- **Terrain mapper local ground + zones (D11).** Drive into and out of a dug pit and check `/terrain/grid` stays free in the
  excavation zone. Check real craters/boulders still show lethal in the obstacle zone, and nothing does before the tags have fixed the arena frame.
- **Point-LIO subscriptions** (`third_party_packages/point_lio/src/laserMapping.cpp`): sensor-data QoS with lidar depth 20
  and IMU depth 2000 (was 5, which dropped IMU under load), plus a fixed pose covariance in `/lio_odom`. Check that no IMU
  or lidar messages drop and that the odometry still converges on the real Mid-360.
- **Launch manager** sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and starts the MCC link (`foxglove_bridge` + `mcc_gateway`).
  Check every robot node still discovers the others, and that the laptop GUI connects only through the relay.
- **Nav2 `service_timeout` 60 s** (bringup was timing out under load). Check that a real bringup still activates.
- **Mission executive** replaces `navigation_client` for full autonomy. Bounded action waits, stuck detection and recovery,
  `finish_heading` (an explicit spin when the robot is at the goal position with the wrong heading). Check against the real skid steer.
- **MCC relay and GUI panels** (`mcc_relay.py`, `mcc_panel.py`): not yet run over a real link.
- *(rest filled in below)*
