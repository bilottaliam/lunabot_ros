#!/usr/bin/env python3
"""Seeded generator for the 2027 Lunabotics Artemis arena (guidebook 2026-2027, section 11).

Writes <out>/arena.sdf, <out>/ground.stl and <out>/ground_truth.json. The world frame IS the
arena frame: origin at the starting-zone corner (inside the ducts), X along the 6.88 m length,
Y across the 5.0 m width. ground_truth.json is for the test harness only; autonomy must never read it.

    generate_arena.py --seed 7 --out ~/.ros/lunabot_sim/arena_7 [--random-start] [--heading east]
"""
import argparse
import json
import math
import os
import struct
from collections import deque

import numpy as np

# --- Rulebook geometry (section 11, Figure 9) -------------------------------------------------
ARENA_X, ARENA_Y = 6.88, 5.0          # interior, between the ducts
DUCT_D = 0.17                         # PVC duct along the inside walls
START_ZONE = (0.0, 0.0, 2.0, 2.0)     # xmin, ymin, xmax, ymax; no rocks or craters
EXCAVATION_X = 2.5                    # excavation zone: x < 2.5 (overlaps the starting zone)
CONSTRUCTION = (3.88, 0.0, ARENA_X, 1.5)  # ~1.5 m deep along the y=0 wall, Figure 9
BERM_CENTER, BERM_SIZE = (5.38, 0.9), (2.2, 0.9)
# The guidebook gives no coordinates for the column; this is read off Figure 9 (scaled drawing).
COLUMN_CENTER, COLUMN_SIZE = (3.6, 2.65), 0.45
BOULDER_D, CRATER_D = (0.30, 0.40), (0.40, 0.50)

# Robot footprint about base_link, as in nav2_real_bot_params.yaml: the rear (-0.45) includes the fisheye
# mast (mount x = -0.34 m), which sticks out past the chassis.
FOOTPRINT = np.array([[0.6, 0.41], [0.6, -0.41], [-0.45, -0.41], [-0.45, 0.41]])
HEADINGS = {"east": 0.0, "north": math.pi / 2, "west": math.pi, "south": -math.pi / 2}

# AprilTag 36h11 on the starting-zone frame (the 2 allowed sides). size = black-border edge.
TAG_SIZE = 0.25
TAG_Z = 0.45   # centre; the 0.31 m printed tag clears the 0.28 m duct top
TAGS = [  # id, x, y, yaw of the tag's outward normal (into the arena). Spread to the ends of each
    # 2 m side: the yaw error of the fit (and so the error at the berm, 5 m away) scales with 1/baseline.
    # On the wall face behind the 17 cm ducts, where the starting-zone frame (80/20) is.
    (0, 0.25, -0.12, math.pi / 2), (1, 1.75, -0.12, math.pi / 2),
    (2, -0.12, 0.25, 0.0), (3, -0.12, 1.75, 0.0),
]


def in_rect(x, y, r, m=0.0):
    return r[0] - m <= x <= r[2] + m and r[1] - m <= y <= r[3] + m


def zone_of(x, y):
    if in_rect(x, y, CONSTRUCTION):
        return "construction"
    return "excavation" if x < EXCAVATION_X else "obstacle"


def place_obstacles(rng):
    """Boulders and craters per 11.4, non-overlapping, outside the starting/construction zones."""
    obstacles = []

    def try_place(kind, d, xlo, xhi):
        r = d / 2
        for _ in range(500):
            x, y = rng.uniform(xlo + r, xhi - r), rng.uniform(r + 0.1, ARENA_Y - r - 0.1)
            if in_rect(x, y, START_ZONE, r + 0.3) or in_rect(x, y, CONSTRUCTION, r + 0.1):
                continue
            if abs(x - COLUMN_CENTER[0]) < COLUMN_SIZE / 2 + r + 0.15 and \
               abs(y - COLUMN_CENTER[1]) < COLUMN_SIZE / 2 + r + 0.15:
                continue
            if any(math.hypot(x - o["x"], y - o["y"]) < r + o["d"] / 2 + 0.15 for o in obstacles):
                continue
            o = {"kind": kind, "x": round(x, 3), "y": round(y, 3), "d": round(d, 3)}
            if kind == "boulder":
                o["h"] = round(rng.uniform(0.12, 0.35), 3)
            else:
                o["depth"] = round(rng.uniform(0.20, 0.45), 3)
            o["zone"] = zone_of(x, y)
            obstacles.append(o)
            return

    for _ in range(rng.integers(3, 6)):
        try_place("crater", rng.uniform(*CRATER_D), EXCAVATION_X + 0.1, ARENA_X - 0.1)
    for _ in range(rng.integers(3, 6)):
        try_place("boulder", rng.uniform(*BOULDER_D), EXCAVATION_X + 0.1, ARENA_X - 0.1)
    for _ in range(rng.integers(0, 3)):  # 11.4.1: boulders may also appear in the excavation zone
        try_place("boulder", rng.uniform(*BOULDER_D), 0.1, EXCAVATION_X)
    return obstacles


def route_exists(obstacles, clearance=0.55, res=0.05):
    """Free-space BFS (robot half-width 0.4 + 0.15 margin) from the excavation zone to the berm."""
    nx, ny = int(ARENA_X / res), int(ARENA_Y / res)
    xs, ys = (np.arange(nx) + 0.5) * res, (np.arange(ny) + 0.5) * res
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    blocked = (X < clearance) | (X > ARENA_X - clearance) | (Y < clearance) | (Y > ARENA_Y - clearance)
    blocked |= (np.abs(X - COLUMN_CENTER[0]) < COLUMN_SIZE / 2 + clearance) & \
               (np.abs(Y - COLUMN_CENTER[1]) < COLUMN_SIZE / 2 + clearance)
    for o in obstacles:
        blocked |= np.hypot(X - o["x"], Y - o["y"]) < o["d"] / 2 + clearance
    start = (X < EXCAVATION_X) & ~blocked
    goal = (np.abs(X - BERM_CENTER[0]) < BERM_SIZE[0] / 2) & (Y < 1.5) & ~blocked
    seen = start.copy()
    q = deque(zip(*np.nonzero(start)))
    while q:
        i, j = q.popleft()
        if goal[i, j]:
            return True
        for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= a < nx and 0 <= b < ny and not blocked[a, b] and not seen[a, b]:
                seen[a, b] = True
                q.append((a, b))
    return False


def random_start(rng, heading=None):
    """Random pose with the whole footprint inside the starting zone (13.9.3, 13.9.5)."""
    yaw = HEADINGS[heading] if heading else HEADINGS[rng.choice(list(HEADINGS))]
    c, s = math.cos(yaw), math.sin(yaw)
    pts = FOOTPRINT @ np.array([[c, s], [-s, c]])
    lo, hi = -pts.min(axis=0) + 0.15, 2.0 - pts.max(axis=0) - 0.15   # judges won't place it touching a wall
    x, y = rng.uniform(lo[0], hi[0]), rng.uniform(lo[1], hi[1])
    return {"x": round(x, 3), "y": round(y, 3), "yaw": yaw,
            "heading": [k for k, v in HEADINGS.items() if v == yaw][0]}


def ground_heights(obstacles, rng, res=0.025):
    """Regolith height field: gentle raked undulation (+-1.5 cm) with bowl craters cut in."""
    xs = np.linspace(-0.2, ARENA_X + 0.2, int((ARENA_X + 0.4) / res) + 1)
    ys = np.linspace(-0.2, ARENA_Y + 0.2, int((ARENA_Y + 0.4) / res) + 1)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    Z = np.zeros_like(X)
    for _ in range(3):
        k = rng.uniform(1.0, 3.0, 2)
        Z += 0.005 * np.sin(k[0] * X + rng.uniform(0, 6.3)) * np.cos(k[1] * Y + rng.uniform(0, 6.3))
    for o in obstacles:
        if o["kind"] == "crater":
            r2 = ((X - o["x"]) ** 2 + (Y - o["y"]) ** 2) / (o["d"] / 2) ** 2
            Z = np.where(r2 < 1, np.minimum(Z, -o["depth"] * (1 - r2)), Z)
    return xs, ys, Z


def write_stl(path, xs, ys, Z):
    """Binary STL of the height field (two triangles per cell, normals up)."""
    nx, ny = Z.shape
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    V = np.stack([X, Y, Z], axis=-1).astype(np.float32)
    a, b, c, d = V[:-1, :-1], V[1:, :-1], V[1:, 1:], V[:-1, 1:]
    tris = np.concatenate([np.stack([a, b, c], -2).reshape(-1, 3, 3),
                           np.stack([a, c, d], -2).reshape(-1, 3, 3)])
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tris
    with open(path, "wb") as f:
        f.write(b"lunabot arena ground".ljust(80, b"\0") + struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


def box(name, x, y, z, sx, sy, sz, rgba, yaw=0.0, collide=True):
    geo = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    col = f"<collision name='c'>{geo}</collision>" if collide else ""
    return f"""
    <model name='{name}'><static>true</static><pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name='link'>{col}
        <visual name='v'>{geo}<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual>
      </link></model>"""


def ground_model(name, stl):
    """Static regolith model; tools/run_eval.py swaps in edited copies (dug pits, dumped piles)."""
    g = "0.32 0.31 0.29 1"
    return f"""
    <model name='{name}'><static>true</static>
      <link name='link'>
        <collision name='c'><geometry><mesh><uri>{stl}</uri></mesh></geometry>
          <surface><friction><ode><mu>0.4</mu><mu2>0.4</mu2></ode></friction></surface></collision>
        <visual name='v'><geometry><mesh><uri>{stl}</uri></mesh></geometry>
          <material><ambient>{g}</ambient><diffuse>{g}</diffuse></material></visual>
      </link></model>"""


def world_sdf(gt, out, rtf=1.0):
    m = [ground_model("regolith", f"{out}/ground.stl")]
    # Walls, 17 cm ducts, window frames, windows and building columns: the team's Artemis arena meshes,
    # which are this arena (ducts 7.22 x 5.34 m outside = 6.88 x 5.0 m between them), shifted so the
    # interior corner is the origin. Plain box walls made a near-featureless room in which Point-LIO
    # diverged over a 5 m drive; with these meshes it was measured at 1-10 cm (REVIEW.md).
    ox, oy = ARENA_X / 2, ARENA_Y / 2
    for part in ("arena_walls", "arena_pipe", "arena_columns", "arena_window_frames", "arena_windows"):
        mat = ("<material><ambient>0.5 0.5 0.5 0.3</ambient><diffuse>0.7 0.7 0.7 0.3</diffuse></material>"
               if part == "arena_windows" else "")
        m.append(f"""
    <model name='{part}'><static>true</static><pose>{ox} {oy} 1.248 0 0 0</pose>
      <link name='link'>
        <collision name='c'><geometry><mesh><uri>model://artemis_arena/{part}/{part}.stl</uri></mesh></geometry></collision>
        <visual name='v'><geometry><mesh><uri>model://artemis_arena/{part}/{part}.dae</uri></mesh></geometry>{mat}</visual>
      </link></model>""")
    cx, cy = COLUMN_CENTER
    m.append(box("column", cx, cy, 1.0, COLUMN_SIZE, COLUMN_SIZE, 2.4, "0.6 0.2 0.2 1"))
    # Starting-zone frame (80/20 rails) on its two wall sides, and the zone/berm markings (visual only)
    for i, z in enumerate((0.33, 0.62)):   # above the ducts (top at 0.28 m)
        m.append(box(f"frame_s{i}", 1.0, -DUCT_D + 0.02, z, 2.0, 0.04, 0.04, "0.75 0.75 0.78 1"))
        m.append(box(f"frame_w{i}", -DUCT_D + 0.02, 1.0, z, 0.04, 2.0, 0.04, "0.75 0.75 0.78 1"))
    for name, (x0, y0, x1, y1), rgba in [
        ("mark_start", START_ZONE, "0.2 0.8 0.2 1"), ("mark_construction", CONSTRUCTION, "0.9 0.9 0.2 1"),
        ("mark_berm", (BERM_CENTER[0] - 1.1, BERM_CENTER[1] - 0.45, BERM_CENTER[0] + 1.1, BERM_CENTER[1] + 0.45), "0.9 0.1 0.1 1"),
    ]:
        for k, (x, y, sx, sy) in enumerate([((x0 + x1) / 2, y0, x1 - x0, 0.02), ((x0 + x1) / 2, y1, x1 - x0, 0.02),
                                             (x0, (y0 + y1) / 2, 0.02, y1 - y0), (x1, (y0 + y1) / 2, 0.02, y1 - y0)]):
            m.append(box(f"{name}_{k}", x, y, 0.025, sx, sy, 0.002, rgba, collide=False))
    for t_ in gt["tags"]:
        s = TAG_SIZE * 10 / 8  # texture includes the 1-cell white border
        tex = f"model://apriltags/tag_36_11_{t_['id']:05d}/materials/textures/tag_36_11_{t_['id']:05d}.png"
        m.append(f"""
    <model name='tag_{t_['id']}'><static>true</static>
      <pose>{t_['x']} {t_['y']} {t_['z']} {math.pi / 2} 0 {t_['yaw'] + math.pi / 2}</pose>
      <link name='link'><visual name='v'><geometry><box><size>{s} {s} 0.003</size></box></geometry>
        <!-- emissive: the sun is the only light and grazes vertical walls; a print under arena lights is not that dim -->
        <material><diffuse>1 1 1 1</diffuse><emissive>0.6 0.6 0.6 1</emissive>
          <pbr><metal><albedo_map>{tex}</albedo_map><emissive_map>{tex}</emissive_map></metal></pbr></material>
      </visual></link></model>""")
    for i, o in enumerate(gt["obstacles"]):
        if o["kind"] != "boulder":
            continue
        rx = o["d"] / 2
        geo = f"<geometry><ellipsoid><radii>{rx} {rx} {o['h']}</radii></ellipsoid></geometry>"
        m.append(f"""
    <model name='boulder_{i}'><static>true</static><pose>{o['x']} {o['y']} 0 0 0 0</pose>
      <link name='link'><collision name='c'>{geo}</collision>
        <visual name='v'>{geo}<material><ambient>0.45 0.4 0.35 1</ambient><diffuse>0.45 0.4 0.35 1</diffuse></material></visual>
      </link></model>""")
    return f"""<?xml version='1.0'?>
<sdf version='1.9'>
  <world name='default'>
    <!-- generated by lunabot_sim/scripts/generate_arena.py seed={gt['seed']} -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <physics type='ode'><max_step_size>0.001</max_step_size><real_time_factor>{rtf}</real_time_factor></physics>
    <gravity>0 0 -9.81</gravity>
    <scene><ambient>0.4 0.4 0.4 1</ambient><background>0.07 0.07 0.07 1</background><shadows>1</shadows><grid>false</grid></scene>
    <light name='sun' type='directional'><cast_shadows>1</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.5 0.1 -0.9</direction></light>
    {''.join(m)}
  </world>
</sdf>
"""


def generate(seed, out, randomize_start=False, heading=None, rtf=1.0):
    rng = np.random.default_rng(seed)
    for _ in range(100):
        obstacles = place_obstacles(rng)
        n_b = sum(o["kind"] == "boulder" and o["zone"] == "obstacle" for o in obstacles)
        n_c = sum(o["kind"] == "crater" for o in obstacles)
        if n_b >= 3 and n_c >= 3 and route_exists(obstacles):
            break
    else:
        raise RuntimeError(f"seed {seed}: no layout with a traversable route")
    start = random_start(rng, heading) if randomize_start else \
        {"x": 1.0, "y": 1.0, "yaw": HEADINGS[heading or "east"], "heading": heading or "east"}
    gt = {
        "seed": seed, "start": start, "obstacles": obstacles,
        "column": {"x": COLUMN_CENTER[0], "y": COLUMN_CENTER[1], "size": COLUMN_SIZE},
        "tags": [{"id": i, "x": x, "y": y, "z": TAG_Z, "yaw": yaw, "size": TAG_SIZE} for i, x, y, yaw in TAGS],
        "arena": [ARENA_X, ARENA_Y], "construction": CONSTRUCTION,
        "berm": {"x": BERM_CENTER[0], "y": BERM_CENTER[1], "length": BERM_SIZE[0], "width": BERM_SIZE[1]},
    }
    os.makedirs(out, exist_ok=True)
    xs, ys, Z = ground_heights(obstacles, rng)
    write_stl(os.path.join(out, "ground.stl"), xs, ys, Z)
    np.savez(os.path.join(out, "ground.npz"), xs=xs, ys=ys, Z=Z)  # for the harness's dig/dump edits
    with open(os.path.join(out, "arena.sdf"), "w") as f:
        f.write(world_sdf(gt, out, rtf))
    with open(os.path.join(out, "ground_truth.json"), "w") as f:
        json.dump(gt, f, indent=1)
    return gt


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", default=None)
    p.add_argument("--random-start", action="store_true")
    p.add_argument("--heading", choices=list(HEADINGS), default=None)
    p.add_argument("--rtf", type=float, default=1.0,
                   help="real-time factor target; below what the machine sustains, sensors stay in step with physics")
    a = p.parse_args()
    out = os.path.expanduser(a.out or f"~/.ros/lunabot_sim/arena_{a.seed}")
    gt = generate(a.seed, out, a.random_start, a.heading, a.rtf)
    print(json.dumps({"world": os.path.join(out, "arena.sdf"), "start": gt["start"]}))


if __name__ == "__main__":
    main()
