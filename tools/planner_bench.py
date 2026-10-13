#!/usr/bin/env python3
"""Planning-only benchmark of the Nav2 planner candidates on generated 2027 arenas.

Runs planner_server (same params as the sim, nav2_v1_bot_params.yaml) against grids rasterized
from the generator's ground truth exactly as terrain_mapper would mark them (rocks/craters +
hard margins on /terrain/grid, walls + column on /map), then plans the real mission legs:
start -> excavation lane, lane end -> each dump slot, dump slot -> excavation lanes.

    tools/planner_bench.py --seeds 1-20 [--planners GridBased,Hybrid,HybridRS,Lattice] [--out results.json]

Metrics per leg: success, planning time, length, total heading change, in-place rotation,
sharp turns (>60 deg within 0.5 m), reversing distance, min clearance (footprint edge) to
rocks / craters / walls.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "lunabot_sim", "scripts"))
sys.path.insert(0, os.path.join(REPO, "lunabot_nav", "scripts"))
import generate_arena as ga  # noqa: E402
from mission_executive import path_metrics, wrap  # noqa: E402

PARAMS = os.path.join(REPO, "lunabot_config", "params", "nav2", "nav2_v1_bot_params.yaml")
FP = ga.FOOTPRINT
RES, ORIGIN, N = 0.05, -1.0, 180   # grid covering x,y in [-1, 8]


def grids(gt, rock_margin=0.15, crater_margin=0.20):
    c = (np.arange(N) + 0.5) * RES + ORIGIN
    X, Y = np.meshgrid(c, c, indexing="ij")
    terrain = np.zeros((N, N), np.int8)
    for o in gt["obstacles"]:
        m = rock_margin if o["kind"] == "boulder" else crater_margin
        terrain[np.hypot(X - o["x"], Y - o["y"]) < o["d"] / 2 + m] = 100
    walls = np.zeros((N, N), np.int8)
    walls[(X < 0) | (X > ga.ARENA_X) | (Y < 0) | (Y > ga.ARENA_Y)] = 100   # ducts, as lidar sees them
    col = gt["column"]
    walls[(abs(X - col["x"]) < col["size"] / 2) & (abs(Y - col["y"]) < col["size"] / 2)] = 100
    return terrain, walls


def grid_msg(data, node):
    m = OccupancyGrid()
    m.header.frame_id = "map"
    m.header.stamp = node.get_clock().now().to_msg()
    m.info.resolution, m.info.width, m.info.height = RES, N, N
    m.info.origin.position.x = m.info.origin.position.y = ORIGIN
    m.info.origin.orientation.w = 1.0
    m.data = data.T.ravel().tolist()
    return m


def clearance(path, gt):
    """Min distance from the footprint edge to rocks, craters and the arena boundary along a path."""
    best = {"rock": math.inf, "crater": math.inf, "wall": math.inf}
    for q in path.poses[::2]:
        x, y = q.pose.position.x, q.pose.position.y
        yaw = 2 * math.atan2(q.pose.orientation.z, q.pose.orientation.w)
        c, s = math.cos(yaw), math.sin(yaw)
        for o in gt["obstacles"]:
            # obstacle centre in the robot frame -> distance to the footprint rectangle
            dx, dy = o["x"] - x, o["y"] - y
            lx, ly = c * dx + s * dy, -s * dx + c * dy
            ex = max(FP[:, 0].min() - lx, 0, lx - FP[:, 0].max())
            ey = max(-0.4 - ly, 0, ly - 0.4)
            d = math.hypot(ex, ey) - o["d"] / 2
            k = "rock" if o["kind"] == "boulder" else "crater"
            best[k] = min(best[k], d)
        pts = FP @ np.array([[c, s], [-s, c]]) + [x, y]
        best["wall"] = min(best["wall"], pts[:, 0].min(), pts[:, 1].min(),
                           ga.ARENA_X - pts[:, 0].max(), ga.ARENA_Y - pts[:, 1].max())
    return best


def turn_stats(path):
    p = np.array([[q.pose.position.x, q.pose.position.y, 2 * math.atan2(q.pose.orientation.z, q.pose.orientation.w)]
                  for q in path.poses])
    if len(p) < 2:
        return 0.0, 0
    step = np.hypot(*np.diff(p[:, :2], axis=0).T)
    dyaw = np.array([wrap(a) for a in np.diff(p[:, 2])])
    in_place = float(np.abs(dyaw[step < 0.01]).sum())
    sharp, i = 0, 0
    s_cum = np.concatenate([[0], np.cumsum(step)])
    yaw_cum = np.concatenate([[0], np.cumsum(dyaw)])
    while i < len(p):
        j = np.searchsorted(s_cum, s_cum[i] + 0.5)
        j = min(j, len(p) - 1)
        if abs(yaw_cum[j] - yaw_cum[i]) > math.radians(60):
            sharp += 1
            i = j + 1
        else:
            i += 1
    return in_place, sharp


def legs(gt):
    """The executive's legs, in arena coordinates (sim: arena frame == map frame).
    A 'loaded' leg is a list of alternative approach poses for one dump slot (reverse 0.6/0.3/0 m);
    the cheapest one that plans is scored, as the executive does."""
    s = gt["start"]
    start = (s["x"], s["y"], s["yaw"])
    L = 0.6
    lanes = [(1.25, y, 0.0) for y in (1.3, 1.9, 2.5, 3.1, 3.7)]
    lanes = [ln for ln in lanes if all(math.hypot(ln[0] + d - o["x"], ln[1] - o["y"]) > o["d"] / 2 + 0.6
                                       for o in gt["obstacles"] for d in (0, L))][:3]
    out = [("start->lane", start, [lanes[0]])]
    for ln in lanes:
        end = (ln[0] + L, ln[1], ln[2])
        out += [("loaded", end, [(x, 0.9 + 0.45 + r, math.pi / 2) for r in (0.6, 0.3, 0.0)]) for x in (4.68, 5.38, 6.08)]
    out += [("empty", (x, 1.35, math.pi / 2), [ln]) for x, ln in zip((4.68, 5.38, 6.08), lanes)]
    return out


def stamped(nav, x, y, yaw):
    from geometry_msgs.msg import PoseStamped
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="1-10")
    ap.add_argument("--planners", default="GridBased,Hybrid,HybridRS,Lattice")
    ap.add_argument("--out", default=os.path.join(HERE, "results", "planner_bench.json"))
    a = ap.parse_args()
    lo, _, hi = a.seeds.partition("-")
    seeds = range(int(lo), int(hi or lo) + 1)
    planners = a.planners.split(",")

    env = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    procs = [subprocess.Popen(c, env=env, start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) for c in (
        ["ros2", "run", "nav2_planner", "planner_server", "--ros-args", "--params-file", PARAMS, "-p", "use_sim_time:=false",
         "-r", "__ns:=/", "-p", "global_costmap.global_costmap.use_sim_time:=false"],
        ["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager", "--ros-args", "-p", "node_names:=[planner_server]",
         "-p", "autostart:=true", "-p", "use_sim_time:=false"],
        ["ros2", "run", "tf2_ros", "static_transform_publisher", "--frame-id", "map", "--child-frame-id", "base_link"],
    )]
    rclpy.init()
    nav = BasicNavigator("planner_bench")
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub_t = nav.create_publisher(OccupancyGrid, "/terrain/grid", latched)
    pub_m = nav.create_publisher(OccupancyGrid, "/map", latched)
    results = []
    try:
        nav._waitForNodeToActivate("planner_server")
        for seed in seeds:
            gt = ga.generate(seed, os.path.expanduser(f"~/.ros/lunabot_sim/bench_{seed}"), randomize_start=True)
            terrain, walls = grids(gt)
            pub_t.publish(grid_msg(terrain, nav))
            pub_m.publish(grid_msg(walls, nav))
            end = time.monotonic() + 3.0   # let both static layers take the new maps
            while time.monotonic() < end:
                rclpy.spin_once(nav, timeout_sec=0.1)
            for k, (leg, s, goals) in enumerate(legs(gt)):
                for pl in planners:
                    best, dt = None, 0.0
                    for g in goals:
                        t = time.monotonic()
                        path = nav.getPath(stamped(nav, *s), stamped(nav, *g), pl, use_start=True)
                        dt += time.monotonic() - t
                        if path and path.poses:
                            c = path_metrics(path)
                            c = c[0] + 0.5 * c[1] + 3.0 * c[2]   # executive's cost-to-go weights
                            if best is None or c < best[0]:
                                best = (c, path)
                    path = best and best[1]
                    r = {"seed": seed, "leg": leg, "k": k, "planner": pl, "ok": bool(path), "plan_s": round(dt, 3)}
                    if r["ok"]:
                        length, turn, rev = path_metrics(path)
                        in_place, sharp = turn_stats(path)
                        r.update(length=round(length, 3), turn=round(turn, 3), in_place=round(in_place, 3),
                                 sharp=sharp, reverse=round(rev, 3),
                                 **{f"clear_{k}": round(v, 3) for k, v in clearance(path, gt).items()})
                    results.append(r)
            print(f"seed {seed}: {sum(r['ok'] for r in results if r['seed'] == seed)}/{len(legs(gt)) * len(planners)} plans", flush=True)
    finally:
        for p in procs:
            os.killpg(p.pid, signal.SIGINT)
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        with open(a.out, "w") as f:
            json.dump(results, f, indent=0)
    summarize(results, planners)


def feasible(r):
    """A plan only counts if the real footprint stays clear of rocks, craters and walls along it."""
    return r["ok"] and min(r["clear_rock"], r["clear_crater"], r["clear_wall"]) >= -0.02


def summarize(results, planners):
    solvable = {(r["seed"], r["k"]) for r in results if feasible(r)}
    print(f"\nSuccess = a collision-free path (footprint vs ground truth) on legs that some planner solved collision-free ({len(solvable)} legs);"
          " 'returned' counts any path the planner returned.")
    print(f"| planner | success | returned | plan time mean/max (s) | length (m) | heading change (rad) | in-place rot (rad) | sharp turns | reverse (m) | min clearance rock / crater / wall (m) |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for pl in planners:
        rs = [r for r in results if r["planner"] == pl and (r["seed"], r["k"]) in solvable]
        ok = [r for r in rs if feasible(r)]
        returned = sum(r["ok"] for r in rs)
        m = lambda k: np.mean([r[k] for r in ok]) if ok else float("nan")
        mn = lambda k: min(r[k] for r in ok) if ok else float("nan")
        print(f"| {pl} | {len(ok)}/{len(rs)} | {returned} | {m('plan_s'):.2f} / {max(r['plan_s'] for r in rs):.2f} | {m('length'):.2f} | "
              f"{m('turn'):.2f} | {m('in_place'):.2f} | {m('sharp'):.2f} | {m('reverse'):.2f} | "
              f"{mn('clear_rock'):.2f} / {mn('clear_crater'):.2f} / {mn('clear_wall'):.2f} |")


if __name__ == "__main__":
    main()
