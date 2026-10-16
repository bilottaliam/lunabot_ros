#!/usr/bin/env python3
"""Batch evaluation: N seeds x random starts, full stack headless, one run at a time.

    tools/run_eval.py --seeds 1-5 --tag baseline [--planner Hybrid] [--run-duration 900]
    tools/run_eval.py --monitor            # attach to an already running sim and print live penalties

Each run starts sim_autonomy_launch.py in its own systemd scope (MemoryMax=7G), so stopping the
scope kills everything it started. Ground truth (testing only, never fed to autonomy) comes from
the generator's ground_truth.json and the robot's pose over gz transport.

Per run: segments and cycles, rock contacts / crater entries in the obstacle zone (penalty points),
wall (duct) contacts, berm drops inside/outside the box, localization error vs ground truth, time
per leg, average MCC-link bandwidth (serialized bytes of the curated topics) and an estimated
score from the rulebook formulas (guidebook 16.6, 17.1).
"""
import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.entity_factory_pb2 import EntityFactory
from gz.msgs.entity_pb2 import Entity
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node as GzNode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "lunabot_sim", "scripts"))
import generate_arena as ga  # noqa: E402

WHEELS = [(0.215, 0.25), (0.215, -0.25), (-0.235, 0.25), (-0.235, -0.25)]
DUMP_OFFSET = 0.45
# Topics that cross the MCC link (lunabot_config/params/mcc/foxglove_bridge.yaml allow-list)
MCC_TOPICS = ["/mcc/map", "/mcc/plan", "/mcc/pose", "/mcc/obstacles", "/mission/state", "/arena_localizer/status",
              "/power_monitor", "/tf_static"]


def footprint(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return ga.FOOTPRINT @ np.array([[c, s], [-s, c]]) + [x, y]


def rect_circle_dist(x, y, yaw, ox, oy):
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = ox - x, oy - y
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    fx, fy = ga.FOOTPRINT[:, 0], ga.FOOTPRINT[:, 1]
    return math.hypot(max(fx.min() - lx, 0, lx - fx.max()), max(fy.min() - ly, 0, ly - fy.max()))


class Ground:
    """Pits dug and piles dumped in the sim: edits the generator's height field and swaps the regolith model.
    Testing aid; the regolith itself isn't simulated (no payload mass, no granular flow)."""
    CUT = (-0.2, 0.4, 0.35)          # ASSUMPTION: body-x span the bucket lip sweeps in a 0.6 m pass, cut width (URDF bucket)
    PILE_BACK = 0.6                  # ASSUMPTION: pile centre behind base_link when tipping over the rear
    REPOSE = math.tan(math.radians(35))

    def __init__(self, arena_dir, gz):
        g = np.load(os.path.join(arena_dir, "ground.npz"))
        self.xs, self.ys, self.Z = g["xs"], g["ys"], g["Z"].copy()
        self.X, self.Y = np.meshgrid(self.xs, self.ys, indexing="ij")
        self.dir, self.gz, self.n, self.name = arena_dir, gz, 0, "regolith"

    def dig(self, pose, liters):
        """Rectangular cut under the pass that just ended at `pose`, edges at the angle of repose."""
        x, y, yaw = pose
        x0, x1, w = self.CUT
        c, s = math.cos(yaw), math.sin(yaw)
        lx, ly = c * (self.X - x) + s * (self.Y - y), -s * (self.X - x) + c * (self.Y - y)
        d = liters / 1000 / ((x1 - x0) * w)
        inside = np.minimum.reduce([lx - x0, x1 - lx, w / 2 - np.abs(ly)])  # < 0 outside the cut
        self.Z -= d * np.clip(1 + inside * self.REPOSE / d, 0, 1)
        return self.swap()

    def pile(self, x, y, liters):
        """Cone at the angle of repose; piles add up (ponytail: no slumping, so a tall berm stays steeper than real)."""
        r = (3 * liters / 1000 / (math.pi * self.REPOSE)) ** (1 / 3)
        self.Z += np.clip(self.REPOSE * (r - np.hypot(self.X - x, self.Y - y)), 0, None)
        return self.swap()

    def swap(self):
        """New model first, then remove the old one, so the robot never loses its floor."""
        self.n += 1
        name, stl = f"regolith_{self.n}", os.path.join(self.dir, f"ground_{self.n}.stl")  # new path: gz caches meshes
        ga.write_stl(stl, self.xs, self.ys, self.Z)
        # blocking create: a remove queued in the same step as a non-blocking create was dropped
        ok, rep = self.gz.request("/world/default/create/blocking",
                                  EntityFactory(sdf=f"<sdf version='1.9'>{ga.ground_model(name, stl)}</sdf>"), EntityFactory, Boolean, 10000)
        if not (ok and rep.data):
            print(f"ground swap: create {name} failed", flush=True)
            return False
        # (remove/blocking removed the model but replied false)
        self.gz.request("/world/default/remove", Entity(name=self.name, type=Entity.MODEL), Entity, Boolean, 5000)
        self.name = name
        return True


class Monitor(Node):
    """Live penalty/contact monitor from ground truth, plus the mission's own reports."""

    def __init__(self, gt, robot="v2_bot", liters=None):
        super().__init__("eval_monitor")
        self.gt, self.robot = gt, robot
        self.pose = None
        self.lock = threading.Lock()
        self.touching = set()
        self.contacts = {"rock": 0, "crater": 0, "wall": 0, "rock_outside_obstacle_zone": 0}
        self.contact_log = []
        self.loc_err = []
        self.trace = []   # 1 Hz: elapsed, ground truth x/y/yaw, estimate x/y/yaw (arena frame)
        self.state = {}
        self.dumps = []
        self.seen_events = set()
        self.bytes = {t: 0 for t in MCC_TOPICS}
        self.t_start = time.monotonic()
        self.detections = {}
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, "/world/default/pose/info", self.on_gz)
        self.liters, self.ground = liters, None
        if liters:  # dig pits and dump piles into the sim's ground
            self.ground = Ground(os.path.expanduser(f"~/.ros/lunabot_sim/arena_{gt['seed']}"), self.gz)
        self.create_subscription(String, "/mission/state", self.on_state, 10)
        self.subscribed = set()
        self.create_timer(1.0, self.tick)
        self.create_timer(0.05, self.check_contacts)

    def on_gz(self, msg):
        for p in msg.pose:
            if p.name == self.robot:
                q = p.orientation
                with self.lock:
                    self.pose = (p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))

    def check_contacts(self):
        with self.lock:
            pose = self.pose
        if pose is None:
            return
        x, y, yaw = pose
        now = set()
        for i, o in enumerate(self.gt["obstacles"]):
            if o["kind"] == "boulder":
                if rect_circle_dist(x, y, yaw, o["x"], o["y"]) < o["d"] / 2:
                    now.add(("rock", i))
            else:
                c, s = math.cos(yaw), math.sin(yaw)
                if any(math.hypot(x + c * wx - s * wy - o["x"], y + s * wx + c * wy - o["y"]) < o["d"] / 2 for wx, wy in WHEELS):
                    now.add(("crater", i))
        fp = footprint(x, y, yaw)
        if fp[:, 0].min() < 0 or fp[:, 1].min() < 0 or fp[:, 0].max() > ga.ARENA_X or fp[:, 1].max() > ga.ARENA_Y:
            now.add(("wall", -1))
        for k, i in now - self.touching:  # rising edges = new contacts
            zone = self.gt["obstacles"][i]["zone"] if i >= 0 else "-"
            key = k if (k == "wall" or zone == "obstacle") else "rock_outside_obstacle_zone"
            self.contacts[key] += 1
            self.contact_log.append({"t": round(time.monotonic() - self.t_start, 1), "kind": k, "id": i, "zone": zone,
                                     "pose": [round(v, 3) for v in pose]})
        self.touching = now

    def on_state(self, msg):
        self.state = json.loads(msg.data)
        with self.lock:
            pose = self.pose
        for e in self.state.get("events", []):
            key = (e["t"], e["msg"])
            if key in self.seen_events:
                continue
            self.seen_events.add(key)
            if e["msg"] == "excavation complete" and pose and self.ground:
                self.ground.dig(pose, self.liters)
            if e["msg"].startswith("dump complete") and pose:
                px, py = pose[0] - DUMP_OFFSET * math.cos(pose[2]), pose[1] - DUMP_OFFSET * math.sin(pose[2])
                b = self.gt["berm"]
                inside = abs(px - b["x"]) < b["length"] / 2 and abs(py - b["y"]) < b["width"] / 2
                self.dumps.append({"t": e["t"], "pile": [round(px, 3), round(py, 3)], "inside": inside})
                if self.ground:
                    back = Ground.PILE_BACK
                    self.ground.pile(pose[0] - back * math.cos(pose[2]), pose[1] - back * math.sin(pose[2]), self.liters)

    def tick(self):
        # Subscribe raw to MCC topics as they appear, to count the bytes the link would carry
        for name, types in self.get_topic_names_and_types():
            if name in MCC_TOPICS and name not in self.subscribed:
                self.subscribed.add(name)
                self.create_subscription(get_message(types[0]), name,
                                         lambda m, n=name: self.bytes.__setitem__(n, self.bytes[n] + len(m)),
                                         qos_profile_sensor_data if name != "/tf_static" else 10, raw=True)
        with self.lock:
            pose = self.pose
        try:
            t = self.tf.lookup_transform("arena", "base_link", Time())
            if pose:
                tr = t.transform.translation
                yaw = 2 * math.atan2(t.transform.rotation.z, t.transform.rotation.w)
                self.loc_err.append((math.hypot(tr.x - pose[0], tr.y - pose[1]), abs(math.remainder(yaw - pose[2], math.tau))))
                self.trace.append([round(time.monotonic() - self.t_start, 1), *[round(v, 3) for v in pose],
                                   round(tr.x, 3), round(tr.y, 3), round(yaw, 3)])
        except TransformException:
            pass

    def report(self, liters_per_dump):
        s = self.state
        seg = s.get("segments", {"excavation": [0, 0], "travel": [0, 0], "dump": [0, 0]})
        elapsed = max(time.monotonic() - self.t_start, 1.0)
        mbps = sum(self.bytes.values()) * 8 / elapsed / 1e6
        inside = sum(d["inside"] for d in self.dumps)
        volume = inside * liters_per_dump
        mult = min(1.0, volume / 25.0)
        pct = {k: (v[0] / v[1] if v[1] else 0.0) for k, v in seg.items()}
        cycles = s.get("cycles", 0)
        if cycles >= 2 and all(v[0] == v[1] for v in seg.values()) and volume > 0:
            autonomy = 1400 * mult   # hands-free from the start in sim
        elif all(p == 1.0 for p in pct.values()):
            autonomy = 1000 * mult
        else:
            autonomy = (200 * pct["excavation"] + 100 * pct["dump"] + 500 * pct["travel"]) * mult
        n = self.contacts["rock"] + self.contacts["crater"]
        penalty = 0 if n == 0 else min(100, 50 + 25 * (n - 1))
        bandwidth = 0 if mbps > 4 else 120 - 30 * mbps
        err = np.array(self.loc_err) if self.loc_err else np.zeros((1, 2))
        return {
            "cycles": cycles, "segments": seg, "phase": s.get("phase"), "leg_times": s.get("leg_times", []),
            "contacts": self.contacts, "contact_log": self.contact_log, "penalty": -penalty,
            "dumps": self.dumps, "dumps_inside": inside, "berm_liters_est": volume,
            "loc_err_mean_m": round(float(err[:, 0].mean()), 3), "loc_err_max_m": round(float(err[:, 0].max()), 3),
            "loc_yaw_err_max_deg": round(math.degrees(float(err[:, 1].max())), 1),
            "mcc_mbps": round(mbps, 4), "mcc_bytes": self.bytes, "trace": self.trace,
            "score": {"autonomy": round(autonomy, 1), "bandwidth": round(bandwidth, 1), "penalty": -penalty,
                      "total": round(autonomy + bandwidth - penalty, 1)},
        }


def free_gb():
    with open("/proc/meminfo") as f:
        return int(next(l for l in f if l.startswith("MemAvailable")).split()[1]) / 1e6


def run_one(seed, a, outdir):
    while free_gb() < 6.0:
        print(f"waiting for memory ({free_gb():.1f} GB available)", flush=True)
        time.sleep(10)
    unit = f"lunabot-eval-{seed}"
    log = open(os.path.join(outdir, f"seed_{seed}.log"), "w")
    cmd = ["systemd-run", "--user", "--scope", "--collect", "-p", "MemoryMax=7G", "-p", "TimeoutStopSec=20", f"--unit={unit}",
           "ros2", "launch", "lunabot_bringup", "sim_autonomy_launch.py", f"seed:={seed}", "random_start:=true",
           f"max_cycles:={a.max_cycles}", f"run_duration:={a.run_duration}",
           f"test_odom:={'true' if a.test_odom else 'false'}"] + \
          ([f"planner_id:={a.planner}"] if a.planner else []) + \
          ([f"controller_id:={a.controller}"] if a.controller else [])
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(3)
    gt = json.load(open(os.path.expanduser(f"~/.ros/lunabot_sim/arena_{seed}/ground_truth.json")))
    mon = Monitor(gt, liters=None if a.no_ground_edits else a.liters_per_dump)
    t_end = time.monotonic() + a.wall_timeout
    try:
        while time.monotonic() < t_end:
            rclpy.spin_once(mon, timeout_sec=0.1)
            if mon.state.get("phase") in ("DONE", "INHIBITED") and mon.state.get("inhibited"):
                break
            if proc.poll() is not None:
                break
    finally:
        rep = mon.report(a.liters_per_dump)
        rep.update(odometry="TEST-ONLY physics odometry" if a.test_odom else "Point-LIO + wheels + gyro (EKF)",
                   seed=seed, start=gt["start"], wall_s=round(time.monotonic() - mon.t_start), timed_out=time.monotonic() >= t_end)
        mon.destroy_node()
        subprocess.run(["systemctl", "--user", "stop", f"{unit}.scope"], check=False)
        proc.wait(timeout=60)
    with open(os.path.join(outdir, f"seed_{seed}.json"), "w") as f:
        json.dump(rep, f, indent=1)
    return rep


def table(reps):
    rows = ["| seed | start | cycles | exc / travel / dump (auto/total) | contacts rock/crater/wall | penalty | drops in/total | "
            "loc err mean/max (m) | MCC Mbps | est. score |", "|---|---|---|---|---|---|---|---|---|---|"]
    for r in reps:
        s = r["segments"]
        rows.append(f"| {r['seed']} | {r['start']['heading']} ({r['start']['x']:.1f},{r['start']['y']:.1f}) | {r['cycles']} | "
                    f"{s['excavation'][0]}/{s['excavation'][1]} · {s['travel'][0]}/{s['travel'][1]} · {s['dump'][0]}/{s['dump'][1]} | "
                    f"{r['contacts']['rock']}/{r['contacts']['crater']}/{r['contacts']['wall']} | {r['penalty']} | "
                    f"{r['dumps_inside']}/{len(r['dumps'])} | {r['loc_err_mean_m']:.2f}/{r['loc_err_max_m']:.2f} | "
                    f"{r['mcc_mbps']:.3f} | {r['score']['total']} |")
    n = len(reps)
    if n:
        rows.append(f"\nSuccess (>=2 cycles): {sum(r['cycles'] >= 2 for r in reps)}/{n}, mean cycles {np.mean([r['cycles'] for r in reps]):.2f}, "
                    f"mean penalty {np.mean([r['penalty'] for r in reps]):.0f}, wall contacts {sum(r['contacts']['wall'] for r in reps)}, "
                    f"mean MCC {np.mean([r['mcc_mbps'] for r in reps]):.3f} Mbps, mean est. score {np.mean([r['score']['total'] for r in reps]):.0f}")
    return "\n".join(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="1-3")
    ap.add_argument("--tag", default="eval")
    ap.add_argument("--planner", default="")
    ap.add_argument("--controller", default="")
    ap.add_argument("--max-cycles", type=int, default=0)
    ap.add_argument("--run-duration", type=float, default=900.0, help="mission run clock (sim seconds)")
    ap.add_argument("--wall-timeout", type=float, default=3600.0)
    ap.add_argument("--liters-per-dump", type=float, default=8.0, help="ASSUMPTION: regolith per bucket load")
    ap.add_argument("--test-odom", action="store_true",
                    help="TEST ONLY: physics-engine odometry instead of Point-LIO (results are labelled)")
    ap.add_argument("--no-ground-edits", action="store_true", help="don't dig pits / dump piles into the sim ground")
    ap.add_argument("--monitor", action="store_true")
    ap.add_argument("--seed", type=int, default=0, help="--monitor: seed of the running sim")
    a = ap.parse_args()
    rclpy.init()
    if a.monitor:
        gt = json.load(open(os.path.expanduser(f"~/.ros/lunabot_sim/arena_{a.seed}/ground_truth.json")))
        mon = Monitor(gt)
        last = None
        try:
            while True:
                rclpy.spin_once(mon, timeout_sec=0.5)
                cur = (dict(mon.contacts), mon.state.get("phase"), mon.state.get("cycles"))
                if cur != last:
                    print(json.dumps({"contacts": cur[0], "phase": cur[1], "cycles": cur[2]}), flush=True)
                    last = cur
        except KeyboardInterrupt:
            print(json.dumps(mon.report(a.liters_per_dump), indent=1))
        return
    lo, _, hi = a.seeds.partition("-")
    outdir = os.path.join(HERE, "results", a.tag)
    os.makedirs(outdir, exist_ok=True)
    reps = []
    for seed in range(int(lo), int(hi or lo) + 1):
        print(f"=== seed {seed} ===", flush=True)
        reps.append(run_one(seed, a, outdir))
        print(table(reps[-1:]).splitlines()[-1], flush=True)
    out = (f"Odometry: {reps[0]['odometry']}\n\n" if reps else "") + table(reps)
    with open(os.path.join(outdir, "summary.md"), "w") as f:
        f.write(out + "\n")
    print(out)


if __name__ == "__main__":
    main()
