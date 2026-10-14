#!/usr/bin/env python3
"""Lunabotics mission executive: excavate -> travel loaded -> dump -> travel empty, until time runs out.

Replaces navigation_client. Every motion goes through Nav2 (NavigateToPose, BackUp, Spin) or the
existing excavation/depositing action servers. All goals are in the `arena` frame published by
arena_localizer; the berm and zones are the rulebook's coordinates (guidebook 11.2, Figure 9).

State is published on /mission/state (JSON, 1 Hz) for the MCC: phase, segment counters
(automated/total), run clock, the current goal and a timestamped log of autonomy attempts.
/mission/inhibit (std_msgs/Empty) or the end of the run timer stops everything (15.4.1).
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lunabot_msgs.action import Depositing, Excavation
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener

# Rulebook geometry (arena frame: origin at the starting-zone corner, X along the 6.88 m length)
ARENA = (6.88, 5.0)
EXCAVATION_X = 2.5
BERM = (5.38, 0.9, 2.2, 0.9)   # centre x, y, length (x), width (y)
FRONT, REAR, HALF_W = 0.6, 0.45, 0.41   # footprint about base_link (nav2 params; rear = fisheye mast)


def yaw_q(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


def wrap(a):
    return math.remainder(a, math.tau)


def path_metrics(path):
    """Length, total heading change and reversing length of a nav_msgs/Path."""
    p = np.array([[q.pose.position.x, q.pose.position.y, 2 * math.atan2(q.pose.orientation.z, q.pose.orientation.w)]
                  for q in path.poses])
    if len(p) < 2:
        return 0.0, 0.0, 0.0
    d = np.diff(p[:, :2], axis=0)
    seg = np.hypot(d[:, 0], d[:, 1])
    heading = np.cos(p[:-1, 2]) * d[:, 0] + np.sin(p[:-1, 2]) * d[:, 1]
    turn = np.abs(np.vectorize(wrap)(np.diff(p[:, 2]))).sum()
    return float(seg.sum()), float(turn), float(seg[heading < -1e-3].sum())


class MissionExecutive(BasicNavigator):
    def __init__(self):
        super().__init__("mission_executive")
        params = {
            "run_duration": 900.0,         # 15 min run (12)
            "elapsed_at_start": 0.0,       # run-clock seconds already used (setup/RC localization)
            "start_deadline": 300.0,       # must initiate a mission cycle within 5 min (15.3.1)
            "max_cycles": 0,               # 0 = until time runs out
            "planner_id": "HybridRS",      # see AUTONOMY.md, planner benchmark
            "excavation_distance": 0.6,    # straight pass the excavation server drives (sim: 3 s @ 0.2 m/s)
            "excavate_time": 10.0, "dump_time": 15.0, "travel_loaded_time": 90.0,  # leg-time priors; replaced by measured means
            "dump_offset": 0.45,           # pile centre behind base_link when the bucket tips (rear dump)
            "dump_reverse_options": [0.6, 0.3, 0.0],  # final straight reverse into the dump pose (Nav2 BackUp);
                                           # shorter when rocks block the longer approach
            "dump_server_reverse": 0.0,    # distance the depositing server itself reverses (real: measure)
            "dump_slots": [4.68, 5.38, 6.08],  # x of each dump along the 2.2 m berm box
            "lane_ys": [0.7, 1.3, 1.9, 2.5, 3.1, 3.7, 4.3],
            "lane_headings": [0.0, 0.785, -0.785],
            "turn_weight": 0.5,            # metres of driving one radian of turning is worth
            "reverse_weight": 3.0,         # extra cost per metre reversed (limited rear sensing)
            "candidates_to_plan": 4,
            "stuck_time": 6.0, "localize_timeout": 40.0, "nav_retries": 3,
            "log_dir": os.path.expanduser("~/.ros/lunabot_mission"),
        }
        for k, v in params.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in params}

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.excavate_client = ActionClient(self, Excavation, "auto_excavation_action")
        self.dump_client = ActionClient(self, Depositing, "auto_depositing_action")
        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.planner_pub = self.create_publisher(String, "planner_selector", latched)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.relocalize_pub = self.create_publisher(String, "/arena_localizer/request", 10)
        self.create_subscription(Empty, "/mission/inhibit", lambda _: self.inhibit("MCC inhibit command"), 10)
        self.create_subscription(String, "/arena_localizer/status", self.on_loc, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(OccupancyGrid, "/terrain/grid", lambda m: setattr(self, "grid", m), latched)
        self.create_timer(1.0, self.publish_state)

        self.loc = {}
        self.grid = None
        self.last_cmd = (0.0, -1e9)   # (linear x, ROS-clock seconds) of the last non-zero /cmd_vel
        self.inhibited = False
        self.phase = "WAIT_NAV2"
        self.goal_desc = ""
        self.events = []
        self.counts = {s: [0, 0] for s in ("excavation", "travel", "dump")}  # [automated, total]
        self.cycles = 0
        self.dumps = {x: 0 for x in self.p["dump_slots"]}
        self.dug = []
        self.leg_times = []
        self.t0 = None
        self.action_handle = None
        os.makedirs(self.p["log_dir"], exist_ok=True)
        self.log_file = open(os.path.join(self.p["log_dir"], time.strftime("mission_%Y%m%d_%H%M%S.jsonl")), "a")

    # ---------------------------------------------------------------- bookkeeping
    def elapsed(self):
        return self.p["elapsed_at_start"] + (0.0 if self.t0 is None else (self.get_clock().now() - self.t0).nanoseconds * 1e-9)

    def remaining(self):
        return self.p["run_duration"] - self.elapsed()

    def event(self, text, **extra):
        e = {"t": round(self.elapsed(), 1), "wall": time.strftime("%H:%M:%S"), "phase": self.phase, "msg": text, **extra}
        self.events = (self.events + [e])[-30:]
        self.log_file.write(json.dumps(e) + "\n")
        self.log_file.flush()
        self.info(f"[{e['t']:6.1f}s] {text}")

    def segment(self, name, ok=True):
        self.counts[name][1] += 1
        self.counts[name][0] += int(ok)

    def publish_state(self):
        pose = self.robot_pose()
        self.state_pub.publish(String(data=json.dumps({
            "phase": self.phase, "inhibited": self.inhibited, "elapsed": round(self.elapsed(), 1),
            "remaining": round(self.remaining(), 1), "cycles": self.cycles, "segments": self.counts,
            "dumps": {f"{k:.2f}": v for k, v in self.dumps.items()}, "goal": self.goal_desc,
            "pose": None if pose is None else [round(v, 3) for v in pose],
            "localized": self.loc.get("localized", False), "events": self.events[-8:],
            "leg_times": self.leg_times[-12:]})))

    def on_loc(self, msg):
        self.loc = json.loads(msg.data)

    def on_cmd(self, msg):
        if abs(msg.linear.x) > 0.03 or abs(msg.angular.z) > 0.15:
            self.last_cmd = (msg.linear.x, self.get_clock().now().nanoseconds * 1e-9)

    def inhibit(self, why):
        if self.inhibited:
            return
        self.inhibited = True
        self.cancelTask()
        if self.action_handle:
            self.action_handle.cancel_goal_async()
        for _ in range(5):
            self.cmd_pub.publish(Twist())
        self.phase = "INHIBITED"
        self.event(f"INHIBIT: {why}")

    def check_clock(self):
        if not self.inhibited and self.remaining() <= 0:
            self.inhibit("run timer expired")
        return not self.inhibited

    # ---------------------------------------------------------------- geometry
    def robot_pose(self, frame="arena"):
        try:
            t = self.tf.lookup_transform(frame, "base_link", Time())
        except TransformException:
            return None
        q = t.transform.rotation
        return t.transform.translation.x, t.transform.translation.y, 2 * math.atan2(q.z, q.w)

    def pose_now(self, timeout=10.0):
        """Robot pose in the arena frame, waiting briefly for TF (the arena frame can be seconds old)."""
        end = time.monotonic() + timeout
        while (pose := self.robot_pose()) is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return pose

    def stamped(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = "arena"
        # stamp 0 = "latest TF": goals are fixed in the arena, and a 'now' stamp made Nav2 wait for a
        # map->odom (RTAB-Map) that lags by >1 s, which stalled planning for as long as it lagged.
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = yaw_q(yaw)
        return p

    def send_goal(self, client, goal, timeout=10.0):
        """send_goal_async with a bounded wait: BasicNavigator waits forever for the goal response, and
        under load one that never arrives hung the whole mission (clock and inhibit checks included)."""
        f = client.send_goal_async(goal, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
        if not f.done() or f.result() is None or not f.result().accepted:
            return None
        return f.result()

    def go_to(self, pose):
        """NavigateToPose that keeps BasicNavigator's state, so isTaskComplete()/cancelTask() work."""
        self.clearPreviousState()
        handle = None
        if self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            handle = self.send_goal(self.nav_to_pose_client, NavigateToPose.Goal(pose=pose))
        if handle is None:   # see plan(): recreate the client for fresh discovery
            self.nav_to_pose_client.destroy()
            self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
            return False
        self.goal_handle, self.result_future = handle, handle.get_result_async()
        return True

    def plan(self, start, goal, timeout=15.0):
        """ComputePathToPose with timeouts; None if it fails or takes too long. Smac caps its search at
        max_planning_time (2 s), but the first request also builds lookup tables, and a cancelled request
        keeps the planner busy, so a short timeout here made every following request time out too."""
        g = ComputePathToPose.Goal(goal=goal, planner_id=self.p["planner_id"], use_start=start is not None)
        if start is not None:
            g.start = start
        handle = None
        if self.compute_path_to_pose_client.wait_for_server(timeout_sec=5.0):
            handle = self.send_goal(self.compute_path_to_pose_client, g)
        if handle is None:
            # Seen in the sim: this process's client never matched the planner's action server (a fresh
            # client in another process planned fine). A new client entity gets fresh discovery.
            self.compute_path_to_pose_client.destroy()
            self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
            self.event("planner unreachable; recreated the ComputePathToPose client")
            return None
        r = handle.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=timeout)
        if not r.done():
            handle.cancel_goal_async()
            return None
        res = r.result()
        return res.result.path if res.status == GoalStatus.STATUS_SUCCEEDED else None

    def plan_cost(self, start, goal):
        """Cost-to-go of a leg from Nav2's own planner (same costmap and planner the robot will drive)."""
        t = time.monotonic()
        path = self.plan(self.stamped(*start) if start else None, self.stamped(*goal))
        dt = time.monotonic() - t
        if path is None or not path.poses:
            return math.inf, dt
        length, turn, rev = path_metrics(path)
        return length + self.p["turn_weight"] * turn + self.p["reverse_weight"] * rev, dt

    def lane_clear(self, x0, y0, yaw, length):
        """Swept footprint of a straight excavation pass is inside the excavation zone and not lethal."""
        c, s = math.cos(yaw), math.sin(yaw)
        xs, ys = [], []
        for d in np.linspace(0, length, 6):
            for fx, fy in ((FRONT, HALF_W), (FRONT, -HALF_W), (-REAR, HALF_W), (-REAR, -HALF_W)):
                xs.append(x0 + c * (d + fx) - s * fy)
                ys.append(y0 + s * (d + fx) + c * fy)
        if min(xs) < 0.3 or max(xs) > EXCAVATION_X - 0.05 or min(ys) < 0.3 or max(ys) > ARENA[1] - 0.3:
            return False
        if self.grid is None:
            return True
        try:
            t = self.tf.lookup_transform(self.grid.header.frame_id, "arena", Time())
        except TransformException:
            return True
        q = t.transform.rotation
        a = 2 * math.atan2(q.z, q.w)
        info = self.grid.info
        data = np.array(self.grid.data, dtype=np.int8).reshape(info.height, info.width)
        for d in np.arange(-REAR, length + FRONT, info.resolution):
            for w in np.arange(-HALF_W, HALF_W + 1e-6, info.resolution):
                ax, ay = x0 + c * d - s * w, y0 + s * d + c * w
                gx = t.transform.translation.x + math.cos(a) * ax - math.sin(a) * ay
                gy = t.transform.translation.y + math.sin(a) * ax + math.cos(a) * ay
                i = int((gx - info.origin.position.x) / info.resolution)
                j = int((gy - info.origin.position.y) / info.resolution)
                if 0 <= i < info.width and 0 <= j < info.height and data[j, i] >= 100:
                    return False
        return True

    def predump_pose(self, slot_x, reverse):
        # Rear (dump side) toward the berm, its 0.8 m edge along the box's long axis
        return slot_x, BERM[1] + self.p["dump_offset"] + reverse + self.p["dump_server_reverse"], math.pi / 2

    def choose_dump(self, start):
        """(cost, slot, reverse): least-used slots first (spread along the box), then cheapest approach."""
        for n in sorted(set(self.dumps.values())):
            costs = [(self.plan_cost(start, self.predump_pose(x, r))[0], x, r)
                     for x in self.dumps if self.dumps[x] == n for r in self.p["dump_reverse_options"]]
            best = min(costs)
            if best[0] < math.inf:
                return best
        return None

    def choose_lane(self, start):
        """Excavation pass whose start is cheap to reach and whose end points at a cheap route to the berm."""
        if start is None:
            return None
        L = self.p["excavation_distance"]
        slot = min(self.dumps, key=lambda x: (self.dumps[x], x))
        target = self.predump_pose(slot, self.p["dump_reverse_options"][0])
        cands = []
        for y0 in self.p["lane_ys"]:
            for yaw in self.p["lane_headings"]:
                c, s = math.cos(yaw), math.sin(yaw)
                # as close to the obstacle field as the excavation zone allows
                x0 = EXCAVATION_X - 0.05 - max(FRONT * c + HALF_W * abs(s), 0) - L * c
                x0 = min(x0, 2.0)
                if not self.lane_clear(x0, y0, yaw, L):
                    continue
                if any(math.hypot(x0 - dx, y0 - dy) < 0.5 for dx, dy in self.dug):
                    continue  # already dug; fresh regolith fills the bucket better
                end = (x0 + L * c, y0 + L * s, yaw)
                h = math.hypot(x0 - start[0], y0 - start[1]) + math.hypot(target[0] - end[0], target[1] - end[1]) + \
                    self.p["turn_weight"] * (abs(wrap(math.atan2(y0 - start[1], x0 - start[0]) - start[2])) +
                                             abs(wrap(math.atan2(target[1] - end[1], target[0] - end[0]) - yaw)))
                cands.append((h, (x0, y0, yaw), end))
        if not cands:
            self.dug.clear()  # every lane used once; start over
            return None
        cands.sort()
        best, plan_s = None, 0.0
        for _, lane, end in cands[: self.p["candidates_to_plan"]]:
            c1, t1 = self.plan_cost(start, lane)
            c2, t2 = self.plan_cost(end, target)
            plan_s += t1 + t2
            if best is None or c1 + c2 < best[0]:
                best = (c1 + c2, lane)
        self.event(f"lane chosen {tuple(round(v, 2) for v in best[1])} cost {best[0]:.2f} (planning {plan_s:.1f} s)")
        return best[1] if best[0] < math.inf else None

    def estimate(self, leg):
        t = [x["s"] for x in self.leg_times if x["leg"] == leg and x["ok"]]
        return sum(t) / len(t) if t else self.p[f"{leg}_time"]

    # ---------------------------------------------------------------- execution
    def drive(self, goal, desc):
        """NavigateToPose with stuck detection (commanded vs measured motion) and back-off-and-replan."""
        self.goal_desc = desc
        for attempt in range(self.p["nav_retries"]):
            if not self.check_clock():
                return False
            if not self.go_to(self.stamped(*goal)):
                self.event(f"{desc}: NavigateToPose not accepted, retry {attempt + 1}")
                continue
            track = []
            while not self.isTaskComplete():
                if not self.check_clock():
                    return False
                now = self.get_clock().now()
                pose = self.robot_pose("odom")
                if pose:
                    track = [(t, p) for t, p in track if (now - t).nanoseconds * 1e-9 < self.p["stuck_time"]] + [(now, pose)]
                commanding = now.nanoseconds * 1e-9 - self.last_cmd[1] < 1.0
                span = (now - track[0][0]).nanoseconds * 1e-9 if track else 0.0
                if commanding and span > self.p["stuck_time"] * 0.9:
                    moved = math.hypot(track[-1][1][0] - track[0][1][0], track[-1][1][1] - track[0][1][1])
                    turned = abs(wrap(track[-1][1][2] - track[0][1][2]))
                    if moved < 0.05 and turned < 0.1:
                        self.event(f"STUCK on {desc}: commanded motion, measured {moved:.2f} m; backing off")
                        self.cancelTask()
                        self.recover(self.last_cmd[0])
                        break
            else:
                if self.getResult() == TaskResult.SUCCEEDED:
                    return True
                self.event(f"{desc}: Nav2 {self.getResult().name} ({self.getTaskError()[1] if hasattr(self, 'getTaskError') else ''}), retry {attempt + 1}")
        return False

    def recover(self, last_v):
        """Move 0.3 m away from whatever stopped us (opposite to the last commanded direction), then replan."""
        task = self.driveOnHeading(0.3, 0.1, 8) if last_v < 0 else self.backup(0.3, 0.1, 8)
        while task and not self.isTaskComplete():
            pass
        self.relocalize_pub.publish(String(data="stuck recovery"))

    def run_action(self, client, goal, desc, timeout):
        if not client.wait_for_server(timeout_sec=10.0):
            self.event(f"{desc}: action server unavailable")
            return False
        f = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        handle = f.result()
        if handle is None or not handle.accepted:
            return False
        self.action_handle = handle
        r = handle.get_result_async()
        deadline = time.monotonic() + timeout
        while not r.done():
            rclpy.spin_until_future_complete(self, r, timeout_sec=0.2)
            # Offloading may finish after the timer if it had started (15.4.1); excavation may not
            if self.remaining() <= 0 and client is self.excavate_client:
                handle.cancel_goal_async()
                self.check_clock()
                return False
            if self.inhibited or time.monotonic() > deadline:
                handle.cancel_goal_async()
                return False
        self.action_handle = None
        return r.result().status == GoalStatus.STATUS_SUCCEEDED and r.result().result.success

    def timed(self, name, fn):
        t = self.elapsed()
        ok = fn()
        self.leg_times.append({"leg": name, "s": round(self.elapsed() - t, 1), "ok": bool(ok)})
        return ok

    def wait_localized(self):
        self.phase = "LOCALIZE"
        t_start = self.elapsed()
        spins = 0
        while not self.loc.get("localized"):
            rclpy.spin_once(self, timeout_sec=0.2)
            if not self.check_clock():
                return False
            waited = self.elapsed() - t_start
            if waited > self.p["localize_timeout"] * (spins + 1) and spins < 4:
                spins += 1
                self.event("no tag fix yet; turning 90 deg in the starting zone to look for tags")
                if not self.run_behavior(lambda: self.spin(1.57, 30)):
                    # Too close to a wall to turn in place (the footprint sweeps 0.73 m): move off it first.
                    # Nav2 collision-checks both; the moves stay inside the 2 x 2 m starting zone.
                    self.event("spin blocked; moving 0.4 m off the wall, then turning")
                    if not self.run_behavior(lambda: self.driveOnHeading(0.4, 0.1, 15)):
                        self.run_behavior(lambda: self.backup(0.3, 0.1, 15))
                    self.run_behavior(lambda: self.spin(1.57, 30))
            if self.elapsed() > self.p["start_deadline"] - 60:
                return False
        self.event(f"localized: tags {self.loc.get('tags')} residual {self.loc.get('residual'):.3f} m")
        return True

    def run_behavior(self, start):
        """Run a Nav2 behavior (spin/backup/drive) to completion; True if it succeeded."""
        task = start()
        while task and not self.isTaskComplete():
            pass
        return bool(task) and self.getResult() == TaskResult.SUCCEEDED

    def excavate_here(self):
        self.phase = "EXCAVATE"
        self.event("AUTONOMY ATTEMPT: excavation (hands-free)")
        ok = self.timed("excavate", lambda: self.run_action(self.excavate_client, Excavation.Goal(), "excavation", 120))
        self.segment("excavation", ok)
        self.event(f"excavation {'complete' if ok else 'FAILED'}")
        return ok

    def run(self):
        self.waitUntilNav2Active(localizer="robot_localization")  # no AMCL; RTAB-Map/Point-LIO localize
        if self.p["planner_id"]:
            self.planner_pub.publish(String(data=self.p["planner_id"]))
        self.t0 = self.get_clock().now()
        self.event("mission start (hands-free from the starting zone)")
        if not self.wait_localized():
            if self.inhibited:
                return
            # 5-minute rule fallback: excavate straight ahead in the starting zone, keep looking for tags
            self.event("no arena fix before the 5-min rule; excavating in place to start the cycle")
            self.excavate_here()
            if not self.wait_localized():
                return

        loaded = False
        while self.check_clock() and (not self.p["max_cycles"] or self.cycles < self.p["max_cycles"]):
            here = self.pose_now()
            if here is None:
                self.event("no arena pose (TF); waiting")
                self.spin_wait(2.0)
                continue
            if not loaded:
                lane = self.choose_lane(here) or self.choose_lane(here)
                if lane is None:
                    self.event("no clear excavation lane; waiting for the map")
                    self.spin_wait(2.0)
                    continue
                # Never start a cycle whose dump can't happen before the timer (only dumped regolith scores)
                need = self.estimate("excavate") + self.estimate("travel_loaded") + self.estimate("dump")
                if self.remaining() < need:
                    self.event(f"not starting another cycle: {self.remaining():.0f} s left, ~{need:.0f} s needed")
                    break
                self.phase = "TO_EXCAVATION"
                if not self.timed("to_lane", lambda: self.drive(lane, "excavation lane")):
                    self.dug.append(lane[:2])  # unreachable; try another lane
                    continue
                if self.excavate_here():
                    self.dug.append(lane[:2])
                    loaded = True
                continue

            choice = self.choose_dump(self.pose_now())
            if choice is None:
                self.event("no path to any dump slot; waiting for the map")
                self.spin_wait(2.0)
                continue
            _, slot, reverse = choice
            self.phase = "TRAVEL_LOADED"
            self.event(f"AUTONOMY ATTEMPT: travel loaded to dump slot x={slot:.2f}, reverse {reverse:.1f} m (hands-free)")
            ok = self.timed("travel_loaded", lambda: self.drive(self.predump_pose(slot, reverse), f"dump slot {slot:.2f}"))
            if not ok:
                self.segment("travel", False)
                continue
            self.segment("travel")
            self.phase = "DUMP"
            self.event("AUTONOMY ATTEMPT: dump (hands-free)")
            if reverse > 0:
                task = self.backup(reverse, 0.08, 20)
                while task and not self.isTaskComplete():
                    pass
            ok = self.timed("dump", lambda: self.run_action(self.dump_client, Depositing.Goal(), "dump", 120))
            self.segment("dump", ok)
            self.dumps[slot] += int(ok)
            pose = self.robot_pose()
            self.event(f"dump {'complete' if ok else 'FAILED'}", pose=pose and [round(v, 3) for v in pose],
                       pile=pose and [round(pose[0] - self.p["dump_offset"] * math.cos(pose[2]), 3),
                                      round(pose[1] - self.p["dump_offset"] * math.sin(pose[2]), 3)])
            loaded = False
            if not self.check_clock():
                break
            # Travel empty: back to the excavation zone completes the cycle (16.5.3)
            self.phase = "TRAVEL_EMPTY"
            self.event("AUTONOMY ATTEMPT: travel empty to the excavation zone (hands-free)")
            # The next excavation lane is the goal, so the loop top finds the robot already there
            goal = self.choose_lane(self.pose_now()) or (1.8, 2.5, math.pi)
            ok = self.timed("travel_empty", lambda: self.drive(goal, "return to excavation zone"))
            self.segment("travel", ok)
            if ok:
                self.cycles += 1
                self.event(f"CYCLE {self.cycles} complete")
                self.relocalize_pub.publish(String(data="back in the excavation zone"))  # tags are in view here
        self.phase = "DONE" if not self.inhibited else "INHIBITED"
        self.event(f"mission end: {self.cycles} cycles, segments {self.counts}")
        if not self.inhibited:
            self.inhibit("mission complete")

    def spin_wait(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = MissionExecutive()
    try:
        node.run()
        node.spin_wait(3.0)
    except KeyboardInterrupt:
        node.inhibit("shutdown")
    finally:
        node.log_file.close()


if __name__ == "__main__":
    main()
