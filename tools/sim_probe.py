#!/usr/bin/env python3
"""One-shot look at a running sim: robot pose (estimate vs ground truth) and the global costmap
around it in the arena frame. Testing aid only.

    tools/sim_probe.py [--seed N]
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node as GzNode
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node("sim_probe")
    buf = Buffer()
    TransformListener(buf, n)
    st = {}

    def on_gz(msg):
        try:
            for p in msg.pose:
                if p.name == "v2_bot":
                    q = p.orientation
                    st["gt"] = (p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))
        except Exception:
            pass

    gz = GzNode()
    gz.subscribe(Pose_V, "/world/default/pose/info", on_gz)
    n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: st.__setitem__("cm", m),
                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    end = time.time() + 6
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    R = buf.lookup_transform("arena", "base_link", Time()).transform
    est = (R.translation.x, R.translation.y, 2 * math.atan2(R.rotation.z, R.rotation.w))
    g = st.get("gt")
    print(f"estimate ({est[0]:.2f}, {est[1]:.2f}, {math.degrees(est[2]):.0f} deg)  ground truth "
          + (f"({g[0]:.2f}, {g[1]:.2f}, {math.degrees(g[2]):.0f} deg)  error {math.hypot(est[0] - g[0], est[1] - g[1]):.3f} m" if g else "n/a"))
    gt = json.load(open(os.path.expanduser(f"~/.ros/lunabot_sim/arena_{a.seed}/ground_truth.json")))
    near = [(o["kind"], o["x"], o["y"]) for o in gt["obstacles"] if g and math.hypot(o["x"] - g[0], o["y"] - g[1]) < 1.5]
    print("ground-truth obstacles within 1.5 m:", near)
    m = st.get("cm")
    if m:
        arr = np.array(m.data).reshape(m.info.height, m.info.width)
        o, r = m.info.origin.position, m.info.resolution
        T = buf.lookup_transform(m.header.frame_id, "arena", Time()).transform
        yaw = 2 * math.atan2(T.rotation.z, T.rotation.w)
        c, s = math.cos(yaw), math.sin(yaw)
        print("costmap around the robot, arena frame, 0.1 m/char (# lethal/inscribed, + high, . free, ? unknown, R robot):")
        for ay in np.arange(est[1] + 1.5, est[1] - 1.55, -0.1):
            row = ""
            for ax in np.arange(est[0] - 2.0, est[0] + 2.05, 0.1):
                X, Y = T.translation.x + c * ax - s * ay, T.translation.y + s * ax + c * ay
                i, j = int((Y - o.y) / r), int((X - o.x) / r)
                v = arr[i, j] if 0 <= i < arr.shape[0] and 0 <= j < arr.shape[1] else -1
                row += "R" if abs(ax - est[0]) < 0.06 and abs(ay - est[1]) < 0.06 else \
                    ("#" if v >= 99 else "+" if v >= 60 else "?" if v < 0 else ".")
            print(f"{ay:5.1f} {row}")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
