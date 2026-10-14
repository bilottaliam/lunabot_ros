#!/usr/bin/env python3
"""Plain-assert checks of the autonomy stack's pure logic (no ROS graph, no sim):

    source ~/lunabot_ws/install/setup.bash && python3 tools/test_autonomy_logic.py
"""
import math
import os
import sys
import tempfile

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(REPO, "lunabot_sim", "scripts"), os.path.join(REPO, "lunabot_nav", "scripts")]
import generate_arena as ga  # noqa: E402
from arena_localizer import solve_2d  # noqa: E402
from mission_executive import path_metrics  # noqa: E402


def test_generator():
    with tempfile.TemporaryDirectory() as d:
        for seed in range(30):
            gt = ga.generate(seed, d, randomize_start=True)
            obs = gt["obstacles"]
            assert sum(o["kind"] == "boulder" and o["zone"] == "obstacle" for o in obs) >= 3, seed
            assert sum(o["kind"] == "crater" for o in obs) >= 3, seed
            for o in obs:
                lo, hi = (ga.BOULDER_D if o["kind"] == "boulder" else ga.CRATER_D)
                assert lo <= o["d"] <= hi, (seed, o)
                assert not ga.in_rect(o["x"], o["y"], ga.START_ZONE, o["d"] / 2), (seed, o)   # 11: none in the starting zone
                assert not ga.in_rect(o["x"], o["y"], ga.CONSTRUCTION, o["d"] / 2), (seed, o)
                assert o["kind"] == "boulder" or o["zone"] == "obstacle", (seed, o)            # craters only in the obstacle zone
            for i, a in enumerate(obs):
                for b in obs[i + 1:]:
                    assert math.hypot(a["x"] - b["x"], a["y"] - b["y"]) > (a["d"] + b["d"]) / 2, (seed, a, b)
            assert ga.route_exists(obs), seed
            s = gt["start"]
            fp = ga.FOOTPRINT @ np.array([[math.cos(s["yaw"]), math.sin(s["yaw"])], [-math.sin(s["yaw"]), math.cos(s["yaw"])]]) + [s["x"], s["y"]]
            assert (fp >= 0).all() and (fp <= 2.0).all(), (seed, s)                            # footprint inside the 2 x 2 m zone
            assert s["heading"] in ga.HEADINGS
    # same seed -> same layout
    with tempfile.TemporaryDirectory() as d1, tempfile.TemporaryDirectory() as d2:
        assert ga.generate(7, d1, True) == ga.generate(7, d2, True)


def test_solve_2d():
    rng = np.random.default_rng(0)
    tags = np.array([[t["x"], t["y"]] for t in __import__("yaml").safe_load(open(os.path.join(
        REPO, "lunabot_config", "params", "arena", "arena_tags.yaml")))["tags"]])
    berm = np.array([5.38, 0.9])
    for _ in range(50):
        yaw, t = rng.uniform(-math.pi, math.pi), rng.uniform(-3, 3, 2)
        R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        in_map = (tags - t) @ R            # tag positions as seen in a map frame rotated/shifted from the arena
        berm_map = (berm - t) @ R
        for k, tol in ((2, 0.10), (4, 0.06)):  # works from any 2+ tags; 1 cm tag noise -> berm error
            R2, t2 = solve_2d(in_map[:k] + rng.normal(0, 0.01, (k, 2)), tags[:k])
            assert np.linalg.norm(R2 @ berm_map + t2 - berm) < tol, (k, np.linalg.norm(R2 @ berm_map + t2 - berm))


def test_single_tag_normal():
    """Tag z axis -> ground-plane normal yaw, as arena_localizer computes it from the TF quaternion."""
    def mul(a, b):  # quaternion product, (x, y, z, w)
        ax, ay, az, aw = a; bx, by, bz, bw = b
        return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)
    for yaw in np.linspace(-3, 3, 7):
        # a vertical tag facing `yaw`: rotate the tag's z onto +x (90 deg about y), then turn about z
        q = mul((0, 0, math.sin(yaw / 2), math.cos(yaw / 2)), (0, math.sin(math.pi / 4), 0, math.cos(math.pi / 4)))
        nx, ny = 2 * (q[0] * q[2] + q[1] * q[3]), 2 * (q[1] * q[2] - q[0] * q[3])
        assert abs(math.remainder(math.atan2(ny, nx) - yaw, math.tau)) < 1e-6


class _P:
    def __init__(self, x, y, yaw):
        self.pose = type("pose", (), {})()
        self.pose.position = type("pt", (), {"x": x, "y": y})()
        self.pose.orientation = type("q", (), {"z": math.sin(yaw / 2), "w": math.cos(yaw / 2)})()


def test_path_metrics():
    fwd = type("path", (), {"poses": [_P(0.1 * i, 0, 0) for i in range(11)]})()
    assert np.allclose(path_metrics(fwd), (1.0, 0.0, 0.0))
    rev = type("path", (), {"poses": [_P(-0.1 * i, 0, 0) for i in range(11)]})()   # facing +x, moving -x
    assert np.allclose(path_metrics(rev), (1.0, 0.0, 1.0))
    turn = type("path", (), {"poses": [_P(0, 0, a) for a in np.linspace(0, math.pi / 2, 10)]})()
    assert np.allclose(path_metrics(turn)[:2], (0.0, math.pi / 2))


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"ok {name}")
