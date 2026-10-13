"""MCC autonomy views: live arena map (map, obstacles, plan, pose) and the autonomy/run panel.

Everything here reads the small /mcc/* and /mission/state topics that cross the link
(lunabot_util/mcc_gateway.py), already in the arena frame.
"""
import json
import math
import os
import threading
import time

import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import (QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QSpinBox,
                             QVBoxLayout)
from std_msgs.msg import Empty, String
from visualization_msgs.msg import MarkerArray

ARENA = (6.88, 5.0)
ZONES = {"start": (0, 0, 2, 2), "construction": (3.88, 0, 6.88, 1.5), "berm": (4.28, 0.45, 6.48, 1.35)}
FOOTPRINT = np.array([[0.6, 0.41], [0.6, -0.41], [-0.45, -0.41], [-0.45, 0.41]])
RUN_S, START_RULE_S = 900, 300


class MccState:
    """Thread-safe latest copies of the MCC topics, plus the MCC -> robot publishers."""

    def __init__(self, node):
        self.lock = threading.Lock()
        self.grid = self.plan = self.pose = None
        self.obstacles = []
        self.mission = {}
        node.create_subscription(OccupancyGrid, "/mcc/map", lambda m: self.set("grid", m), 1)
        node.create_subscription(Path, "/mcc/plan", lambda m: self.set("plan", m), 1)
        node.create_subscription(PoseStamped, "/mcc/pose", lambda m: self.set("pose", m), 1)
        node.create_subscription(MarkerArray, "/mcc/obstacles", lambda m: self.set("obstacles", m.markers), 1)
        node.create_subscription(String, "/mission/state", lambda m: self.set("mission", json.loads(m.data)), 10)
        self.inhibit_pub = node.create_publisher(Empty, "/mission/inhibit", 10)
        self.cmd_vel_pub = node.create_publisher(Twist, "/cmd_vel", 10)

    def set(self, k, v):
        with self.lock:
            setattr(self, k, v)

    def snapshot(self):
        with self.lock:
            return self.grid, self.plan, self.pose, list(self.obstacles), dict(self.mission)

    def inhibit(self):
        for _ in range(3):
            self.inhibit_pub.publish(Empty())
            self.cmd_vel_pub.publish(Twist())


class MapView(QLabel):
    """Top-down arena view drawn with OpenCV: what the robot mapped, detected, and plans to drive."""

    PX = 90  # pixels per metre

    def __init__(self, state):
        super().__init__("Waiting for /mcc/map")
        self.state = state
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background-color: #0d0d0d; color: #888;")

    def px(self, x, y, h):
        return int(20 + x * self.PX), int(h - 20 - y * self.PX)

    def render(self):
        grid, plan, pose, obstacles, _ = self.state.snapshot()
        w, h = int(ARENA[0] * self.PX) + 40, int(ARENA[1] * self.PX) + 40
        img = np.full((h, w, 3), 18, np.uint8)
        if grid is not None:
            a = np.array(grid.data, np.int16).reshape(grid.info.height, grid.info.width)
            o = grid.info.origin
            yaw = 2 * math.atan2(o.orientation.z, o.orientation.w)
            c, s, r = math.cos(yaw), math.sin(yaw), grid.info.resolution
            colors = {-1: (30, 30, 30), 0: (70, 70, 70), 60: (40, 110, 170), 100: (40, 40, 200)}
            for (i, j), v in np.ndenumerate(a):
                gx, gy = (j + 0.5) * r, (i + 0.5) * r
                x, y = o.position.x + c * gx - s * gy, o.position.y + s * gx + c * gy
                p = self.px(x, y, h)
                half = max(1, int(r * self.PX / 2))
                cv2.rectangle(img, (p[0] - half, p[1] - half), (p[0] + half, p[1] + half), colors.get(int(v), (70, 70, 70)), -1)
        cv2.rectangle(img, self.px(0, 0, h), self.px(*ARENA, h), (200, 200, 200), 1)
        for name, (x0, y0, x1, y1) in ZONES.items():
            col = {"start": (80, 200, 80), "construction": (60, 200, 200), "berm": (60, 60, 255)}[name]
            cv2.rectangle(img, self.px(x0, y0, h), self.px(x1, y1, h), col, 2 if name == "berm" else 1)
        cv2.line(img, self.px(2.5, 0, h), self.px(2.5, ARENA[1], h), (120, 120, 120), 1)
        for m in obstacles:
            col = (40, 40, 230) if m.ns == "rock" else (0, 140, 255)
            cv2.circle(img, self.px(m.pose.position.x, m.pose.position.y, h), max(3, int(m.scale.x / 2 * self.PX)), col, 2)
        if plan is not None and len(plan.poses) > 1:
            pts = np.array([self.px(p.pose.position.x, p.pose.position.y, h) for p in plan.poses], np.int32)
            cv2.polylines(img, [pts], False, (255, 220, 0), 2)
        if pose is not None:
            yaw = 2 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w)
            c, s = math.cos(yaw), math.sin(yaw)
            fp = FOOTPRINT @ np.array([[c, s], [-s, c]]) + [pose.pose.position.x, pose.pose.position.y]
            cv2.polylines(img, [np.array([self.px(x, y, h) for x, y in fp], np.int32)], True, (255, 255, 255), 2)
            tip = self.px(pose.pose.position.x + 0.5 * c, pose.pose.position.y + 0.5 * s, h)
            cv2.arrowedLine(img, self.px(pose.pose.position.x, pose.pose.position.y, h), tip, (255, 255, 255), 2)
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        q = QImage(rgb.data, rgb.shape[1], rgb.shape[0], 3 * rgb.shape[1], QImage.Format_RGB888)
        self.setPixmap(QPixmap.fromImage(q).scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))


class AutonomyPanel(QGroupBox):
    """Run timer with the 5-minute rule, mission phase and segment counters, inhibit, bandwidth score,
    and a timestamped autonomy-attempt log (also written to ~/.ros/lunabot_mcc/)."""

    def __init__(self, state, robot, log_fn):
        super().__init__("Autonomy / Run")
        self.state, self.robot, self.log_fn = state, robot, log_fn
        self.run_t0 = None
        self.first_cycle_t = None
        self.seen = set()
        os.makedirs(os.path.expanduser("~/.ros/lunabot_mcc"), exist_ok=True)
        self.log_file = open(os.path.expanduser(time.strftime("~/.ros/lunabot_mcc/run_%Y%m%d_%H%M%S.log")), "a")
        mono = QFont("Monospace", 10)
        g = QGridLayout()
        self.setLayout(g)
        self.timer_label = QLabel("--:--")
        self.timer_label.setFont(QFont("Monospace", 18, QFont.Bold))
        self.rule_label = QLabel("5-min rule: run not started")
        self.phase_label, self.seg_label, self.loc_label, self.score_label = (QLabel("-") for _ in range(4))
        for lab in (self.rule_label, self.phase_label, self.seg_label, self.loc_label, self.score_label):
            lab.setFont(mono)
            lab.setWordWrap(True)
        start = QPushButton("Start run timer")
        start.clicked.connect(self.start_run)
        inhibit = QPushButton("INHIBIT ROBOT")
        inhibit.setStyleSheet("background-color: #b00020; color: white; font-weight: bold; padding: 8px;")
        inhibit.clicked.connect(self.inhibit)
        self.cams = QSpinBox()
        self.cams.setRange(0, 2)
        self.cams.setPrefix("NASA cams used: ")
        row = QHBoxLayout()
        row.addWidget(self.timer_label)
        row.addWidget(start)
        g.addLayout(row, 0, 0, 1, 2)
        for i, lab in enumerate((self.rule_label, self.phase_label, self.seg_label, self.loc_label), start=1):
            g.addWidget(lab, i, 0, 1, 2)
        g.addWidget(self.cams, 5, 0)
        g.addWidget(self.score_label, 5, 1)
        g.addWidget(inhibit, 6, 0, 1, 2)

    def start_run(self):
        self.run_t0, self.first_cycle_t = time.monotonic(), None
        self.announce("RUN TIMER STARTED")

    def inhibit(self):
        self.state.inhibit()
        self.announce("INHIBIT command sent")

    def announce(self, text):
        t = self.run_clock()
        line = f"[{time.strftime('%H:%M:%S')}] [run {t}] {text}"
        self.log_file.write(line + "\n")
        self.log_file.flush()
        self.log_fn(line)

    def run_clock(self):
        if self.run_t0 is None:
            return "--:--"
        s = int(time.monotonic() - self.run_t0)
        return f"{s // 60:02d}:{s % 60:02d}"

    def refresh(self):
        _, _, _, _, m = self.state.snapshot()
        elapsed = time.monotonic() - self.run_t0 if self.run_t0 else None
        self.timer_label.setText(self.run_clock())
        self.timer_label.setStyleSheet("color: #ff5050;" if elapsed and elapsed > RUN_S else "")
        seg = m.get("segments", {})
        started = any(v[1] for v in seg.values()) or m.get("phase") in ("TO_EXCAVATION", "EXCAVATE")
        if started and self.first_cycle_t is None and elapsed is not None:
            self.first_cycle_t = elapsed
        if elapsed is None:
            self.rule_label.setText("5-min rule: run not started")
        elif self.first_cycle_t is not None:
            self.rule_label.setText(f"5-min rule: OK, cycle started at {int(self.first_cycle_t) // 60}:{int(self.first_cycle_t) % 60:02d}")
            self.rule_label.setStyleSheet("color: #50d050;")
        else:
            left = START_RULE_S - elapsed
            self.rule_label.setText(f"5-min rule: start a cycle within {max(0, int(left)) // 60}:{max(0, int(left)) % 60:02d}")
            self.rule_label.setStyleSheet("color: #ff5050;" if left < 60 else "color: #ffb000;")
        self.phase_label.setText(f"Phase: {m.get('phase', '-')}   cycles: {m.get('cycles', 0)}   "
                                 f"mission clock {m.get('elapsed', 0):.0f}s")
        self.seg_label.setText("Segments auto/total: " + "  ".join(f"{k[:4]} {v[0]}/{v[1]}" for k, v in seg.items()))
        self.loc_label.setText(f"Localized: {'yes' if m.get('localized') else 'NO'}   goal: {m.get('goal', '')}")
        mbps = self.robot.bandwidth_avg_total + 0.5 * self.cams.value()
        self.score_label.setText(f"BW {self.robot.bandwidth_avg_total:.3f} Mbps -> score {0 if mbps > 4 else 120 - 30 * mbps:.0f}/120")
        for e in m.get("events", []):
            key = (e.get("t"), e.get("msg"))
            if key not in self.seen:
                self.seen.add(key)
                self.announce(f"robot t={e.get('t')}s {e.get('msg')}")
