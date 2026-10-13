#!/usr/bin/env python3
"""Arena frame from AprilTags on the starting-zone frame (guidebook 13.7.2).

apriltag_ros publishes camera->tag TF. Each detection's tag centre is looked up in `map`
(Point-LIO + RTAB-Map) at the image stamp and kept for `window` seconds. With >= 2 distinct tags
the 2D rigid transform arena->map is solved from tag centres alone (their arena positions are
measured at setup, lunabot_config/params/arena/arena_tags.yaml), so the tag-frame orientation
convention and the noisy single-tag PnP orientation never enter. The result is published as the
TF arena -> map and re-solved whenever tags are seen again, which absorbs map drift.

Health: a jump in map->odom (RTAB-Map new map / big correction) or an implausible Point-LIO speed
drops the samples taken before it and requests a re-observation (status.need_tags).
"""
import json
import math
from collections import deque

import numpy as np
import rclpy
import yaml
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


def solve_2d(src, dst):
    """Least-squares R, t with dst = R @ src + t (2D Kabsch)."""
    cs, cd = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - cs).T @ (dst - cd))
    D = np.diag([1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cd - R @ cs


class ArenaLocalizer(Node):
    def __init__(self):
        super().__init__("arena_localizer")
        for k, v in {"tags_file": "", "map_frame": "map", "arena_frame": "arena", "odom_frame": "odom",
                     "tag_frame_prefix": "tag36h11:", "window": 120.0, "min_samples": 3,
                     "max_residual": 0.15, "max_tag_range": 4.0, "jump_threshold": 0.25,
                     "max_lio_speed": 1.0, "pan_positions": [-1.4, -0.7, 0.0, 0.7, 1.4, 0.7, 0.0, -0.7],
                     "pan_dwell": 2.0}.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        with open(g("tags_file")) as f:
            self.tags = {t["id"]: np.array([t["x"], t["y"]]) for t in yaml.safe_load(f)["tags"]}
        self.map, self.arena, self.odom, self.prefix = g("map_frame"), g("arena_frame"), g("odom_frame"), g("tag_frame_prefix")
        self.window, self.min_samples, self.max_res = g("window"), g("min_samples"), g("max_residual")
        self.max_range, self.jump, self.max_speed = g("max_tag_range"), g("jump_threshold"), g("max_lio_speed")
        self.pans, self.dwell = g("pan_positions"), g("pan_dwell")

        self.samples = deque()        # (t, id, x_map, y_map)
        self.solution = None          # (x, y, yaw) of map in arena
        self.residual = None
        self.need_tags = True
        self.last_fault = None
        self.events = []
        self.last_map_odom = None
        self.pan_i, self.pan_t = 0, 0.0

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.tf_pub = StaticTransformBroadcaster(self)
        self.status_pub = self.create_publisher(String, "/arena_localizer/status", 10)
        self.pan_pub = self.create_publisher(Float64MultiArray, "/camera_controller/commands", 10)
        self.create_subscription(AprilTagDetectionArray, "/detections", self.on_detections, 10)
        self.create_subscription(Odometry, "/lio_odom", self.on_odom, 10)
        self.create_subscription(String, "/arena_localizer/request", lambda m: self.request(m.data), 10)
        self.create_timer(0.5, self.tick)

    def request(self, why):
        if not self.need_tags:
            self.get_logger().info(f"re-observation requested: {why}")
        self.need_tags = True

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_detections(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        for d in msg.detections:
            if d.id not in self.tags:
                continue
            try:
                cam = self.tf.lookup_transform(msg.header.frame_id, f"{self.prefix}{d.id}", stamp, Duration(seconds=0.3))
                t = self.tf.lookup_transform(self.map, f"{self.prefix}{d.id}", stamp, Duration(seconds=0.3))
            except TransformException:
                continue
            c = cam.transform.translation
            if math.sqrt(c.x ** 2 + c.y ** 2 + c.z ** 2) > self.max_range:  # PnP range error grows with distance
                continue
            self.samples.append((stamp.nanoseconds * 1e-9, d.id, t.transform.translation.x, t.transform.translation.y))
        self.solve()

    def solve(self):
        now = self.now_s()
        while self.samples and now - self.samples[0][0] > self.window:
            self.samples.popleft()
        per_tag = {}
        for _, i, x, y in self.samples:
            per_tag.setdefault(i, []).append((x, y))
        ids = [i for i, v in per_tag.items() if len(v) >= self.min_samples]
        if len(ids) < 2:
            return
        src = np.array([np.median(per_tag[i], axis=0) for i in ids])
        dst = np.array([self.tags[i] for i in ids])
        R, t = solve_2d(src, dst)
        res = float(np.sqrt(np.mean(np.sum((src @ R.T + t - dst) ** 2, axis=1))))
        if res > self.max_res:
            self.get_logger().warning(f"tag fit rejected: residual {res:.3f} m over tags {ids}", throttle_duration_sec=5.0)
            return
        new = (float(t[0]), float(t[1]), math.atan2(R[1, 0], R[0, 0]))
        if self.solution is None or self.need_tags:
            delta = None if self.solution is None else (math.hypot(new[0] - self.solution[0], new[1] - self.solution[1]),
                                                        abs(math.remainder(new[2] - self.solution[2], math.tau)))
            self.event(f"arena fixed from tags {sorted(ids)} residual {res:.3f} m" +
                       ("" if delta is None else f", correction {delta[0]:.3f} m / {math.degrees(delta[1]):.1f} deg"))
        self.solution, self.residual, self.tags_used, self.need_tags = new, res, sorted(ids), False
        self.broadcast()

    def broadcast(self):
        x, y, yaw = self.solution
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = self.arena, self.map
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf_pub.sendTransform(tf)

    def on_odom(self, msg):
        v = msg.twist.twist.linear
        if math.hypot(v.x, v.y) > self.max_speed:
            self.health_fault(f"Point-LIO speed {math.hypot(v.x, v.y):.2f} m/s is implausible (diverging?)")

    def health_fault(self, why):
        self.last_fault = self.now_s()
        if self.need_tags:
            return
        self.samples.clear()  # samples taken in a frame that just moved are no longer comparable
        self.event(why)
        self.request(why)

    def event(self, text):
        self.get_logger().info(text)
        self.events = (self.events + [f"{self.now_s():.1f} {text}"])[-10:]

    def tick(self):
        try:
            t = self.tf.lookup_transform(self.map, self.odom, Time())
            mo = (t.transform.translation.x, t.transform.translation.y, 2 * math.atan2(t.transform.rotation.z, t.transform.rotation.w))
            if self.last_map_odom and (math.hypot(mo[0] - self.last_map_odom[0], mo[1] - self.last_map_odom[1]) > self.jump or
                                       abs(math.remainder(mo[2] - self.last_map_odom[2], math.tau)) > 0.17):
                self.health_fault("map->odom jumped (RTAB-Map correction or new map)")
            self.last_map_odom = mo
        except TransformException:
            pass
        if self.need_tags:  # sweep the fisheye; it is the only camera that sees the wall tags (D456s look 48 deg down)
            if self.now_s() - self.pan_t > self.dwell:
                self.pan_i, self.pan_t = (self.pan_i + 1) % len(self.pans), self.now_s()
                self.pan_pub.publish(Float64MultiArray(data=[float(self.pans[self.pan_i])]))
        elif self.pan_i != -1:
            self.pan_i = -1
            self.pan_pub.publish(Float64MultiArray(data=[0.0]))
        if self.solution:
            self.broadcast()  # refresh the stamp for late-joining listeners
        self.status_pub.publish(String(data=json.dumps({
            "localized": self.solution is not None, "need_tags": self.need_tags,
            "residual": self.residual, "tags": getattr(self, "tags_used", []),
            "map_in_arena": self.solution, "last_fault": self.last_fault, "events": self.events})))


def main():
    rclpy.init()
    rclpy.spin(ArenaLocalizer())


if __name__ == "__main__":
    main()
