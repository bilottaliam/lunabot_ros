#!/usr/bin/env python3
"""Robot-side half of the MCC link: the small, arena-frame topics foxglove_bridge is allowed to send.

Everything the judges need for travel autonomy (guidebook 16, "visualization of the robot's real
time obstacle detection, associated mapping of obstacles, and resulting path planning"), at a
fixed low rate and already in the arena frame, so no /tf has to cross the link:
  /mcc/map        OccupancyGrid  global costmap max-pooled to `map_resolution`, cropped to known cells, 1 Hz
  /mcc/plan       Path           Nav2's current global plan, <= 60 poses, on change (<= 1 Hz)
  /mcc/pose       PoseStamped    robot pose, 2 Hz
  /mcc/obstacles  MarkerArray    detected rocks (red) and craters (orange), 1 Hz
Commands from the MCC arrive on /mission/command (std_msgs/String) and run here, on the robot,
because foxglove_bridge does not carry actions: excavate, dump, cancel, and
"snapshot <front|back|fisheye>", which sends one JPEG still on /mcc/snapshot (judges' regolith checks).
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from lunabot_msgs.action import Depositing, Excavation
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def yaw_of(q):
    return 2 * math.atan2(q.z, q.w)


class MccGateway(Node):
    def __init__(self):
        super().__init__("mcc_gateway")
        self.declare_parameter("frame", "arena")
        self.declare_parameter("map_resolution", 0.1)
        self.frame = self.get_parameter("frame").value
        self.res = self.get_parameter("map_resolution").value
        self.tf = Buffer()
        TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, "/mcc/map", 1)
        self.plan_pub = self.create_publisher(Path, "/mcc/plan", 1)
        self.pose_pub = self.create_publisher(PoseStamped, "/mcc/pose", 1)
        self.obs_pub = self.create_publisher(MarkerArray, "/mcc/obstacles", 1)
        self.snap_pub = self.create_publisher(CompressedImage, "/mcc/snapshot", 1)
        self.snap_sub = None
        self.costmap = self.plan = None
        self.plan_sent = 0.0
        self.obstacles = {"rock": None, "crater": None}
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: setattr(self, "costmap", m), latched)
        self.create_subscription(Path, "/plan", self.on_plan, 1)
        self.create_subscription(PoseArray, "/terrain/rocks", lambda m: self.obstacles.__setitem__("rock", m), latched)
        self.create_subscription(PoseArray, "/terrain/craters", lambda m: self.obstacles.__setitem__("crater", m), latched)
        self.create_subscription(String, "/mission/command", self.on_command, 10)
        self.excavate = ActionClient(self, Excavation, "auto_excavation_action")
        self.dump = ActionClient(self, Depositing, "auto_depositing_action")
        self.handles = []
        self.create_timer(1.0, self.slow)
        self.create_timer(0.5, self.fast)

    def to_arena(self, frame):
        """(x, y, yaw) of `frame` in the arena frame, or None before the tags have fixed it."""
        try:
            t = self.tf.lookup_transform(self.frame, frame, Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def stamp(self, msg):
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def fast(self):
        p = self.to_arena("base_link")
        if p:
            m = self.stamp(PoseStamped())
            m.pose.position.x, m.pose.position.y = p[0], p[1]
            m.pose.orientation.z, m.pose.orientation.w = math.sin(p[2] / 2), math.cos(p[2] / 2)
            self.pose_pub.publish(m)

    def on_plan(self, msg):
        self.plan = msg
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.plan_sent >= 1.0:
            self.send_plan()

    def send_plan(self):
        T = self.to_arena(self.plan.header.frame_id) if self.plan else None
        if T is None:
            return
        c, s = math.cos(T[2]), math.sin(T[2])
        out = self.stamp(Path())
        poses = self.plan.poses
        for q in poses[:: max(1, len(poses) // 60)] + poses[-1:]:
            p = PoseStamped(header=out.header)
            x, y = q.pose.position.x, q.pose.position.y
            p.pose.position.x, p.pose.position.y = T[0] + c * x - s * y, T[1] + s * x + c * y
            yaw = yaw_of(q.pose.orientation) + T[2]
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            out.poses.append(p)
        self.plan_pub.publish(out)
        self.plan_sent = self.get_clock().now().nanoseconds * 1e-9

    def slow(self):
        if self.costmap:
            self.send_map()
        T = self.to_arena(self.costmap.header.frame_id) if self.costmap else None
        if T is None:
            return
        c, s = math.cos(T[2]), math.sin(T[2])
        arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for kind, rgb in (("rock", (0.9, 0.1, 0.1)), ("crater", (1.0, 0.55, 0.0))):
            for p in (self.obstacles[kind].poses if self.obstacles[kind] else []):
                if p.position.z > 0.4:  # walls, column: large blobs are already in the map
                    continue
                m = Marker(ns=kind, id=len(arr.markers), type=Marker.CYLINDER, action=Marker.ADD)
                m.header = self.stamp(PoseStamped()).header
                m.pose.position.x = T[0] + c * p.position.x - s * p.position.y
                m.pose.position.y = T[1] + s * p.position.x + c * p.position.y
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = 2 * max(p.position.z, 0.1)
                m.scale.z = 0.05
                m.color.r, m.color.g, m.color.b, m.color.a = *rgb, 0.9
                arr.markers.append(m)
        self.obs_pub.publish(arr)

    def send_map(self):
        g = self.costmap
        T = self.to_arena(g.header.frame_id)
        if T is None:
            return
        a = np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
        k = max(1, round(self.res / g.info.resolution))
        h, w = (a.shape[0] // k) * k, (a.shape[1] // k) * k
        a = a[:h, :w].reshape(h // k, k, w // k, k)
        known = (a >= 0).any(axis=(1, 3))
        a = np.where(known, a.max(axis=(1, 3)), -1)   # max-pool: never hide an obstacle by downsampling
        rows, cols = np.nonzero(known)
        if not len(rows):
            return
        r0, r1, c0, c1 = rows.min(), rows.max() + 1, cols.min(), cols.max() + 1
        out = self.stamp(OccupancyGrid())
        out.info.resolution = g.info.resolution * k
        out.info.height, out.info.width = r1 - r0, c1 - c0
        # cropped origin, expressed in the arena frame (the grid may be rotated relative to it)
        ox = g.info.origin.position.x + c0 * out.info.resolution
        oy = g.info.origin.position.y + r0 * out.info.resolution
        cy, sy = math.cos(T[2]), math.sin(T[2])
        out.info.origin.position.x = T[0] + cy * ox - sy * oy
        out.info.origin.position.y = T[1] + sy * ox + cy * oy
        out.info.origin.orientation.z, out.info.origin.orientation.w = math.sin(T[2] / 2), math.cos(T[2] / 2)
        # costmap 0-100 already (Nav2 publishes scaled occupancy); keep 3 levels to compress well
        b = a[r0:r1, c0:c1]
        out.data = np.where(b < 0, -1, np.where(b >= 99, 100, np.where(b > 50, 60, 0))).astype(np.int8).ravel().tolist()
        self.map_pub.publish(out)

    def snapshot(self, camera):
        """Subscribe to the camera's JPEG topic just long enough to forward one frame."""
        def once(img):
            self.snap_pub.publish(img)
            self.destroy_subscription(self.snap_sub)
            self.snap_sub = None
        if self.snap_sub is None:
            self.snap_sub = self.create_subscription(CompressedImage, f"/camera_{camera}/color/image_compressed", once, 1)

    def on_command(self, msg):
        cmd = msg.data.strip()
        if cmd.startswith("snapshot"):
            self.snapshot((cmd.split() + ["front"])[1])
            return
        if cmd == "cancel":
            for h in self.handles:
                h.cancel_goal_async()
            self.handles = []
            return
        client, goal = {"excavate": (self.excavate, Excavation.Goal()), "dump": (self.dump, Depositing.Goal())}.get(cmd, (None, None))
        if client is None:
            self.get_logger().warning(f"unknown MCC command '{cmd}'")
            return
        if not client.server_is_ready():
            self.get_logger().warning(f"{cmd}: action server not running")
            return
        client.send_goal_async(goal).add_done_callback(lambda f: self.handles.append(f.result()))
        self.get_logger().info(f"MCC command: {cmd}")


def main():
    rclpy.init()
    rclpy.spin(MccGateway())


if __name__ == "__main__":
    main()
