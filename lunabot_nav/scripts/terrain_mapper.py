#!/usr/bin/env python3
"""Height-based obstacle layer: rocks above the ground plane, craters (negative obstacles) below it.

Accumulates Point-LIO's body-frame cloud (/cloud_registered_body), placed in the gravity-aligned odom frame, into a
per-cell evidence grid in `grid_frame` and publishes:
  /terrain/grid    OccupancyGrid for Nav2 (StaticLayer, trinary_costmap: false):
                   0 = seen free, `unknown_value` = never seen, 100 = obstacle/crater + hard margin
  /terrain/rocks, /terrain/craters   PoseArray of detected obstacle centres (MCC, test harness)

Ground reference: the local ground around each cell (mean of the cells' lowest returns within
`ground_window`), not the robot's own height. A robot standing in the pit it dug would otherwise see the
ground around it as rock, and it makes the map independent of odom z drift.

Zones (arena frame, from the arena localizer's TF): craters are lethal in the obstacle zone only. In the
excavation and construction zones depressions are our own pits, legal to drive into (guidebook 17.1 item 8),
and the robot has to drive into them to keep digging. Without the arena frame everything is obstacle zone.
"""
import math
from array import array

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tf_matrix(t):
    M = np.eye(4)
    M[:3, :3] = quat_matrix(t.transform.rotation)
    p = t.transform.translation
    M[:3, 3] = p.x, p.y, p.z
    return M


def classify_scan(ij, z, n, rock_h, crater_d, window):
    """Per-scan cell classes from points in grid cells ij (N x 2) with heights z: (high, low, ground) n x n masks.
    Heights are judged against the local ground: the mean of the cells' lowest returns within `window` cells,
    computed twice so rocks and craters don't bias their own reference."""
    flat = ij[:, 0] * n + ij[:, 1]
    zmin = np.full(n * n, np.inf)
    zmax = np.full(n * n, -np.inf)
    np.minimum.at(zmin, flat, z)
    np.maximum.at(zmax, flat, z)
    zmin, zmax = zmin.reshape(n, n), zmax.reshape(n, n)
    seen = np.isfinite(zmin)
    use = seen
    for _ in range(2):
        w = use.astype(np.float32)
        total = cv2.boxFilter(np.where(use, zmin, 0).astype(np.float32), -1, (window, window), normalize=False)
        ground = total / np.maximum(cv2.boxFilter(w, -1, (window, window), normalize=False), 1)
        hi = seen & (zmax - ground > rock_h)
        lo = seen & (zmin - ground < -crater_d)
        use = seen & ~hi & ~lo
    return hi, lo, use & (np.abs(zmin - ground) < 0.05)


def tilt(q):
    """4x4 rotation with only the roll/pitch of quaternion q (its yaw removed)."""
    R = quat_matrix(q)
    yaw = math.atan2(R[1, 0], R[0, 0])
    T = np.eye(4)
    T[:3, :3] = np.array([[math.cos(yaw), math.sin(yaw), 0], [-math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]) @ R
    return T


class TerrainMapper(Node):
    def __init__(self):
        super().__init__("terrain_mapper")
        p = {
            # grid in Point-LIO's frame: RTAB-Map's map corrections don't smear it
            "cloud_topic": "/cloud_registered_body", "grid_frame": "odom", "base_frame": "base_link",
            "resolution": 0.05, "size": 16.0,
            "ground_offset": -0.02,      # wheel contact below base_link (URDF: wheel z 0.08, r 0.1016)
            "rock_height": 0.10,         # above ground: boulders are 30-40 cm across, of varying height
            "crater_depth": 0.10,        # below ground: craters are 40-50 cm wide and deep
            "max_height": 1.5, "max_range": 6.0, "min_hits": 2,
            "rock_margin": 0.15,         # hard clearance painted lethal around rocks (-50 per contact)
            "crater_margin": 0.20,       # a wheel in a crater is a penalty and may end the run
            "max_rock_radius": 0.35,     # blobs larger than this (walls, column) get no hard margin
            "ground_window": 1.0,        # local ground reference (m): several crater widths, < a typical dug pit
            # arena-frame rects [xmin, ymin, xmax, ymax, ...] where depressions aren't craters: the excavation
            # zone (x < 2.5, incl. the starting zone) and the construction zone (11.1, Figure 9)
            "dig_zones": [-1.0, -1.0, 2.5, 6.0, 3.88, -1.0, 7.9, 1.5],
            "arena_frame": "arena",
            "unknown_value": 25, "publish_rate": 2.0,
        }
        for k, v in p.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        self.frame, self.base = g("grid_frame"), g("base_frame")
        self.res, self.min_hits = g("resolution"), g("min_hits")
        self.rock_h, self.crater_d, self.max_h, self.max_r = g("rock_height"), g("crater_depth"), g("max_height"), g("max_range")
        self.ground_offset, self.unknown = g("ground_offset"), g("unknown_value")
        self.max_rock_radius = g("max_rock_radius")
        self.window = 2 * int(g("ground_window") / self.res / 2) + 1
        self.dig_zones = np.array(g("dig_zones")).reshape(-1, 4)
        self.arena = g("arena_frame")
        self.n = int(g("size") / self.res)
        self.origin = -g("size") / 2
        self.high = np.zeros((self.n, self.n), np.uint8)
        self.low = np.zeros_like(self.high)
        self.ground = np.zeros_like(self.high)
        self.kernels = {k: cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * int(g(k) / self.res) + 1,) * 2)
                        for k in ("rock_margin", "crater_margin")}

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(PointCloud2, g("cloud_topic"), self.on_cloud, qos_profile_sensor_data)
        # Roll/pitch from Point-LIO's attitude: the sim's odom->base_link comes from a 2D EKF (no roll/pitch),
        # and a 2 deg tilt puts flat ground 17 cm off at 5 m, i.e. "rock" or "crater".
        self.lio_q = None
        self.create_subscription(Odometry, "/lio_odom", lambda m: setattr(self, "lio_q", m.pose.pose.orientation), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/terrain/grid", latched)
        self.rock_pub = self.create_publisher(PoseArray, "/terrain/rocks", latched)
        self.crater_pub = self.create_publisher(PoseArray, "/terrain/craters", latched)
        self.create_timer(1.0 / g("publish_rate"), self.publish)

    def on_cloud(self, msg):
        try:
            # Body-frame cloud placed with the odom->base_link TF (fused in the sim); TF can lag ~1 s, so use latest
            M = tf_matrix(self.tf.lookup_transform(self.frame, msg.header.frame_id, Time()))
            B = M @ tf_matrix(self.tf.lookup_transform(msg.header.frame_id, self.base, Time()))
            T = M
            if self.lio_q is not None:
                T = M @ tilt(self.lio_q)
        except TransformException as e:
            self.get_logger().warning(f"no transform: {e}", throttle_duration_sec=5.0)
            return
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True).astype(np.float64)
        pts = pts @ T[:3, :3].T + T[:3, 3]
        robot = B[:3, 3]
        ground_z = robot[2] + self.ground_offset
        d = np.hypot(pts[:, 0] - robot[0], pts[:, 1] - robot[1])
        keep = (d < self.max_r) & (pts[:, 2] - ground_z < self.max_h)  # crop ceiling/upper walls only
        pts = pts[keep]
        ij = ((pts[:, :2] - self.origin) / self.res).astype(int)
        ok = (ij >= 0).all(1) & (ij < self.n).all(1)
        hi, lo, gr = classify_scan(ij[ok], pts[ok, 2], self.n, self.rock_h, self.crater_d, self.window)
        # Per-scan evidence, capped so later views can overturn earlier ones
        for acc, f in ((self.high, hi), (self.low, lo), (self.ground, gr)):
            acc[f] = np.minimum(acc[f] + 1, 20)

    def classify(self):
        rock = (self.high >= self.min_hits) & (self.high > self.ground // 2)
        crater = (self.low >= self.min_hits) & (self.low > self.ground // 2)
        return rock, crater

    def publish(self):
        rock, crater = self.classify()
        dig = self.dig_zone_mask()
        crater = crater & ~dig
        # Hard margins go around rock-sized blobs only. Walls, ducts and the column are long/large blobs:
        # they stay lethal and get the costmap's soft inflation, but no margin, so a robot placed close
        # to a wall in the starting zone isn't boxed in by a margin that overlaps its own footprint.
        n, labels, stats, _ = cv2.connectedComponentsWithStats(rock.astype(np.uint8), connectivity=8)
        big = np.zeros(n, bool)
        big[1:] = np.sqrt(stats[1:, cv2.CC_STAT_AREA] / math.pi) * self.res > self.max_rock_radius
        # The hard margins are for the obstacle-zone contact penalty. In the dig zones a rock-sized blob (often our
        # own dump pile) stays lethal, but a margin there blocked the next dump slot beside the last pile.
        small_rock = rock & ~big[labels] & ~dig
        lethal = cv2.dilate(small_rock.astype(np.uint8), self.kernels["rock_margin"]) | \
            cv2.dilate(crater.astype(np.uint8), self.kernels["crater_margin"]) | rock.astype(np.uint8)
        seen = (self.high | self.low | self.ground) > 0
        data = np.where(lethal > 0, 100, np.where(seen, 0, self.unknown)).astype(np.int8)

        msg = OccupancyGrid()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.res
        msg.info.width = msg.info.height = self.n
        msg.info.origin.position.x = msg.info.origin.position.y = self.origin
        msg.info.origin.orientation.w = 1.0
        msg.data = array("b", data.T.tobytes())  # row-major in y; a list of 100k ints cost ~0.1 s per publish
        self.grid_pub.publish(msg)
        for pub, mask in ((self.rock_pub, rock), (self.crater_pub, crater)):
            pub.publish(self.centres(mask, msg.header))

    def dig_zone_mask(self):
        """Grid cells inside the excavation/construction zones; none if the arena frame isn't known yet."""
        try:
            t = self.tf.lookup_transform(self.arena, self.frame, Time()).transform
        except TransformException:
            return np.zeros((self.n, self.n), bool)
        c = self.origin + (np.arange(self.n) + 0.5) * self.res
        X, Y = np.meshgrid(c, c, indexing="ij")   # grid index order [i (x), j (y)]
        yaw = 2 * math.atan2(t.rotation.z, t.rotation.w)
        ax = t.translation.x + math.cos(yaw) * X - math.sin(yaw) * Y
        ay = t.translation.y + math.sin(yaw) * X + math.cos(yaw) * Y
        return np.any([(ax >= x0) & (ax <= x1) & (ay >= y0) & (ay <= y1) for x0, y0, x1, y1 in self.dig_zones], axis=0)

    def centres(self, mask, header):
        """One pose per connected blob: centre, with the blob's equivalent radius in position.z."""
        out = PoseArray(header=header)
        n, _, stats, cent = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
        for k in range(1, n):
            p = Pose()
            p.position.x = self.origin + (cent[k][1] + 0.5) * self.res  # cv2 (col, row) = (j, i)
            p.position.y = self.origin + (cent[k][0] + 0.5) * self.res
            p.position.z = math.sqrt(stats[k, cv2.CC_STAT_AREA] / math.pi) * self.res
            p.orientation.w = 1.0
            out.poses.append(p)
        return out


def main():
    rclpy.init()
    rclpy.spin(TerrainMapper())


if __name__ == "__main__":
    main()
