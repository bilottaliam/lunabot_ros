#!/usr/bin/env python3
"""MCC-laptop half of the MCC link: the robot's foxglove_bridge <-> this laptop's local ROS graph.

Robot and laptop both run with ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST, so no DDS crosses Wi-Fi.
This relay is the only connection: a foxglove WebSocket client (ws-protocol v1) that
  - republishes each robot topic locally (same name and type) but subscribes to it on the robot only
    while something on the laptop subscribes, so cameras and other heavy topics are on demand;
  - forwards the local topics in `to_robot` (teleop, inhibit, mission commands) to the robot;
  - serves /launch_system and /stop_system locally by calling them on the robot.
The bridge's allow-lists (lunabot_config/params/mcc/foxglove_bridge.yaml) decide what may cross.
Message payloads are passed through as CDR bytes; nothing is decoded on the way.
"""
import json
import struct
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from websockets.sync.client import connect

TO_ROBOT = {
    "/cmd_vel": "geometry_msgs/msg/Twist",
    "/mission/inhibit": "std_msgs/msg/Empty",
    "/mission/command": "std_msgs/msg/String",
    "/emergency_stop": "std_msgs/msg/Bool",
    "/mode_switch": "std_msgs/msg/Bool",
    "/position_controller/commands": "std_msgs/msg/Float64MultiArray",
    "/camera_controller/commands": "std_msgs/msg/Float64MultiArray",
    "/excavation_angle": "std_msgs/msg/Float64",
}
SERVICES = {"/launch_system": "lunabot_msgs/srv/LaunchSystem", "/stop_system": "lunabot_msgs/srv/StopSystem"}


def get_service(name):
    pkg, _, srv = name.split("/")
    return getattr(__import__(f"{pkg}.srv", fromlist=[srv]), srv)


class MccRelay(Node):
    def __init__(self):
        super().__init__("mcc_relay")
        self.declare_parameter("robot_url", "ws://192.168.0.250:8765")
        self.url = self.get_parameter("robot_url").value
        self.ws = None
        self.lock = threading.Lock()
        self.channels = {}      # robot channel id -> (topic, local publisher)
        self.subs = {}          # robot channel id -> our subscription id while subscribed
        self.by_sub = {}        # subscription id -> local publisher
        self.services = {}      # service name -> robot service id
        self.pending = {}       # call id -> [event, payload]
        self.next_id = 1
        self.rx_bytes = 0
        group = ReentrantCallbackGroup()
        for i, (topic, typ) in enumerate(TO_ROBOT.items(), start=1):
            self.create_subscription(get_message(typ), topic, lambda b, cid=i: self.send_client(cid, b), 10, raw=True)
        for name, typ in SERVICES.items():
            self.create_service(get_service(typ), name, lambda req, resp, n=name: self.call(n, req, resp), callback_group=group)
        self.create_timer(1.0, self.sync_subscriptions)
        threading.Thread(target=self.run_ws, daemon=True).start()

    # ------------------------------------------------------------- websocket
    def run_ws(self):
        while rclpy.ok():
            try:
                with connect(self.url, subprotocols=["foxglove.websocket.v1"], open_timeout=5, max_size=None) as ws:
                    self.get_logger().info(f"connected to {self.url}")
                    self.ws = ws
                    self.send_json({"op": "advertise", "channels": [
                        {"id": i, "topic": t, "encoding": "cdr", "schemaName": typ}
                        for i, (t, typ) in enumerate(TO_ROBOT.items(), start=1)]})
                    for msg in ws:
                        self.on_message(msg)
            except Exception as e:  # link loss: the robot keeps running on its own; keep retrying
                self.get_logger().warning(f"MCC link down ({e}); retrying", throttle_duration_sec=10.0)
            with self.lock:
                self.ws = None
                self.subs.clear()
                self.by_sub.clear()
            time.sleep(1.0)

    def send_json(self, obj):
        with self.lock:
            if self.ws:
                self.ws.send(json.dumps(obj))

    def send_bin(self, data):
        with self.lock:
            if self.ws:
                self.ws.send(data)

    def on_message(self, msg):
        if isinstance(msg, str):
            m = json.loads(msg)
            if m["op"] == "advertise":
                for ch in m["channels"]:
                    if ch["encoding"] != "cdr" or ch["topic"] in TO_ROBOT:
                        continue
                    pub = self.create_publisher(get_message(ch["schemaName"]), ch["topic"], 10)
                    self.channels[ch["id"]] = (ch["topic"], pub)
            elif m["op"] == "unadvertise":
                for cid in m["channelIds"]:
                    self.channels.pop(cid, None)
            elif m["op"] == "advertiseServices":
                for s in m["services"]:
                    self.services[s["name"]] = s["id"]
            return
        self.rx_bytes += len(msg)
        op = msg[0]
        if op == 0x01:  # message data: op, u32 subscription id, u64 timestamp, payload
            sub_id = struct.unpack_from("<I", msg, 1)[0]
            pub = self.by_sub.get(sub_id)
            if pub:
                pub.publish(bytes(msg[13:]))
        elif op == 0x03:  # service call response: op, u32 service id, u32 call id, u32 enc len, enc, payload
            _, call_id, n = struct.unpack_from("<III", msg, 1)
            waiter = self.pending.get(call_id)
            if waiter:
                waiter[1] = bytes(msg[13 + n:])
                waiter[0].set()

    def sync_subscriptions(self):
        """Subscribe on the robot exactly to the topics someone on the laptop is listening to."""
        if not self.ws:
            return
        want = {cid for cid, (_, pub) in self.channels.items() if pub.get_subscription_count() > 0}
        add, drop = want - set(self.subs), set(self.subs) - want
        if add:
            new = []
            for cid in add:
                sid, self.next_id = self.next_id, self.next_id + 1
                self.subs[cid], self.by_sub[sid] = sid, self.channels[cid][1]
                new.append({"id": sid, "channelId": cid})
            self.send_json({"op": "subscribe", "subscriptions": new})
        if drop:
            ids = [self.subs.pop(cid) for cid in drop]
            for sid in ids:
                self.by_sub.pop(sid, None)
            self.send_json({"op": "unsubscribe", "subscriptionIds": ids})

    def send_client(self, channel_id, payload):
        self.send_bin(struct.pack("<BI", 0x01, channel_id) + payload)

    def call(self, name, req, resp):
        sid = self.services.get(name)
        if sid is None or not self.ws:
            resp.success, resp.message = False, "robot not connected"
            return resp
        call_id, self.next_id = self.next_id, self.next_id + 1
        waiter = self.pending[call_id] = [threading.Event(), None]
        self.send_bin(struct.pack("<BIII", 0x02, sid, call_id, 3) + b"cdr" + serialize_message(req))
        ok = waiter[0].wait(30.0)
        self.pending.pop(call_id, None)
        if not ok:
            resp.success, resp.message = False, "robot did not answer"
            return resp
        return deserialize_message(waiter[1], type(resp))


def main():
    rclpy.init()
    node = MccRelay()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    ex.spin()


if __name__ == "__main__":
    main()
