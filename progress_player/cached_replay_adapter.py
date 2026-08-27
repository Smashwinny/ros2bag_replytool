#!/usr/bin/env python3
"""Publish cached first-pass trajectories and process state by bag time."""

import argparse
import bisect
import json
import math
from pathlib import Path as FsPath
import sqlite3

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from persistent_replay_cache import nearest_process_record


class CachedReplayAdapter(Node):
    def __init__(self, database: FsPath, process_log: FsPath,
                 target_yaml: FsPath):
        super().__init__("eskf_cached_replay_adapter")
        self.database, self.process_log = database, process_log
        self.trajectories = self.load_trajectories(database)
        self.stamps = {key: [item[0] for item in value]
                       for key, value in self.trajectories.items()}
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.path_pubs = {
            "legacy": self.create_publisher(Path, "/compare/legacy_path", qos),
            "eskf": self.create_publisher(Path, "/compare/eskf_path", qos),
            "eskf_provisional": self.create_publisher(
                Path, "/compare/eskf_provisional_path", qos),
        }
        self.pose_pub = self.create_publisher(
            PoseStamped, "/eskf/fusion_location_xy", qos)
        self.process_pub = self.create_publisher(
            String, "/rosbag_progress/cached_process_record", qos)
        marker_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_pub = self.create_publisher(
            MarkerArray, "/compare/target_points", marker_qos)
        self.targets = self.load_targets(target_yaml)
        clock_qos = QoSProfile(depth=10,
                               reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, "/clock", self.on_clock, clock_qos)
        self.create_subscription(Float64, "/rosbag_progress/visual_seek",
                                 self.on_seek, 10)
        self.cutoff_ns = min((items[0][0] for items in self.trajectories.values()
                              if items), default=0)
        self.last_published_ns = -1
        self.create_timer(0.2, self.publish_view)
        self.create_timer(1.0, self.publish_targets)

    @staticmethod
    def load_trajectories(database):
        result = {"legacy": [], "eskf": [], "eskf_provisional": []}
        with sqlite3.connect(database) as connection:
            for row in connection.execute(
                    "SELECT kind,stamp_ns,x,y,z,qw,qx,qy,qz "
                    "FROM trajectory ORDER BY kind,stamp_ns"):
                result.setdefault(row[0], []).append(row[1:])
        return result

    def on_clock(self, message):
        self.cutoff_ns = (message.clock.sec * 1_000_000_000 +
                          message.clock.nanosec)

    def on_seek(self, message):
        if math.isfinite(message.data) and message.data >= 0:
            self.cutoff_ns = int(round(message.data * 1_000_000_000))
            self.last_published_ns = -1

    @staticmethod
    def make_pose(row):
        stamp_ns, x, y, z, qw, qx, qy, qz = row
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.pose.position.x, message.pose.position.y = x, y
        message.pose.position.z = 0.0
        (message.pose.orientation.w, message.pose.orientation.x,
         message.pose.orientation.y, message.pose.orientation.z) = qw, qx, qy, qz
        return message

    def publish_view(self):
        if self.cutoff_ns == self.last_published_ns:
            return
        for kind, publisher in self.path_pubs.items():
            count = bisect.bisect_right(self.stamps.get(kind, []), self.cutoff_ns)
            path = Path()
            path.header.frame_id = "map"
            path.poses = [self.make_pose(row)
                          for row in self.trajectories.get(kind, [])[:count]]
            if path.poses:
                path.header.stamp = path.poses[-1].header.stamp
            publisher.publish(path)
            if kind == "eskf" and path.poses:
                self.pose_pub.publish(path.poses[-1])
        record = nearest_process_record(
            self.database, self.process_log, self.cutoff_ns)
        if record is not None:
            message = String()
            message.data = json.dumps(record, separators=(",", ":"),
                                      ensure_ascii=False)
            self.process_pub.publish(message)
        self.last_published_ns = self.cutoff_ns

    @staticmethod
    def load_targets(path):
        result = MarkerArray()
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        marker_id = 0
        for target_path in data.get("paths", []):
            line = Marker()
            line.header.frame_id, line.ns, line.id = "map", "target_path", marker_id
            marker_id += 1
            line.type, line.action, line.scale.x = Marker.LINE_STRIP, Marker.ADD, 0.035
            line.color.r, line.color.g, line.color.a = 1.0, 0.75, 0.9
            for point_data in target_path.get("path", []):
                point = PoseStamped().pose.position
                point.x, point.y = float(point_data["x"]), float(point_data["y"])
                line.points.append(point)
            result.markers.append(line)
        return result

    def publish_targets(self):
        if self.targets.markers:
            now = self.get_clock().now().to_msg()
            for marker in self.targets.markers:
                marker.header.stamp = now
            self.target_pub.publish(self.targets)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--database", type=FsPath, required=True)
    parser.add_argument("--process-log", type=FsPath, required=True)
    parser.add_argument("--target-yaml", type=FsPath, required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = CachedReplayAdapter(args.database, args.process_log, args.target_yaml)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
