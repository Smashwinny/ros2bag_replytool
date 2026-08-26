#!/usr/bin/env python3
"""Deterministic rosbag reader with global ordinals and ACK-gated ingress."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosbag2_interfaces.srv import Pause, Resume, Seek, SetRate
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String, UInt8MultiArray

from ordinal_protocol import BagRecord, TOPIC_IDS, ordinal_at_or_after, pack_ingress


def load_records(bag_dir: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    records = []
    ordinal = 0
    while reader.has_next():
        topic, payload, timestamp_ns = reader.read_next()
        records.append(BagRecord(
            ordinal=ordinal, topic=topic, timestamp_ns=int(timestamp_ns),
            payload=bytes(payload)))
        ordinal += 1
    if not records:
        raise RuntimeError("bag contains no records")
    return records, topic_types


class OrdinalReplayReader(Node):
    def __init__(self, records, topic_types):
        super().__init__("rosbag2_player")
        self.records = records
        self.topic_types = topic_types
        self.cursor = 0
        self.epoch = 0
        self.paused = True
        self.rate = 1.0
        self.pending_ack = None
        self.pending_deadline = 0.0
        self.last_timestamp_ns = None
        self.last_wall = None
        self.stopping = False
        self.condition = threading.Condition()
        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.clock_pub = self.create_publisher(Clock, "/clock", clock_qos)
        self.ingress_pub = self.create_publisher(
            UInt8MultiArray, "/eskf/replay_ingress",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.status_pub = self.create_publisher(
            String, "/rosbag_progress/ordinal_reader_status", 10)
        self.ordinal_seek_result_pub = self.create_publisher(
            String, "/rosbag_progress/ordinal_seek_result", 10)
        self.display_publishers = {}
        self.display_types = {}
        for source, target in (("/fusion_location", "/legacy/fusion_location"),
                               ("/motor_speed", "/motor_speed")):
            if source in topic_types:
                message_type = get_message(topic_types[source])
                self.display_types[source] = message_type
                self.display_publishers[source] = self.create_publisher(
                    message_type, target, 10)
        self.create_service(Pause, "/rosbag2_player/pause", self.on_pause)
        self.create_service(Resume, "/rosbag2_player/resume", self.on_resume)
        self.create_service(Seek, "/rosbag2_player/seek", self.on_seek)
        self.create_service(SetRate, "/rosbag2_player/set_rate", self.on_rate)
        self.create_subscription(
            String, "/rosbag_progress/replay_epoch", self.on_epoch, 10)
        self.create_subscription(
            String, "/rosbag_progress/ordinal_seek_request",
            self.on_ordinal_seek, 10)
        self.create_subscription(
            String, "/eskf/replay_ingress_ack", self.on_ack,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def reset_timing_locked(self):
        self.last_timestamp_ns = None
        self.last_wall = None

    def cancel_pending_locked(self):
        self.pending_ack = None
        self.pending_deadline = 0.0

    def on_pause(self, _request, response):
        with self.condition:
            self.paused = True
            self.condition.notify_all()
        return response

    def on_resume(self, _request, response):
        with self.condition:
            self.paused = False
            self.reset_timing_locked()
            self.condition.notify_all()
        return response

    def on_seek(self, request, response):
        target_ns = request.time.sec * 1_000_000_000 + request.time.nanosec
        with self.condition:
            self.cursor = ordinal_at_or_after(self.records, target_ns)
            response.success = self.cursor < len(self.records)
            self.cancel_pending_locked()
            self.reset_timing_locked()
            self.condition.notify_all()
        return response

    def on_rate(self, request, response):
        with self.condition:
            response.success = 0.01 <= request.rate <= 100.0
            if response.success:
                self.rate = float(request.rate)
                self.reset_timing_locked()
                self.condition.notify_all()
        return response

    def on_epoch(self, message):
        try:
            payload = json.loads(message.data)
            epoch = int(payload["epoch"])
            if payload.get("schema") != "rosbag_replay_epoch/v1":
                return
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        with self.condition:
            if epoch <= self.epoch:
                return
            self.epoch = epoch
            self.cancel_pending_locked()
            self.reset_timing_locked()
            self.condition.notify_all()

    def on_ordinal_seek(self, message):
        success = False
        reason = "invalid_request"
        requested = -1
        try:
            payload = json.loads(message.data)
            if payload.get("schema") != "rosbag_ordinal_seek/v1":
                raise ValueError("schema")
            epoch = int(payload["epoch"])
            requested = int(payload["bag_ordinal"])
            with self.condition:
                if epoch != self.epoch:
                    reason = "wrong_epoch"
                elif 0 <= requested <= len(self.records):
                    self.cursor = requested
                    self.cancel_pending_locked()
                    self.reset_timing_locked()
                    self.condition.notify_all()
                    success, reason = True, "ok"
                else:
                    reason = "ordinal_out_of_range"
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            pass
        result = String()
        result.data = json.dumps({
            "schema": "rosbag_ordinal_seek_result/v1",
            "epoch": self.epoch, "bag_ordinal": requested,
            "success": success, "reason": reason,
        }, separators=(",", ":"), sort_keys=True)
        self.ordinal_seek_result_pub.publish(result)

    def on_ack(self, message):
        try:
            payload = json.loads(message.data)
            key = (int(payload["epoch"]), int(payload["bag_ordinal"]))
            accepted = payload["accepted"] is True
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        with self.condition:
            if key != self.pending_ack:
                return
            if accepted:
                self.cursor += 1
                self.pending_ack = None
                self.condition.notify_all()
            else:
                self.paused = True
                self.pending_ack = None
                self.publish_status("ack_rejected", key[1])

    def publish_status(self, state, ordinal):
        message = String()
        message.data = json.dumps({
            "schema": "rosbag_ordinal_reader_status/v1",
            "state": state, "epoch": self.epoch, "bag_ordinal": ordinal,
        }, separators=(",", ":"), sort_keys=True)
        self.status_pub.publish(message)

    def publish_clock(self, timestamp_ns):
        message = Clock()
        message.clock.sec = timestamp_ns // 1_000_000_000
        message.clock.nanosec = timestamp_ns % 1_000_000_000
        self.clock_pub.publish(message)

    def wait_pacing(self, timestamp_ns):
        with self.condition:
            if self.last_timestamp_ns is None:
                self.last_timestamp_ns = timestamp_ns
                self.last_wall = time.monotonic()
                return True
            target_wall = self.last_wall + max(
                0.0, (timestamp_ns - self.last_timestamp_ns) / 1e9 / self.rate)
            while not self.stopping and not self.paused:
                remaining = target_wall - time.monotonic()
                if remaining <= 0:
                    self.last_timestamp_ns = timestamp_ns
                    self.last_wall = target_wall
                    return True
                self.condition.wait(timeout=min(remaining, 0.1))
            return False

    def run(self):
        while True:
            with self.condition:
                while (not self.stopping and
                       (self.paused or self.pending_ack is not None or
                        self.cursor >= len(self.records))):
                    if (self.pending_ack is not None and
                            time.monotonic() >= self.pending_deadline):
                        ordinal = self.pending_ack[1]
                        self.pending_ack = None
                        self.paused = True
                        self.publish_status("ack_timeout", ordinal)
                    self.condition.wait(timeout=0.05)
                if self.stopping:
                    return
                record = self.records[self.cursor]
                epoch = self.epoch
            if not self.wait_pacing(record.timestamp_ns):
                continue
            self.publish_clock(record.timestamp_ns)
            if record.topic in TOPIC_IDS:
                frame = UInt8MultiArray()
                frame.data = pack_ingress(epoch, record)
                with self.condition:
                    if self.paused or epoch != self.epoch:
                        continue
                    self.pending_ack = (epoch, record.ordinal)
                    self.pending_deadline = time.monotonic() + 2.0
                self.ingress_pub.publish(frame)
            else:
                publisher = self.display_publishers.get(record.topic)
                if publisher is not None:
                    publisher.publish(deserialize_message(
                        record.payload, self.display_types[record.topic]))
                with self.condition:
                    if self.cursor == record.ordinal:
                        self.cursor += 1

    def destroy_node(self):
        with self.condition:
            self.stopping = True
            self.condition.notify_all()
        self.worker.join(timeout=2.0)
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    records, topic_types = load_records(args.bag.resolve())
    rclpy.init()
    node = OrdinalReplayReader(records, topic_types)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
