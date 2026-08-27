#!/usr/bin/env python3
"""Build a persistent trajectory and full-process-log time index."""

import argparse
import json
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message
import rosbag2_py

from persistent_replay_cache import (
    cache_fingerprint, create_database, index_process_log,
    insert_trajectory, reset_build_database, write_manifest)


def legacy_poses(bag_dir: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    while reader.has_next():
        topic, payload, timestamp_ns = reader.read_next()
        if topic != "/fusion_location":
            continue
        message = deserialize_message(payload, PoseStamped)
        stamp_ns = (message.header.stamp.sec * 1_000_000_000 +
                    message.header.stamp.nanosec)
        pose = message.pose
        yield (stamp_ns, pose.position.x, pose.position.y, 0.0,
               pose.orientation.w, pose.orientation.x,
               pose.orientation.y, pose.orientation.z)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--target-yaml", type=Path, required=True)
    parser.add_argument("--build-state", type=Path, required=True)
    parser.add_argument("--process-log", type=Path, required=True)
    parser.add_argument("--cache-root", type=Path, required=True)
    args = parser.parse_args()
    fingerprint, identity = cache_fingerprint(
        args.bag, args.target_yaml, args.build_state)
    cache_dir = args.cache_root / fingerprint
    cache_dir.mkdir(parents=True, exist_ok=True)
    temporary = cache_dir / "index.sqlite3.building"
    reset_build_database(temporary)
    connection = create_database(temporary)
    try:
        indexed, eskf_poses = index_process_log(connection, args.process_log)
        legacy_count = insert_trajectory(connection, "legacy",
                                         legacy_poses(args.bag))
        connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
    finally:
        connection.close()
    database = cache_dir / "index.sqlite3"
    temporary.replace(database)
    manifest = write_manifest(
        cache_dir, fingerprint, identity, args.process_log,
        {"process_records": indexed, "eskf_poses": eskf_poses,
         "legacy_poses": legacy_count})
    print(json.dumps({"fingerprint": fingerprint, "manifest": str(manifest)}))


if __name__ == "__main__":
    main()
