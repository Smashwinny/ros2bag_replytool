#!/usr/bin/env python3
"""Binary protocol shared by the ordinal rosbag reader and ESKF ingress."""

from __future__ import annotations

from dataclasses import dataclass
import bisect
import struct


MAGIC = b"ERB1"
HEADER = struct.Struct(">4sQQqB")
TOPIC_IDS = {
    "/imu": 1,
    "/gps_data": 2,
    "/odometry": 3,
    "/cmd_vel": 4,
}


@dataclass(frozen=True)
class BagRecord:
    ordinal: int
    topic: str
    timestamp_ns: int
    payload: bytes


def pack_ingress(epoch: int, record: BagRecord) -> bytes:
    if epoch < 0 or record.ordinal < 0:
        raise ValueError("epoch and ordinal must be non-negative")
    topic_id = TOPIC_IDS.get(record.topic)
    if topic_id is None:
        raise ValueError(f"topic is not an ESKF ingress: {record.topic}")
    return HEADER.pack(MAGIC, epoch, record.ordinal, record.timestamp_ns,
                       topic_id) + record.payload


def unpack_header(frame: bytes):
    if len(frame) <= HEADER.size:
        raise ValueError("frame has no payload")
    magic, epoch, ordinal, timestamp_ns, topic_id = HEADER.unpack_from(frame)
    if magic != MAGIC:
        raise ValueError("invalid ingress magic")
    return epoch, ordinal, timestamp_ns, topic_id, frame[HEADER.size:]


def ordinal_at_or_after(records: list[BagRecord], timestamp_ns: int) -> int:
    timestamps = [record.timestamp_ns for record in records]
    return bisect.bisect_left(timestamps, timestamp_ns)


def ordinal_at_or_before(records: list[BagRecord], timestamp_ns: int) -> int:
    timestamps = [record.timestamp_ns for record in records]
    return bisect.bisect_right(timestamps, timestamp_ns) - 1


def completed_stop_should_release(stop_after_ordinal: int | None,
                                  cursor: int) -> bool:
    """Return whether resume follows a completed bounded replay.

    A bounded replay advances ``cursor`` past its inclusive stop ordinal before
    pausing.  Keeping that old stop point would make the next resume publish
    exactly one record and pause again.
    """
    return (stop_after_ordinal is not None and
            cursor > stop_after_ordinal)
