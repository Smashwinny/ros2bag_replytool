#!/usr/bin/env python3
import unittest

from ordinal_protocol import (BagRecord, HEADER, ordinal_at_or_after,
                              ordinal_at_or_before,
                              completed_stop_should_release, pack_ingress,
                              unpack_header)


class OrdinalProtocolTest(unittest.TestCase):
    def test_frame_preserves_epoch_ordinal_timestamp_and_payload(self):
        record = BagRecord(42, "/imu", 123456789, b"cdr-payload")
        frame = pack_ingress(7, record)
        self.assertEqual(len(frame), HEADER.size + len(record.payload))
        self.assertEqual(unpack_header(frame),
                         (7, 42, 123456789, 1, b"cdr-payload"))

    def test_global_ordinal_seek_keeps_equal_timestamp_order(self):
        records = [
            BagRecord(0, "/imu", 10, b"a"),
            BagRecord(1, "/gps_data", 10, b"b"),
            BagRecord(2, "/odometry", 11, b"c"),
        ]
        self.assertEqual(ordinal_at_or_after(records, 10), 0)
        self.assertEqual([r.ordinal for r in records[:2]], [0, 1])
        self.assertEqual(ordinal_at_or_after(records, 11), 2)
        self.assertEqual(ordinal_at_or_before(records, 10), 1)

    def test_non_ingress_topic_is_rejected(self):
        with self.assertRaises(ValueError):
            pack_ingress(0, BagRecord(0, "/tf", 1, b"x"))

    def test_resume_releases_only_a_completed_bounded_replay(self):
        self.assertTrue(completed_stop_should_release(42, 43))
        self.assertFalse(completed_stop_should_release(42, 42))
        self.assertFalse(completed_stop_should_release(42, 10))
        self.assertFalse(completed_stop_should_release(None, 43))


if __name__ == "__main__":
    unittest.main()
