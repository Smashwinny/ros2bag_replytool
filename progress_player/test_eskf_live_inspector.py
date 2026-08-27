#!/usr/bin/env python3
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("eskf_live_inspector.py")
SPEC = importlib.util.spec_from_file_location("eskf_live_inspector", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class JsonlFollowerTest(unittest.TestCase):
    def test_incremental_partial_and_truncate(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parents[1] / "tmp") as directory:
            path = Path(directory) / "process.jsonl"
            path.write_text('{"sequence":1}\n{"sequence":', encoding="utf-8")
            follower = MODULE.JsonlFollower(str(path))
            self.assertEqual(follower.read_latest()["sequence"], 1)
            with path.open("a", encoding="utf-8") as stream:
                stream.write("2}\n")
            self.assertEqual(follower.read_latest()["sequence"], 2)
            path.write_text('{"sequence":3}\n', encoding="utf-8")
            self.assertEqual(follower.read_latest()["sequence"], 3)

    def test_invalid_json_is_skipped(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parents[1] / "tmp") as directory:
            path = Path(directory) / "process.jsonl"
            path.write_text('bad\n{"sequence":4}\n', encoding="utf-8")
            self.assertEqual(MODULE.JsonlFollower(str(path)).read_latest()["sequence"], 4)


class SummaryTest(unittest.TestCase):
    def test_update_summary_has_live_quantities(self):
        record = {
            "event": "measurement_update", "sequence": 9, "filter_time_s": 5.25,
            "position_wb": [1.0, 2.0, 3.0], "velocity_wb": [0.1, 0.2, 0.0],
            "orientation_wb_wxyz": [1.0, 0.0, 0.0, 0.0],
            "gyro_bias_b": [0.01, 0.02, 0.03], "accel_bias_b": [0.1, 0.2, 0.3],
            "covariance_diag": [0.01, 0.04, 0.09] + [1.0] * 12,
            "pending_gps": 1, "pending_odom": 2,
            "oosm_replay_success": 3, "oosm_replay_failure": 0,
            "computation_trace": {
                "stage": "accepted", "dt": 0.01, "nis": 2.5,
                "position_gain_scale": 1.0, "velocity_gain_scale": 0.5,
                "prior_state_pvqwxyz_bg_ba": [0.0] * 16,
                "candidate_state_pvqwxyz_bg_ba": [1.0] * 16,
                "F": [1.0] * 225, "G": [1.0] * 180, "Qc": [1.0] * 144,
                "Phi": [1.0] * 225, "Qd": [1.0] * 225,
                "H": [1.0] * 45, "R": [1.0] * 9, "S": [1.0] * 9,
                "K": [1.0] * 45, "residual": [1.0] * 3,
                "delta_x": [1.0] * 15, "joseph_update": [1.0] * 225,
                "reset": [1.0] * 225,
            },
        }
        title, compact, detail = MODULE.process_summary(record)
        self.assertIn("#9", title)
        self.assertIn("NIS 2.500", compact)
        self.assertIn("σp [0.100, 0.200, 0.300]", compact)
        self.assertIn("F n=225", detail)
        self.assertIn("residual n=3", detail)
        self.assertIn("bg [0.010, 0.020, 0.030]", detail)

    def test_non_computation_event_does_not_reuse_trace(self):
        _, compact, detail = MODULE.process_summary({"event": "full_checkpoint_restored"})
        self.assertIn("NIS --", compact)
        self.assertIn("没有 computation_trace", detail)


class StorageControlTest(unittest.TestCase):
    def test_format_bytes(self):
        self.assertEqual(MODULE.format_bytes(0), "0.0 B")
        self.assertEqual(MODULE.format_bytes(1536), "1.5 KiB")
        self.assertEqual(MODULE.format_bytes(2 * 1024**3), "2.0 GiB")

    def test_clean_preserves_active_run(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parents[1] / "tmp") as directory:
            root = Path(directory) / "runs"
            active = root / "active" / "eskf_process.jsonl"
            old = root / "old" / "eskf_process.jsonl"
            active.parent.mkdir(parents=True)
            old.parent.mkdir(parents=True)
            active.write_bytes(b"active")
            old.write_bytes(b"old-data")
            removed, failures = MODULE.clean_completed_runs(root, active)
            self.assertEqual((removed, failures), (8, 0))
            self.assertTrue(active.exists())
            self.assertFalse(old.parent.exists())
            self.assertEqual(MODULE.directory_size(root), 6)

    def test_clean_rejects_active_outside_root(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parents[1] / "tmp") as directory:
            base = Path(directory)
            root = base / "runs"
            root.mkdir()
            with self.assertRaises(ValueError):
                MODULE.clean_completed_runs(root, base / "outside.jsonl")

    def test_clean_preserves_other_active_domains(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parents[1] / "tmp") as directory:
            root = Path(directory) / "runs"
            own_log = root / "domain180" / "eskf_process.jsonl"
            other_log = root / "domain181" / "eskf_process.jsonl"
            old_log = root / "old" / "eskf_process.jsonl"
            for path in (own_log, other_log, old_log):
                path.parent.mkdir(parents=True)
                path.write_bytes(b"data")
            (other_log.parent / ".active").touch()
            MODULE.clean_completed_runs(root, own_log)
            self.assertTrue(own_log.exists())
            self.assertTrue(other_log.exists())
            self.assertFalse(old_log.parent.exists())

    def test_clean_preserves_run_referenced_by_persistent_cache(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parents[1] / "tmp") as directory:
            base = Path(directory)
            root, cache = base / "runs", base / "cache" / "fingerprint"
            active = root / "active" / "eskf_process.jsonl"
            cached = root / "cached" / "eskf_process.jsonl"
            old = root / "old" / "eskf_process.jsonl"
            for path in (active, cached, old):
                path.parent.mkdir(parents=True)
                path.write_bytes(b"data")
            cache.mkdir(parents=True)
            (cache / "manifest.json").write_text(
                json.dumps({"process_log": str(cached)}), encoding="utf-8")
            MODULE.clean_completed_runs(root, active, base / "cache")
            self.assertTrue(active.exists())
            self.assertTrue(cached.exists())
            self.assertFalse(old.parent.exists())


if __name__ == "__main__":
    unittest.main()
