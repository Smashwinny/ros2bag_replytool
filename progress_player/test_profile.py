#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import tempfile
import unittest


PLAYER = Path(__file__).resolve().parents[1] / "rosbag_progress_player.py"
SPEC = importlib.util.spec_from_file_location("rosbag_progress_player", PLAYER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ProfileTest(unittest.TestCase):
    def write(self, text):
        temp_dir = Path(__file__).resolve().parents[1] / "tmp" / "tests"
        temp_dir.mkdir(parents=True, exist_ok=True)
        handle = tempfile.NamedTemporaryFile(
            "w", suffix=".yaml", delete=False, dir=temp_dir)
        handle.write(text)
        handle.close()
        self.addCleanup(Path(handle.name).unlink)
        return Path(handle.name)

    def test_valid_profile_is_normalized(self):
        profile = MODULE.read_profile(self.write(
            "managed_command: [bash, run.sh]\nrebuild_rate: 20\nenv: {FLAG: 1}\n"))
        self.assertEqual(profile["managed_command"], ["bash", "run.sh"])
        self.assertEqual(profile["rebuild_rate"], 20.0)
        self.assertEqual(profile["env"], {"FLAG": "1"})

    def test_shell_string_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "字符串数组"):
            MODULE.read_profile(self.write("managed_command: bash run.sh\n"))

    def test_rate_is_bounded(self):
        with self.assertRaisesRegex(ValueError, "1～100"):
            MODULE.read_profile(self.write("rebuild_rate: 1000\n"))

    def test_time_offset_accepts_seconds_and_clock_formats(self):
        self.assertEqual(MODULE.parse_time_offset_ns("12.345"), 12_345_000_000)
        self.assertEqual(MODULE.parse_time_offset_ns("02:03.004"), 123_004_000_000)
        self.assertEqual(
            MODULE.parse_time_offset_ns("01:02:03.000000001"),
            3_723_000_000_001,
        )

    def test_time_offset_rejects_invalid_values(self):
        for value in ("", "-1", "1:60", "00:60:00", "abc", "1:2:3:4", "NaN"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    MODULE.parse_time_offset_ns(value)

if __name__ == "__main__":
    unittest.main()
