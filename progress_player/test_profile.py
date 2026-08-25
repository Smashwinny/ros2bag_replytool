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
            "managed_command: [bash, run.sh]\nrebuild_rate: 20\n"
            "checkpoint_restore: true\n"
            "bag_play_args: [--remap, /odometry:=/bag/odometry]\n"
            "bag_target_map: {/bags/a: /maps/a.yaml}\n"
            "env: {FLAG: 1}\n"))
        self.assertEqual(profile["managed_command"], ["bash", "run.sh"])
        self.assertEqual(profile["rebuild_rate"], 20.0)
        self.assertEqual(profile["env"], {"FLAG": "1"})
        self.assertTrue(profile["checkpoint_restore"])
        self.assertEqual(
            profile["bag_play_args"], ["--remap", "/odometry:=/bag/odometry"])
        self.assertEqual(profile["bag_target_map"], {"/bags/a": "/maps/a.yaml"})

    def test_shell_string_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "字符串数组"):
            MODULE.read_profile(self.write("managed_command: bash run.sh\n"))

    def test_rate_is_bounded(self):
        with self.assertRaisesRegex(ValueError, "1～100"):
            MODULE.read_profile(self.write("rebuild_rate: 1000\n"))

    def test_bag_play_args_reject_non_string_values(self):
        with self.assertRaisesRegex(ValueError, "bag_play_args"):
            MODULE.read_profile(self.write("bag_play_args: [--rate, 2]\n"))

    def test_bag_target_map_rejects_non_string_values(self):
        with self.assertRaisesRegex(ValueError, "bag_target_map"):
            MODULE.read_profile(self.write("bag_target_map: {/bags/a: 2}\n"))

    def test_documented_bag_resolves_exact_yaml(self):
        profile = MODULE.read_profile(
            Path(__file__).with_name("eskf_compare.yaml"))
        bag = Path("/home/hulk/ros2bag/rosbag2_2026_08_07-16_40_39")
        self.assertEqual(
            MODULE.resolve_target_yaml(profile, bag),
            str(bag / "target_pos_all_4.yaml"))

    def test_unregistered_bag_fails_closed(self):
        profile = MODULE.read_profile(
            Path(__file__).with_name("eskf_compare.yaml"))
        with self.assertRaisesRegex(ValueError, "没有登记"):
            MODULE.resolve_target_yaml(profile, Path("/bags/not_registered"))


if __name__ == "__main__":
    unittest.main()
