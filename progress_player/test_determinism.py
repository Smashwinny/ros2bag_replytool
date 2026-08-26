import unittest
from determinism import compare_three, first_differences, snapshot_hashes


class DeterminismTest(unittest.TestCase):
    def snapshot(self):
        return {
            "nominal_state_pvqwxyz_bg_ba": [float(i) for i in range(16)],
            "covariance_row_major": [float(i) / 10 for i in range(225)],
            "node_state_summary": {"sequence": 3, "ready": True,
                                   "trace": {"dt": 0.01}},
            "full_checkpoint_state": {
                "core": "1" * 64,
                "queue": "2" * 64,
                **{f"field_{index}": "0" * 64 for index in range(102)},
            },
            "output_trajectory": [[7, 1.25, 1, 2, 3, 1, 0, 0, 0]],
        }

    def test_identical_snapshots_pass(self):
        hashes = snapshot_hashes(self.snapshot())
        self.assertTrue(compare_three([hashes, hashes.copy(), hashes.copy()])["bit_exact"])

    def test_covariance_change_is_isolated(self):
        original = snapshot_hashes(self.snapshot())
        changed = self.snapshot()
        changed["covariance_row_major"][17] += 1
        changed_hash = snapshot_hashes(changed)
        self.assertEqual(original["state_sha256"], changed_hash["state_sha256"])
        self.assertNotEqual(original["covariance_sha256"], changed_hash["covariance_sha256"])

    def test_negative_zero_is_canonical(self):
        left, right = self.snapshot(), self.snapshot()
        right["nominal_state_pvqwxyz_bg_ba"][0] = -0.0
        self.assertEqual(snapshot_hashes(left)["state_summary_sha256"],
                         snapshot_hashes(right)["state_summary_sha256"])

    def test_old_node_summary_is_diagnostic_only(self):
        left, right = self.snapshot(), self.snapshot()
        right["node_state_summary"]["trace"]["dt"] = 0.02
        self.assertEqual(snapshot_hashes(left)["state_sha256"],
                         snapshot_hashes(right)["state_sha256"])
        self.assertNotEqual(snapshot_hashes(left)["state_summary_sha256"],
                            snapshot_hashes(right)["state_summary_sha256"])

    def test_full_checkpoint_field_is_part_of_state_hash(self):
        left, right = self.snapshot(), self.snapshot()
        right["full_checkpoint_state"]["queue"] = "3" * 64
        self.assertNotEqual(snapshot_hashes(left)["state_sha256"],
                            snapshot_hashes(right)["state_sha256"])
        self.assertEqual(first_differences([left, right, self.snapshot()])["state"],
                         "$.state.queue")

    def test_invalid_field_digest_is_rejected(self):
        snapshot = self.snapshot()
        snapshot["full_checkpoint_state"]["queue"] = "not-a-sha256"
        with self.assertRaisesRegex(ValueError, "field digest"):
            snapshot_hashes(snapshot)

    def test_partial_manifest_is_rejected(self):
        snapshot = self.snapshot()
        del snapshot["full_checkpoint_state"]["queue"]
        with self.assertRaisesRegex(ValueError, "manifest"):
            snapshot_hashes(snapshot)

    def test_first_different_matrix_index_is_reported(self):
        first, second, third = self.snapshot(), self.snapshot(), self.snapshot()
        second["covariance_row_major"][17] += 1
        self.assertEqual(first_differences([first, second, third])["covariance"],
                         "$.covariance[17]")


if __name__ == "__main__":
    unittest.main()
