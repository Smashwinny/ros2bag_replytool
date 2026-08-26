#!/usr/bin/env python3
import unittest

from replay_history import ImmutableTimeline, ReplayEpochClock


class ReplayHistoryTest(unittest.TestCase):
    def test_backward_view_does_not_destroy_future(self):
        timeline = ImmutableTimeline(lambda item: item[0], lambda item: item)
        timeline.replace_for_test([(1, "a"), (3, "b"), (5, "c")])
        self.assertEqual(timeline.view(3), [(1, "a"), (3, "b")])
        self.assertEqual(timeline.view(5), [(1, "a"), (3, "b"), (5, "c")])
        self.assertEqual(timeline.size, 3)

    def test_sealed_history_rejects_replay_append(self):
        timeline = ImmutableTimeline(lambda item: item[0], lambda item: item)
        self.assertTrue(timeline.append((1, "a")))
        timeline.seal()
        self.assertFalse(timeline.append((2, "b")))
        self.assertEqual(timeline.view(None), [(1, "a")])

    def test_stable_key_deduplicates_old_epoch_delivery(self):
        timeline = ImmutableTimeline(lambda item: item[0], lambda item: item[1])
        self.assertTrue(timeline.append((1, "ordinal-7")))
        self.assertFalse(timeline.append((1, "ordinal-7")))

    def test_epoch_is_strictly_monotonic(self):
        clock = ReplayEpochClock()
        first = clock.next(5, False)
        second = clock.next(2, True)
        self.assertEqual((first.epoch, second.epoch), (1, 2))
        self.assertTrue(second.first_pass_complete)


if __name__ == "__main__":
    unittest.main()
