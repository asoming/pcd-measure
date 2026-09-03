from __future__ import annotations

import unittest

from rosbag_diag.metrics import IntervalAccumulator, median_absolute_deviation, percentile


class PercentileTests(unittest.TestCase):
    def test_empty_and_interpolated_values(self) -> None:
        self.assertIsNone(percentile([], 0.5))
        self.assertEqual(percentile([10], 0.95), 10)
        self.assertAlmostEqual(percentile([0, 10], 0.5), 5.0)

    def test_median_absolute_deviation_ignores_non_finite_values(self) -> None:
        self.assertEqual(median_absolute_deviation([1, 1, 2, float("nan")]), 0.0)


class IntervalAccumulatorTests(unittest.TestCase):
    def test_regular_topic_has_expected_rate_without_jitter(self) -> None:
        accumulator = IntervalAccumulator()
        for index in range(101):
            accumulator.add(1_700_000_000_000_000_000 + index * 10_000_000)

        result = accumulator.summary(gap_factor=3.0)

        self.assertEqual(result["count"], 101)
        self.assertAlmostEqual(result["mean_hz"], 100.0)
        self.assertAlmostEqual(result["median_period_ms"], 10.0)
        self.assertAlmostEqual(result["jitter_cv_percent"], 0.0)
        self.assertEqual(result["estimated_drops"], 0)

    def test_gap_is_counted_as_missing_but_not_scheduler_jitter(self) -> None:
        accumulator = IntervalAccumulator()
        timestamps = [0, 10, 20, 30, 80, 90, 100]
        for timestamp in timestamps:
            accumulator.add(timestamp * 1_000_000)

        result = accumulator.summary(gap_factor=3.0)

        self.assertEqual(result["gap_count"], 1)
        self.assertEqual(result["estimated_drops"], 4)
        self.assertAlmostEqual(result["jitter_cv_percent"], 0.0)

    def test_duplicate_and_backward_timestamps_are_exact(self) -> None:
        accumulator = IntervalAccumulator()
        for timestamp in [100, 110, 110, 105, 120]:
            accumulator.add(timestamp)

        result = accumulator.summary(gap_factor=3.0)

        self.assertEqual(result["duplicate_timestamps"], 1)
        self.assertEqual(result["backward_timestamps"], 1)

    def test_interval_sample_is_bounded(self) -> None:
        accumulator = IntervalAccumulator(sample_limit=100)
        for timestamp in range(10_000):
            accumulator.add(timestamp)

        self.assertLessEqual(len(accumulator.intervals_ns), 100)


if __name__ == "__main__":
    unittest.main()
