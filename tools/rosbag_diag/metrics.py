"""Numerical helpers for timing diagnostics."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from statistics import fmean, median
from typing import Iterable


def percentile(values: Iterable[float], probability: float) -> float | None:
    """Return a linearly interpolated percentile, or ``None`` for no values."""

    ordered = sorted(float(value) for value in values if math.isfinite(value))
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    position = min(1.0, max(0.0, probability)) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def median_absolute_deviation(values: Iterable[float]) -> float | None:
    """Return the median absolute deviation from the median."""

    finite = [float(value) for value in values if math.isfinite(value)]
    if not finite:
        return None
    center = median(finite)
    return median(abs(value - center) for value in finite)


@dataclass
class IntervalAccumulator:
    """Collect exact counters and a bounded deterministic interval sample."""

    sample_limit: int = 200_000
    count: int = 0
    first_ns: int | None = None
    last_ns: int | None = None
    minimum_ns: int | None = None
    maximum_ns: int | None = None
    previous_ns: int | None = None
    duplicate_timestamps: int = 0
    backward_timestamps: int = 0
    interval_count: int = 0
    positive_interval_sum_ns: int = 0
    positive_interval_squared_sum_ns: float = 0.0
    maximum_positive_interval_ns: int = 0
    intervals_ns: list[int] = field(default_factory=list)

    def add(self, timestamp_ns: int) -> None:
        timestamp_ns = int(timestamp_ns)
        self.count += 1
        if self.first_ns is None:
            self.first_ns = timestamp_ns
            self.last_ns = timestamp_ns
            self.minimum_ns = timestamp_ns
            self.maximum_ns = timestamp_ns
            self.previous_ns = timestamp_ns
            return

        assert self.previous_ns is not None
        interval = timestamp_ns - self.previous_ns
        self.interval_count += 1
        if interval == 0:
            self.duplicate_timestamps += 1
        elif interval < 0:
            self.backward_timestamps += 1
        else:
            self.positive_interval_sum_ns += interval
            self.positive_interval_squared_sum_ns += float(interval) * float(interval)
            self.maximum_positive_interval_ns = max(self.maximum_positive_interval_ns, interval)
            self._sample_interval(interval)

        self.previous_ns = timestamp_ns
        self.last_ns = timestamp_ns
        self.minimum_ns = min(self.minimum_ns or timestamp_ns, timestamp_ns)
        self.maximum_ns = max(self.maximum_ns or timestamp_ns, timestamp_ns)

    def _sample_interval(self, interval_ns: int) -> None:
        if self.sample_limit <= 0:
            return
        seen = self.interval_count
        if len(self.intervals_ns) < self.sample_limit:
            self.intervals_ns.append(interval_ns)
            return
        # Deterministic reservoir sampling keeps tests reproducible while
        # preventing multi-million-message bags from exhausting memory.
        slot = ((seen * 2_654_435_761) & 0xFFFFFFFF) % max(1, seen)
        if slot < self.sample_limit:
            self.intervals_ns[slot] = interval_ns

    def summary(self, *, gap_factor: float) -> dict[str, float | int | None]:
        positive = self.intervals_ns
        median_interval = percentile(positive, 0.5)
        p95_interval = percentile(positive, 0.95)
        p99_interval = percentile(positive, 0.99)
        duration_ns = 0
        if self.minimum_ns is not None and self.maximum_ns is not None:
            duration_ns = max(0, self.maximum_ns - self.minimum_ns)

        average_hz = None
        if duration_ns > 0 and self.count > 1:
            average_hz = (self.count - 1) * 1_000_000_000.0 / duration_ns
        median_hz = 1_000_000_000.0 / median_interval if median_interval else None

        mean_interval = None
        standard_deviation = None
        regular_intervals = positive
        if median_interval:
            # Long dropouts belong to the gap metric, not scheduler jitter.
            # Excluding them prevents one outage from turning an otherwise
            # stable 200 Hz topic into a misleading 300% jitter result.
            regular_intervals = [
                value for value in positive
                if value <= median_interval * max(1.1, gap_factor)
            ]
        if regular_intervals:
            mean_interval = fmean(regular_intervals)
            if len(regular_intervals) > 1:
                standard_deviation = math.sqrt(
                    fmean((value - mean_interval) ** 2 for value in regular_intervals)
                )
            else:
                standard_deviation = 0.0

        jitter_cv = None
        if mean_interval and standard_deviation is not None:
            jitter_cv = standard_deviation / mean_interval * 100.0

        p95_jitter = None
        if median_interval and regular_intervals:
            p95_jitter = percentile(
                (abs(value - median_interval) for value in regular_intervals),
                0.95,
            )

        estimated_missing = 0
        gap_count = 0
        if median_interval and median_interval > 0:
            threshold = median_interval * max(1.1, gap_factor)
            for interval in positive:
                if interval > threshold:
                    gap_count += 1
                    estimated_missing += max(1, int(round(interval / median_interval)) - 1)
            if self.interval_count > len(positive) and positive:
                scale = self.interval_count / len(positive)
                estimated_missing = int(round(estimated_missing * scale))
                gap_count = int(round(gap_count * scale))

        denominator = self.count + estimated_missing
        estimated_drop_ratio = estimated_missing / denominator * 100.0 if denominator else 0.0
        to_ms = lambda value: value / 1_000_000.0 if value is not None else None
        return {
            "count": self.count,
            "first_time_ns": self.minimum_ns,
            "last_time_ns": self.maximum_ns,
            "duration_sec": duration_ns / 1_000_000_000.0,
            "mean_hz": average_hz,
            "median_hz": median_hz,
            "mean_period_ms": to_ms(mean_interval),
            "median_period_ms": to_ms(median_interval),
            "p95_period_ms": to_ms(p95_interval),
            "p99_period_ms": to_ms(p99_interval),
            "jitter_std_ms": to_ms(standard_deviation),
            "jitter_cv_percent": jitter_cv,
            "p95_jitter_ms": to_ms(p95_jitter),
            "max_gap_ms": self.maximum_positive_interval_ns / 1_000_000.0,
            "gap_count": gap_count,
            "estimated_drops": estimated_missing,
            "estimated_drop_ratio_percent": estimated_drop_ratio,
            "duplicate_timestamps": self.duplicate_timestamps,
            "backward_timestamps": self.backward_timestamps,
            "timing_sample_count": len(positive),
        }


@dataclass
class ScalarAccumulator:
    """Bounded scalar samples with invalid-value accounting."""

    sample_limit: int = 20_000
    seen: int = 0
    invalid: int = 0
    values: list[float] = field(default_factory=list)

    def add(self, value: float) -> None:
        self.seen += 1
        try:
            value = float(value)
        except (TypeError, ValueError):
            self.invalid += 1
            return
        if not math.isfinite(value):
            self.invalid += 1
            return
        if len(self.values) < self.sample_limit:
            self.values.append(value)
            return
        slot = ((self.seen * 2_246_822_519) & 0xFFFFFFFF) % self.seen
        if slot < self.sample_limit:
            self.values[slot] = value

    def summary(self) -> dict[str, float | int | None]:
        if not self.values:
            return {
                "samples": self.seen,
                "invalid": self.invalid,
                "minimum": None,
                "maximum": None,
                "mean": None,
                "median": None,
                "p95": None,
                "mad": None,
            }
        return {
            "samples": self.seen,
            "invalid": self.invalid,
            "minimum": min(self.values),
            "maximum": max(self.values),
            "mean": fmean(self.values),
            "median": median(self.values),
            "p95": percentile(self.values, 0.95),
            "mad": median_absolute_deviation(self.values),
        }
