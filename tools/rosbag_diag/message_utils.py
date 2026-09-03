"""Small adapters over native ROS and rosbags message objects."""

from __future__ import annotations

import math
from collections.abc import Iterable
from typing import Any


def attr(value: Any, name: str, default: Any = None) -> Any:
    """Read an attribute or mapping key without depending on message runtime."""

    if isinstance(value, dict):
        return value.get(name, default)
    return getattr(value, name, default)


def stamp_to_ns(stamp: Any) -> int | None:
    """Convert ROS1 or ROS2 time objects to nanoseconds."""

    if stamp is None:
        return None
    sec = attr(stamp, "sec", attr(stamp, "secs", None))
    nanosec = attr(stamp, "nanosec", attr(stamp, "nsec", attr(stamp, "nsecs", None)))
    if sec is None or nanosec is None:
        return None
    try:
        return int(sec) * 1_000_000_000 + int(nanosec)
    except (TypeError, ValueError, OverflowError):
        return None


def message_stamp_ns(message: Any) -> int | None:
    """Return ``message.header.stamp`` when present."""

    return stamp_to_ns(attr(attr(message, "header"), "stamp"))


def vector3(value: Any) -> tuple[float, float, float] | None:
    try:
        result = (float(attr(value, "x")), float(attr(value, "y")), float(attr(value, "z")))
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(component) for component in result) else None


def quaternion(value: Any) -> tuple[float, float, float, float] | None:
    try:
        result = (
            float(attr(value, "x")),
            float(attr(value, "y")),
            float(attr(value, "z")),
            float(attr(value, "w")),
        )
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(component) for component in result) else None


def magnitude(values: Iterable[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def quaternion_angle_degrees(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> float:
    """Return the shortest angular distance between two quaternions."""

    first_norm = magnitude(first)
    second_norm = magnitude(second)
    if first_norm <= 1e-12 or second_norm <= 1e-12:
        return float("nan")
    dot = abs(sum(a * b for a, b in zip(first, second))) / (first_norm * second_norm)
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))


def sequence(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    try:
        return list(value)
    except TypeError:
        return []


def text(value: Any) -> str:
    if value is None:
        return ""
    return str(value)
