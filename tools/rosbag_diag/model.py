"""Domain models for ROS bag diagnostics.

The models deliberately contain no ROS or Qt imports.  This keeps report
generation and all diagnostic rules directly unit-testable.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class TopicDefinition:
    """Metadata known about one recorded topic."""

    name: str
    msgtype: str
    serialization_format: str = ""
    offered_qos_profiles: str = ""
    declared_count: int = 0


@dataclass
class BagInformation:
    """Format-independent bag metadata."""

    path: Path
    ros_version: int
    storage: str
    reader: str
    files: list[Path] = field(default_factory=list)
    topics: dict[str, TopicDefinition] = field(default_factory=dict)
    size_bytes: int = 0
    message_count: int = 0
    start_time_ns: int | None = None
    end_time_ns: int | None = None
    duration_ns: int = 0
    compression_format: str = ""
    compression_mode: str = ""
    metadata_warnings: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class DecodedRecord:
    """A sampled, deserialized message."""

    topic: str
    msgtype: str
    timestamp_ns: int
    message: Any


@dataclass(frozen=True)
class Issue:
    """One actionable finding in a diagnostic report."""

    issue_id: str
    severity: str
    category: str
    title: str
    evidence: str
    suggestion: str
    topic: str = ""
    impact: str = ""
    confidence: str = "medium"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class DiagnosticReport:
    """Complete serializable report."""

    generated_at: str
    bag: dict[str, Any]
    summary: dict[str, Any]
    thresholds: dict[str, Any]
    topics: list[dict[str, Any]]
    tf: dict[str, Any]
    sensors: list[dict[str, Any]]
    issues: list[Issue]
    recommendations: list[str]
    limitations: list[str]
    schema: str = "rosbag-diagnostic-report"
    version: int = 1

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": self.schema,
            "version": self.version,
            "generated_at": self.generated_at,
            "bag": self.bag,
            "summary": self.summary,
            "thresholds": self.thresholds,
            "topics": self.topics,
            "tf": self.tf,
            "sensors": self.sensors,
            "issues": [issue.to_dict() for issue in self.issues],
            "recommendations": self.recommendations,
            "limitations": self.limitations,
        }


class BagDiagnosticError(RuntimeError):
    """Base class for expected user-facing diagnostic failures."""


class UnsupportedBagError(BagDiagnosticError):
    """The selected path is not a supported bag."""


class MissingDependencyError(BagDiagnosticError):
    """An optional reader required for this bag is unavailable."""
