"""Transform-tree continuity and drift diagnostics."""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass, field
from statistics import median
from typing import Any

from .message_utils import (
    attr,
    magnitude,
    quaternion,
    quaternion_angle_degrees,
    stamp_to_ns,
    text,
    vector3,
)
from .metrics import percentile
from .model import Issue


def _frame_name(value: Any) -> str:
    return text(value).strip().lstrip("/")


@dataclass
class _TransformEdge:
    parent: str
    child: str
    is_static: bool
    samples: int = 0
    invalid_samples: int = 0
    first_time_ns: int | None = None
    last_time_ns: int | None = None
    first_translation: tuple[float, float, float] | None = None
    last_translation: tuple[float, float, float] | None = None
    first_rotation: tuple[float, float, float, float] | None = None
    last_rotation: tuple[float, float, float, float] | None = None
    previous_translation: tuple[float, float, float] | None = None
    previous_rotation: tuple[float, float, float, float] | None = None
    previous_time_ns: int | None = None
    translation_steps: list[float] = field(default_factory=list)
    rotation_steps_deg: list[float] = field(default_factory=list)
    rotation_norm_errors: list[float] = field(default_factory=list)
    intervals_ns: list[int] = field(default_factory=list)
    maximum_speed_mps: float = 0.0
    timestamp_backwards: int = 0
    static_changed: int = 0

    def add(
        self,
        timestamp_ns: int,
        translation: tuple[float, float, float],
        rotation: tuple[float, float, float, float],
    ) -> None:
        self.samples += 1
        self.rotation_norm_errors.append(abs(magnitude(rotation) - 1.0))
        if self.first_time_ns is None:
            self.first_time_ns = timestamp_ns
            self.first_translation = translation
            self.first_rotation = rotation
        if self.previous_translation is not None and self.previous_rotation is not None:
            delta_translation = magnitude(
                current - previous
                for current, previous in zip(translation, self.previous_translation)
            )
            delta_rotation = quaternion_angle_degrees(self.previous_rotation, rotation)
            if math.isfinite(delta_rotation):
                self.rotation_steps_deg.append(delta_rotation)
            self.translation_steps.append(delta_translation)
            if self.previous_time_ns is not None:
                interval = timestamp_ns - self.previous_time_ns
                if interval > 0:
                    self.intervals_ns.append(interval)
                    self.maximum_speed_mps = max(
                        self.maximum_speed_mps,
                        delta_translation / (interval / 1_000_000_000.0),
                    )
                elif interval < 0:
                    self.timestamp_backwards += 1
            if self.is_static and (delta_translation > 1e-5 or delta_rotation > 0.01):
                self.static_changed += 1

        self.previous_translation = translation
        self.previous_rotation = rotation
        self.previous_time_ns = timestamp_ns
        self.last_translation = translation
        self.last_rotation = rotation
        self.last_time_ns = timestamp_ns

    def summary(self) -> dict[str, Any]:
        net_translation = None
        net_rotation = None
        if self.first_translation is not None and self.last_translation is not None:
            net_translation = magnitude(
                last - first
                for first, last in zip(self.first_translation, self.last_translation)
            )
        if self.first_rotation is not None and self.last_rotation is not None:
            net_rotation = quaternion_angle_degrees(self.first_rotation, self.last_rotation)
        median_step = median(self.translation_steps) if self.translation_steps else 0.0
        median_rotation = median(self.rotation_steps_deg) if self.rotation_steps_deg else 0.0
        median_period_ms = (
            median(self.intervals_ns) / 1_000_000.0 if self.intervals_ns else None
        )
        maximum_gap_ms = max(self.intervals_ns) / 1_000_000.0 if self.intervals_ns else None
        return {
            "parent": self.parent,
            "child": self.child,
            "static": self.is_static,
            "samples": self.samples,
            "invalid_samples": self.invalid_samples,
            "first_time_ns": self.first_time_ns,
            "last_time_ns": self.last_time_ns,
            "net_translation_m": net_translation,
            "translation_path_m": sum(self.translation_steps),
            "median_step_m": median_step,
            "p95_step_m": percentile(self.translation_steps, 0.95),
            "max_step_m": max(self.translation_steps) if self.translation_steps else 0.0,
            "net_rotation_deg": net_rotation,
            "median_rotation_step_deg": median_rotation,
            "p95_rotation_step_deg": percentile(self.rotation_steps_deg, 0.95),
            "max_rotation_step_deg": max(self.rotation_steps_deg)
            if self.rotation_steps_deg
            else 0.0,
            "step_count": len(self.translation_steps),
            "rotation_step_count": len(self.rotation_steps_deg),
            "p95_rotation_norm_error": percentile(self.rotation_norm_errors, 0.95),
            "median_period_ms": median_period_ms,
            "max_gap_ms": maximum_gap_ms,
            "max_speed_mps": self.maximum_speed_mps,
            "timestamp_backwards": self.timestamp_backwards,
            "static_changed": self.static_changed,
        }


class TfAnalyzer:
    """Analyze frame graph integrity and transform discontinuities."""

    def __init__(self) -> None:
        self._edges: dict[tuple[str, str, bool], _TransformEdge] = {}
        self._parents_by_child: dict[str, set[str]] = defaultdict(set)
        self.message_count = 0
        self.transform_count = 0
        self.invalid_count = 0

    def consume(self, topic: str, message: Any, storage_timestamp_ns: int) -> bool:
        if topic not in {"/tf", "tf", "/tf_static", "tf_static"}:
            return False
        self.message_count += 1
        is_static = topic.endswith("tf_static")
        for transform_stamped in attr(message, "transforms", []) or []:
            self.transform_count += 1
            header = attr(transform_stamped, "header")
            parent = _frame_name(attr(header, "frame_id"))
            child = _frame_name(attr(transform_stamped, "child_frame_id"))
            transform = attr(transform_stamped, "transform")
            translation = vector3(attr(transform, "translation"))
            rotation = quaternion(attr(transform, "rotation"))
            timestamp_ns = stamp_to_ns(attr(header, "stamp"))
            if timestamp_ns in (None, 0):
                timestamp_ns = storage_timestamp_ns
            if (
                not parent
                or not child
                or translation is None
                or rotation is None
                or magnitude(rotation) <= 1e-12
            ):
                self.invalid_count += 1
                continue
            edge_key = (parent, child, is_static)
            edge = self._edges.setdefault(edge_key, _TransformEdge(parent, child, is_static))
            edge.add(timestamp_ns, translation, rotation)
            self._parents_by_child[child].add(parent)
        return True

    def finish(
        self,
        *,
        translation_jump_m: float,
        rotation_jump_deg: float,
        maximum_speed_mps: float,
        gap_factor: float,
        timing_complete: bool = True,
    ) -> tuple[dict[str, Any], list[Issue]]:
        edge_summaries = [edge.summary() for edge in self._edges.values()]
        edge_summaries.sort(key=lambda item: (item["parent"], item["child"], item["static"]))
        issues: list[Issue] = []

        if not edge_summaries:
            issues.append(Issue(
                issue_id="TF_NOT_RECORDED",
                severity="notice",
                category="TF",
                title="bag 中未记录 TF",
                evidence="没有发现可解码的 /tf 或 /tf_static 消息。",
                impact="无法离线验证坐标树连通性、跳变和长期位姿变化。",
                suggestion="录包时加入 /tf 与 /tf_static，并确保对应消息类型可用。",
                confidence="high",
            ))
            return {
                "available": False,
                "message_count": self.message_count,
                "transform_count": self.transform_count,
                "invalid_transform_count": self.invalid_count,
                "frame_count": 0,
                "edge_count": 0,
                "connected_components": 0,
                "cycles": [],
                "multiple_parent_frames": {},
                "timing_analysis_complete": timing_complete,
                "edges": [],
            }, issues

        for child, parents in sorted(self._parents_by_child.items()):
            if len(parents) > 1:
                issues.append(Issue(
                    issue_id=f"TF_MULTIPLE_PARENTS:{child}",
                    severity="critical",
                    category="TF",
                    topic="/tf",
                    title=f"坐标系 {child} 出现多个父节点",
                    evidence=f"检测到父节点：{', '.join(sorted(parents))}。",
                    impact="同一时刻的 TF 树可能不唯一，查询变换会失败或产生跳变。",
                    suggestion="检查重复 static_transform_publisher、frame_id 配置和命名空间。",
                    confidence="high",
                ))

        for summary in edge_summaries:
            edge_name = f"{summary['parent']} → {summary['child']}"
            if summary["static_changed"]:
                issues.append(Issue(
                    issue_id=f"TF_STATIC_CHANGED:{edge_name}",
                    severity="critical",
                    category="TF",
                    topic="/tf_static",
                    title="静态 TF 在录制期间发生变化",
                    evidence=f"{edge_name} 有 {summary['static_changed']} 次不一致更新。",
                    impact="静态外参不再唯一，传感器融合和点云投影可能错位。",
                    suggestion="只保留一个静态 TF 发布者，并核对标定外参。",
                    confidence="high",
                ))
            if summary["timestamp_backwards"]:
                issues.append(Issue(
                    issue_id=f"TF_TIME_BACKWARDS:{edge_name}",
                    severity="warning",
                    category="时间戳",
                    topic="/tf",
                    title="TF 时间戳发生回退",
                    evidence=f"{edge_name} 检测到 {summary['timestamp_backwards']} 次回退。",
                    impact="TF 缓存可能拒绝旧数据并导致 extrapolation 错误。",
                    suggestion="统一时钟源，检查驱动是否混用系统时间、设备时间和仿真时间。",
                    confidence="high",
                ))
            if not summary["static"]:
                median_step = float(summary["median_step_m"] or 0.0)
                adaptive_translation = translation_jump_m
                if int(summary["step_count"]) >= 5:
                    adaptive_translation = max(translation_jump_m, median_step * 10.0)
                median_rotation = float(summary["median_rotation_step_deg"] or 0.0)
                adaptive_rotation = rotation_jump_deg
                if int(summary["rotation_step_count"]) >= 5:
                    adaptive_rotation = max(rotation_jump_deg, median_rotation * 10.0)
                if float(summary["max_step_m"] or 0.0) > adaptive_translation:
                    issues.append(Issue(
                        issue_id=f"TF_TRANSLATION_JUMP:{edge_name}",
                        severity="warning",
                        category="TF",
                        topic="/tf",
                        title="TF 出现平移跳变",
                        evidence=(
                            f"{edge_name} 最大单步 {summary['max_step_m']:.3f} m，"
                            f"自适应阈值 {adaptive_translation:.3f} m。"
                        ),
                        impact="可能表现为地图撕裂、定位瞬移或控制指令突变。",
                        suggestion="核查里程计重置、SLAM 重定位、时间同步和错误外参。",
                        confidence="medium",
                    ))
                if float(summary["max_rotation_step_deg"] or 0.0) > adaptive_rotation:
                    issues.append(Issue(
                        issue_id=f"TF_ROTATION_JUMP:{edge_name}",
                        severity="warning",
                        category="TF",
                        topic="/tf",
                        title="TF 出现旋转跳变",
                        evidence=(
                            f"{edge_name} 最大单步 {summary['max_rotation_step_deg']:.2f}°，"
                            f"自适应阈值 {adaptive_rotation:.2f}°。"
                        ),
                        impact="可能造成点云重影、姿态突变或局部地图错层。",
                        suggestion="检查 IMU 方向、四元数归一化、重定位和时间同步。",
                        confidence="medium",
                    ))
                if float(summary["max_speed_mps"] or 0.0) > maximum_speed_mps:
                    issues.append(Issue(
                        issue_id=f"TF_SPEED_SPIKE:{edge_name}",
                        severity="warning",
                        category="TF",
                        topic="/tf",
                        title="TF 推算速度异常",
                        evidence=(
                            f"{edge_name} 相邻样本推算最大速度 "
                            f"{summary['max_speed_mps']:.2f} m/s。"
                        ),
                        impact="可能存在时间间隔过小、位姿跳变或单位错误。",
                        suggestion="将该时刻与 odometry、IMU 和定位日志交叉检查。",
                        confidence="medium",
                    ))
                median_period = summary["median_period_ms"]
                maximum_gap = summary["max_gap_ms"]
                if (
                    timing_complete
                    and median_period
                    and maximum_gap
                    and maximum_gap > median_period * gap_factor
                ):
                    issues.append(Issue(
                        issue_id=f"TF_GAP:{edge_name}",
                        severity="warning",
                        category="TF",
                        topic="/tf",
                        title="TF 更新链出现长时间断档",
                        evidence=(
                            f"{edge_name} 最大间隔 {maximum_gap:.1f} ms，"
                            f"中位周期 {median_period:.1f} ms。"
                        ),
                        impact="查询断档时段内的变换可能超时或需要外推。",
                        suggestion="检查 TF 发布节点阻塞、CPU 负载和录包写盘带宽。",
                        confidence="high",
                    ))
            rotation_norm_error = float(summary["p95_rotation_norm_error"] or 0.0)
            if rotation_norm_error > 0.05:
                issues.append(Issue(
                    issue_id=f"TF_QUATERNION_NORM:{edge_name}",
                    severity="warning",
                    category="TF",
                    topic="/tf_static" if summary["static"] else "/tf",
                    title="TF 四元数未正确归一化",
                    evidence=f"{edge_name} 的四元数模长误差 P95 为 {rotation_norm_error:.4f}。",
                    impact="旋转插值和坐标变换可能产生系统误差或被 TF2 拒绝。",
                    suggestion="在发布前归一化四元数，并检查姿态转换与无效值保护。",
                    confidence="high",
                ))

        frames, components = _graph_components(edge_summaries)
        cycles = _find_cycles(edge_summaries)
        if len(components) > 1:
            component_text = "；".join(
                ", ".join(sorted(component)) for component in components[:4]
            )
            issues.append(Issue(
                issue_id="TF_DISCONNECTED_GRAPH",
                severity="warning",
                category="TF",
                topic="/tf",
                title="TF 坐标图不连通",
                evidence=f"检测到 {len(components)} 个独立分量：{component_text}",
                impact="不同分量之间无法查询坐标变换。",
                suggestion="补录缺失的静态外参或检查 frame 名称/命名空间拼写。",
                confidence="high",
            ))
        if cycles:
            issues.append(Issue(
                issue_id="TF_GRAPH_CYCLE",
                severity="critical",
                category="TF",
                topic="/tf",
                title="TF 坐标图存在环",
                evidence="；".join(" → ".join(cycle) for cycle in cycles[:3]),
                impact="TF 必须是一棵无环树，环会造成父子关系冲突。",
                suggestion="移除反向或重复发布的变换，保证每个 child 只有一个 parent。",
                confidence="high",
            ))

        return {
            "available": True,
            "message_count": self.message_count,
            "transform_count": self.transform_count,
            "invalid_transform_count": self.invalid_count,
            "frame_count": len(frames),
            "edge_count": len(edge_summaries),
            "connected_components": len(components),
            "components": [sorted(component) for component in components],
            "cycles": cycles,
            "multiple_parent_frames": {
                child: sorted(parents)
                for child, parents in sorted(self._parents_by_child.items())
                if len(parents) > 1
            },
            "timing_analysis_complete": timing_complete,
            "edges": edge_summaries,
        }, issues


def _graph_components(edges: list[dict[str, Any]]) -> tuple[set[str], list[set[str]]]:
    frames: set[str] = set()
    adjacency: dict[str, set[str]] = defaultdict(set)
    for edge in edges:
        parent = str(edge["parent"])
        child = str(edge["child"])
        frames.update((parent, child))
        adjacency[parent].add(child)
        adjacency[child].add(parent)
    components: list[set[str]] = []
    unseen = set(frames)
    while unseen:
        root = min(unseen)
        stack = [root]
        component: set[str] = set()
        while stack:
            frame = stack.pop()
            if frame in component:
                continue
            component.add(frame)
            unseen.discard(frame)
            stack.extend(adjacency[frame] - component)
        components.append(component)
    components.sort(key=lambda item: (-len(item), sorted(item)))
    return frames, components


def _find_cycles(edges: list[dict[str, Any]]) -> list[list[str]]:
    children: dict[str, set[str]] = defaultdict(set)
    for edge in edges:
        children[str(edge["parent"])].add(str(edge["child"]))
    visited: set[str] = set()
    active: list[str] = []
    cycles: list[list[str]] = []

    def visit(frame: str) -> None:
        if frame in active:
            start = active.index(frame)
            cycle = active[start:] + [frame]
            if cycle not in cycles:
                cycles.append(cycle)
            return
        if frame in visited:
            return
        active.append(frame)
        for child in sorted(children.get(frame, set())):
            visit(child)
        active.pop()
        visited.add(frame)

    for frame in sorted(set(children) | {child for values in children.values() for child in values}):
        visit(frame)
    return cycles
