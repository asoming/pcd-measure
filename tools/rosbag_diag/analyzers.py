"""Application service that orchestrates all offline bag diagnostics."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from statistics import median
from typing import Any, Callable

from .message_utils import attr, message_stamp_ns
from .metrics import IntervalAccumulator, percentile
from .model import DiagnosticReport, Issue
from .readers import BagSource, open_bag
from .sensor_analysis import SensorAnalyzer
from .tf_analysis import TfAnalyzer


ProgressCallback = Callable[[int, str], None]


@dataclass(frozen=True)
class DiagnosticOptions:
    """Tunable thresholds for one diagnostic run."""

    gap_factor: float = 3.0
    jitter_warning_percent: float = 20.0
    jitter_critical_percent: float = 60.0
    drop_warning_percent: float = 1.0
    drop_critical_percent: float = 5.0
    control_minimum_hz: float = 10.0
    tf_translation_jump_m: float = 0.5
    tf_rotation_jump_deg: float = 30.0
    tf_maximum_speed_mps: float = 10.0
    header_lag_warning_ms: float = 100.0
    maximum_timing_samples: int = 200_000
    maximum_payload_samples_per_topic: int = 2_000
    deep_analysis: bool = True

    def validate(self) -> None:
        if self.gap_factor <= 1.0:
            raise ValueError("gap_factor 必须大于 1。")
        if self.jitter_warning_percent <= 0.0:
            raise ValueError("jitter_warning_percent 必须大于 0。")
        if self.jitter_critical_percent < self.jitter_warning_percent:
            raise ValueError("严重抖动阈值不能小于警告阈值。")
        if self.control_minimum_hz <= 0.0:
            raise ValueError("control_minimum_hz 必须大于 0。")
        if self.maximum_timing_samples < 100:
            raise ValueError("maximum_timing_samples 不能小于 100。")
        if self.maximum_payload_samples_per_topic < 0:
            raise ValueError("maximum_payload_samples_per_topic 不能为负数。")

    def to_dict(self) -> dict[str, Any]:
        return {
            "gap_factor": self.gap_factor,
            "jitter_warning_percent": self.jitter_warning_percent,
            "jitter_critical_percent": self.jitter_critical_percent,
            "drop_warning_percent": self.drop_warning_percent,
            "drop_critical_percent": self.drop_critical_percent,
            "control_minimum_hz": self.control_minimum_hz,
            "tf_translation_jump_m": self.tf_translation_jump_m,
            "tf_rotation_jump_deg": self.tf_rotation_jump_deg,
            "tf_maximum_speed_mps": self.tf_maximum_speed_mps,
            "header_lag_warning_ms": self.header_lag_warning_ms,
            "maximum_timing_samples": self.maximum_timing_samples,
            "maximum_payload_samples_per_topic": self.maximum_payload_samples_per_topic,
            "deep_analysis": self.deep_analysis,
        }


@dataclass
class _HeaderTiming:
    count: int = 0
    zero_count: int = 0
    backwards: int = 0
    previous_ns: int | None = None
    lag_ms: list[float] | None = None
    intervals: IntervalAccumulator | None = None

    def __post_init__(self) -> None:
        if self.lag_ms is None:
            self.lag_ms = []
        if self.intervals is None:
            self.intervals = IntervalAccumulator(sample_limit=20_000)

    def add(self, storage_ns: int, header_ns: int | None) -> None:
        self.count += 1
        if header_ns in (None, 0):
            self.zero_count += 1
            return
        if self.previous_ns is not None and header_ns < self.previous_ns:
            self.backwards += 1
        self.previous_ns = header_ns
        assert self.intervals is not None
        self.intervals.add(header_ns)
        assert self.lag_ms is not None
        if len(self.lag_ms) < 20_000:
            self.lag_ms.append((storage_ns - header_ns) / 1_000_000.0)

    def summary(self, *, topic_count: int, gap_factor: float) -> dict[str, Any]:
        assert self.lag_ms is not None
        assert self.intervals is not None
        absolute = [abs(value) for value in self.lag_ms]
        timing = self.intervals.summary(gap_factor=gap_factor)
        valid_samples = max(0, self.count - self.zero_count)
        scale = topic_count / valid_samples if valid_samples else 1.0
        if valid_samples >= 2 and topic_count >= valid_samples:
            duration_sec = float(timing.get("duration_sec", 0.0) or 0.0)
            if duration_sec > 0.0:
                timing["mean_hz"] = (topic_count - 1) / duration_sec
            if timing.get("median_hz") is not None:
                timing["median_hz"] = float(timing["median_hz"]) * scale
            for key in (
                "mean_period_ms",
                "median_period_ms",
                "p95_period_ms",
                "p99_period_ms",
                "jitter_std_ms",
                "p95_jitter_ms",
                "max_gap_ms",
            ):
                if timing.get(key) is not None:
                    timing[key] = float(timing[key]) / scale
            timing["estimated_drops"] = int(
                round(float(timing.get("estimated_drops", 0) or 0) * scale)
            )
            denominator = topic_count + timing["estimated_drops"]
            timing["estimated_drop_ratio_percent"] = (
                timing["estimated_drops"] / denominator * 100.0 if denominator else 0.0
            )
        return {
            "decoded_samples": self.count,
            "valid_stamp_samples": valid_samples,
            "sample_scale": scale,
            "missing_or_zero_stamps": self.zero_count,
            "backward_stamps": self.backwards,
            "lag_mean_ms": sum(self.lag_ms) / len(self.lag_ms) if self.lag_ms else None,
            "lag_median_ms": median(self.lag_ms) if self.lag_ms else None,
            "lag_p95_abs_ms": percentile(absolute, 0.95),
            "lag_max_abs_ms": max(absolute) if absolute else None,
            "timing": timing,
        }


def diagnose_bag(
    path: str | Path,
    options: DiagnosticOptions | None = None,
    progress: ProgressCallback | None = None,
) -> DiagnosticReport:
    """Diagnose a ROS1/ROS2 bag and return an actionable report."""

    options = options or DiagnosticOptions()
    options.validate()
    report_progress = progress or (lambda _percent, _message: None)
    report_progress(1, "识别 bag 格式")
    source = open_bag(path)
    report_progress(4, "读取话题与录制元数据")

    topic_accumulators = {
        name: IntervalAccumulator(sample_limit=options.maximum_timing_samples)
        for name in source.info.topics
    }
    observed_messages = 0
    update_every = max(1, source.info.message_count // 80) if source.info.message_count else 10_000
    for topic, timestamp_ns in source.iter_timestamps():
        accumulator = topic_accumulators.setdefault(
            topic,
            IntervalAccumulator(sample_limit=options.maximum_timing_samples),
        )
        accumulator.add(timestamp_ns)
        observed_messages += 1
        if observed_messages % update_every == 0:
            fraction = observed_messages / max(observed_messages, source.info.message_count)
            report_progress(min(54, 5 + int(49 * fraction)), "统计话题频率、抖动与间隙")

    issues: list[Issue] = []
    topic_results: list[dict[str, Any]] = []
    missing_qos_topics: list[str] = []
    for topic, accumulator in sorted(topic_accumulators.items()):
        definition = source.info.topics.get(topic)
        result = accumulator.summary(gap_factor=options.gap_factor)
        result.update({
            "name": topic,
            "type": definition.msgtype if definition else "",
            "serialization_format": definition.serialization_format if definition else "",
            "declared_count": definition.declared_count if definition else 0,
        })
        qos = _analyze_qos(
            topic,
            definition.offered_qos_profiles if definition else "",
        )
        result["qos"] = qos
        if source.info.ros_version == 2 and qos["status"] == "unknown":
            missing_qos_topics.append(topic)
        result["status"] = "normal"
        result["time_basis"] = "storage"
        result["expected_minimum_hz"] = _expected_minimum_hz(
            topic,
            result["type"],
            options.control_minimum_hz,
        )
        topic_results.append(result)

    if missing_qos_topics:
        preview = ", ".join(missing_qos_topics[:6])
        if len(missing_qos_topics) > 6:
            preview += f" 等 {len(missing_qos_topics)} 个话题"
        issues.append(Issue(
            issue_id="QOS_METADATA_MISSING",
            severity="notice",
            category="QoS",
            title="录包未保存可验证的 QoS 配置",
            evidence=f"以下话题缺少 offered QoS 元数据：{preview}。",
            impact="无法仅凭 bag 证明回放发布者与目标订阅者兼容。",
            suggestion="回放失败时用 ros2 topic info -v 核对订阅端，并提供 QoS override YAML。",
            confidence="high",
        ))

    for warning_index, warning in enumerate(source.info.metadata_warnings):
        issues.append(Issue(
            issue_id=f"BAG_METADATA:{warning_index}",
            severity="warning",
            category="录包完整性",
            title="bag 元数据与存储内容不一致",
            evidence=warning,
            impact="bag 可能未正常关闭、被截断或 metadata.yaml 已过期。",
            suggestion="先备份原始 bag，再运行 ros2 bag reindex 后重新诊断。",
            confidence="high",
        ))

    tf_analyzer = TfAnalyzer()
    sensor_analyzer = SensorAnalyzer()
    header_timings: dict[str, _HeaderTiming] = {}
    decoded_messages = 0
    decoded_topics: set[str] = set()
    if options.deep_analysis and options.maximum_payload_samples_per_topic > 0:
        report_progress(58, "抽样解析 TF 与传感器载荷")
        for record in source.iter_decoded_samples(options.maximum_payload_samples_per_topic):
            decoded_messages += 1
            decoded_topics.add(record.topic)
            if attr(record.message, "header") is not None:
                header = header_timings.setdefault(record.topic, _HeaderTiming())
                header.add(record.timestamp_ns, message_stamp_ns(record.message))
            tf_analyzer.consume(record.topic, record.message, record.timestamp_ns)
            sensor_analyzer.consume(record.topic, record.msgtype, record.message)
            if decoded_messages % 500 == 0:
                report_progress(65, "检查 TF 跳变、传感器无效值与冻结帧")

    tf_result, tf_issues = tf_analyzer.finish(
        translation_jump_m=options.tf_translation_jump_m,
        rotation_jump_deg=options.tf_rotation_jump_deg,
        maximum_speed_mps=options.tf_maximum_speed_mps,
        gap_factor=options.gap_factor,
    )
    sensors, sensor_issues = sensor_analyzer.finish()
    issues.extend(tf_issues)
    issues.extend(sensor_issues)

    header_summaries = {
        topic: state.summary(
            topic_count=int(topic_accumulators[topic].count),
            gap_factor=options.gap_factor,
        )
        for topic, state in header_timings.items()
        if topic in topic_accumulators
    }
    timing_fields = (
        "first_time_ns",
        "last_time_ns",
        "duration_sec",
        "mean_hz",
        "median_hz",
        "mean_period_ms",
        "median_period_ms",
        "p95_period_ms",
        "p99_period_ms",
        "jitter_std_ms",
        "jitter_cv_percent",
        "p95_jitter_ms",
        "max_gap_ms",
        "gap_count",
        "estimated_drops",
        "estimated_drop_ratio_percent",
        "duplicate_timestamps",
        "backward_timestamps",
        "timing_sample_count",
    )
    for topic_result in topic_results:
        header_summary = header_summaries.get(topic_result["name"], {
            "decoded_samples": 0,
            "valid_stamp_samples": 0,
            "sample_scale": 1.0,
            "missing_or_zero_stamps": 0,
            "backward_stamps": 0,
            "lag_mean_ms": None,
            "lag_median_ms": None,
            "lag_p95_abs_ms": None,
            "lag_max_abs_ms": None,
            "timing": {},
        })
        topic_result["header_timing"] = header_summary
        valid_header_samples = int(header_summary.get("valid_stamp_samples", 0))
        decoded_header_samples = int(header_summary.get("decoded_samples", 0))
        if (
            valid_header_samples >= 10
            and valid_header_samples / max(1, decoded_header_samples) >= 0.8
        ):
            topic_result["storage_timing"] = {
                key: topic_result.get(key) for key in timing_fields
            }
            header_rate = header_summary.get("timing", {})
            for key in timing_fields:
                if key in header_rate:
                    topic_result[key] = header_rate[key]
            # Count remains the exact storage count rather than the decoded
            # sample count used to estimate header timing.
            topic_result["count"] = topic_accumulators[topic_result["name"]].count
            topic_result["time_basis"] = "header"

        topic_issues = _topic_timing_issues(topic_result, options)
        topic_issues.extend(_qos_issues(topic_result["name"], topic_result["qos"]))
        header_issues = _header_timing_issues(topic_result["name"], header_summary, options)
        topic_issues.extend(header_issues)
        issues.extend(topic_issues)
        topic_result["status"] = _status_for_issues(topic_issues)

    if source.decode_warnings:
        warning_preview = "；".join(source.decode_warnings[:5])
        issues.append(Issue(
            issue_id="PAYLOAD_DECODE_PARTIAL",
            severity="notice",
            category="解析能力",
            title="部分消息只能完成时序诊断",
            evidence=warning_preview,
            impact="对应话题不会进行字段级噪声、冻结和时间戳检查。",
            suggestion="启动前 source 对应工作空间，或安装 rosbags 并提供自定义消息定义。",
            confidence="high",
        ))
    if not options.deep_analysis:
        issues.append(Issue(
            issue_id="DEEP_ANALYSIS_DISABLED",
            severity="notice",
            category="解析能力",
            title="已关闭消息载荷深度分析",
            evidence="本次只分析了存储时间戳、频率、间隙和元数据。",
            impact="TF、header 时间戳和传感器字段异常未检查。",
            suggestion="启用深度分析后重新运行。",
            confidence="high",
        ))

    if observed_messages == 0:
        issues.append(Issue(
            issue_id="BAG_EMPTY",
            severity="critical",
            category="录包完整性",
            title="bag 中没有消息",
            evidence="存储索引返回 0 条消息。",
            impact="无法回放或进行机器人数据分析。",
            suggestion="检查录制命令、话题选择、磁盘空间和 bag 是否正常关闭。",
            confidence="high",
        ))

    report_progress(86, "汇总故障模式与修复建议")
    issues = _deduplicate_issues(issues)
    issues.sort(key=lambda issue: (_severity_rank(issue.severity), issue.category, issue.title))
    for topic_result in topic_results:
        topic_result["status"] = _status_for_issues([
            issue for issue in issues if issue.topic == topic_result["name"]
        ])
    summary = _build_summary(
        issues,
        observed_messages=observed_messages,
        decoded_messages=decoded_messages,
        decoded_topics=len(decoded_topics),
        total_topics=len(topic_results),
    )
    bag = _bag_to_dict(source, observed_messages, decoded_messages, decoded_topics)
    recommendations = _recommendations(issues)
    limitations = [
        "丢包数根据时间间隙和中位周期估算；没有发布端序号时不能区分真实丢包、停发和录包漏写。",
        "QoS 不匹配需要订阅端配置才能最终确认；离线报告只标记缺失元数据或同话题配置冲突风险。",
        "TF 的净位移是位姿变化指标；机器人正常运动也会产生变化，跳变告警才表示可疑不连续。",
        "传感器噪声使用抽样与通用阈值筛查；运动工况、设备量程和厂商规格可能改变合理范围。",
        "未安装自定义消息类型时，该话题仍会完成存储层时序分析，但无法检查消息字段。",
    ]
    report = DiagnosticReport(
        generated_at=datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        bag=bag,
        summary=summary,
        thresholds=options.to_dict(),
        topics=topic_results,
        tf=tf_result,
        sensors=sensors,
        issues=issues,
        recommendations=recommendations,
        limitations=limitations,
    )
    report_progress(100, "诊断完成")
    return report


def _topic_timing_issues(topic: dict[str, Any], options: DiagnosticOptions) -> list[Issue]:
    issues: list[Issue] = []
    name = topic["name"]
    count = int(topic.get("count", 0))
    expected = _expected_minimum_hz(name, str(topic.get("type", "")), options.control_minimum_hz)
    mean_hz = topic.get("mean_hz")
    if count == 1:
        issues.append(Issue(
            issue_id=f"TOPIC_SINGLE_MESSAGE:{name}",
            severity="notice",
            category="话题频率",
            topic=name,
            title="话题只有一条消息",
            evidence="无法计算发布频率、抖动或连续丢包。",
            impact="如果不是静态/一次性配置话题，可能存在录制或发布故障。",
            suggestion="确认该话题是否按设计只发布一次。",
            confidence="medium",
        ))
    if expected and mean_hz is not None and mean_hz < expected:
        severity = "critical" if mean_hz < expected * 0.5 else "warning"
        issues.append(Issue(
            issue_id=f"TOPIC_RATE_LOW:{name}",
            severity=severity,
            category="控制周期" if _is_control_topic(name, str(topic.get("type", ""))) else "话题频率",
            topic=name,
            title="控制/关键传感器频率未达到内置基线",
            evidence=f"实测 {mean_hz:.2f} Hz，基线最低 {expected:.2f} Hz。",
            impact="控制延迟、定位质量或障碍物更新速度可能下降。",
            suggestion="核查节点定时器、CPU 调度、DDS 队列、设备带宽，并按设备规格调整阈值。",
            confidence="medium",
        ))

    jitter = topic.get("jitter_cv_percent")
    if count >= 10 and jitter is not None and jitter > options.jitter_warning_percent:
        severity = "critical" if jitter > options.jitter_critical_percent else "warning"
        issues.append(Issue(
            issue_id=f"TOPIC_JITTER:{name}",
            severity=severity,
            category="控制周期" if _is_control_topic(name, str(topic.get("type", ""))) else "话题频率",
            topic=name,
            title="发布周期抖动偏高",
            evidence=(
                f"周期变异系数 {jitter:.1f}%，P95 周期 "
                f"{_format_optional(topic.get('p95_period_ms'), ' ms')}。"
            ),
            impact="控制环时延不稳定，传感器融合的时间插值误差增大。",
            suggestion="检查回调阻塞、线程优先级、CPU/磁盘负载和 executor 配置。",
            confidence="high",
        ))

    drop_ratio = float(topic.get("estimated_drop_ratio_percent", 0.0) or 0.0)
    if drop_ratio > options.drop_warning_percent:
        severity = "critical" if drop_ratio > options.drop_critical_percent else "warning"
        issues.append(Issue(
            issue_id=f"TOPIC_GAPS:{name}",
            severity=severity,
            category="消息丢包",
            topic=name,
            title="检测到疑似消息缺失或停发间隙",
            evidence=(
                f"估算缺失 {topic.get('estimated_drops', 0)} 条（{drop_ratio:.2f}%），"
                f"最大间隔 {_format_optional(topic.get('max_gap_ms'), ' ms')}。"
            ),
            impact="下游算法可能使用陈旧数据或触发超时。",
            suggestion="对照发布端序号/日志，检查 QoS reliability、队列深度、网络和录包磁盘写入。",
            confidence="medium",
        ))
    if int(topic.get("backward_timestamps", 0)):
        issues.append(Issue(
            issue_id=f"STORAGE_TIME_BACKWARDS:{name}",
            severity="critical",
            category="时间戳",
            topic=name,
            title="bag 存储时间戳发生回退",
            evidence=f"按录制顺序检测到 {topic['backward_timestamps']} 次时间回退。",
            impact="回放顺序、同步器和 TF 缓存可能出现时序错乱。",
            suggestion="检查系统时钟跳变、NTP/PTP 配置和 rosbag 写入时间源。",
            confidence="high",
        ))
    duplicate_count = int(topic.get("duplicate_timestamps", 0))
    if count > 1 and duplicate_count / (count - 1) > 0.01:
        issues.append(Issue(
            issue_id=f"STORAGE_TIME_DUPLICATE:{name}",
            severity="warning",
            category="时间戳",
            topic=name,
            title="大量消息共享同一存储时间戳",
            evidence=f"重复时间戳 {duplicate_count} 次，占相邻消息的 {duplicate_count / (count - 1) * 100:.2f}%。",
            impact="高频数据可能被批量写入，时间分辨率或时间源配置异常。",
            suggestion="检查驱动时间戳精度和录包时钟来源。",
            confidence="medium",
        ))
    return issues


def _header_timing_issues(
    topic: str,
    summary: dict[str, Any],
    options: DiagnosticOptions,
) -> list[Issue]:
    issues: list[Issue] = []
    decoded = int(summary.get("decoded_samples", 0))
    zero = int(summary.get("missing_or_zero_stamps", 0))
    if decoded and zero / decoded > 0.05:
        issues.append(Issue(
            issue_id=f"HEADER_STAMP_MISSING:{topic}",
            severity="warning",
            category="时间戳",
            topic=topic,
            title="消息 header 时间戳缺失或为零",
            evidence=f"抽样 {decoded} 条中 {zero} 条没有有效 header.stamp。",
            impact="message_filters、TF 对齐和多传感器同步可能失败。",
            suggestion="由采集驱动写入采样时刻，不要在下游节点用接收时刻替代。",
            confidence="high",
        ))
    backwards = int(summary.get("backward_stamps", 0))
    if backwards:
        issues.append(Issue(
            issue_id=f"HEADER_STAMP_BACKWARDS:{topic}",
            severity="critical",
            category="时间戳",
            topic=topic,
            title="消息 header 时间戳发生回退",
            evidence=f"抽样序列检测到 {backwards} 次回退。",
            impact="同步器会丢弃消息或产生负时间差。",
            suggestion="统一设备时间到 ROS 时间，检查计数器溢出、时钟复位和转换精度。",
            confidence="high",
        ))
    lag_p95 = summary.get("lag_p95_abs_ms")
    if lag_p95 is not None and lag_p95 > options.header_lag_warning_ms:
        severity = "critical" if lag_p95 > max(10_000.0, options.header_lag_warning_ms * 100.0) else "warning"
        issues.append(Issue(
            issue_id=f"HEADER_STORAGE_LAG:{topic}",
            severity=severity,
            category="话题延迟",
            topic=topic,
            title="采样时间与 bag 接收时间偏差较大",
            evidence=(
                f"|存储时间-header.stamp| P95 为 {lag_p95:.1f} ms，"
                f"中位偏差 {_format_optional(summary.get('lag_median_ms'), ' ms')}。"
            ),
            impact="多传感器融合、TF 插值和控制状态估计会错位。",
            suggestion="检查设备时钟同步、驱动 timestamp_mode、NTP/PTP 和传输排队。",
            confidence="high" if lag_p95 > 1000.0 else "medium",
        ))
    return issues


def _analyze_qos(topic: str, profiles: str) -> dict[str, Any]:
    if not profiles.strip():
        return {
            "status": "unknown",
            "profiles": [],
            "reliability": [],
            "durability": [],
            "risk": "录包未保存 offered QoS 元数据",
        }
    normalized = profiles.upper()
    reliabilities = []
    for token in ("RELIABLE", "BEST_EFFORT"):
        if token in normalized:
            reliabilities.append(token)
    durabilities = []
    for token in ("TRANSIENT_LOCAL", "VOLATILE"):
        if token in normalized:
            durabilities.append(token)
    profile_count = max(
        normalized.count("HISTORY:"),
        normalized.count("RELIABILITY:"),
        1,
    )
    risk = ""
    status = "recorded"
    if len(reliabilities) > 1 or len(durabilities) > 1:
        status = "mixed"
        risk = "同一话题记录到多种 QoS 策略，回放订阅端可能只兼容其中一部分"
    if topic.endswith("tf_static") and "VOLATILE" in durabilities:
        status = "risky"
        risk = "/tf_static 通常需要 TRANSIENT_LOCAL durability"
    return {
        "status": status,
        "profiles": [profiles],
        "profile_count": profile_count,
        "reliability": reliabilities,
        "durability": durabilities,
        "risk": risk,
    }


def _qos_issues(topic: str, qos: dict[str, Any]) -> list[Issue]:
    if qos["status"] not in {"mixed", "risky"}:
        return []
    return [Issue(
        issue_id=f"QOS_RISK:{topic}",
        severity="warning",
        category="QoS",
        topic=topic,
        title="话题存在 QoS 回放兼容风险",
        evidence=str(qos["risk"]),
        impact="回放时订阅端可能收不到消息，表现为节点正常但数据为零。",
        suggestion="生成 qos-profile-overrides.yaml，显式匹配目标订阅者的 reliability/durability/depth。",
        confidence="medium",
    )]


def _is_control_topic(topic: str, msgtype: str) -> bool:
    text = f"{topic} {msgtype}".lower()
    return bool(
        re.search(
            r"(^|[/_\s])(cmd_vel|command|setpoint|control|actuator|motor)([/_\s]|$)",
            text,
        )
    )


def _expected_minimum_hz(topic: str, msgtype: str, control_minimum_hz: float) -> float | None:
    lowered = msgtype.lower()
    if _is_control_topic(topic, msgtype):
        return control_minimum_hz
    if lowered.endswith("/imu"):
        return 50.0
    if lowered.endswith("/odometry"):
        return 10.0
    if lowered.endswith("/laserscan") or lowered.endswith("/pointcloud2"):
        return 5.0
    if topic.rstrip("/").endswith("/tf") or topic == "/tf":
        return 10.0
    return None


def _status_for_issues(issues: list[Issue]) -> str:
    if any(issue.severity == "critical" for issue in issues):
        return "critical"
    if any(issue.severity == "warning" for issue in issues):
        return "warning"
    if issues:
        return "notice"
    return "normal"


def _severity_rank(severity: str) -> int:
    return {"critical": 0, "warning": 1, "notice": 2}.get(severity, 3)


def _deduplicate_issues(issues: list[Issue]) -> list[Issue]:
    unique: dict[str, Issue] = {}
    for issue in issues:
        existing = unique.get(issue.issue_id)
        if existing is None or _severity_rank(issue.severity) < _severity_rank(existing.severity):
            unique[issue.issue_id] = issue
    return list(unique.values())


def _build_summary(
    issues: list[Issue],
    *,
    observed_messages: int,
    decoded_messages: int,
    decoded_topics: int,
    total_topics: int,
) -> dict[str, Any]:
    counts = {
        severity: sum(issue.severity == severity for issue in issues)
        for severity in ("critical", "warning", "notice")
    }
    deduction = counts["critical"] * 18 + counts["warning"] * 7 + counts["notice"] * 1
    score = max(0, min(100, 100 - deduction))
    status = "critical" if counts["critical"] else "warning" if counts["warning"] else "healthy"
    return {
        "score": score,
        "status": status,
        "critical_count": counts["critical"],
        "warning_count": counts["warning"],
        "notice_count": counts["notice"],
        "observed_messages": observed_messages,
        "decoded_messages": decoded_messages,
        "decoded_topics": decoded_topics,
        "total_topics": total_topics,
        "deep_analysis_coverage_percent": decoded_topics / total_topics * 100.0 if total_topics else 0.0,
    }


def _bag_to_dict(
    source: BagSource,
    observed_messages: int,
    decoded_messages: int,
    decoded_topics: set[str],
) -> dict[str, Any]:
    info = source.info
    return {
        "path": str(info.path),
        "ros_version": info.ros_version,
        "storage": info.storage,
        "reader": info.reader,
        "files": [str(path) for path in info.files],
        "size_bytes": info.size_bytes,
        "message_count": observed_messages,
        "declared_message_count": info.message_count,
        "topic_count": len(info.topics),
        "start_time_ns": info.start_time_ns,
        "end_time_ns": info.end_time_ns,
        "duration_sec": info.duration_ns / 1_000_000_000.0,
        "start_time": _ns_to_iso(info.start_time_ns),
        "end_time": _ns_to_iso(info.end_time_ns),
        "compression_format": info.compression_format,
        "compression_mode": info.compression_mode,
        "deep_analysis": decoded_messages > 0,
        "decoded_topics": sorted(decoded_topics),
        "decode_warnings": source.decode_warnings,
    }


def _ns_to_iso(value: int | None) -> str | None:
    if value is None:
        return None
    try:
        return datetime.fromtimestamp(value / 1_000_000_000.0, timezone.utc).astimezone().isoformat()
    except (OSError, OverflowError, ValueError):
        return None


def _recommendations(issues: list[Issue]) -> list[str]:
    ordered: list[str] = []
    for issue in issues:
        if issue.suggestion and issue.suggestion not in ordered:
            ordered.append(issue.suggestion)
    if not ordered:
        ordered.append("当前阈值下未发现明确异常；仍建议用设备规格和任务要求复核关键频率。")
    return ordered[:12]


def _format_optional(value: Any, suffix: str) -> str:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "—"
    if not math.isfinite(numeric):
        return "—"
    return f"{numeric:.2f}{suffix}"
