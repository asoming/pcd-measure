"""Heuristic sensor-health and hardware-fault pattern analysis."""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field
from statistics import median
from typing import Any

from .message_utils import attr, magnitude, quaternion, sequence, text, vector3
from .metrics import ScalarAccumulator, median_absolute_deviation, percentile
from .model import Issue


def _role_for(msgtype: str, topic: str) -> str | None:
    normalized = msgtype.lower()
    topic_lower = topic.lower()
    if normalized.endswith("/imu"):
        return "IMU"
    if normalized.endswith("/laserscan"):
        return "激光扫描"
    if normalized.endswith("/pointcloud2") or "pointcloud" in normalized:
        return "点云"
    if "livox" in normalized and normalized.endswith("/custommsg"):
        return "Livox 雷达"
    if normalized.endswith("/image") or normalized.endswith("/compressedimage"):
        return "图像"
    if normalized.endswith("/odometry"):
        return "里程计"
    if normalized.endswith("/jointstate"):
        return "关节状态"
    if normalized.endswith("/navsatfix"):
        return "GNSS"
    if normalized.endswith("/batterystate"):
        return "电池"
    if normalized.endswith("/diagnosticarray") or topic_lower == "/diagnostics":
        return "硬件诊断"
    return None


def _finite_sequence(values: Any, maximum: int = 4096) -> tuple[list[float], int]:
    finite: list[float] = []
    invalid = 0
    for index, value in enumerate(sequence(values)):
        if index >= maximum:
            break
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            invalid += 1
            continue
        if math.isfinite(numeric):
            finite.append(numeric)
        else:
            invalid += 1
    return finite, invalid


def _payload_signature(value: Any) -> str:
    try:
        payload = bytes(value)
    except (TypeError, ValueError):
        return ""
    if not payload:
        return "empty"
    sample = payload if len(payload) <= 4096 else payload[:2048] + payload[-2048:]
    return f"{len(payload)}:{hashlib.blake2b(sample, digest_size=8).hexdigest()}"


@dataclass
class _SensorState:
    topic: str
    msgtype: str
    role: str
    samples: int = 0
    invalid_samples: int = 0
    empty_samples: int = 0
    malformed_samples: int = 0
    saturated_samples: int = 0
    repeated_samples: int = 0
    maximum_repeat_run: int = 0
    current_repeat_run: int = 0
    previous_signature: str = ""
    metrics: dict[str, ScalarAccumulator] = field(default_factory=dict)
    counters: dict[str, int] = field(default_factory=dict)
    previous_values: dict[str, Any] = field(default_factory=dict)

    def metric(self, name: str) -> ScalarAccumulator:
        return self.metrics.setdefault(name, ScalarAccumulator())

    def observe_signature(self, signature: str) -> None:
        if not signature:
            return
        if signature == self.previous_signature:
            self.repeated_samples += 1
            self.current_repeat_run += 1
            self.maximum_repeat_run = max(self.maximum_repeat_run, self.current_repeat_run)
        else:
            self.current_repeat_run = 0
        self.previous_signature = signature


class SensorAnalyzer:
    """Analyze common standard messages without depending on ROS classes."""

    def __init__(self) -> None:
        self._states: dict[str, _SensorState] = {}
        self._diagnostic_faults: dict[tuple[str, int, str], int] = {}

    def consume(self, topic: str, msgtype: str, message: Any) -> bool:
        role = _role_for(msgtype, topic)
        if role is None:
            return False
        state = self._states.setdefault(topic, _SensorState(topic, msgtype, role))
        state.samples += 1
        if role == "IMU":
            self._consume_imu(state, message)
        elif role == "激光扫描":
            self._consume_scan(state, message)
        elif role == "点云":
            self._consume_point_cloud(state, message)
        elif role == "Livox 雷达":
            self._consume_livox(state, message)
        elif role == "图像":
            self._consume_image(state, message)
        elif role == "里程计":
            self._consume_odometry(state, message)
        elif role == "关节状态":
            self._consume_joint_state(state, message)
        elif role == "GNSS":
            self._consume_gnss(state, message)
        elif role == "电池":
            self._consume_battery(state, message)
        elif role == "硬件诊断":
            self._consume_diagnostics(state, message)
        return True

    @staticmethod
    def _consume_imu(state: _SensorState, message: Any) -> None:
        acceleration = vector3(attr(message, "linear_acceleration"))
        angular_velocity = vector3(attr(message, "angular_velocity"))
        orientation = quaternion(attr(message, "orientation"))
        if acceleration is None or angular_velocity is None:
            state.invalid_samples += 1
            return
        acceleration_magnitude = magnitude(acceleration)
        angular_magnitude = magnitude(angular_velocity)
        state.metric("acceleration_magnitude_mps2").add(acceleration_magnitude)
        state.metric("angular_velocity_magnitude_radps").add(angular_magnitude)
        if orientation is not None:
            orientation_norm = magnitude(orientation)
            state.metric("orientation_norm").add(orientation_norm)
            state.metric("orientation_norm_error").add(abs(orientation_norm - 1.0))
        previous_acceleration = state.previous_values.get("acceleration")
        previous_angular = state.previous_values.get("angular_velocity")
        if previous_acceleration is not None:
            state.metric("acceleration_step_mps2").add(
                magnitude(a - b for a, b in zip(acceleration, previous_acceleration))
            )
        if previous_angular is not None:
            state.metric("angular_velocity_step_radps").add(
                magnitude(a - b for a, b in zip(angular_velocity, previous_angular))
            )
        state.previous_values["acceleration"] = acceleration
        state.previous_values["angular_velocity"] = angular_velocity
        if acceleration_magnitude > 100.0 or angular_magnitude > 35.0:
            state.saturated_samples += 1
        signature = ":".join(f"{value:.7g}" for value in acceleration + angular_velocity)
        state.observe_signature(signature)

    @staticmethod
    def _consume_scan(state: _SensorState, message: Any) -> None:
        raw_ranges = sequence(attr(message, "ranges"))
        if not raw_ranges:
            state.empty_samples += 1
            return
        ranges, invalid = _finite_sequence(raw_ranges)
        range_min = float(attr(message, "range_min", 0.0) or 0.0)
        range_max = float(attr(message, "range_max", float("inf")) or float("inf"))
        outside = sum(value < range_min or value > range_max for value in ranges)
        state.metric("points_per_scan").add(len(raw_ranges))
        state.metric("invalid_range_ratio_percent").add(
            (invalid + outside) / max(1, min(len(raw_ranges), 4096)) * 100.0
        )
        if not ranges:
            state.empty_samples += 1
        signature_values = ranges[:16] + ranges[-16:]
        state.observe_signature(
            f"{len(raw_ranges)}:" + ":".join(f"{value:.5g}" for value in signature_values)
        )

    @staticmethod
    def _consume_point_cloud(state: _SensorState, message: Any) -> None:
        width = int(attr(message, "width", 0) or 0)
        height = int(attr(message, "height", 0) or 0)
        point_step = int(attr(message, "point_step", 0) or 0)
        row_step = int(attr(message, "row_step", 0) or 0)
        data = attr(message, "data", b"")
        try:
            data_length = len(data)
        except TypeError:
            data_length = 0
        points = width * height
        state.metric("points_per_frame").add(points)
        if points <= 0 or data_length <= 0:
            state.empty_samples += 1
        expected_minimum = row_step * height if row_step and height else points * point_step
        if point_step <= 0 or (expected_minimum > 0 and data_length < expected_minimum):
            state.malformed_samples += 1
        state.observe_signature(_payload_signature(data))

    @staticmethod
    def _consume_livox(state: _SensorState, message: Any) -> None:
        declared_points = int(attr(message, "point_num", 0) or 0)
        points = sequence(attr(message, "points"))
        state.metric("points_per_frame").add(declared_points or len(points))
        if declared_points <= 0 or not points:
            state.empty_samples += 1
            return
        if declared_points != len(points):
            state.malformed_samples += 1
        invalid = 0
        distances: list[float] = []
        reflectivities: list[float] = []
        for point in points[:2048]:
            position = vector3(point)
            if position is None:
                invalid += 1
                continue
            distances.append(magnitude(position))
            try:
                reflectivities.append(float(attr(point, "reflectivity", 0)))
            except (TypeError, ValueError):
                invalid += 1
        if invalid:
            state.invalid_samples += 1
            state.counters["invalid_points"] = state.counters.get("invalid_points", 0) + invalid
        if distances:
            state.metric("range_m").add(median(distances))
        if reflectivities:
            state.metric("reflectivity").add(median(reflectivities))
        timebase = attr(message, "timebase")
        if timebase is not None:
            state.metric("timebase_ns").add(timebase)
        signature_points = points[:4] + points[-4:]
        signature = [str(declared_points)]
        for point in signature_points:
            position = vector3(point)
            if position is not None:
                signature.extend(f"{value:.4g}" for value in position)
        state.observe_signature(":".join(signature))

    @staticmethod
    def _consume_image(state: _SensorState, message: Any) -> None:
        width = int(attr(message, "width", 0) or 0)
        height = int(attr(message, "height", 0) or 0)
        step = int(attr(message, "step", 0) or 0)
        data = attr(message, "data", b"")
        try:
            data_length = len(data)
        except TypeError:
            data_length = 0
        state.metric("pixels_per_frame").add(width * height)
        state.metric("payload_bytes").add(data_length)
        if width <= 0 or height <= 0 or data_length <= 0:
            state.empty_samples += 1
        if step > 0 and height > 0 and data_length < step * height:
            state.malformed_samples += 1
        state.observe_signature(_payload_signature(data))

    @staticmethod
    def _consume_odometry(state: _SensorState, message: Any) -> None:
        pose = attr(attr(message, "pose"), "pose", attr(message, "pose"))
        position = vector3(attr(pose, "position"))
        if position is None:
            state.invalid_samples += 1
            return
        previous = state.previous_values.get("position")
        if previous is not None:
            state.metric("position_step_m").add(
                magnitude(a - b for a, b in zip(position, previous))
            )
        state.previous_values["position"] = position
        state.observe_signature(":".join(f"{value:.6g}" for value in position))

    @staticmethod
    def _consume_joint_state(state: _SensorState, message: Any) -> None:
        names = sequence(attr(message, "name"))
        positions, invalid = _finite_sequence(attr(message, "position"))
        velocities, velocity_invalid = _finite_sequence(attr(message, "velocity"))
        state.metric("joint_count").add(len(names))
        if invalid or velocity_invalid:
            state.invalid_samples += 1
        if positions and names and len(positions) != len(names):
            state.malformed_samples += 1
        if velocities and names and len(velocities) != len(names):
            state.malformed_samples += 1
        state.observe_signature(
            f"{len(names)}:" + ":".join(f"{value:.6g}" for value in positions[:16])
        )

    @staticmethod
    def _consume_gnss(state: _SensorState, message: Any) -> None:
        latitude = attr(message, "latitude")
        longitude = attr(message, "longitude")
        altitude = attr(message, "altitude")
        values = []
        for value in (latitude, longitude, altitude):
            try:
                values.append(float(value))
            except (TypeError, ValueError):
                values.append(float("nan"))
        if not all(math.isfinite(value) for value in values):
            state.invalid_samples += 1
        raw_status = attr(attr(message, "status"), "status", -1)
        status = int(raw_status if raw_status is not None else -1)
        if status < 0:
            state.counters["no_fix"] = state.counters.get("no_fix", 0) + 1
        covariance, covariance_invalid = _finite_sequence(attr(message, "position_covariance"))
        if covariance_invalid:
            state.invalid_samples += 1
        if covariance:
            state.metric("position_covariance_max").add(max(covariance))
        state.observe_signature(":".join(f"{value:.8g}" for value in values))

    @staticmethod
    def _consume_battery(state: _SensorState, message: Any) -> None:
        for field_name in ("voltage", "current", "percentage", "temperature"):
            value = attr(message, field_name)
            if value is not None:
                state.metric(field_name).add(value)
        try:
            voltage = float(attr(message, "voltage", float("nan")))
        except (TypeError, ValueError):
            voltage = float("nan")
        if not math.isfinite(voltage) or voltage <= 0.0:
            state.invalid_samples += 1

    def _consume_diagnostics(self, state: _SensorState, message: Any) -> None:
        statuses = sequence(attr(message, "status"))
        if not statuses:
            state.empty_samples += 1
            return
        for status in statuses:
            level = int(attr(status, "level", 0) or 0)
            if level < 1:
                continue
            name = text(attr(status, "name", "未命名硬件"))
            message_text = text(attr(status, "message", ""))
            key = (name, level, message_text)
            self._diagnostic_faults[key] = self._diagnostic_faults.get(key, 0) + 1

    def finish(self) -> tuple[list[dict[str, Any]], list[Issue]]:
        results: list[dict[str, Any]] = []
        issues: list[Issue] = []
        for topic, state in sorted(self._states.items()):
            metrics = {name: accumulator.summary() for name, accumulator in state.metrics.items()}
            result = {
                "topic": topic,
                "type": state.msgtype,
                "role": state.role,
                "decoded_samples": state.samples,
                "invalid_samples": state.invalid_samples,
                "empty_samples": state.empty_samples,
                "malformed_samples": state.malformed_samples,
                "saturated_samples": state.saturated_samples,
                "repeated_samples": state.repeated_samples,
                "maximum_repeat_run": state.maximum_repeat_run,
                "counters": state.counters,
                "metrics": metrics,
                "status": "normal",
            }
            topic_issues = self._issues_for_state(state, metrics)
            if any(issue.severity == "critical" for issue in topic_issues):
                result["status"] = "critical"
            elif any(issue.severity == "warning" for issue in topic_issues):
                result["status"] = "warning"
            elif topic_issues:
                result["status"] = "notice"
            results.append(result)
            issues.extend(topic_issues)

        for (name, level, message_text), count in sorted(self._diagnostic_faults.items()):
            severity = "critical" if level >= 2 else "warning"
            issues.append(Issue(
                issue_id=f"HARDWARE_DIAGNOSTIC:{name}:{level}",
                severity=severity,
                category="硬件",
                topic="/diagnostics",
                title=f"硬件诊断上报{'错误' if level >= 2 else '警告'}：{name}",
                evidence=f"出现 {count} 次；消息：{message_text or '未提供详情'}。",
                impact="设备驱动已经主动报告异常状态。",
                suggestion="优先检查该硬件的供电、连接、温度、驱动日志和诊断键值。",
                confidence="high",
            ))
        return results, issues

    @staticmethod
    def _issues_for_state(
        state: _SensorState,
        metrics: dict[str, dict[str, Any]],
    ) -> list[Issue]:
        issues: list[Issue] = []
        if state.invalid_samples:
            ratio = state.invalid_samples / max(1, state.samples) * 100.0
            severity = "critical" if ratio > 20.0 else "warning"
            issues.append(Issue(
                issue_id=f"SENSOR_INVALID:{state.topic}",
                severity=severity,
                category="传感器",
                topic=state.topic,
                title=f"{state.role}消息包含无效数值",
                evidence=f"抽样 {state.samples} 条中 {state.invalid_samples} 条无效（{ratio:.1f}%）。",
                impact="NaN、Inf 或缺失字段会污染滤波、定位和控制结果。",
                suggestion="检查驱动有效性标志、标定参数、数据类型和异常值过滤。",
                confidence="high",
            ))
        if state.empty_samples or state.malformed_samples:
            severity = "critical" if state.malformed_samples else "warning"
            issues.append(Issue(
                issue_id=f"SENSOR_EMPTY_OR_MALFORMED:{state.topic}",
                severity=severity,
                category="传感器",
                topic=state.topic,
                title=f"{state.role}存在空帧或数据长度异常",
                evidence=(
                    f"空帧 {state.empty_samples}，结构/载荷不一致 "
                    f"{state.malformed_samples}。"
                ),
                impact="下游节点可能丢帧、越界或输出不完整结果。",
                suggestion="核对 width/height/step/data 字段和设备传输链路。",
                confidence="high",
            ))
        if state.saturated_samples:
            issues.append(Issue(
                issue_id=f"SENSOR_SATURATION:{state.topic}",
                severity="warning",
                category="传感器",
                topic=state.topic,
                title=f"{state.role}出现疑似量程饱和",
                evidence=f"抽样中有 {state.saturated_samples} 条超过通用物理阈值。",
                impact="饱和数据会导致积分漂移或姿态估计失败。",
                suggestion="检查量程配置、单位、机械冲击和安装刚性。",
                confidence="medium",
            ))
        if state.maximum_repeat_run >= 10:
            state_can_legitimately_hold = state.role in {"里程计", "关节状态"}
            issues.append(Issue(
                issue_id=f"SENSOR_FROZEN:{state.topic}",
                severity="notice" if state_can_legitimately_hold else "warning",
                category="传感器" if state_can_legitimately_hold else "硬件",
                topic=state.topic,
                title=(
                    f"{state.role}长时间保持相同状态"
                    if state_can_legitimately_hold
                    else f"{state.role}疑似冻结或重复帧"
                ),
                evidence=f"连续相同样本最长 {state.maximum_repeat_run + 1} 帧。",
                impact=(
                    "机器人静止时可能完全正常；若当时有运动指令，则数据可能停滞。"
                    if state_can_legitimately_hold
                    else "话题仍有频率，但硬件数据可能已经停止更新。"
                ),
                suggestion=(
                    "结合 cmd_vel、TF 和现场运动状态复核，不单凭重复值判定硬件故障。"
                    if state_can_legitimately_hold
                    else "检查设备心跳、DMA/USB/网络传输和驱动缓存复用。"
                ),
                confidence="low" if state_can_legitimately_hold else "medium",
            ))

        if state.role == "IMU":
            orientation_error = metrics.get("orientation_norm_error", {}).get("p95")
            if orientation_error is not None and orientation_error > 0.05:
                issues.append(Issue(
                    issue_id=f"IMU_QUATERNION_NORM:{state.topic}",
                    severity="warning",
                    category="传感器",
                    topic=state.topic,
                    title="IMU 四元数未正确归一化",
                    evidence=f"四元数模长误差 P95 为 {orientation_error:.4f}。",
                    impact="姿态旋转和融合结果会产生系统误差。",
                    suggestion="在驱动输出或融合入口归一化四元数，并检查无效姿态标记。",
                    confidence="high",
                ))
            acceleration_steps = state.metrics.get("acceleration_step_mps2")
            angular_steps = state.metrics.get("angular_velocity_step_radps")
            acceleration_noise = (
                median_absolute_deviation(acceleration_steps.values)
                if acceleration_steps
                else None
            )
            angular_noise = (
                median_absolute_deviation(angular_steps.values) if angular_steps else None
            )
            if (acceleration_noise or 0.0) > 1.0 or (angular_noise or 0.0) > 0.2:
                issues.append(Issue(
                    issue_id=f"IMU_HIGH_SHORT_TERM_VARIATION:{state.topic}",
                    severity="notice",
                    category="传感器",
                    topic=state.topic,
                    title="IMU 短期变化较大",
                    evidence=(
                        f"相邻差分 MAD：加速度 {acceleration_noise or 0.0:.3f} m/s²，"
                        f"角速度 {angular_noise or 0.0:.3f} rad/s。"
                    ),
                    impact="可能是振动/噪声，也可能来自真实快速运动。",
                    suggestion="选择静止片段复核零偏与噪声，并检查减振、采样率和滤波参数。",
                    confidence="low",
                ))
        elif state.role == "激光扫描":
            invalid_p95 = metrics.get("invalid_range_ratio_percent", {}).get("p95")
            if invalid_p95 is not None and invalid_p95 > 30.0:
                issues.append(Issue(
                    issue_id=f"LIDAR_INVALID_RANGES:{state.topic}",
                    severity="warning",
                    category="传感器",
                    topic=state.topic,
                    title="激光扫描无效距离比例偏高",
                    evidence=f"单帧无效比例 P95 为 {invalid_p95:.1f}%。",
                    impact="定位和避障可用特征减少，可能出现局部失锁。",
                    suggestion="检查遮挡、反光表面、range_min/max 和雷达清洁度。",
                    confidence="medium",
                ))
        elif state.role == "里程计":
            max_step = metrics.get("position_step_m", {}).get("maximum")
            if max_step is not None and max_step > 2.0:
                issues.append(Issue(
                    issue_id=f"ODOMETRY_POSITION_JUMP:{state.topic}",
                    severity="warning",
                    category="传感器",
                    topic=state.topic,
                    title="里程计位置出现大幅跳变",
                    evidence=f"抽样相邻位置最大变化 {max_step:.3f} m。",
                    impact="可能导致局部规划和地图配准瞬移。",
                    suggestion="核对重定位/复位事件、坐标系和时间戳；提高载荷抽样后复查原始时刻。",
                    confidence="medium",
                ))
        elif state.role == "GNSS":
            no_fix = state.counters.get("no_fix", 0)
            if no_fix:
                issues.append(Issue(
                    issue_id=f"GNSS_NO_FIX:{state.topic}",
                    severity="warning",
                    category="传感器",
                    topic=state.topic,
                    title="GNSS 存在无定位状态",
                    evidence=f"抽样中 {no_fix}/{state.samples} 条状态为 NO_FIX。",
                    impact="依赖 GNSS 的定位或地图对齐在这些时段不可用。",
                    suggestion="检查天线视野、差分源、串口链路和状态过滤。",
                    confidence="high",
                ))
        elif state.role == "电池":
            minimum_charge = metrics.get("percentage", {}).get("minimum")
            maximum_temperature = metrics.get("temperature", {}).get("maximum")
            if minimum_charge is not None and 0.0 <= minimum_charge < 0.1:
                issues.append(Issue(
                    issue_id=f"BATTERY_LOW:{state.topic}",
                    severity="warning",
                    category="硬件",
                    topic=state.topic,
                    title="电池电量过低",
                    evidence=f"抽样最低电量为 {minimum_charge * 100.0:.1f}%。",
                    impact="设备可能降频、掉电或在写 bag 时异常关机。",
                    suggestion="检查电池健康、供电压降和低电量保护阈值，并在复现前充电。",
                    confidence="high",
                ))
            if maximum_temperature is not None and maximum_temperature > 60.0:
                issues.append(Issue(
                    issue_id=f"BATTERY_TEMPERATURE_HIGH:{state.topic}",
                    severity="critical" if maximum_temperature > 80.0 else "warning",
                    category="硬件",
                    topic=state.topic,
                    title="电池温度过高",
                    evidence=f"抽样最高温度为 {maximum_temperature:.1f} °C。",
                    impact="可能触发保护、供电不稳或造成硬件安全风险。",
                    suggestion="停止高负载运行，检查散热、环境温度、电芯与温度单位配置。",
                    confidence="high",
                ))
        return issues
