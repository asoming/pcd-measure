#!/usr/bin/env python3
"""Command-line entry point for ROS bag offline diagnostics."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from rosbag_diag import DiagnosticOptions, diagnose_bag
from rosbag_diag.model import BagDiagnosticError
from rosbag_diag.reporting import report_json, write_html_report, write_json_report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="离线诊断 ROS1/ROS2 bag 的时序、TF、QoS 和传感器健康状态。"
    )
    parser.add_argument("bag", help="ROS1 .bag、ROS2 bag 目录、.db3 或 .mcap")
    parser.add_argument("--json", dest="json_path", help="写入 JSON 报告")
    parser.add_argument("--html", dest="html_path", help="写入自包含 HTML 报告")
    parser.add_argument("--no-deep", action="store_true", help="不反序列化消息载荷")
    parser.add_argument("--gap-factor", type=float, default=3.0, help="长间隙相对中位周期倍数")
    parser.add_argument("--jitter-warning", type=float, default=20.0, help="抖动 CV 警告百分比")
    parser.add_argument("--jitter-critical", type=float, default=60.0, help="抖动 CV 严重百分比")
    parser.add_argument("--control-min-hz", type=float, default=10.0, help="控制话题最低频率")
    parser.add_argument("--tf-jump-m", type=float, default=0.5, help="TF 平移跳变基础阈值")
    parser.add_argument("--tf-jump-deg", type=float, default=30.0, help="TF 旋转跳变基础阈值")
    parser.add_argument("--payload-samples", type=int, default=2000, help="每话题最大载荷样本")
    parser.add_argument(
        "--progress",
        action="store_true",
        help="在 stderr 输出 PROGRESS\\t百分比\\t状态，供 GUI 使用",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    options = DiagnosticOptions(
        gap_factor=args.gap_factor,
        jitter_warning_percent=args.jitter_warning,
        jitter_critical_percent=args.jitter_critical,
        control_minimum_hz=args.control_min_hz,
        tf_translation_jump_m=args.tf_jump_m,
        tf_rotation_jump_deg=args.tf_jump_deg,
        maximum_payload_samples_per_topic=args.payload_samples,
        deep_analysis=not args.no_deep,
    )

    last_progress = [-1, ""]

    def progress(percent: int, message: str) -> None:
        if args.progress and [percent, message] != last_progress:
            print(f"PROGRESS\t{percent}\t{message}", file=sys.stderr, flush=True)
            last_progress[:] = [percent, message]

    try:
        report = diagnose_bag(Path(args.bag), options, progress)
        if args.json_path:
            write_json_report(report, args.json_path)
        if args.html_path:
            write_html_report(report, args.html_path)
        if not args.json_path and not args.html_path:
            print(report_json(report))
    except (BagDiagnosticError, OSError, ValueError) as exc:
        print(f"诊断失败：{exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
