"""JSON and self-contained HTML output for bag diagnostics."""

from __future__ import annotations

import html
import json
import math
import os
import tempfile
from pathlib import Path
from typing import Any

from .model import DiagnosticReport


def report_json(report: DiagnosticReport, *, pretty: bool = True) -> str:
    """Serialize a report without non-standard NaN/Infinity values."""

    return json.dumps(
        _finite_json(report.to_dict()),
        ensure_ascii=False,
        indent=2 if pretty else None,
        allow_nan=False,
    )


def write_json_report(report: DiagnosticReport, path: str | Path) -> Path:
    destination = Path(path).expanduser().resolve()
    _atomic_write(destination, report_json(report) + "\n")
    return destination


def write_html_report(report: DiagnosticReport, path: str | Path) -> Path:
    destination = Path(path).expanduser().resolve()
    _atomic_write(destination, build_html_report(report))
    return destination


def build_html_report(report: DiagnosticReport) -> str:
    data = _finite_json(report.to_dict())
    bag = data["bag"]
    summary = data["summary"]
    topics = data["topics"]
    issues = data["issues"]
    tf_data = data["tf"]
    sensors = data["sensors"]

    status_label = {
        "healthy": "未发现明确故障",
        "warning": "需要复核",
        "critical": "存在严重异常",
    }.get(summary["status"], summary["status"])
    issue_cards = "".join(
        f"""
        <article class="issue {html.escape(issue['severity'])}">
          <div class="issue-head"><span>{_severity_label(issue['severity'])}</span>
            <strong>{html.escape(issue['title'])}</strong></div>
          <div class="tagline">{html.escape(issue.get('category', ''))} · {html.escape(issue.get('topic') or '全局')}</div>
          <p><b>证据</b>　{html.escape(issue.get('evidence', ''))}</p>
          <p><b>建议</b>　{html.escape(issue.get('suggestion', ''))}</p>
        </article>
        """
        for issue in issues
    ) or '<div class="empty">当前阈值下没有诊断项。</div>'

    topic_rows = "".join(
        "<tr>"
        f"<td><code>{html.escape(topic['name'])}</code><small>{html.escape(topic.get('type', ''))}</small></td>"
        f"<td>{_number(topic.get('count'), 0)}</td>"
        f"<td>{_number(topic.get('mean_hz'), 2)}</td>"
        f"<td>{_number(topic.get('jitter_cv_percent'), 1)}%</td>"
        f"<td>{_number(topic.get('max_gap_ms'), 1)} ms</td>"
        f"<td>{_number(topic.get('estimated_drops'), 0)}</td>"
        f"<td>{html.escape(topic.get('qos', {}).get('status', 'unknown'))}</td>"
        f'<td><span class="pill {html.escape(topic.get("status", "normal"))}">{_status_label(topic.get("status", "normal"))}</span></td>'
        "</tr>"
        for topic in topics
    )

    tf_rows = "".join(
        "<tr>"
        f"<td><code>{html.escape(edge['parent'])} → {html.escape(edge['child'])}</code></td>"
        f"<td>{'静态' if edge.get('static') else '动态'}</td>"
        f"<td>{_number(edge.get('samples'), 0)}</td>"
        f"<td>{_number(edge.get('net_translation_m'), 3)} m</td>"
        f"<td>{_number(edge.get('max_step_m'), 3)} m</td>"
        f"<td>{_number(edge.get('max_rotation_step_deg'), 2)}°</td>"
        f"<td>{_number(edge.get('max_gap_ms'), 1)} ms</td>"
        "</tr>"
        for edge in tf_data.get("edges", [])
    ) or '<tr><td colspan="7" class="empty">没有可解码的 TF 边。</td></tr>'

    sensor_rows = "".join(
        "<tr>"
        f"<td><code>{html.escape(sensor['topic'])}</code></td>"
        f"<td>{html.escape(sensor['role'])}</td>"
        f"<td>{_number(sensor.get('decoded_samples'), 0)}</td>"
        f"<td>{_number(sensor.get('invalid_samples'), 0)}</td>"
        f"<td>{_number(sensor.get('empty_samples'), 0)}</td>"
        f"<td>{_number(sensor.get('maximum_repeat_run'), 0)}</td>"
        f'<td><span class="pill {html.escape(sensor.get("status", "normal"))}">{_status_label(sensor.get("status", "normal"))}</span></td>'
        "</tr>"
        for sensor in sensors
    ) or '<tr><td colspan="7" class="empty">没有可解码的标准传感器消息。</td></tr>'

    recommendations = "".join(
        f"<li>{html.escape(item)}</li>" for item in data["recommendations"]
    )
    limitations = "".join(f"<li>{html.escape(item)}</li>" for item in data["limitations"])
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ROS Bag 离线诊断报告</title>
<style>
:root{{--bg:#06121b;--panel:#0c2230;--panel2:#102b3a;--line:#294b5d;--text:#dce9ed;--muted:#8ca8b5;--cyan:#36d6d0;--amber:#ffb85c;--red:#ff6f66;--blue:#65aee8}}
*{{box-sizing:border-box}} body{{margin:0;background:var(--bg);color:var(--text);font:14px/1.55 "Noto Sans CJK SC","Microsoft YaHei",sans-serif}}
main{{width:min(1400px,94vw);margin:32px auto 64px}} header{{border-top:3px solid var(--cyan);padding:24px 0 18px;display:flex;justify-content:space-between;gap:24px;align-items:flex-end}}
h1{{font-size:30px;letter-spacing:.04em;margin:0}} h2{{font-size:18px;margin:30px 0 12px;color:#eefbfd}} h3{{margin:0}} code,.mono{{font-family:"DejaVu Sans Mono",monospace}} .eyebrow{{color:var(--cyan);font:700 11px/1 monospace;letter-spacing:.18em;margin-bottom:10px}}
.path{{color:var(--muted);word-break:break-all}} .score{{font:800 58px/1 monospace;color:var(--cyan);text-align:right}} .score small{{font:600 13px/1.2 sans-serif;display:block;color:var(--muted);margin-top:8px}}
.grid{{display:grid;grid-template-columns:repeat(6,1fr);gap:10px}} .metric{{background:var(--panel);border:1px solid var(--line);padding:14px 15px;border-radius:8px;min-height:82px}} .metric span{{display:block;color:var(--muted);font-size:12px}} .metric strong{{display:block;margin-top:9px;font:700 19px/1 monospace}}
.issues{{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}} .issue{{background:var(--panel);border:1px solid var(--line);border-left:4px solid var(--blue);padding:14px 16px;border-radius:6px}} .issue.warning{{border-left-color:var(--amber)}} .issue.critical{{border-left-color:var(--red)}} .issue-head{{display:flex;align-items:center;gap:10px}} .issue-head span{{font:700 10px/1 monospace;padding:5px 7px;border:1px solid currentColor;border-radius:4px}} .tagline{{color:var(--muted);font-size:12px;margin:6px 0 9px}} .issue p{{margin:6px 0}}
.table-wrap{{overflow:auto;border:1px solid var(--line);border-radius:8px}} table{{width:100%;border-collapse:collapse;background:var(--panel)}} th{{position:sticky;top:0;background:#153444;color:#a9c1cb;text-align:left;padding:10px;border-bottom:1px solid var(--line);font-size:12px}} td{{padding:9px 10px;border-bottom:1px solid #183646;white-space:nowrap}} td:first-child{{white-space:normal}} td small{{display:block;color:var(--muted);margin-top:3px}} tr:last-child td{{border-bottom:0}}
.pill{{display:inline-block;padding:3px 7px;border-radius:999px;background:#173d48;color:#83e4df;font-size:11px}} .pill.warning{{background:#4a351d;color:#ffd08f}} .pill.critical{{background:#482526;color:#ffaaa4}} .pill.notice{{background:#17344a;color:#9dcdf2}}
.note{{background:var(--panel2);border:1px solid var(--line);padding:16px 20px;border-radius:8px}} li{{margin:5px 0}} .empty{{color:var(--muted);padding:18px;text-align:center}} footer{{color:var(--muted);margin-top:36px;border-top:1px solid var(--line);padding-top:14px}}
@media(max-width:900px){{.grid{{grid-template-columns:repeat(2,1fr)}}.issues{{grid-template-columns:1fr}}header{{align-items:flex-start;flex-direction:column}}.score{{text-align:left}}}}
@media print{{body{{background:white;color:#17232b}}main{{width:100%;margin:0}}.metric,.issue,.table-wrap,table,.note{{background:white}}th{{background:#e9f0f3;color:#263943}}.path,.tagline,footer{{color:#526975}}}}
</style>
</head>
<body><main>
<header><div><div class="eyebrow">ROS BAG / OFFLINE DIAGNOSTICS</div><h1>离线诊断报告</h1><div class="path">{html.escape(bag['path'])}</div></div>
<div class="score">{summary['score']}<small>{html.escape(status_label)}</small></div></header>
<section class="grid">
<div class="metric"><span>ROS / 存储</span><strong>ROS {bag['ros_version']} · {html.escape(bag['storage'])}</strong></div>
<div class="metric"><span>录制时长</span><strong>{_number(bag.get('duration_sec'),2)} s</strong></div>
<div class="metric"><span>消息</span><strong>{_number(bag.get('message_count'),0)}</strong></div>
<div class="metric"><span>话题</span><strong>{_number(bag.get('topic_count'),0)}</strong></div>
<div class="metric"><span>严重 / 警告</span><strong>{summary['critical_count']} / {summary['warning_count']}</strong></div>
<div class="metric"><span>深度解析覆盖</span><strong>{_number(summary.get('deep_analysis_coverage_percent'),1)}%</strong></div>
</section>
<h2>优先处理的问题</h2><section class="issues">{issue_cards}</section>
<h2>话题时序与 QoS</h2><div class="table-wrap"><table><thead><tr><th>话题 / 类型</th><th>消息</th><th>平均 Hz</th><th>抖动 CV</th><th>最大间隙</th><th>估算缺失</th><th>QoS</th><th>状态</th></tr></thead><tbody>{topic_rows}</tbody></table></div>
<h2>TF 连通性与位姿变化</h2><div class="table-wrap"><table><thead><tr><th>变换</th><th>类别</th><th>样本</th><th>净位移</th><th>最大平移步长</th><th>最大旋转步长</th><th>最大间隙</th></tr></thead><tbody>{tf_rows}</tbody></table></div>
<h2>传感器与硬件模式</h2><div class="table-wrap"><table><thead><tr><th>话题</th><th>角色</th><th>解析样本</th><th>无效</th><th>空帧</th><th>最长重复</th><th>状态</th></tr></thead><tbody>{sensor_rows}</tbody></table></div>
<h2>建议执行顺序</h2><div class="note"><ol>{recommendations}</ol></div>
<h2>结论边界</h2><div class="note"><ul>{limitations}</ul></div>
<footer>生成时间：{html.escape(data['generated_at'])} · 读取器：{html.escape(bag['reader'])} · 报告格式 v{data['version']}</footer>
</main></body></html>"""


def _finite_json(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite_json(item) for item in value]
    return value


def _atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name = ""
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            temporary.write(content)
            temporary.flush()
            os.fsync(temporary.fileno())
            temporary_name = temporary.name
        os.replace(temporary_name, path)
    except OSError:
        if temporary_name:
            try:
                os.unlink(temporary_name)
            except OSError:
                pass
        raise


def _number(value: Any, precision: int) -> str:
    if value is None:
        return "—"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return html.escape(str(value))
    if not math.isfinite(number):
        return "—"
    if precision == 0:
        return f"{int(round(number)):,}"
    return f"{number:,.{precision}f}"


def _severity_label(value: str) -> str:
    return {"critical": "严重", "warning": "警告", "notice": "提示"}.get(value, value)


def _status_label(value: str) -> str:
    return {
        "normal": "正常",
        "healthy": "正常",
        "warning": "警告",
        "critical": "严重",
        "notice": "提示",
    }.get(value, value)
