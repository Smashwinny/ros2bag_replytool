#!/usr/bin/env python3
"""Floating sensor-trust and live ESKF computation inspector."""

import argparse
import json
import math
import shutil
import subprocess
import threading
import tkinter as tk
from tkinter import messagebox
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

COLORS = {"TRUSTED": "#27d94f", "REACQUIRING": "#ffd629", "DEGRADED": "#ff8c24",
          "SUSPECT": "#ff8c24", "SLIP_SUSPECTED": "#ff8c24", "FAULT": "#ff3434",
          "INVALID": "#ff3434", "CLOCK_FAULT": "#ff3434", "SLIP_CONFIRMED": "#ff3434"}


def format_bytes(size: int | None) -> str:
    if size is None or size < 0:
        return "--"
    value = float(size)
    for unit in ("B", "KiB", "MiB", "GiB", "TiB"):
        if value < 1024.0 or unit == "TiB":
            return f"{value:.1f} {unit}"
        value /= 1024.0
    return "--"


def directory_size(path: Path) -> int:
    total = 0
    for entry in path.rglob("*"):
        try:
            if entry.is_file() and not entry.is_symlink():
                total += entry.stat().st_size
        except OSError:
            continue
    return total


def clean_completed_runs(runs_dir: Path, active_log: Path) -> tuple[int, int]:
    """Remove direct run children except the active log's run directory."""
    root = runs_dir.resolve(strict=True)
    if not root.is_dir() or root == Path(root.anchor):
        raise ValueError("unsafe runs directory")
    active = active_log.resolve(strict=False)
    try:
        active.relative_to(root)
    except ValueError as error:
        raise ValueError("active log is outside runs directory") from error
    active_run = active.parent
    removed, failures = 0, 0
    for child in root.iterdir():
        if child.resolve(strict=False) == active_run or (child / ".active").exists():
            continue
        try:
            before = directory_size(child) if child.is_dir() else child.stat().st_size
            if child.is_dir() and not child.is_symlink():
                shutil.rmtree(child)
            else:
                child.unlink()
            removed += before
        except OSError:
            failures += 1
    return removed, failures


def _number(value: Any, digits: int = 3) -> str:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        return "--"
    return f"{value:.{digits}f}"


def _vector(value: Any, limit: int = 3, digits: int = 3) -> str:
    if not isinstance(value, list) or not value:
        return "--"
    return "[" + ", ".join(_number(item, digits) for item in value[:limit]) + "]"


def _shape_norm(value: Any) -> str:
    if isinstance(value, dict):
        rows, cols, data = value.get("rows"), value.get("cols"), value.get("data")
        shape = f"{rows}x{cols}"
    elif isinstance(value, list):
        data, shape = value, f"n={len(value)}"
    else:
        return "--"
    finite = [float(item) for item in data if isinstance(item, (int, float)) and math.isfinite(item)]
    norm = math.sqrt(sum(item * item for item in finite)) if finite else 0.0
    return f"{shape} |·|={norm:.3g}"


def _array_norm(value: Any) -> str:
    if not isinstance(value, list):
        return "--"
    finite = [float(item) for item in value if isinstance(item, (int, float)) and math.isfinite(item)]
    return f"n={len(value)} |·|={math.sqrt(sum(item * item for item in finite)):.3g}"


def process_summary(record: dict[str, Any]) -> tuple[str, str, str]:
    """Return title, compact state line and expanded computation text."""
    event, sequence = str(record.get("event", "waiting")), record.get("sequence", "--")
    title = f"#{sequence}  {event}  t={_number(record.get('filter_time_s'), 3)}"
    covariance = record.get("covariance_diag")
    p_sigma = "--"
    if isinstance(covariance, list) and len(covariance) >= 3:
        valid = [max(0.0, float(item)) for item in covariance[:3]
                 if isinstance(item, (int, float)) and math.isfinite(item)]
        if len(valid) == 3:
            p_sigma = _vector([math.sqrt(item) for item in valid])
    trace = record.get("computation_trace")
    nis = trace.get("nis") if isinstance(trace, dict) else None
    compact = (f"pXY {_vector(record.get('position_wb'), 2)}   "
               f"vXY {_vector(record.get('velocity_wb'), 2)}   σp {p_sigma}   "
               f"NIS {_number(nis)}   queue {record.get('pending_gps', '--')}/"
               f"{record.get('pending_odom', '--')}   OOSM "
               f"{record.get('oosm_replay_success', '--')}/"
               f"{record.get('oosm_replay_failure', '--')}")
    if not isinstance(trace, dict):
        return title, compact, "当前事件没有 computation_trace（等待下一次 predict/update）"
    prior, candidate = trace.get("prior_state_pvqwxyz_bg_ba"), trace.get("candidate_state_pvqwxyz_bg_ba")
    delta_state = "--"
    if isinstance(prior, list) and isinstance(candidate, list) and len(prior) == len(candidate):
        delta_state = _array_norm([b - a for a, b in zip(prior, candidate)
                                  if isinstance(a, (int, float)) and isinstance(b, (int, float))])
    fields = {name: _shape_norm(trace.get(name)) for name in
              ("F", "G", "Qc", "Phi", "Qd", "H", "R", "S", "K", "residual",
               "delta_x", "joseph_update", "reset")}
    detail = (f"stage {trace.get('stage', '--')}   dt {_number(trace.get('dt'), 6)}   "
              f"NIS {_number(nis)}   gain p/v {_number(trace.get('position_gain_scale'))}/"
              f"{_number(trace.get('velocity_gain_scale'))}\n"
              f"qWXYZ {_vector(record.get('orientation_wb_wxyz'), 4)}   "
              f"bg {_vector(record.get('gyro_bias_b'))}   ba {_vector(record.get('accel_bias_b'))}\n"
              f"F {fields['F']}   G {fields['G']}   Qc {fields['Qc']}   Φ {fields['Phi']}   Qd {fields['Qd']}\n"
              f"H {fields['H']}   R {fields['R']}   S {fields['S']}   K {fields['K']}\n"
              f"residual {fields['residual']}   δx {fields['delta_x']}   stateΔ {delta_state}\n"
              f"Joseph {fields['joseph_update']}   reset {fields['reset']}")
    return title, compact, detail


class JsonlFollower:
    """Incrementally consume complete JSONL records and survive truncate/replace."""
    def __init__(self, path: str) -> None:
        self.path, self.offset, self.identity, self.pending = Path(path), 0, None, ""

    def read_latest(self) -> dict[str, Any] | None:
        try:
            stat = self.path.stat()
        except OSError:
            return None
        identity = (stat.st_dev, stat.st_ino)
        if self.identity != identity or stat.st_size < self.offset:
            self.offset, self.pending = 0, ""
        self.identity = identity
        try:
            with self.path.open("r", encoding="utf-8") as stream:
                stream.seek(self.offset)
                chunk, self.offset = stream.read(), stream.tell()
        except OSError:
            return None
        lines, latest = (self.pending + chunk).splitlines(keepends=True), None
        self.pending = ""
        for line in lines:
            if not line.endswith(("\n", "\r")):
                self.pending = line
                continue
            try:
                item = json.loads(line)
                if isinstance(item, dict):
                    latest = item
            except json.JSONDecodeError:
                continue
        return latest


class CompactStatus(Node):
    def __init__(self, labels: dict[str, tk.Label]) -> None:
        super().__init__("eskf_compact_sensor_trust")
        self.labels = labels
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/eskf/sensor_trust", self.update, qos)

    def update(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        for sensor, label in self.labels.items():
            item = status.get(sensor, {})
            state, reason = str(item.get("state", "UNKNOWN")), str(item.get("reason", "no data"))
            reason = reason[:29] + ("…" if len(reason) > 30 else "")
            label.configure(text=f"{sensor.upper()}  {state}\n{reason}",
                            fg=COLORS.get(state, "#bfc5cc"))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--title", default="ESKF Live Inspector")
    parser.add_argument("--geometry", default="720x118+0+0")
    parser.add_argument("--process-log", default="")
    parser.add_argument("--runs-dir", default="")
    args = parser.parse_args()
    root = tk.Tk(className="EskfCompactStatus")
    root.title(args.title); root.geometry(args.geometry); root.configure(bg="#202124")
    root.attributes("-topmost", True)
    labels: dict[str, tk.Label] = {}
    for column, sensor in enumerate(("imu", "gnss", "odom")):
        label = tk.Label(root, text=f"{sensor.upper()}  UNKNOWN\nno data", bg="#202124",
                         fg="#bfc5cc", font=("Sans", 9, "bold"), justify="left", anchor="w", padx=8)
        label.grid(row=0, column=column, sticky="nsew")
        root.grid_columnconfigure(column, weight=1, uniform="sensor"); labels[sensor] = label
    header = tk.Label(root, text="等待 ESKF 过程日志…", bg="#151617", fg="#77bdfb",
                      font=("Monospace", 9, "bold"), anchor="w", padx=8)
    header.grid(row=1, column=0, columnspan=3, sticky="ew")
    compact = tk.Label(root, text="", bg="#151617", fg="#e8eaed", font=("Monospace", 8), anchor="w", padx=8)
    compact.grid(row=2, column=0, columnspan=3, sticky="ew")
    detail = tk.Label(root, text="", bg="#101112", fg="#cbd5df", font=("Monospace", 8),
                      justify="left", anchor="nw", padx=8, pady=4)
    expanded = False

    def toggle() -> None:
        nonlocal expanded
        expanded = not expanded
        if expanded:
            detail.grid(row=3, column=0, columnspan=3, sticky="nsew")
            root.geometry(f"{root.winfo_width()}x245+{root.winfo_x()}+{root.winfo_y()}")
        else:
            detail.grid_remove()
            root.geometry(f"{root.winfo_width()}x118+{root.winfo_x()}+{root.winfo_y()}")
        toggle_button.configure(text="▴" if expanded else "▾")

    toggle_button = tk.Button(root, text="▾", command=toggle, bg="#303134", fg="#e8eaed",
                              relief="flat", font=("Sans", 8), padx=3, pady=0)
    toggle_button.place(relx=1.0, x=-4, y=47, anchor="ne")
    storage = tk.Label(root, text="日志 当前 -- / 全部 --", bg="#202124", fg="#9aa0a6",
                       font=("Sans", 8), anchor="w", padx=8)
    storage.place(x=0, rely=1.0, y=-2, anchor="sw")
    root.grid_rowconfigure(0, weight=1)
    follower = JsonlFollower(args.process_log) if args.process_log else None
    process_log = Path(args.process_log) if args.process_log else None
    runs_dir = Path(args.runs_dir) if args.runs_dir else None
    storage_state = {"total": None, "scanning": False, "message": ""}

    def open_logs() -> None:
        if runs_dir is None:
            return
        try:
            subprocess.Popen(["xdg-open", str(runs_dir)], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, start_new_session=True)
        except OSError as error:
            messagebox.showerror("打开日志目录失败", str(error), parent=root)

    def finish_cleanup(removed: int, failures: int, error: str = "") -> None:
        clean_button.configure(state="normal")
        storage_state["scanning"] = False
        storage_state["total"] = None
        if error:
            messagebox.showerror("清理日志失败", error, parent=root)
        else:
            messagebox.showinfo("清理完成",
                                f"已永久删除 {format_bytes(removed)} 历史日志；失败 {failures} 项。\n"
                                "当前运行目录已保留。", parent=root)

    def cleanup_worker() -> None:
        try:
            removed, failures = clean_completed_runs(runs_dir, process_log)
            root.after(0, finish_cleanup, removed, failures, "")
        except (OSError, ValueError) as error:
            root.after(0, finish_cleanup, 0, 0, str(error))

    def confirm_cleanup() -> None:
        if runs_dir is None or process_log is None:
            return
        if not messagebox.askyesno("永久清理历史日志",
                                   "将永久删除 runs/ 下除当前运行目录外的全部历史日志，无法恢复。\n\n继续吗？",
                                   icon="warning", parent=root):
            return
        clean_button.configure(state="disabled")
        threading.Thread(target=cleanup_worker, daemon=True).start()

    open_button = tk.Button(root, text="📂 日志", command=open_logs, bg="#303134", fg="#e8eaed",
                            relief="flat", font=("Sans", 8), padx=5, pady=0)
    open_button.place(relx=1.0, x=-70, rely=1.0, y=-2, anchor="se")
    clean_button = tk.Button(root, text="清理", command=confirm_cleanup, bg="#5f2727", fg="#ffffff",
                             relief="flat", font=("Sans", 8), padx=5, pady=0)
    clean_button.place(relx=1.0, x=-4, rely=1.0, y=-2, anchor="se")
    if runs_dir is None or process_log is None:
        open_button.configure(state="disabled"); clean_button.configure(state="disabled")
    rclpy.init(); node = CompactStatus(labels)

    def scan_storage() -> None:
        try:
            total = directory_size(runs_dir) if runs_dir and runs_dir.is_dir() else None
        except OSError:
            total = None
        storage_state["total"] = total
        storage_state["scanning"] = False

    def refresh_storage() -> None:
        current = None
        if process_log is not None:
            try:
                current = process_log.stat().st_size
            except OSError:
                pass
        storage.configure(text=f"日志 当前 {format_bytes(current)} / 全部 {format_bytes(storage_state['total'])}")
        if runs_dir is not None and not storage_state["scanning"]:
            storage_state["scanning"] = True
            threading.Thread(target=scan_storage, daemon=True).start()
        root.after(2000, refresh_storage)

    def poll() -> None:
        if not rclpy.ok(): root.destroy(); return
        rclpy.spin_once(node, timeout_sec=0.0)
        record = follower.read_latest() if follower else None
        if record is not None:
            title, short, long = process_summary(record)
            header.configure(text=title, fg="#ff6b6b" if record.get("faulted") else "#77bdfb")
            compact.configure(text=short); detail.configure(text=long)
        root.after(40, poll)

    def close() -> None:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close); root.after(0, poll); root.after(0, refresh_storage)
    try: root.mainloop()
    finally:
        if rclpy.ok(): node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()

