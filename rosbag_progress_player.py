#!/usr/bin/env python3
"""Small ROS 2 bag player with a seekable timeline (ROS 2 Humble)."""

from __future__ import annotations

import argparse
from decimal import Decimal, InvalidOperation
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk

import yaml

import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from rosbag2_interfaces.srv import Pause, Resume, Seek, SetRate


PLAYER_NODE = "/rosbag2_player"


def format_time(seconds: float) -> str:
    seconds = max(0, int(seconds))
    hours, remainder = divmod(seconds, 3600)
    minutes, secs = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


def parse_time_offset_ns(value: str) -> int:
    """Parse seconds, MM:SS(.sss), or HH:MM:SS(.sss) into nanoseconds."""
    text = value.strip()
    if not text:
        raise ValueError("时间不能为空")
    parts = text.split(":")
    if len(parts) not in (1, 2, 3):
        raise ValueError("格式应为秒、MM:SS 或 HH:MM:SS.mmm")
    try:
        if len(parts) == 1:
            total = Decimal(parts[0])
        else:
            if not all(part.isdigit() for part in parts[:-1]):
                raise ValueError
            seconds = Decimal(parts[-1])
            if seconds < 0 or seconds >= 60:
                raise ValueError
            if len(parts) == 2:
                total = Decimal(int(parts[0]) * 60) + seconds
            else:
                minutes = int(parts[1])
                if minutes >= 60:
                    raise ValueError
                total = Decimal(int(parts[0]) * 3600 + minutes * 60) + seconds
    except (InvalidOperation, ValueError):
        raise ValueError("格式应为秒、MM:SS 或 HH:MM:SS.mmm") from None
    if not total.is_finite() or total < 0:
        raise ValueError("时间必须是非负有限值")
    return int(total * Decimal(1_000_000_000))


def read_metadata(path: Path) -> tuple[Path, int, int]:
    bag_dir = path.parent if path.name == "metadata.yaml" else path
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        raise ValueError(f"没有找到 {metadata_path}")
    with metadata_path.open("r", encoding="utf-8") as stream:
        root = yaml.safe_load(stream)
    info = root["rosbag2_bagfile_information"]
    start_ns = int(info["starting_time"]["nanoseconds_since_epoch"])
    duration_ns = int(info["duration"]["nanoseconds"])
    if duration_ns <= 0:
        raise ValueError("bag 时长无效")
    return bag_dir.resolve(), start_ns, duration_ns


def read_profile(path: Path | None) -> dict:
    if path is None:
        return {}
    with path.open("r", encoding="utf-8") as stream:
        profile = yaml.safe_load(stream) or {}
    if not isinstance(profile, dict):
        raise ValueError("profile 顶层必须是 YAML mapping")
    command = profile.get("managed_command")
    if command is not None and (not isinstance(command, list) or not command or
                                not all(isinstance(item, str) and item for item in command)):
        raise ValueError("managed_command 必须是非空字符串数组（不通过 shell 执行）")
    rate = float(profile.get("rebuild_rate", 10.0))
    if not 1.0 <= rate <= 100.0:
        raise ValueError("rebuild_rate 必须在 1～100 之间")
    profile["rebuild_rate"] = rate
    profile["startup_wait_s"] = max(0.0, float(profile.get("startup_wait_s", 2.0)))
    extra_env = profile.get("env", {})
    if not isinstance(extra_env, dict) or not all(
            isinstance(key, str) and isinstance(value, (str, int, float, bool))
            for key, value in extra_env.items()):
        raise ValueError("env 必须是标量键值 mapping")
    profile["env"] = {key: str(value) for key, value in extra_env.items()}
    return profile


class PlayerControl(Node):
    def __init__(self, on_clock, context: Context):
        super().__init__("rosbag_progress_control", context=context)
        # rosbag2 Player publishes /clock as BEST_EFFORT on Humble.  The
        # default integer-depth subscription is RELIABLE and therefore cannot
        # connect to it, which leaves the progress bar frozen.
        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Clock, "/clock", on_clock, clock_qos)
        self.pause_client = self.create_client(Pause, f"{PLAYER_NODE}/pause")
        self.resume_client = self.create_client(Resume, f"{PLAYER_NODE}/resume")
        self.seek_client = self.create_client(Seek, f"{PLAYER_NODE}/seek")
        self.rate_client = self.create_client(SetRate, f"{PLAYER_NODE}/set_rate")

    def ready(self) -> bool:
        return self.seek_client.service_is_ready()

    def pause(self):
        return self.pause_client.call_async(Pause.Request())

    def resume(self):
        return self.resume_client.call_async(Resume.Request())

    def seek(self, absolute_ns: int):
        request = Seek.Request()
        request.time.sec = absolute_ns // 1_000_000_000
        request.time.nanosec = absolute_ns % 1_000_000_000
        return self.seek_client.call_async(request)

    def set_rate(self, rate: float):
        request = SetRate.Request()
        request.rate = rate
        return self.rate_client.call_async(request)


class ProgressPlayer:
    def __init__(self, root: tk.Tk, initial_bag: str | None, profile_path: str | None):
        self.root = root
        self.root.title("ROS 2 Bag 进度播放器")
        self.root.geometry("820x310")
        self.root.minsize(680, 290)

        self.bag_dir: Path | None = None
        self.start_ns = 0
        self.duration_ns = 1
        self.current_ns = 0
        self.dragging = False
        self.paused = True
        self.process: subprocess.Popen | None = None
        self.managed_process: subprocess.Popen | None = None
        self.profile = read_profile(Path(profile_path).resolve() if profile_path else None)
        self.rebuild_target_ns: int | None = None
        self.rebuild_started_wall = 0.0
        self.context: Context | None = None
        self.executor: SingleThreadedExecutor | None = None
        self.node: PlayerControl | None = None
        self.spin_thread: threading.Thread | None = None
        self.ros_stop_event: threading.Event | None = None
        self.active_domain_id = 0
        self.active_localhost = True

        self.path_var = tk.StringVar(value="尚未选择 bag")
        self.status_var = tk.StringVar(value="请选择含 metadata.yaml 的 bag 目录")
        self.time_var = tk.StringVar(value="00:00:00 / 00:00:00")
        self.rate_var = tk.StringVar(value="1.0")
        self.slider_var = tk.DoubleVar(value=0)
        self.domain_var = tk.StringVar(value=os.environ.get("ROS_DOMAIN_ID", "0"))
        self.localhost_var = tk.BooleanVar(value=os.environ.get("ROS_LOCALHOST_ONLY", "1") != "0")
        self.build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(100, self.refresh_ui)
        if initial_bag:
            self.load_bag(Path(initial_bag))

    def build_ui(self):
        frame = ttk.Frame(self.root, padding=16)
        frame.pack(fill="both", expand=True)

        top = ttk.Frame(frame)
        top.pack(fill="x")
        ttk.Button(top, text="选择 Bag", command=self.choose_bag).pack(side="left")
        ttk.Label(top, textvariable=self.path_var).pack(side="left", padx=12, fill="x", expand=True)

        network = ttk.LabelFrame(frame, text="DDS 网络配置", padding=(10, 6))
        network.pack(fill="x", pady=(12, 0))
        ttk.Label(network, text="ROS_DOMAIN_ID").pack(side="left")
        ttk.Spinbox(network, from_=0, to=232, textvariable=self.domain_var, width=6).pack(side="left", padx=(6, 18))
        ttk.Checkbutton(
            network, text="仅本机通信（ROS_LOCALHOST_ONLY=1）",
            variable=self.localhost_var).pack(side="left")
        ttk.Button(network, text="应用并重启播放", command=self.apply_network).pack(side="right")

        self.scale = ttk.Scale(frame, from_=0, to=1000, variable=self.slider_var)
        self.scale.pack(fill="x", pady=(28, 5))
        self.scale.bind("<ButtonPress-1>", self.begin_drag)
        self.scale.bind("<ButtonRelease-1>", self.end_drag)
        ttk.Label(frame, textvariable=self.time_var, anchor="center").pack(fill="x")

        controls = ttk.Frame(frame)
        controls.pack(pady=18)
        ttk.Button(controls, text="⏮  -10 秒", command=lambda: self.jump(-10)).pack(side="left", padx=4)
        self.play_button = ttk.Button(controls, text="▶ 播放", command=self.toggle_play, state="disabled")
        self.play_button.pack(side="left", padx=4)
        ttk.Button(controls, text="+10 秒  ⏭", command=lambda: self.jump(10)).pack(side="left", padx=4)
        self.jump_time_icon = tk.PhotoImage(width=16, height=16)
        icon_color = "#326da8"
        for x, y in (
                (6, 1), (7, 1), (8, 1), (9, 1),
                (3, 3), (4, 2), (11, 2), (12, 3),
                (2, 4), (13, 4), (1, 6), (14, 6),
                (1, 7), (14, 7), (1, 8), (14, 8),
                (2, 11), (13, 11), (3, 12), (12, 12),
                (4, 13), (11, 13), (6, 14), (7, 14), (8, 14), (9, 14),
                (7, 5), (7, 6), (7, 7), (8, 8), (9, 9), (10, 10)):
            self.jump_time_icon.put(icon_color, (x, y))
        ttk.Button(
            controls, text="指定时刻", image=self.jump_time_icon,
            compound="left", command=self.jump_to_time).pack(side="left", padx=4)
        ttk.Label(controls, text="  倍速").pack(side="left")
        rate_box = ttk.Combobox(
            controls, textvariable=self.rate_var, values=("0.25", "0.5", "1.0", "2.0", "5.0", "10.0"),
            width=6, state="readonly")
        rate_box.pack(side="left", padx=4)
        rate_box.bind("<<ComboboxSelected>>", self.change_rate)

        ttk.Label(frame, textvariable=self.status_var, anchor="center").pack(fill="x")

    def choose_bag(self):
        selected = filedialog.askdirectory(title="选择 rosbag2 目录")
        if selected:
            self.load_bag(Path(selected))

    def load_bag(self, path: Path):
        try:
            bag_dir, start_ns, duration_ns = read_metadata(path)
            domain_id = int(self.domain_var.get())
            if not 0 <= domain_id <= 232:
                raise ValueError("ROS_DOMAIN_ID 必须在 0～232 之间")
        except Exception as exc:
            messagebox.showerror("无法打开 Bag", str(exc))
            return
        self.stop_player()
        self.stop_ros()
        self.bag_dir, self.start_ns, self.duration_ns = bag_dir, start_ns, duration_ns
        self.current_ns = start_ns
        self.paused = True
        self.path_var.set(str(bag_dir))
        self.status_var.set("正在启动播放器…")
        log_dir = str(Path(__file__).resolve().parent / "tmp" / "rosbag_progress_player_logs")
        os.makedirs(log_dir, exist_ok=True)
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = log_dir
        env["ROS_DOMAIN_ID"] = str(domain_id)
        env["ROS_LOCALHOST_ONLY"] = "1" if self.localhost_var.get() else "0"
        self.active_domain_id = domain_id
        self.active_localhost = self.localhost_var.get()
        # ROS_LOCALHOST_ONLY is consumed by the RMW implementation when this
        # process creates its participant; the child receives the same value.
        os.environ["ROS_LOCALHOST_ONLY"] = env["ROS_LOCALHOST_ONLY"]
        try:
            self.start_ros(domain_id)
        except Exception as exc:
            self.stop_ros()
            messagebox.showerror("DDS 初始化失败", str(exc))
            return
        command = [
            "ros2", "bag", "play", str(bag_dir), "--clock", "30",
            "--start-paused", "--disable-keyboard-controls",
        ]
        try:
            self.process = subprocess.Popen(
                command, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
            self.start_managed_process(env, log_dir)
        except OSError as exc:
            self.process = None
            self.stop_ros()
            messagebox.showerror("启动失败", str(exc))

    def apply_network(self):
        if self.bag_dir:
            self.load_bag(self.bag_dir)
        else:
            self.status_var.set("网络配置已保存，选择 Bag 后生效")

    def start_ros(self, domain_id: int):
        self.context = Context()
        rclpy.init(args=None, context=self.context, domain_id=domain_id)
        self.node = PlayerControl(self.on_clock, self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.ros_stop_event = threading.Event()
        self.spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.spin_thread.start()

    def spin_ros(self):
        assert self.context is not None and self.executor is not None and self.ros_stop_event is not None
        while not self.ros_stop_event.is_set() and rclpy.ok(context=self.context):
            self.executor.spin_once(timeout_sec=0.1)

    def on_clock(self, message: Clock):
        self.current_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec

    def refresh_ui(self):
        if self.process and self.process.poll() is not None:
            self.process = None
            self.paused = True
            self.status_var.set("播放已结束；重新选择 Bag 可再次播放")
            self.play_button.configure(state="disabled", text="▶ 播放")
        elif self.process:
            ready = self.node is not None and self.node.ready()
            self.play_button.configure(state="normal" if ready else "disabled")
            mode = "本机" if self.active_localhost else "网络"
            active = f"Domain {self.active_domain_id} · {mode}"
            if self.rebuild_target_ns is not None:
                target_s = (self.rebuild_target_ns - self.start_ns) / 1e9
                self.status_var.set(f"正在从起点重建状态 → {format_time(target_s)}（{active}）")
            else:
                self.status_var.set(
                    f"已暂停，可拖动进度条（{active}）" if ready and self.paused
                    else (f"正在播放（{active}）" if ready else f"正在等待播放器服务（{active}）…"))

        if self.rebuild_target_ns is not None and self.current_ns >= self.rebuild_target_ns:
            self.finish_rebuild()

        if self.bag_dir:
            offset_ns = min(max(self.current_ns - self.start_ns, 0), self.duration_ns)
            if not self.dragging:
                self.slider_var.set(offset_ns * 1000.0 / self.duration_ns)
            shown_ns = int(self.slider_var.get() * self.duration_ns / 1000) if self.dragging else offset_ns
            self.time_var.set(f"{format_time(shown_ns / 1e9)} / {format_time(self.duration_ns / 1e9)}")
        self.root.after(100, self.refresh_ui)

    def begin_drag(self, _event):
        self.dragging = True

    def end_drag(self, _event):
        if not self.bag_dir or self.node is None or not self.node.ready():
            self.dragging = False
            return
        offset_ns = int(self.slider_var.get() * self.duration_ns / 1000)
        self.rebuild_to(self.start_ns + offset_ns)
        self.dragging = False

    def seek_done(self, future):
        try:
            if not future.result().success:
                self.status_var.set("跳转失败：目标时间不在 bag 范围内")
        except Exception as exc:
            self.status_var.set(f"跳转失败：{exc}")

    def toggle_play(self):
        if self.node is None or not self.node.ready():
            return
        if self.paused:
            self.node.resume()
            self.paused = False
            self.play_button.configure(text="⏸ 暂停")
        else:
            self.node.pause()
            self.paused = True
            self.play_button.configure(text="▶ 播放")

    def jump(self, seconds: float):
        if not self.bag_dir or self.node is None or not self.node.ready():
            return
        target = min(max(self.current_ns + int(seconds * 1e9), self.start_ns), self.start_ns + self.duration_ns - 1)
        self.rebuild_to(target)

    def jump_to_time(self):
        if not self.bag_dir or self.node is None or not self.node.ready():
            messagebox.showinfo("指定时刻", "播放器尚未准备完成", parent=self.root)
            return
        current_offset_s = max(0.0, (self.current_ns - self.start_ns) / 1e9)
        value = simpledialog.askstring(
            "跳转到指定时刻",
            "输入相对 bag 起点的时间：\n秒、MM:SS 或 HH:MM:SS.mmm",
            initialvalue=f"{current_offset_s:.3f}",
            parent=self.root,
        )
        if value is None:
            return
        try:
            offset_ns = parse_time_offset_ns(value)
            if offset_ns >= self.duration_ns:
                raise ValueError(
                    f"目标超出 bag 时长 {format_time(self.duration_ns / 1e9)}")
        except ValueError as exc:
            messagebox.showerror("时间无效", str(exc), parent=self.root)
            return
        self.rebuild_to(self.start_ns + offset_ns)

    def rebuild_to(self, target_ns: int):
        """Restart managed state and replay every recorded input up to target."""
        if self.node is None or not self.node.ready() or self.rebuild_target_ns is not None:
            return
        if not self.profile.get("managed_command"):
            self.current_ns = target_ns
            future = self.node.seek(target_ns)
            future.add_done_callback(lambda result: self.root.after(0, self.seek_done, result))
            return
        self.node.pause()
        self.paused = True
        self.stop_managed_process()
        self.current_ns = self.start_ns
        future = self.node.seek(self.start_ns)
        self.rebuild_target_ns = target_ns
        future.add_done_callback(lambda result: self.root.after(0, self.begin_rebuild, result))

    def begin_rebuild(self, future):
        try:
            if not future.result().success:
                raise RuntimeError("无法跳回 bag 起点")
            env = os.environ.copy()
            env["ROS_DOMAIN_ID"] = str(self.active_domain_id)
            env["ROS_LOCALHOST_ONLY"] = "1" if self.active_localhost else "0"
            log_dir = str(Path(__file__).resolve().parent / "tmp" / "rosbag_progress_player_logs")
            self.start_managed_process(env, log_dir)
            self.node.set_rate(self.profile["rebuild_rate"])
            self.rebuild_started_wall = time.monotonic()
            self.root.after(int(self.profile["startup_wait_s"] * 1000), self.resume_rebuild)
        except Exception as exc:
            self.rebuild_target_ns = None
            self.status_var.set(f"状态重建失败：{exc}")

    def resume_rebuild(self):
        if self.rebuild_target_ns is None or self.node is None:
            return
        if self.rebuild_target_ns <= self.start_ns:
            self.finish_rebuild()
            return
        self.node.resume()
        self.paused = False

    def finish_rebuild(self):
        if self.rebuild_target_ns is None or self.node is None:
            return
        target = self.rebuild_target_ns
        actual = self.current_ns
        self.rebuild_target_ns = None
        self.node.pause()
        self.node.set_rate(float(self.rate_var.get()))
        self.paused = True
        self.play_button.configure(text="▶ 播放")
        overshoot_ms = max(0.0, (actual - target) / 1e6)
        self.status_var.set(
            f"状态已从起点重建并暂停；目标后越界 {overshoot_ms:.1f} ms（未回拨状态）")

    def start_managed_process(self, env: dict, log_dir: str):
        command = self.profile.get("managed_command")
        if not command or self.managed_process is not None:
            return
        cwd = self.profile.get("cwd")
        env = {**env, **self.profile.get("env", {})}
        log_path = Path(log_dir) / "managed_stack.log"
        log_stream = log_path.open("ab")
        try:
            self.managed_process = subprocess.Popen(
                command, cwd=cwd, env=env, stdout=log_stream, stderr=subprocess.STDOUT,
                start_new_session=True)
        finally:
            log_stream.close()

    def stop_managed_process(self):
        process = self.managed_process
        if process and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2)
        self.managed_process = None

    def change_rate(self, _event=None):
        if self.node is not None and self.node.ready():
            self.node.set_rate(float(self.rate_var.get()))

    def stop_player(self):
        self.rebuild_target_ns = None
        self.stop_managed_process()
        if self.process and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
                self.process.wait(timeout=3)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if self.process.poll() is None:
                    os.killpg(self.process.pid, signal.SIGTERM)
        self.process = None

    def stop_ros(self):
        if self.ros_stop_event is not None:
            self.ros_stop_event.set()
        if self.spin_thread is not None:
            self.spin_thread.join(timeout=1)
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=1)
        if self.node is not None:
            self.node.destroy_node()
        if self.context is not None and rclpy.ok(context=self.context):
            self.context.shutdown()
        self.executor = None
        self.node = None
        self.context = None
        self.spin_thread = None
        self.ros_stop_event = None

    def close(self):
        self.stop_player()
        self.stop_ros()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="ROS 2 bag player with a seekable progress bar")
    parser.add_argument("bag", nargs="?", help="bag directory or metadata.yaml")
    parser.add_argument("--profile", help="YAML profile containing a managed stateful command")
    args = parser.parse_args()
    root = tk.Tk()
    ProgressPlayer(root, args.bag, args.profile)
    root.mainloop()


if __name__ == "__main__":
    main()
