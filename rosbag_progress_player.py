#!/usr/bin/env python3
"""Small ROS 2 bag player with a seekable timeline (ROS 2 Humble)."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

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
    def __init__(self, root: tk.Tk, initial_bag: str | None):
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
        log_dir = "/tmp/rosbag_progress_player_logs"
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
            "--start-paused", "--disable-keyboard-controls", "--loop",
        ]
        try:
            self.process = subprocess.Popen(
                command, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
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
            self.status_var.set(
                f"已暂停，可拖动进度条（{active}）" if ready and self.paused
                else (f"正在播放（{active}）" if ready else f"正在等待播放器服务（{active}）…"))

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
        self.current_ns = self.start_ns + offset_ns
        future = self.node.seek(self.current_ns)
        future.add_done_callback(lambda result: self.root.after(0, self.seek_done, result))
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
        self.current_ns = target
        self.node.seek(target)

    def change_rate(self, _event=None):
        if self.node is not None and self.node.ready():
            self.node.set_rate(float(self.rate_var.get()))

    def stop_player(self):
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
    args = parser.parse_args()
    root = tk.Tk()
    ProgressPlayer(root, args.bag)
    root.mainloop()


if __name__ == "__main__":
    main()
