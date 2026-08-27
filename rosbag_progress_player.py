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
import json

from progress_player.replay_history import ReplayEpochClock
from progress_player.determinism import (
    FULL_CHECKPOINT_FIELD_COUNT, compare_three, first_differences,
    snapshot_hashes)
from progress_player.persistent_replay_cache import (
    cache_fingerprint, load_valid_manifest, remove_cache_directories,
    stale_build_cache_directories)

import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64, String
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
    profile["checkpoint_restore"] = bool(profile.get("checkpoint_restore", False))
    profile["ordinal_reader"] = bool(profile.get("ordinal_reader", False))
    if profile["ordinal_reader"] and not profile["checkpoint_restore"]:
        raise ValueError("ordinal_reader 必须与 checkpoint_restore 一起启用")
    bag_play_args = profile.get("bag_play_args", [])
    if not isinstance(bag_play_args, list) or not all(
            isinstance(value, str) for value in bag_play_args):
        raise ValueError("profile bag_play_args 必须是字符串列表")
    profile["bag_play_args"] = bag_play_args
    bag_target_map = profile.get("bag_target_map", {})
    if not isinstance(bag_target_map, dict) or not all(
            isinstance(key, str) and isinstance(value, str)
            for key, value in bag_target_map.items()):
        raise ValueError("profile bag_target_map 必须是 bag 路径到 YAML 路径的字符串映射")
    profile["bag_target_map"] = {
        str(Path(key).resolve()): str(Path(value).resolve())
        for key, value in bag_target_map.items()
    }
    extra_env = profile.get("env", {})
    if not isinstance(extra_env, dict) or not all(
            isinstance(key, str) and isinstance(value, (str, int, float, bool))
            for key, value in extra_env.items()):
        raise ValueError("env 必须是标量键值 mapping")
    profile["env"] = {key: str(value) for key, value in extra_env.items()}
    return profile


def resolve_target_yaml(profile: dict, bag_dir: Path) -> str | None:
    target_map = profile.get("bag_target_map", {})
    if not target_map:
        return None
    target_yaml = target_map.get(str(bag_dir.resolve()))
    if target_yaml is None:
        raise ValueError(
            f"当前 profile 没有登记该 bag 对应的 target YAML：{bag_dir.resolve()}")
    if not Path(target_yaml).is_file():
        raise ValueError(f"对应 target YAML 不存在：{target_yaml}")
    return target_yaml


class PlayerControl(Node):
    def __init__(self, on_clock, on_restore_result, on_ordinal_seek_result,
                 on_target_ordinal_result, on_reader_status,
                 on_snapshot_result, on_seal_result, context: Context):
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
        self.create_subscription(
            String, "/eskf/replay_restore_result", on_restore_result,
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            String, "/rosbag_progress/ordinal_seek_result",
            on_ordinal_seek_result, 10)
        self.create_subscription(String, "/rosbag_progress/target_ordinal_result",
                                 on_target_ordinal_result, 10)
        self.create_subscription(String, "/rosbag_progress/ordinal_reader_status",
                                 on_reader_status, 10)
        result_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/eskf/replay_snapshot_result",
                                 on_snapshot_result, result_qos)
        self.create_subscription(String, "/eskf/replay_seal_result",
                                 on_seal_result, result_qos)
        self.restore_publisher = self.create_publisher(
            Float64, "/eskf/replay_restore_request", 1)
        self.visual_seek_publisher = self.create_publisher(
            Float64, "/rosbag_progress/visual_seek", 10)
        self.replay_epoch_publisher = self.create_publisher(
            String, "/rosbag_progress/replay_epoch", 10)
        self.replay_seal_publisher = self.create_publisher(
            String, "/eskf/replay_seal_request", 1)
        self.ordinal_seek_publisher = self.create_publisher(
            String, "/rosbag_progress/ordinal_seek_request", 10)
        self.target_ordinal_publisher = self.create_publisher(
            String, "/rosbag_progress/target_ordinal_request", 10)
        self.ordinal_restore_publisher = self.create_publisher(
            String, "/eskf/replay_restore_ordinal_request", 10)
        self.snapshot_publisher = self.create_publisher(
            String, "/eskf/replay_snapshot_request", 10)
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

    def restore_checkpoint(self, stamp_s: float):
        message = Float64()
        message.data = stamp_s
        self.restore_publisher.publish(message)

    def announce_visual_seek(self, stamp_s: float, epoch: int,
                             first_pass_complete: bool):
        message = Float64()
        message.data = stamp_s
        self.visual_seek_publisher.publish(message)
        epoch_message = String()
        epoch_message.data = json.dumps({
            "schema": "rosbag_replay_epoch/v1",
            "epoch": epoch,
            "target_ns": int(round(stamp_s * 1e9)),
            "first_pass_complete": first_pass_complete,
        }, separators=(",", ":"), sort_keys=True)
        self.replay_epoch_publisher.publish(epoch_message)

    def seal_first_pass(self, final_ns: int):
        message = String()
        message.data = json.dumps({
            "schema": "eskf_replay_seal/v1",
            "final_ns": final_ns,
        }, separators=(",", ":"), sort_keys=True)
        self.replay_seal_publisher.publish(message)

    def seek_ordinal(self, epoch: int, bag_ordinal: int):
        message = String()
        message.data = json.dumps({
            "schema": "rosbag_ordinal_seek/v1",
            "epoch": epoch,
            "bag_ordinal": bag_ordinal,
        }, separators=(",", ":"), sort_keys=True)
        self.ordinal_seek_publisher.publish(message)

    def request_target_ordinal(self, epoch: int, target_ns: int):
        self._publish_json(self.target_ordinal_publisher, {
            "schema": "rosbag_target_ordinal/v1", "epoch": epoch,
            "target_ns": target_ns})

    def restore_ordinal(self, epoch: int, target_ordinal: int):
        self._publish_json(self.ordinal_restore_publisher, {
            "schema": "eskf_replay_restore_ordinal/v1", "epoch": epoch,
            "target_ordinal": target_ordinal})

    def request_snapshot(self, epoch: int, target_ordinal: int):
        self._publish_json(self.snapshot_publisher, {
            "schema": "eskf_replay_snapshot_request/v1", "epoch": epoch,
            "target_ordinal": target_ordinal})

    @staticmethod
    def _publish_json(publisher, payload):
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        publisher.publish(message)


class ProgressPlayer:
    def __init__(self, root: tk.Tk, initial_bag: str | None, profile_path: str | None,
                 title: str | None = None, geometry: str | None = None):
        self.root = root
        self.root.title(title or "ROS 2 Bag 进度播放器")
        self.root.geometry(geometry or "820x310")
        self.root.minsize(480 if geometry else 680, 290)

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
        self.checkpoint_restore_pending_ns: int | None = None
        self.context: Context | None = None
        self.executor: SingleThreadedExecutor | None = None
        self.node: PlayerControl | None = None
        self.spin_thread: threading.Thread | None = None
        self.ros_stop_event: threading.Event | None = None
        self.active_domain_id = 0
        self.active_localhost = True
        self.epoch_clock = ReplayEpochClock()
        self.first_pass_complete = False
        self.first_pass_sealed = False
        self.determinism_active = False
        self.determinism_target_ns: int | None = None
        self.determinism_target_ordinal: int | None = None
        self.determinism_runs: list[dict] = []
        self.determinism_snapshots: list[dict] = []
        self.determinism_retry_count = 0
        self.sealed_checkpoint_count: int | None = None
        self.sealed_log_size: int | None = None
        self.cache_manifest: dict | None = None
        self.cache_fingerprint: str | None = None
        self.cache_builder: subprocess.Popen | None = None

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
            target_yaml = resolve_target_yaml(self.profile, bag_dir)
        except Exception as exc:
            messagebox.showerror("无法打开 Bag", str(exc))
            return
        self.stop_player()
        self.stop_ros()
        self.bag_dir, self.start_ns, self.duration_ns = bag_dir, start_ns, duration_ns
        self.current_ns = start_ns
        self.first_pass_complete = False
        self.first_pass_sealed = False
        self.determinism_active = False
        self.determinism_runs = []
        self.determinism_snapshots = []
        self.determinism_retry_count = 0
        self.sealed_checkpoint_count = None
        self.sealed_log_size = None
        self.cache_manifest = None
        self.cache_fingerprint = None
        build_state = (Path(__file__).resolve().parent / "progress_player" /
                       "build_state" / "eskf_compare.json")
        if target_yaml is not None and build_state.is_file():
            try:
                fingerprint, identity = cache_fingerprint(
                    bag_dir, Path(target_yaml), build_state)
                self.cache_fingerprint = fingerprint
                cache_root = (Path(__file__).resolve().parent /
                              "progress_player" / "cache")
                self.cache_manifest = load_valid_manifest(cache_root, fingerprint)
                stale_caches = stale_build_cache_directories(
                    cache_root, bag_dir, Path(target_yaml),
                    identity["build_state"])
                if self.cache_manifest is None and stale_caches:
                    should_rebuild = messagebox.askyesno(
                        "编译产物已变化",
                        f"检测到当前编译产物与该数据包的旧缓存不一致。\n\n"
                        f"旧缓存数量：{len(stale_caches)}\n"
                        "旧缓存不能代表当前代码结果，需要清理缓存并重新完整播放一次。\n"
                        "这里只删除缓存索引，不删除首次过程日志。\n\n"
                        "是否立即清理旧缓存并开始重新运行？",
                        parent=self.root)
                    if not should_rebuild:
                        self.path_var.set(str(bag_dir))
                        self.status_var.set(
                            "已保留旧缓存；未启动。清理后需重新完整播放一次")
                        return
                    try:
                        remove_cache_directories(cache_root, stale_caches)
                    except (OSError, ValueError) as exc:
                        messagebox.showerror(
                            "旧缓存清理失败",
                            f"未启动回放，请先处理缓存目录：\n{exc}",
                            parent=self.root)
                        self.path_var.set(str(bag_dir))
                        self.status_var.set("旧缓存清理失败；未启动")
                        return
            except (OSError, ValueError, KeyError, json.JSONDecodeError):
                self.cache_manifest = None
        if self.cache_manifest is not None:
            self.first_pass_complete = True
            self.first_pass_sealed = True
        self.paused = True
        self.path_var.set(str(bag_dir))
        self.status_var.set("正在启动播放器…")
        log_dir = str(Path(__file__).resolve().parent / "tmp" / "rosbag_progress_player_logs")
        os.makedirs(log_dir, exist_ok=True)
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = log_dir
        env["ROS_DOMAIN_ID"] = str(domain_id)
        env["ROS_LOCALHOST_ONLY"] = "1" if self.localhost_var.get() else "0"
        env["ESKF_COMPARE_BAG"] = str(bag_dir)
        if target_yaml is not None:
            env["ESKF_COMPARE_TARGET_YAML"] = target_yaml
        if self.cache_manifest is not None:
            env["ESKF_REPLAY_CACHE_DATABASE"] = self.cache_manifest["database_path"]
            env["ESKF_REPLAY_CACHE_PROCESS_LOG"] = self.cache_manifest["process_log"]
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
        if self.profile.get("ordinal_reader"):
            command = [
                sys.executable,
                str(Path(__file__).resolve().parent / "progress_player" /
                    "ordinal_replay_reader.py"),
                str(bag_dir),
            ]
            if self.cache_manifest is not None:
                command.append("--display-only")
        else:
            command = [
                "ros2", "bag", "play", str(bag_dir), "--clock", "30",
                "--start-paused", "--disable-keyboard-controls",
            ]
            command.extend(self.profile.get("bag_play_args", []))
        supervised_command = [
            sys.executable,
            str(Path(__file__).resolve().parent / "progress_player" /
                "managed_stack_supervisor.py"),
            "--",
            *command,
        ]
        try:
            self.process = subprocess.Popen(
                supervised_command, env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
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
        self.node = PlayerControl(
            self.on_clock, self.on_restore_result,
            self.on_ordinal_seek_result, self.on_target_ordinal_result,
            self.on_reader_status, self.on_snapshot_result,
            self.on_seal_result, self.context)
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
        if (self.profile.get("checkpoint_restore") and self.bag_dir and
                not self.profile.get("ordinal_reader") and
                not self.paused and self.rebuild_target_ns is None and
                self.current_ns >= self.start_ns + self.duration_ns - 100_000_000):
            self.node.pause()
            self.paused = True
            if not self.first_pass_complete:
                self.first_pass_complete = True
                self.node.seal_first_pass(self.current_ns)

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
            elif self.cache_manifest is not None:
                self.status_var.set(
                    f"只读缓存回放：可直接拖动，不启动 ESKF（{active}）")
            else:
                self.status_var.set(
                    f"已暂停，可拖动进度条（{active}）" if ready and self.paused
                    else (f"正在播放（{active}）" if ready else f"正在等待播放器服务（{active}）…"))

        if (not self.determinism_active and self.rebuild_target_ns is not None and
                self.current_ns >= self.rebuild_target_ns):
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
        if self.cache_manifest is not None:
            epoch = self.epoch_clock.next(target_ns, True)
            self.node.pause()
            self.paused = True
            self.current_ns = target_ns
            self.node.announce_visual_seek(target_ns / 1e9, epoch.epoch, True)
            future = self.node.seek(target_ns)
            future.add_done_callback(
                lambda result: self.root.after(0, self.seek_done, result))
            self.play_button.configure(text="▶ 播放")
            return
        if self.profile.get("ordinal_reader"):
            if not self.first_pass_sealed:
                self.status_var.set("需先完整播放并确认 checkpoint/过程日志已封存")
                return
            self.determinism_active = True
            self.determinism_target_ns = target_ns
            self.determinism_target_ordinal = None
            self.determinism_runs = []
            self.determinism_snapshots = []
            self.rebuild_target_ns = target_ns
            self.start_determinism_epoch()
            return
        epoch = self.epoch_clock.next(target_ns, self.first_pass_complete)
        self.node.announce_visual_seek(
            target_ns / 1e9, epoch.epoch, epoch.first_pass_complete)
        if not self.profile.get("managed_command"):
            self.current_ns = target_ns
            future = self.node.seek(target_ns)
            future.add_done_callback(lambda result: self.root.after(0, self.seek_done, result))
            return
        if self.profile.get("checkpoint_restore"):
            self.node.pause()
            self.paused = True
            self.checkpoint_restore_pending_ns = target_ns
            self.status_var.set("正在等待 ESKF 原子恢复 checkpoint…")
            self.root.after(300, lambda: self.node and self.node.restore_checkpoint(
                target_ns / 1e9))
            return
        self.node.pause()
        self.paused = True
        self.stop_managed_process()
        self.current_ns = self.start_ns
        future = self.node.seek(self.start_ns)
        self.rebuild_target_ns = target_ns
        future.add_done_callback(lambda result: self.root.after(0, self.begin_rebuild, result))

    def start_determinism_epoch(self):
        if self.node is None or self.determinism_target_ns is None:
            return
        epoch = self.epoch_clock.next(self.determinism_target_ns, True)
        self.determinism_retry_count = 0
        self.node.pause()
        self.paused = True
        self.node.announce_visual_seek(
            self.determinism_target_ns / 1e9, epoch.epoch, True)
        self.status_var.set(
            f"bit-exact 恢复 {len(self.determinism_runs) + 1}/3：解析目标序号…")
        self.root.after(100, lambda: self.node and self.node.request_target_ordinal(
            epoch.epoch, self.determinism_target_ns))

    def on_target_ordinal_result(self, message: String):
        self.root.after(0, self.handle_target_ordinal_result, message.data)

    def handle_target_ordinal_result(self, payload: str):
        if not self.determinism_active or self.node is None:
            return
        try:
            result = json.loads(payload)
            if result.get("schema") != "rosbag_target_ordinal_result/v1":
                return
            if (result.get("reason") == "wrong_epoch" and
                    int(result.get("requested_epoch", -1)) == self.epoch_clock.current and
                    self.determinism_retry_count < 20):
                self.determinism_retry_count += 1
                self.root.after(100, lambda: self.node and
                                self.node.request_target_ordinal(
                                    self.epoch_clock.current,
                                    self.determinism_target_ns))
                return
            if int(result.get("epoch", -1)) != self.epoch_clock.current:
                return
            if not result.get("success"):
                raise RuntimeError(result.get("reason", "target ordinal rejected"))
            self.determinism_target_ordinal = int(result["target_ordinal"])
            self.checkpoint_restore_pending_ns = self.determinism_target_ns
            self.node.restore_ordinal(
                self.epoch_clock.current, self.determinism_target_ordinal)
        except Exception as exc:
            self.fail_determinism(f"目标序号失败：{exc}")

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

    def on_restore_result(self, message: String):
        self.root.after(0, self.handle_restore_result, message.data)

    def handle_restore_result(self, payload: str):
        if self.checkpoint_restore_pending_ns is None or self.node is None:
            return
        target_ns = self.checkpoint_restore_pending_ns
        self.checkpoint_restore_pending_ns = None
        try:
            result = json.loads(payload)
            if not result.get("success"):
                if (self.determinism_active and
                        result.get("reason") == "unavailable" and
                        self.determinism_retry_count < 20):
                    self.determinism_retry_count += 1
                    self.checkpoint_restore_pending_ns = target_ns
                    self.root.after(100, lambda: self.node and
                                    self.node.restore_ordinal(
                                        self.epoch_clock.current,
                                        self.determinism_target_ordinal))
                    return
                raise RuntimeError(result.get("reason", "checkpoint restore rejected"))
            checkpoint_ns = int(round(float(result["checkpoint_stamp_s"]) * 1e9))
            self.current_ns = checkpoint_ns
            self.rebuild_target_ns = target_ns
            checkpoint_ordinal = result.get("checkpoint_bag_ordinal")
            if self.profile.get("ordinal_reader"):
                if checkpoint_ordinal is None:
                    raise RuntimeError("checkpoint 没有 bag ordinal")
                checkpoint_ordinal = int(checkpoint_ordinal)
                if (self.determinism_active and
                        checkpoint_ordinal >= self.determinism_target_ordinal):
                    self.node.request_snapshot(
                        self.epoch_clock.current, self.determinism_target_ordinal)
                else:
                    self.node.seek_ordinal(
                        self.epoch_clock.current, checkpoint_ordinal + 1)
                return
            future = self.node.seek(checkpoint_ns + 1)
            future.add_done_callback(
                lambda completed: self.root.after(
                    0, self.begin_checkpoint_forward_replay, completed))
        except Exception as exc:
            if self.determinism_active:
                self.fail_determinism(f"Checkpoint 恢复失败：{exc}")
            else:
                self.status_var.set(f"Checkpoint 恢复失败：{exc}")

    def on_ordinal_seek_result(self, message: String):
        self.root.after(0, self.handle_ordinal_seek_result, message.data)

    def handle_ordinal_seek_result(self, payload: str):
        if self.rebuild_target_ns is None:
            return
        try:
            result = json.loads(payload)
            if result.get("schema") != "rosbag_ordinal_seek_result/v1":
                return
            if int(result.get("epoch", -1)) != self.epoch_clock.current:
                return
            if not result.get("success"):
                raise RuntimeError(result.get("reason", "ordinal seek rejected"))
            self.begin_checkpoint_forward_replay(None)
        except Exception as exc:
            if self.determinism_active:
                self.fail_determinism(f"Ordinal 补算定位失败：{exc}")
            else:
                self.rebuild_target_ns = None
                self.status_var.set(f"Ordinal 补算定位失败：{exc}")

    def on_reader_status(self, message: String):
        self.root.after(0, self.handle_reader_status, message.data)

    def handle_reader_status(self, payload: str):
        if self.node is None:
            return
        try:
            result = json.loads(payload)
            if result.get("schema") != "rosbag_ordinal_reader_status/v1":
                return
            if result.get("state") == "end_of_bag":
                self.paused = True
                self.play_button.configure(text="▶ 播放")
                if not self.first_pass_complete:
                    self.first_pass_complete = True
                    self.node.seal_first_pass(self.current_ns)
                return
            if (not self.determinism_active or
                    int(result.get("epoch", -1)) != self.epoch_clock.current):
                return
            if result.get("state") != "target_reached":
                raise RuntimeError(result.get("state", "reader failed"))
            if int(result["bag_ordinal"]) != self.determinism_target_ordinal:
                raise RuntimeError("reader stopped at a different ordinal")
            self.paused = True
            self.node.request_snapshot(
                self.epoch_clock.current, self.determinism_target_ordinal)
        except Exception as exc:
            self.fail_determinism(f"顺序注入失败：{exc}")

    def on_snapshot_result(self, message: String):
        self.root.after(0, self.handle_snapshot_result, message.data)

    def handle_snapshot_result(self, payload: str):
        if not self.determinism_active:
            return
        try:
            snapshot = json.loads(payload)
            if (snapshot.get("schema") != "eskf_replay_snapshot/v2" or
                    int(snapshot.get("epoch", -1)) != self.epoch_clock.current):
                return
            if not snapshot.get("success"):
                raise RuntimeError(snapshot.get("reason", "snapshot rejected"))
            self.determinism_runs.append({
                "epoch": self.epoch_clock.current, **snapshot_hashes(snapshot)})
            self.determinism_snapshots.append(snapshot)
            if len(self.determinism_runs) < 3:
                self.start_determinism_epoch()
                return
            comparison = compare_three(self.determinism_runs)
            final_log_size = self.active_process_log_size()
            checkpoint_counts = [int(snapshot["node_state_summary"][
                "full_replay_checkpoints"]) for snapshot in self.determinism_snapshots]
            manifest_counts = [len(snapshot["full_checkpoint_state"])
                               for snapshot in self.determinism_snapshots]
            immutable = (self.sealed_checkpoint_count is not None and
                         all(count == self.sealed_checkpoint_count
                             for count in checkpoint_counts) and
                         all(count == FULL_CHECKPOINT_FIELD_COUNT
                             for count in manifest_counts) and
                         self.sealed_log_size is not None and
                         final_log_size == self.sealed_log_size)
            comparison["bit_exact"] = comparison["bit_exact"] and immutable
            report = {"schema": "eskf_bit_exact_report/v2",
                      "bag": str(self.bag_dir),
                      "target_ns": self.determinism_target_ns,
                      "target_ordinal": self.determinism_target_ordinal,
                      "runs": self.determinism_runs,
                      "sealed_checkpoint_count": self.sealed_checkpoint_count,
                      "checkpoint_counts_after_runs": checkpoint_counts,
                      "full_checkpoint_manifest_counts": manifest_counts,
                      "sealed_log_size": self.sealed_log_size,
                      "final_log_size": final_log_size,
                      "first_pass_storage_immutable": immutable,
                      "first_differences": first_differences(
                          self.determinism_snapshots), **comparison}
            report_dir = Path(__file__).resolve().parent / "determinism_reports"
            report_dir.mkdir(exist_ok=True)
            report_path = report_dir / (
                f"target_{self.determinism_target_ns}_{int(time.time())}.json")
            report_path.write_text(json.dumps(report, indent=2, sort_keys=True),
                                   encoding="utf-8")
            self.determinism_active = False
            self.rebuild_target_ns = None
            self.status_var.set(
                ("bit-exact 通过" if comparison["bit_exact"] else "bit-exact 失败") +
                f"：三次状态/协方差/轨迹哈希，报告 {report_path}")
        except Exception as exc:
            self.fail_determinism(f"快照比较失败：{exc}")

    def on_seal_result(self, message: String):
        self.root.after(0, self.handle_seal_result, message.data)

    def handle_seal_result(self, payload: str):
        try:
            result = json.loads(payload)
            if (result.get("schema") == "eskf_replay_seal_result/v1" and
                    result.get("success") and result.get("checkpoint_frozen") and
                    result.get("log_sealed")):
                self.first_pass_sealed = True
                self.sealed_checkpoint_count = int(result["checkpoint_count"])
                self.sealed_log_size = self.active_process_log_size()
                self.status_var.set("首次播放已完成：过程日志和 checkpoint 已封存")
                self.start_cache_builder()
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def fail_determinism(self, detail: str):
        self.determinism_active = False
        self.rebuild_target_ns = None
        self.checkpoint_restore_pending_ns = None
        self.paused = True
        self.status_var.set(detail)

    def active_process_log_size(self):
        path = self.active_process_log_path()
        return path.stat().st_size if path else None

    def active_process_log_path(self):
        runs = Path(__file__).resolve().parent / "progress_player" / "runs"
        candidates = sorted(
            runs.glob(f"*-domain{self.active_domain_id}-*/eskf_process.jsonl"),
            key=lambda path: path.stat().st_mtime_ns, reverse=True)
        return candidates[0] if candidates else None

    def start_cache_builder(self):
        if (self.bag_dir is None or self.cache_fingerprint is None or
                self.cache_builder is not None):
            return
        process_log = self.active_process_log_path()
        target_yaml = resolve_target_yaml(self.profile, self.bag_dir)
        if process_log is None or target_yaml is None:
            return
        root = Path(__file__).resolve().parent
        command = [
            sys.executable, str(root / "progress_player" /
                                "build_persistent_replay_cache.py"),
            "--bag", str(self.bag_dir), "--target-yaml", target_yaml,
            "--build-state", str(root / "progress_player" / "build_state" /
                                 "eskf_compare.json"),
            "--process-log", str(process_log),
            "--cache-root", str(root / "progress_player" / "cache"),
        ]
        log_path = root / "tmp" / "rosbag_progress_player_logs" / "cache_builder.log"
        stream = log_path.open("ab")
        try:
            self.cache_builder = subprocess.Popen(
                command, stdout=stream, stderr=subprocess.STDOUT)
        finally:
            stream.close()
        self.status_var.set("首次结果已封存，正在建立持久化轨迹和过程量时间索引…")
        self.root.after(1000, self.poll_cache_builder)

    def poll_cache_builder(self):
        if self.cache_builder is None:
            return
        status = self.cache_builder.poll()
        if status is None:
            self.root.after(1000, self.poll_cache_builder)
            return
        self.cache_builder = None
        if status != 0 or self.cache_fingerprint is None:
            self.status_var.set(
                "持久化缓存建立失败；本次状态仍可用，查看 cache_builder.log")
            return
        manifest = load_valid_manifest(
            Path(__file__).resolve().parent / "progress_player" / "cache",
            self.cache_fingerprint)
        if manifest is None:
            self.status_var.set("持久化缓存校验失败；下次将重新完整计算")
            return
        self.status_var.set("持久化缓存已建立；下次打开相同 bag 将进入只读纯回放")

    def begin_checkpoint_forward_replay(self, future):
        try:
            if future is not None and not future.result().success:
                raise RuntimeError("rosbag reader 无法定位到 checkpoint cursor")
            self.node.set_rate(1.0)
            self.node.resume()
            self.paused = False
        except Exception as exc:
            self.rebuild_target_ns = None
            self.status_var.set(f"Checkpoint 补算失败：{exc}")

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
        command = (["bash", str(Path(__file__).resolve().parent /
                                "progress_player" / "run_cached_replay_stack.sh")]
                   if self.cache_manifest is not None else
                   self.profile.get("managed_command"))
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
        self.checkpoint_restore_pending_ns = None
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
    parser.add_argument("--title", help="window title used by multi-bag layouts")
    parser.add_argument("--geometry", help="Tk geometry, for example 640x310+0+770")
    args = parser.parse_args()
    root = tk.Tk()
    player = ProgressPlayer(root, args.bag, args.profile, args.title, args.geometry)

    def request_close(_signum, _frame):
        root.after(0, player.close)

    signal.signal(signal.SIGINT, request_close)
    signal.signal(signal.SIGTERM, request_close)
    root.mainloop()


if __name__ == "__main__":
    main()
