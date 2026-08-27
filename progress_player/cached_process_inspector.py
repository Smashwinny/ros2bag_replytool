#!/usr/bin/env python3
"""Floating cached-process window with access to the complete indexed record."""

import argparse
import json
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class InspectorNode(Node):
    def __init__(self, callback):
        super().__init__("eskf_cached_process_inspector")
        self.create_subscription(String, "/rosbag_progress/cached_process_record",
                                 lambda msg: callback(msg.data), 10)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--title", default="ESKF Cached Process")
    parser.add_argument("--geometry", default="720x118+0+0")
    args, ros_args = parser.parse_known_args()
    root = tk.Tk()
    root.title(args.title)
    root.geometry(args.geometry)
    compact = tk.StringVar(value="缓存过程量：等待 /clock…")
    detail = {"text": ""}
    ttk.Label(root, textvariable=compact, anchor="w").pack(fill="x", padx=8, pady=5)

    def show_full():
        window = tk.Toplevel(root)
        window.title("当前时刻完整 ESKF 过程量")
        text = tk.Text(window, wrap="none", width=120, height=40)
        text.pack(fill="both", expand=True)
        text.insert("1.0", detail["text"])
        text.configure(state="disabled")

    ttk.Button(root, text="查看当前时刻全部过程量", command=show_full).pack(pady=2)

    def received(payload):
        try:
            record = json.loads(payload)
            detail["text"] = json.dumps(record, indent=2, ensure_ascii=False)
            summary = (f"{record.get('event')}  t={record.get('source_time_s')}  "
                       f"state={record.get('init_state')}  "
                       f"IMU/GNSS/Odom={record.get('imu_trust_state')}/"
                       f"{record.get('gnss_trust_state')}/"
                       f"{record.get('odom_trust_state')}")
            root.after(0, compact.set, summary)
        except (TypeError, json.JSONDecodeError):
            pass

    rclpy.init(args=ros_args)
    node = InspectorNode(received)
    stop = threading.Event()

    def spin():
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()

    def close():
        stop.set()
        thread.join(timeout=1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)
    root.mainloop()


if __name__ == "__main__":
    main()
