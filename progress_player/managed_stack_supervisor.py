#!/usr/bin/env python3
"""Ensure a replay-managed process group dies when its player parent disappears."""

import argparse
import os
import signal
import subprocess
import sys
import time


def terminate_group(process: subprocess.Popen, timeout_s: float = 3.0) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + timeout_s
    while process.poll() is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("a managed command is required after --")

    owner_pid = os.getppid()
    stopping = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stopping
        stopping = True

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, request_stop)

    process = subprocess.Popen(command, start_new_session=True)
    try:
        while process.poll() is None:
            if stopping or os.getppid() != owner_pid:
                terminate_group(process)
                break
            time.sleep(0.25)
        return process.wait()
    finally:
        terminate_group(process)


if __name__ == "__main__":
    sys.exit(main())
