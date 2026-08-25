#!/usr/bin/env python3
import importlib.util
import subprocess
import sys
import time
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("managed_stack_supervisor.py")
SPEC = importlib.util.spec_from_file_location("managed_stack_supervisor", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SupervisorTest(unittest.TestCase):
    def test_terminate_group_stops_child_session(self):
        process = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(30)"],
            start_new_session=True,
        )
        MODULE.terminate_group(process, timeout_s=1.0)
        self.assertIsNotNone(process.poll())

    def test_signal_stops_supervised_command(self):
        supervisor = subprocess.Popen([
            sys.executable, str(MODULE_PATH), "--", sys.executable, "-c",
            "import time; time.sleep(30)",
        ])
        time.sleep(0.2)
        supervisor.terminate()
        self.assertIsNotNone(supervisor.wait(timeout=3.0))


if __name__ == "__main__":
    unittest.main()
