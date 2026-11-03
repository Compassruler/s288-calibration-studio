"""Hand-position, capture/save, reload and return to the recorded joint poses."""
import math
from pathlib import Path
import sys
import tempfile
import time
import tkinter as tk
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import app
from config import default_profiles, load_profiles
from test_real_id_control import WireSerial


class ZeroWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.path = Path(self.folder.name)/"calibration.json"
        self.wire = WireSerial()
        self.enterContext(patch("serial.Serial", return_value=self.wire))
        self.enterContext(patch("controller.SimBus", side_effect=AssertionError("wire test only")))
        self.enterContext(patch.object(app.CalibratorApp, "_refresh_ports"))
        self.errors = self.enterContext(patch.object(app.messagebox, "showerror"))
        self.root = tk.Tk()
        self.root.withdraw()
        self.callback_errors = []
        self.root.report_callback_exception = lambda *error: self.callback_errors.append(error)
        self.application = app.CalibratorApp(self.root, config_path=self.path)
        self.application.port.set("OFFLINE_ZERO_WORKFLOW")
        self.application.ids.set("0,7,14")
        self.application._connect()
        self.addCleanup(self.cleanup)
        self.wait(lambda: self.application.connected)

    def wait(self, predicate, timeout=6):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.root.update()
            if predicate():
                return
            time.sleep(.005)
        self.fail(f"zero workflow did not finish: {self.application.home_text.get()}")

    def cleanup(self):
        self.application.controller.send("shutdown")
        self.application.controller.join(3)
        self.assertFalse(self.application.controller.is_alive())
        for callback in self.root.tk.call("after", "info"):
            self.root.after_cancel(callback)
        self.root.destroy()
        self.folder.cleanup()
        self.assertEqual(self.callback_errors, [])

    def test_hand_position_is_saved_and_all_servos_return_and_keep_holding(self):
        application = self.application
        poses = {0:.12, 7:-.24, 14:.36}
        for motor_id, pose in poses.items():
            with self.wire.lock:
                self.wire.positions[motor_id] = pose
            application.tree.selection_set(str(motor_id))
            self.wait(lambda: application.selected == motor_id and application.controller.selected == motor_id)
            started = time.monotonic()
            self.wait(lambda: time.monotonic()-started > .7)
            application.capture_button.invoke()
            self.wait(lambda: application.profiles[motor_id]["zero_offset"] is not None
                      and application._capture_save_id is None)
            self.assertAlmostEqual(load_profiles(self.path)[motor_id]["zero_offset"], pose, places=5)
        self.assertTrue(all(request["kp"] == 0 for request in self.wire.sent()))
        saved = self.path.read_bytes()
        application.profiles = default_profiles()
        application._load_file(self.path)
        self.assertTrue(all(request["kp"] == 0 for request in self.wire.sent()))
        with self.wire.lock:
            for motor_id, pose in poses.items():
                self.wire.positions[motor_id] = pose+math.radians(4)
        application._home_all()
        self.wait(lambda: not application.homing and application.holding_ids == set(poses))
        self.assertIn("回零完成", application.home_text.get())
        for motor_id, pose in poses.items():
            self.assertAlmostEqual(self.wire.positions[motor_id], pose, places=5)
        self.assertEqual(self.path.read_bytes(), saved)
        self.assertTrue(application.angle_slider.instate(["disabled"]))
        self.assertTrue(application.capture_button.instate(["disabled"]))
        application._cancel_home()
        self.wait(lambda: not application.holding_ids and application.controller.armed is None)
        self.errors.assert_not_called()

    def test_all_home_requires_every_recorded_id_to_be_connected(self):
        application = self.application
        application.profiles[3]["zero_offset"] = .1
        application._home_all()
        self.assertFalse(application.homing)
        self.assertIn("未连接", str(self.errors.call_args))
        self.assertFalse(any(request["kp"] for request in self.wire.sent()))


if __name__ == "__main__":
    unittest.main()
