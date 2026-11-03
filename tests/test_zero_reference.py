"""Connection origins must not masquerade as saved mechanical calibration."""
import json
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
import controller
from config import default_profiles, save_profiles
from protocol import SimBus
import ui


class ZeroReferenceTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.path = Path(self.folder.name) / "demo.json"
        profiles = default_profiles()
        profiles[14].update(zero_offset=.5, reference_expos=.5)
        save_profiles(self.path, profiles, simulation=True)
        self.saved = self.path.read_bytes()
        self.enterContext(patch.object(ui, "THEME_PATH", self.path.with_name("appearance.json")))
        self.enterContext(patch.object(app.CalibratorApp, "_refresh_ports"))
        self.serial = self.enterContext(patch("serial.Serial", side_effect=AssertionError("no real motor")))
        self.root = tk.Tk()
        self.root.withdraw()
        self.errors = []
        self.root.report_callback_exception = lambda *error: self.errors.append(error)
        self.application = app.CalibratorApp(self.root, demo=True, config_path=self.path)
        self.addCleanup(self.cleanup)
        self.pump_until(lambda: self.application.connected and 14 in self.application.snapshots)

    def cleanup(self):
        self.application.controller.send("shutdown")
        self.application.controller.join(2)
        self.assertFalse(self.application.controller.is_alive())
        for callback in self.root.tk.call("after", "info"):
            self.root.after_cancel(callback)
        self.root.destroy()
        self.folder.cleanup()
        self.serial.assert_not_called()
        self.assertEqual(self.errors, [])

    def pump_until(self, condition):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            self.root.update()
            if condition():
                return
            time.sleep(.005)
        self.fail("expected simulation state was not displayed")

    def test_reconnection_keeps_saved_zero_instead_of_using_the_new_connection_origin(self):
        tool = self.application
        self.assertIn("零位角度", tool.position_caption.get())
        tool._disconnect()
        self.pump_until(lambda: not tool.connected)

        def moved_motor():
            bus = SimBus()
            bus.positions[14] = .9
            return bus

        with patch.object(controller, "SimBus", side_effect=moved_motor):
            tool._connect()
            self.pump_until(lambda: tool.connected and 14 in tool.snapshots
                            and tool.snapshots[14]["origin"] == .9)
        self.assertAlmostEqual(float(tool.position_text.get().removesuffix("°")),
                               math.degrees(.9-.5), places=2)
        self.assertEqual(tool.profiles[14]["zero_offset"], .5)
        self.assertEqual(tool.profiles[14]["reference_expos"], .5)
        self.assertEqual(self.path.read_bytes(), self.saved)
        self.assertTrue(all(packet["kp"] == packet["kd"] == 0
                            for packet in tool.controller.bus.history))

    def test_unrecorded_id_is_labelled_temporary_and_live_angles_still_follow_feedback(self):
        tool = self.application
        tool.tree.selection_set("0")
        self.pump_until(lambda: tool.selected == 0 and 0 in tool.snapshots)
        self.assertIn("临时角度", tool.position_caption.get())
        self.assertIn("未标定", tool.tree.item("0", "values")[2])
        self.assertIn("本次连接临时0°", tool.zero_text.get())
        self.assertIsNone(tool.profiles[0]["zero_offset"])
        origin = tool.snapshots[0]["origin"]
        tool.controller.bus.positions[0] = origin + .06
        self.pump_until(lambda: abs(tool.snapshots[0]["feedback"]["raw_pos"] - origin - .06) < 1e-9)
        self.assertAlmostEqual(float(tool.position_text.get().removesuffix("°")), math.degrees(.06), places=2)
        self.assertIn("临时角度", tool.position_caption.get())
        self.assertEqual(self.path.read_bytes(), self.saved)
        self.assertEqual(sum(joint["zero_offset"] is not None for joint in tool.profiles.values()), 1)

    def test_pose_timeout_updates_ui_without_disconnect_or_automatic_reenable(self):
        tool = self.application
        worker, bus = tool.controller, tool.controller.bus
        progress = worker._check_home_progress

        def expire_home():
            if worker.homing:
                worker.homing["deadline"] = time.monotonic() - 1
            progress()

        with patch.object(worker, "_check_home_progress", side_effect=expire_home):
            tool.current_home_button.invoke()
            self.pump_until(lambda: "连接保留" in tool.home_text.get())
        self.assertTrue(tool.connected)
        self.assertFalse(tool.faulted)
        self.assertFalse(tool.homing)
        self.assertIsNone(tool.armed_id)
        self.assertFalse(tool.holding_ids)
        self.assertFalse(tool.arm_button.instate(["disabled"]))
        self.assertFalse(tool.current_home_button.instate(["disabled"]))
        self.assertIs(worker.bus, bus)
        self.assertFalse(bus.closed)
        before = worker.feedback[14]["timestamp"]
        self.pump_until(lambda: tool.snapshots[14]["feedback"]["timestamp"] > before + .05)
        self.assertIsNone(worker.armed)
        self.assertEqual(tool.profiles[14]["zero_offset"], .5)
        self.assertFalse(tool.dirty)
        self.assertEqual(self.path.read_bytes(), self.saved)


if __name__ == "__main__":
    unittest.main()
