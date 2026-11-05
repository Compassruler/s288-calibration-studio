"""Exit integration checks with a loaded preview and simulated transport only."""
import copy
import gc
from pathlib import Path
import sys
import tempfile
import time
import tkinter as tk
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import app
import robot_view
from test_embedded_preview import PreparedRenderer
from test_robot_view import scene


class ShutdownTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.folder = Path(self.directory.name)
        self.path = self.folder / "calibration.json"
        profiles = app.default_profiles()
        for mid, profile in profiles.items():
            profile.update(zero_offset=mid + .25, reference_expos=.5,
                           min_deg=-60., max_deg=60., kp=1., kd=.01)
        app.save_profiles(self.path, profiles, simulation=False)
        self.original = self.path.read_bytes()
        self.serial = self.enterContext(patch("serial.Serial", side_effect=AssertionError("no real serial")))
        self.enterContext(patch.object(app.CalibratorApp, "_refresh_ports", return_value=None))
        self.enterContext(patch.object(app.messagebox, "askyesno", return_value=True))
        self.enterContext(patch.object(app.messagebox, "showwarning"))
        self.enterContext(patch.object(app.messagebox, "showerror"))
        model = scene()
        model["_fingerprint"] = "shutdown-test"
        self.enterContext(patch.object(robot_view, "load_scene", return_value=model))
        self.enterContext(patch.object(robot_view, "_Renderer", PreparedRenderer))
        self.enterContext(patch.object(robot_view, "BINDINGS_PATH", self.folder / "bindings.json"))
        self.root = tk.Tk()
        self.root.withdraw()
        self.callback_errors = []
        self.root.report_callback_exception = lambda *exc: self.callback_errors.append(exc)
        self.tool = app.CalibratorApp(self.root, config_path=self.path)
        self.tool._ensure_robot_view()
        self.addCleanup(self.cleanup)
        self.wait_until(lambda: self.tool.robot_view.renderer is not None)
        self.viewer = self.tool.robot_view
        self.assertTrue(self.viewer.reference_button.winfo_exists())
        self.profiles = copy.deepcopy(self.tool.profiles)

    def exists(self):
        try:
            return bool(self.root.winfo_exists())
        except tk.TclError:
            return False

    def wait_until(self, predicate, timeout=2):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.exists():
                self.root.update()
            if predicate():
                return
            time.sleep(.005)
        self.fail("Shutdown did not finish before its deadline")

    def cleanup(self):
        self.tool.controller.send("shutdown")
        self.tool.controller.join(2)
        if self.exists():
            for callback in self.root.tk.call("after", "info"):
                self.root.after_cancel(callback)
            try:
                self.viewer.close()
            finally:
                self.root.destroy()
        self.directory.cleanup()
        self.serial.assert_not_called()
        gc.collect()

    def assert_exited(self):
        self.wait_until(lambda: not self.exists() and not self.tool.controller.is_alive())
        self.assertEqual(self.callback_errors, [])
        self.assertEqual(self.path.read_bytes(), self.original)
        self.assertEqual(self.tool.profiles, self.profiles)

    def test_loaded_preview_offline_close_exits_without_touching_settings(self):
        self.tool._close()
        self.assert_exited()

    def test_repeated_close_still_completes_shutdown(self):
        with patch.object(self.tool.controller, "send", wraps=self.tool.controller.send) as sent:
            self.tool._close()
            self.tool._close()
            self.assert_exited()
        self.assertEqual([call.args[0] for call in sent.call_args_list], ["shutdown"])

    def test_renderer_cleanup_failure_does_not_prevent_transport_shutdown(self):
        with patch.object(self.viewer.renderer, "close", side_effect=RuntimeError("GL cleanup failed")):
            self.tool._close()
            self.assert_exited()

    def test_closed_preview_reset_does_not_touch_destroyed_widgets(self):
        self.viewer._temporary_expos_references["14"] = 1.2
        self.viewer.close()
        self.viewer.reset_temporary_references()
        self.assertEqual(self.viewer._temporary_expos_references, {})
        self.tool._close()
        self.assert_exited()

    def test_connected_simulator_is_released_and_closed_before_exit(self):
        self.tool.controller.send("connect", port="", ids=[14], demo=True)
        self.wait_until(lambda: self.tool.connected and 14 in self.tool.snapshots)
        self.tool._arm()
        self.wait_until(lambda: self.tool.armed_id == 14)
        bus = self.tool.controller.bus
        with patch.object(bus, "exchange", wraps=bus.exchange) as exchange:
            self.tool._close()
            self.assert_exited()
        self.assertTrue(bus.closed)
        self.assertTrue(exchange.call_args_list)
        self.assertTrue(any(command["kp"] > 0 for command in bus.history))
        self.assertEqual((bus.history[-1]["kp"], bus.history[-1]["kd"]), (0, 0))

    def test_cancel_unsaved_changes_keeps_window_and_worker_running(self):
        self.tool.dirty = True
        with patch.object(app.messagebox, "askyesno", return_value=False), \
                patch.object(self.tool.controller, "send", wraps=self.tool.controller.send) as sent:
            self.tool._close()
        sent.assert_not_called()
        self.assertTrue(self.exists())
        self.assertTrue(self.tool.controller.is_alive())
        self.assertFalse(self.tool.closing)
        self.assertTrue(self.viewer.reference_button.winfo_exists())
        self.assertEqual(self.path.read_bytes(), self.original)
        self.tool._close()
        self.assert_exited()


if __name__ == "__main__":
    unittest.main(verbosity=2)
