"""Idle work reduction must preserve feedback and control-state updates."""

from pathlib import Path
import sys
import tempfile
import time
import tkinter as tk
from tkinter import ttk
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import app


class IdleRefreshTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.serial = self.enterContext(patch("serial.Serial", side_effect=AssertionError("no hardware in UI tests")))
        self.enterContext(patch.object(app.CalibratorApp, "_refresh_ports", return_value=None))
        self.enterContext(patch.object(app.messagebox, "showerror", return_value=None))
        self.enterContext(patch.object(app.messagebox, "showwarning", return_value=None))
        self.enterContext(patch.object(app.messagebox, "askyesno", return_value=True))
        self.root = tk.Tk()
        self.root.withdraw()
        self.errors = []
        self.root.report_callback_exception = lambda *error: self.errors.append(error)
        path = Path(self.folder.name) / "unused.json"
        self.app = app.CalibratorApp(self.root, demo=False, config_path=path)
        self.addCleanup(self.cleanup_app)
        self.root.update()

    def cleanup_app(self):
        self.app.controller.send("shutdown")
        self.app.controller.join(2)
        self.assertFalse(self.app.controller.is_alive())
        for callback in self.root.tk.call("after", "info"):
            self.root.after_cancel(callback)
        self.root.update_idletasks()
        self.root.destroy()
        self.folder.cleanup()
        self.serial.assert_not_called()
        self.assertEqual(self.errors, [])

    def pump_until(self, condition, timeout=2):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.root.update()
            if condition():
                return
            time.sleep(.005)
        self.fail("UI did not update within deadline")

    def pump_for(self, duration=.18):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.root.update()
            time.sleep(.005)

    def connect_demo(self):
        self.app.demo.set(True)
        self.app._connect()
        self.pump_until(lambda: self.app.connected and 14 in self.app.snapshots)

    def test_disconnected_idle_poll_still_services_events_without_rendering(self):
        with patch.object(self.app, "_render", wraps=self.app._render) as render:
            self.pump_for()
            render.assert_not_called()
            self.app.events.put({"type": "status", "message": "new worker event"})
            self.pump_until(lambda: render.call_count > 0)
            self.assertEqual(render.call_count, 1)
            self.pump_for(.1)
            self.assertEqual(render.call_count, 1)

    def test_unchanged_render_does_not_write_tk_text_rows_or_options(self):
        text_writes, option_writes = [], []
        original_set = tk.StringVar.set
        original_configure = ttk.Widget.configure

        def set_text(variable, value):
            text_writes.append((str(variable), value))
            return original_set(variable, value)

        def configure(widget, *args, **kwargs):
            option_writes.append((str(widget), args, kwargs))
            return original_configure(widget, *args, **kwargs)

        with patch.object(tk.StringVar, "set", new=set_text), \
                patch.object(ttk.Widget, "configure", new=configure), \
                patch.object(self.app.tree, "item", wraps=self.app.tree.item) as rows:
            for _ in range(100):
                self.app._render()
            rows.assert_not_called()
        self.assertEqual(text_writes, [])
        self.assertEqual(option_writes, [])

    def test_offline_profile_change_updates_only_the_changed_row(self):
        with patch.object(self.app.tree, "item", wraps=self.app.tree.item) as rows:
            self.app.profiles[14]["name"] = "左膝关节"
            self.app._show_profile()
            self.assertEqual(rows.call_count, 1)
            self.assertEqual(rows.call_args.args[0], "14")
            self.assertEqual(self.app.tree.item("14", "values")[1], "左膝关节")

    def test_new_feedback_and_arm_release_states_still_update_promptly(self):
        self.connect_demo()
        self.assertFalse(self.app.arm_button.instate(["disabled"]))
        before = float(self.app.values["raw"].get())
        self.app.controller.bus.positions[14] += .01
        self.pump_until(lambda: float(self.app.values["raw"].get()) > before + .009)
        self.app.arm_button.invoke()
        self.pump_until(lambda: self.app.armed_id == 14 and not self.app.angle_slider.instate(["disabled"]))
        self.assertTrue(self.app.capture_button.instate(["disabled"]))
        self.app.release_button.invoke()
        self.pump_until(lambda: self.app.armed_id is None and self.app.angle_slider.instate(["disabled"]))
        self.assertFalse(self.app.capture_button.instate(["disabled"]))

    def test_connected_poll_marks_feedback_stale_without_new_events(self):
        self.connect_demo()
        with patch.object(self.app, "_handle", return_value=None):
            self.app.snapshots[14]["feedback"]["timestamp"] = time.monotonic() - 1
            self.pump_for(.08)
            self.assertTrue(self.app.arm_button.instate(["disabled"]))
            self.assertTrue(self.app.capture_button.instate(["disabled"]))
            self.assertEqual(self.app.position_text.get(), "—")
            self.assertEqual(self.app.tree.item("14", "values")[2], "反馈过期")


if __name__ == "__main__":
    unittest.main()
