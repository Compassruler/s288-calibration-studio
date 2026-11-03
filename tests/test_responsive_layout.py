"""Main-window density changes preserve state, dialogs and callback lifetime."""
import copy
from pathlib import Path
import sys
import tempfile
import time
import tkinter as tk
from tkinter import font as tkfont, ttk
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import app
import ui


class ResponsiveLayoutTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.directory = Path(self.folder.name)
        self.serial = self.enterContext(patch("serial.Serial", side_effect=AssertionError("no real hardware")))
        self.enterContext(patch.object(app.CalibratorApp, "_refresh_ports"))
        self.enterContext(patch.object(ui, "THEME_PATH", self.directory / "appearance.json"))
        self.root = tk.Tk()
        self.root.withdraw()
        self.errors = []
        self.root.report_callback_exception = lambda *error: self.errors.append(error)
        self.application = app.CalibratorApp(self.root, config_path=self.directory / "calibration.json")
        self.layout = self.application.responsive_ui
        self.root.update_idletasks()
        self.addCleanup(self.cleanup)

    def exists(self):
        try:
            return bool(self.root.winfo_exists())
        except tk.TclError:
            return False

    def cleanup(self):
        self.application.controller.send("shutdown")
        self.application.controller.join(2)
        self.assertFalse(self.application.controller.is_alive())
        for callback in self.root.tk.call("after", "info"):
            self.root.after_cancel(callback)
        if self.exists():
            self.root.destroy()
        self.folder.cleanup()
        self.serial.assert_not_called()
        self.assertEqual(self.errors, [])

    def settle(self, width, height):
        if self.layout.after_id is not None:
            self.root.after_cancel(self.layout.after_id)
            self.layout.after_id = None
        with patch.object(self.root, "winfo_width", return_value=width), \
                patch.object(self.root, "winfo_height", return_value=height):
            self.layout._settle()
            self.root.update_idletasks()

    def requested(self):
        return self.root.winfo_reqwidth(), self.root.winfo_reqheight()

    def font_size(self, specification):
        return tkfont.Font(root=self.root, font=specification).actual("size")

    def pump_until(self, condition, timeout=1):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.root.update()
            if condition():
                return
            time.sleep(.005)
        self.fail("UI condition did not complete within its deadline")

    def test_wide_round_trip_restores_exact_requested_size_and_control_state(self):
        self.settle(1360, 940)
        original_size = self.requested()
        profiles = copy.deepcopy(self.application.profiles)
        self.application.fields["name"].set("尚未应用的名称")
        self.application.target.set("7.5")
        with patch.object(self.application.controller, "send") as sent:
            for _ in range(3):
                self.settle(2560, 1360)
                self.assertTrue(self.layout.expanded)
                self.assertGreater(self.requested()[0], original_size[0])
                self.assertGreater(self.requested()[1], original_size[1])
                self.settle(1360, 940)
                self.assertFalse(self.layout.expanded)
                self.assertEqual(self.requested(), original_size)
            sent.assert_not_called()
        self.assertEqual(self.application.profiles, profiles)
        self.assertEqual(self.application.fields["name"].get(), "尚未应用的名称")
        self.assertEqual(self.application.target.get(), "7.5")

    def test_theme_keeps_wide_fonts_and_does_not_enlarge_external_dialog(self):
        dialog = tk.Toplevel(self.root)
        dialog.withdraw()
        button = ttk.Button(dialog, text="独立窗口", style="Quiet.TButton")
        button.pack()
        self.root.update_idletasks()
        dialog_size = dialog.winfo_reqwidth(), dialog.winfo_reqheight()
        base_size = self.font_size(ttk.Style(self.root).lookup("Quiet.TButton", "font"))
        self.settle(2560, 1360)
        main_size = self.font_size(ttk.Style(self.root).lookup(self.application.arm_button.cget("style"), "font"))
        self.assertGreater(main_size, base_size)
        for mode in ("light", "dark", "light"):
            ui.apply_theme(self.root, mode)
            self.root.update_idletasks()
            self.assertTrue(self.layout.expanded)
            self.assertEqual(self.font_size(ttk.Style(self.root).lookup(
                self.application.arm_button.cget("style"), "font")), main_size)
            self.assertEqual(self.font_size(ttk.Style(self.root).lookup(button.cget("style"), "font")), base_size)
            self.assertEqual((dialog.winfo_reqwidth(), dialog.winfo_reqheight()), dialog_size)
            self.assertFalse(getattr(button, "_workspace_member", False))

    def test_configure_events_are_coalesced_and_child_events_are_ignored(self):
        root_event = SimpleNamespace(widget=self.root)
        with patch.object(self.root, "winfo_width", return_value=2560), \
                patch.object(self.root, "winfo_height", return_value=1360), \
                patch.object(self.layout, "_settle", wraps=self.layout._settle) as settled:
            old_ids = []
            for _ in range(20):
                self.layout._resized(root_event)
                old_ids.append(self.layout.after_id)
            pending = self.layout.after_id
            self.layout._resized(SimpleNamespace(widget=self.application.target_entry))
            self.assertEqual(self.layout.after_id, pending)
            scheduled = self.root.tk.call("after", "info")
            self.assertIn(pending, scheduled)
            self.assertFalse(set(old_ids[:-1]).intersection(scheduled))
            self.pump_until(lambda: settled.call_count == 1)
            self.assertTrue(self.layout.expanded)
            self.assertEqual(settled.call_count, 1)

    def test_close_cancels_pending_resize_before_any_late_layout(self):
        self.layout._resized(SimpleNamespace(widget=self.root))
        pending = self.layout.after_id
        with patch.object(self.layout, "_style_density", wraps=self.layout._style_density) as density:
            self.application._close()
            self.layout._resized(SimpleNamespace(widget=self.root))
            self.pump_until(lambda: not self.exists())
            density.assert_not_called()
        self.assertIsNone(self.layout.after_id)
        self.assertEqual(self.layout.fonts, {})
        self.assertIsNone(self.layout.app)
        self.assertNotIn(pending, self.root.tk.call("after", "info"))


if __name__ == "__main__":
    unittest.main()
