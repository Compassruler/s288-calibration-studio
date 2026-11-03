"""Button acknowledgement never duplicates motion or pretends it is complete."""
from pathlib import Path
import sys
import tempfile
import time
import tkinter as tk
from tkinter import ttk
import unittest
from unittest.mock import Mock, patch
import weakref

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import app
import ui


class FeedbackSink:
    def __init__(self):
        self.messages = []

    def show(self, message, tone="info"):
        self.messages.append((message, tone))


class ButtonFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.errors = []
        self.root.report_callback_exception = lambda *error: self.errors.append(error)
        self.sink = FeedbackSink()
        self.root._action_feedback = weakref.WeakMethod(self.sink.show)
        ui.apply_theme(self.root)
        self.enterContext(patch.object(ui, "BUTTON_FLASH_MS", 25))
        self.addCleanup(self.cleanup)

    def exists(self):
        try:
            return bool(self.root.winfo_exists())
        except tk.TclError:
            return False

    def cleanup(self):
        for callback in self.root.tk.call("after", "info"):
            self.root.after_cancel(callback)
        if self.exists():
            self.root.destroy()
        self.assertEqual(self.errors, [])

    def pump_until(self, condition, timeout=.5):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.root.update()
            if condition():
                return
            time.sleep(.005)
        self.fail("Button acknowledgement did not settle")

    def test_invoke_calls_original_once_and_preserves_result_without_claiming_success(self):
        action = Mock(return_value=True)
        button = ui.FeedbackButton(self.root, text="执行输入", command=action)
        result = button.invoke()
        action.assert_called_once_with()
        self.assertEqual(str(result), "1")
        self.assertEqual(self.sink.messages, [("已收到：执行输入。", "info")])
        self.assertTrue(button.instate(["alternate"]))

    def test_dynamic_caption_survives_the_end_of_flash(self):
        button = ui.FeedbackButton(self.root, text="半透明", style="Quiet.TButton")
        button._action = lambda: button.configure(text="恢复实体", style="Soft.TButton")
        button.invoke()
        self.assertTrue(button.instate(["alternate"]))
        self.pump_until(lambda: button._flash_id is None)
        self.assertFalse(button.instate(["alternate"]))
        self.assertEqual(button.cget("text"), "恢复实体")
        self.assertEqual(button.cget("style"), "Soft.TButton")

    def test_disabled_press_warns_but_never_executes(self):
        action = Mock()
        button = ui.FeedbackButton(self.root, text="使能", command=action, state="disabled")
        button.invoke()  # ttk itself refuses a disabled invocation.
        action.assert_not_called()
        self.assertEqual(self.sink.messages, [])
        button._disabled_press(None)
        self.assertEqual(self.sink.messages, [("当前不可用：使能。", "warning")])
        self.assertTrue(button.instate(["disabled", "invalid"]))
        self.pump_until(lambda: button._flash_id is None)
        self.assertTrue(button.instate(["disabled"]))
        self.assertFalse(button.instate(["invalid"]))

    def test_repeated_clicks_replace_flash_timer_and_destroy_cancels_it(self):
        action = Mock()
        button = ui.FeedbackButton(self.root, text="+0.1°", command=action)
        button.invoke()
        first = button._flash_id
        button.invoke()
        second = button._flash_id
        scheduled = self.root.tk.call("after", "info")
        self.assertNotIn(first, scheduled)
        self.assertIn(second, scheduled)
        self.assertEqual(action.call_count, 2)
        button.destroy()
        self.assertNotIn(second, self.root.tk.call("after", "info"))
        self.assertIsNone(button._flash_id)
        self.root.update()

    def test_command_may_hide_or_destroy_its_own_toplevel(self):
        for operation in ("withdraw", "destroy"):
            with self.subTest(operation=operation):
                window = tk.Toplevel(self.root)
                window.withdraw()
                button = ui.FeedbackButton(window, text="返回", command=getattr(window, operation))
                button.invoke()
                self.root.update_idletasks()
                if operation == "withdraw":
                    self.assertEqual(window.state(), "withdrawn")
                    window.destroy()
        self.assertEqual(self.errors, [])

    def test_command_may_destroy_root_without_a_late_flash_callback(self):
        button = ui.FeedbackButton(self.root, text="关闭", command=self.root.destroy)
        button.invoke()
        self.assertIsNone(button._flash_id)
        self.assertEqual(self.errors, [])


class AppActionFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.root = tk.Tk()
        self.root.withdraw()
        self.serial = self.enterContext(patch("serial.Serial", side_effect=AssertionError("real serial forbidden")))
        self.enterContext(patch.object(app.CalibratorApp, "_refresh_ports"))
        self.enterContext(patch.object(app.messagebox, "showerror"))
        self.application = app.CalibratorApp(self.root, config_path=Path(self.folder.name) / "unused.json")
        self.root.update_idletasks()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.application.controller.send("shutdown")
        self.application.controller.join(2)
        self.assertFalse(self.application.controller.is_alive())
        for callback in self.root.tk.call("after", "info"):
            self.root.after_cancel(callback)
        self.root.destroy()
        self.folder.cleanup()
        self.serial.assert_not_called()

    def test_return_binding_invokes_execute_once_and_submits_one_target(self):
        self.application.armed_id = 14
        self.application.target.set("5")
        self.application.execute_button.state(["!disabled"])
        # Call the real Return binding's Tcl callback without moving OS focus.
        script = self.application.target_entry.bind("<Return>")
        command = script.split("[", 1)[1].split(" ", 1)[0]
        with patch.object(self.application.execute_button, "invoke", wraps=self.application.execute_button.invoke) as invoke, \
                patch.object(self.application.controller, "send") as sent:
            self.root.tk.call(command, *(["0"] * len(self.root._subst_format)))
        invoke.assert_called_once_with()
        sent.assert_called_once_with("target", id=14, angle_deg=5.0)
        self.assertIn("目标已提交", self.application.action_text.get())
        self.assertIn("实际到位请看反馈", self.application.action_text.get())

    def test_async_operation_stays_pending_until_actual_event_and_error_is_explicit(self):
        with patch.object(self.application.controller, "send") as sent:
            self.application._send("arm", id=14)
        sent.assert_called_once_with("arm", id=14)
        self.assertEqual(self.application.action_text.get(), "正在使能当前舵机…")
        self.assertNotIn("已从当前位置接管", self.application.action_text.get())
        self.application._handle({"type": "armed", "id": 14, "raw_pos": .79})
        self.assertIn("已从当前位置接管", self.application.action_text.get())
        self.application._handle({"type": "error", "id": 14, "message": "未收到反馈"})
        self.assertEqual(self.application.action_text.get(), "操作未完成：未收到反馈")
        self.assertTrue(str(self.application.action_label.cget("style")).endswith(ui.ACTION_STYLES["error"]))

    def test_repeated_jogs_acknowledge_without_flooding_log(self):
        self.application.armed_id = 14
        button = self.application.jog_buttons[2]
        button.state(["!disabled"])
        before = self.application.log.get("1.0", "end")
        with patch.object(self.application.controller, "send") as sent:
            for _ in range(5):
                button.invoke()
        targets = [call for call in sent.call_args_list if call.args and call.args[0] == "target"]
        self.assertEqual(len(targets), 5)
        self.assertEqual(self.application.log.get("1.0", "end"), before)
        self.assertIn("目标已提交", self.application.action_text.get())


if __name__ == "__main__":
    unittest.main()
