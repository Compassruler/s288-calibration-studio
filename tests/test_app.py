"""Withdrawn Tk integration tests using virtual motors and stubbed dialogs."""

import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import time
import tkinter as tk
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import app


class AppIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.path = Path(self.folder.name) / "demo_calibration.json"
        self.serial = self.enterContext(patch("serial.Serial", side_effect=AssertionError("no real serial in GUI tests")))
        self.enterContext(patch.object(app.CalibratorApp, "_refresh_ports", return_value=None))
        self.errors = self.enterContext(patch.object(app.messagebox, "showerror", return_value=None))
        self.warnings = self.enterContext(patch.object(app.messagebox, "showwarning", return_value=None))
        self.enterContext(patch.object(app.messagebox, "askyesno", return_value=True))
        self.enterContext(patch.object(app.filedialog, "asksaveasfilename", return_value=str(self.path)))
        self.enterContext(patch.object(app.filedialog, "askopenfilename", return_value=str(self.path)))
        self.root = tk.Tk()
        self.root.withdraw()
        self.callback_errors = []
        self.root.report_callback_exception = lambda *error: self.callback_errors.append(error)
        self.app = app.CalibratorApp(self.root, demo=True, config_path=self.path)
        self.addCleanup(self.cleanup_app)

    def exists(self):
        try:
            return bool(self.root.winfo_exists())
        except tk.TclError:
            return False

    def pump_until(self, predicate, timeout=2):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.exists():
                self.root.update()
            if predicate():
                return
            time.sleep(.01)
        self.fail("GUI condition did not become true before its deadline")

    def connect_demo(self):
        self.pump_until(lambda: self.app.connected and 14 in self.app.snapshots)
        self.serial.assert_not_called()

    def pump_for(self, duration=.15):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if self.exists():
                self.root.update()
            time.sleep(.005)

    def arm_demo(self):
        self.connect_demo()
        self.app.arm_button.invoke()
        self.pump_until(lambda: self.app.armed_id == 14)

    @staticmethod
    def target_calls(spy):
        return [call for call in spy.call_args_list if call.args and call.args[0] == "target"]

    def cleanup_app(self):
        self.app.controller.send("shutdown")
        self.app.controller.join(2)
        self.assertFalse(self.app.controller.is_alive())
        if self.exists():
            for callback in self.root.tk.call("after", "info"):
                self.root.after_cancel(callback)
            self.root.update_idletasks()
            self.root.destroy()
        self.folder.cleanup()
        self.serial.assert_not_called()
        self.assertEqual(self.callback_errors, [], "Tk callback raised unexpectedly")

    def test_demo_capture_save_and_reload_simulation_profile(self):
        self.connect_demo()
        self.assertIsNone(self.app.armed_id)
        self.assertFalse(self.app.arm_button.instate(["disabled"]))
        self.pump_until(lambda: len(self.app.controller.history.get(14, [])) >= 25)
        self.app.capture_button.invoke()
        self.pump_until(lambda: self.app.profiles[14]["zero_offset"] is not None)
        zero = self.app.profiles[14]["zero_offset"]
        self.assertAlmostEqual(zero, self.app.controller.bus.positions[14])
        self.app._save()
        document = json.loads(self.path.read_text(encoding="utf-8"))
        self.assertTrue(document["simulation"])
        saved = next(item for item in document["joints"] if item["id"] == 14)
        self.assertAlmostEqual(saved["zero_offset"], zero)
        self.app.fields["name"].set("unsaved temporary name")
        self.app._apply_profile()
        self.assertTrue(self.app.dirty)
        self.app._load()
        self.assertEqual(self.app.profiles[14]["name"], saved["name"])
        self.assertFalse(self.app.dirty)
        self.assertIsNone(self.app.armed_id)
        self.errors.assert_not_called()

    def test_default_layout_fits_without_hiding_control_sections(self):
        self.connect_demo()
        self.root.update_idletasks()
        min_width, min_height = self.root.minsize()
        self.assertLessEqual(self.root.winfo_reqheight(), min_height)
        self.assertLessEqual(self.root.winfo_reqwidth(), min_width)

    def seed_saved_zeros(self):
        for motor_id in (3, 14):
            self.app.profiles[motor_id].update(
                name=f"关节 {motor_id}", zero_offset=.79 if motor_id == 14 else .3,
                reference_expos=1.2, calibrated_at="2026-10-28T12:00:00+08:00", verified=True,
                direction=-1, min_deg=-20., max_deg=25., kp=.5, kd=.03,
                rate_deg_s=8., direct_follow=False, notes="保留设置")
        self.app._show_profile()
        app.save_profiles(self.path, self.app.profiles, simulation=True)
        self.app.dirty = False
        return copy.deepcopy(self.app.profiles), self.path.read_bytes()

    def test_clear_current_zero_preserves_other_profiles_and_saves_backup(self):
        self.connect_demo()
        before, old_file = self.seed_saved_zeros()
        self.app.fields["name"].set("尚未应用的名称")
        with patch.object(self.app.controller, "send", wraps=self.app.controller.send) as sent:
            self.app.clear_zero_button.invoke()
            sent.assert_not_called()
        expected = copy.deepcopy(before)
        expected[14].update(zero_offset=None, reference_expos=None, calibrated_at=None, verified=False)
        self.assertEqual(self.app.profiles, expected)
        self.assertEqual(app.load_profiles(self.path, simulation=True), expected)
        self.assertEqual(Path(str(self.path)+".bak").read_bytes(), old_file)
        self.assertFalse(self.app.dirty)
        self.assertEqual(self.app.fields["name"].get(), "尚未应用的名称")
        self.assertIn("尚未标定", self.app.zero_text.get())
        self.errors.assert_not_called()

    def test_clear_all_zeros_includes_offline_ids_and_keeps_settings(self):
        self.app.ids.set("14")
        self.connect_demo()
        self.assertEqual(self.app.monitored, [14])
        before, old_file = self.seed_saved_zeros()
        with patch.object(self.app.controller, "send", wraps=self.app.controller.send) as sent:
            self.app.clear_all_zeros_button.invoke()
            sent.assert_not_called()
        expected = copy.deepcopy(before)
        for profile in expected.values():
            profile.update(zero_offset=None, reference_expos=None, calibrated_at=None, verified=False)
        self.assertEqual(self.app.profiles, expected)
        self.assertEqual(app.load_profiles(self.path, simulation=True), expected)
        self.assertEqual(Path(str(self.path)+".bak").read_bytes(), old_file)
        self.assertFalse(self.app.dirty)
        self.errors.assert_not_called()

    def test_clear_selected_zero_is_available_without_a_connection(self):
        self.app.ids.set("14")
        self.connect_demo()
        before, _ = self.seed_saved_zeros()
        self.app._disconnect()
        self.pump_until(lambda: not self.app.connected)
        self.app.tree.selection_set("3")
        self.pump_until(lambda: self.app.selected == 3)
        self.assertFalse(self.app.clear_zero_button.instate(["disabled"]))
        with patch.object(self.app.controller, "send", wraps=self.app.controller.send) as sent:
            self.app.clear_zero_button.invoke()
            sent.assert_not_called()
        expected = copy.deepcopy(before)
        expected[3].update(zero_offset=None, reference_expos=None, calibrated_at=None, verified=False)
        self.assertEqual(self.app.profiles, expected)
        self.assertEqual(app.load_profiles(self.path, simulation=True), expected)
        self.errors.assert_not_called()

    def test_cancel_clear_zero_does_not_change_configuration_or_send_commands(self):
        self.connect_demo()
        before, old_file = self.seed_saved_zeros()
        with patch.object(app.messagebox, "askyesno", return_value=False) as confirm, \
                patch.object(self.app.controller, "send", wraps=self.app.controller.send) as sent:
            self.app._clear_zero()
            self.app._clear_zero(all_joints=True)
            self.assertEqual(confirm.call_count, 2)
            sent.assert_not_called()
        self.assertEqual(self.app.profiles, before)
        self.assertEqual(self.path.read_bytes(), old_file)
        self.assertFalse(Path(str(self.path)+".bak").exists())
        self.assertFalse(self.app.dirty)
        self.errors.assert_not_called()

    def test_clear_zero_is_disabled_and_rejected_while_motor_is_enabled(self):
        self.connect_demo()
        before, old_file = self.seed_saved_zeros()
        self.app.arm_button.invoke()
        self.assertTrue(self.app._arm_pending)
        with patch.object(app.messagebox, "askyesno") as pending_confirm, \
                patch.object(app, "save_profiles") as pending_save:
            self.app._clear_zero()
            pending_confirm.assert_not_called()
            pending_save.assert_not_called()
        self.pump_until(lambda: self.app.armed_id == 14)
        self.assertTrue(self.app.clear_zero_button.instate(["disabled"]))
        self.assertTrue(self.app.clear_all_zeros_button.instate(["disabled"]))
        with patch.object(app.messagebox, "askyesno") as confirm, \
                patch.object(self.app.controller, "send", wraps=self.app.controller.send) as sent:
            self.app.clear_zero_button.invoke()
            self.app.clear_all_zeros_button.invoke()
            self.app._clear_zero()
            self.app._clear_zero(all_joints=True)
            confirm.assert_not_called()
            sent.assert_not_called()
        self.assertEqual(self.app.profiles, before)
        self.assertEqual(self.path.read_bytes(), old_file)

    def test_clear_zero_save_failure_keeps_original_profiles_and_dirty_state(self):
        self.connect_demo()
        before, old_file = self.seed_saved_zeros()
        self.app.dirty = True
        with patch.object(app, "save_profiles", side_effect=OSError("write failed")) as save, \
                patch.object(self.app.controller, "send", wraps=self.app.controller.send) as sent:
            self.app._clear_zero()
            save.assert_called_once()
            sent.assert_not_called()
        self.assertEqual(self.app.profiles, before)
        self.assertEqual(self.path.read_bytes(), old_file)
        self.assertFalse(Path(str(self.path)+".bak").exists())
        self.assertTrue(self.app.dirty)
        self.errors.assert_called_once()

    def test_pending_arm_and_batch_or_connection_states_prevent_clear(self):
        self.connect_demo()
        before, old_file = self.seed_saved_zeros()
        states = (("_arm_pending", True), ("homing", True), ("holding_ids", {3}),
                  ("_capture_save_id", 14), ("connecting", True), ("closing", True), ("faulted", True))
        with patch.object(app.messagebox, "askyesno") as confirm, \
                patch.object(self.app.controller, "send", wraps=self.app.controller.send) as sent:
            for key, value in states:
                original = getattr(self.app, key)
                try:
                    setattr(self.app, key, value)
                    self.app._render()
                    with self.subTest(state=key):
                        self.assertTrue(self.app.clear_zero_button.instate(["disabled"]))
                        self.assertTrue(self.app.clear_all_zeros_button.instate(["disabled"]))
                        self.app._clear_zero()
                        self.app._clear_zero(all_joints=True)
                finally:
                    setattr(self.app, key, original)
            confirm.assert_not_called()
            sent.assert_not_called()
        self.assertEqual(self.app.profiles, before)
        self.assertEqual(self.path.read_bytes(), old_file)

    def test_clear_rechecks_pending_arm_after_confirmation(self):
        self.connect_demo()
        before, old_file = self.seed_saved_zeros()
        def confirmed(*_args, **_kwargs):
            self.app._arm_pending = True
            return True
        try:
            with patch.object(app.messagebox, "askyesno", side_effect=confirmed), \
                    patch.object(self.app.controller, "send", wraps=self.app.controller.send) as sent, \
                    patch.object(app, "save_profiles") as save:
                self.app._clear_zero(all_joints=True)
                save.assert_not_called()
                sent.assert_not_called()
        finally:
            self.app._arm_pending = False
        self.assertEqual(self.app.profiles, before)
        self.assertEqual(self.path.read_bytes(), old_file)

    def test_all_fifteen_demo_ids_can_be_selected_enabled_and_moved(self):
        self.connect_demo()
        self.assertEqual(self.app.monitored, list(range(15)))
        for motor_id in range(15):
            self.app.tree.selection_set(str(motor_id))
            self.pump_until(lambda: self.app.selected == motor_id and
                            self.app.controller.selected == motor_id and self.app._can_slide() is False)
            self.pump_for(.06)
            self.app.arm_button.invoke()
            self.pump_until(lambda: self.app.armed_id == motor_id)
            origin = self.app.controller.bus.positions[motor_id]
            self.app._slide("5")
            self.pump_until(lambda: self.app.controller.bus.positions[motor_id] > origin + math.radians(1))
            self.app.release_button.invoke()
            self.pump_until(lambda: self.app.armed_id is None)
        self.errors.assert_not_called()
        self.serial.assert_not_called()

    def test_arm_jog_hold_release_updates_gui_and_controller(self):
        self.connect_demo()
        self.app.arm_button.invoke()
        self.pump_until(lambda: self.app.armed_id == 14)
        self.assertTrue(self.app.capture_button.instate(["disabled"]))
        self.app._jog(1)
        origin = self.app.snapshots[14]["origin"]
        self.pump_until(lambda: self.app.controller.armed is not None and
                        abs(self.app.controller.armed["requested_raw"] - origin) > math.radians(.9))
        self.app.hold_button.invoke()
        self.pump_until(lambda: abs(float(self.app.target.get()) - self.app.requested_deg) < .001
                        and abs(self.app.controller.armed["target_raw"] - self.app.controller.armed["requested_raw"]) < 1e-9)
        self.app.release_button.invoke()
        self.pump_until(lambda: self.app.armed_id is None and not self.app.capture_button.instate(["disabled"]))
        self.assertIsNone(self.app.controller.armed)
        self.assertEqual(self.app.controller.bus.history[-1]["kp"], 0)
        self.errors.assert_not_called()

    def test_selecting_another_joint_releases_previous_one_first(self):
        self.app.ids.set("13,14")
        self.connect_demo()
        self.pump_until(lambda: self.app.controller.selected == 14)
        self.app.arm_button.invoke()
        self.pump_until(lambda: self.app.armed_id == 14)
        bus = self.app.controller.bus
        before = len(bus.history)
        self.app.tree.selection_set("13")
        self.pump_until(lambda: self.app.selected == 13 and self.app.controller.selected == 13
                        and self.app.armed_id is None)
        self.assertIsNone(self.app.controller.armed)
        sent = list(bus.history)[before:]
        self.assertTrue(any(item["id"] == 14 and item["kp"] == 0 for item in sent))
        self.assertFalse(any(item["kp"] > 0 for item in sent))
        self.errors.assert_not_called()

    def test_pending_arm_keeps_selected_id_until_result_then_switches_safely(self):
        self.app.ids.set("13,14")
        self.connect_demo()
        self.pump_until(lambda: self.app.controller.selected == 14)
        entered, proceed = threading.Event(), threading.Event()
        arm = self.app.controller._arm

        def delayed_arm(*args, **kwargs):
            entered.set()
            if not proceed.wait(2):
                raise AssertionError("test did not release pending arm")
            return arm(*args, **kwargs)

        with patch.object(self.app.controller, "_arm", side_effect=delayed_arm):
            try:
                self.app.arm_button.invoke()
                self.pump_until(entered.is_set)
                self.app.tree.selection_set("13")
                self.pump_for(.06)
                self.assertTrue(self.app._arm_pending)
                self.assertEqual(self.app.selected, 14)
                self.assertEqual(self.app.tree.selection(), ("14",))
                self.assertEqual(self.app.controller.selected, 14)
            finally:
                proceed.set()
            self.pump_until(lambda: self.app.armed_id == 14)
        self.app.tree.selection_set("13")
        self.pump_until(lambda: self.app.selected == 13 and self.app.controller.selected == 13
                        and self.app.armed_id is None)
        self.assertIsNone(self.app.controller.armed)
        self.errors.assert_not_called()

    def test_pending_capture_keeps_the_current_selection(self):
        self.connect_demo()
        self.app._capture_save_id = 14
        try:
            with patch.object(self.app.controller, "send") as sent:
                self.app.tree.selection_set("13")
                self.pump_for(.06)
                self.assertEqual(self.app.selected, 14)
                self.assertEqual(self.app.tree.selection(), ("14",))
                sent.assert_not_called()
        finally:
            self.app._capture_save_id = None

    def test_release_cancels_queued_capture_and_unlocks_the_interface(self):
        self.connect_demo()
        entered, proceed = threading.Event(), threading.Event()
        poll = self.app.controller._poll_one

        def delayed_poll(*args, **kwargs):
            entered.set()
            if not proceed.wait(2):
                raise AssertionError("test did not resume polling")
            return poll(*args, **kwargs)

        with patch.object(self.app.controller, "_poll_one", side_effect=delayed_poll):
            try:
                self.pump_until(entered.is_set)
                self.app.capture_button.invoke()
                self.assertEqual(self.app._capture_save_id, 14)
                self.app.release_button.invoke()
            finally:
                proceed.set()
            self.pump_until(lambda: self.app._capture_save_id is None and not self.app._control_busy())
        self.assertIsNone(self.app.profiles[14]["zero_offset"])
        self.assertFalse(self.path.exists())
        self.assertFalse(self.app.capture_button.instate(["disabled"]))
        self.assertFalse(self.app.apply_button.instate(["disabled"]))
        self.errors.assert_not_called()

    def test_connection_can_be_cancelled_while_worker_is_connecting(self):
        entered, proceed = threading.Event(), threading.Event()
        connect = self.app.controller._connect

        def delayed_connect(*args, **kwargs):
            entered.set()
            if not proceed.wait(2):
                raise AssertionError("test did not release pending connection")
            return connect(*args, **kwargs)

        with patch.object(self.app.controller, "_connect", side_effect=delayed_connect):
            try:
                self.app._connect()
                self.pump_until(entered.is_set)
                self.pump_for(.25)  # Let the demo startup callback observe the pending connection.
                self.app._render()
                self.assertFalse(self.app.disconnect_button.instate(["disabled"]))
                self.assertEqual(self.app.disconnect_button.cget("text"), "取消连接")
                self.app.disconnect_button.invoke()
            finally:
                proceed.set()
            self.pump_until(lambda: not self.app.connecting and not self.app.connected
                            and self.app.controller.bus is None)
        self.assertTrue(self.app.disconnect_button.instate(["disabled"]))
        self.assertEqual(self.app.disconnect_button.cget("text"), "断开")
        self.serial.assert_not_called()

    def test_invalid_follow_mode_does_not_silently_change_control_settings(self):
        before = copy.deepcopy(self.app.profiles)
        self.app.fields["follow_mode"].set("未知模式")
        with patch.object(self.app.controller, "send") as sent:
            self.assertFalse(self.app._apply_profile())
            sent.assert_not_called()
        self.assertEqual(self.app.profiles, before)
        self.errors.assert_called_once()

    def test_fault_latches_ui_and_close_stops_worker(self):
        self.connect_demo()
        self.app.arm_button.invoke()
        self.pump_until(lambda: self.app.armed_id == 14)
        bus = self.app.controller.bus
        bus.fail_ids.add(14)
        self.pump_until(lambda: self.app.faulted and self.app.armed_id is None)
        self.assertTrue(self.app.arm_button.instate(["disabled"]))
        self.assertTrue(self.app.capture_button.instate(["disabled"]))
        self.assertIn("锁止", self.app.mode_text.get())
        self.app._close()
        self.pump_until(lambda: not self.app.controller.is_alive() and not self.exists())
        self.assertTrue(bus.closed)

    def test_disabled_slider_cannot_send_motion(self):
        self.connect_demo()
        self.assertTrue(self.app.angle_slider.instate(["disabled"]))
        with patch.object(self.app.controller, "send", wraps=self.app.controller.send) as sent:
            self.app._slide("5")
            self.pump_for()
            self.assertEqual(self.target_calls(sent), [])
        self.assertIsNone(self.app.controller.armed)

    def test_slider_burst_coalesces_into_only_the_latest_target(self):
        self.arm_demo()
        origin = self.app.snapshots[14]["origin"]
        with patch.object(self.app.controller, "send", wraps=self.app.controller.send) as sent:
            for angle in (-3, -2, 1.25, 3.5):
                self.app._slide(str(angle))
            self.assertEqual(self.target_calls(sent), [])
            self.pump_for(.18)
            targets = self.target_calls(sent)
            self.assertEqual(len(targets), 1)
            self.assertEqual(targets[0].kwargs["id"], 14)
            self.assertAlmostEqual(targets[0].kwargs["angle_deg"], 3.5)
        self.assertAlmostEqual(self.app.controller.armed["requested_raw"], origin + math.radians(3.5))
        self.assertAlmostEqual(float(self.app.target.get()), 3.5)
        self.assertAlmostEqual(self.app.slider_value.get(), 3.5)

    def test_ttk_scale_command_sends_latest_angle_through_real_widget_binding(self):
        self.arm_demo()
        origin = self.app.snapshots[14]["origin"]
        with patch.object(self.app.controller, "send", wraps=self.app.controller.send) as sent:
            self.app.angle_slider.set(-2.2)
            self.app.angle_slider.set(1.3)
            self.app.angle_slider.set(4.8)
            self.assertAlmostEqual(float(self.app.target.get()), 4.8)
            self.assertEqual(self.target_calls(sent), [])
            self.pump_for(.18)
            targets = self.target_calls(sent)
            self.assertEqual(len(targets), 1)
            self.assertEqual(targets[0].kwargs["id"], 14)
            self.assertAlmostEqual(targets[0].kwargs["angle_deg"], 4.8)
        self.assertAlmostEqual(self.app.controller.armed["requested_raw"], origin + math.radians(4.8))

    def test_release_cancels_a_slider_target_waiting_for_flush(self):
        self.arm_demo()
        with patch.object(self.app.controller, "send", wraps=self.app.controller.send) as sent:
            self.app._slide("9")
            self.app.release_button.invoke()
            self.pump_for(.2)
            self.assertEqual(self.target_calls(sent), [])
        self.assertIsNone(self.app.armed_id)
        self.assertIsNone(self.app.controller.armed)

    def test_joint_change_cancels_slider_target_for_the_previous_id(self):
        self.app.ids.set("13,14")
        self.arm_demo()
        with patch.object(self.app.controller, "send", wraps=self.app.controller.send) as sent:
            self.app._slide("9")
            self.app.tree.selection_set("13")
            self.pump_until(lambda: self.app.selected == 13 and self.app.controller.selected == 13
                            and self.app.armed_id is None)
            self.pump_for(.1)
            self.assertEqual(self.target_calls(sent), [])
        self.assertIsNone(self.app.controller.armed)

    def test_limits_and_programmatic_target_sync_do_not_send_motion(self):
        self.connect_demo()
        self.app.fields["min_deg"].set("-8")
        self.app.fields["max_deg"].set("12")
        self.assertTrue(self.app._apply_profile())
        self.assertAlmostEqual(float(self.app.angle_slider["from"]), -8)
        self.assertAlmostEqual(float(self.app.angle_slider["to"]), 12)
        self.app.arm_button.invoke()
        self.pump_until(lambda: self.app.armed_id == 14)
        original_target = self.app.controller.armed["requested_raw"]
        with patch.object(self.app.controller, "send", wraps=self.app.controller.send) as sent:
            self.app._sync_target(6.25)
            self.pump_for()
            self.assertAlmostEqual(self.app.slider_value.get(), 6.25)
            self.assertAlmostEqual(float(self.app.target.get()), 6.25)
            self.assertEqual(self.target_calls(sent), [])
        self.assertAlmostEqual(self.app.controller.armed["requested_raw"], original_target)

    def test_slider_flush_rechecks_fault_and_feedback_freshness(self):
        self.arm_demo()
        with patch.object(self.app.controller, "send", wraps=self.app.controller.send) as sent:
            # Freeze GUI snapshots only, keeping the deliberately injected UI
            # fault/stale states from being replaced by healthy simulated frames.
            with patch.object(self.app, "_handle", return_value=None):
                self.app._slide("8")
                self.app.faulted = True
                self.pump_for(.15)
                self.assertEqual(self.target_calls(sent), [])
                self.app.faulted = False
                self.app.snapshots[14]["feedback"]["timestamp"] = time.monotonic() - 1
                self.app._slide("7")
                self.pump_for(.15)
            self.assertEqual(self.target_calls(sent), [])

    def test_shared_gains_apply_to_all_profiles_save_and_survive_id_selection(self):
        self.app.fields["kp"].set("0.65")
        self.app.fields["kd"].set("0.04")
        with patch.object(self.app.controller, "send") as sent:
            self.app.apply_gains_button.invoke()
            sent.assert_not_called()
        loaded = app.load_profiles(self.path, simulation=True)
        self.assertTrue(all((p["kp"], p["kd"]) == (.65, .04) for p in loaded.values()))
        self.assertEqual(self.app.profiles, loaded)
        self.app.tree.selection_set("3")
        self.pump_until(lambda: self.app.selected == 3)
        self.assertEqual(self.app.fields["kp"].get(), "0.65")
        self.assertEqual(self.app.fields["kd"].get(), "0.04")
        self.errors.assert_not_called()

    def test_shared_gain_save_failure_preserves_existing_profiles_and_file(self):
        before, old_file = self.seed_saved_zeros()
        self.app.fields["kp"].set("0.8")
        self.app.fields["kd"].set("0.05")
        with patch.object(app, "save_profiles", side_effect=OSError("write failed")):
            self.app._apply_global_gains()
        self.assertEqual(self.app.profiles, before)
        self.assertEqual(self.path.read_bytes(), old_file)
        self.assertIn("write failed", str(self.errors.call_args))

    def test_shared_gain_invalid_pair_cannot_partially_change_any_profile(self):
        before, old_file = self.seed_saved_zeros()
        self.app.fields["kp"].set("0.6")
        self.app.fields["kd"].set("nan")
        self.app._apply_global_gains()
        self.assertEqual(self.app.profiles, before)
        self.assertEqual(self.path.read_bytes(), old_file)
        self.errors.assert_called_once()

    def test_shared_gain_controls_are_blocked_while_motor_is_enabled(self):
        self.arm_demo()
        self.assertTrue(self.app.apply_gains_button.instate(["disabled"]))
        previous = copy.deepcopy(self.app.profiles)
        self.app._apply_global_gains()
        self.assertEqual(self.app.profiles, previous)
        self.assertIn("释放", str(self.errors.call_args))

    def test_all_capture_saves_every_connected_zero_and_preserves_offline_profiles(self):
        self.app.ids.set("0,7,14")
        self.connect_demo()
        before, old_file = self.seed_saved_zeros()
        poses = {0: .12, 7: -.24, 14: .36}
        self.app.controller.bus.positions.update(poses)
        self.app.capture_all_button.invoke()
        self.assertEqual(self.app._capture_all_ids, (0, 7, 14))
        self.pump_until(lambda: self.app._capture_all_ids is None and self.path.exists())
        loaded = app.load_profiles(self.path, simulation=True)
        for mid, pose in poses.items():
            self.assertAlmostEqual(loaded[mid]["reference_expos"], pose % math.tau)
            expected_anchor = (before[mid]["zero_offset"] + app.wrap_pi(pose-before[mid]["reference_expos"])) if before[mid]["reference_expos"] is not None else pose
            self.assertAlmostEqual(loaded[mid]["zero_offset"], expected_anchor)
            self.assertFalse(loaded[mid]["verified"])
        self.assertEqual(loaded[3], before[3])
        self.assertEqual(Path(str(self.path)+".bak").read_bytes(), old_file)
        self.assertEqual(self.app.profiles, loaded)
        self.assertIsNone(self.app.controller.armed)
        self.assertFalse(self.app.dirty)
        self.errors.assert_not_called()

    def test_all_capture_failed_save_keeps_every_previous_zero(self):
        before, old_file = self.seed_saved_zeros()
        self.app._capture_all_ids = (3, 14)
        captures = [dict(id=mid, zero_offset=.2 + mid / 100, expos=.2,
                         calibrated_at="2026-11-04T19:00:00+08:00") for mid in (3, 14)]
        with patch.object(app, "save_profiles", side_effect=OSError("disk full")):
            self.app._handle(dict(type="captured_all", captures=captures))
        self.assertEqual(self.app.profiles, before)
        self.assertEqual(self.path.read_bytes(), old_file)
        self.assertIsNone(self.app._capture_all_ids)
        self.assertIn("disk full", str(self.errors.call_args))

    def test_incomplete_all_capture_and_cancel_keep_previous_zeros(self):
        before, old_file = self.seed_saved_zeros()
        self.app._capture_all_ids = (3, 14)
        self.app._handle(dict(type="captured_all", captures=[dict(id=3, zero_offset=.2,
            expos=.2, calibrated_at="2026-11-04T19:00:00+08:00")]))
        self.assertEqual(self.app.profiles, before)
        self.assertEqual(self.path.read_bytes(), old_file)
        self.assertIsNone(self.app._capture_all_ids)
        self.app._capture_all_ids = (3, 14)
        self.app._handle(dict(type="capture_cancelled", scope="all", message="记录已取消"))
        self.assertIsNone(self.app._capture_all_ids)
        self.assertEqual(self.app.profiles, before)


if __name__ == "__main__":
    unittest.main()
