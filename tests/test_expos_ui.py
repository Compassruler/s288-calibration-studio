"""UI output-encoder coordinates stay consistent across reboot, wrap and recapture."""
import copy
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
from config import default_profiles, load_profiles, save_profiles
import robot_view
from test_embedded_preview import PreparedRenderer
from test_real_id_control import WireSerial
from test_robot_view import scene
from zero_coordinates import wrap_pi


class ModelSnapshotTests(unittest.TestCase):
    def test_profile_refresh_replaces_derived_fields_and_preserves_sensor_snapshot(self):
        tool = app.CalibratorApp.__new__(app.CalibratorApp)
        tool.profiles = default_profiles()
        tool.profiles[14].update(zero_offset=12, reference_expos=.4)
        snapshot = dict(id=14, origin=-50, origin_expos=.1, armed=False, faulted=False,
                        feedback=dict(raw_pos=-49, expos=.6, timestamp=123,
                                      model_raw_pos=-999, model_reference_error="old error"))
        original = copy.deepcopy(snapshot)
        refreshed = tool._model_snapshot(14, snapshot)
        self.assertEqual(snapshot, original)
        self.assertAlmostEqual(refreshed["feedback"].pop("model_raw_pos"), 12.2)
        self.assertNotIn("model_reference_error", refreshed["feedback"])
        original["feedback"].pop("model_raw_pos")
        original["feedback"].pop("model_reference_error")
        self.assertEqual(refreshed, original)
        tool.profiles[14]["reference_expos"] = None
        missing_reference = tool._model_snapshot(14, snapshot)
        self.assertNotIn("model_raw_pos", missing_reference["feedback"])
        self.assertIn("ExPos", missing_reference["feedback"]["model_reference_error"])


class ExposUiTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        directory = Path(self.folder.name)
        self.path = directory / "calibration.json"
        self.binding_path = directory / "model_bindings.json"
        self.encoder_zero, self.anchor, self.model_reference = 6.26, 2.4, 2.45
        profiles = default_profiles()
        profiles[14].update(zero_offset=self.anchor, reference_expos=self.encoder_zero,
                            direction=-1, min_deg=-60, max_deg=60)
        profiles[0].update(zero_offset=-3, reference_expos=.3, min_deg=-60, max_deg=60)
        save_profiles(self.path, profiles, simulation=False)
        model = scene()
        model["_fingerprint"] = "expos-ui"
        self.binding_path.write_text(json.dumps(dict(version=1, simulation=False,
            model_sha256=model["_fingerprint"], bindings={"14": dict(joint="hip", direction=-1,
                reference_raw=self.model_reference)})), encoding="utf-8")
        self.saved, self.saved_binding = self.path.read_bytes(), self.binding_path.read_bytes()
        self.wire = None
        self.enterContext(patch("serial.Serial", side_effect=lambda *_a, **_k: self.wire))
        self.enterContext(patch("controller.SimBus", side_effect=AssertionError("real wire stub required")))
        self.enterContext(patch.object(app.CalibratorApp, "_refresh_ports"))
        self.errors = self.enterContext(patch.object(app.messagebox, "showerror"))
        self.enterContext(patch.object(robot_view, "load_scene", return_value=model))
        self.enterContext(patch.object(robot_view, "_Renderer", PreparedRenderer))
        self.enterContext(patch.object(robot_view, "BINDINGS_PATH", self.binding_path))
        self.root = tk.Tk()
        self.root.withdraw()
        self.callback_errors = []
        self.root.report_callback_exception = lambda *error: self.callback_errors.append(error)
        self.tool = app.CalibratorApp(self.root, config_path=self.path)
        self.tool.port.set("OFFLINE_EXPOS_UI")
        self.tool.ids.set("0,14")
        self.tool._ensure_robot_view()
        self.addCleanup(self.cleanup)
        self.wait(lambda: self.tool.robot_view.renderer is not None)
        picture = self.tool.robot_view.picture
        self.enterContext(patch.object(picture, "winfo_ismapped", return_value=True))
        self.enterContext(patch.object(picture, "winfo_width", return_value=240))
        self.enterContext(patch.object(picture, "winfo_height", return_value=160))

    def cleanup(self):
        self.tool.robot_view.close()
        self.tool.controller.send("shutdown")
        self.tool.controller.join(3)
        self.assertFalse(self.tool.controller.is_alive())
        for callback in self.root.tk.call("after", "info"):
            self.root.after_cancel(callback)
        self.root.destroy()
        self.folder.cleanup()
        self.assertEqual(self.callback_errors, [])

    def wait(self, condition, timeout=4):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.root.update()
            if condition():
                return
            time.sleep(.005)
        self.fail(f"ExPos UI condition not reached: {self.tool.mode_text.get()}")

    def connect_pose(self, raw, expos):
        if self.tool.connected:
            self.tool._disconnect()
            self.wait(lambda: not self.tool.connected)
        self.wire = WireSerial()
        self.wire.positions[14] = raw
        self.wire.output_offsets[14] = expos-raw
        self.tool._connect()
        self.wait(lambda: self.tool.connected and 14 in self.tool.snapshots and 0 in self.tool.snapshots)

    def tick_model(self):
        viewer = self.tool.robot_view
        if viewer.after_id:
            viewer.window.after_cancel(viewer.after_id)
        viewer.after_id = None
        viewer._tick()

    def assert_displays_agree(self, expected_angle):
        self.assertAlmostEqual(float(self.tool.position_text.get().removesuffix("°")), expected_angle, delta=.03)
        self.assertAlmostEqual(float(self.tool.tree.item("14", "values")[3]), expected_angle, delta=.03)
        self.assertAlmostEqual(self.tool.slider_value.get(), expected_angle, delta=.03)
        self.tick_model()
        expos = self.tool.snapshots[14]["feedback"]["expos"]
        profile = self.tool.profiles[14]
        expected_model = -wrap_pi(expos-profile["reference_expos"])
        self.assertAlmostEqual(self.tool.robot_view.angles["hip"], expected_model, places=7)

    def test_raw_reboots_and_released_output_wrap_keep_all_displays_consistent(self):
        for raw in (-40, 38, -3):
            self.connect_pose(raw, (self.encoder_zero+math.radians(3)) % math.tau)
            self.assert_displays_agree(-3)
            self.assertAlmostEqual(float(self.tool.values["raw"].get()), raw, places=5)
        # Move the output across its zero independently of the rotor counter.
        start = time.monotonic()
        with self.wire.lock:
            self.wire.positions[14] += math.radians(.2)
            self.wire.output_offsets[14] = self.encoder_zero-math.radians(4)-self.wire.positions[14]
        self.wait(lambda: self.tool.snapshots[14]["feedback"]["timestamp"] > start)
        self.assert_displays_agree(4)
        self.assertNotIn("基准未恢复", self.tool.mode_text.get())
        self.assertFalse(self.tool.faulted)
        self.assertTrue(all(packet["kp"] == packet["kd"] == 0 for packet in self.wire.sent()))
        self.assertEqual(self.path.read_bytes(), self.saved)
        self.assertEqual(self.binding_path.read_bytes(), self.saved_binding)

    def test_expos_target_preview_and_enable_hold_use_separate_coordinate_types(self):
        raw = -40
        self.connect_pose(raw, (self.encoder_zero+math.radians(3)) % math.tau)
        self.tool.arm_button.invoke()
        self.wait(lambda: self.tool.armed_id == 14)
        self.assertAlmostEqual(next(packet for packet in self.wire.sent() if packet["kp"])["target"], raw, places=5)
        self.tool._sync_target(8)
        self.tool.robot_view.view_mode.set("目标预览")
        self.assertAlmostEqual(self.tool.preview_target_expos()[1], (self.encoder_zero-math.radians(8)) % math.tau)
        self.tick_model()
        self.assertAlmostEqual(self.tool.robot_view.display_angles["hip"], math.radians(8))
        self.assertEqual(self.path.read_bytes(), self.saved)

    def test_single_and_all_recapture_unify_model_zero_after_raw_reset_and_reload(self):
        for all_joints, raw, degrees in ((False, -40, 10), (True, 35, 18)):
            with self.subTest(all_joints=all_joints):
                self.connect_pose(raw, (self.encoder_zero+math.radians(degrees)) % math.tau)
                self.tick_model()
                self.assertGreater(abs(self.tool.robot_view.angles["hip"]), math.radians(1))
                with patch.object(robot_view.messagebox, "askyesno", return_value=True):
                    self.tool.robot_view._capture()
                self.assertIn("14", self.tool.robot_view._temporary_expos_references)
                before = copy.deepcopy(self.tool.profiles[14])
                self.wait(lambda: len(self.tool.controller.history.get(14, [])) >= 25)
                button = self.tool.capture_all_button if all_joints else self.tool.capture_button
                button.invoke()
                self.wait(lambda: self.tool._capture_save_id is None and self.tool._capture_all_ids is None
                          and self.tool.profiles[14]["reference_expos"] != before["reference_expos"])
                after = load_profiles(self.path)[14]
                self.assertAlmostEqual(after["zero_offset"], before["zero_offset"]+wrap_pi(after["reference_expos"]-before["reference_expos"]))
                self.tick_model()
                self.assertNotIn("14", self.tool.robot_view._temporary_expos_references)
                self.assertAlmostEqual(self.tool.robot_view.angles["hip"], 0, places=6)
                saved = self.path.read_bytes()
                self.tool._load_file(self.path)
                self.connect_pose(raw+11, after["reference_expos"])
                self.assert_displays_agree(0)
                self.assertAlmostEqual(self.tool.robot_view.angles["hip"], 0, delta=math.radians(.03))
                self.assertEqual(self.path.read_bytes(), saved)
                self.assertEqual(self.binding_path.read_bytes(), self.saved_binding)
        self.errors.assert_not_called()

    def test_missing_expos_requires_recording_but_direction_change_preserves_saved_values(self):
        self.connect_pose(-40, (self.encoder_zero+math.radians(3)) % math.tau)
        original = copy.deepcopy(self.tool.profiles[14])
        self.tool.fields["direction"].set("+1")
        self.assertTrue(self.tool._apply_profile())
        self.assertEqual(self.tool.profiles[14]["zero_offset"], original["zero_offset"])
        self.assertEqual(self.tool.profiles[14]["reference_expos"], original["reference_expos"])
        self.assert_displays_agree(3)
        self.tool.profiles[14]["reference_expos"] = None
        self.tool._show_profile()
        self.assertEqual(self.tool.position_text.get(), "—")
        self.assertIn("缺少 ExPos", self.tool.zero_text.get())
        self.assertTrue(self.tool.arm_button.instate(["disabled"]))
        self.tool._arm()
        self.assertTrue(all(packet["kp"] == packet["kd"] == 0 for packet in self.wire.sent()))
        self.errors.assert_called_once()

    def test_calibrated_none_binding_reference_does_not_initialize_or_write(self):
        self.connect_pose(-40, self.encoder_zero)
        viewer = self.tool.robot_view
        viewer.bindings["14"]["reference_raw"] = None
        with patch.object(viewer, "_persist", side_effect=AssertionError("display must not save binding")):
            self.tick_model()
        self.assertAlmostEqual(viewer.angles["hip"], 0, delta=math.radians(.03))
        self.assertIsNone(viewer.bindings["14"]["reference_raw"])
        self.assertEqual(self.path.read_bytes(), self.saved)
        self.assertEqual(self.binding_path.read_bytes(), self.saved_binding)

    def test_temporary_reference_is_visible_reversible_and_display_only_in_both_modes(self):
        self.connect_pose(-40, (self.encoder_zero+math.radians(3)) % math.tau)
        viewer = self.tool.robot_view
        self.assertEqual(viewer.view_mode.get(), "实际反馈")
        profiles = copy.deepcopy(self.tool.profiles)
        with patch.object(robot_view.messagebox, "askyesno", return_value=True), \
                patch.object(self.tool.controller, "send") as send:
            viewer._capture()
            send.assert_not_called()
        self.tick_model()
        self.assertAlmostEqual(viewer.angles["hip"], 0)
        self.assertIn("临时模型参考", viewer.status.get())
        self.assertEqual(viewer.reference_button.cget("text"), "恢复软件零位显示")
        self.assertAlmostEqual(float(self.tool.position_text.get().removesuffix("°")), -3, delta=.03)
        self.tool.arm_button.invoke()
        self.wait(lambda: self.tool.armed_id == 14)
        viewer.view_mode.set("目标预览")
        self.tool._sync_target(8)
        self.tick_model()
        self.assertAlmostEqual(viewer.display_angles["hip"], math.radians(11), delta=math.radians(.03))
        self.assertAlmostEqual(viewer.angles["hip"], 0)
        self.tool.release_button.invoke()
        self.wait(lambda: self.tool.armed_id is None)
        self.assertIn("14", viewer._temporary_expos_references, "release must not clear the display override")
        viewer._capture()
        self.tick_model()
        self.assertNotIn("14", viewer._temporary_expos_references)
        self.assertAlmostEqual(viewer.angles["hip"], -math.radians(3), delta=math.radians(.03))
        self.assertEqual(viewer.reference_button.cget("text"), "临时对齐模型参考")
        self.assertEqual(self.tool.profiles, profiles)
        self.assertEqual(self.path.read_bytes(), self.saved)
        self.assertEqual(self.binding_path.read_bytes(), self.saved_binding)

    def test_load_and_reconnect_clear_temporary_reference_without_changing_saved_files(self):
        self.connect_pose(-40, (self.encoder_zero+math.radians(3)) % math.tau)
        viewer = self.tool.robot_view
        for operation in (lambda: self.tool._load_file(self.path),
                          lambda: self.connect_pose(31, (self.encoder_zero+math.radians(3)) % math.tau)):
            with patch.object(robot_view.messagebox, "askyesno", return_value=True):
                viewer._capture()
            self.assertIn("14", viewer._temporary_expos_references)
            operation()
            self.assertNotIn("14", viewer._temporary_expos_references)
            self.tick_model()
            self.assertAlmostEqual(viewer.angles["hip"], -math.radians(3), delta=math.radians(.03))
        self.assertEqual(self.path.read_bytes(), self.saved)
        self.assertEqual(self.binding_path.read_bytes(), self.saved_binding)

    def test_home_with_2_68_degree_residual_still_draws_actual_residual_and_target_preview_zero(self):
        self.connect_pose(-40, (self.encoder_zero+math.radians(3)) % math.tau)
        viewer, original = self.tool.robot_view, self.wire.write
        introduced = False

        def residual_output(packet):
            nonlocal introduced
            if (packet[2] & 15) == 14 and int.from_bytes(packet[12:14], "little", signed=True) > 0 and not introduced:
                self.wire.output_offsets[14] += math.radians(2.68)
                introduced = True
            return original(packet)

        with patch.object(self.wire, "write", side_effect=residual_output):
            self.tool.current_home_button.invoke()
            self.wait(lambda: not self.tool.homing and self.tool.holding_ids == {14})
            self.assertIn("回零完成", self.tool.home_text.get())
            self.assertAlmostEqual(float(self.tool.position_text.get().removesuffix("°")), -2.68, delta=.03)
            self.tick_model()
            self.assertEqual(viewer.view_mode.get(), "实际反馈")
            self.assertAlmostEqual(viewer.display_angles["hip"], -math.radians(2.68), delta=math.radians(.03))
            self.tool._cancel_home()
            self.wait(lambda: not self.tool.holding_ids and self.tool.controller.armed is None)
            self.tool.arm_button.invoke()
            self.wait(lambda: self.tool.armed_id == 14)
            self.tool._sync_target(0)
            viewer.view_mode.set("目标预览")
            self.tick_model()
            self.assertAlmostEqual(viewer.display_angles["hip"], 0)
            self.assertAlmostEqual(viewer.angles["hip"], -math.radians(2.68), delta=math.radians(.03))
            viewer.view_mode.set("实际反馈")
            self.tick_model()
            self.assertAlmostEqual(viewer.display_angles["hip"], viewer.angles["hip"])
        self.assertEqual(self.path.read_bytes(), self.saved)
        self.assertEqual(self.binding_path.read_bytes(), self.saved_binding)

    def test_loading_missing_output_reference_refreshes_model_cache_before_any_new_feedback(self):
        self.connect_pose(-40, (self.encoder_zero+math.radians(3)) % math.tau)
        self.tool.robot_view.bindings["14"]["reference_raw"] = None
        self.tick_model()
        pose = self.tool.robot_view.angles["hip"]
        original = copy.deepcopy(self.tool.snapshots[14]["feedback"])
        missing = copy.deepcopy(self.tool.profiles)
        missing[14]["reference_expos"] = None
        path = self.path.with_name("missing_expos.json")
        save_profiles(path, missing, simulation=False)
        self.tool._load_file(path)
        snapshot = self.tool.snapshots[14]
        self.assertNotIn("model_raw_pos", snapshot["feedback"])
        self.assertIn("model_reference_error", snapshot["feedback"])
        for key in ("raw_pos", "expos", "timestamp"):
            self.assertEqual(snapshot["feedback"][key], original[key])
        self.tick_model()
        self.assertEqual(self.tool.robot_view.angles["hip"], pose)
        self.assertIn("冻结", self.tool.robot_view.status.get())
        self.tool._load_file(self.path)
        self.assertNotIn("model_reference_error", self.tool.snapshots[14]["feedback"])
        self.tick_model()
        self.assertAlmostEqual(self.tool.robot_view.angles["hip"], pose)
        self.assertEqual(self.path.read_bytes(), self.saved)
        self.assertEqual(self.binding_path.read_bytes(), self.saved_binding)


if __name__ == "__main__":
    unittest.main()
