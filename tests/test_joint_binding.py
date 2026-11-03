"""Relative CAD linkage starts from fresh feedback and never drives hardware."""

import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import tkinter as tk
from tkinter import ttk
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robot_view as view
from test_robot_view import scene


class ReadOnlyApp:
    @property
    def controller(self):
        raise AssertionError("model bindings must never access motor control")

    @property
    def profiles(self):
        raise AssertionError("model bindings must never read or edit software zeros")


class FakeRenderer:
    def __init__(self, model):
        self.highlights = []
        self.poses = None
        self.closed = False
        self.camera = SimpleNamespace(azimuth=225., elevation=-20., distance=1., lookat=(0., 0., 0.))

    def frame(self, poses, highlight=None, transparent=False):
        self.poses = poses
        self.highlights.append(highlight)
        return np.zeros((20, 30, 3), dtype=np.uint8)

    def close(self):
        self.closed = True


class AutomaticBindingTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.path = Path(self.directory.name) / "model_bindings.json"
        self.root = tk.Tk()
        self.root.withdraw()
        self.app = ReadOnlyApp()
        self.app.root = self.root
        self.app.selected = 14
        self.app.connected = True
        self.app.demo = tk.BooleanVar(self.root, value=True)
        self.app.snapshots = {14: self.snapshot(.56)}
        model = scene()
        model["_fingerprint"] = "fake-model"
        real_toplevel = tk.Toplevel

        def withdrawn(master):
            window = real_toplevel(master)
            window.withdraw()
            return window

        self.enterContext(patch.object(view, "load_scene", return_value=model))
        self.enterContext(patch.object(view, "BINDINGS_PATH", self.path))
        self.enterContext(patch.object(view, "_Renderer", FakeRenderer))
        self.enterContext(patch.object(view.tk, "Toplevel", side_effect=withdrawn))
        self.errors = self.enterContext(patch.object(view.messagebox, "showerror"))
        self.enterContext(patch.object(view.messagebox, "showinfo"))
        self.enterContext(patch.object(view.messagebox, "askyesno", return_value=True))
        self.viewer = view.RobotView(self.app)
        self.addCleanup(self.cleanup)
        # Keep test windows hidden while giving the preview a settled viewport.
        self.enterContext(patch.object(self.viewer.picture, "winfo_ismapped", return_value=True))
        self.enterContext(patch.object(self.viewer.picture, "winfo_width", return_value=240))
        self.enterContext(patch.object(self.viewer.picture, "winfo_height", return_value=160))
        self.tick()

    def cleanup(self):
        self.viewer.close()
        self.root.update_idletasks()
        self.root.destroy()
        self.directory.cleanup()

    @staticmethod
    def snapshot(raw, **updates):
        feedback = dict(raw_pos=raw, timestamp=time.monotonic(), mode=1,
                        fault=0, warning=0, timeout=0)
        feedback.update(updates)
        return {"feedback": feedback, "faulted": False}

    def tick(self):
        if self.viewer.after_id is not None:
            self.viewer.window.after_cancel(self.viewer.after_id)
            self.viewer.after_id = None
        self.viewer._tick()

    def saved(self):
        filename = self.path.with_name("demo_model_bindings.json")
        return json.loads(filename.read_text(encoding="utf-8"))

    def test_binding_with_feedback_immediately_enables_parent_and_downstream_linkage(self):
        self.viewer._bind()  # First explicitly selected model joint is hip.
        self.assertAlmostEqual(self.viewer.bindings["14"]["reference_raw"], .56)
        self.app.snapshots[14] = self.snapshot(.76)
        self.tick()
        self.assertAlmostEqual(self.viewer.angles["hip"], .2)
        initial = view.component_poses(self.viewer.scene, {})
        current = self.viewer.renderer.poses
        self.assertFalse(np.allclose(current["upper"], initial["upper"]))
        self.assertFalse(np.allclose(current["lower"], initial["lower"]))
        np.testing.assert_allclose(current["fixed"], initial["fixed"])
        self.assertAlmostEqual(self.saved()["bindings"]["14"]["reference_raw"], .56)
        self.errors.assert_not_called()

    def test_static_model_does_not_redraw_but_motion_and_camera_still_do(self):
        self.viewer._bind()
        self.tick()
        count = len(self.viewer.renderer.highlights)
        for _ in range(10):
            self.tick()
        self.assertEqual(len(self.viewer.renderer.highlights), count)
        self.app.snapshots[14] = self.snapshot(.66)
        self.tick()
        self.assertEqual(len(self.viewer.renderer.highlights), count+1)
        self.viewer.renderer.camera.azimuth += 10
        self.tick()
        self.assertEqual(len(self.viewer.renderer.highlights), count+2)

    def test_offline_binding_initializes_once_when_first_good_feedback_arrives(self):
        self.app.connected = False
        self.app.snapshots = {}
        self.viewer._bind()
        self.assertIsNone(self.viewer.bindings["14"]["reference_raw"])
        self.app.snapshots[14] = self.snapshot(.7)
        self.tick()
        self.assertIsNone(self.viewer.bindings["14"]["reference_raw"])
        self.assertIn("ID 14 → 髋", self.viewer.status.get())
        self.assertIn("等待首次正常反馈", self.viewer.status.get())
        self.app.connected = True
        with patch.object(self.viewer, "_persist", wraps=self.viewer._persist) as persist:
            self.tick()
            self.assertEqual(persist.call_count, 1)
            self.assertAlmostEqual(self.viewer.bindings["14"]["reference_raw"], .7)
            self.app.snapshots[14] = self.snapshot(.9)
            self.tick()
            self.assertEqual(persist.call_count, 1)
        self.assertAlmostEqual(self.viewer.angles["hip"], .2)
        self.assertAlmostEqual(self.saved()["bindings"]["14"]["reference_raw"], .7)

    def test_abnormal_or_stale_feedback_does_not_initialize_reference(self):
        cases = [
            self.snapshot(.5, timestamp=time.monotonic() - 2),
            self.snapshot(.5, warning=1), self.snapshot(.5, fault=1),
            self.snapshot(.5, timeout=1), self.snapshot(.5, mode=0),
            self.snapshot(math.nan), {**self.snapshot(.5), "faulted": True},
        ]
        for snapshot in cases:
            with self.subTest(snapshot=snapshot):
                self.viewer.bindings.clear()
                self.app.snapshots = {14: snapshot}
                self.viewer._bind()
                self.tick()
                self.assertIsNone(self.viewer.bindings["14"]["reference_raw"])
                self.assertNotIn("hip", self.viewer.angles)

    def test_rebinding_same_joint_and_direction_keeps_existing_reference(self):
        self.viewer._bind()
        self.app.snapshots[14] = self.snapshot(.91)
        self.viewer._bind()
        self.assertAlmostEqual(self.viewer.bindings["14"]["reference_raw"], .56)
        self.tick()
        self.assertAlmostEqual(self.viewer.angles["hip"], .35)
        self.viewer.direction.set("−1")
        self.viewer._bind()
        self.assertAlmostEqual(self.viewer.bindings["14"]["reference_raw"], .56)
        self.tick()
        self.assertAlmostEqual(self.viewer.angles["hip"], -.35)
        self.assertAlmostEqual(self.saved()["bindings"]["14"]["reference_raw"], .56)

    def test_binding_table_updates_only_changes_and_selection_does_not_change_real_id(self):
        self.assertEqual(self.viewer.binding_table.item("hip", "values")[1], "未绑定")
        self.viewer._bind()
        self.assertEqual(self.viewer.binding_table.item("hip", "values")[1], "14")
        with patch.object(self.viewer.binding_table, "item", wraps=self.viewer.binding_table.item) as item:
            self.viewer._refresh_bindings()
            self.viewer._refresh_bindings()
            item.assert_not_called()
        self.viewer.binding_table.selection_set("knee")
        self.root.update()
        self.assertEqual(self.viewer.labels[self.viewer.joint.get()], "knee")
        self.assertEqual(self.app.selected, 14)
        self.assertEqual(self.viewer.motor.get(), "14")
        self.tick()
        self.assertEqual(self.viewer.renderer.highlights[-1], "lower")
        self.viewer._unbind()
        self.assertEqual(self.viewer.binding_table.item("hip", "values")[1], "未绑定")

    def test_bound_joint_cannot_be_stolen_by_another_motor(self):
        self.viewer._bind()
        original = copy.deepcopy(self.viewer.bindings)
        self.viewer.follow.set(False)
        self.viewer.motor.set("13")
        self.viewer._select_id()
        self.viewer._bind()
        self.errors.assert_called_once()
        self.assertEqual(self.viewer.bindings, original)

    def test_dropdown_and_table_choose_one_highlight_without_changing_id_or_binding(self):
        self.viewer._bind()
        original = copy.deepcopy(self.viewer.bindings)
        filename = self.path.with_name("demo_model_bindings.json")
        saved = filename.read_bytes()
        box = next(widget for row in self.viewer.binding_panel.winfo_children()
                   for widget in row.winfo_children()
                   if isinstance(widget, ttk.Combobox)
                   and str(widget.cget("textvariable")) == str(self.viewer.joint))
        for key, source in (("knee", "dropdown"), ("hip", "table"), ("knee", "table"), ("hip", "dropdown")):
            with self.subTest(key=key, source=source):
                if source == "dropdown":
                    self.viewer.joint.set(next(label for label, value in self.viewer.labels.items() if value == key))
                    box.event_generate("<<ComboboxSelected>>")
                else:
                    self.viewer.binding_table.selection_set(key)
                self.root.update()
                self.app.snapshots[14] = self.snapshot(.56)
                self.tick()
                self.assertEqual(self.viewer.binding_table.selection(), (key,))
                self.assertEqual(self.viewer.labels[self.viewer.joint.get()], key)
                self.assertEqual(self.viewer.renderer.highlights[-1], "upper" if key == "hip" else "lower")
                self.assertEqual(self.app.selected, 14)
                self.assertEqual(self.viewer.motor.get(), "14")
                self.assertEqual(self.viewer.bindings, original)
                self.assertEqual(filename.read_bytes(), saved)
        self.errors.assert_not_called()

    def test_main_panel_api_uses_passed_id_and_returns_copies(self):
        self.viewer._bind()
        self.app.snapshots[13] = self.snapshot(.7)
        self.app._model_binding_changed = Mock()
        self.assertTrue(self.viewer.set_binding(13, "knee", -1))
        self.assertEqual(self.app.selected, 14)
        self.assertEqual(self.saved()["bindings"]["13"],
                         {"joint": "knee", "direction": -1, "reference_raw": .7})
        self.assertEqual(self.viewer.get_binding(14)["joint"], "hip")
        binding = self.viewer.get_binding(13)
        binding["direction"] = 1
        self.assertEqual(self.viewer.get_binding(13)["direction"], -1)
        options = self.viewer.binding_options()
        options.clear()
        self.assertEqual(set(self.viewer.binding_options().values()), {"hip", "knee"})
        self.root.update()
        self.app._model_binding_changed.assert_called_once_with()

    def test_binding_api_write_failure_preserves_memory_file_and_candidate(self):
        self.viewer._bind()
        self.tick()
        self.viewer.bindings["13"] = {"joint": "knee", "direction": 1, "reference_raw": None}
        self.viewer._demo_generated.add("13")
        original = copy.deepcopy(self.viewer.bindings)
        angles, generated = dict(self.viewer.angles), set(self.viewer._demo_generated)
        selected = self.viewer.joint.get()
        filename = self.path.with_name("demo_model_bindings.json")
        saved = filename.read_bytes()
        with patch.object(view.os, "replace", side_effect=OSError("disk unavailable")):
            with self.assertRaises(OSError):
                self.viewer.set_binding(14, "knee", -1)
        self.assertEqual(self.viewer.bindings, original)
        self.assertEqual(self.viewer.angles, angles)
        self.assertEqual(self.viewer._demo_generated, generated)
        self.assertEqual(self.viewer.joint.get(), selected)
        self.assertEqual(filename.read_bytes(), saved)
        self.assertEqual(list(Path(self.directory.name).glob("*.tmp")), [])

    def test_candidate_preview_and_cancel_do_not_save_or_change_binding(self):
        self.viewer._bind()
        original = copy.deepcopy(self.viewer.bindings)
        filename = self.path.with_name("demo_model_bindings.json")
        saved = filename.read_bytes()
        self.viewer.preview_joint(14, "knee")
        self.root.update()
        self.tick()
        self.assertEqual(self.viewer.binding_table.selection(), ("knee",))
        self.assertEqual(self.viewer.renderer.highlights[-1], "lower")
        self.viewer.preview_joint(14, None)
        self.root.update()
        self.tick()
        self.assertEqual(self.viewer.binding_table.selection(), ())
        self.assertIsNone(self.viewer.renderer.highlights[-1])
        self.assertEqual(self.app.selected, 14)
        self.assertEqual(self.viewer.bindings, original)
        self.assertEqual(filename.read_bytes(), saved)

    def test_binding_api_rejects_occupied_or_invalid_parameters_without_changes(self):
        self.viewer._bind()
        original = copy.deepcopy(self.viewer.bindings)
        for args in ((13, "hip", 1), (15, "knee", 1), (14, "missing", 1),
                     (14, [], 1), (14, "hip", True), (14, "hip", 0)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.viewer.set_binding(*args)
            self.assertEqual(self.viewer.bindings, original)
        self.errors.assert_not_called()

    def test_binding_api_none_removes_only_passed_id(self):
        self.viewer._bind()
        self.app.snapshots[13] = self.snapshot(.7)
        self.viewer.set_binding(13, "knee", -1)
        kept = self.viewer.get_binding(13)
        self.viewer.set_binding(14, None)
        self.assertIsNone(self.viewer.get_binding(14))
        self.assertEqual(self.viewer.get_binding(13), kept)
        self.assertEqual(self.saved()["bindings"], {"13": kept})

    def test_model_ready_notifies_main_panel_after_options_exist(self):
        self.app._model_binding_changed = Mock()
        viewer = view.RobotView(self.app)
        try:
            self.root.update()
            self.app._model_binding_changed.assert_called_once_with()
            self.assertEqual(set(viewer.binding_options().values()), {"hip", "knee"})
        finally:
            viewer.close()


    def test_capture_save_failure_keeps_reference_pose_and_file_until_manual_retry(self):
        self.viewer._bind()
        self.tick()
        bindings = copy.deepcopy(self.viewer.bindings)
        angles = dict(self.viewer.angles)
        filename = self.path.with_name("demo_model_bindings.json")
        saved = filename.read_bytes()
        self.app.snapshots[14] = self.snapshot(.91)
        with patch.object(view.os, "replace", side_effect=OSError("disk unavailable")):
            self.viewer._capture()
        self.assertEqual(self.viewer.bindings, bindings)
        self.assertEqual(self.viewer.angles, angles)
        self.assertEqual(filename.read_bytes(), saved)
        self.errors.assert_called_once()
        self.viewer._capture()
        self.assertEqual(self.viewer.bindings["14"]["reference_raw"], .91)
        self.assertEqual(self.saved()["bindings"]["14"]["reference_raw"], .91)

    def test_initial_reference_save_failure_pauses_automatic_retries_and_keeps_none(self):
        self.app.connected, self.app.snapshots = False, {}
        self.viewer._bind()
        filename = self.path.with_name("demo_model_bindings.json")
        saved = filename.read_bytes()
        self.app.connected = True
        self.app.snapshots[14] = self.snapshot(.7)
        with patch.object(view.os, "replace", side_effect=OSError("disk unavailable")) as replace:
            self.viewer._initialize_references()
            self.viewer._initialize_references()
            self.assertEqual(replace.call_count, 1)
        self.assertIsNone(self.viewer.bindings["14"]["reference_raw"])
        self.assertEqual(filename.read_bytes(), saved)
        self.assertTrue(self.viewer._reference_save_failed)
        self.assertIn("模型参考未保存", self.viewer.notice)
        self.errors.assert_not_called()
        self.viewer.set_binding(14, "hip")
        self.assertFalse(self.viewer._reference_save_failed)
        self.assertEqual(self.saved()["bindings"]["14"]["reference_raw"], .7)

    def test_capture_rechecks_feedback_after_the_confirmation_dialog(self):
        self.viewer._bind()

        def moved_while_confirming(*args, **kwargs):
            self.app.snapshots[14] = self.snapshot(.91)
            return True

        with patch.object(view.messagebox, "askyesno", side_effect=moved_while_confirming):
            self.viewer._capture()
        self.assertEqual(self.viewer.bindings["14"]["reference_raw"], .91)

        def stale_while_confirming(*args, **kwargs):
            self.app.snapshots[14] = self.snapshot(.95, timestamp=time.monotonic()-2)
            return True

        with patch.object(view.messagebox, "askyesno", side_effect=stale_while_confirming):
            self.viewer._capture()
        self.assertEqual(self.viewer.bindings["14"]["reference_raw"], .91)
        self.assertEqual(self.saved()["bindings"]["14"]["reference_raw"], .91)

    def test_binding_notifications_are_coalesced_and_cancelled_on_close(self):
        self.app._model_binding_changed = Mock()
        for _ in range(3):
            self.viewer._notify_binding_changed()
        pending = self.viewer._notify_id
        self.assertIn(pending, self.root.tk.call("after", "info"))
        self.root.update()
        self.app._model_binding_changed.assert_called_once_with()
        self.viewer._notify_binding_changed()
        pending = self.viewer._notify_id
        self.viewer.close()
        self.assertNotIn(pending, self.root.tk.call("after", "info"))
        self.root.update()
        self.app._model_binding_changed.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
