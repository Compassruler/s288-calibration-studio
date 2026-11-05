"""Embedded preview stays read-only and distinguishes targets from feedback."""
import math
import gc
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import tkinter as tk
from tkinter import ttk
import unittest
import weakref
from unittest.mock import patch

import numpy as np
from PIL import ImageTk

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robot_view as view
from test_joint_binding import FakeRenderer, ReadOnlyApp
from test_robot_view import scene


class PreparedRenderer(FakeRenderer):
    compile_threads = []
    render_threads = []

    @staticmethod
    def compile(model):
        PreparedRenderer.compile_threads.append(threading.get_ident())
        return object()

    def __init__(self, model, prepared=None, size=None):
        PreparedRenderer.render_threads.append(threading.get_ident())
        super().__init__(model)


class EmbeddedPreviewTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.root = tk.Tk()
        self.root.withdraw()
        self.host = ttk.Frame(self.root)
        self.host.pack(fill="both", expand=True)
        self.app = ReadOnlyApp()
        self.app.root, self.app.selected, self.app.connected = self.root, 14, True
        self.app.demo = tk.BooleanVar(self.root, value=True)
        self.app.snapshots = {14: {"feedback": dict(raw_pos=.56, timestamp=time.monotonic())}}
        self.app.preview_target_raw = lambda: (14, .56+math.radians(20))
        model = scene()
        model["_fingerprint"] = "embedded-test"
        self.enterContext(patch.object(view, "load_scene", return_value=model))
        self.enterContext(patch.object(view, "_Renderer", PreparedRenderer))
        self.enterContext(patch.object(view, "BINDINGS_PATH", Path(self.directory.name)/"bindings.json"))
        self.errors = self.enterContext(patch.object(view.messagebox, "showerror"))
        self.viewer = view.RobotView(self.app, parent=self.host)
        self.addCleanup(self.cleanup)
        self.wait_ready()
        # The hidden test root needs an explicit, settled preview viewport.
        self.enterContext(patch.object(self.viewer.picture, "winfo_ismapped", return_value=True))
        self.enterContext(patch.object(self.viewer.picture, "winfo_width", return_value=240))
        self.enterContext(patch.object(self.viewer.picture, "winfo_height", return_value=160))
        self.tick()

    def cleanup(self):
        self.viewer.close()
        self.root.destroy()
        self.directory.cleanup()

    def wait_ready(self):
        deadline = time.monotonic()+2
        while self.viewer.renderer is None and time.monotonic() < deadline:
            self.root.update()
            time.sleep(.005)
        self.assertIsNotNone(self.viewer.renderer)

    def tick(self):
        self.viewer.window.after_cancel(self.viewer.after_id)
        self.viewer.after_id = None
        self.viewer._tick()

    def test_embedded_frame_and_background_compile_keep_gl_on_main_thread(self):
        self.assertIs(self.viewer.window.master, self.host)
        self.assertFalse(isinstance(self.viewer.window, tk.Toplevel))
        main = threading.get_ident()
        self.assertNotEqual(PreparedRenderer.compile_threads[-1], main)
        self.assertEqual(PreparedRenderer.render_threads[-1], main)
        self.assertEqual(self.viewer.view_mode.get(), "实际反馈")
        self.assertFalse(self.viewer.binding_panel.winfo_manager())
        self.viewer.show_bindings()
        self.assertEqual(self.viewer.binding_panel.winfo_manager(), "pack")
        self.viewer.toggle_bindings()
        self.assertFalse(self.viewer.binding_panel.winfo_manager())

    def test_first_frame_waits_for_mapping_and_valid_layout_without_caching_blank(self):
        foreground = np.zeros((160, 240, 3), dtype=np.uint8)
        foreground[50:100, 90:150] = (220, 30, 40)
        self.viewer._frame_key = None
        self.viewer.picture.configure(image="")
        with patch.object(self.viewer.renderer, "frame", return_value=foreground) as render, \
                patch.object(self.viewer.picture, "winfo_ismapped", return_value=False) as mapped, \
                patch.object(self.viewer.picture, "winfo_width", return_value=240) as width:
            self.tick()
            render.assert_not_called()
            self.assertIsNone(self.viewer._frame_key)
            self.assertIsNotNone(self.viewer.after_id)
            mapped.return_value = True
            width.return_value = 1
            self.tick()
            render.assert_not_called()
            self.assertIsNone(self.viewer._frame_key)
            width.return_value = 240
            self.tick()
            render.assert_called_once()
            self.assertIsNotNone(self.viewer._frame_key)
            shown = np.asarray(ImageTk.getimage(self.viewer.photo))
            np.testing.assert_array_equal(shown[75, 120, :3], foreground[75, 120])
            image_names = self.viewer.picture.tk.splitlist(self.viewer.picture.cget("image"))
            self.assertEqual(tuple(map(str, image_names)), (str(self.viewer.photo),))
            self.tick()
            render.assert_called_once()  # A settled static frame still reuses its cache.

    def test_target_preview_does_not_replace_actual_feedback_or_write_software_zero(self):
        self.viewer.view_mode.set("目标预览")
        self.viewer._bind()
        self.tick()
        self.assertEqual(self.viewer.angles["hip"], 0)
        self.assertAlmostEqual(self.viewer.display_angles["hip"], math.radians(20))
        self.assertIn("目标预览 +20.0°", self.viewer.status.get())
        self.assertIn("实际反馈 +0.0°", self.viewer.status.get())
        self.viewer.view_mode.set("实际反馈")
        self.tick()
        self.assertEqual(self.viewer.display_angles["hip"], 0)
        self.errors.assert_not_called()

    def test_compact_preview_status_wrap_does_not_resize_viewport(self):
        self.viewer.close()
        self.app.compact_robot_view = self.app.central_robot_view = True
        self.viewer = view.RobotView(self.app, parent=self.host)
        self.wait_ready()
        self.root.geometry("500x700")
        self.root.deiconify()
        self.root.update()
        viewport = self.viewer.picture.master
        original = (viewport.winfo_x(), viewport.winfo_y(), viewport.winfo_width(), viewport.winfo_height())
        footer_height = self.viewer.footer.winfo_height()
        for message in ("结构就绪", "目标预览 +20.0° · 实际反馈 +0.0° · 模拟示例绑定，与实机 ID 无关",
                        "反馈过期／异常，冻结：0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14"):
            self.viewer.status.set(message)
            self.root.update_idletasks()
            self.assertEqual((viewport.winfo_x(), viewport.winfo_y(), viewport.winfo_width(), viewport.winfo_height()), original)
            self.assertEqual(self.viewer.footer.winfo_height(), footer_height)

    def test_demo_defaults_link_missing_ids_without_saving_temporary_examples(self):
        self.assertEqual(set(self.viewer.bindings), {"14", "0"})
        self.assertEqual(self.viewer._demo_generated, {"14", "0"})
        self.assertFalse(self.viewer.bindings_path.exists())
        self.assertEqual(self.viewer.bindings["14"]["reference_raw"], .56)
        self.assertIn("模拟示例绑定", self.viewer.status.get())
        self.viewer._bind()
        document = json.loads(self.viewer.bindings_path.read_text(encoding="utf-8"))
        self.assertEqual(set(document["bindings"]), {"14"})

    def test_manual_binding_can_replace_another_temporary_demo_owner(self):
        self.viewer.joint.set(next(label for label,key in self.viewer.labels.items() if key == "knee"))
        self.viewer._bind()
        self.assertEqual(self.viewer.bindings["14"]["joint"], "knee")
        self.assertNotIn("0", self.viewer.bindings)
        self.assertNotIn("14", self.viewer._demo_generated)
        self.errors.assert_not_called()

    def test_releasing_or_losing_fresh_feedback_cancels_target_overlay(self):
        self.viewer.view_mode.set("目标预览")
        self.viewer._bind()
        self.tick()
        self.assertNotEqual(self.viewer.display_angles["hip"], 0)
        self.app.preview_target_raw = lambda: None
        self.tick()
        self.assertEqual(self.viewer.display_angles["hip"], 0)
        self.app.preview_target_raw = lambda: (14, 1.2)
        self.app.snapshots[14]["feedback"]["timestamp"] = time.monotonic()-2
        self.tick()
        self.assertEqual(self.viewer.display_angles["hip"], 0)
        self.assertIn("冻结", self.viewer.status.get())

    def test_target_preview_reference_note_follows_preview_motor_not_binding_editor(self):
        self.app.model_zero_references = lambda: {"14": .56, "0": .3}
        self.app.snapshots[14]["feedback"]["expos"] = .56
        self.app.snapshots[0] = dict(feedback=dict(expos=.3, timestamp=time.monotonic()))
        self.app.preview_target_expos = lambda: (14, .76)
        self.viewer.view_mode.set("目标预览")
        self.viewer.follow.set(False)
        self.viewer.motor.set("0")
        self.viewer._temporary_expos_references = {"14": .56}
        self.tick()
        self.assertIn("ID 14 · 目标预览", self.viewer.status.get())
        self.assertIn("临时模型参考", self.viewer.status.get())
        self.assertNotIn("软件零位参考", self.viewer.status.get())
        self.viewer._temporary_expos_references = {"0": .3}
        self.tick()
        self.assertIn("软件零位参考", self.viewer.status.get())
        self.assertNotIn("临时模型参考", self.viewer.status.get())

    def test_load_failure_still_detaches_renderer_when_native_cleanup_also_fails(self):
        with patch.object(view.RobotView, "_prepare"), \
                patch.object(view.RobotView, "_build", side_effect=ValueError("invalid preview")), \
                patch.object(PreparedRenderer, "close", side_effect=RuntimeError("native cleanup failed")):
            loading = view.RobotView(self.app, parent=self.host)
            try:
                loading.window.after_cancel(loading.after_id)
                model = scene()
                model["_fingerprint"] = "load-failure"
                loading._prepared.put((model, object(), None))
                loading._finish_prepare()
                self.assertIsNone(loading.renderer)
                self.assertIsNone(loading.after_id)
                messages = [str(widget.cget("text")) for widget in loading.window.winfo_children()
                            if isinstance(widget, ttk.Label)]
                self.assertTrue(any("invalid preview" in message for message in messages))
                self.assertTrue(any("native cleanup failed" in message for message in messages))
            finally:
                loading.close()

    def test_close_during_loading_does_not_initialize_gl_or_leave_tk_callback(self):
        gate = threading.Event()
        def prepare(model):
            gate.wait(1)
            return object()
        with patch.object(PreparedRenderer, "compile", side_effect=prepare):
            loading = view.RobotView(self.app, parent=self.host)
            pending = loading.after_id
            loading.show_bindings()
            loading.close()
            gate.set()
            self.root.update()
            self.assertIsNone(loading.renderer)
            self.assertTrue(loading.closed)
            self.assertNotIn(pending, self.root.tk.call("after", "info"))


    def test_background_loader_does_not_keep_a_closed_tk_view_alive(self):
        gate = threading.Event()

        def prepare(model):
            gate.wait(1)
            return object()

        with patch.object(PreparedRenderer, "compile", side_effect=prepare):
            loading = view.RobotView(self.app, parent=self.host)
            reference = weakref.ref(loading)
            loading.close()
            del loading
            try:
                gc.collect()
                self.assertIsNone(reference())
            finally:
                gate.set()

    def test_calibrated_jig_offset_is_rendered_but_footer_and_target_are_relative(self):
        offset = math.radians(79)
        self.viewer.scene["joints"][0]["zero_pose_offset_rad"] = offset
        self.app.model_zero_references = lambda: {"14": .56}
        self.app.snapshots[14]["feedback"]["expos"] = .56
        self.app.preview_target_expos = lambda: (14, .56+math.radians(20))
        self.tick()
        self.assertAlmostEqual(self.viewer.angles["hip"], offset)
        self.assertIn("相对支架角度 +0.0°", self.viewer.status.get())
        self.viewer.view_mode.set("目标预览")
        self.tick()
        self.assertAlmostEqual(self.viewer.display_angles["hip"], offset+math.radians(20))
        self.assertIn("目标预览 +20.0°", self.viewer.status.get())
        self.assertIn("实际反馈 +0.0°", self.viewer.status.get())
        pending, texts = [self.viewer.window], []
        while pending:
            widget = pending.pop()
            if "text" in widget.keys():
                texts.append(str(widget.cget("text")))
            pending.extend(widget.winfo_children())
        self.assertIn("视角复位", texts)

    def test_no_data_reference_pose_uses_offsets_and_stale_preserves_previous_pose(self):
        offsets = {"hip": math.radians(79), "knee": -math.radians(60)}
        for joint in self.viewer.scene["joints"]:
            joint["zero_pose_offset_rad"] = offsets[joint["key"]]
        self.viewer.bindings, self.viewer.angles, self.app.snapshots = {}, {}, {}
        self.tick()
        self.assertEqual(self.viewer.display_angles, offsets)
        expected = view.component_poses(self.viewer.scene, offsets)
        for part, pose in self.viewer.renderer.poses.items():
            np.testing.assert_allclose(pose, expected[part])
        self.viewer.bindings = {"14": dict(joint="hip", direction=1, reference_raw=None)}
        self.app.model_zero_references = lambda: {"14": .56}
        self.viewer.angles = {"hip": offsets["hip"]+.2}
        self.app.snapshots = {14: dict(feedback=dict(expos=.56, timestamp=time.monotonic()-2))}
        self.tick()
        self.assertAlmostEqual(self.viewer.display_angles["hip"], offsets["hip"]+.2)
        self.assertIn("冻结", self.viewer.status.get())

    def test_temporary_calibrated_reference_uses_same_jig_offset_without_persistence(self):
        offset = math.radians(79)
        self.viewer.scene["joints"][0]["zero_pose_offset_rad"] = offset
        self.app.model_zero_references = lambda: {"14": .56}
        self.app.snapshots[14]["feedback"]["expos"] = .76
        self.app.preview_target_expos = lambda: (14, .56)
        with patch.object(view.messagebox, "askyesno", return_value=True), \
                patch.object(self.viewer, "_persist", side_effect=AssertionError("temporary display cannot save")):
            self.viewer._capture()
            self.tick()
            self.assertAlmostEqual(self.viewer.angles["hip"], offset)
            self.viewer.view_mode.set("目标预览")
            self.tick()
            self.assertAlmostEqual(self.viewer.display_angles["hip"], offset-.2)
            self.assertIn("目标预览 -11.5°", self.viewer.status.get())
            self.viewer._capture()
            self.viewer.view_mode.set("实际反馈")
            self.tick()
            self.assertAlmostEqual(self.viewer.angles["hip"], offset+.2)

    def test_uncalibrated_legacy_reference_does_not_add_jig_offset(self):
        self.viewer.scene["joints"][0]["zero_pose_offset_rad"] = math.radians(79)
        self.viewer._bind()
        reference = self.viewer.bindings["14"]["reference_raw"]
        self.app.snapshots[14]["feedback"]["raw_pos"] = reference+.2
        self.tick()
        self.assertAlmostEqual(self.viewer.angles["hip"], .2)
        self.assertIn("相对角度 +11.5°", self.viewer.status.get())


if __name__ == "__main__":
    unittest.main()
