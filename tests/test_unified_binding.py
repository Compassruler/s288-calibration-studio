"""One main-panel apply joins control settings and read-only model association."""
import copy
from pathlib import Path
import sys
import tempfile
import tkinter as tk
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import app


class BindingView:
    def __init__(self, application):
        self.app = application
        self.ready = True
        self.bindings = {14: {"joint": "hip", "direction": 1, "reference_raw": .79}}
        self.saved = []
        self.previews = []
        self.failure = None

    def binding_options(self):
        return {"髋关节 · M1": "hip", "膝关节 · M2": "knee"} if self.ready else {}

    def get_binding(self, mid):
        return copy.deepcopy(self.bindings.get(mid))

    def set_binding(self, mid, key, direction=1):
        if self.failure:
            raise self.failure
        if key is None:
            self.bindings.pop(mid, None)
        else:
            self.bindings[mid] = dict(joint=key, direction=direction, reference_raw=.79)
        self.saved.append((mid, key, direction))
        self.app.root.after_idle(self.app._model_binding_changed)
        return True

    def preview_joint(self, mid, key):
        self.previews.append((mid, key))


class UnifiedBindingTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.enterContext(patch("serial.Serial", side_effect=AssertionError("no hardware")))
        self.enterContext(patch.object(app.CalibratorApp, "_refresh_ports"))
        self.errors = self.enterContext(patch.object(app.messagebox, "showerror"))
        self.root = tk.Tk()
        self.root.withdraw()
        self.application = app.CalibratorApp(self.root, demo=True,
                                            config_path=Path(self.folder.name) / "demo.json")
        self.viewer = BindingView(self.application)
        self.application.robot_view = self.viewer
        self.application._model_binding_changed()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.application.controller.send("shutdown")
        self.application.controller.join(2)
        for callback in self.root.tk.call("after", "info"):
            self.root.after_cancel(callback)
        self.root.destroy()
        self.folder.cleanup()

    def prepare_edit(self):
        fields = self.application.fields
        fields["direction"].set("−1")
        fields["model_direction"].set("−1")
        fields["model_joint"].set("膝关节 · M2")
        fields["name"].set("右膝")
        fields["kp"].set("0.5")

    def test_apply_updates_both_directions_and_binding_without_sending_motion(self):
        self.prepare_edit()
        with patch.object(self.application.controller, "send") as send:
            self.assertTrue(self.application._apply_profile())
            send.assert_not_called()
        self.assertEqual(self.application.profiles[14]["direction"], -1)
        self.assertEqual(self.application.profiles[14]["name"], "右膝")
        self.assertEqual(self.application.profiles[14]["kp"], .5)
        self.assertEqual(self.viewer.bindings[14]["joint"], "knee")
        self.assertEqual(self.viewer.bindings[14]["direction"], -1)
        self.assertEqual(self.viewer.saved, [(14, "knee", -1)])
        self.assertNotIn("model_joint", self.application.profiles[14])

    def test_binding_failure_leaves_control_profile_and_binding_unchanged(self):
        for failure in (ValueError("joint occupied"), OSError("cannot save")):
            with self.subTest(failure=failure):
                previous_profile = copy.deepcopy(self.application.profiles)
                previous_bindings = copy.deepcopy(self.viewer.bindings)
                self.prepare_edit()
                self.viewer.failure = failure
                self.assertFalse(self.application._apply_profile())
                self.assertEqual(self.application.profiles, previous_profile)
                self.assertEqual(self.viewer.bindings, previous_bindings)
                self.assertEqual(self.application.fields["name"].get(), "右膝")

    def test_profile_validation_fails_before_binding_write(self):
        self.prepare_edit()
        self.application.fields["kp"].set("invalid")
        previous = copy.deepcopy(self.viewer.bindings)
        self.assertFalse(self.application._apply_profile())
        self.assertEqual(self.viewer.bindings, previous)
        self.assertEqual(self.viewer.saved, [])

    def test_binding_callback_preserves_other_unapplied_fields(self):
        fields = self.application.fields
        fields["name"].set("unsaved name")
        fields["kp"].set("0.8")
        fields["notes"].set("unsaved notes")
        self.viewer.bindings[14]["direction"] = -1
        previous_previews = list(self.viewer.previews)
        self.application._model_binding_changed()
        self.assertEqual([fields[key].get() for key in ("name", "kp", "notes")],
                         ["unsaved name", "0.8", "unsaved notes"])
        self.assertEqual(fields["model_direction"].get(), "−1")
        self.assertEqual(self.viewer.previews, previous_previews)

    def test_no_binding_change_does_not_write_binding_file(self):
        self.application.fields["kp"].set("0.7")
        self.assertTrue(self.application._apply_profile())
        self.assertEqual(self.viewer.saved, [])

    def test_candidate_selection_only_previews_and_unbound_apply_removes_binding(self):
        self.application.fields["model_joint"].set("膝关节 · M2")
        with patch.object(self.application.controller, "send") as send:
            self.application._preview_model_joint()
            send.assert_not_called()
        self.assertEqual(self.viewer.previews[-1], (14, "knee"))
        self.assertEqual(self.viewer.bindings[14]["joint"], "hip")
        self.assertEqual(self.viewer.saved, [])
        self.application.fields["model_joint"].set("未绑定")
        self.assertTrue(self.application._apply_profile())
        self.assertNotIn(14, self.viewer.bindings)

    def test_not_ready_only_disables_model_controls(self):
        self.viewer.ready = False
        self.application._model_binding_changed()
        controls = dict(self.application.settings_widgets)
        for key in ("model_joint", "model_direction"):
            self.assertTrue(controls[key].instate(["disabled"]))
        self.assertFalse(controls["direction"].instate(["disabled"]))
        self.application.fields["kp"].set("0.4")
        self.assertTrue(self.application._apply_profile())
        self.assertEqual(self.viewer.saved, [])

    def test_main_id_change_previews_its_binding_instead_of_old_popup_selection(self):
        self.viewer.bindings[4] = dict(joint="knee", direction=-1, reference_raw=.2)
        self.application.selected = 4
        self.application._show_profile()
        self.assertEqual(self.application.fields["model_joint"].get(), "膝关节 · M2")
        self.assertEqual(self.viewer.previews[-1], (4, "knee"))

    def test_model_controls_lock_during_pending_enable(self):
        self.application._arm_pending = True
        self.application._render()
        controls = dict(self.application.settings_widgets)
        for key in ("model_joint", "model_direction", "direction"):
            self.assertTrue(controls[key].instate(["disabled"]))
        self.application._arm_pending = False
        self.application._render()
        self.assertFalse(controls["model_joint"].instate(["disabled"]))

    def test_new_controls_fit_existing_minimum_window(self):
        self.root.update_idletasks()
        width, height = self.root.minsize()
        self.assertLessEqual(self.root.winfo_reqwidth(), width)
        self.assertLessEqual(self.root.winfo_reqheight(), height)

    def test_long_joint_name_does_not_expand_the_main_workspace(self):
        name = "左腿髋关节的完整名称" * 12
        self.application.profiles[14]["name"] = name
        self.application._show_profile()
        self.root.update_idletasks()
        width, height = self.root.minsize()
        self.assertEqual(self.application.fields["name"].get(), name)
        self.assertLessEqual(self.root.winfo_reqwidth(), width)
        self.assertLessEqual(self.root.winfo_reqheight(), height)


if __name__ == "__main__":
    unittest.main()
