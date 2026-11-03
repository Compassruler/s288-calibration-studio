"""部署映射/导出走真实按钮流程；所有串口打开均被禁止。"""
import copy
import json
from pathlib import Path
import sys
import tempfile
import tkinter as tk
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import app
from deployment import REFERENCE_JOINT_TO_MOTOR_ID, REFERENCE_POLICY_JOINT_ORDER, apply_layout


class DeploymentProfileCompatibilityTests(unittest.TestCase):
    def test_old_zero_configuration_loads_without_changing_values(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            path = Path(directory) / "old.json"
            original = app.default_profiles()[14]
            original.pop("deployment_joint")
            original.pop("policy_index")
            original.update(zero_offset=1.2, reference_expos=.4, direction=-1, verified=True)
            path.write_text(json.dumps({"schema_version": 1, "units": "rad", "simulation": False,
                                        "joints": [original]}), encoding="utf-8")
            loaded = app.load_profiles(path, simulation=False)[14]
            self.assertEqual({key: loaded[key] for key in original}, original)
            self.assertEqual(loaded["deployment_joint"], "")
            self.assertIsNone(loaded["policy_index"])

    def test_deployment_mapping_survives_software_profile_save_and_load(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            path = Path(directory) / "profile.json"
            profiles = app.default_profiles()
            profiles[14].update(deployment_joint="neck_extra", policy_index=15)
            app.save_profiles(path, profiles)
            self.assertEqual(app.load_profiles(path), profiles)

    def test_invalid_deployment_names_or_noninteger_indexes_are_rejected(self):
        for updates in ({"deployment_joint": "left knee"}, {"policy_index": True},
                        {"policy_index": 1.5}, {"policy_index": 16}):
            profile = app.default_profiles()[0]
            profile.update(updates)
            with self.subTest(updates=updates), self.assertRaises(ValueError):
                app.validate_profile(profile)


class DeploymentUiTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.addCleanup(self.folder.cleanup)
        self.directory = Path(self.folder.name)
        self.serial = self.enterContext(patch("serial.Serial", side_effect=AssertionError("no hardware")))
        self.enterContext(patch.object(app.CalibratorApp, "_refresh_ports"))
        self.errors = self.enterContext(patch.object(app.messagebox, "showerror"))
        self.root = tk.Tk()
        self.root.withdraw()
        self.tool = app.CalibratorApp(self.root, config_path=self.directory / "source.json")
        self.addCleanup(self.cleanup)
        self.layout = {"joint_to_motor_id": REFERENCE_JOINT_TO_MOTOR_ID,
                       "policy_joint_order": list(REFERENCE_POLICY_JOINT_ORDER)}
        self.layout_path = self.directory / "layout.json"
        self.layout_path.write_text(json.dumps(self.layout), encoding="utf-8")
        self.output = self.directory / "export.json"

    def cleanup(self):
        self.tool.controller.send("shutdown")
        self.tool.controller.join(2)
        self.assertFalse(self.tool.controller.is_alive())
        for callback in self.root.tk.call("after", "info"):
            self.root.after_cancel(callback)
        self.root.destroy()
        self.serial.assert_not_called()

    def seed(self, count=14):
        for mid, profile in self.tool.profiles.items():
            profile.update(zero_offset=100 + mid, reference_expos=.2 + mid * .01, verified=True)
        if count == 14:
            self.tool.profiles = apply_layout(self.tool.profiles, self.layout)
        else:
            for mid, profile in self.tool.profiles.items():
                profile.update(deployment_joint=f"joint_{mid}", policy_index=15-mid)
        self.tool._show_profile()
        app.save_profiles(self.tool.config_path, self.tool.profiles)
        self.tool.dirty = False

    def test_import_preserves_zeros_controls_and_id14_and_sends_no_command(self):
        self.seed(15)
        before = copy.deepcopy(self.tool.profiles)
        with patch.object(app.filedialog, "askopenfilename", return_value=str(self.layout_path)), \
                patch.object(app.messagebox, "askyesno", return_value=True) as confirm, \
                patch.object(self.tool.controller, "send") as sent:
            self.tool.import_deployment_button.invoke()
            sent.assert_not_called()
        expected = apply_layout(before, self.layout)
        self.assertEqual(self.tool.profiles, expected)
        self.assertIn("ID 14", confirm.call_args.args[1])
        self.assertEqual(self.tool.profiles[14]["zero_offset"], 114)
        self.assertTrue(self.tool.dirty)
        self.errors.assert_not_called()

    def test_cancel_import_keeps_mapping_and_saved_file(self):
        self.seed(15)
        before, contents = copy.deepcopy(self.tool.profiles), self.tool.config_path.read_bytes()
        with patch.object(app.filedialog, "askopenfilename", return_value=str(self.layout_path)), \
                patch.object(app.messagebox, "askyesno", return_value=False):
            self.tool._import_deployment()
        self.assertEqual(self.tool.profiles, before)
        self.assertEqual(self.tool.config_path.read_bytes(), contents)

    def test_reference_export_uses_expos_and_explicitly_excludes_id14_without_motion(self):
        self.seed()
        before = self.tool.config_path.read_bytes()
        with patch.object(app.filedialog, "asksaveasfilename", return_value=str(self.output)), \
                patch.object(app.messagebox, "askyesno", return_value=True) as confirm, \
                patch.object(self.tool.controller, "send") as sent:
            self.tool.export_deployment_button.invoke()
            sent.assert_not_called()
        document = json.loads(self.output.read_text(encoding="utf-8"))
        self.assertEqual(len(document["joints"]), 14)
        self.assertEqual(document["joints"][0]["encoder_home_rad"], .2)
        self.assertIn("ID 14", confirm.call_args.args[1])
        self.assertIn("q=0", confirm.call_args.args[1])
        self.assertEqual(self.tool.config_path.read_bytes(), before)
        self.errors.assert_not_called()

    def test_custom15_and_4mbps_warn_about_runtime_requirements(self):
        self.seed(15)
        self.tool.baudrate.set("4000000")
        with patch.object(app.filedialog, "asksaveasfilename", return_value=str(self.output)), \
                patch.object(app.messagebox, "askyesno", return_value=True) as confirm:
            self.tool._export_deployment()
        document = json.loads(self.output.read_text(encoding="utf-8"))
        self.assertEqual(document["joints"][-1]["motor_id"], 14)
        self.assertEqual(document["deployment"]["policy_joint_order"][0], "joint_14")
        self.assertEqual(document["deployment"]["baudrate"], 4000000)
        self.assertIn("4 Mbps", confirm.call_args.args[1])
        self.assertIn("需要配套适配", self.tool.action_text.get())

    def test_simulation_incomplete_and_unverified_exports_never_open_save_dialog(self):
        self.seed()
        with patch.object(app.filedialog, "asksaveasfilename") as save, \
                patch.object(app.messagebox, "askyesno") as confirm:
            self.tool.demo.set(True)
            self.tool._render()
            self.assertTrue(self.tool.export_deployment_button.instate(["disabled"]))
            self.tool._export_deployment()
            self.tool.demo.set(False)
            self.tool.profiles[3]["verified"] = False
            self.tool._export_deployment()
            self.tool.profiles[3]["verified"] = True
            self.tool.profiles[3]["reference_expos"] = None
            self.tool._export_deployment()
            save.assert_not_called()
            confirm.assert_not_called()
        self.assertFalse(self.output.exists())

    def test_cancel_confirmation_and_source_file_overwrite_do_not_write(self):
        self.seed()
        before = self.tool.config_path.read_bytes()
        with patch.object(app.messagebox, "askyesno", return_value=False), \
                patch.object(app.filedialog, "asksaveasfilename") as save:
            self.tool._export_deployment()
            save.assert_not_called()
        with patch.object(app.messagebox, "askyesno", return_value=True), \
                patch.object(app.filedialog, "asksaveasfilename", return_value=str(self.tool.config_path)):
            self.tool._export_deployment()
        self.assertEqual(self.tool.config_path.read_bytes(), before)
        self.assertFalse(self.output.exists())

    def test_duplicate_mapping_is_rejected_before_commit(self):
        self.tool.profiles[0].update(deployment_joint="left_knee", policy_index=1)
        before = copy.deepcopy(self.tool.profiles)
        for field, value in (("deployment_joint", "left_knee"), ("policy_index", "1")):
            self.tool.fields["deployment_joint"].set("new_joint")
            self.tool.fields["policy_index"].set("")
            self.tool.fields[field].set(value)
            with patch.object(self.tool.controller, "send") as sent:
                self.assertFalse(self.tool._apply_profile())
                sent.assert_not_called()
            self.assertEqual(self.tool.profiles, before)

    def test_export_rechecks_control_state_after_confirmation(self):
        self.seed()
        def confirm(*args, **kwargs):
            self.tool._arm_pending = True
            return True
        with patch.object(app.messagebox, "askyesno", side_effect=confirm), \
                patch.object(app.filedialog, "asksaveasfilename", return_value=str(self.output)):
            self.tool._export_deployment()
        self.assertFalse(self.output.exists())
        self.tool._arm_pending = False

    def test_connection_passes_4mbps_and_locks_rate_while_connected(self):
        self.tool.baudrate.set("4000000")
        with patch.object(self.tool.controller, "send") as sent:
            self.tool._connect()
            self.assertEqual(sent.call_args.kwargs["baudrate"], 4000000)
        self.tool._render()
        self.assertTrue(self.tool.baudrate_box.instate(["disabled"]))
        self.tool.connecting = False
        self.tool._render()
        self.assertTrue(self.tool.baudrate_box.instate(["readonly"]))


if __name__ == "__main__":
    unittest.main()
