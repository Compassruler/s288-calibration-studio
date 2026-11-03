"""部署文件转换只使用合成配置和临时文件，不连接设备。"""

import copy
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import default_profiles, save_profiles
from deployment import (
    REFERENCE_JOINT_TO_MOTOR_ID, REFERENCE_POLICY_JOINT_ORDER,
    apply_layout, build_calibration, load_layout, write_calibration,
)
from protocol import RATIO, build_packet


def reference_layout():
    return {"joint_to_motor_id": dict(REFERENCE_JOINT_TO_MOTOR_ID),
            "policy_joint_order": list(REFERENCE_POLICY_JOINT_ORDER)}


def custom_layout():
    names = [f"joint_{mid}" for mid in range(15)]
    return {"joint_to_motor_id": dict(zip(names, range(15))),
            "policy_joint_order": list(reversed(names))}


def calibrated_profiles(custom=False):
    profiles = apply_layout(default_profiles(), custom_layout() if custom else reference_layout())
    for mid, profile in profiles.items():
        profile.update(zero_offset=20.0 + mid, reference_expos=0.1 + mid * 0.1,
                       verified=True, calibrated_at="2026-10-31T12:00:00+08:00",
                       direction=-1 if mid % 2 else 1)
    return profiles


def exported(profiles=None, **kwargs):
    options = {"simulation": False, "baudrate": 6000000, "confirmed_reference": True}
    options.update(kwargs)
    return build_calibration(calibrated_profiles() if profiles is None else profiles, **options)


class DeploymentLayoutTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.addCleanup(self.folder.cleanup)
        self.path = Path(self.folder.name) / "layout.json"

    def write_layout(self, document):
        self.path.write_text(json.dumps(document), encoding="utf-8")
        return self.path

    def test_loads_reference_without_importing_gains_or_home(self):
        layout = reference_layout()
        document = dict(layout, gain_profiles={"s288": {"kp": 999}}, stand_positions={"bad": 9})
        self.assertEqual(load_layout(self.write_layout(document)), layout)

    def test_custom_order_is_preserved_instead_of_sorted_by_id(self):
        layout = custom_layout()
        profiles = apply_layout(default_profiles(), load_layout(self.write_layout(layout)))
        self.assertEqual(profiles[14]["policy_index"], 1)
        self.assertEqual(profiles[0]["policy_index"], 15)

    def test_import_only_changes_mapping_fields_and_does_not_mutate_input(self):
        original = calibrated_profiles(custom=True)
        snapshot = copy.deepcopy(original)
        updated = apply_layout(original, reference_layout())
        self.assertEqual(original, snapshot)
        for mid in original:
            for key in original[mid]:
                if key not in ("deployment_joint", "policy_index"):
                    self.assertEqual(updated[mid][key], original[mid][key], (mid, key))
        self.assertEqual(updated[14]["deployment_joint"], "")
        self.assertIsNone(updated[14]["policy_index"])
        self.assertEqual(updated[14]["zero_offset"], 34.0)

    def test_rejects_duplicate_json_key(self):
        self.path.write_text('{"joint_to_motor_id": {}, "joint_to_motor_id": {}}', encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "重复字段"):
            load_layout(self.path)

    def test_rejects_incomplete_invalid_id_and_invalid_names(self):
        changes = [
            {"head_roll": True}, {"head_roll": 14}, {"head_roll": 1},
        ]
        for change in changes:
            with self.subTest(change=change):
                layout = reference_layout()
                layout["joint_to_motor_id"].update(change)
                with self.assertRaises(ValueError):
                    load_layout(self.write_layout(layout))
        layout = custom_layout()
        del layout["joint_to_motor_id"]["joint_14"]
        with self.assertRaises(ValueError):
            load_layout(self.write_layout(layout))
        for name in ("左膝", "left-knee", "2knee", "", "x y"):
            with self.subTest(name=name):
                layout = reference_layout()
                layout["joint_to_motor_id"][name] = layout["joint_to_motor_id"].pop("head_roll")
                with self.assertRaises(ValueError):
                    load_layout(self.write_layout(layout))

    def test_rejects_missing_duplicate_and_nonstring_policy_names(self):
        for order in (list(REFERENCE_POLICY_JOINT_ORDER[:-1]),
                      ["head_roll"] * 14, [{"not": "a name"}] * 14):
            with self.subTest(order=order):
                layout = reference_layout()
                layout["policy_joint_order"] = order
                with self.assertRaises(ValueError):
                    load_layout(self.write_layout(layout))


class DeploymentExportTests(unittest.TestCase):
    def test_uses_output_encoder_reference_and_keeps_control_direction(self):
        profiles = calibrated_profiles()
        document = exported(profiles)
        item = next(j for j in document["joints"] if j["motor_id"] == 7)
        self.assertAlmostEqual(item["encoder_home_rad"], profiles[7]["reference_expos"])
        self.assertNotEqual(item["encoder_home_rad"], profiles[7]["zero_offset"])
        self.assertEqual(item["direction"], -1)
        self.assertEqual(item["model_home_rad"], 0.0)
        self.assertEqual(document["reference_pose"], "mujoco_reset_qpos_zero")
        self.assertEqual(document["robot"], "s288")

    def test_encoder_upper_endpoint_is_wrapped_to_zero(self):
        profiles = calibrated_profiles()
        profiles[0]["reference_expos"] = math.tau
        self.assertEqual(exported(profiles)["joints"][0]["encoder_home_rad"], 0.0)

    def test_reference_export_records_excluded_id_without_discarding_profile(self):
        profiles = calibrated_profiles()
        snapshot = copy.deepcopy(profiles)
        document = exported(profiles)
        self.assertEqual(profiles, snapshot)
        self.assertEqual(document["deployment"]["layout_kind"], "reference_14")
        self.assertEqual(document["deployment"]["excluded_ids"], [14])
        self.assertFalse(document["deployment"]["requires_runtime_adaptation"])
        self.assertEqual(len(document["joints"]), 14)

    def test_custom_fifteen_keeps_id14_and_explicit_action_order(self):
        document = exported(calibrated_profiles(custom=True), baudrate=4000000)
        deployment = document["deployment"]
        self.assertEqual([j["motor_id"] for j in document["joints"]], list(range(15)))
        self.assertEqual(deployment["policy_joint_order"], custom_layout()["policy_joint_order"])
        self.assertEqual(deployment["joint_to_motor_id"]["joint_14"], 14)
        self.assertTrue(deployment["requires_runtime_adaptation"])
        self.assertEqual(deployment["layout_kind"], "custom_15")
        self.assertEqual(deployment["baudrate"], 4000000)
        self.assertEqual(deployment["excluded_ids"], [])

    def test_rejects_simulation_and_missing_reference_confirmation(self):
        for options in ({"simulation": True}, {"simulation": 0},
                        {"confirmed_reference": False}, {"confirmed_reference": 1}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                exported(**options)
        with self.assertRaises(ValueError):
            build_calibration(calibrated_profiles(), simulation=False, baudrate=6000000)

    def test_rejects_unverified_and_missing_measurements(self):
        for key, value in (("verified", False), ("reference_expos", None),
                           ("reference_expos", float("nan")), ("zero_offset", None)):
            with self.subTest(key=key):
                profiles = calibrated_profiles()
                profiles[7][key] = value
                with self.assertRaises(ValueError):
                    exported(profiles)

    def test_rejects_repeated_names_and_policy_indexes(self):
        for key, value in (("deployment_joint", "head_yaw"), ("policy_index", 1),
                           ("policy_index", None), ("policy_index", True)):
            with self.subTest(key=key, value=value):
                profiles = calibrated_profiles()
                profiles[0][key] = value
                with self.assertRaises(ValueError):
                    exported(profiles)

    def test_fourteen_requires_exact_reference_mapping_and_order(self):
        profiles = calibrated_profiles()
        profiles[0]["deployment_joint"], profiles[1]["deployment_joint"] = (
            profiles[1]["deployment_joint"], profiles[0]["deployment_joint"])
        with self.assertRaisesRegex(ValueError, "原例程"):
            exported(profiles)
        profiles = calibrated_profiles()
        profiles[0]["policy_index"], profiles[1]["policy_index"] = (
            profiles[1]["policy_index"], profiles[0]["policy_index"])
        with self.assertRaisesRegex(ValueError, "原例程"):
            exported(profiles)

    def test_partial_id14_mapping_cannot_be_silently_excluded(self):
        profiles = calibrated_profiles()
        profiles[14]["deployment_joint"] = "extra_neck"
        with self.assertRaises(ValueError):
            exported(profiles)
        profiles[14]["deployment_joint"] = ""
        profiles[14]["policy_index"] = 15
        with self.assertRaises(ValueError):
            exported(profiles)

    def test_rejects_bad_baudrate_and_profile_identity(self):
        for rate in (True, "6000000", 6000000.0, 3000000):
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                exported(baudrate=rate)
        profiles = calibrated_profiles()
        profiles[0]["id"] = 1
        with self.assertRaises(ValueError):
            exported(profiles)

    def test_gains_are_explicit_calibration_values_in_physical_and_sdk_units(self):
        profiles = calibrated_profiles()
        document = exported(profiles)
        gain = document["deployment"]["calibration_gains"]["head_roll"]
        self.assertEqual(gain["kp_output_nm_per_rad"], profiles[0]["kp"])
        self.assertEqual(gain["kd_output_nm_s_per_rad"], profiles[0]["kd"])
        self.assertAlmostEqual(gain["kp_sdk"], profiles[0]["kp"] / RATIO**2)
        self.assertAlmostEqual(gain["kd_sdk"], profiles[0]["kd"] / RATIO**2)
        packet = build_packet(0, kp=profiles[0]["kp"], kd=profiles[0]["kd"])
        raw_kp, raw_kd = struct.unpack_from("<hh", packet, 12)
        self.assertEqual(round(gain["kp_sdk"] * 1280000), raw_kp)
        self.assertEqual(round(gain["kd_sdk"] * 128000000), raw_kd)
        self.assertNotIn("gain_profiles", document["deployment"])
        self.assertNotIn("stand_positions", document["deployment"])
        self.assertEqual(document["deployment"]["target_speed_rad_s"], 0.0)
        self.assertEqual(document["deployment"]["feedforward_nm"], 0.0)


class DeploymentFileTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.addCleanup(self.folder.cleanup)
        self.path = Path(self.folder.name) / "s288_calibration.json"

    def test_write_and_replace_preserve_previous_export_as_backup(self):
        original = exported()
        write_calibration(self.path, original)
        self.assertEqual(json.loads(self.path.read_text(encoding="utf-8")), original)
        previous = self.path.read_bytes()
        changed = exported(calibrated_profiles(custom=True))
        write_calibration(self.path, changed)
        self.assertEqual(self.path.with_suffix(".json.bak").read_bytes(), previous)
        self.assertEqual(json.loads(self.path.read_text(encoding="utf-8")), changed)
        self.assertFalse(list(self.path.parent.glob("*.tmp")))

    def test_never_overwrites_software_calibration_or_training_layout(self):
        for kind in ("software", "layout", "invalid_json"):
            with self.subTest(kind=kind):
                if kind == "software":
                    save_profiles(self.path, default_profiles())
                elif kind == "layout":
                    self.path.write_text(json.dumps(reference_layout()), encoding="utf-8")
                else:
                    self.path.write_text("not JSON", encoding="utf-8")
                previous = self.path.read_bytes()
                with self.assertRaises(ValueError):
                    write_calibration(self.path, exported())
                self.assertEqual(self.path.read_bytes(), previous)
                self.assertFalse(list(self.path.parent.glob("*.tmp")))

    def test_failed_replace_does_not_damage_existing_export(self):
        write_calibration(self.path, exported())
        previous = self.path.read_bytes()
        import os
        replace = os.replace

        def fail_primary(source, target):
            if Path(target) == self.path:
                raise OSError("synthetic write failure")
            return replace(source, target)

        with patch("deployment.os.replace", side_effect=fail_primary), self.assertRaises(OSError):
            write_calibration(self.path, exported(calibrated_profiles(custom=True)))
        self.assertEqual(self.path.read_bytes(), previous)
        self.assertFalse(list(self.path.parent.glob("*.tmp")))

    def test_rejects_invalid_output_before_creating_file(self):
        for document in ({}, reference_layout(), {"robot": "s288", "joints": []}):
            with self.subTest(document=document), self.assertRaises(ValueError):
                write_calibration(self.path, document)
        self.assertFalse(self.path.exists())
        with self.assertRaises(ValueError):
            write_calibration(self.path.with_suffix(".bak"), exported())


if __name__ == "__main__":
    unittest.main()
