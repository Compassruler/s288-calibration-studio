"""Configuration durability, identity and units checks in temporary folders."""

import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config


class ConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.addCleanup(self.folder.cleanup)
        self.path = Path(self.folder.name) / "joint_zeros.json"
        self.profiles = config.default_profiles()

    def write_document(self, **updates):
        document = {"schema_version": 1, "units": "rad", "simulation": False,
                    "joints": [self.profiles[14]]}
        document.update(updates)
        self.path.write_text(json.dumps(document), encoding="utf-8")

    def test_default_profiles_are_independent_and_not_calibrated(self):
        self.assertEqual(set(self.profiles), set(range(15)))
        self.profiles[0]["name"] = "left knee"
        self.assertNotEqual(self.profiles[0]["name"], self.profiles[1]["name"])
        for profile in self.profiles.values():
            self.assertIsNone(profile["zero_offset"])
            self.assertFalse(profile["verified"])

    def test_roundtrip_preserves_radians_direction_and_partial_set(self):
        profile = self.profiles[14]
        profile.update(name="左膝", zero_offset=-.82, direction=-1,
                       calibrated_at="2026-10-25T12:00:00+08:00",
                       reference_expos=2.1, verified=True)
        config.save_profiles(self.path, {14: profile})
        loaded = config.load_profiles(self.path, simulation=False)
        self.assertEqual(len(loaded), 15)
        self.assertEqual(loaded[14], profile)
        self.assertIsNone(loaded[0]["zero_offset"])
        doc = json.loads(self.path.read_text(encoding="utf-8"))
        self.assertEqual(doc["units"], "rad")
        self.assertEqual(doc["joints"][0]["zero_offset"], -.82)

    def test_save_preserves_last_good_version_and_load_backup(self):
        config.save_profiles(self.path, self.profiles)
        previous = self.path.read_bytes()
        self.profiles[14]["zero_offset"] = .2
        config.save_profiles(self.path, self.profiles)
        backup = Path(str(self.path) + ".bak")
        self.assertEqual(backup.read_bytes(), previous)
        self.assertIsNone(config.load_profiles(backup)[14]["zero_offset"])
        self.assertEqual(config.load_profiles(self.path)[14]["zero_offset"], .2)

    def test_invalid_save_cannot_damage_existing_file_or_backup(self):
        config.save_profiles(self.path, self.profiles)
        self.profiles[14]["zero_offset"] = .1
        config.save_profiles(self.path, self.profiles)
        previous = self.path.read_bytes()
        backup = Path(str(self.path) + ".bak").read_bytes()
        self.profiles[14]["zero_offset"] = math.nan
        with self.assertRaises(ValueError):
            config.save_profiles(self.path, self.profiles)
        self.assertEqual(self.path.read_bytes(), previous)
        self.assertEqual(Path(str(self.path) + ".bak").read_bytes(), backup)

    def test_replace_failure_preserves_original_and_cleans_temporary_files(self):
        config.save_profiles(self.path, self.profiles)
        previous = self.path.read_bytes()
        with patch.object(config.os, "replace", side_effect=OSError("simulated disk error")):
            with self.assertRaises(OSError):
                config.save_profiles(self.path, self.profiles)
        self.assertEqual(self.path.read_bytes(), previous)
        self.assertFalse(list(self.path.parent.glob("*.tmp")))

    def test_simulation_profiles_cannot_load_as_hardware_profiles(self):
        config.save_profiles(self.path, self.profiles, simulation=True)
        with self.assertRaises(ValueError):
            config.load_profiles(self.path, simulation=False)
        self.assertEqual(len(config.load_profiles(self.path, simulation=True)), 15)

    def test_ids_are_explicit_valid_and_unique(self):
        self.assertEqual(config.parse_ids("14"), [14])
        self.assertEqual(config.parse_ids("0-14"), list(range(15)))
        self.assertEqual(config.parse_ids("0,2 14,2"), [0, 2, 14])
        for value in ["15", "-1", "", "14-0", "NaN", "1.5", "0-15"]:
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    config.parse_ids(value)

    def test_duplicate_id_or_key_wrong_schema_and_units_rejected(self):
        for updates in [
            {"joints": [self.profiles[14], self.profiles[14]]},
            {"schema_version": 2}, {"schema_version": True}, {"units": "deg"},
            {"simulation": "false"}, {"joints": []},
        ]:
            with self.subTest(updates=updates):
                self.write_document(**updates)
                with self.assertRaises(ValueError):
                    config.load_profiles(self.path)
        self.path.write_text('{"schema_version":1,"schema_version":1}', encoding="utf-8")
        with self.assertRaises(ValueError):
            config.load_profiles(self.path)

    def test_nonfinite_out_of_range_and_boolean_numbers_rejected(self):
        invalid = [
            ("id", True), ("id", 15), ("direction", True), ("direction", 0),
            ("zero_offset", math.nan), ("zero_offset", 2000),
            ("kp", True), ("kp", 0), ("kp", 2.6),
            ("kd", -.1), ("kd", math.inf), ("rate_deg_s", 0),
            ("min_deg", -181), ("max_deg", -40), ("name", "  "),
            ("reference_expos", -1), ("verified", True),
        ]
        for field, value in invalid:
            with self.subTest(field=field, value=value):
                profile = copy.deepcopy(self.profiles[14])
                profile[field] = value
                with self.assertRaises(ValueError):
                    config.validate_profile(profile)

    def test_id_mapping_mismatch_cannot_be_saved(self):
        with self.assertRaises(ValueError):
            config.save_profiles(self.path, {0: self.profiles[14]})


if __name__ == "__main__":
    unittest.main()
