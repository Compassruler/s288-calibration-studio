"""Preview zero is checked against the independent native jig design targets."""
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import robot_view


class JigReferenceTests(unittest.TestCase):
    def setUp(self):
        self.scene = robot_view.load_scene(ROOT / "assets/robot/model.json")
        self.offsets = robot_view.reference_pose_angles(self.scene)

    def test_zero_feedback_matches_independent_fifteen_motor_design_poses(self):
        data = json.loads((ROOT / "tests/fixtures/jig_zero_targets.json").read_text(encoding="utf-8"))
        now = time.monotonic()
        bindings, snapshots, references = {}, {}, {}
        for mid, joint in enumerate(self.scene["joints"]):
            reference = (.2 + mid * .4) % math.tau
            bindings[str(mid)] = dict(joint=joint["key"], direction=-1 if mid % 2 else 1, reference_raw=None)
            references[str(mid)] = reference
            snapshots[mid] = {"feedback": dict(raw_pos=-400 + mid, model_raw_pos=999,
                expos=reference, timestamp=now, mode=1, fault=0, warning=0, timeout=0)}
        angles, stale = robot_view.feedback_angles(bindings, snapshots, now=now,
            zero_references=references, zero_offsets=self.offsets)
        self.assertEqual(stale, set())
        self.assertEqual(angles, self.offsets)
        poses = robot_view.component_poses(self.scene, angles)
        display = np.eye(4)
        display[:3, :3] = np.array(self.scene["display_rotation"]).reshape(3, 3)
        for name, expected in data["targets"].items():
            with self.subTest(component=name):
                actual = np.linalg.inv(display) @ poses[name]
                expected = np.array(expected)
                self.assertLess(float(np.max(np.abs(actual[:3, :3] - expected[:3, :3]))), 1e-4)
                self.assertLess(float(np.linalg.norm(actual[:3, 3] - expected[:3, 3])), 1e-5)

    def test_trunk_yaw_housings_stay_fixed_while_output_links_move(self):
        baseline = robot_view.component_poses(self.scene, self.offsets)
        for joint, motor, child in (("a_hip_yaw", 5, "foot_left-1"),
                                     ("b_hip_yaw", 12, "镜向foot_left-2")):
            angles = dict(self.offsets)
            angles[joint] += .3
            moved = robot_view.component_poses(self.scene, angles)
            np.testing.assert_allclose(moved[f"铝壳装配无刷版本-合并修改版-{motor}"],
                baseline[f"铝壳装配无刷版本-合并修改版-{motor}"], atol=1e-12)
            self.assertGreater(float(np.max(np.abs(moved[child] - baseline[child]))), .01)

    def test_verified_previous_bindings_load_without_rewriting_any_values(self):
        binding = {"0": dict(joint="a_hip_pitch", direction=-1, reference_raw=.71)}
        old_hash = self.scene["reference_pose"]["source_model_sha256"]
        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as folder:
            path = Path(folder) / "bindings.json"
            path.write_text(json.dumps(dict(version=1, simulation=False,
                model_sha256=old_hash, bindings=binding)), encoding="utf-8")
            before = path.read_bytes()
            loaded = robot_view.load_bindings(path, self.offsets.keys(), simulation=False,
                fingerprint=self.scene["_fingerprint"],
                compatible_fingerprints=self.scene["compatible_bindings_sha256"])
            self.assertEqual(loaded, binding)
            self.assertEqual(path.read_bytes(), before)

    def test_uncalibrated_independent_reference_does_not_gain_jig_offset(self):
        now = time.monotonic()
        bindings = {"0": dict(joint="a_hip_pitch", direction=-1, reference_raw=.4)}
        frames = {0: {"feedback": dict(raw_pos=.6, timestamp=now)}}
        angles, stale = robot_view.feedback_angles(bindings, frames, now=now, zero_offsets=self.offsets)
        self.assertEqual(stale, set())
        self.assertAlmostEqual(angles["a_hip_pitch"], -.2)
        self.assertGreater(abs(self.offsets["a_hip_pitch"]), 1)


if __name__ == "__main__":
    unittest.main()
