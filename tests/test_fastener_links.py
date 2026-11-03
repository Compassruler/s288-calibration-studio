"""CAD screw-seat regressions: fasteners remain rigidly attached to their hosts."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_view import load_scene, component_poses


class FastenerLinkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene = load_scene()
        cls.owners = {name: j["key"] for j in cls.scene["joints"] for name in j["parts"]}
        cls.initial = component_poses(cls.scene, {})

    def test_lower_leg_foot_and_mirrored_fasteners_have_correct_rigid_groups(self):
        cases = {62: "a_knee", 105: "b_knee", **{i:"a_ankle" for i in range(74, 80)},
                 **{i:"b_ankle" for i in (100, 96, 99, 97, 98, 101)}, 122:"head_yaw"}
        for number, owner in cases.items():
            with self.subTest(screw=number):
                self.assertEqual(self.owners[f"标准杯头内六角螺钉-{number}"], owner)

    def test_screws_keep_their_relative_pose_through_parent_and_local_joint_motion(self):
        poses = component_poses(self.scene, {"a_hip_pitch": .25, "a_knee": .3, "a_ankle": -.2,
            "b_hip_pitch": -.15, "b_knee": .35, "b_ankle": .1, "neck_pitch": .2, "head_yaw": -.3})
        hosts = {62:"leg-1", 105:"leg-2", 74:"ankle_left-2", 100:"镜向ankle_left-3", 122:"yaw_roll_motion-1"}
        for number, host in hosts.items():
            name = f"标准杯头内六角螺钉-{number}"
            with self.subTest(screw=number, host=host):
                expected = np.linalg.inv(self.initial[host]) @ self.initial[name]
                actual = np.linalg.inv(poses[host]) @ poses[name]
                np.testing.assert_allclose(actual, expected, atol=1e-10)
                self.assertFalse(np.allclose(poses[name], self.initial[name]))

    def test_fixed_trunk_fasteners_do_not_follow_moving_servos(self):
        poses = component_poses(self.scene, {j["key"]: .3 for j in self.scene["joints"]})
        for number in (26, 37, 39, 42, 43, 81, 83, 84):
            name = f"标准杯头内六角螺钉-{number}"
            with self.subTest(screw=number):
                self.assertNotIn(name, self.owners)
                np.testing.assert_allclose(poses[name], self.initial[name], atol=1e-10)

    def test_rdk_board_case_and_distribution_board_remain_on_trunk(self):
        self.assertEqual(self.scene["model_name"], "microduck_仅RDK")
        poses = component_poses(self.scene, {j["key"]: .3 for j in self.scene["joints"]})
        names = ("RDK_X5_散热外壳组合-1/RDK_X5_板卡-1", "RDK_X5_散热外壳组合-1/散热壳_上盖-1",
                 "RDK_X5_散热外壳组合-1/散热壳_底盖-1", "RDK_贴身平板_可编辑-1", "分线板_背板直装_可编辑-1")
        for name in names:
            with self.subTest(component=name):
                self.assertNotIn(name, self.owners)
                np.testing.assert_allclose(poses[name], self.initial[name], atol=1e-10)

    def test_public_metadata_cleanup_keeps_verified_lower_leg_attachment(self):
        component = next(c for c in self.scene["components"] if c["name"] == "标准杯头内六角螺钉-62")
        for item in self.scene["components"]:
            self.assertFalse({"source_name", "source_path", "source_configuration"} & set(item))
        self.assertNotIn("source_assembly", self.scene)
        self.assertEqual(self.owners[component["name"]], "a_knee")


if __name__ == "__main__":
    unittest.main()
