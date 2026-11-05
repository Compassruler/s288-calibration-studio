"""Single-turn zero math, including circular samples across 0/360 degrees."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from zero_coordinates import circular_mean, joint_angle_rad, wrap_pi


class ZeroCoordinateTests(unittest.TestCase):
    def test_both_directions_and_crossing_single_turn_boundary(self):
        profile = dict(zero_offset=-400, reference_expos=math.radians(359), direction=1)
        self.assertAlmostEqual(joint_angle_rad(profile, math.radians(1)), math.radians(2))
        profile["direction"] = -1
        self.assertAlmostEqual(joint_angle_rad(profile, math.radians(1)), -math.radians(2))
        profile.update(reference_expos=math.radians(1), direction=1)
        self.assertAlmostEqual(joint_angle_rad(profile, math.radians(359)), -math.radians(2))

    def test_raw_zero_is_metadata_and_output_reference_has_priority(self):
        for raw in (None, -1000, 0, 1200):
            self.assertAlmostEqual(joint_angle_rad(dict(zero_offset=raw, reference_expos=.2, direction=-1), .3), -.1)

    def test_unrecorded_zero_uses_this_connections_output_origin(self):
        profile = dict(zero_offset=None, reference_expos=None, direction=1)
        self.assertAlmostEqual(joint_angle_rad(profile, .05, math.tau - .05), .1)
        with self.assertRaisesRegex(ValueError, "本次连接"):
            joint_angle_rad(profile, .05)

    def test_old_accumulated_zero_without_expos_is_an_explicit_error(self):
        with self.assertRaisesRegex(ValueError, "缺少 ExPos"):
            joint_angle_rad(dict(zero_offset=.5, reference_expos=None, direction=1), .6, .5)

    def test_exact_half_turn_is_ambiguous_and_invalid_values_are_rejected(self):
        profile = dict(zero_offset=1, reference_expos=0, direction=1)
        with self.assertRaisesRegex(ValueError, "180°"):
            joint_angle_rad(profile, math.pi)
        for value in (True, math.nan, math.inf, -1, math.tau):
            with self.subTest(value=value), self.assertRaises(ValueError):
                joint_angle_rad(profile, value)
        for value in (True, math.nan, math.inf):
            with self.assertRaises(ValueError):
                wrap_pi(value)

    def test_circular_mean_and_std_do_not_average_boundary_samples_to_pi(self):
        mean, std = circular_mean([math.radians(359.98), math.radians(.02)] * 10)
        self.assertAlmostEqual(wrap_pi(mean), 0, places=10)
        self.assertAlmostEqual(math.degrees(std), .02, places=5)
        for samples in ([], [0, math.pi], [True], [math.nan]):
            with self.subTest(samples=samples), self.assertRaises(ValueError):
                circular_mean(samples)


if __name__ == "__main__":
    unittest.main()
