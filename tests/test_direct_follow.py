"""Direct targets stay responsive while limits and urgent actions still apply."""
import math
from pathlib import Path
import queue
import sys
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config
from controller import Controller
from protocol import SimBus


class DirectFollowTests(unittest.TestCase):
    def setUp(self):
        self.bus = SimBus()
        self.events = queue.Queue()
        self.worker = Controller(self.events)
        self.enterContext(patch("controller.Bus", return_value=self.bus))
        self.worker._connect("TEST", [14])
        self.addCleanup(self.cleanup_worker)

    def cleanup_worker(self):
        if self.worker.is_alive():
            self.worker.send("shutdown")
            self.worker.join(2)
        elif self.worker.bus:
            self.worker._disconnect()
        self.assertFalse(self.worker.is_alive())

    def arm(self, direct=True):
        self.worker._arm(14, kp=.3, kd=.02, zero_offset=None, direction=-1,
                         min_deg=-30, max_deg=30, rate_deg_s=1, direct_follow=direct)

    def test_direct_target_reaches_next_packet_without_slew_ramp(self):
        self.arm()
        origin = self.worker.origins[14]
        self.worker._set_target(14, 24)
        self.worker._poll_one(14)
        self.assertAlmostEqual(self.bus.history[-1]["raw_target"], origin - math.radians(24))

    def test_optional_speed_limit_still_interpolates(self):
        self.arm(False)
        origin = self.worker.origins[14]
        self.worker._set_target(14, 24)
        self.worker.armed["last_move"] = time.monotonic() - .02
        self.worker._poll_one(14)
        target = self.bus.history[-1]["raw_target"]
        self.assertGreater(target, origin - math.radians(.1))
        self.assertLess(target, origin)

    def test_direct_mode_preserves_limits_and_first_enable_pose(self):
        pose = self.bus.positions[14]
        self.arm()
        first = next(item for item in self.bus.history if item["kp"] > 0)
        self.assertAlmostEqual(first["raw_target"], pose)
        with self.assertRaises(ValueError):
            self.worker._set_target(14, 31)

    def test_worker_skips_obsolete_targets_before_dispatch(self):
        self.arm()
        for angle in range(10):
            self.worker.send("target", id=14, angle_deg=angle)
        with patch.object(self.worker, "_set_target", wraps=self.worker._set_target) as targets:
            self.worker.start()
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline and not targets.called:
                time.sleep(.005)
            self.worker.send("shutdown")
            self.worker.join(2)
            self.assertEqual(targets.call_args_list, [unittest.mock.call(14, 9)])

    def test_old_profiles_load_with_direct_follow_and_roundtrip(self):
        old = config.default_profiles()[14]
        old.pop("direct_follow")
        self.assertTrue(config.validate_profile(old)["direct_follow"])
        old["direct_follow"] = False
        self.assertFalse(config.validate_profile(old)["direct_follow"])
        old["direct_follow"] = "true"
        with self.assertRaises(ValueError):
            config.validate_profile(old)


if __name__ == "__main__":
    unittest.main()
