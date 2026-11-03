"""Cross-check direct-follow cancellation and the preview's coordinate mapping."""
import math
from pathlib import Path
import queue
import sys
import time
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app import CalibratorApp
from controller import Controller


class FollowSafetyReviewTests(unittest.TestCase):
    def make_preview(self, zero, direction):
        state = SimpleNamespace(
            connected=True, armed_id=14, selected=14, faulted=False, closing=False,
            requested_deg=18,
            profiles={14: dict(zero_offset=zero, direction=direction)},
            snapshots={14: dict(origin=.72, feedback=dict(timestamp=time.monotonic(), mode=1,
                                                          fault=0, warning=0, timeout=0))},
        )
        state._can_slide = lambda: CalibratorApp._can_slide(state)
        return state

    def test_preview_uses_motor_coordinate_with_software_direction_and_saved_zero(self):
        state = self.make_preview(.31, -1)
        motor, raw = CalibratorApp.preview_target_raw(state)
        self.assertEqual(motor, 14)
        self.assertAlmostEqual(raw, .31-math.radians(18))
        self.assertEqual(state.profiles[14]["zero_offset"], .31)

    def test_preview_uses_connection_origin_until_calibrated(self):
        state = self.make_preview(None, 1)
        self.assertAlmostEqual(CalibratorApp.preview_target_raw(state)[1], .72+math.radians(18))

    def test_preview_rejects_release_stale_timeout_and_fault(self):
        for change in ("release", "stale", "timeout", "fault", "future"):
            with self.subTest(change=change):
                state = self.make_preview(None, 1)
                if change == "release":
                    state.armed_id = None
                elif change == "stale":
                    state.snapshots[14]["feedback"]["timestamp"] -= 1
                elif change == "future":
                    state.snapshots[14]["feedback"]["timestamp"] += 1
                elif change == "timeout":
                    state.snapshots[14]["feedback"]["timeout"] = 1
                else:
                    state.faulted = True
                self.assertIsNone(CalibratorApp.preview_target_raw(state))

    def test_direct_mode_urgent_actions_invalidate_entire_older_target_burst(self):
        for urgent in ("hold", "release"):
            with self.subTest(urgent=urgent):
                worker = Controller(queue.Queue())
                worker._connect("", [14], demo=True)
                try:
                    worker._arm(14, kp=.3, kd=.02, zero_offset=None, direction=1,
                                min_deg=-30, max_deg=30, rate_deg_s=15, direct_follow=True)
                    original = worker.feedback[14]["raw_pos"]
                    before = len(worker.bus.history)
                    for angle in (3, 9, 18, 27):
                        worker.send("target", id=14, angle_deg=angle)
                    worker.send(urgent, id=14)
                    worker.start()
                    deadline = time.monotonic()+1
                    while time.monotonic() < deadline:
                        if (urgent == "release" and worker.armed is None or
                                urgent == "hold" and worker.commands.empty()):
                            break
                        time.sleep(.005)
                    self.assertTrue(worker.commands.empty())
                    sent = list(worker.bus.history)[before:]
                    if urgent == "release":
                        self.assertTrue(all(packet["kp"] == packet["kd"] == 0 for packet in sent))
                    else:
                        self.assertAlmostEqual(worker.armed["requested_raw"], original)
                        self.assertTrue(all(abs(packet["raw_target"]-original) < 1e-8 for packet in sent))
                finally:
                    worker.send("shutdown")
                    worker.join(2)
                    self.assertFalse(worker.is_alive())


if __name__ == "__main__":
    unittest.main()
