"""Active motion stops on stale/outside feedback; zero-gain release remains usable."""
import math
from pathlib import Path
import queue
import sys
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import controller
from controller import ACTIVE_LIMIT_MARGIN_DEG, Controller
from protocol import ProtocolError


class ActiveFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.enterContext(patch.object(controller, "Bus", side_effect=AssertionError("real bus forbidden")))
        self.worker = Controller(queue.Queue())
        self.worker._connect("", [0, 7, 14], demo=True)
        self.worker._select(7)
        self.zero = self.worker.feedback[7]["raw_pos"]
        self.worker._arm(7, kp=.3, kd=.02, zero_offset=self.zero, reference_expos=self.zero % math.tau, direction=1,
                         min_deg=-30, max_deg=30, rate_deg_s=15, direct_follow=True)
        self.bus = self.worker.bus
        self.addCleanup(self.worker._disconnect)

    def test_stale_or_future_cached_feedback_blocks_the_next_nonzero_request(self):
        for offset in (-1, 1):
            with self.subTest(offset=offset):
                self.worker.feedback[7]["timestamp"] = time.monotonic() + offset
                before = len(self.bus.history)
                with self.assertRaisesRegex(ProtocolError, "反馈过期或时间异常"):
                    self.worker._poll_one(7)
                self.assertEqual(len(self.bus.history), before)

    def test_outside_cached_position_blocks_manual_and_held_motion(self):
        state = self.worker.armed
        for held in (False, True):
            self.worker.armed = None if held else state
            self.worker.holding = {7: state} if held else {}
            for angle in (-40, 40, 120):
                with self.subTest(held=held, angle=angle):
                    self.worker.feedback[7].update(raw_pos=self.zero + math.radians(angle),
                                                   expos=(self.zero + math.radians(angle)) % math.tau,
                                                   timestamp=time.monotonic())
                    before = len(self.bus.history)
                    with self.assertRaisesRegex(ProtocolError, "超出活动限位"):
                        self.worker._poll_one(7)
                    self.assertEqual(len(self.bus.history), before)

    def test_bad_reply_latches_and_release_still_uses_only_zero_gains(self):
        exchange = self.bus.exchange

        def shifted_feedback(*args, **kwargs):
            return dict(exchange(*args, **kwargs), raw_pos=self.zero + math.radians(40),
                        expos=(self.zero + math.radians(40)) % math.tau)

        before = len(self.bus.history)
        with patch.object(self.bus, "exchange", side_effect=shifted_feedback):
            with self.assertRaisesRegex(ProtocolError, "超出活动限位") as failure:
                self.worker._poll_one(7)
            self.worker._fault(failure.exception)
        sent = list(self.bus.history)[before:]
        self.assertEqual(sum(packet["kp"] > 0 for packet in sent), 1)
        self.assertTrue(all(packet["kp"] == packet["kd"] == 0 for packet in sent[1:]))
        self.assertEqual({packet["id"] for packet in sent[1:]}, {0, 7, 14})
        self.assertTrue(self.worker.faulted)
        self.assertIsNone(self.worker.armed)
        self.assertEqual(self.worker.holding, {})
        released = [event for event in list(self.worker.events.queue) if event["type"] == "released"]
        self.assertEqual(len(released), 3)
        self.assertTrue(all(event["confirmed"] for event in released))

    def test_small_overshoot_margin_does_not_relax_target_limits(self):
        angle = 30 + ACTIVE_LIMIT_MARGIN_DEG / 2
        self.bus.positions[7] = self.zero + math.radians(angle)
        self.worker.feedback[7].update(raw_pos=self.bus.positions[7], expos=self.bus.positions[7] % math.tau,
                                       timestamp=time.monotonic())
        self.worker._poll_one(7)
        with self.assertRaisesRegex(ValueError, "目标角度超出"):
            self.worker._set_target(7, angle)
        self.worker.feedback[7].update(
            raw_pos=self.zero + math.radians(30 + ACTIVE_LIMIT_MARGIN_DEG + .1),
            expos=(self.zero + math.radians(30 + ACTIVE_LIMIT_MARGIN_DEG + .1)) % math.tau,
            timestamp=time.monotonic())
        before = len(self.bus.history)
        with self.assertRaisesRegex(ProtocolError, "超出活动限位"):
            self.worker._poll_one(7)
        self.assertEqual(len(self.bus.history), before)

    def test_cached_device_fault_blocks_a_nonzero_request(self):
        self.worker.feedback[7]["fault"] = 1
        before = len(self.bus.history)
        with self.assertRaisesRegex(ProtocolError, "状态异常"):
            self.worker._poll_one(7)
        self.assertEqual(len(self.bus.history), before)

    def test_active_raw_counter_jump_with_unchanged_expos_blocks_before_next_packet(self):
        for jump in (-math.tau, math.tau):
            with self.subTest(jump=jump):
                self.worker.feedback[7].update(raw_pos=self.zero + jump,
                                               expos=self.zero % math.tau, timestamp=time.monotonic())
                before = len(self.bus.history)
                with self.assertRaisesRegex(ProtocolError, "活动连续位置"):
                    self.worker._poll_one(7)
                self.assertEqual(len(self.bus.history), before)

    def test_measured_limits_use_the_saved_control_direction(self):
        self.worker.armed.update(direction=-1, min_deg=-5, max_deg=30)
        self.worker.feedback[7].update(raw_pos=self.zero + math.radians(10),
                                       expos=(self.zero + math.radians(10)) % math.tau,
                                       timestamp=time.monotonic())
        before = len(self.bus.history)
        with self.assertRaisesRegex(ProtocolError, "超出活动限位"):
            self.worker._poll_one(7)
        self.assertEqual(len(self.bus.history), before)
        self.bus.positions[7] = self.zero - math.radians(10)
        self.worker.feedback[7].update(raw_pos=self.bus.positions[7], expos=self.bus.positions[7] % math.tau,
                                       timestamp=time.monotonic())
        self.worker._poll_one(7)
        self.assertGreater(self.bus.history[-1]["kp"], 0)


if __name__ == "__main__":
    unittest.main()
