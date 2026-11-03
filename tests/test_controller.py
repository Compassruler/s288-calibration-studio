"""Controller invariants exercised with an in-memory bus, never COM ports."""

from collections import deque
import math
from pathlib import Path
import queue
import sys
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import controller
from protocol import ProtocolError, SimBus


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.events = queue.Queue()
        self.bus = SimBus()
        self.factory = patch.object(controller, "Bus", return_value=self.bus)
        self.factory_mock = self.factory.start()
        self.addCleanup(self.factory.stop)
        self.worker = controller.Controller(self.events)
        self.worker._connect("TEST", [14])
        self.addCleanup(self.close_worker)

    def close_worker(self):
        if self.worker.is_alive():
            self.worker.send("shutdown")
            self.worker.join(2)
            self.assertFalse(self.worker.is_alive(), "worker did not close in time")
        elif self.worker.bus is not None:
            self.worker._disconnect()

    def arm(self, **updates):
        settings = dict(kp=.3, kd=.02, zero_offset=None, direction=1,
                        min_deg=-30, max_deg=30, rate_deg_s=5)
        settings.update(updates)
        self.worker._arm(14, **settings)

    def stable_history(self, *, speed=0, span=0, age=0):
        now = time.monotonic() - age
        raw = self.bus.positions[14]
        base = self.bus.exchange(14)
        samples = []
        for index in range(25):
            sample = dict(base, raw_pos=raw + span * index / 24,
                          speed=speed, timestamp=now - .48 + .02 * index)
            samples.append(sample)
        self.worker.history[14] = deque(samples, maxlen=100)
        self.worker.feedback[14] = samples[-1]

    def collect_events(self):
        result = []
        while True:
            try:
                result.append(self.events.get_nowait())
            except queue.Empty:
                return result

    def test_connect_is_zero_gain_and_arm_first_targets_latest_position(self):
        self.assertTrue(all(item["kp"] == item["kd"] == 0 for item in self.bus.history))
        self.bus.positions[14] += .07  # Pose changes after the connection reference.
        pose = self.bus.positions[14]
        self.arm()
        active = [item for item in self.bus.history if item["kp"] > 0]
        self.assertTrue(active)
        self.assertAlmostEqual(active[0]["raw_target"], pose)
        self.assertAlmostEqual(self.worker.armed["requested_raw"], pose)

    def test_reverse_direction_and_offset_transform_without_immediate_jump(self):
        zero = self.bus.positions[14]
        self.arm(zero_offset=zero, direction=-1)
        self.worker._set_target(14, 10)
        state = self.worker.armed
        self.assertAlmostEqual(state["requested_raw"], zero - math.radians(10))
        self.assertAlmostEqual(state["target_raw"], zero)
        state["last_move"] = time.monotonic() - .02
        self.worker._poll_one(14)
        self.assertGreaterEqual(self.bus.history[-1]["raw_target"], zero - math.radians(.4))
        self.assertLess(self.bus.history[-1]["raw_target"], zero)

    def test_invalid_or_outside_target_never_reaches_bus(self):
        self.arm()
        original = self.worker.armed["requested_raw"]
        count = len(self.bus.history)
        for target in [31, -31, math.nan, math.inf, True]:
            with self.subTest(target=target):
                with self.assertRaises(ValueError):
                    self.worker._set_target(14, target)
        self.assertEqual(len(self.bus.history), count)
        self.assertEqual(self.worker.armed["requested_raw"], original)

    def test_bad_gain_or_offset_cannot_enable(self):
        for updates in [{"kp": 0}, {"kp": 2.6}, {"kd": -.1}, {"kd": math.nan},
                        {"rate_deg_s": 0}, {"zero_offset": math.inf},
                        {"direction": True}, {"direction": -1.0},
                        {"min_deg": -181}, {"max_deg": 181}, {"zero_offset": 2000}]:
            with self.subTest(updates=updates):
                with self.assertRaises(ValueError):
                    self.arm(**updates)
                self.assertIsNone(self.worker.armed)
        self.assertFalse(any(item["kp"] > 0 for item in self.bus.history))

    def test_hold_cancels_unreached_target(self):
        self.arm()
        self.worker._set_target(14, 20)
        pose = self.worker.feedback[14]["raw_pos"]
        self.worker._hold(14)
        self.assertEqual(self.worker.armed["requested_raw"], pose)
        self.assertEqual(self.worker.armed["target_raw"], pose)

    def test_release_prioritizes_and_invalidates_queued_motion(self):
        self.arm()
        before = len(self.bus.history)
        self.worker.send("target", id=14, angle_deg=25)
        self.worker.send("release", id=14)
        self.worker.send("target", id=14, angle_deg=20)
        self.worker.start()
        deadline = time.monotonic() + 1
        released = False
        while time.monotonic() < deadline:
            if any(event["type"] == "released" for event in self.collect_events()):
                released = True
                break
            time.sleep(.01)
        self.assertTrue(released)
        self.assertIsNone(self.worker.armed)
        self.assertTrue(all(item["kp"] == item["kd"] == 0 for item in list(self.bus.history)[before:]))

    def test_capture_requires_stable_fresh_disabled_feedback(self):
        cases = [{"speed": .1}, {"span": .01}, {"age": 1}]
        for params in cases:
            with self.subTest(params=params):
                self.stable_history(**params)
                with self.assertRaises(ValueError):
                    self.worker._capture_zero(14)
        self.stable_history()
        self.arm()
        with self.assertRaises(ValueError):
            self.worker._capture_zero(14)

    def test_stable_capture_is_configuration_data_and_does_not_move(self):
        self.stable_history()
        self.collect_events()
        before = len(self.bus.history)
        self.worker._capture_zero(14)
        event = next(e for e in self.collect_events() if e["type"] == "captured")
        self.assertAlmostEqual(event["zero_offset"], self.bus.positions[14])
        self.assertEqual(len(self.bus.history), before)
        self.assertIsNone(self.worker.armed)

    def test_active_failure_latches_and_release_is_not_falsely_confirmed(self):
        self.arm()
        self.bus.fail_ids.add(14)
        with self.assertRaises(ProtocolError) as caught:
            self.worker._poll_one(14)
        self.worker._fault(caught.exception)
        self.assertTrue(self.worker.faulted)
        self.assertIsNone(self.worker.armed)
        released = [e for e in self.collect_events() if e["type"] == "released"]
        self.assertTrue(released)
        self.assertTrue(all(not e["confirmed"] for e in released))
        self.bus.fail_ids.clear()
        with self.assertRaises(ValueError):
            self.arm()

    def test_reconnection_forgets_active_target_and_uses_one_bus_for_15_ids(self):
        self.arm()
        self.worker._set_target(14, 20)
        self.worker._disconnect()
        replacement = SimBus()
        self.factory_mock.return_value = replacement
        self.worker._connect("TEST", list(range(15)))
        self.assertIsNone(self.worker.armed)
        self.assertEqual(self.factory_mock.call_count, 2)  # one connection per session
        self.assertEqual(len(self.worker.feedback), 15)
        self.assertTrue(all(item["kp"] == item["kd"] == 0 for item in replacement.history))

    def test_release_all_prioritizes_the_currently_active_motor(self):
        self.worker._disconnect()
        replacement = SimBus()
        self.factory_mock.return_value = replacement
        self.worker._connect("TEST", list(range(15)))
        self.worker._select(14)
        self.arm()
        before = len(replacement.history)
        self.worker._release_all()
        sent = list(replacement.history)[before:]
        self.assertEqual(sent[0]["id"], 14)
        self.assertTrue(all(item["kp"] == item["kd"] == 0 for item in sent))

    def test_hold_prioritizes_and_discards_older_queued_target(self):
        self.arm()
        pose = self.worker.feedback[14]["raw_pos"]
        self.worker.send("target", id=14, angle_deg=25)
        self.worker.send("hold", id=14)
        self.worker.start()
        deadline = time.monotonic() + 1
        held = False
        while time.monotonic() < deadline:
            if any(event["type"] == "held" for event in self.collect_events()):
                held = True
                break
            time.sleep(.01)
        self.assertTrue(held)
        self.assertAlmostEqual(self.worker.armed["requested_raw"], pose)
        self.assertAlmostEqual(self.worker.armed["target_raw"], pose)

    def test_disconnect_can_cancel_connection_handshake(self):
        self.worker._disconnect()
        self.collect_events()
        started = threading.Event()
        proceed = threading.Event()

        class BlockingBus(SimBus):
            def exchange(self, *args, **kwargs):
                if not started.is_set():
                    started.set()
                    if not proceed.wait(1):
                        raise AssertionError("test handshake was not released")
                return super().exchange(*args, **kwargs)

        replacement = BlockingBus()
        self.factory_mock.return_value = replacement
        self.worker.send("connect", port="TEST", ids=list(range(15)))
        self.worker.start()
        try:
            self.assertTrue(started.wait(1))
            self.worker.send("disconnect")
        finally:
            proceed.set()
        deadline = time.monotonic() + 1
        events = []
        while time.monotonic() < deadline:
            events.extend(self.collect_events())
            if replacement.closed:
                break
            time.sleep(.01)
        self.assertTrue(replacement.closed)
        self.assertFalse(any(event["type"] == "connected" for event in events))
        self.assertIsNone(self.worker.armed)
        self.assertTrue(all(item["kp"] == item["kd"] == 0 for item in replacement.history))


if __name__ == "__main__":
    unittest.main()
