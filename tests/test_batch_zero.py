"""All-ID fixture capture is atomic, released and cancellable; no COM ports."""

import math
from pathlib import Path
import queue
import sys
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import default_profiles
import controller
from controller import Controller
from protocol import ProtocolError


class BatchZeroTests(unittest.TestCase):
    def setUp(self):
        self.worker = Controller(queue.Queue())
        self.worker._connect("", [0, 7, 14], demo=True)
        self.bus = self.worker.bus
        self.addCleanup(self.close_worker)
        self.clock = time.monotonic()
        self.enterContext(patch("controller.time.monotonic", side_effect=lambda: self.clock))
        self.enterContext(patch("controller.time.sleep", side_effect=self.advance))
        self.drain()

    def advance(self, duration):
        self.clock += duration

    def close_worker(self):
        if self.worker.is_alive():
            self.worker.send("shutdown")
            self.worker.join(2)
            self.assertFalse(self.worker.is_alive())
        if self.worker.bus is not None:
            self.bus.fail_ids.clear()
            self.worker._disconnect()

    def drain(self):
        events = []
        while not self.worker.events.empty():
            events.append(self.worker.events.get_nowait())
        return events

    def captures(self):
        return [event for event in self.drain() if event["type"] in ("captured", "captured_all")]

    def test_capture_releases_active_motor_then_emits_one_complete_result(self):
        settings = default_profiles()[0]
        self.worker._arm(0, **{key: settings[key] for key in (
            "kp", "kd", "zero_offset", "direction", "min_deg", "max_deg", "rate_deg_s")})
        self.worker._set_target(0, 20)
        before = len(self.bus.history)
        poses = dict(self.bus.positions)
        self.worker._dispatch("capture_all_zero", {"ids": [0, 7, 14]})
        captured = self.captures()
        self.assertEqual(len(captured), 1)
        self.assertEqual(captured[0]["type"], "captured_all")
        records = {record["id"]: record for record in captured[0]["captures"]}
        self.assertEqual(set(records), {0, 7, 14})
        for motor_id in records:
            self.assertAlmostEqual(records[motor_id]["zero_offset"], poses[motor_id])
            self.assertAlmostEqual(records[motor_id]["expos"], poses[motor_id] % math.tau)
            self.assertGreaterEqual(len(self.worker.history[motor_id]), 10)
        self.assertIsNone(self.worker.armed)
        self.assertEqual(self.worker.holding, {})
        self.assertIsNone(self.worker.homing)
        self.assertEqual(self.bus.positions, poses)
        self.assertTrue(all(request["kp"] == request["kd"] == 0
                            for request in list(self.bus.history)[before:]))

    def test_all_fifteen_ids_are_sampled_in_one_operation(self):
        self.worker._disconnect()
        self.worker._connect("", list(range(15)), demo=True)
        self.bus = self.worker.bus
        self.worker._capture_all_zero()
        captured = self.captures()
        self.assertEqual(len(captured), 1)
        self.assertEqual({record["id"] for record in captured[0]["captures"]}, set(range(15)))

    def test_one_unstable_motor_discards_the_entire_batch(self):
        exchange = self.bus.exchange

        def moving(motor_id, *args, **kwargs):
            feedback = exchange(motor_id, *args, **kwargs)
            if motor_id == 14:
                feedback["speed"] = .04
            return feedback

        with patch.object(self.bus, "exchange", side_effect=moving):
            with self.assertRaisesRegex(ValueError, "ID 14.*仍在运动"):
                self.worker._capture_all_zero()
        self.assertEqual(self.captures(), [])
        self.assertIsNone(self.worker.armed)
        self.assertTrue(all(request["kp"] == request["kd"] == 0 for request in self.bus.history))

    def test_mid_sample_bus_failure_discards_the_entire_batch(self):
        exchange = self.bus.exchange
        last_motor_calls = 0

        def disconnected(motor_id, *args, **kwargs):
            nonlocal last_motor_calls
            if motor_id == 14:
                last_motor_calls += 1
                if last_motor_calls == 5:
                    raise ProtocolError("last motor lost")
            return exchange(motor_id, *args, **kwargs)

        with patch.object(self.bus, "exchange", side_effect=disconnected):
            with self.assertRaisesRegex(ProtocolError, "last motor lost"):
                self.worker._capture_all_zero()
        self.assertEqual(self.captures(), [])

    def test_bad_or_changed_id_list_is_rejected_before_release_or_capture(self):
        before = len(self.bus.history)
        for ids in ([0, 7], [0, 7, 14, 14], [0, 7, 13], [], "0-14", [True, 7, 14]):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                self.worker._capture_all_zero(ids)
        self.assertEqual(len(self.bus.history), before)
        self.assertEqual(self.captures(), [])

    def test_urgent_release_during_sampling_prevents_any_capture(self):
        def release_during_wait(duration):
            self.advance(duration)
            self.worker.send("release_all")

        with patch("controller.time.sleep", side_effect=release_during_wait):
            with self.assertRaisesRegex(ValueError, "已取消"):
                self.worker._capture_all_zero()
        self.assertEqual(self.captures(), [])
        self.assertIsNone(self.worker.armed)
        self.assertEqual(self.worker.holding, {})

    def test_urgent_release_during_final_validation_discards_complete_samples(self):
        record = self.worker._zero_record

        def cancel_after_validation(motor_id, samples):
            result = record(motor_id, samples)
            if motor_id == 14:
                self.worker.send("release_all")
            return result

        with patch.object(self.worker, "_zero_record", side_effect=cancel_after_validation):
            with self.assertRaisesRegex(ValueError, "已取消"):
                self.worker._capture_all_zero()
        self.assertEqual(self.captures(), [])

    def test_queued_capture_invalidated_by_release_reports_cancellation(self):
        self.worker.send("capture_all_zero", ids=[0, 7, 14])
        self.worker.send("release_all")
        self.worker.start()
        events = []
        while True:
            event = self.worker.events.get(timeout=1)
            events.append(event)
            if event["type"] == "capture_cancelled":
                break
        self.worker.send("shutdown")
        self.worker.join(2)
        self.assertFalse(self.worker.is_alive())
        self.assertEqual(event["scope"], "all")
        self.assertFalse(any(item["type"] in ("captured", "captured_all") for item in events))
        self.assertIsNone(self.worker.armed)


class HomeToleranceTests(unittest.TestCase):
    def test_five_degree_boundary_keeps_speed_and_target_settling_requirements(self):
        state = {"zero": .5, "target_raw": .5, "reference_expos": .5}
        for degrees in (-5, -4.5, -2.68, 0, 2.68, 4.5, 5):
            with self.subTest(degrees=degrees):
                self.assertTrue(Controller._at_home(state, {
                    "raw_pos": .5 + math.radians(degrees), "expos": .5 + math.radians(degrees), "speed": 0}))
        for degrees in (-5.001, 5.001):
            with self.subTest(degrees=degrees):
                self.assertFalse(Controller._at_home(state, {
                    "raw_pos": .5 + math.radians(degrees), "expos": .5 + math.radians(degrees), "speed": 0}))
        self.assertFalse(Controller._at_home(state, {"raw_pos": .5, "expos": .5, "speed": math.radians(2.1)}))
        self.assertFalse(Controller._at_home(dict(state, target_raw=.501), {"raw_pos": .5, "expos": .5, "speed": 0}))

    def test_homing_completes_with_four_and_a_half_degree_residual(self):
        worker = Controller(queue.Queue())
        worker._connect("", [0], demo=True)
        self.addCleanup(worker._disconnect)
        profile = dict(default_profiles()[0], zero_offset=worker.bus.positions[0] + .08,
                       reference_expos=(worker.bus.positions[0] + .08) % math.tau)
        clock = time.monotonic()
        exchange = worker.bus.exchange

        def residual(motor_id, target=0, kp=0, kd=0, **kwargs):
            feedback = exchange(motor_id, target, kp, kd, **kwargs)
            if kp:
                position = target + math.radians(4.5)
                worker.bus.positions[motor_id] = position
                feedback.update(raw_pos=position, expos=position % math.tau, speed=0)
            return feedback

        with patch("controller.time.monotonic", side_effect=lambda: clock), \
                patch.object(worker.bus, "exchange", side_effect=residual):
            worker._start_home({0: profile})
            for _ in range(150):
                clock += .02
                worker._poll_one(0)
                worker._check_home_progress()
                if worker.homing is None:
                    break
            self.assertIsNone(worker.homing)
            self.assertIn(0, worker.holding)
            self.assertAlmostEqual(worker.holding[0]["target_raw"], profile["zero_offset"])
            self.assertAlmostEqual(math.degrees(worker.feedback[0]["raw_pos"]-profile["zero_offset"]), 4.5)


if __name__ == "__main__":
    unittest.main()
