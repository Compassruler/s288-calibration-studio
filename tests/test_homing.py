"""Saved-pose recall stays bounded, measured and interruptible; no hardware."""
import copy
import math
from pathlib import Path
import queue
import sys
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import default_profiles
from controller import Controller, HomeTimeout


class HomingTests(unittest.TestCase):
    def setUp(self):
        self.worker = Controller(queue.Queue())
        self.worker._connect("", [0, 7, 14], demo=True)
        self.bus = self.worker.bus
        self.profiles = {}
        for motor_id in self.worker.ids:
            profile = default_profiles()[motor_id]
            profile.update(zero_offset=self.bus.positions[motor_id] + .08,
                           reference_expos=(self.bus.positions[motor_id] + .08) % math.tau,
                           direction=-1 if motor_id == 7 else 1)
            self.profiles[motor_id] = profile
        self.addCleanup(self.close_worker)

    def close_worker(self):
        if self.worker.is_alive():
            self.worker.send("shutdown")
            self.worker.join(2)
        elif self.worker.bus is not None:
            self.bus.fail_ids.clear()
            self.worker._disconnect()
        self.assertFalse(self.worker.is_alive())

    def drain_events(self):
        events = []
        while not self.worker.events.empty():
            events.append(self.worker.events.get_nowait())
        return events

    def test_home_uses_saved_zero_and_initial_enable_holds_measured_pose(self):
        raw = self.bus.positions[7]
        self.worker._start_home({7: self.profiles[7]})
        active = [request for request in self.bus.history if request["kp"] > 0]
        self.assertAlmostEqual(active[0]["raw_target"], raw)
        self.assertEqual(active[0]["id"], 7)
        state = self.worker.armed
        self.assertAlmostEqual(state["requested_raw"], self.profiles[7]["zero_offset"])
        self.assertNotEqual(state["requested_raw"], self.worker.origins[7])
        self.assertFalse(state["direct_follow"])
        state["last_move"] = time.monotonic() - .02
        self.worker._poll_one(7)
        step = self.bus.history[-1]["raw_target"] - raw
        self.assertGreater(step, 0)
        self.assertLess(step, math.radians(.4))

    def test_invalid_batch_never_sends_nonzero_gain_to_any_motor(self):
        for invalid in ("missing_zero", "zero_outside_limits", "current_outside_limits", "unconnected"):
            with self.subTest(invalid=invalid):
                profiles = copy.deepcopy(self.profiles)
                if invalid == "missing_zero":
                    profiles[14].update(zero_offset=None, reference_expos=None)
                elif invalid == "zero_outside_limits":
                    profiles[14].update(min_deg=1, max_deg=30)
                elif invalid == "current_outside_limits":
                    profiles[14].update(min_deg=-1, max_deg=1)
                else:
                    profiles[13] = dict(default_profiles()[13], zero_offset=.5)
                before = len(self.bus.history)
                with self.assertRaises(ValueError):
                    self.worker._start_home(profiles)
                self.assertIsNone(self.worker.homing)
                self.assertIsNone(self.worker.armed)
                self.assertEqual(self.worker.holding, {})
                self.assertTrue(all(request["kp"] == request["kd"] == 0
                                    for request in list(self.bus.history)[before:]))

    def test_sequential_home_retains_and_refreshes_all_completed_zero_targets(self):
        clock = time.monotonic()
        with patch("controller.time.monotonic", side_effect=lambda: clock):
            self.worker._start_home(self.profiles)
            first_held = None
            held_started = None
            for _ in range(400):
                clock += .02
                if self.worker.armed:
                    self.worker._poll_one(self.worker.armed["id"])
                for motor_id in list(self.worker.holding):
                    self.worker._poll_one(motor_id)
                self.worker._check_home_progress()
                if self.worker.holding and first_held is None:
                    first_held = next(iter(self.worker.holding))
                    held_started = len(self.bus.history)
                if self.worker.homing is None:
                    break
            self.assertIsNone(self.worker.homing)
            self.assertIsNone(self.worker.armed)
            self.assertEqual(set(self.worker.holding), set(self.profiles))
            for motor_id, profile in self.profiles.items():
                self.worker._poll_one(motor_id)
                self.assertAlmostEqual(self.bus.history[-1]["raw_target"], profile["zero_offset"])
                self.assertGreater(self.bus.history[-1]["kp"], 0)
                self.assertLessEqual(abs(self.bus.positions[motor_id]-profile["zero_offset"]), math.radians(1))
            later = [request for request in list(self.bus.history)[held_started:]
                     if request["id"] == first_held]
            self.assertGreater(len(later), 5)
            self.assertTrue(all(request["kp"] > 0 for request in later))

    def test_ramp_completion_alone_or_motion_near_zero_is_not_settled(self):
        clock = time.monotonic()
        with patch("controller.time.monotonic", side_effect=lambda: clock):
            self.worker._start_home({0: self.profiles[0]})
            state = self.worker.armed
            state["target_raw"] = state["zero"]
            self.worker.feedback[0].update(raw_pos=state["zero"]+math.radians(6),
                                           expos=(state["reference_expos"]+math.radians(6)) % math.tau,
                                           speed=0, timestamp=clock)
            self.worker._check_home_progress()
            self.assertIsNone(self.worker.homing["stable_since"])
            self.worker.feedback[0].update(raw_pos=state["zero"], expos=state["reference_expos"], speed=math.radians(3))
            self.worker._check_home_progress()
            self.assertIsNone(self.worker.homing["stable_since"])
            self.bus.positions[0] = state["zero"]
            self.worker.feedback[0]["speed"] = 0
            self.worker._check_home_progress()
            clock += .24
            self.worker.feedback[0]["timestamp"] = clock
            self.worker._check_home_progress()
            self.assertIsNotNone(self.worker.homing)
            clock += .02
            self.worker.feedback[0]["timestamp"] = clock
            self.worker._check_home_progress()
            self.assertIn(0, self.worker.holding)
            self.assertIsNotNone(self.worker.homing)
            self.worker._check_home_progress()
            clock += .26
            self.worker.feedback[0]["timestamp"] = clock
            self.worker._check_home_progress()
            self.assertIsNone(self.worker.homing)

    def test_timeout_releases_every_motor_without_latching_healthy_connection(self):
        self.worker._start_home(self.profiles)
        self.worker.homing["deadline"] = time.monotonic()-1
        with self.assertRaises(HomeTimeout) as failure:
            self.worker._check_home_progress()
        before = len(self.bus.history)
        self.worker._home_failed(failure.exception)
        self.assertFalse(self.worker.faulted)
        self.assertIs(self.worker.bus, self.bus)
        self.assertFalse(self.bus.closed)
        self.assertIsNone(self.worker.homing)
        self.assertIsNone(self.worker.armed)
        self.assertEqual(self.worker.holding, {})
        sent = list(self.bus.history)[before:]
        self.assertEqual({request["id"] for request in sent}, set(self.profiles))
        self.assertTrue(all(request["kp"] == request["kd"] == 0 for request in sent))
        self.worker._check_home_progress()
        self.assertIsNone(self.worker.armed)

    def test_first_joint_moving_out_of_limits_after_preflight_cleans_startup_plan(self):
        exchange = self.bus.exchange
        calls = 0
        def moved_between_checks(*args, **kwargs):
            nonlocal calls
            calls += 1
            if calls == 2:
                self.bus.positions[0] = self.profiles[0]["zero_offset"] + math.radians(40)
            return exchange(*args, **kwargs)
        before = len(self.bus.history)
        with patch.object(self.bus, "exchange", side_effect=moved_between_checks):
            with self.assertRaises(ValueError):
                self.worker._start_home({0: self.profiles[0]})
        self.assertIsNone(self.worker.homing)
        self.assertIsNone(self.worker.armed)
        self.assertEqual(self.worker.holding, {})
        self.assertTrue(all(request["kp"] == request["kd"] == 0
                            for request in list(self.bus.history)[before:]))

    def test_final_verification_requires_earlier_held_joints_to_stay_at_zero(self):
        clock = time.monotonic()
        with patch("controller.time.monotonic", side_effect=lambda: clock):
            self.worker._start_home({0: self.profiles[0]})
            state = self.worker.armed
            state.update(target_raw=state["zero"], requested_raw=state["zero"])
            self.worker.holding[0] = state
            self.worker.armed = None
            self.worker.homing.update(pending=[], deadline=clock+5, stable_since=None)
            self.worker.feedback[0].update(raw_pos=state["zero"]+math.radians(6),
                                           expos=(state["reference_expos"]+math.radians(6)) % math.tau,
                                           speed=0, timestamp=clock)
            self.worker._check_home_progress()
            clock += .3
            self.worker.feedback[0]["timestamp"] = clock
            self.worker._check_home_progress()
            self.assertIsNotNone(self.worker.homing)
            self.assertIsNone(self.worker.homing["stable_since"])
            self.worker.feedback[0]["raw_pos"] = state["zero"]
            self.worker.feedback[0]["expos"] = state["reference_expos"]
            self.worker._check_home_progress()
            clock += .26
            self.worker.feedback[0]["timestamp"] = clock
            self.worker._check_home_progress()
            self.assertIsNone(self.worker.homing)

    def test_pending_urgent_release_cancels_running_and_queued_home_without_rearming(self):
        self.worker._start_home(self.profiles)
        before = len(self.bus.history)
        self.worker.send("home", profiles=self.profiles)
        self.worker.send("release_all")
        released = threading.Event()
        original = self.worker._release_all
        def observe_release(*args, **kwargs):
            result = original(*args, **kwargs)
            released.set()
            return result
        with patch.object(self.worker, "_release_all", side_effect=observe_release):
            self.worker.start()
            self.assertTrue(released.wait(1))
            self.worker.send("shutdown")
            self.worker.join(2)
        self.assertIsNone(self.worker.homing)
        self.assertIsNone(self.worker.armed)
        self.assertEqual(self.worker.holding, {})
        self.assertTrue(all(request["kp"] == request["kd"] == 0
                            for request in list(self.bus.history)[before:]))

    def test_release_then_new_home_still_sends_initial_hold_before_trajectory(self):
        pose = self.bus.positions[7]
        started = threading.Event()
        initial_packets = []
        original = self.worker._home_status

        def observe_status(message):
            original(message)
            if message.startswith("正在回零："):
                initial_packets.extend(request for request in self.bus.history if request["kp"] > 0)
                started.set()

        self.worker.send("release_all")
        self.worker.send("home", profiles={7: self.profiles[7]})
        with patch.object(self.worker, "_home_status", side_effect=observe_status):
            self.worker.start()
            self.assertTrue(started.wait(1), "new home was not started")
            self.close_worker()
        self.assertEqual(len(initial_packets), 1)
        self.assertEqual(initial_packets[0]["id"], 7)
        self.assertEqual(initial_packets[0]["raw_target"], pose)

    def test_disconnect_and_manual_actions_do_not_resume_a_home_plan(self):
        self.worker._start_home(self.profiles)
        for action in (lambda: self.worker._select(7),
                       lambda: self.worker._capture_zero(7),
                       lambda: self.worker._set_target(0, 2)):
            with self.assertRaises(ValueError):
                action()
        before = len(self.bus.history)
        self.worker._disconnect()
        self.worker._check_home_progress()
        self.assertIsNone(self.worker.homing)
        self.assertEqual(self.worker.holding, {})
        self.assertIsNone(self.worker.armed)
        self.assertTrue(self.bus.closed)
        self.assertTrue(all(request["kp"] == request["kd"] == 0
                            for request in list(self.bus.history)[before:]))


if __name__ == "__main__":
    unittest.main()
