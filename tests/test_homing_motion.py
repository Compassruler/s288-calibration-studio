"""Home S-curves exercised through actual Controller position packets."""
import copy
import math
from pathlib import Path
import queue
import sys
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import default_profiles
from controller import Controller, HomeTimeout
from homing_motion import HomingMotion
from zero_coordinates import wrap_pi
import test_output_zero_controller as output_fixture


class HomingMotionMathTests(unittest.TestCase):
    def test_long_move_has_cruise_and_only_one_ramp_time_of_duration_overhead(self):
        motion = HomingMotion(0, math.radians(60), math.radians(15))
        self.assertAlmostEqual(motion.duration, 4.6)
        self.assertAlmostEqual(motion.ramp_s, .6)
        self.assertAlmostEqual(motion.cruise_s, 3.4)
        self.assertEqual(motion.peak_speed, motion.max_speed)
        self.assertEqual(motion.position_at(0), 0)
        self.assertEqual(motion.position_at(motion.duration), motion.target_raw)

    def test_short_moves_lower_peak_velocity_without_a_minimum_distance_jump(self):
        for degrees in (.000001, .001, .1, 1, 5, 8.99):
            with self.subTest(degrees=degrees):
                motion = HomingMotion(0, math.radians(degrees), math.radians(15))
                self.assertLessEqual(motion.peak_speed, motion.max_speed)
                self.assertEqual(motion.cruise_s, 0)
                self.assertGreaterEqual(motion.ramp_s, .12)
                self.assertLess(motion.ramp_s, .6)
                self.assertGreater(motion.position_at(motion.duration * .1), 0)
                self.assertLess(motion.position_at(motion.duration * .1), motion.distance * .1)
        self.assertLess(HomingMotion(0, math.radians(.1), math.radians(15)).duration, .3)

    def test_velocity_and_acceleration_join_continuously_at_ramp_cruise_and_end(self):
        for target in (1.0, -1.0, .01):
            motion = HomingMotion(0, target, .25)
            eps = .0001
            for join in (0, motion.ramp_s, motion.ramp_s + motion.cruise_s, motion.duration):
                left = (motion.position_at(join)-motion.position_at(join-eps))/eps
                right = (motion.position_at(join+eps)-motion.position_at(join))/eps
                self.assertAlmostEqual(left, right, delta=1e-6)
                left_acc = (motion.position_at(join)-2*motion.position_at(join-eps)
                            + motion.position_at(join-2*eps))/eps**2
                right_acc = (motion.position_at(join+2*eps)-2*motion.position_at(join+eps)
                             + motion.position_at(join))/eps**2
                self.assertAlmostEqual(left_acc, right_acc, delta=.0001)

    def test_zero_move_and_invalid_inputs(self):
        motion = HomingMotion(3, 3, .1)
        self.assertEqual(motion.duration, 0)
        self.assertEqual(motion.position_at(10), 3)
        for values in ((0, 1, 0), (0, 1, -1), (True, 1, 1), (0, math.nan, 1), (0, 1, math.inf),
                       (0, 1, 1e-320)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                HomingMotion(*values)
        with self.assertRaises(ValueError):
            motion.position_at(math.nan)


class SmoothHomeControllerTests(unittest.TestCase):
    def setUp(self):
        self.clock = time.monotonic()
        self.enterContext(patch("controller.time.monotonic", side_effect=lambda: self.clock))
        self.enterContext(patch("controller.time.sleep", side_effect=self.advance))
        self.profiles = default_profiles()
        for mid, p in self.profiles.items():
            p.update(zero_offset=3 + mid*.1, reference_expos=.02 + mid*.4,
                     min_deg=-60, max_deg=60, direction=1 if mid % 2 else -1)
        self.bus = output_fixture.OutputBus(self.profiles)
        self.enterContext(patch("controller.Bus", return_value=self.bus))
        self.worker = Controller(queue.Queue())
        self.worker._connect("MEMORY", list(range(15)))
        self.addCleanup(self.cleanup_worker)

    def advance(self, seconds):
        self.clock += seconds

    def cleanup_worker(self):
        if self.worker.is_alive():
            self.worker.send("shutdown")
            self.worker.join(2)
            self.assertFalse(self.worker.is_alive())
        if self.worker.bus:
            self.worker._disconnect()

    def start_home(self, angle_deg, direction=1):
        p = self.profiles[0]
        p["direction"] = direction
        self.bus.positions[0] = p["reference_expos"] + direction*math.radians(angle_deg)
        before = len(self.bus.history)
        started = self.clock
        self.worker._start_home({0: p})
        state = self.worker.armed
        initial = [packet for packet in self.bus.history[before:] if packet["kp"]]
        self.assertEqual(len(initial), 1)
        self.assertEqual(initial[0]["raw_target"], self.bus.positions[0]+self.bus.shifts[0])
        self.assertEqual(state["home_elapsed"], 0)
        return state, started

    def test_packets_ease_in_out_for_both_directions_short_and_long_distances(self):
        for direction, degrees in ((1, 60), (-1, 60), (1, -60), (-1, -.1), (1, .001), (1, 5)):
            with self.subTest(direction=direction, degrees=degrees):
                state, _ = self.start_home(degrees, direction)
                motion = state["home_motion"]
                samples = [(state["home_elapsed"], state["target_raw"])]
                while state["home_elapsed"] < motion.duration:
                    self.advance(.02)
                    self.worker._poll_one(0)
                    samples.append((state["home_elapsed"], self.bus.history[-1]["raw_target"]))
                rates = [abs(q1-q0)/(t1-t0) for (t0,q0),(t1,q1) in zip(samples,samples[1:])]
                self.assertLessEqual(max(rates), state["rate"] + 1e-10)
                self.assertLess(rates[0], max(rates)*.1)
                self.assertLess(rates[-1], max(rates)*.1)
                self.assertEqual(samples[-1][1], state["zero"])
                self.assertAlmostEqual(wrap_pi(self.worker.feedback[0]["expos"]-self.profiles[0]["reference_expos"]), 0)
                self.worker._release_all()

    def test_capped_dt_advances_local_trajectory_without_wall_clock_catchup(self):
        state, started = self.start_home(60)
        motion = state["home_motion"]
        self.assertAlmostEqual(self.worker.homing["deadline"]-started, motion.duration+5)
        self.advance(.2)  # Cached feedback remains within the .25s safety bound.
        self.worker._poll_one(0)
        self.assertAlmostEqual(state["home_elapsed"], .08)
        self.assertEqual(state["target_raw"], motion.position_at(.08))
        self.advance(.02)
        self.worker._poll_one(0)
        self.assertAlmostEqual(state["home_elapsed"], .1)
        self.assertAlmostEqual(state["target_raw"], motion.position_at(.1))

    def test_zero_distance_has_no_added_ramp_delay_and_final_holding_drops_motion(self):
        state, started = self.start_home(0)
        self.assertEqual(state["home_motion"].duration, 0)
        while self.worker.homing is not None:
            self.advance(.02)
            self.worker._poll_one(0)
            self.worker._check_home_progress()
        self.assertLess(self.clock-started, .7)
        self.assertNotIn("home_motion", self.worker.holding[0])
        self.assertNotIn("home_elapsed", self.worker.holding[0])

    def test_smooth_home_keeps_expos_arrival_tolerance_and_timeout_release(self):
        state, _ = self.start_home(5)
        motion = state["home_motion"]
        while state["home_elapsed"] < motion.duration:
            self.advance(.02)
            self.worker._poll_one(0)
        self.worker.feedback[0].update(expos=(state["reference_expos"]+math.radians(6))%math.tau,
                                       timestamp=self.clock, speed=0)
        self.worker.homing["deadline"] = self.clock-1
        with self.assertRaises(HomeTimeout) as failure:
            self.worker._check_home_progress()
        self.worker._home_failed(failure.exception)
        self.assertIsNone(self.worker.armed)
        self.assertFalse(self.worker.holding)
        self.assertIsNone(self.worker.homing)
        self.assertFalse(self.worker.faulted)

    def test_urgent_release_discards_queued_home_and_never_resumes_trajectory(self):
        self.start_home(45)
        self.advance(.02)
        self.worker._poll_one(0)
        before = len(self.bus.history)
        self.worker.send("home", profiles={0: self.profiles[0]})
        self.worker.send("release_all")
        self.worker.start()
        while True:
            event = self.worker.events.get(timeout=1)
            if event["type"] == "home_state" and "全部释放已确认" in event["message"]:
                break
        self.worker.send("shutdown")
        self.worker.join(2)
        self.assertFalse(self.worker.is_alive())
        self.assertTrue(all(p["kp"]==p["kd"]==0 for p in self.bus.history[before:]))
        self.assertIsNone(self.worker.armed)
        self.assertIsNone(self.worker.homing)

    def test_fifteen_sequential_homes_preserve_output_references_and_profile_values(self):
        saved = copy.deepcopy(self.profiles)
        for mid, p in self.profiles.items():
            self.bus.positions[mid] += p["direction"]*math.radians(.1 if mid%2 else 12)
        self.worker._start_home(self.profiles)
        for _ in range(2500):
            self.advance(.02)
            for mid in [*self.worker.holding,*([self.worker.armed["id"]] if self.worker.armed else [])]:
                self.worker._poll_one(mid)
            self.worker._check_home_progress()
            if self.worker.homing is None:
                break
        self.assertIsNone(self.worker.homing)
        self.assertEqual(len(self.worker.holding), 15)
        for mid,p in self.profiles.items():
            self.assertAlmostEqual(wrap_pi(self.worker.feedback[mid]["expos"]-p["reference_expos"]), 0)
            self.assertNotIn("home_motion", self.worker.holding[mid])
        self.assertEqual(self.profiles, saved)


if __name__ == "__main__":
    unittest.main()
