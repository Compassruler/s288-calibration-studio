"""Mechanical ExPos is independent of a rebootable, continuous rotor count."""
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
from protocol import POSITION_LIMIT, ProtocolError, build_packet
from zero_coordinates import joint_angle_rad, wrap_pi


class OutputBus:
    def __init__(self, profiles):
        self.positions = {mid: p["reference_expos"] for mid, p in profiles.items()}
        self.shifts = {mid: (-10 if mid % 2 else 15) + mid * .1 for mid in profiles}
        self.lag = dict.fromkeys(profiles, 0.0)
        self.history = []
        self.max_step = math.inf
        self.closed = False

    def exchange(self, motor_id, raw_target=0, kp=0, kd=0, timeout_flag=1):
        build_packet(motor_id, raw_target, kp, kd, timeout_flag)
        if self.closed:
            raise ProtocolError("closed memory bus")
        self.history.append(dict(id=motor_id, raw_target=raw_target, kp=kp, kd=kd))
        if kp:
            delta = raw_target - self.shifts[motor_id] - self.positions[motor_id]
            self.positions[motor_id] += max(-self.max_step, min(self.max_step, delta))
        return dict(id=motor_id, raw_pos=self.positions[motor_id] + self.shifts[motor_id],
                    expos=(self.positions[motor_id] + self.lag[motor_id]) % math.tau,
                    speed=0, torque=0, voltage=12, temp=30, mode=1, fault=0,
                    warning=0, timeout=0, timestamp=time.monotonic())

    def close(self):
        self.closed = True


class OutputZeroControllerTests(unittest.TestCase):
    def setUp(self):
        self.clock = time.monotonic()
        self.enterContext(patch("controller.time.monotonic", side_effect=lambda: self.clock))
        self.enterContext(patch("controller.time.sleep", side_effect=self.advance))
        self.profiles = default_profiles()
        for mid, profile in self.profiles.items():
            profile.update(zero_offset=3 + mid * .1, reference_expos=(.02 + mid * .4) % math.tau,
                           direction=1 if mid % 2 else -1, min_deg=-60, max_deg=60)
        self.bus = OutputBus(self.profiles)
        self.factory = self.enterContext(patch("controller.Bus", return_value=self.bus))
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

    def arm(self, mid=0, profile=None):
        if self.worker.selected != mid:
            self.worker._select(mid)
        p = profile or self.profiles[mid]
        self.worker._arm(mid, **{key: p[key] for key in (
            "zero_offset", "reference_expos", "kp", "kd", "direction", "min_deg", "max_deg", "rate_deg_s", "direct_follow")})

    def test_all_ids_arbitrary_rotor_offsets_hold_first_packet_and_zero_actual_output(self):
        saved = copy.deepcopy(self.profiles)
        for mid, profile in self.profiles.items():
            self.bus.positions[mid] += profile["direction"] * math.radians(12)
            before = len(self.bus.history)
            initial_wire = self.bus.positions[mid] + self.bus.shifts[mid]
            self.arm(mid)
            first = next(packet for packet in self.bus.history[before:] if packet["kp"])
            self.assertAlmostEqual(first["raw_target"], initial_wire)
            event = [event for event in list(self.worker.events.queue) if event["type"] == "armed"][-1]
            self.assertAlmostEqual(joint_angle_rad(profile, event["expos"]), math.radians(12))
            self.worker._set_target(mid, 0)
            self.advance(.02)
            self.worker._poll_one(mid)
            self.assertAlmostEqual(self.bus.positions[mid], profile["reference_expos"])
            self.assertAlmostEqual(self.bus.history[-1]["raw_target"], profile["reference_expos"] + self.bus.shifts[mid])
            self.worker._release(mid)
        self.assertEqual(self.profiles, saved)

    def test_release_hand_movement_and_counter_reset_then_rearm_realigns_only_once(self):
        self.arm()
        self.worker._release(0)
        self.bus.shifts[0] -= math.radians(420)
        self.bus.positions[0] -= math.radians(25)  # Crosses the ExPos 0 boundary.
        self.worker._zero_exchange(0)
        self.assertFalse(self.worker.faulted)
        self.assertAlmostEqual(math.degrees(joint_angle_rad(self.profiles[0], self.worker.feedback[0]["expos"])), 25)
        expected_wire = self.bus.positions[0] + self.bus.shifts[0]
        before = len(self.bus.history)
        self.arm()
        self.assertAlmostEqual(next(p for p in self.bus.history[before:] if p["kp"])["raw_target"], expected_wire)
        aligned = self.worker.armed["zero"]
        self.bus.lag[0] = math.radians(1)
        self.worker._poll_one(0)
        self.assertEqual(self.worker.armed["zero"], aligned, "ExPos is not used to silently rebase each packet")
        self.worker._set_target(0, 0)
        self.worker._poll_one(0)
        self.assertAlmostEqual(self.bus.positions[0], self.profiles[0]["reference_expos"])

    def test_snapshot_has_independent_output_origin_and_reconnect_discards_control_state(self):
        snapshot = [e for e in list(self.worker.events.queue) if e["type"] == "snapshot" and e["id"] == 0][-1]
        self.assertAlmostEqual(snapshot["origin_expos"], self.profiles[0]["reference_expos"])
        self.assertNotAlmostEqual(snapshot["origin"], snapshot["origin_expos"])
        self.arm()
        self.worker._disconnect()
        replacement = OutputBus(self.profiles)
        replacement.shifts[0] = -20
        self.bus = replacement
        self.factory.return_value = replacement
        self.worker._connect("MEMORY", [0])
        self.assertIsNone(self.worker.armed)
        self.arm()
        self.assertAlmostEqual(self.worker.armed["target_raw"], replacement.positions[0] - 20)

    def test_actual_expos_limits_and_missing_reference_block_without_driving(self):
        for profile in (dict(self.profiles[0], reference_expos=None),
                        dict(self.profiles[0], min_deg=-180), dict(self.profiles[0], max_deg=180)):
            before = len(self.bus.history)
            with self.assertRaises(ValueError):
                self.arm(profile=profile)
            self.assertFalse(any(p["kp"] for p in self.bus.history[before:]))
        self.bus.positions[0] += math.radians(75)
        with self.assertRaisesRegex(ValueError, "超出设定范围"):
            self.arm()

    def test_all_home_targets_output_references_not_saved_raw_metadata(self):
        profiles = copy.deepcopy(self.profiles)
        for mid, p in profiles.items():
            self.bus.positions[mid] += p["direction"] * math.radians(8)
        self.worker._start_home(profiles)
        for _ in range(1500):
            self.advance(.02)
            ids = [*self.worker.holding, *([self.worker.armed["id"]] if self.worker.armed else [])]
            for mid in ids:
                self.worker._poll_one(mid)
            self.worker._check_home_progress()
            if self.worker.homing is None:
                break
        self.assertIsNone(self.worker.homing)
        self.assertEqual(len(self.worker.holding), 15)
        for mid, p in profiles.items():
            self.assertAlmostEqual(wrap_pi(self.worker.feedback[mid]["expos"]-p["reference_expos"]), 0)
        self.assertEqual(profiles, self.profiles)

    def test_home_arrival_uses_actual_output_error_not_matched_raw_target(self):
        self.bus.lag[0] = math.radians(6)
        self.worker._start_home({0: self.profiles[0]})
        state = self.worker.armed
        state["target_raw"] = state["requested_raw"] = state["zero"]
        self.worker.feedback[0].update(raw_pos=state["zero"], expos=(state["reference_expos"] + math.radians(6)) % math.tau,
                                       speed=0, timestamp=self.clock)
        self.assertFalse(self.worker._at_home(state, self.worker.feedback[0]))
        self.worker.homing["deadline"] = self.clock - 1
        with self.assertRaisesRegex(HomeTimeout, "位置偏差 -6.00°"):
            self.worker._check_home_progress()

    def test_active_raw_counter_reset_stops_but_released_manual_motion_has_no_lost_flag(self):
        self.arm()
        self.bus.max_step = .001
        self.bus.shifts[0] += math.radians(420)
        self.advance(.02)
        with self.assertRaisesRegex(ProtocolError, "活动连续位置") as error:
            self.worker._poll_one(0)
        self.worker._fault(error.exception)
        self.assertIsNone(self.worker.armed)
        self.assertTrue(self.worker.faulted)

    def test_later_batch_wire_zero_outside_protocol_rejects_every_motor_before_first_drive(self):
        self.bus.positions[14] = self.profiles[14]["reference_expos"] - .1
        self.bus.shifts[14] = POSITION_LIMIT - .01 - self.bus.positions[14]
        before = len(self.bus.history)
        with self.assertRaisesRegex(ValueError, "ID 14.*协议范围"):
            self.worker._start_home(self.profiles)
        self.assertFalse(any(p["kp"] for p in self.bus.history[before:]))
        self.assertIsNone(self.worker.armed)
        self.assertIsNone(self.worker.homing)

    def test_circular_capture_across_zero_and_batch_failure_are_atomic(self):
        exchange = self.bus.exchange
        count = 0

        def near_boundary(mid, *args, **kwargs):
            nonlocal count
            value = exchange(mid, *args, **kwargs)
            count += 1
            value["expos"] = (math.tau + math.radians(.04 if count % 2 else -.04)) % math.tau
            return value

        with patch.object(self.bus, "exchange", side_effect=near_boundary):
            self.worker._capture_all_zero(list(range(15)))
        event = [e for e in list(self.worker.events.queue) if e["type"] == "captured_all"][-1]
        self.assertEqual(len(event["captures"]), 15)
        self.assertTrue(all(abs(wrap_pi(record["expos"])) < math.radians(.01) for record in event["captures"]))
        self.worker.events = queue.Queue()

        def one_unstable_output(mid, *args, **kwargs):
            value = near_boundary(mid, *args, **kwargs)
            if mid == 14:
                value["expos"] = math.radians(1 if count % 2 else 2)
            return value

        with patch.object(self.bus, "exchange", side_effect=one_unstable_output):
            with self.assertRaisesRegex(ValueError, "ID 14.*输出编码器"):
                self.worker._capture_all_zero(list(range(15)))
        self.assertFalse(any(e["type"] in ("captured", "captured_all") for e in list(self.worker.events.queue)))

    def test_queued_single_capture_cancellation_ack_has_current_scope_and_id(self):
        self.worker.send("capture_zero", id=0)
        self.worker.send("release_all")
        self.worker.start()
        while True:
            event = self.worker.events.get(timeout=1)
            if event["type"] == "capture_cancelled":
                break
        self.worker.send("shutdown")
        self.worker.join(2)
        self.assertEqual((event["scope"], event["id"]), ("current", 0))


if __name__ == "__main__":
    unittest.main()
