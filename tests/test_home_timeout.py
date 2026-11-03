"""A missed saved pose releases safely without hiding real bus failures."""

from pathlib import Path
import queue
import sys
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import controller
from config import default_profiles
from controller import Controller, HomeTimeout
from protocol import ProtocolError


class HomeTimeoutTests(unittest.TestCase):
    def setUp(self):
        self.workers = []
        self.seen = {}
        # Every test connects the in-memory simulator. A real-bus regression
        # must fail here rather than open a port.
        bus = patch.object(controller, "Bus", side_effect=AssertionError("real bus forbidden"))
        bus.start()
        self.addCleanup(bus.stop)
        self.addCleanup(self.close_workers)

    def close_workers(self):
        for worker in self.workers:
            if worker.is_alive():
                worker.send("shutdown")
                worker.join(2)
            elif worker.bus is not None:
                worker._disconnect()
            self.assertFalse(worker.is_alive())

    def make_home(self, ids=(0, 7, 14)):
        worker = Controller(queue.Queue())
        worker._connect("", list(ids), demo=True)
        self.workers.append(worker)
        self.seen[worker] = []
        profiles = {mid: dict(default_profiles()[mid],
                             zero_offset=worker.bus.positions[mid] + .08,
                             direction=-1 if mid == 7 else 1)
                    for mid in ids}
        worker._start_home(profiles)
        worker.homing["deadline"] = time.monotonic() - 1
        return worker, worker.bus, profiles

    def wait_event(self, worker, predicate, timeout=1):
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.fail(f"controller event did not arrive: {self.seen[worker]}")
            try:
                event = worker.events.get(timeout=remaining)
            except queue.Empty:
                self.fail(f"controller event did not arrive: {self.seen[worker]}")
            self.seen[worker].append(event)
            if predicate(event):
                return event

    @staticmethod
    def arm_settings(profile):
        return {key: profile[key] for key in (
            "kp", "kd", "zero_offset", "direction", "min_deg", "max_deg",
            "rate_deg_s", "direct_follow")}

    def test_thread_timeout_discards_old_motion_keeps_polling_and_allows_manual_arm(self):
        worker, bus, profiles = self.make_home()
        check = worker._check_home_progress
        queued = threading.Event()

        def queue_before_timeout():
            if not queued.is_set():
                worker.send("arm", id=0, **self.arm_settings(profiles[0]))
                worker.send("target", id=0, angle_deg=20)
                worker.send("home", profiles=profiles)
                queued.set()
            check()

        with patch.object(worker, "_check_home_progress", side_effect=queue_before_timeout), \
             patch.object(worker, "_dispatch", wraps=worker._dispatch) as dispatch:
            worker.start()
            failure = self.wait_event(worker, lambda event: event["type"] == "error"
                                      and "可手动重试" in event["message"])
            self.assertIn("回零超时", failure["message"])
            self.assertIn("位置偏差", failure["message"])
            self.assertIn("速度", failure["message"])
            self.assertTrue(queued.is_set())
            self.assertFalse(worker.faulted)
            self.assertIs(worker.bus, bus)
            self.assertFalse(bus.closed)
            self.assertIsNone(worker.homing)
            self.assertIsNone(worker.armed)
            self.assertEqual(worker.holding, {})
            before = len(bus.history)
            timestamp = worker.feedback[0]["timestamp"]
            self.wait_event(worker, lambda event: event["type"] == "snapshot" and event["id"] == 0
                            and event["feedback"]["timestamp"] > timestamp + .05)
            idle_requests = list(bus.history)[before:]
            self.assertGreaterEqual(len(idle_requests), 3)
            self.assertTrue(all(item["kp"] == item["kd"] == 0 for item in idle_requests))
            self.assertTrue(worker.commands.empty())
            self.assertFalse(dispatch.called, "older arm/target/home must all be invalidated")
            self.assertIsNone(worker.armed)

            # A new explicit user command may enable again, at the latest
            # measured pose rather than the abandoned zero target.
            pose = worker.feedback[0]["raw_pos"]
            before = len(bus.history)
            worker.send("arm", id=0, **self.arm_settings(profiles[0]))
            self.wait_event(worker, lambda event: event["type"] == "armed")
            active = [item for item in list(bus.history)[before:] if item["kp"] > 0]
            self.assertTrue(active)
            self.assertAlmostEqual(active[0]["raw_target"], pose)
            self.assertAlmostEqual(worker.armed["requested_raw"], pose)

    def test_final_group_timeout_reports_unsettled_joint_and_releases_all(self):
        worker, bus, profiles = self.make_home()
        template = worker.armed
        for mid, profile in profiles.items():
            zero = profile["zero_offset"]
            held = dict(template, id=mid, zero=zero, direction=profile["direction"],
                        target_raw=zero, requested_raw=zero)
            worker.holding[mid] = held
            bus.positions[mid] = zero + (.05 if mid == 7 else 0)
            worker.feedback[mid].update(raw_pos=bus.positions[mid], speed=.1 if mid == 7 else 0,
                                        timestamp=time.monotonic())
        worker.armed = None
        worker.homing["pending"].clear()
        with self.assertRaises(HomeTimeout) as failure:
            worker._check_home_progress()
        self.assertIn("整组回零超时", str(failure.exception))
        self.assertIn("ID 7", str(failure.exception))
        self.assertIn("位置偏差 -2.86°", str(failure.exception))
        self.assertIn("速度 -5.73°/s", str(failure.exception))
        before = len(bus.history)
        worker._home_failed(failure.exception)
        self.assertFalse(worker.faulted)
        self.assertIsNone(worker.homing)
        self.assertIsNone(worker.armed)
        self.assertEqual(worker.holding, {})
        requests = list(bus.history)[before:]
        self.assertEqual({item["id"] for item in requests}, set(profiles))
        self.assertTrue(all(item["kp"] == item["kd"] == 0 and item["timeout_flag"] == 1
                            for item in requests))
        event = self.wait_event(worker, lambda event: event["type"] == "home_state"
                                and "可手动重试" in event["message"])
        self.assertFalse(event["active"])
        self.assertEqual(event["holding_ids"], [])

    def test_failed_timeout_release_stays_latched_and_does_not_clear_protection(self):
        worker, bus, profiles = self.make_home()
        failed = worker._home_failed
        exchange = bus.exchange
        aborting = threading.Event()
        requests = []

        def fail_release(exc):
            aborting.set()
            failed(exc)

        def lose_one_release(motor_id, raw_target=0, kp=0, kd=0, timeout_flag=1):
            if aborting.is_set():
                requests.append((motor_id, kp, kd, timeout_flag))
                if motor_id == 7:
                    raise ProtocolError("模拟释放时失联")
            return exchange(motor_id, raw_target, kp, kd, timeout_flag)

        with patch.object(worker, "_home_failed", side_effect=fail_release), \
             patch.object(bus, "exchange", side_effect=lose_one_release):
            worker.start()
            self.wait_event(worker, lambda event: event["type"] == "error"
                            and "回零超时" in event["message"] and "已锁止" in event["message"])
            self.assertTrue(worker.faulted)
            self.assertIsNone(worker.armed)
            self.assertIsNone(worker.homing)
            self.assertEqual(worker.holding, {})
            self.assertEqual([item[0] for item in requests], [0, 7, 14])
            self.assertTrue(all(kp == kd == 0 and flag == 1 for _, kp, kd, flag in requests))
            self.assertTrue(any(event["type"] == "released" and event["id"] == 7
                                and not event["confirmed"] for event in self.seen[worker]))
        # A subsequent healthy bus does not automatically unlatch a failed
        # release. The user still has to inspect and reconnect.
        before = len(bus.history)
        worker.send("arm", id=0, **self.arm_settings(profiles[0]))
        self.wait_event(worker, lambda event: event["type"] == "error"
                        and "通信异常已锁止" in event["message"])
        self.assertEqual(len(bus.history), before)
        self.assertIsNone(worker.armed)

    def test_real_bus_and_device_errors_still_latch_in_worker_loop(self):
        for kind in ("communication", "fault", "warning", "timeout", "mode"):
            with self.subTest(kind=kind):
                worker, bus, profiles = self.make_home((0,))
                exchange = bus.exchange

                def unhealthy(motor_id, raw_target=0, kp=0, kd=0, timeout_flag=1):
                    if kp and kind == "communication":
                        raise ProtocolError("模拟 CRC 错误")
                    feedback = exchange(motor_id, raw_target, kp, kd, timeout_flag)
                    if kp:
                        feedback[kind] = 0 if kind == "mode" else 1
                    return feedback

                with patch.object(bus, "exchange", side_effect=unhealthy):
                    worker.start()
                    failure = self.wait_event(worker, lambda event: event["type"] == "error"
                                              and "已停止主动控制并锁止" in event["message"])
                    self.assertNotIn("可手动重试", failure["message"])
                    self.assertTrue(worker.faulted)
                    self.assertIsNone(worker.homing)
                    self.assertIsNone(worker.armed)
                    self.assertEqual(worker.holding, {})
                    worker.send("arm", id=0, **self.arm_settings(profiles[0]))
                    self.wait_event(worker, lambda event: event["type"] == "error"
                                    and "通信异常已锁止" in event["message"])
                    worker.send("shutdown")
                    worker.join(2)
                    self.assertFalse(worker.is_alive())

    def test_stale_feedback_is_not_reclassified_as_a_pose_timeout(self):
        worker, _, _ = self.make_home((0,))
        worker.feedback[0]["timestamp"] = time.monotonic() - 1
        with self.assertRaisesRegex(ValueError, "反馈已过期"):
            worker._check_home_progress()

    def test_healthy_release_cannot_clear_an_existing_fault_latch(self):
        worker, _, _ = self.make_home((0,))
        worker.faulted = True
        worker._home_failed(HomeTimeout("回零超时"))
        self.assertTrue(worker.faulted)
        self.assertIsNone(worker.armed)
        self.assertIsNone(worker.homing)


if __name__ == "__main__":
    unittest.main()
