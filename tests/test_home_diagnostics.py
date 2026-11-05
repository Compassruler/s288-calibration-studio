"""Cached home diagnostics distinguish tracking lag from output-only residuals."""
from contextlib import nullcontext
import math
from pathlib import Path
import queue
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import default_profiles
from controller import Controller, HomeTimeout
from protocol import build_packet
import test_output_zero_controller as output_fixture


class PacketBus(output_fixture.OutputBus):
    def __init__(self, profiles):
        super().__init__(profiles)
        self.packets = []

    def exchange(self, motor_id, raw_target=0, kp=0, kd=0, timeout_flag=1):
        self.packets.append(build_packet(motor_id, raw_target, kp, kd, timeout_flag))
        return super().exchange(motor_id, raw_target, kp, kd, timeout_flag)


class HomeDiagnosticTests(unittest.TestCase):
    def run_case(self, *, locked=False, output_residual=0, initial_degrees=12, report=True):
        clock = [100.0]
        p = default_profiles()[5]
        p.update(zero_offset=1.233071405, reference_expos=1.234764300,
                 kp=1, kd=.01, rate_deg_s=15, direction=1, min_deg=-60, max_deg=60)
        bus = PacketBus({5: p})
        bus.positions[5] += math.radians(initial_degrees)
        if locked:
            bus.max_step = 0
        with patch("controller.Bus", return_value=bus), patch("controller.time.monotonic", side_effect=lambda: clock[0]):
            worker = Controller(queue.Queue())
            worker._connect("MEMORY", [5])
            timeout_message = None
            try:
                worker._start_home({5: p})
                state = worker.armed
                zero = state["zero"]
                deadline = worker.homing["deadline"]
                bus.lag[5] = math.radians(output_residual)
                reporting = nullcontext() if report else patch.object(worker, "_report_home_endpoint", return_value=None)
                with reporting:
                    for _ in range(1000):
                        clock[0] += .02
                        worker._poll_one(5)
                        try:
                            worker._check_home_progress()
                        except HomeTimeout as error:
                            timeout_message = str(error)
                            worker._home_failed(error)
                            break
                        if worker.homing is None:
                            break
                events = list(worker.events.queue)
                result = dict(packets=list(bus.packets), events=events, timeout=timeout_message,
                              zero=state["zero"], requested=state["requested_raw"],
                              target=state["target_raw"], elapsed=clock[0]-100,
                              original_zero=zero, deadline=deadline)
            finally:
                worker._disconnect()
        return result

    def assert_diagnostics_do_not_change_packets(self, scenario, expected_tracking):
        reported = self.run_case(**scenario)
        silent = self.run_case(**scenario, report=False)
        self.assertEqual(reported["packets"], silent["packets"], "logging cannot send extra packets or change packet bytes")
        self.assertEqual(reported["elapsed"], silent["elapsed"])
        self.assertEqual(reported["deadline"], silent["deadline"])
        self.assertEqual(reported["target"], reported["original_zero"])
        self.assertEqual(reported["zero"], reported["original_zero"])
        self.assertEqual(reported["requested"], reported["original_zero"])
        endpoints = [event for event in reported["events"]
                     if event["type"] == "status" and event["message"].startswith("回零轨迹目标已发到终点")]
        self.assertEqual(len(endpoints), 1)
        self.assertIn("输出距零=+5.68°", endpoints[0]["message"])
        self.assertIn("控制位置跟踪差=" + expected_tracking, endpoints[0]["message"])
        self.assertIn("控制位置跟踪差=" + expected_tracking, reported["timeout"])
        self.assertIn("位置偏差 +5.68°", reported["timeout"])
        self.assertIn("速度 +0.00°/s", reported["timeout"])
        for label in ("raw实测=", "raw目标=", "raw名义零位=", "ExPos=", "E0=",
                      "实测扭矩=", "V=", "KP=1", "KD=0.01", "mode/fault/warning/timeout", "轨迹="):
            self.assertIn(label, endpoints[0]["message"])

    def test_raw_and_output_both_stalled_has_raw_tracking_error_and_one_endpoint_report(self):
        self.assert_diagnostics_do_not_change_packets(dict(locked=True, initial_degrees=5.68), "-5.68°")

    def test_raw_at_goal_output_still_offset_has_zero_raw_tracking_error_and_same_packets(self):
        self.assert_diagnostics_do_not_change_packets(dict(output_residual=5.68), "+0.00°")

    def test_inside_five_degree_residual_is_accepted_without_changing_zero_goal(self):
        for scenario, tracking in ((dict(locked=True, initial_degrees=2.68), "-2.68°"),
                                   (dict(output_residual=2.68), "+0.00°")):
            with self.subTest(scenario=scenario):
                result = self.run_case(**scenario)
                self.assertIsNone(result["timeout"])
                self.assertEqual(result["target"], result["original_zero"])
                endpoint = [event for event in result["events"] if event["type"] == "status"
                            and event["message"].startswith("回零轨迹目标已发到终点")]
                self.assertEqual(len(endpoint), 1)
                self.assertIn("输出距零=+2.68°", endpoint[0]["message"])
                self.assertIn("控制位置跟踪差="+tracking, endpoint[0]["message"])

    def test_zero_distance_also_reports_once_without_an_extra_ramp_or_packet(self):
        reported = self.run_case(initial_degrees=0)
        silent = self.run_case(initial_degrees=0, report=False)
        self.assertIsNone(reported["timeout"])
        self.assertLess(reported["elapsed"], .7)
        self.assertEqual(reported["packets"], silent["packets"])
        self.assertEqual(sum(event["type"] == "status" and event["message"].startswith("回零轨迹目标已发到终点")
                             for event in reported["events"]), 1)


if __name__ == "__main__":
    unittest.main()
