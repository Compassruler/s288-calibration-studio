"""Host-rate selection never writes configuration commands to real motors."""

from pathlib import Path
import queue
import sys
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import controller
import protocol


class BaudrateTests(unittest.TestCase):
    def test_serial_uses_selected_host_rate_and_default_stays_6_mbps(self):
        for requested, expected in ((None, 6_000_000), (4_000_000, 4_000_000),
                                    (6_000_000, 6_000_000)):
            with self.subTest(baudrate=requested):
                wire = Mock()
                factory = Mock(return_value=wire)
                with patch.dict(sys.modules, {"serial": types.SimpleNamespace(Serial=factory)}):
                    options = {} if requested is None else {"baudrate": requested}
                    bus = protocol.Bus(" TEST ", .02, **options)
                    self.assertEqual(bus.baudrate, expected)
                    factory.assert_called_once_with("TEST", expected, timeout=.005, write_timeout=.02)
                    wire.write.assert_not_called()
                    bus.close()

    def test_invalid_rates_reject_before_opening_real_or_simulated_bus(self):
        factory = Mock(side_effect=AssertionError("must not open hardware"))
        with patch.dict(sys.modules, {"serial": types.SimpleNamespace(Serial=factory)}):
            for rate in (None, True, False, 4_000_000.0, "4000000", 0, -1, 115200, 5_000_000):
                with self.subTest(rate=rate):
                    with self.assertRaises(ValueError):
                        protocol.Bus("TEST", baudrate=rate)
                    with self.assertRaises(ValueError):
                        protocol.SimBus(baudrate=rate)
        factory.assert_not_called()

    def test_controller_dispatch_passes_4_mbps_and_reports_actual_selection(self):
        worker = controller.Controller(queue.Queue())
        bus = protocol.SimBus(baudrate=4_000_000)
        with patch.object(controller, "Bus", return_value=bus) as factory:
            worker._dispatch("connect", {"port": "TEST", "ids": [0, 14], "baudrate": 4_000_000})
            factory.assert_called_once_with("TEST", baudrate=4_000_000)
            self.assertEqual(bus.baudrate, 4_000_000)
            connected = next(event for event in list(worker.events.queue) if event["type"] == "connected")
            self.assertEqual(connected["baudrate"], 4_000_000)
            self.assertTrue(all(item["kp"] == item["kd"] == 0 for item in bus.history))
            worker._disconnect()

    def test_controller_rejects_invalid_rate_without_opening_or_changing_state(self):
        worker = controller.Controller(queue.Queue())
        with patch.object(controller, "Bus") as real, patch.object(controller, "SimBus") as simulated:
            for demo in (False, True):
                with self.subTest(demo=demo):
                    with self.assertRaises(ValueError):
                        worker._dispatch("connect", {"port": "TEST", "ids": [14],
                                                     "demo": demo, "baudrate": "4000000"})
            real.assert_not_called()
            simulated.assert_not_called()
        self.assertIsNone(worker.bus)
        self.assertEqual(worker.ids, [])
        self.assertTrue(worker.events.empty())

    def test_default_simulation_and_selected_rate_do_not_open_serial(self):
        factory = Mock(side_effect=AssertionError("simulation must not open serial"))
        with patch.dict(sys.modules, {"serial": types.SimpleNamespace(Serial=factory)}):
            for rate in (6_000_000, 4_000_000):
                with self.subTest(rate=rate):
                    worker = controller.Controller(queue.Queue())
                    worker._connect("SIM", [14], demo=True, baudrate=rate)
                    self.assertEqual(worker.bus.baudrate, rate)
                    worker._disconnect()
        factory.assert_not_called()


if __name__ == "__main__":
    unittest.main()
