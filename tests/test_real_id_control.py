"""Real-mode UI -> controller -> S288 wire packets, with no physical serial port."""
import math
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
import tkinter as tk
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import app
import protocol


class WireSerial:
    """Replies to actual Bus packets, keeping independent encoder values per ID."""
    def __init__(self):
        self.positions = {mid: -.4 + mid*.075 for mid in range(15)}
        self.history = []
        self.rx = bytearray()
        self.lock = threading.Lock()
        self.closed = False
        self.wrong_ids = set()
        self.missing_ids = set()

    def sent(self):
        with self.lock:
            return list(self.history)

    def reset_input_buffer(self):
        with self.lock:
            self.rx.clear()

    def write(self, packet):
        if self.closed:
            raise OSError("test port is closed")
        if len(packet) != 20 or packet[:2] != b"\xfe\xee":
            raise AssertionError("incorrect outgoing frame")
        if struct.unpack("<I", packet[16:])[0] != protocol.crc32(packet[:16]):
            raise AssertionError("incorrect outgoing CRC")
        motor = packet[2] & 15
        torque, speed, raw, kp, kd = struct.unpack("<hhihh", packet[4:16])
        target = raw*math.tau/32768/protocol.RATIO
        with self.lock:
            self.history.append(dict(id=motor, target=target, kp=kp, kd=kd,
                                     torque=torque, speed=speed))
            if kp:
                self.positions[motor] = target
            if motor in self.missing_ids:
                return len(packet)
            position = round(self.positions[motor]*protocol.RATIO*32768/math.tau)
            reply_id = (motor+1) % 15 if motor in self.wrong_ids else motor
            output = round((self.positions[motor] % math.tau)*8192/math.tau) % 8192
            reply = b"\xfc\xee" + bytes([reply_id | 0x10]) + struct.pack(
                "<bBBhhiIH2x", 35, 0, 24, 0, 0, position, 0, output)
            self.rx.extend(reply + struct.pack("<I", protocol.crc32(reply[2:22])))
        return len(packet)

    def read(self, size):
        with self.lock:
            count = min(size, 7, len(self.rx))  # Exercise split serial feedback too.
            result = bytes(self.rx[:count])
            del self.rx[:count]
            return result

    def close(self):
        self.closed = True


class RealIdControlTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.wire = WireSerial()
        self.factory = self.enterContext(patch("serial.Serial", return_value=self.wire))
        self.enterContext(patch("controller.SimBus", side_effect=AssertionError("real path must use Bus")))
        self.enterContext(patch.object(app.CalibratorApp, "_refresh_ports"))
        self.errors = self.enterContext(patch.object(app.messagebox, "showerror"))
        self.enterContext(patch.object(app.messagebox, "showwarning"))
        self.root = tk.Tk()
        self.root.withdraw()
        self.callback_errors = []
        self.root.report_callback_exception = lambda *error: self.callback_errors.append(error)
        self.application = app.CalibratorApp(self.root, demo=False,
            config_path=Path(self.folder.name)/"calibration.json")
        self.application.port.set("OFFLINE_WIRE_TEST")
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.application.controller.send("shutdown")
        self.application.controller.join(3)
        self.assertFalse(self.application.controller.is_alive())
        for callback in self.root.tk.call("after", "info"):
            self.root.after_cancel(callback)
        self.root.destroy()
        self.folder.cleanup()
        self.assertEqual(self.callback_errors, [])

    def wait(self, predicate, timeout=3):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.root.update()
            if predicate():
                return
            time.sleep(.005)
        self.fail("real-mode wire path did not reach expected state")

    def connect(self, ids):
        self.application.ids.set(ids)
        self.application._connect()
        self.wait(lambda: self.application.connected and not self.application.connecting)
        self.assertIsInstance(self.application.controller.bus, protocol.Bus)

    def test_all_ids_encode_the_selected_motor_and_its_own_zero_and_direction(self):
        application = self.application
        self.assertEqual(application.ids.get(), "14")
        for mid, profile in application.profiles.items():
            profile.update(zero_offset=self.wire.positions[mid]+.02,
                           direction=1 if mid % 2 else -1)
        self.connect("0-14")
        for mid in range(15):
            with self.subTest(id=mid):
                application.tree.selection_set(str(mid))
                self.wait(lambda: application.selected == mid and application.controller.selected == mid
                          and application.armed_id is None)
                self.wait(lambda: not application.arm_button.instate(["disabled"]))
                before = len(self.wire.sent())
                measured = application.snapshots[mid]["feedback"]["raw_pos"]
                application.arm_button.invoke()
                self.wait(lambda: application.armed_id == mid)
                active = [request for request in self.wire.sent()[before:] if request["kp"]]
                self.assertEqual(active[0]["id"], mid)
                self.assertAlmostEqual(active[0]["target"], measured, places=5)
                profile = application.profiles[mid]
                for angle, input_method in ((6.5, "slider"), (-4.2, "typed"), (-4.1, "jog")):
                    start = len(self.wire.sent())
                    expected = profile["zero_offset"]+profile["direction"]*math.radians(angle)
                    if input_method == "slider":
                        application._slide(str(angle))
                    elif input_method == "typed":
                        application.target.set(str(angle))
                        application.execute_button.invoke()
                    else:
                        application._jog(.1)
                    self.wait(lambda: any(request["id"] == mid and request["kp"] and
                              abs(request["target"]-expected) < 1e-5 for request in self.wire.sent()[start:]))
                    sent = self.wire.sent()[start:]
                    self.assertTrue(all(request["id"] == mid for request in sent if request["kp"]))
                    self.assertTrue(all(request["torque"] == request["speed"] == 0 for request in sent))
                application.release_button.invoke()
                self.wait(lambda: application.armed_id is None)
                self.assertTrue(any(request["id"] == mid and request["kp"] == request["kd"] == 0
                                    for request in self.wire.sent()[before:]))
        self.factory.assert_called_once_with("OFFLINE_WIRE_TEST", 6000000,
                                             timeout=.005, write_timeout=.03)
        self.errors.assert_not_called()

    def test_unconnected_id_cannot_enable_or_send_a_position_target(self):
        self.connect("0,7,14")
        self.application.tree.selection_set("3")
        self.wait(lambda: self.application.selected == 3)
        self.assertTrue(self.application.arm_button.instate(["disabled"]))
        self.assertTrue(self.application.angle_slider.instate(["disabled"]))
        self.assertTrue(self.application.release_button.instate(["disabled"]))
        self.assertIn("ID 3 未连接", self.application.mode_text.get())
        self.application._slide("20")
        self.assertFalse(any(request["id"] == 3 for request in self.wire.sent()))
        self.assertIsNone(self.application.armed_id)

    def test_switching_active_ids_releases_old_address_before_enabling_new_one(self):
        application = self.application
        self.connect("5,11")
        self.wait(lambda: application.selected == 5 and application.controller.selected == 5)
        self.wait(lambda: not application.arm_button.instate(["disabled"]))
        application.arm_button.invoke()
        self.wait(lambda: application.armed_id == 5)
        start = len(self.wire.sent())
        application._slide("8")
        application.tree.selection_set("11")
        self.wait(lambda: application.selected == 11 and application.controller.selected == 11
                  and application.armed_id is None)
        self.wait(lambda: not application.arm_button.instate(["disabled"]))
        application.arm_button.invoke()
        self.wait(lambda: application.armed_id == 11)
        sent = self.wire.sent()[start:]
        first_new = next(index for index, request in enumerate(sent)
                         if request["id"] == 11 and request["kp"])
        self.assertTrue(any(request["id"] == 5 and request["kp"] == request["kd"] == 0
                            for request in sent[:first_new]))
        self.assertFalse(any(request["id"] == 5 and request["kp"] for request in sent[first_new:]))
        self.errors.assert_not_called()

    def test_wrong_or_missing_id_feedback_cannot_complete_real_connection(self):
        for failure in (self.wire.wrong_ids, self.wire.missing_ids):
            with self.subTest(failure="wrong" if failure is self.wire.wrong_ids else "missing"):
                self.wire.closed = False
                failure.add(7)
                self.application.ids.set("7")
                self.application._connect()
                self.wait(lambda: not self.application.connecting and self.wire.closed)
                self.assertFalse(self.application.connected)
                self.assertIsNone(self.application.armed_id)
                self.assertTrue(all(request["kp"] == request["kd"] == 0 for request in self.wire.sent()))
                failure.clear()


if __name__ == "__main__":
    unittest.main()
