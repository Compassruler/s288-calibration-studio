"""Wire compatibility and failure handling; never opens a real serial port."""

import math
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import protocol


# Fixed compatibility vectors generated from the previously exercised driver,
# without importing BAM or writing to its directory during this test run.
ZERO_REQUEST = bytes.fromhex("feee9000000000000000000000000000ee08c88f")
POSITION_REQUEST = bytes.fromhex("feee9e0000000000e2501c0008001f00e017edda")
REPLY = bytes.fromhex("fcee1e2a0018fa009cff60e31600040012000050000062f79727")


class FakeSerial:
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.written = []
        self.closed = False

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.written.append(data)
        return len(data)

    def read(self, size):
        if not self.chunks:
            return b""
        chunk = self.chunks.pop(0)
        if len(chunk) > size:
            self.chunks.insert(0, chunk[size:])
        return chunk[:size]

    def close(self):
        self.closed = True


class ProtocolTests(unittest.TestCase):
    def test_commands_match_fixed_20_byte_vectors(self):
        self.assertEqual(protocol.build_packet(0), ZERO_REQUEST)
        request = protocol.build_packet(14, raw_target=1.234, kp=.5, kd=.02)
        self.assertEqual(request, POSITION_REQUEST)
        self.assertEqual(len(request), 20)
        torque, velocity = struct.unpack("<hh", request[4:8])
        self.assertEqual((torque, velocity), (0, 0))

    def test_feedback_units_and_separate_encoders(self):
        feedback = protocol.parse_feedback(REPLY, expected_id=14)
        self.assertEqual(feedback["id"], 14)
        self.assertEqual(feedback["mode"], 1)
        self.assertEqual(feedback["fault"], 0x120004)
        self.assertEqual(feedback["warning"], 2)
        self.assertEqual(feedback["temp"], 42)
        self.assertEqual(feedback["voltage"], 12)
        self.assertAlmostEqual(feedback["raw_pos"], math.tau * 1500000 / 32768 / (70070 / 243))
        self.assertAlmostEqual(feedback["expos"], math.pi)
        self.assertLess(feedback["speed"], 0)
        self.assertGreater(feedback["timestamp"], 0)

    def test_bad_feedback_never_accepted(self):
        corrupt = bytearray(REPLY)
        corrupt[10] ^= 0x10
        cases = [(bytes(corrupt), 14), (REPLY[:-1], 14), (REPLY, 13)]
        for packet, expected_id in cases:
            with self.subTest(packet=packet.hex(), expected_id=expected_id):
                with self.assertRaises(protocol.ProtocolError):
                    protocol.parse_feedback(packet, expected_id)

    def test_dangerous_inputs_rejected_instead_of_clamped(self):
        cases = [
            {"motor_id": 15}, {"motor_id": -1}, {"motor_id": True},
            {"motor_id": 14, "raw_target": math.nan},
            {"motor_id": 14, "raw_target": math.inf},
            {"motor_id": 14, "raw_target": 2000},
            {"motor_id": 14, "kp": -1},
            {"motor_id": 14, "kd": math.nan},
            {"motor_id": 14, "kp": 3000},
            {"motor_id": 14, "timeout_flag": 0, "kp": .3},
        ]
        for kwargs in cases:
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    protocol.build_packet(**kwargs)

    def test_split_serial_reply_and_one_owner(self):
        serial = FakeSerial([b"noise", REPLY[:1], REPLY[1:7], REPLY[7:]])
        factory = unittest.mock.Mock(return_value=serial)
        with patch.dict(sys.modules, {"serial": types.SimpleNamespace(Serial=factory)}):
            bus = protocol.Bus("TEST")
            reply = bus.exchange(14)
            bus.close()
        factory.assert_called_once()
        self.assertEqual(reply["id"], 14)
        self.assertEqual(len(serial.written), 1)
        self.assertTrue(serial.closed)

    def test_serial_timeout_is_bounded_and_rejects_corrupt_reply(self):
        corrupt = bytearray(REPLY)
        corrupt[-1] ^= 1
        serial = FakeSerial([bytes(corrupt)])
        with patch.dict(sys.modules, {"serial": types.SimpleNamespace(Serial=lambda *a, **k: serial)}):
            bus = protocol.Bus("TEST", timeout=.002)
            with self.assertRaises(protocol.ProtocolError):
                bus.exchange(14)
            bus.close()

    def test_demo_never_opens_serial(self):
        forbidden = unittest.mock.Mock(side_effect=AssertionError("serial must not open"))
        with patch.dict(sys.modules, {"serial": types.SimpleNamespace(Serial=forbidden)}):
            bus = protocol.SimBus()
            self.assertEqual(bus.exchange(14)["id"], 14)
            self.assertEqual(len(bus.positions), 15)
            bus.close()
        forbidden.assert_not_called()


if __name__ == "__main__":
    unittest.main()
