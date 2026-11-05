"""Transient file sharing must not discard model settings or hide permanent failures."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robot_view


class BindingSaveRetryTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.addCleanup(self.folder.cleanup)
        self.viewer = object.__new__(robot_view.RobotView)
        self.viewer.app = SimpleNamespace(model_zero_references=lambda: {})
        self.viewer.keys = {"hip"}
        self.viewer.bindings_path = Path(self.folder.name) / "model_bindings.json"
        self.viewer.simulation = False
        self.viewer.scene = {"_fingerprint": "test-model"}
        self.original = {"0": {"joint": "hip", "direction": 1, "reference_raw": .4}}
        self.viewer.bindings = self.original
        self.viewer._demo_generated = set()
        self.viewer.angles = {"hip": .1}
        self.viewer._refresh_bindings = Mock()
        self.viewer._save_bindings(self.original, set())
        self.saved = self.viewer.bindings_path.read_bytes()

    @staticmethod
    def sharing_error():
        error = PermissionError("file is temporarily unavailable")
        error.winerror = 32
        return error

    def test_short_sharing_conflict_preserves_original_until_successful_replace(self):
        replace = robot_view.os.replace
        attempts = []

        def busy_twice(source, destination):
            attempts.append(source)
            if len(attempts) <= 2:
                self.assertEqual(self.viewer.bindings_path.read_bytes(), self.saved)
                self.assertIs(self.viewer.bindings, self.original)
                raise self.sharing_error()
            return replace(source, destination)

        candidate = {"0": dict(self.original["0"], reference_raw=.7)}
        with patch.object(robot_view.os, "replace", side_effect=busy_twice), \
             patch.object(robot_view.time, "sleep") as pause:
            self.viewer._persist(candidate, set())
        self.assertEqual(len(attempts), 3)
        self.assertEqual([call.args[0] for call in pause.call_args_list], [.02, .04])
        self.assertEqual(self.viewer.bindings, candidate)
        self.assertEqual(json.loads(self.viewer.bindings_path.read_text(encoding="utf-8"))["bindings"], candidate)
        self.assertEqual(list(Path(self.folder.name).glob("*.tmp")), [])

    def test_persistent_conflict_is_bounded_and_keeps_file_and_memory_unchanged(self):
        with patch.object(robot_view.os, "replace", side_effect=self.sharing_error()) as replace, \
             patch.object(robot_view.time, "sleep") as pause:
            with self.assertRaises(PermissionError):
                self.viewer._persist({"0": dict(self.original["0"], reference_raw=.9)}, set())
        self.assertEqual(replace.call_count, 5)
        self.assertAlmostEqual(sum(call.args[0] for call in pause.call_args_list), .2)
        self.assertEqual(self.viewer.bindings_path.read_bytes(), self.saved)
        self.assertIs(self.viewer.bindings, self.original)
        self.viewer._refresh_bindings.assert_not_called()
        self.assertEqual(list(Path(self.folder.name).glob("*.tmp")), [])

    def test_other_save_errors_are_reported_without_retrying(self):
        with patch.object(robot_view.os, "replace", side_effect=PermissionError("read-only directory")) as replace, \
             patch.object(robot_view.time, "sleep") as pause:
            with self.assertRaises(PermissionError):
                self.viewer._save_bindings(self.original, set())
        replace.assert_called_once()
        pause.assert_not_called()
        self.assertEqual(self.viewer.bindings_path.read_bytes(), self.saved)


if __name__ == "__main__":
    unittest.main()
