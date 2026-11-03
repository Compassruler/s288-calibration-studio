"""A simulated servo slider must move its bound CAD link and descendants."""
from contextlib import ExitStack
from pathlib import Path
import sys
import tempfile
import time
import tkinter as tk
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import app
import robot_view


class StructureLinkTests(unittest.TestCase):
    def test_main_slider_moves_bound_lower_leg_and_foot_without_real_serial(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory, ExitStack() as mocks:
            directory = Path(directory)
            serial = mocks.enter_context(patch("serial.Serial", side_effect=AssertionError("simulation only")))
            mocks.enter_context(patch.object(app.CalibratorApp, "_refresh_ports"))
            mocks.enter_context(patch.object(robot_view, "BINDINGS_PATH", directory / "bindings.json"))
            errors = mocks.enter_context(patch.object(app.messagebox, "showerror"))
            mocks.enter_context(patch.object(app.messagebox, "showwarning"))
            mocks.enter_context(patch.object(app.messagebox, "askyesno", return_value=True))
            original_toplevel = tk.Toplevel
            def withdrawn_toplevel(parent):
                window = original_toplevel(parent)
                window.withdraw()
                return window
            mocks.enter_context(patch.object(robot_view.tk, "Toplevel", side_effect=withdrawn_toplevel))
            root = tk.Tk()
            root.withdraw()
            callback_errors = []
            root.report_callback_exception = lambda *error: callback_errors.append(error)
            application = app.CalibratorApp(root, demo=True, config_path=directory / "calibration.json")
            view = None
            def wait_until(condition, timeout=4):
                deadline = time.monotonic()+timeout
                while time.monotonic() < deadline:
                    root.update()
                    if condition():
                        return
                    time.sleep(.005)
                self.fail("simulated servo/model link did not update")
            try:
                wait_until(lambda: application.connected and 14 in application.snapshots)
                application._open_robot_view()
                view = application.robot_view
                wait_until(lambda: view.renderer is not None and hasattr(view, "labels"), timeout=30)
                # The hidden integration window represents an already mapped viewport.
                mocks.enter_context(patch.object(view.picture, "winfo_ismapped", return_value=True))
                mocks.enter_context(patch.object(view.picture, "winfo_width", return_value=240))
                mocks.enter_context(patch.object(view.picture, "winfo_height", return_value=160))
                self.assertTrue(view.window.winfo_exists())
                view.joint.set(next(label for label, key in view.labels.items() if key == "a_knee"))
                view._bind()
                self.assertEqual(view.binding_table.selection(), ("a_knee",))
                wait_until(lambda: view.bindings.get("14", {}).get("reference_raw") is not None)
                reference = robot_view.component_poses(view.scene, {})
                application.arm_button.invoke()
                wait_until(lambda: application.armed_id == 14)
                application._slide("12.0")
                wait_until(lambda: view.angles.get("a_knee", 0) > .02)
                poses = robot_view.component_poses(view.scene, view.angles)
                self.assertFalse(np.allclose(poses["leg-1"], reference["leg-1"]))
                self.assertFalse(np.allclose(poses["foot_left-1"], reference["foot_left-1"]))
                self.assertTrue(np.allclose(poses["left_upper_leg-1"], reference["left_upper_leg-1"]))
                self.assertTrue(np.allclose(poses["trunk_base-1"], reference["trunk_base-1"]))
                self.assertIsNone(application.profiles[14]["zero_offset"])
                serial.assert_not_called()
                errors.assert_not_called()
                self.assertEqual(callback_errors, [])
            finally:
                if view:
                    view.close()
                application.controller.send("shutdown")
                application.controller.join(2)
                for callback in root.tk.call("after", "info"):
                    root.after_cancel(callback)
                root.destroy()
                self.assertFalse(application.controller.is_alive())


if __name__ == "__main__":
    unittest.main()
