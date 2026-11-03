"""CAD reference transforms and read-only preview behavior; no real motor devices."""
import copy
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import time
import tkinter as tk
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robot_view as view


def transform(position, rotation=None, scale=1):
    return [*(np.eye(3) if rotation is None else rotation).reshape(-1),
            *position, scale, 0, 0, 0]


def scene():
    return {
        "version": 1, "units": "m", "display_rotation": np.eye(3).reshape(-1).tolist(),
        "components": [
            {"name": name, "mesh": "part.stl", "transform": transform(position)}
            for name, position in (("upper", [.5, 0, 0]), ("lower", [2, 0, 0]), ("fixed", [4, 0, 0]))
        ],
        "joints": [
            {"key": "hip", "label": "髋", "parent": None, "axis": [0, 0, 1],
             "pivot": [0, 0, 0], "parts": ["upper"], "motor_component": "upper"},
            {"key": "knee", "label": "膝", "parent": "hip", "axis": [0, 0, 1],
             "pivot": [1, 0, 0], "parts": ["lower"], "motor_component": "lower"},
        ],
    }


def tetrahedron(path):
    points = [(0, 0, 0), (.03, 0, 0), (0, .04, 0), (0, 0, .05)]
    faces = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]
    data = bytearray(80) + struct.pack("<I", len(faces))
    for face in faces:
        a, b, c = [np.asarray(points[i]) for i in face]
        normal = np.cross(b-a, c-a)
        normal = normal / np.linalg.norm(normal)
        data.extend(struct.pack("<12fH", *normal, *a, *b, *c, 0))
    path.write_bytes(data)


class PoseTests(unittest.TestCase):
    def test_solidworks_row_basis_and_translation(self):
        model = scene()
        row_basis = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        model["components"][0]["transform"] = transform([2, 3, 4], row_basis)
        pose = view.component_poses(model, {})["upper"]
        np.testing.assert_allclose(pose @ [1, 0, 0, 1], [2, 4, 4, 1], atol=1e-12)

    def test_parent_moves_descendants_but_child_does_not_move_parent(self):
        model = scene()
        poses = view.component_poses(model, {"hip": math.pi/2})
        np.testing.assert_allclose(poses["lower"][:3, 3], [0, 2, 0], atol=1e-12)
        poses = view.component_poses(model, {"knee": math.pi/2})
        np.testing.assert_allclose(poses["upper"][:3, 3], [.5, 0, 0], atol=1e-12)
        np.testing.assert_allclose(poses["lower"][:3, 3], [1, 1, 0], atol=1e-12)

    def test_reference_world_axes_compose_in_parent_order(self):
        poses = view.component_poses(scene(), {"hip": math.pi/2, "knee": math.pi/2})
        np.testing.assert_allclose(poses["lower"][:3, 3], [-1, 1, 0], atol=1e-12)
        np.testing.assert_allclose(poses["fixed"][:3, 3], [4, 0, 0], atol=1e-12)

    def test_unbound_zero_pose_and_display_rotation(self):
        model = scene()
        initial = view.component_poses(model, {})
        np.testing.assert_allclose(initial["lower"][:3, 3], [2, 0, 0])
        model["display_rotation"] = [0, -1, 0, 1, 0, 0, 0, 0, 1]
        rotated = view.component_poses(model, {})
        np.testing.assert_allclose(rotated["fixed"][:3, 3], [0, 4, 0])

    def test_mesh_scale_does_not_corrupt_body_rotation(self):
        model = scene()
        model["components"][0]["transform"][12] = 3
        pose = view.component_poses(model, {})["upper"]
        np.testing.assert_allclose(pose[:3, :3], np.eye(3))
        with self.assertRaises(ValueError):
            view.component_poses(model, {"hip": float("nan")})

    def test_bindings_are_explicit_one_to_one_and_finite(self):
        good = {"14": {"joint": "knee", "direction": -1, "reference_raw": None}}
        self.assertEqual(view.validate_bindings(good, {"hip", "knee"}), good)
        invalid = [
            {"15": good["14"]}, {"14": {**good["14"], "joint": "absent"}},
            {"14": {**good["14"], "direction": True}},
            {"14": {**good["14"], "direction": 1.0}},
            {"14": {**good["14"], "reference_raw": float("nan")}},
            {"14": good["14"], "13": good["14"]},
        ]
        for bindings in invalid:
            with self.subTest(bindings=bindings), self.assertRaises(ValueError):
                view.validate_bindings(bindings, {"hip", "knee"})

    def test_feedback_reference_is_independent_of_software_zero(self):
        bindings = {"14": {"joint": "knee", "direction": -1, "reference_raw": 3}}
        snapshots = {14: {"zero_offset": 999, "feedback": {"raw_pos": 3.5, "timestamp": 10}}}
        angles, stale = view.feedback_angles(bindings, snapshots, now=10)
        self.assertEqual(stale, set())
        self.assertAlmostEqual(angles["knee"], -.5)
        bindings["14"]["reference_raw"] = None
        angles, stale = view.feedback_angles(bindings, snapshots, previous=angles, now=10)
        self.assertNotIn("knee", angles)

    def test_stale_missing_faulty_feedback_freezes_previous_pose(self):
        binding = {"14": {"joint": "knee", "direction": 1, "reference_raw": 0}}
        cases = [{}, {14: {"feedback": {"raw_pos": 3, "timestamp": 9}}},
                 {14: {"feedback": {"raw_pos": float("nan"), "timestamp": 10}}},
                 {14: {"feedback": {"raw_pos": 3, "timestamp": 10, "fault": 1}}}]
        for snapshots in cases:
            angles, stale = view.feedback_angles(binding, snapshots, {"knee": .2}, now=10)
            self.assertEqual(angles, {"knee": .2})
            self.assertEqual(stale, {14})

    def test_binding_file_rejects_other_mode_or_changed_assets(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as temporary:
            path = Path(temporary) / "bindings.json"
            bindings = {"14": {"joint": "knee", "direction": 1, "reference_raw": .4}}
            path.write_text(json.dumps({"version": 1, "simulation": False,
                "model_sha256": "original", "bindings": bindings}), encoding="utf-8")
            self.assertEqual(view.load_bindings(path, {"knee"}, simulation=False,
                fingerprint="original"), bindings)
            for mode, fingerprint in ((True, "original"), (False, "changed")):
                with self.assertRaises(ValueError):
                    view.load_bindings(path, {"knee"}, simulation=mode, fingerprint=fingerprint)

    def test_scene_rejects_bad_parent_axes_and_ownership(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as temporary:
            directory = Path(temporary)
            tetrahedron(directory / "part.stl")
            for mutation in ("parent", "axis", "owner", "path"):
                model = scene()
                if mutation == "parent":
                    model["joints"][0]["parent"] = "knee"
                elif mutation == "axis":
                    model["joints"][0]["axis"] = [0, 0, 2]
                elif mutation == "owner":
                    model["joints"][1]["parts"] = ["upper"]
                else:
                    model["components"][0]["mesh"] = "../part.stl"
                filename = directory / "model.json"
                filename.write_text(json.dumps(model), encoding="utf-8")
                with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                    view.load_scene(filename)

    def test_explicit_compatible_model_keeps_binding_but_other_models_and_modes_do_not(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as temporary:
            path = Path(temporary) / "bindings.json"
            bindings = {"14": {"joint": "knee", "direction": -1, "reference_raw": .79}}
            path.write_text(json.dumps({"version": 1, "simulation": True,
                "model_sha256": "previous", "bindings": bindings}), encoding="utf-8")
            self.assertEqual(view.load_bindings(path, {"knee"}, simulation=True,
                fingerprint="corrected", compatible_fingerprints=("previous",)), bindings)
            for mode, accepted in ((True, ()), (False, ("previous",))):
                with self.assertRaises(ValueError):
                    view.load_bindings(path, {"knee"}, simulation=mode,
                        fingerprint="corrected", compatible_fingerprints=accepted)


class RendererTests(unittest.TestCase):
    def test_mocap_preview_does_not_step_physics(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as temporary:
            directory = Path(temporary)
            tetrahedron(directory / "part.stl")
            model = scene()
            model["components"][0]["transform"] = transform([0, 0, 0])
            model["components"][1]["transform"] = transform([.07, 0, 0])
            model["components"][2]["transform"] = transform([0, .07, 0])
            model["joints"][1]["pivot"] = [.04, 0, 0]
            filename = directory / "model.json"
            filename.write_text(json.dumps(model), encoding="utf-8")
            model = view.load_scene(filename)
            renderer = view._Renderer(model)
            try:
                frame = renderer.frame(view.component_poses(model, {"hip": .2}), "upper")
                self.assertEqual(frame.shape, (500, 800, 3))
                self.assertGreater(frame.max(), 0, "initial context must draw the model")
                self.assertEqual(renderer.model.njnt, 0)
                self.assertEqual(renderer.model.nmocap, 3)
                self.assertEqual(renderer.data.time, 0)
                compiled_model, data, camera = renderer.model, renderer.data, renderer.camera
                for size in ((900, 500), (400, 700)):
                    with patch.object(view._Renderer, "compile", side_effect=AssertionError("must reuse model")):
                        self.assertTrue(renderer.resize(size))
                        resized = renderer.frame(view.component_poses(model, {"hip": .2}), "upper")
                    self.assertEqual(resized.shape, (size[1], size[0], 3))
                    self.assertGreater(resized.max(), 0, "resized context must still draw the model")
                    self.assertIs(renderer.model, compiled_model)
                    self.assertIs(renderer.data, data)
                    self.assertIs(renderer.camera, camera)
                self.assertFalse(renderer.resize((400, 700)))
                self.assertEqual(renderer.data.time, 0)
            finally:
                renderer.close()


class ReadOnlyWindowTests(unittest.TestCase):
    def test_binding_capture_and_render_never_access_motor_controller(self):
        class App:
            @property
            def controller(self):
                raise AssertionError("3D viewer must not access motor control")
            @property
            def profiles(self):
                raise AssertionError("3D reference must not depend on software zero")
        class Renderer:
            def __init__(self, _):
                self.closed = False
                self.camera = SimpleNamespace(azimuth=225., elevation=-20., distance=1., lookat=(0., 0., 0.))
            def frame(self, *_):
                return np.zeros((20, 30, 3), dtype=np.uint8)
            def close(self):
                self.closed = True
        root = tk.Tk()
        root.withdraw()
        app = App()
        app.root, app.selected, app.demo = root, 14, tk.BooleanVar(root, value=True)
        app.snapshots = {14: {"feedback": {"raw_pos": .56, "timestamp": time.monotonic()}}}
        real_toplevel = tk.Toplevel
        def withdrawn(master):
            window = real_toplevel(master)
            window.withdraw()
            return window
        try:
            with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as temporary:
                path = Path(temporary) / "model_bindings.json"
                model = scene()
                model["_fingerprint"] = "test-assets"
                with patch.object(view, "load_scene", return_value=model), \
                     patch.object(view, "BINDINGS_PATH", path), \
                     patch.object(view, "_Renderer", Renderer), \
                     patch.object(view.tk, "Toplevel", side_effect=withdrawn), \
                     patch.object(view.messagebox, "askyesno", return_value=True), \
                     patch.object(view.messagebox, "showerror") as errors:
                    viewer = view.RobotView(app)
                    try:
                        viewer._bind()
                        viewer._capture()
                        root.update()
                        saved = json.loads(path.with_name("demo_model_bindings.json").read_text(encoding="utf-8"))
                        self.assertTrue(saved["simulation"])
                        self.assertEqual(saved["model_sha256"], "test-assets")
                        self.assertEqual(saved["bindings"]["14"]["reference_raw"], .56)
                        errors.assert_not_called()
                    finally:
                        viewer.close()
        finally:
            root.destroy()


if __name__ == "__main__":
    unittest.main()

