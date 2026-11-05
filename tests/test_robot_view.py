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

    def test_calibrated_fifteen_old_model_offsets_are_zero_at_saved_output_zero(self):
        offsets = (-1.9362, -1.3519, .713, 1.5388, -2.8282, .2505, 1.1037,
                   -3.619, 4.208, 5.524, -.6964, -2.3575, -2.0067, 3.1672, .6827)
        negative = {0, 1, 3, 4, 5, 6, 8, 9, 11, 14}
        bindings, snapshots, references = {}, {}, {}
        for mid, offset in enumerate(offsets):
            direction, anchor, reference = (-1 if mid in negative else 1), -30+mid, (.2+mid*.4) % math.tau
            bindings[str(mid)] = dict(joint=f"joint_{mid}", direction=direction,
                reference_raw=anchor-math.radians(offset)/direction)
            snapshots[mid] = dict(feedback=dict(raw_pos=1000-mid*300, model_raw_pos=anchor,
                expos=reference, timestamp=10))
            references[str(mid)] = reference
        original = copy.deepcopy(bindings)
        angles, stale = view.feedback_angles(bindings, snapshots, now=10, zero_references=references)
        self.assertFalse(stale)
        self.assertEqual(angles, {f"joint_{mid}": 0 for mid in range(15)})
        self.assertEqual(bindings, original)

    def test_calibrated_expos_ignores_old_anchor_cache_and_missing_legacy_reference(self):
        for direction in (-1, 1):
            for raw in (-100, 300):
                for reference in (0.01, math.tau-.01):
                    output = (reference+math.radians(2.68)) % math.tau
                    binding = {"14": dict(joint="knee", direction=direction, reference_raw=None)}
                    snapshots = {14: dict(feedback=dict(raw_pos=raw, model_raw_pos=-999,
                        model_reference_error="旧缓存缺少参考", expos=output, timestamp=10))}
                    angles, stale = view.feedback_angles(binding, snapshots, now=10,
                        zero_references={"14": reference})
                    self.assertFalse(stale)
                    self.assertAlmostEqual(angles["knee"], direction*math.radians(2.68))
                    snapshots[14]["feedback"]["timestamp"] = 9
                    frozen, stale = view.feedback_angles(binding, snapshots, angles, now=10,
                        zero_references={"14": reference})
                    self.assertEqual(frozen, angles)
                    self.assertEqual(stale, {14})

    def test_missing_output_reference_error_freezes_none_legacy_reference_and_marks_stale(self):
        binding = {"14": dict(joint="knee", direction=1, reference_raw=None)}
        snapshots = {14: dict(feedback=dict(raw_pos=-40, expos=.7, timestamp=10,
            model_reference_error="旧零位缺少 ExPos"))}
        for previous in ({}, {"knee": .2}):
            frozen, stale = view.feedback_angles(binding, snapshots, previous, now=10)
            self.assertEqual(frozen, previous)
            self.assertEqual(stale, {14})
        angles, stale = view.feedback_angles(binding, snapshots, now=10, zero_references={"14": .5})
        self.assertAlmostEqual(angles["knee"], .2)
        self.assertFalse(stale)

    def test_calibrated_invalid_output_or_reference_freezes_with_all_binding_reference_states(self):
        for legacy in (None, -80):
            binding = {"14": dict(joint="knee", direction=-1, reference_raw=legacy)}
            for invalid in (None, float("nan"), float("inf"), -.1, math.tau):
                feedback = dict(raw_pos=900, model_raw_pos=-999, expos=invalid, timestamp=10)
                if invalid is None:
                    feedback.pop("expos")
                frozen, stale = view.feedback_angles(binding, {14: dict(feedback=feedback)}, {"knee": .2},
                    now=10, zero_references={"14": .5})
                self.assertEqual(frozen, {"knee": .2})
                self.assertEqual(stale, {14})
            for invalid_reference in (float("nan"), float("inf"), -.1, math.tau+.1):
                frozen, stale = view.feedback_angles(binding,
                    {14: dict(feedback=dict(expos=.7, timestamp=10))}, {"knee": .2},
                    now=10, zero_references={"14": invalid_reference})
                self.assertEqual(frozen, {"knee": .2})
                self.assertEqual(stale, {14})

    def test_reference_pose_offsets_are_optional_finite_radians(self):
        model = scene()
        self.assertEqual(view.reference_pose_angles(model), {"hip": 0, "knee": 0})
        model["joints"][0]["zero_pose_offset_rad"] = math.radians(79)
        self.assertAlmostEqual(view.reference_pose_angles(model)["hip"], math.radians(79))
        for value in (True, None, "1", float("nan"), float("inf")):
            model["joints"][0]["zero_pose_offset_rad"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                view.reference_pose_angles(model)

    def test_jig_offset_only_applies_to_calibrated_feedback_and_stale_keeps_rendered_pose(self):
        binding = {"14": dict(joint="hip", direction=-1, reference_raw=.5)}
        snapshot = {14: dict(feedback=dict(raw_pos=.7, expos=.7, timestamp=10))}
        offset = math.radians(79)
        actual, stale = view.feedback_angles(binding, snapshot, now=10,
            zero_references={"14": .5}, zero_offsets={"hip": offset})
        self.assertFalse(stale)
        self.assertAlmostEqual(actual["hip"], offset-.2)
        legacy, stale = view.feedback_angles(binding, snapshot, now=10, zero_offsets={"hip": offset})
        self.assertFalse(stale)
        self.assertAlmostEqual(legacy["hip"], -.2)
        snapshot[14]["feedback"]["timestamp"] = 9
        frozen, stale = view.feedback_angles(binding, snapshot, actual, now=10,
            zero_references={"14": .5}, zero_offsets={"hip": offset})
        self.assertEqual(frozen, actual)
        self.assertEqual(stale, {14})

    def test_loader_rejects_invalid_jig_offset(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as temporary:
            directory = Path(temporary)
            tetrahedron(directory / "part.stl")
            model = scene()
            model["joints"][0]["zero_pose_offset_rad"] = "79 degrees"
            filename = directory / "model.json"
            filename.write_text(json.dumps(model), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "有限弧度"):
                view.load_scene(filename)

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
    def test_compiled_initial_geometry_uses_jig_reference_pose(self):
        import mujoco
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as temporary:
            directory = Path(temporary)
            tetrahedron(directory / "part.stl")
            model = scene()
            model["joints"][0]["zero_pose_offset_rad"] = math.pi/2
            model["joints"][1]["zero_pose_offset_rad"] = -.3
            filename = directory / "model.json"
            filename.write_text(json.dumps(model), encoding="utf-8")
            model = view.load_scene(filename)
            compiled = view._Renderer.compile(model)
            poses = view.component_poses(model, view.reference_pose_angles(model))
            for index, component in enumerate(model["components"]):
                body = mujoco.mj_name2id(compiled, mujoco.mjtObj.mjOBJ_BODY, f"c{index}")
                np.testing.assert_allclose(compiled.body_pos[body], poses[component["name"]][:3, 3])

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
