"""Selection colors only the chosen motor, while parent/child poses remain intact."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robot_view as view
from test_robot_view import scene, tetrahedron, transform


class JointHighlightTests(unittest.TestCase):
    def test_real_renderer_switches_one_motor_and_restores_all_other_colors(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as temporary:
            directory = Path(temporary)
            tetrahedron(directory / "part.stl")
            model = scene()
            colors = [[.2, .4, .7, 1], [.8, .3, .2, 1], [.5, .5, .5, 1]]
            for component, position, color in zip(model["components"],
                    ([0, 0, 0], [.07, 0, 0], [0, .07, 0]), colors):
                component.update(transform=transform(position), color=color)
            model["joints"][1]["pivot"] = [.04, 0, 0]
            filename = directory / "model.json"
            filename.write_text(json.dumps(model), encoding="utf-8")
            model = view.load_scene(filename)
            poses = view.component_poses(model, {"hip": .1, "knee": .2})
            renderer = view._Renderer(model, size=(320, 240))
            try:
                compiled, data, camera = renderer.model, renderer.data, renderer.camera
                for mode in ("dark", "light"):
                    renderer.set_theme(mode)
                    highlight_color = view.PALETTES[mode]["highlight"]
                    for selected in ("upper", "lower", None, "upper", None):
                        with self.subTest(mode=mode, selected=selected):
                            pixels = renderer.frame(poses, selected)
                            self.assertGreater(pixels.max(), 0)
                            expected = np.array(colors)
                            if selected:
                                expected[0 if selected == "upper" else 1] = highlight_color
                            np.testing.assert_allclose(renderer.model.geom_rgba, expected)
                            self.assertEqual(sum(np.allclose(color, highlight_color)
                                                 for color in renderer.model.geom_rgba), int(selected is not None))
                            np.testing.assert_allclose(renderer.data.mocap_pos,
                                [poses[component["name"]][:3, 3] for component in model["components"]])
                            self.assertIs(renderer.model, compiled)
                            self.assertIs(renderer.data, data)
                            self.assertIs(renderer.camera, camera)
                            self.assertEqual(renderer.data.time, 0)
            finally:
                renderer.close()


if __name__ == "__main__":
    unittest.main()
