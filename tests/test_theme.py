"""Appearance changes stay independent of calibration and device commands."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import tkinter as tk
from tkinter import ttk
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import app
from guide import SECTIONS, show_guide
import ui


def descendants(widget):
    for child in widget.winfo_children():
        yield child
        yield from descendants(child)


class ThemeTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.directory = Path(self.folder.name)
        self.appearance = self.directory / "configs" / "appearance.json"
        self.config_path = self.appearance.with_name("calibration.json")
        self.enterContext(patch.object(ui, "THEME_PATH", self.appearance))
        self.serial = self.enterContext(patch("serial.Serial", side_effect=AssertionError("no real hardware")))
        self.enterContext(patch.object(app.CalibratorApp, "_refresh_ports"))
        self.root = tk.Tk()
        self.root.withdraw()
        self.errors = []
        self.root.report_callback_exception = lambda *error: self.errors.append(error)
        self.application = None
        self.addCleanup(self.cleanup)

    def cleanup(self):
        if self.application is not None:
            if self.application.robot_view is not None:
                self.application.robot_view.close()
            self.application.controller.send("shutdown")
            self.application.controller.join(2)
            self.assertFalse(self.application.controller.is_alive())
        for callback in self.root.tk.call("after", "info"):
            self.root.after_cancel(callback)
        self.root.destroy()
        self.folder.cleanup()
        self.serial.assert_not_called()
        self.assertEqual(self.errors, [])

    def make_app(self):
        self.application = app.CalibratorApp(self.root, config_path=self.config_path)
        self.root.update_idletasks()
        self.assertIsNone(self.application.robot_view)
        return self.application

    def test_missing_corrupt_and_unsupported_appearance_use_dark(self):
        self.assertEqual(ui.load_theme(), "dark")
        self.appearance.parent.mkdir(parents=True)
        for content in ("{", "null", "[]", '{"theme":"sepia"}', '{"theme":[]}'):
            with self.subTest(content=content):
                self.appearance.write_text(content, encoding="utf-8")
                self.assertEqual(ui.load_theme(), "dark")

    def test_appearance_persistence_does_not_write_calibration_or_binding_files(self):
        self.appearance.parent.mkdir(parents=True)
        protected = {self.config_path: b'calibration content\n',
                     self.appearance.with_name("model_bindings.json"): b'bindings content\n'}
        for path, content in protected.items():
            path.write_bytes(content)
        for mode in ("light", "dark"):
            ui.save_theme(mode)
            self.assertEqual(ui.load_theme(), mode)
            self.assertEqual(json.loads(self.appearance.read_text(encoding="utf-8"))["theme"], mode)
            self.assertEqual({path: path.read_bytes() for path in protected}, protected)
        previous = self.appearance.read_bytes()
        with self.assertRaises(ValueError):
            ui.save_theme("sepia")
        self.assertEqual(self.appearance.read_bytes(), previous)
        self.assertFalse(list(self.appearance.parent.glob("appearance_*.tmp")))

    def test_semantic_widget_and_text_tag_colors_update_without_changing_content(self):
        ui.apply_theme(self.root, "dark")
        text = ui.theme_widget(tk.Text(self.root), background="surface", foreground="ink",
                               selectbackground="accent", selectforeground="accent_ink",
                               tags={"title": {"foreground": "accent"}})
        label = ui.theme_widget(ttk.Label(self.root, text="保留标签"), foreground="muted")
        text.tag_configure("title", font=("Microsoft YaHei UI", 16, "bold"))
        text.insert("end", "标题\n", "title")
        text.insert("end", "正文保持不变")
        text.configure(state="disabled")
        content, font = text.get("1.0", "end"), text.tag_cget("title", "font")
        for mode in ("light", "dark"):
            ui.apply_theme(self.root, mode)
            colors = ui.PALETTES[mode]
            self.assertEqual(self.root.cget("background"), colors["bg"])
            self.assertEqual(text.cget("background"), colors["surface"])
            self.assertEqual(text.cget("foreground"), colors["ink"])
            self.assertEqual(text.cget("selectforeground"), colors["accent_ink"])
            self.assertEqual(text.tag_cget("title", "foreground"), colors["accent"])
            self.assertEqual(str(label.cget("foreground")), colors["muted"])
            self.assertEqual(text.get("1.0", "end"), content)
            self.assertEqual(text.tag_cget("title", "font"), font)
            self.assertEqual(text.cget("state"), "disabled")

    def test_palette_lookup_uses_owning_root_even_for_a_child_window(self):
        ui.apply_theme(self.root, "light")
        child = tk.Toplevel(self.root)
        child.withdraw()
        label = ttk.Label(child)
        other = tk.Tk()
        other.withdraw()
        try:
            ui.apply_theme(other, "dark")
            self.assertEqual(ui.theme_colors(label), ui.PALETTES["light"])
            self.assertEqual(ui.theme_colors(ttk.Label(other)), ui.PALETTES["dark"])
        finally:
            other.destroy()

    def test_cached_combobox_listbox_and_new_popdown_both_follow_theme(self):
        ui.apply_theme(self.root, "dark")
        combo = ttk.Combobox(self.root, values=("A", "B"), state="readonly")
        popup = str(self.root.tk.call("ttk::combobox::PopdownWindow", str(combo)))
        listbox = popup + ".f.l"
        self.assertTrue(self.root.tk.call("winfo", "exists", listbox))
        for mode in ("light", "dark"):
            ui.apply_theme(self.root, mode)
            colors = ui.PALETTES[mode]
            self.assertEqual(str(self.root.tk.call(listbox, "cget", "-background")), colors["input"])
            self.assertEqual(str(self.root.tk.call(listbox, "cget", "-foreground")), colors["ink"])
            self.assertEqual(str(self.root.tk.call(listbox, "cget", "-selectbackground")), colors["accent"])
            self.assertEqual(str(self.root.tk.call(listbox, "cget", "-selectforeground")), colors["accent_ink"])
            self.assertEqual(str(self.root.tk.call("ttk::combobox::PopdownWindow", str(combo))), popup)
        ui.apply_theme(self.root, "light")
        fresh = ttk.Combobox(self.root, values=("C", "D"))
        fresh_popup = str(self.root.tk.call("ttk::combobox::PopdownWindow", str(fresh)))
        self.assertEqual(str(self.root.tk.call(fresh_popup+".f.l", "cget", "-background")),
                         ui.PALETTES["light"]["input"])

    def test_theme_button_changes_only_appearance_and_preserves_pending_work(self):
        ui.save_theme("dark")
        application = self.make_app()
        application.profiles[14].update(zero_offset=.4, reference_expos=.4, verified=True)
        app.save_profiles(self.config_path, application.profiles, simulation=False)
        saved = self.config_path.read_bytes()
        application.profiles[14]["notes"] = "尚未保存的修改"
        application.dirty = True
        application.fields["name"].set("尚未应用的名称")
        application._sync_target(5.5)
        profiles = copy.deepcopy(application.profiles)
        target = (application.target.get(), application.slider_value.get(), application.requested_deg)
        with patch.object(application.controller, "send", wraps=application.controller.send) as sent:
            for mode, label in (("light", "切换深色"), ("dark", "切换浅色")):
                application.theme_button.invoke()
                self.root.update_idletasks()
                self.assertEqual(self.root._theme_mode, mode)
                self.assertEqual(application.theme_button.cget("text"), label)
                self.assertEqual(ui.load_theme(), mode)
            sent.assert_not_called()
        self.assertEqual(application.profiles, profiles)
        self.assertEqual(self.config_path.read_bytes(), saved)
        self.assertTrue(application.dirty)
        self.assertEqual(application.fields["name"].get(), "尚未应用的名称")
        self.assertEqual((application.target.get(), application.slider_value.get(), application.requested_deg), target)
        self.assertEqual(application.log.cget("background"), ui.PALETTES["dark"]["surface"])

    def test_saved_light_mode_is_loaded_when_application_starts(self):
        ui.save_theme("light")
        application = self.make_app()
        self.assertEqual(self.root._theme_mode, "light")
        self.assertEqual(application.theme_button.cget("text"), "切换深色")
        self.assertEqual(application.log.cget("background"), ui.PALETTES["light"]["surface"])

    def test_open_guide_keeps_chapter_and_read_only_content_across_toggle(self):
        ui.save_theme("dark")
        application = self.make_app()
        window = show_guide(self.root)
        window.withdraw()
        navigation = next(w for w in descendants(window) if isinstance(w, ttk.Treeview))
        text = next(w for w in descendants(window) if isinstance(w, tk.Text))
        navigation.selection_set("3")
        navigation.event_generate("<<TreeviewSelect>>")
        content = text.get("1.0", "end")
        self.assertIn(SECTIONS[3][1], content)
        with patch.object(application.controller, "send", wraps=application.controller.send) as sent:
            application.theme_button.invoke()
            sent.assert_not_called()
        colors = ui.PALETTES["light"]
        self.assertEqual(navigation.selection(), ("3",))
        self.assertEqual(text.get("1.0", "end"), content)
        self.assertEqual(text.cget("state"), "disabled")
        self.assertEqual(text.cget("background"), colors["surface"])
        self.assertEqual(text.cget("foreground"), colors["ink"])
        self.assertEqual(text.tag_cget("title", "foreground"), colors["accent"])
        self.assertIs(show_guide(self.root), window)
        window.withdraw()
        self.assertEqual(navigation.selection(), ("3",))


class RendererThemeTests(unittest.TestCase):
    def test_background_round_trip_keeps_geometry_camera_and_physics_unchanged(self):
        import robot_view as view
        from test_robot_view import scene, tetrahedron

        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            directory = Path(folder)
            tetrahedron(directory / "part.stl")
            path = directory / "model.json"
            path.write_text(json.dumps(scene()), encoding="utf-8")
            model = view.load_scene(path)
            poses = view.component_poses(model, {"hip": .15})
            renderer = view._Renderer(model)
            try:
                renderer.camera.azimuth = 135
                dark = renderer.frame(poses)
                original = renderer.model, renderer.data, renderer.camera
                camera = (renderer.camera.azimuth, renderer.camera.elevation, renderer.camera.distance,
                          tuple(renderer.camera.lookat))
                positions = renderer.data.mocap_pos.copy()
                renderer.set_theme("light")
                light = renderer.frame(poses)
                self.assertGreater(light.max(), 0)
                self.assertGreater(float(light[:20, :20].mean()), float(dark[:20, :20].mean())+50)
                context = renderer.renderer
                renderer.set_theme("light")
                self.assertIs(renderer.renderer, context)
                renderer.set_theme("dark")
                restored = renderer.frame(poses)
                np.testing.assert_allclose(restored, dark, atol=1)
                self.assertIs(renderer.model, original[0])
                self.assertIs(renderer.data, original[1])
                self.assertIs(renderer.camera, original[2])
                self.assertEqual((renderer.camera.azimuth, renderer.camera.elevation, renderer.camera.distance,
                                  tuple(renderer.camera.lookat)), camera)
                np.testing.assert_allclose(renderer.data.mocap_pos, positions, atol=1e-12)
                self.assertEqual(renderer.data.time, 0)
            finally:
                renderer.close()


if __name__ == "__main__":
    unittest.main()
