"""Rounded ttk paint preserves native editing and bounded image lifetimes."""
import gc
from pathlib import Path
import sys
import tkinter as tk
from tkinter import ttk
import unittest
import weakref

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ui


def layout_elements(layout):
    for name, options in layout:
        yield name
        yield from layout_elements(options.get("children", ()))


def layout_nodes(layout):
    for name, options in layout:
        yield name, options
        yield from layout_nodes(options.get("children", ()))


class RoundedThemeTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.errors = []
        self.root.report_callback_exception = lambda *error: self.errors.append(error)
        self.addCleanup(self.cleanup)

    def cleanup(self):
        try:
            if self.root.winfo_exists():
                self.root.destroy()
        except tk.TclError:
            pass
        self.assertEqual(self.errors, [])

    def resources(self, root):
        return (frozenset(root.tk.call("image", "names")),
                frozenset(ttk.Style(root).element_names()))

    def test_repeated_theme_changes_reuse_images_and_elements(self):
        for mode in ("dark", "light"):
            ui.apply_theme(self.root, mode)
        manager = self.root._rounded_theme
        resources = self.resources(self.root)
        cache_sizes = len(manager.images), len(manager.elements)
        self.assertGreater(cache_sizes[0], 0)
        self.assertGreater(cache_sizes[1], 0)

        for mode in ("dark", "light") * 6:
            ui.apply_theme(self.root, mode)
            self.root.update_idletasks()
            self.assertIs(self.root._rounded_theme, manager)
            self.assertEqual(self.resources(self.root), resources)
            self.assertEqual((len(manager.images), len(manager.elements)), cache_sizes)

    def test_separate_roots_own_their_images_and_active_layout(self):
        ui.apply_theme(self.root, "light")
        first = self.root._rounded_theme
        resources = self.resources(self.root)
        first_layout = ttk.Style(self.root).layout("TEntry")
        other = tk.Tk()
        other.withdraw()
        other.report_callback_exception = lambda *error: self.errors.append(error)
        try:
            ui.apply_theme(other, "dark")
            second = other._rounded_theme
            self.assertIsNot(first, second)
            self.assertTrue(all(photo.tk is self.root.tk for photo in first.images.values()))
            self.assertTrue(all(photo.tk is other.tk for photo in second.images.values()))
            ui.apply_theme(other, "light")
            self.assertEqual(self.resources(self.root), resources)
            self.assertEqual(ttk.Style(self.root).layout("TEntry"), first_layout)
            ui.apply_theme(other, "dark")
            self.assertNotEqual(ttk.Style(other).layout("TEntry"), first_layout)
        finally:
            other.destroy()
        self.assertTrue(first.images)
        self.assertEqual(second.images, {})

    def test_root_destroy_releases_cached_image_objects(self):
        for mode in ("dark", "light"):
            ui.apply_theme(self.root, mode)
        manager = self.root._rounded_theme
        photos = [weakref.ref(photo) for photo in manager.images.values()]
        self.root.destroy()
        gc.collect()
        self.assertEqual(manager.images, {})
        self.assertEqual(manager.elements, {})
        self.assertIsNone(manager.root)
        self.assertTrue(all(photo() is None for photo in photos))

    def test_button_and_badge_backgrounds_draw_before_their_text(self):
        # Tk's layout border flag paints a node after its children. An opaque
        # replacement background with that flag hides the label completely.
        for mode in ("dark", "light"):
            ui.apply_theme(self.root, mode)
            style = ttk.Style(self.root)
            names = [context + tone + ".TButton"
                     for context in ("", "Page.") for tone in ui.PALETTES[mode]["buttons"]]
            names += [name + ".Badge.TLabel" for name in ui.PALETTES[mode]["badges"]]
            for name in names:
                with self.subTest(mode=mode, style=name):
                    layout = style.layout(name)
                    text_element = "Button.label" if name.endswith("TButton") else "Label.label"
                    backgrounds = [(element, options) for element, options in layout_nodes(layout)
                                   if element.startswith("Rounded.") and options.get("children")]
                    self.assertTrue(backgrounds)
                    for _element, options in backgrounds:
                        self.assertFalse(self.root.tk.getboolean(options.get("border", False)))
                        self.assertIn(text_element, set(layout_elements(options["children"])))

    def test_native_entry_and_combobox_editing_and_selection_survive_theme_changes(self):
        ui.apply_theme(self.root, "dark")
        entry = ttk.Entry(self.root)
        combo = ttk.Combobox(self.root, values=("未绑定", "髋关节", "膝关节"), state="readonly")
        entry_bindings = entry.bindtags(), entry.bind_class("TEntry", "<KeyPress>")
        combo_bindings = combo.bindtags(), combo.bind_class("TCombobox", "<Button-1>")
        self.assertIn("TEntry", entry_bindings[0])
        self.assertIn("TCombobox", combo_bindings[0])
        self.assertIn("ttk::combobox::Press", combo_bindings[1])
        entry.insert(0, "12.34")
        entry.selection_range(0, 2)
        entry.icursor("end")
        selected = []
        combo.bind("<<ComboboxSelected>>", lambda _event: selected.append(combo.get()))

        for mode in ("light", "dark", "light"):
            ui.apply_theme(self.root, mode)
            self.assertEqual(entry.get(), "12.34")
            self.assertTrue(entry.selection_present())
            self.assertEqual((entry.index("sel.first"), entry.index("sel.last")), (0, 2))
            self.assertEqual((entry.bindtags(), entry.bind_class("TEntry", "<KeyPress>")), entry_bindings)
            self.assertEqual((combo.bindtags(), combo.bind_class("TCombobox", "<Button-1>")), combo_bindings)
            entry_layout = set(layout_elements(ttk.Style(self.root).layout("TEntry")))
            combo_layout = set(layout_elements(ttk.Style(self.root).layout("TCombobox")))
            self.assertTrue({"Entry.padding", "Entry.textarea"}.issubset(entry_layout))
            self.assertTrue({"Combobox.padding", "Combobox.textarea"}.issubset(combo_layout))
            self.assertTrue(any(name.endswith("Combobox.downarrow") for name in combo_layout))
            self.assertTrue(combo.instate(["readonly", "!disabled"]))

        # Use the same Tcl edit/selection paths as the native class bindings,
        # without changing OS focus, opening a popup, or creating an app/controller.
        entry.tk.call("ttk::entry::Insert", str(entry), "9")
        self.assertEqual(entry.get(), "9.34")
        combo.tk.call("ttk::combobox::SelectEntry", str(combo), 2)
        self.root.update()
        self.assertEqual(selected, ["膝关节"])
        combo.insert(0, "不可直接输入")
        self.assertEqual(combo.get(), "膝关节")
        entry.state(["disabled"])
        combo.state(["disabled"])
        ui.apply_theme(self.root, "dark")
        entry.delete(0, "end")
        entry.insert(0, "不可修改")
        self.assertEqual(entry.get(), "9.34")
        self.assertTrue(entry.instate(["disabled"]))
        self.assertTrue(combo.instate(["disabled", "readonly"]))


if __name__ == "__main__":
    unittest.main()
