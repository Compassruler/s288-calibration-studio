"""圆角外观：只替换 ttk 绘制元素，保留原控件、布局和键鼠行为。"""
from copy import deepcopy
from tkinter import ttk
from PIL import Image, ImageDraw, ImageTk


CARD_RADIUS = 14
CONTROL_RADIUS = 8
TILE_SIZE = 64  # ttk 会平铺图片中心，使用较大色块避免全屏时产生大量细小绘制。


def _replace(layout, replacements):
    result = []
    for name, options in deepcopy(layout):
        if name in replacements:
            options.pop("border", None)  # 图片先绘制，避免盖住后面的按钮/标签文字。
        if "children" in options:
            options["children"] = _replace(options["children"], replacements)
        if name == "Button.focus":
            # 焦点已由圆角状态图片绘制，去掉 clam 的矩形焦点框。
            result.extend(options["children"])
        else:
            result.append((replacements.get(name, name), options))
    return result


class RoundedTheme:
    def __init__(self, root):
        self.root, self.style = root, ttk.Style(root)
        self.layouts = {name: self.style.layout(name)
                        for name in ("TButton", "TEntry", "TCombobox", "TLabel")}
        self.images, self.elements = {}, {}
        root.bind("<Destroy>", self._close, add="+")

    def _image(self, outer, fill, border, radius):
        key = outer, fill, border, radius
        if key not in self.images:
            # 小图三倍采样后缩小，九切片只拉伸直边，圆角不随窗口变形。
            size, factor = (radius + 1) * 2 + TILE_SIZE, 3
            image = Image.new("RGB", (size * factor, size * factor), outer)
            draw = ImageDraw.Draw(image)
            draw.rounded_rectangle((factor, factor, (size - 1) * factor, (size - 1) * factor),
                                   radius * factor, fill=fill, outline=border, width=factor)
            image = image.resize((size, size), Image.Resampling.LANCZOS)
            self.images[key] = ImageTk.PhotoImage(image, master=self.root)
        return self.images[key]

    def _element(self, name, outer, radius, default, states=()):
        if name not in self.elements:
            normal = self._image(outer, *default, radius)
            variants = [(state, self._image(outer, *colors, radius)) for state, colors in states]
            # border 只定义九切片；padding=0 避免圆角额外撑大最小窗口。
            self.style.element_create(name, "image", normal, *variants,
                                      border=radius + 1, padding=0, width=0, height=0,
                                      sticky="nswe")
            self.elements[name] = name
        return name

    def _slider(self, prefix, colors):
        track, thumb = prefix + "Scale.trough", prefix + "Scale.slider"
        if track not in self.elements:
            image = Image.new("RGB", (18, 6), colors["surface"])
            ImageDraw.Draw(image).rounded_rectangle((0, 0, 17, 5), radius=3, fill=colors["track"])
            self.images[track] = ImageTk.PhotoImage(image, master=self.root)
            self.style.element_create(track, "image", self.images[track], border=3,
                                      padding=0, width=0, height=0, sticky="we")
            variants = []
            for state, fill in (("normal", colors["accent"]), ("disabled", colors["scale_disabled"]),
                                ("focus", colors["accent"])):
                image = Image.new("RGBA", (60, 60))
                draw = ImageDraw.Draw(image)
                if state == "focus":
                    draw.ellipse((1, 1, 58, 58), outline=fill, width=3)
                draw.ellipse((9, 9, 50, 50), fill=fill)
                photo = ImageTk.PhotoImage(image.resize((20, 20), Image.Resampling.LANCZOS), master=self.root)
                self.images[thumb, state] = photo
                variants.append(photo)
            self.style.element_create(thumb, "image", variants[0], ("disabled", variants[1]),
                                      ("focus", variants[2]), width=20, height=20, sticky="")
            self.elements[track], self.elements[thumb] = track, thumb
        return track, thumb

    def _arrow(self, name, colors):
        if name not in self.elements:
            variants = []
            for color in (colors["muted"], colors["disabled_ink"]):
                key = "arrow", color
                if key not in self.images:
                    image = Image.new("RGBA", (48, 48))
                    ImageDraw.Draw(image).line(((12, 18), (24, 30), (36, 18)), fill=color, width=5)
                    self.images[key] = ImageTk.PhotoImage(
                        image.resize((16, 16), Image.Resampling.LANCZOS), master=self.root)
                variants.append(self.images[key])
            self.style.element_create(name, "image", variants[0], ("disabled", variants[1]),
                                      padding=0, width=20, height=0, sticky="")
            self.elements[name] = name
        return name

    def apply(self, mode, colors):
        bg, surface, border, accent = (colors[key] for key in ("bg", "surface", "border", "accent"))
        disabled, fault = colors["disabled"], colors["badges"]["Fault"]
        prefix = "Rounded." + mode + "."
        card = self._element(prefix + "card", bg, CARD_RADIUS, (surface, border))
        self.style.layout("Card.TFrame", [(card, {"sticky": "nswe"})])
        viewport = self._element(prefix + "viewport", surface, 12, (bg, border))
        self.style.configure("Viewport.TFrame", background=surface)
        self.style.layout("Viewport.TFrame", [(viewport, {"sticky": "nswe"})])

        for tone, (normal, hover, _pressed, foreground) in colors["buttons"].items():
            outline = border if tone == "Quiet" else normal
            for context, outer in (("", surface), ("Page.", bg)):
                face = self._element(prefix + context + tone + ".button", outer, CONTROL_RADIUS,
                    (normal, outline), (
                        ("invalid", (fault[0], fault[1])),
                        ("disabled", (disabled, disabled)),
                        ("alternate", (accent, accent)),
                        ("pressed", (accent, accent)),
                        ("active focus", (hover, foreground if tone in {"Danger", "Release"} else accent)),
                        ("focus", (normal, foreground if tone in {"Danger", "Release"} else accent)),
                        ("active", (hover, border if tone == "Quiet" else hover)),
                    ))
                layout = _replace(self.layouts["TButton"], {"Button.border": face})
                name = context + tone + ".TButton"
                self.style.layout(name, layout)
                if not context and tone == "Neutral":
                    self.style.layout("TButton", layout)

        for widget in ("Entry", "Combobox"):
            face = self._element(prefix + widget + ".field", surface, CONTROL_RADIUS,
                (colors["input"], border), (("disabled", (disabled, border)),
                    ("focus", (colors["input"], accent))))
            if widget == "Entry":
                layout = _replace(self.layouts["TEntry"], {"Entry.field": face})
            else:
                # clam 原布局把箭头放在外框旁边；包进同一圆角外框，保留 downarrow 命中区。
                arrow = self._arrow(prefix + "Combobox.downarrow", colors)
                contents = self.layouts["TCombobox"][1][1]["children"]
                layout = [(face, {"sticky": "nswe", "children": [
                    (arrow, {"side": "right", "sticky": "ns"}), *deepcopy(contents)]})]
            self.style.layout("T" + widget, layout)

        for name, (fill, _foreground) in colors["badges"].items():
            face = self._element(prefix + name + ".badge", surface, 12, (fill, fill))
            self.style.layout(name + ".Badge.TLabel",
                _replace(self.layouts["TLabel"], {"Label.border": face}))

        track, thumb = self._slider(prefix, colors)
        self.style.configure("Angle.Horizontal.TScale", background=surface, bordercolor=surface,
                             lightcolor=surface, darkcolor=surface)
        self.style.map("Angle.Horizontal.TScale",
                       **{option: [] for option in ("background", "bordercolor", "lightcolor", "darkcolor")})
        self.style.layout("Angle.Horizontal.TScale", [(track, {"sticky": "nswe", "children": [
            (thumb, {"side": "left", "sticky": ""})]})])

    def _close(self, event):
        if event.widget is self.root:
            # PhotoImage 必须在所属窗口的主线程里释放；不同 Tk 窗口不共享缓存。
            self.images.clear()
            self.elements.clear()
            self.root = None


def apply_rounding(root, mode, colors):
    if not hasattr(root, "_rounded_theme"):
        root._rounded_theme = RoundedTheme(root)
    root._rounded_theme.apply(mode, colors)
