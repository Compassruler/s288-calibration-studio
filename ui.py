"""主界面布局、配色和宽屏适配；这里不执行舵机通信。"""
import json
import os
from pathlib import Path
import tempfile
import tkinter as tk
from tkinter import font as tkfont, ttk
from guide import show_guide
from rounded_theme import apply_rounding

THEME_PATH = Path(__file__).resolve().parent / "configs" / "appearance.json"
BODY_FONT = ("Microsoft YaHei UI", 12)
# 常改的显示参数集中在这里；进入/退出门槛不同，防止窗口临界尺寸反复切换。
DEFAULT_WINDOW_SIZE = (1720, 1080)
MIN_WINDOW_SIZE = (1360, 940)
WIDE_ENTER_SIZE = (1900, 1220)
WIDE_EXIT_SIZE = (1840, 1180)
WIDE_SCALE = 1.25
WIDE_SPACING_SCALE = 1.6
RESIZE_DELAY_MS = 120
BUTTON_FLASH_MS = 450  # 点击后至少保留一次可见反馈，不延迟原操作。
ACTION_STYLES = {tone: f"Action{tone.title()}.TLabel" for tone in ("info", "success", "warning", "error")}
PALETTES = {
    "dark": {
        "bg": "#0d1113", "surface": "#151c1f", "ink": "#e2e8e8", "muted": "#9aa9a0",
        "accent": "#bff575", "accent_ink": "#17200f", "border": "#283135", "input": "#10191c",
        "sky": None, "highlight": (.75, .96, .46, 1),
        "readout": "#d9e8ce", "selection": "#2e412b", "selection_ink": "#d4f7ac",
        "disabled": "#202a2d", "disabled_ink": "#83918a", "scroll": "#405249",
        "track": "#33433b", "check_border": "#5b6c61", "scale_disabled": "#596b5e",
        "buttons": {
            "Primary": ("#bff575", "#d0ff93", "#a8dc64", "#17200f"),
            "Soft": ("#293b2c", "#334932", "#3c5638", "#cbe7b5"),
            "Neutral": ("#283438", "#324145", "#3c4d51", "#e0e8e0"),
            "Quiet": ("#151c1f", "#223034", "#2d3d40", "#c0cdc3"),
            "Danger": ("#3b292f", "#4b3038", "#5d3741", "#f4b4b9"),
            "Release": ("#35292d", "#442f35", "#55353e", "#edb5bd"),
        },
        "badges": {"Idle": ("#222d30", "#a4b1a9"), "Live": ("#273b2c", "#c7f594"),
                   "Demo": ("#343426", "#e2daa1"), "Fault": ("#41282e", "#f5b1b5")},
    },
    "light": {
        "bg": "#f1f5f2", "surface": "#ffffff", "ink": "#24332b", "muted": "#5e7065",
        "accent": "#3f7b22", "accent_ink": "#ffffff", "border": "#d8e2db", "input": "#f6f9f7",
        "sky": (236, 242, 238), "highlight": (.26, .53, .12, 1),
        "readout": "#35582a", "selection": "#e1efd8", "selection_ink": "#305824",
        "disabled": "#edf1ee", "disabled_ink": "#7a8980", "scroll": "#b5c5b9",
        "track": "#dbe7da", "check_border": "#8faaa0", "scale_disabled": "#b9c8bc",
        "buttons": {
            "Primary": ("#3f7b22", "#4b8b2b", "#32671b", "#ffffff"),
            "Soft": ("#e5f0dc", "#d8e8cb", "#c7dfb6", "#365c27"),
            "Neutral": ("#e9efeb", "#dfe8e2", "#d0ddd4", "#2c4033"),
            "Quiet": ("#ffffff", "#f0f5f1", "#e3ece6", "#465f50"),
            "Danger": ("#fbe8ec", "#f6d9e0", "#eec4ce", "#a13b50"),
            "Release": ("#f8edef", "#f2e0e5", "#ead0d8", "#934456"),
        },
        "badges": {"Idle": ("#e9efeb", "#5b7062"), "Live": ("#e1f0d9", "#3b642b"),
                   "Demo": ("#f7efd5", "#7a631c"), "Fault": ("#fbe2e7", "#a13b50")},
    },
}


def load_theme():
    try:
        document = json.loads(THEME_PATH.read_text(encoding="utf-8"))
        mode = document.get("theme") if isinstance(document, dict) else None
        return mode if isinstance(mode, str) and mode in PALETTES else "dark"
    except (OSError, ValueError):
        return "dark"


def save_theme(mode):
    """Keep appearance separate from calibration and model bindings."""
    if mode not in PALETTES:
        raise ValueError("未知的界面主题")
    THEME_PATH.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=THEME_PATH.parent,
                                     prefix="appearance_", suffix=".tmp", delete=False) as stream:
        temporary = Path(stream.name)
        json.dump({"theme": mode}, stream)
        stream.write("\n")
    try:
        os.replace(temporary, THEME_PATH)
    finally:
        temporary.unlink(missing_ok=True)


def theme_colors(widget):
    return PALETTES[getattr(widget._root(), "_theme_mode", "dark")]


def theme_widget(widget, *, tags=None, **roles):
    """Remember semantic colors for widgets not fully controlled by ttk styles."""
    widget._theme_roles = roles
    widget._theme_tags = tags or {}
    _paint_widget(widget, theme_colors(widget))
    return widget


def _paint_widget(widget, colors):
    roles = getattr(widget, "_theme_roles", {})
    if roles:
        widget.configure(**{option: colors[role] for option, role in roles.items()})
    for tag, roles in getattr(widget, "_theme_tags", {}).items():
        widget.tag_configure(tag, **{option: colors[role] for option, role in roles.items()})
    if isinstance(widget, ttk.Combobox):
        # Tk caches these listboxes; option_add alone only affects new popdowns.
        listbox = f"{widget}.popdown.f.l"
        if widget.tk.call("winfo", "exists", listbox):
            widget.tk.call(listbox, "configure", "-background", colors["input"],
                           "-foreground", colors["ink"], "-selectbackground", colors["accent"],
                           "-selectforeground", colors["accent_ink"])


def _paint_children(widget, colors):
    _paint_widget(widget, colors)
    for child in widget.children.values():
        _paint_children(child, colors)


def update_text(variable, value):
    if variable.get() != value:
        variable.set(value)


def update_options(widget, **values):
    changed = {}
    for key, value in values.items():
        if key == "style":
            value = workspace_style(widget, value)
        current = widget.cget(key)
        if current != value and str(current) != str(value):
            changed[key] = value
    if changed:
        widget.configure(**changed)


def workspace_style(widget, name):
    if getattr(widget, "_workspace_member", False):
        return "Workspace." + name.removeprefix("Workspace.")
    return name


def show_action(root, message, tone="info"):
    """按钮和子窗口共用提示入口；弱引用不会延长应用/窗口的生命周期。"""
    reference = getattr(root, "_action_feedback", None)
    callback = reference() if reference else None
    if callback:
        callback(message, tone)


class FeedbackButton(ttk.Button):
    """统一鼠标、空格及 invoke 的反馈；原命令始终只执行一次。"""

    def __init__(self, parent, *, command=None, **options):
        super().__init__(parent, command=self._invoke, **options)
        self._action, self._flash_id = command, None
        self._closed = False  # 操作本身可能关闭窗口，之后不能再登记 Tk 回调。
        self.bind("<ButtonPress-1>", self._disabled_press, add="+")
        self.bind("<KeyPress-space>", self._disabled_press, add="+")
        self.bind("<Destroy>", self._destroyed, add="+")

    def _flash(self, rejected=False):
        if self._flash_id is not None:
            self._root().after_cancel(self._flash_id)
            self._flash_id = None
        self.state(["!alternate", "invalid"] if rejected else ["!invalid", "alternate"])

    def _finish_flash(self):
        if not self._closed:
            self._flash_id = self._root().after(BUTTON_FLASH_MS, self._clear_flash)

    def _clear_flash(self):
        self._flash_id = None
        self.state(["!alternate", "!invalid"])

    def _disabled_press(self, _event):
        if self.instate(["disabled"]):
            show_action(self._root(), f"当前不可用：{self.cget('text')}。", "warning")
            self._flash(rejected=True)
            self._finish_flash()

    def _invoke(self):
        self._flash()
        show_action(self._root(), f"已收到：{self.cget('text')}。")
        try:
            # 不使用 after/update 包装控制命令，避免延迟急停或重复发目标。
            return self._action() if self._action else None
        finally:
            self._finish_flash()

    def _destroyed(self, event):
        if event.widget is self:
            self._closed = True
            if self._flash_id is not None:
                self._root().after_cancel(self._flash_id)
                self._flash_id = None
            self._action = None


class ResponsiveLayout:
    """窗口稳定后切换两档密度；只更新字体和间距，不重建控件。"""

    def __init__(self, app, workspace, status_frames):
        self.app, self.root, self.workspace = app, app.root, workspace
        self.style = ttk.Style(self.root)
        self.status_frames = status_frames
        self.expanded, self.scale, self.after_id = False, 1.0, None
        self.fonts, self.styles = {}, set()
        self.columns = {key: app.tree.column(key, "width") for key in app.tree["columns"]}
        self.spacing = []
        for panel, manager in ((app.panels["control"], "pack"), (app.panels["settings"], "grid")):
            for widget in panel.winfo_children():
                info = getattr(widget, manager + "_info")()
                value = info.get("pady", 0)
                self.spacing.append((widget, manager, tuple(map(int, value if isinstance(value, tuple) else (value,)))))
        self.styles.update(name + ".Badge.TLabel" for name in PALETTES["dark"]["badges"])
        self.styles.update(ACTION_STYLES.values())
        self.register(self.root)
        self.root.bind("<Configure>", self._resized, add="+")
        self.root.bind("<Destroy>", self._destroyed, add="+")

    def _font(self, specification):
        # 复用命名字体，避免每次缩放产生新 Tk 资源。
        key = str(specification)
        if key not in self.fonts:
            font = tkfont.Font(root=self.root, font=specification)
            size = int(font.cget("size"))
            font.configure(size=round(size * self.scale))
            self.fonts[key] = font, size
        return self.fonts[key][0]

    def register(self, parent):
        """Keep help and binding dialogs at their own size; no controls are rebuilt."""
        pending = list(parent.winfo_children())
        while pending:
            widget = pending.pop()
            if widget.winfo_toplevel() is widget:
                continue
            if not getattr(widget, "_workspace_member", False):
                widget._workspace_member = True
                if isinstance(widget, ttk.Widget):
                    name = str(widget.cget("style")) or widget.winfo_class()
                    self.styles.add(name)
                    if isinstance(widget, ttk.Treeview):
                        self.styles.add(name + ".Heading")
                    widget.configure(style=workspace_style(widget, name))
                if "font" in widget.keys() and widget.cget("font"):
                    font = self._font(widget.cget("font"))
                    widget.configure(font=font)
                    if isinstance(widget, ttk.Combobox):
                        widget.tk.call("ttk::combobox::PopdownWindow", str(widget))
                        widget.tk.call(f"{widget}.popdown.f.l", "configure", "-font", font)
            pending.extend(widget.winfo_children())
        self._style_density()

    def _style_density(self):
        for name in self.styles:
            options = {"font": self._font(self.style.lookup(name, "font") or BODY_FONT)}
            if name.endswith("TButton"):
                options["padding"] = (round(10 * self.scale), round(6 * self.scale))
            elif name.endswith("TEntry"):
                options["padding"] = (round(8 * self.scale), round(5 * self.scale))
            elif name.endswith("TCombobox"):
                options["padding"] = (round(7 * self.scale), round(5 * self.scale))
                options["arrowsize"] = round(13 * self.scale)
            if name == "Sidebar.Treeview":
                options["rowheight"] = round(40 * self.scale)
            self.style.configure("Workspace." + name, **options)

    def _resized(self, event):
        if event.widget is not self.root or self.app is None or self.app.closing:
            return
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
        self.after_id = self.root.after(RESIZE_DELAY_MS, self._settle)

    def _settle(self):
        self.after_id = None
        if self.app is None or self.app.closing:
            return
        width, height = self.root.winfo_width(), self.root.winfo_height()
        minimum = WIDE_EXIT_SIZE if self.expanded else WIDE_ENTER_SIZE
        expanded = width >= minimum[0] and height >= minimum[1]
        if expanded == self.expanded:
            return
        self.expanded, self.scale = expanded, WIDE_SCALE if expanded else 1.0
        for font, size in tuple(self.fonts.values()):
            font.configure(size=round(size * self.scale))
        self._style_density()
        for widget, manager, padding in self.spacing:
            factor = WIDE_SPACING_SCALE if expanded else 1
            getattr(widget, manager + "_configure")(pady=tuple(round(p * factor) for p in padding))
        height = self._font(self.style.lookup("Muted.TLabel", "font")).metrics("linespace") * 2
        for frame in self.status_frames:
            frame.configure(height=height)
        for key, width in self.columns.items():
            self.app.tree.column(key, width=round(width * self.scale))
        self.workspace.rowconfigure(2, weight=3 if expanded else 1)
        self.workspace.rowconfigure(5, weight=1 if expanded else 0)

    def _destroyed(self, event):
        if event.widget is not self.root:
            return
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
            self.after_id = None
        # Release Tk objects here on the UI thread, including the app back-reference.
        self.fonts.clear()
        self.spacing.clear()
        self.status_frames = ()
        self.app = None


def apply_theme(root, mode="dark"):
    """Apply colors once on request, without rebuilding widgets or polling."""
    colors = PALETTES[mode]
    root._theme_mode = mode
    bg, surface, ink, muted, accent, border, field = (
        colors[key] for key in ("bg", "surface", "ink", "muted", "accent", "border", "input"))
    disabled, disabled_ink = colors["disabled"], colors["disabled_ink"]
    style = ttk.Style(root)
    if style.theme_use() != "clam":
        style.theme_use("clam")
    style.configure(".", font=BODY_FONT, background=bg, foreground=ink)
    style.configure("TFrame", background=bg)
    style.configure("TLabel", background=bg, foreground=ink)
    style.configure("Card.TFrame", background=surface, padding=18, borderwidth=1,
                    relief="solid", bordercolor=border, lightcolor=border, darkcolor=border)
    style.layout("Card.TFrame", [("Frame.border", {"sticky": "nswe"})])
    for name, foreground, font in (
        ("Card", ink, BODY_FONT),
        ("Muted", muted, BODY_FONT),
        ("Section", ink, ("Microsoft YaHei UI", 15, "bold")),
        ("Eyebrow", muted, ("Segoe UI", 10, "bold")),
        ("Big", ink, ("Segoe UI", 40, "bold")),
        ("Metric", ink, ("Segoe UI", 20, "bold")),
        ("Readout", colors["readout"], ("Cascadia Mono", 14, "bold")),
    ):
        style.configure(name+".TLabel", background=surface, foreground=foreground, font=font)
    style.configure("Accent.TLabel", background=surface, foreground=accent)
    style.configure("Plain.TFrame", background=surface)
    for name, (badge_bg, badge_ink) in colors["badges"].items():
        style.configure(name+".Badge.TLabel", background=badge_bg, foreground=badge_ink,
                        padding=(12, 6), font=(*BODY_FONT, "bold"))
    for tone, foreground in (("info", ink), ("success", accent),
                             ("warning", colors["badges"]["Demo"][1]), ("error", colors["badges"]["Fault"][1])):
        style.configure(ACTION_STYLES[tone], background=bg, foreground=foreground, font=(*BODY_FONT, "bold"))
    for name, (normal, hover, pressed, foreground) in colors["buttons"].items():
        button_style = name+".TButton"
        outline = border if name == "Quiet" else normal
        style.layout(button_style, style.layout("TButton"))
        style.configure(button_style, foreground=foreground, background=normal, padding=(10, 6),
                        font=BODY_FONT, borderwidth=1, relief="flat",
                        bordercolor=outline, lightcolor=normal, darkcolor=normal,
                        focuscolor=foreground if name in {"Danger", "Release"} else accent,
                        focusthickness=1, focussolid=True)
        backgrounds = [("invalid", colors["badges"]["Fault"][0]), ("disabled", disabled),
                       ("alternate", accent), ("pressed", accent), ("active", hover)]
        style.map(button_style, background=backgrounds, lightcolor=backgrounds, darkcolor=backgrounds,
                  foreground=[("invalid", colors["badges"]["Fault"][1]), ("disabled", disabled_ink),
                              ("alternate", colors["accent_ink"]), ("pressed", colors["accent_ink"])],
                  bordercolor=[("invalid", colors["badges"]["Fault"][1]), ("disabled", border),
                               ("alternate", accent), ("pressed", accent), ("active", border if name == "Quiet" else hover)])
    style.configure("TEntry", padding=(8, 5), fieldbackground=field, foreground=ink,
                    borderwidth=1, relief="flat", bordercolor=border, lightcolor=border,
                    darkcolor=border, insertcolor=ink, selectbackground=accent,
                    selectforeground=colors["accent_ink"])
    style.map("TEntry", bordercolor=[("focus", accent)], lightcolor=[("focus", accent)],
              darkcolor=[("focus", accent)], foreground=[("disabled", disabled_ink)],
              fieldbackground=[("disabled", disabled)])
    style.configure("TCombobox", padding=(7, 5), arrowsize=13, arrowcolor=muted,
                    fieldbackground=field, background=field, bordercolor=border,
                    lightcolor=border, darkcolor=border, borderwidth=1)
    style.map("TCombobox", fieldbackground=[("disabled", disabled), ("readonly", field)],
              foreground=[("disabled", disabled_ink), ("readonly", ink)],
              selectbackground=[("readonly", field)], selectforeground=[("readonly", ink)])
    for option, color in (("background", field), ("foreground", ink),
                          ("selectBackground", accent), ("selectForeground", colors["accent_ink"])):
        root.option_add("*TCombobox*Listbox."+option, color)
    root.option_add("*TCombobox*Listbox.font", BODY_FONT)
    style.configure("TCheckbutton", background=surface, foreground=muted, padding=(2, 3),
                    indicatorbackground=field, indicatorforeground=colors["accent_ink"],
                    upperbordercolor=colors["check_border"], lowerbordercolor=colors["check_border"],
                    focuscolor=accent, focusthickness=1, focussolid=True)
    style.map("TCheckbutton", foreground=[("disabled", disabled_ink)], background=[("active", surface)],
              indicatorbackground=[("disabled", disabled), ("selected", accent)],
              indicatorforeground=[("disabled", disabled_ink)],
              upperbordercolor=[("disabled", border), ("selected", accent)],
              lowerbordercolor=[("disabled", border), ("selected", accent)])
    style.configure("Sidebar.Treeview", background=surface, fieldbackground=surface,
                    foreground=ink, rowheight=40, borderwidth=0, relief="flat", font=BODY_FONT)
    style.layout("Sidebar.Treeview", [("Treeview.treearea", {"sticky": "nswe"})])
    style.map("Sidebar.Treeview", background=[("selected", colors["selection"])],
              foreground=[("selected", colors["selection_ink"])])
    style.configure("Sidebar.Treeview.Heading", background=surface, foreground=muted,
                    borderwidth=0, relief="flat", padding=(4, 9), font=("Microsoft YaHei UI", 10))
    style.map("Sidebar.Treeview.Heading", background=[("active", surface)])
    style.configure("Side.Vertical.TScrollbar", troughcolor=surface, background=colors["scroll"],
                    bordercolor=surface, lightcolor=surface, darkcolor=surface,
                    arrowcolor=muted, arrowsize=9, width=9)
    style.layout("Side.Vertical.TScrollbar", [("Vertical.Scrollbar.trough", {"sticky": "ns",
        "children": [("Vertical.Scrollbar.thumb", {"sticky": "nswe", "expand": 1})]})])
    style.configure("Side.Horizontal.TProgressbar", troughcolor=colors["track"],
                    background=accent, borderwidth=0, thickness=4,
                    lightcolor=surface, darkcolor=surface, bordercolor=surface)
    style.layout("Angle.Horizontal.TScale", style.layout("Horizontal.TScale"))
    style.configure("Angle.Horizontal.TScale", background=accent, troughcolor=colors["track"],
                    bordercolor=accent, lightcolor=accent, darkcolor=accent, borderwidth=1,
                    sliderlength=24, gripsize=0, focuscolor=accent, focusthickness=1, focussolid=True)
    for option in ("background", "bordercolor", "lightcolor", "darkcolor"):
        style.map("Angle.Horizontal.TScale", **{option: [("disabled", colors["scale_disabled"])]})
    style.configure("TSeparator", background=border)
    style.configure("Treeview", background=field, fieldbackground=field, foreground=ink,
                    rowheight=27, bordercolor=border)
    style.map("Treeview", background=[("selected", colors["selection"])],
              foreground=[("selected", colors["selection_ink"])])
    style.configure("Treeview.Heading", background=surface, foreground=muted, bordercolor=border)
    style.map("Treeview.Heading", background=[("active", colors["buttons"]["Quiet"][1])])
    style.configure("TScrollbar", troughcolor=field, background=colors["scroll"], bordercolor=border,
                    lightcolor=colors["scroll"], darkcolor=colors["scroll"], arrowcolor=muted)
    apply_rounding(root, mode, colors)
    theme_widget(root, background="bg")
    _paint_children(root, colors)
    return style


def toggle_theme(app):
    mode = "light" if app.root._theme_mode == "dark" else "dark"
    apply_theme(app.root, mode)
    app.theme_button.configure(text="切换深色" if mode == "light" else "切换浅色")
    if app.robot_view is not None:
        try:
            app.robot_view.set_theme()
        except Exception as exc:
            app._log(f"外观已切换，三维预览更新失败：{exc}")
    try:
        save_theme(mode)
    except OSError as exc:
        app._log(f"外观已切换，本次选择未能保存：{exc}")
        show_action(app.root, f"外观已切换，但偏好未能保存：{exc}", "warning")
    else:
        show_action(app.root, "已切换为浅色模式。" if mode == "light" else "已切换为深色模式。", "success")


def build_ui(self):
    root = self.root
    root.title("S288 Studio · 软件零位标定")
    # Size the workspace to the display without changing the user's DPI settings.
    screen_width, screen_height = root.winfo_screenwidth(), root.winfo_screenheight()
    width = min(DEFAULT_WINDOW_SIZE[0], int(screen_width * .91))
    height = min(DEFAULT_WINDOW_SIZE[1], screen_height - 100)
    root.geometry(f"{width}x{height}+{max(0, (screen_width-width)//2)}+{max(0, (screen_height-height-60)//2)}")
    root.minsize(min(MIN_WINDOW_SIZE[0], screen_width - 48), min(MIN_WINDOW_SIZE[1], screen_height - 100))
    style = apply_theme(root, load_theme())
    status_height = tkfont.Font(root=root, font=style.lookup("Muted.TLabel", "font")).metrics("linespace") * 2
    self.compact_robot_view = True
    self.central_robot_view = True
    root.columnconfigure(1, weight=1)
    root.rowconfigure(0, weight=1)
    self.panels = {}

    sidebar = ttk.Frame(root, style="Plain.TFrame", padding=(16, 18, 12, 14))
    sidebar.grid(row=0, column=0, sticky="nsew")
    sidebar.columnconfigure(0, weight=1)
    sidebar.rowconfigure(2, weight=1)
    brand = ttk.Frame(sidebar, style="Plain.TFrame")
    brand.grid(row=0, column=0, sticky="we", pady=(0, 14))
    ttk.Label(brand, text="S288", style="Accent.TLabel",
              font=("Segoe UI", 28, "bold")).pack(anchor="w")
    ttk.Label(brand, text="S288 · CALIBRATION", style="Muted.TLabel",
              font=("Segoe UI", 9)).pack(anchor="w")
    ttk.Label(sidebar, text="关节通道", style="Section.TLabel").grid(
        row=1, column=0, sticky="w", pady=(0, 10))
    listing = ttk.Frame(sidebar, style="Plain.TFrame")
    listing.grid(row=2, column=0, sticky="nsew")
    listing.columnconfigure(0, weight=1)
    listing.rowconfigure(0, weight=1)
    self.tree = ttk.Treeview(listing, columns=("id", "name", "state", "angle"),
        show="headings", height=15, selectmode="browse", style="Sidebar.Treeview")
    for key, title, width in (("id", "ID", 30), ("name", "名称", 108),
                              ("state", "状态", 70), ("angle", "角度 °", 70)):
        self.tree.heading(key, text=title)
        self.tree.column(key, width=width, anchor="w" if key == "name" else "center", stretch=False)
    self.tree.grid(row=0, column=0, sticky="nsew")
    scroll = ttk.Scrollbar(listing, orient="vertical", command=self.tree.yview,
                           style="Side.Vertical.TScrollbar")
    scroll.grid(row=0, column=1, sticky="ns")
    self.tree.configure(yscrollcommand=scroll.set)
    for mid in range(15):
        self.tree.insert("", "end", iid=str(mid), values=(mid, f"舵机 {mid:02d}", "未连接", "—"))
    self.tree.selection_set("14")
    self.tree.bind("<<TreeviewSelect>>", self._select)
    progress = ttk.Frame(sidebar, style="Plain.TFrame")
    progress.grid(row=3, column=0, sticky="we", pady=(14, 8))
    self.calibration_text = tk.StringVar(value="0 / 15 已记录")
    ttk.Label(progress, text="零位记录", style="Muted.TLabel").pack(side="left")
    ttk.Label(progress, textvariable=self.calibration_text, style="Card.TLabel").pack(side="right")
    self.profile_progress = ttk.Progressbar(sidebar, maximum=15, style="Side.Horizontal.TProgressbar")
    self.profile_progress.grid(row=4, column=0, sticky="we", pady=(0, 14))
    self.stop_button = FeedbackButton(sidebar, text="全部请求零力矩", style="Danger.TButton",
                                  command=lambda: self._send("release_all"))
    self.stop_button.grid(row=5, column=0, sticky="we")

    workspace = ttk.Frame(root, padding=(16, 12, 16, 8))
    workspace.grid(row=0, column=1, sticky="nsew")
    workspace.columnconfigure(0, weight=1)
    workspace.rowconfigure(2, weight=1)
    header = ttk.Frame(workspace)
    header.grid(row=0, column=0, sticky="we", pady=(0, 6))
    ttk.Label(header, text="零位工作台", font=("Microsoft YaHei UI", 22, "bold")).pack(side="left")
    self.guide_button = FeedbackButton(header, text="使用指引", style="Page.Quiet.TButton", command=lambda: show_guide(root))
    self.guide_button.pack(side="right", padx=(8, 0))
    self.theme_button = FeedbackButton(header, text="切换深色" if root._theme_mode == "light" else "切换浅色",
                                   style="Page.Quiet.TButton", command=lambda: toggle_theme(self))
    self.theme_button.pack(side="right", padx=(8, 0))
    FeedbackButton(header, text="保存配置", style="Page.Primary.TButton", command=self._save).pack(side="right")
    FeedbackButton(header, text="关节绑定", style="Page.Quiet.TButton", command=self._open_robot_view).pack(
        side="right", padx=8)
    toolbar = ttk.Frame(workspace, style="Card.TFrame", padding=(12, 8))
    toolbar.grid(row=1, column=0, sticky="we", pady=(0, 8))
    ttk.Label(toolbar, text="串口", style="Muted.TLabel").pack(side="left", padx=(0, 7))
    self.port_box = ttk.Combobox(toolbar, textvariable=self.port, width=8,
                               font=BODY_FONT)
    self.port_box.pack(side="left")
    self.refresh_button = FeedbackButton(toolbar, text="↻", width=2, style="Quiet.TButton",
                                     command=self._refresh_ports)
    self.refresh_button.pack(side="left", padx=(3, 14))
    ttk.Label(toolbar, text="波特率", style="Muted.TLabel").pack(side="left", padx=(0, 7))
    self.baudrate_box = ttk.Combobox(toolbar, textvariable=self.baudrate, values=("6000000", "4000000"),
                                   width=8, state="readonly", font=BODY_FONT)
    self.baudrate_box.pack(side="left", padx=(0, 14))
    ttk.Label(toolbar, text="ID", style="Muted.TLabel").pack(side="left", padx=(0, 7))
    self.ids_entry = ttk.Entry(toolbar, textvariable=self.ids, width=10,
                              font=BODY_FONT)
    self.ids_entry.pack(side="left")
    self.ids_entry.pack_configure(padx=(0, 14))
    self.demo_check = ttk.Checkbutton(toolbar, text="模拟设备", variable=self.demo, command=self._change_mode)
    self.demo_check.pack(side="left")
    self.disconnect_button = FeedbackButton(toolbar, text="断开", style="Quiet.TButton", command=self._disconnect)
    self.disconnect_button.pack(side="right")
    self.connect_button = FeedbackButton(toolbar, text="连接设备", style="Primary.TButton", command=self._connect)
    self.connect_button.pack(side="right", padx=(12, 7))
    self.badge = ttk.Label(header, textvariable=self.connection_text, style="Idle.Badge.TLabel")
    self.badge.pack(side="left", padx=(14, 0))

    body = ttk.Frame(workspace)
    body.grid(row=2, column=0, sticky="nsew")
    for column, (weight, size) in enumerate(((1, 300), (2, 360), (1, 340))):
        body.columnconfigure(column, weight=weight, minsize=size)
    body.rowconfigure(0, weight=1)

    def card(name, parent, **grid):
        frame = ttk.Frame(parent, style="Card.TFrame", padding=(12, 12))
        frame.grid(sticky="nsew", **grid)
        self.panels[name] = frame
        return frame

    control = card("control", body, row=0, column=0, padx=(0, 10))
    settings = card("settings", body, row=0, column=2, padx=(10, 0))
    self.robot_host = ttk.Frame(body, style="Card.TFrame", padding=8, width=360, height=420)
    self.robot_host.grid(row=0, column=1, sticky="nsew")
    self.panels["structure"] = self.robot_host
    feedback = card("feedback", workspace, row=3, column=0, pady=(8, 0))
    feedback.configure(padding=(12, 8))

    ttk.Label(control, text="舵机调节", style="Section.TLabel").pack(anchor="w")
    # 名称完整内容可在右侧编辑；这里固定一行，避免长名称撑宽/撑高工作区。
    self.joint_title = ttk.Label(control, text="ID 14", style="Card.TLabel", width=1, anchor="w")
    self.joint_title.pack(fill="x", pady=(2, 4))
    ttk.Label(control, textvariable=self.position_caption, style="Muted.TLabel").pack(anchor="w")
    ttk.Label(control, textvariable=self.position_text, style="Big.TLabel").pack(anchor="w")
    mode_area = ttk.Frame(control, style="Plain.TFrame", height=status_height)
    mode_area.pack(fill="x", pady=(0, 4))
    mode_area.pack_propagate(False)
    ttk.Label(mode_area, textvariable=self.mode_text, style="Muted.TLabel", wraplength=280,
              anchor="nw").pack(fill="both", expand=True)
    self.arm_button = FeedbackButton(control, text="使能 · 保持当前位置", style="Primary.TButton", command=self._arm)
    self.arm_button.pack(fill="x", pady=(0, 6))
    target_row = ttk.Frame(control, style="Plain.TFrame")
    target_row.pack(fill="x", pady=(0, 6))
    ttk.Label(target_row, text="目标 / °", style="Card.TLabel").pack(side="left", padx=(0, 6))
    self.target_entry = ttk.Entry(target_row, textvariable=self.target, width=7,
                                  font=("Cascadia Mono", 15, "bold"))
    self.target_entry.pack(side="left", fill="x", expand=True)
    self.target_entry.bind("<Return>", lambda _: self.execute_button.invoke())
    self.execute_button = FeedbackButton(target_row, text="执行输入", width=0,
        style="Neutral.TButton", command=self._execute)
    self.execute_button.pack(side="left", padx=(6, 0))
    self.angle_slider = ttk.Scale(control, from_=-30, to=30, orient="horizontal",
        variable=self.slider_value, command=self._slide, style="Angle.Horizontal.TScale")
    self.angle_slider.pack(fill="x", pady=(8, 4))
    limits = ttk.Frame(control, style="Plain.TFrame")
    limits.pack(fill="x")
    self.slider_min_label = ttk.Label(limits, text="−30°", style="Muted.TLabel")
    self.slider_min_label.pack(side="left")
    ttk.Label(limits, text="0.1° 精调", style="Muted.TLabel").pack(side="left", expand=True)
    self.slider_max_label = ttk.Label(limits, text="30°", style="Muted.TLabel")
    self.slider_max_label.pack(side="right")
    jog = ttk.Frame(control, style="Plain.TFrame")
    jog.pack(fill="x", pady=(8, 6))
    self.jog_buttons = []
    for index, (delta, label) in enumerate(((-1, "−1°"), (-.1, "−0.1°"), (.1, "+0.1°"), (1, "+1°"))):
        button = FeedbackButton(jog, text=label, width=0, style="Quiet.TButton", command=lambda d=delta: self._jog(d))
        button.pack(side="left", expand=True, fill="x", padx=(0 if index == 0 else 4, 0))
        self.jog_buttons.append(button)
    actions = ttk.Frame(control, style="Plain.TFrame")
    actions.pack(fill="x", pady=(0, 6))
    self.hold_button = FeedbackButton(actions, text="停止并保持", style="Neutral.TButton",
                                  command=lambda: self._send("hold", id=self.selected))
    self.hold_button.pack(side="left", fill="x", expand=True, padx=(0, 6))
    self.release_button = FeedbackButton(actions, text="释放当前舵机", style="Release.TButton",
                                     command=lambda: self._send("release", id=self.selected))
    self.release_button.pack(side="left", fill="x", expand=True)

    ttk.Label(settings, text="关节设置", style="Section.TLabel").grid(
        row=0, column=0, sticky="w", pady=(0, 8))
    ttk.Label(settings, textvariable=self.deployment_text, style="Muted.TLabel").grid(
        row=0, column=1, sticky="e", pady=(0, 8))
    settings.columnconfigure(1, weight=1)
    self.settings_widgets = []

    def field(key, label, row, choices=None):
        ttk.Label(settings, text=label, style="Muted.TLabel").grid(
            row=row, column=0, sticky="w", padx=(0, 10), pady=(0, 3))
        if choices:
            widget = ttk.Combobox(settings, textvariable=self.fields[key], values=choices,
                                 width=9, state="readonly", font=BODY_FONT)
        else:
            widget = ttk.Entry(settings, textvariable=self.fields[key], width=9,
                               font=BODY_FONT)
        widget.grid(row=row, column=1, sticky="we", pady=(0, 3))
        self.settings_widgets.append((key, widget))
        return widget

    def paired_fields(row, items):
        line = ttk.Frame(settings, style="Plain.TFrame")
        line.grid(row=row, column=0, columnspan=2, sticky="we", pady=(0, 3))
        for index, (key, label, choices) in enumerate(items):
            column = index * 2
            ttk.Label(line, text=label, style="Muted.TLabel").grid(
                row=0, column=column, sticky="w", padx=(0 if index == 0 else 10, 5))
            if choices:
                widget = ttk.Combobox(line, textvariable=self.fields[key], values=choices,
                                     width=3, state="readonly", font=BODY_FONT)
            else:
                widget = ttk.Entry(line, textvariable=self.fields[key], width=6,
                                   font=BODY_FONT)
            widget.grid(row=0, column=column+1, sticky="we")
            line.columnconfigure(column+1, weight=1)
            self.settings_widgets.append((key, widget))

    field("name", "关节名称", 1)
    self.model_joint_box = field("model_joint", "模型关节", 2, choices=("未绑定",))
    self.model_joint_box.bind("<<ComboboxSelected>>", self._preview_model_joint)
    paired_fields(3, (("direction", "控制方向", ("+1", "−1")),
                      ("model_direction", "模型方向", ("+1", "−1"))))
    paired_fields(4, (("min_deg", "下限°", None), ("max_deg", "上限°", None)))
    field("kp", "Kp / N·m/rad", 5)
    field("kd", "Kd / N·m·s/rad", 6)
    field("follow_mode", "目标跟随", 7, choices=("直接跟随", "限速移动"))
    field("rate_deg_s", "回零／限速 °/s", 8)
    field("notes", "姿态备注", 9)
    ttk.Label(settings, text="部署名 / 序号", style="Muted.TLabel").grid(
        row=10, column=0, sticky="w", padx=(0, 10), pady=(0, 3))
    deployment_fields = ttk.Frame(settings, style="Plain.TFrame")
    deployment_fields.grid(row=10, column=1, sticky="we", pady=(0, 3))
    # 英文名与 1–15 动作序号共用一行，保持原最小窗口能看到全部操作。
    for key, width in (("deployment_joint", 9), ("policy_index", 2)):
        entry = ttk.Entry(deployment_fields, textvariable=self.fields[key], width=width, font=BODY_FONT)
        entry.pack(side="left", fill="x" if key == "deployment_joint" else "none",
                   expand=key == "deployment_joint", padx=(0 if key == "deployment_joint" else 5, 0))
        self.settings_widgets.append((key, entry))
    self.apply_button = FeedbackButton(settings, text="应用设置", style="Soft.TButton", command=self._apply_profile)
    self.apply_button.grid(row=11, column=0, columnspan=2, sticky="we", pady=(2, 6))
    deployment_actions = ttk.Frame(settings, style="Plain.TFrame")
    deployment_actions.grid(row=12, column=0, columnspan=2, sticky="we")
    self.import_deployment_button = FeedbackButton(deployment_actions, text="导入部署映射", style="Quiet.TButton",
                                                   command=self._import_deployment)
    self.import_deployment_button.pack(side="left", fill="x", expand=True, padx=(0, 6))
    self.export_deployment_button = FeedbackButton(deployment_actions, text="导出部署标定", style="Soft.TButton",
                                                   command=self._export_deployment)
    self.export_deployment_button.pack(side="left", fill="x", expand=True)

    ttk.Label(feedback, text="实时反馈", style="Section.TLabel").grid(
        row=0, column=0, sticky="w", padx=(0, 14))
    for index, (key, label) in enumerate((("raw", "outputPos / rad"), ("expos", "ExPos / rad"),
        ("speed", "速度 / °/s"), ("torque", "估算力矩 / N·m"), ("voltage", "电压 / V"),
        ("temp", "温度 / °C"), ("age", "间隔 / ms"), ("state", "状态 F / W / T"))):
        feedback.columnconfigure(index+1, weight=1)
        slot = ttk.Frame(feedback, style="Plain.TFrame")
        slot.grid(row=0, column=index+1, sticky="we", padx=(0, 8 if index < 7 else 0))
        ttk.Label(slot, text=label, style="Muted.TLabel").pack(anchor="w")
        ttk.Label(slot, textvariable=self.values[key], style="Readout.TLabel",
                  width=12 if key == "state" else 9 if key in ("raw", "expos") else 7).pack(anchor="w", pady=(2, 0))

    home = ttk.Frame(workspace, style="Card.TFrame", padding=(12, 6))
    home.grid(row=4, column=0, sticky="we", pady=(8, 0))
    self.panels["home"] = home
    heading = ttk.Frame(home, style="Plain.TFrame", height=status_height)
    heading.pack(fill="x", pady=(0, 5))
    heading.pack_propagate(False)
    ttk.Label(heading, text="软件零位", style="Section.TLabel").pack(side="left", anchor="n", padx=(0, 16))
    ttk.Label(heading, textvariable=self.zero_text, style="Muted.TLabel", wraplength=420,
              anchor="nw").pack(side="left", anchor="n")
    ttk.Label(heading, textvariable=self.home_text, style="Muted.TLabel", wraplength=340,
              anchor="nw").pack(side="right", anchor="n", padx=(14, 0))
    row = ttk.Frame(home, style="Plain.TFrame")
    row.pack(fill="x")
    self.capture_button = FeedbackButton(row, text="记录为0°并保存", style="Soft.TButton", command=self._capture_zero)
    self.capture_button.pack(side="left", padx=(0, 6))
    self.verify_button = FeedbackButton(row, text="标记验证", style="Quiet.TButton", command=self._verified)
    self.verify_button.pack(side="left", padx=(0, 16))
    ttk.Separator(row, orient="vertical").pack(side="left", fill="y", padx=(0, 16))
    self.current_home_button = FeedbackButton(row, text="当前回零", style="Soft.TButton", command=self._home_current)
    self.current_home_button.pack(side="left", padx=(0, 6))
    self.home_all_button = FeedbackButton(row, text="全部回零", style="Primary.TButton", command=self._home_all)
    self.home_all_button.pack(side="left", padx=(0, 6))
    self.home_stop_button = FeedbackButton(row, text="停止并释放", style="Release.TButton", command=self._cancel_home)
    self.home_stop_button.pack(side="left")
    self.clear_all_zeros_button = FeedbackButton(row, text="清除全部零位", style="Release.TButton",
                                            command=lambda: self._clear_zero(all_joints=True))
    self.clear_all_zeros_button.pack(side="right")
    self.clear_zero_button = FeedbackButton(row, text="清除当前零位", style="Quiet.TButton", command=self._clear_zero)
    self.clear_zero_button.pack(side="right", padx=(6, 6))

    history = ttk.Frame(workspace, style="Card.TFrame", padding=(12, 8))
    history.grid(row=5, column=0, sticky="nsew", pady=(8, 0))
    self.panels["history"] = history
    history.columnconfigure(1, weight=1)
    history.rowconfigure(0, weight=1)
    ttk.Label(history, text="操作日志", style="Section.TLabel").grid(
        row=0, column=0, sticky="nw", padx=(0, 16))
    self.log = tk.Text(history, height=3, font=BODY_FONT,
                       relief="flat", wrap="word", highlightthickness=0, padx=0, pady=0, state="disabled")
    theme_widget(self.log, background="surface", foreground="muted", insertbackground="ink",
                 selectbackground="accent", selectforeground="accent_ink")
    self.log.grid(row=0, column=1, sticky="nsew")
    log_scroll = ttk.Scrollbar(history, orient="vertical", command=self.log.yview,
                              style="Side.Vertical.TScrollbar")
    log_scroll.grid(row=0, column=2, sticky="ns", padx=(8, 0))
    self.log.configure(yscrollcommand=log_scroll.set)
    footer = ttk.Frame(workspace)
    footer.grid(row=6, column=0, sticky="we", pady=(6, 0))
    FeedbackButton(footer, text="加载配置", style="Page.Quiet.TButton", command=self._load).pack(side="left")
    FeedbackButton(footer, text="恢复备份", style="Page.Quiet.TButton", command=self._restore).pack(side="left", padx=5)
    theme_widget(ttk.Label(footer, textvariable=self.path_text, width=28, anchor="e"), foreground="muted").pack(side="right")
    self.action_label = ttk.Label(footer, textvariable=self.action_text, style=ACTION_STYLES["info"], width=1, anchor="w")
    self.action_label.pack(side="left", fill="x", expand=True, padx=(14, 8))
    self.responsive_ui = ResponsiveLayout(self, workspace, (mode_area, heading))
    self._log("工作台已就绪。调节、设置、反馈和日志同时显示。")


def refresh_chrome(self):
    count = sum(p["zero_offset"] is not None for p in self.profiles.values())
    update_text(self.calibration_text, f"{count} / 15 已记录")
    update_options(self.profile_progress, value=count)
    badge = "Fault" if self.faulted and self.connected else (
        "Demo" if self.connected and self.demo.get() else "Live" if self.connected else "Idle"
    )
    update_options(self.badge, style=badge+".Badge.TLabel")

