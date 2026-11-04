"""Offline, invisible-window geometry audit using isolated configuration and a renderer stub."""
from contextlib import ExitStack
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import tkinter as tk
from tkinter import ttk
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "tests")]
import app
import robot_view
import ui
from test_embedded_preview import PreparedRenderer
from test_robot_view import scene


def run():
    def trace(message):
        with (ROOT / "build" / "qa_layout_matrix.trace").open("a", encoding="utf-8") as output:
            output.write(message + "\n")
    trace("starting")
    results, callback_errors = [], []
    with tempfile.TemporaryDirectory(dir=ROOT / "build" / "tmp") as folder, ExitStack() as patches:
        folder = Path(folder)
        model = scene()
        model["_fingerprint"] = "isolated-layout-audit"
        patches.enter_context(patch("serial.Serial", side_effect=AssertionError("no hardware during layout audit")))
        patches.enter_context(patch.object(app.CalibratorApp, "_refresh_ports"))
        patches.enter_context(patch.object(ui, "THEME_PATH", folder / "appearance.json"))
        patches.enter_context(patch.object(robot_view, "BINDINGS_PATH", folder / "bindings.json"))
        patches.enter_context(patch.object(robot_view, "load_scene", return_value=model))
        patches.enter_context(patch.object(robot_view, "_Renderer", PreparedRenderer))
        root = tk.Tk()
        trace("tk created")
        root.withdraw()
        root.attributes("-alpha", 0.0)
        # Allow an invisible audit window to extend beyond this monitor's work area.
        root.maxsize(4096, 2160)
        root.report_callback_exception = lambda *error: callback_errors.append(str(error))
        application = app.CalibratorApp(root, config_path=folder / "calibration.json")
        trace("app created")
        root.geometry("1360x940+0+0")
        root.deiconify()

        def pump(duration=.25):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                root.update()
                time.sleep(.005)

        def box(widget):
            return (widget.winfo_rootx()-root.winfo_rootx(), widget.winfo_rooty()-root.winfo_rooty(),
                    widget.winfo_width(), widget.winfo_height())

        def bounds():
            value = {name: box(panel) for name, panel in application.panels.items()}
            viewer = application.robot_view
            value["viewport"] = box(viewer.picture.master)
            value["model_status"] = box(viewer.footer)
            value["sidebar"] = box(application.tree)
            return value

        try:
            deadline = time.monotonic() + 3
            while not application.robot_view or not application.robot_view.renderer:
                root.update()
                if time.monotonic() > deadline:
                    raise AssertionError("preview did not initialize")
                time.sleep(.01)
            trace("preview ready")
            pump()
            for width, height in ((1360, 940), (1720, 1080), (1900, 1220), (2560, 1360),
                                  (1900, 1210), (1720, 1080), (1360, 940)):
                root.geometry(f"{width}x{height}+0+0")
                trace(f"size {width}x{height}")
                pump()
                expected_size = root.winfo_width(), root.winfo_height()
                actual = bounds()
                for name, (x, y, w, h) in actual.items():
                    assert x >= 0 and y >= 0 and x+w <= expected_size[0] and y+h <= expected_size[1], (name, actual, expected_size)
                mutation_checks = 0
                for theme in ("light", "dark", "light"):
                    trace(f"theme {theme}")
                    ui.apply_theme(root, theme)
                    root.update_idletasks()
                    assert bounds() == actual, ("theme", theme, width, height, actual, bounds())
                    for motor in range(15):
                        application.selected = motor
                        application.profiles[motor]["name"] = "已标定关节名称" * (1 + motor)
                        application._show_profile()
                        application.position_text.set(("—", "+0.01°", "-12345.67°")[motor % 3])
                        application.connection_text.set(("未连接", "正在请求零力矩并关闭…")[motor % 2])
                        application.mode_text.set("当前禁用位置控制 · 可读取并记录稳定零位" * (1 + motor % 2))
                        application.home_text.set("回零超时，实际位置尚未稳定到位；请检查负载与增益。" if motor % 2 else "回零完成：15 台保持在已记录零位")
                        application.values["age"].set("10000 · 过期" if motor % 2 else "10")
                        application.values["raw"].set("-12345.123456" if motor % 2 else "0.000000")
                        application.robot_view.status.set(("结构就绪", "目标预览 +20.0° · 实际反馈 +0.0° · 模拟示例绑定，与实机 ID 无关", "反馈过期／异常，冻结：" + ", ".join(map(str, range(15))))[motor % 3])
                        root.update_idletasks()
                        assert bounds() == actual, ("content", theme, motor, width, height, actual, bounds())
                        mutation_checks += 1
                # Every primary action remains visible and contained at this window size.
                actions = ("arm_button", "execute_button", "hold_button", "release_button", "apply_button", "apply_gains_button", "capture_button", "capture_all_button", "verify_button", "current_home_button", "home_all_button", "home_stop_button", "clear_zero_button", "clear_all_zeros_button", "import_deployment_button", "export_deployment_button", "stop_button")
                for name in actions:
                    widget = getattr(application, name)
                    x, y, w, h = box(widget)
                    assert widget.winfo_ismapped() and w > 1 and h > 1, ("hidden action", name, expected_size)
                    assert x >= 0 and y >= 0 and x+w <= expected_size[0] and y+h <= expected_size[1], ("clipped action", name, box(widget), expected_size)
                    px, py, pw, ph = box(widget.master)
                    assert x >= px and y >= py and x+w <= px+pw and y+h <= py+ph, ("action outside parent", name, box(widget), box(widget.master))
                results.append(dict(requested=[width, height], actual=expected_size, expanded=application.responsive_ui.expanded,
                                    stable_mutations=mutation_checks, bounds=actual))
            assert not callback_errors, callback_errors
            print(json.dumps(dict(passed=True, configurations=len(results), stable_content_checks=sum(row["stable_mutations"] for row in results),
                                  themes=["light", "dark", "light"], primary_actions_checked=17, results=results), ensure_ascii=False, indent=2))
        finally:
            if application.robot_view:
                application.robot_view.close()
            application.controller.send("shutdown")
            application.controller.join(3)
            for callback in root.tk.call("after", "info"):
                root.after_cancel(callback)
            root.destroy()


if __name__ == "__main__":
    run()
