"""S288 软件零位标定界面。实机连接始终由用户发起。"""
from __future__ import annotations

import argparse
from datetime import datetime
import math
from pathlib import Path
import queue
import time
import weakref
import tkinter as tk
from tkinter import filedialog, messagebox

from config import default_profiles, load_profiles, parse_ids, save_profiles, validate_profile
from controller import Controller
from deployment import apply_layout, build_calibration, load_layout, write_calibration
from protocol import validate_baudrate
from ui import ACTION_STYLES, build_ui, refresh_chrome, update_options, update_text

ROOT = Path(__file__).resolve().parent


class CalibratorApp:
    """主线程只处理界面和文件；串口交给 Controller，返回结果统一在 _handle 处理。"""

    def __init__(self, root: tk.Tk, demo: bool = False, config_path: Path | None = None):
        self.root = root
        self.root._action_feedback = weakref.WeakMethod(self._show_action)
        self.profiles = default_profiles()
        self.events = queue.Queue()
        self.controller = Controller(self.events)
        self.controller.start()
        self.snapshots = {}
        self.connected = False
        self.connecting = False
        self.monitored = []
        self.selected = 14
        self.armed_id = None
        self._arm_pending = False
        self.faulted = False
        self.closing = False
        self._stopped = False
        self.dirty = False
        self.requested_deg = 0.0
        self._slider_after = None
        self._slider_pending = None
        self._slider_generation = 0
        self._slider_sync = False
        self.robot_view = None
        self.homing = False
        self.holding_ids = set()
        self._capture_save_id = None
        self.path_override = config_path
        self.config_path = self._default_path(demo)
        self.demo = tk.BooleanVar(value=demo)
        self.port = tk.StringVar(value="COM3")
        self.baudrate = tk.StringVar(value="6000000")
        self.ids = tk.StringVar(value="0-14" if demo else "14")
        self.target = tk.StringVar(value="0.0")
        self.slider_value = tk.DoubleVar(value=0.0)
        self.connection_text = tk.StringVar(value="未连接")
        self.position_text = tk.StringVar(value="—")
        self.position_caption = tk.StringVar(value="临时角度 / °")
        self.zero_text = tk.StringVar(value="尚未标定")
        self.mode_text = tk.StringVar(value="选择舵机，连接后读取当前位置")
        self.path_text = tk.StringVar(value=f"配置：{self.config_path.name}")
        self.home_text = tk.StringVar(value="先释放并手动摆好关节，记录为0°并保存。")
        self.action_text = tk.StringVar(value="等待操作。")
        self.deployment_text = tk.StringVar(value="部署 0/15")
        self.values = {key: tk.StringVar() for key in (
            "raw", "expos", "speed", "torque", "voltage", "temp", "age", "state"
        )}
        self.fields = {key: tk.StringVar() for key in (
            "name", "direction", "min_deg", "max_deg", "kp", "kd", "rate_deg_s", "notes", "follow_mode",
            "model_joint", "model_direction", "deployment_joint", "policy_index"
        )}
        self._model_options = {}
        self._displayed_rows = {}
        self._build()
        self._refresh_ports()
        self._load_initial()
        self._show_profile()
        self.robot_host.bind("<Map>", lambda _: self._ensure_robot_view(), add="+")
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(50, self._poll)
        if demo:
            self.root.after(200, self._connect)

    def _default_path(self, demo):
        return self.path_override or ROOT / "configs" / (
            "demo_calibration.json" if demo else "calibration.json"
        )

    def _build(self):
        build_ui(self)

    def _ensure_robot_view(self):
        if self.closing or self.robot_view and self.robot_view.window.winfo_exists():
            return
        from robot_view import RobotView
        self.robot_view = RobotView(self, parent=self.robot_host)
        self._model_binding_changed()

    def _model_binding_changed(self):
        """Refresh model fields without discarding edits to the control profile."""
        viewer = self.robot_view
        self._model_options = viewer.binding_options() if viewer else {}
        update_options(self.model_joint_box, values=("未绑定", *self._model_options))
        binding = viewer.get_binding(self.selected) if self._model_options else None
        label = next((label for label, key in self._model_options.items()
                      if binding and key == binding["joint"]), "未绑定")
        update_text(self.fields["model_joint"], label if self._model_options else "模型载入中…")
        update_text(self.fields["model_direction"], "−1" if binding and binding["direction"] == -1 else "+1")
        self._render()

    def _preview_model_joint(self, _=None):
        if self.robot_view and self._model_options:
            self.robot_view.preview_joint(self.selected, self._model_options.get(self.fields["model_joint"].get()))

    def _open_robot_view(self):
        self._ensure_robot_view()
        if self.robot_view:
            self.robot_view.show_bindings()

    def preview_target_raw(self):
        """Requested pose for the labelled target preview; never measured feedback."""
        if not self._can_slide():
            return None
        p = self.profiles[self.selected]
        zero = p["zero_offset"]
        if zero is None:
            zero = self.snapshots[self.selected]["origin"]
        return self.selected, zero + p["direction"] * math.radians(self.requested_deg)

    def _refresh_ports(self):
        try:
            from serial.tools import list_ports
            self.port_box["values"] = [p.device for p in list_ports.comports()]
            self._show_action(f"串口列表已刷新，共 {len(self.port_box['values'])} 个端口。", "success")
        except ImportError:
            self.port_box["values"] = []
            self._show_action("未能读取串口列表，请检查串口依赖。", "warning")

    def _show_action(self, message, tone="info"):
        """独立操作提示，不改设备状态，也不把“收到点击”当作“实机到位”。"""
        if self.closing:
            return
        update_text(self.action_text, message)
        update_options(self.action_label, style=ACTION_STYLES.get(tone, ACTION_STYLES["info"]))

    def _log(self, message):
        self._show_action(message)
        self.log.configure(state="normal")
        self.log.insert("end", f"{datetime.now():%H:%M:%S}  {message}\n")
        if int(self.log.index("end-1c").split(".")[0]) > 160:
            self.log.delete("1.0", "40.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _error(self, exc):
        self._log(str(exc))
        self._show_action(f"操作未完成：{exc}", "error")
        messagebox.showerror("操作未完成", str(exc), parent=self.root)

    def _send(self, action, **kwargs):
        if action in {"hold", "release", "release_all", "disconnect", "shutdown"}:
            self._cancel_slider()
        if not self.closing:
            # 这里只表示已提交；完成/失败仍由 _handle 的实际反馈确认。
            pending = {
                "connect": "正在连接设备…", "arm": "正在使能当前舵机…",
                "capture_zero": "正在读取并保存当前零位…", "home": "正在检查并返回保存零位…",
                "hold": "正在请求保持当前位置…", "release": "正在请求释放当前舵机…",
                "release_all": "正在请求全部零力矩…",
                "disconnect": "正在取消连接…" if self.connecting else "正在断开连接…",
            }
            if action in pending:
                self._show_action(pending[action])
            elif action == "target":
                self._show_action(f"ID {kwargs['id']} 目标已提交：{kwargs['angle_deg']:g}°；实际到位请看反馈。")
            self.controller.send(action, **kwargs)

    def _cancel_slider(self):
        self._slider_generation += 1
        if self._slider_after is not None:
            self.root.after_cancel(self._slider_after)
        self._slider_after = None
        self._slider_pending = None

    def _sync_target(self, angle, *, cancel_pending=True):
        """Synchronize controls without issuing a motor command."""
        if cancel_pending:
            self._cancel_slider()
        p = self.profiles[self.selected]
        angle = max(p["min_deg"], min(p["max_deg"], float(angle)))
        self._slider_sync = True
        try:
            update_options(self.angle_slider, **{"from": p["min_deg"], "to": p["max_deg"]})
            self.slider_value.set(angle)
            update_text(self.target, f"{angle:.3f}")
            self.requested_deg = angle
            update_options(self.slider_min_label, text=f"{p['min_deg']:g}°")
            update_options(self.slider_max_label, text=f"{p['max_deg']:g}°")
        finally:
            self._slider_sync = False

    def _can_slide(self):
        snap = self.snapshots.get(self.selected)
        return bool(self.connected and self.armed_id == self.selected and not self.faulted
                    and not getattr(self, "homing", False) and not getattr(self, "holding_ids", ())
                    and not self.closing and snap
                    and not snap.get("faulted", False)
                    and snap["feedback"].get("mode", 1) == 1
                    and not any(snap["feedback"].get(k, 0) for k in ("fault", "warning", "timeout"))
                    and -.05 <= time.monotonic() - snap["feedback"]["timestamp"] <= .3)

    def _slide(self, value):
        if self._slider_sync or not self._can_slide():
            return
        angle = float(value)
        if not math.isfinite(angle):
            return
        self._sync_target(round(angle, 1), cancel_pending=False)
        self._slider_pending = (self.selected, self.requested_deg)
        if self._slider_after is None:
            generation = self._slider_generation
            self._slider_after = self.root.after(16, lambda: self._flush_slider(generation))

    def _flush_slider(self, generation):
        if generation != self._slider_generation:
            return
        self._slider_after = None
        pending, self._slider_pending = self._slider_pending, None
        if pending and pending[0] == self.selected and self._can_slide():
            self.controller.send("target", id=pending[0], angle_deg=pending[1])

    def _load_initial(self):
        if self.config_path.exists():
            try:
                self.profiles = load_profiles(self.config_path, simulation=self.demo.get())
                self._log(f"已加载配置：{self.config_path.name}。连接后仍默认禁用。")
            except (ValueError, OSError) as exc:
                self._error(exc)

    def _change_mode(self):
        if self.connected or self.connecting:
            return
        if self.dirty and not messagebox.askyesno("切换配置", "当前改动尚未保存，放弃这些改动并切换模式？", parent=self.root):
            self.demo.set(not self.demo.get())
            return
        if self.robot_view:
            self.robot_view.close()
            self.robot_view = None
        self.profiles = default_profiles()
        self.dirty = False
        self.config_path = self._default_path(self.demo.get())
        self.ids.set("0-14" if self.demo.get() else "14")
        self.path_text.set(f"配置：{self.config_path.name}")
        self.snapshots.clear()
        self._load_initial()
        self._show_profile()
        self._ensure_robot_view()

    def _connect(self):
        if self.connected or self.connecting:
            return
        try:
            ids = parse_ids(self.ids.get())
            baudrate = validate_baudrate(int(self.baudrate.get()))
            if not self.demo.get() and not self.port.get().strip():
                raise ValueError("请填写串口。")
        except ValueError as exc:
            self._error(exc)
            return
        self.connecting = True
        self.connection_text.set("正在连接模拟设备…" if self.demo.get() else "正在连接…")
        self._send("connect", port=self.port.get().strip(), ids=ids, demo=self.demo.get(), baudrate=baudrate)

    def _disconnect(self):
        self._send("disconnect")

    def _select(self, _=None):
        selection = self.tree.selection()
        if not selection:
            return
        next_id = int(selection[0])
        if next_id == self.selected:
            return
        # 结果返回前不能换 ID，否则旧舵机可能已使能、界面却切到了新舵机。
        if self._arm_pending or self._capture_save_id is not None:
            self.tree.selection_set(str(self.selected))
            self._log("正在等待使能或记录结果，请稍候再切换舵机。")
            return
        if self.homing or self.holding_ids:
            self.tree.selection_set(str(self.selected))
            self._log("回零或保持零位期间不能切换调节，请先全部释放。")
            return
        self._cancel_slider()
        if self.armed_id is not None:
            self._send("release", id=self.armed_id)
        self.selected = next_id
        self.requested_deg = 0.0
        self.target.set("0.0")
        self._show_profile()
        if self.connected and self.selected in self.monitored:
            self._send("select", id=self.selected)

    def _show_profile(self):
        p = self.profiles[self.selected]
        self.joint_title.configure(text=f"ID {self.selected:02d} · {p['name']}")
        for key, var in self.fields.items():
            if key in {"model_joint", "model_direction"}:
                continue
            if key == "follow_mode":
                var.set("直接跟随" if p["direct_follow"] else "限速移动")
            elif key == "policy_index":
                var.set("" if p[key] is None else str(p[key]))
            else:
                var.set(("+1" if p["direction"] == 1 else "−1") if key == "direction" else str(p[key]))
        offset = p["zero_offset"]
        self.zero_text.set("尚未标定 · 本次连接临时0°" if offset is None
                           else f"已记录零位 {offset:.6f} rad  ·  {'已标记验证' if p['verified'] else '待验证'}")
        self._sync_target(self.requested_deg)
        self._model_binding_changed()
        self._preview_model_joint()

    def _apply_profile(self, silent=False):
        if self._control_busy():
            if not silent:
                self._error(ValueError("请先释放舵机，再修改方向、限位或控制参数。"))
            return False
        try:
            old = self.profiles[self.selected]
            p = dict(old)
            p["name"] = self.fields["name"].get().strip()
            p["notes"] = self.fields["notes"].get().strip()
            p["deployment_joint"] = self.fields["deployment_joint"].get().strip()
            order = self.fields["policy_index"].get().strip()
            p["policy_index"] = int(order) if order else None
            if self.fields["direction"].get() not in {"+1", "−1"}:
                raise ValueError("控制方向只能为 +1 或 −1。")
            p["direction"] = 1 if self.fields["direction"].get() == "+1" else -1
            follow_mode = self.fields["follow_mode"].get()
            if follow_mode not in {"直接跟随", "限速移动"}:
                raise ValueError("目标跟随只能选择“直接跟随”或“限速移动”。")
            p["direct_follow"] = follow_mode == "直接跟随"
            for key in ("min_deg", "max_deg", "kp", "kd", "rate_deg_s"):
                p[key] = float(self.fields[key].get())
            p = validate_profile(p)
            for mid, other in self.profiles.items():
                if mid == self.selected:
                    continue
                if p["deployment_joint"] and p["deployment_joint"] == other["deployment_joint"]:
                    raise ValueError(f"部署关节名已由 ID {mid} 使用，请先清除旧映射。")
                if p["policy_index"] is not None and p["policy_index"] == other["policy_index"]:
                    raise ValueError(f"策略序号已由 ID {mid} 使用，请先清除旧序号。")
            if any(p[k] != old[k] for k in ("direction", "min_deg", "max_deg")):
                p["verified"] = False
            if self._model_options:
                label = self.fields["model_joint"].get()
                if label != "未绑定" and label not in self._model_options:
                    raise ValueError("请选择列表中的模型关节，或选择“未绑定”。")
                sign = self.fields["model_direction"].get()
                if sign not in {"+1", "−1"}:
                    raise ValueError("模型方向只能为 +1 或 −1。")
                key, display_direction = self._model_options.get(label), 1 if sign == "+1" else -1
                binding = self.robot_view.get_binding(self.selected)
                selected_binding = (key, display_direction) if key is not None else None
                current_binding = (binding["joint"], binding["direction"]) if binding else None
                if selected_binding != current_binding:
                    # 先完成绑定保存，再提交内存设置；绑定失败时保留原控制参数。
                    self.robot_view.set_binding(self.selected, key, display_direction)
                    self._log(f"ID {self.selected} 模型关节和显示方向已保存。")
            if p != old:
                self.profiles[self.selected] = p
                self.dirty = True
                self._log(f"ID {self.selected} 设置已应用；请保存配置。")
            self._show_profile()
            return True
        except (ValueError, TypeError, OSError) as exc:
            self._error(exc)
            return False

    def _deployment_idle(self):
        """文件操作不访问总线，也不在运动/连接未完成时切换部署定义。"""
        return not (self._control_busy() or self.connecting or self.closing
                    or self.connected and self.faulted)

    def _import_deployment(self):
        if not self._deployment_idle():
            self._error(ValueError("请先结束当前操作并释放舵机，再导入部署映射。"))
            return
        if not self._apply_profile(silent=True):
            return
        filename = filedialog.askopenfilename(
            title="选择训练/部署关节配置（如 reference_14dof.json）",
            filetypes=[("关节映射 JSON", "*.json")], parent=self.root)
        if not filename:
            self._show_action("已取消导入部署映射。")
            return
        try:
            layout = load_layout(filename)
            updated = apply_layout(self.profiles, layout)
            ids = sorted(layout["joint_to_motor_id"].values())
            note = "\nID 14 不参与该14关节布局；它的零位记录仍保留。" if len(ids) == 14 else ""
            if not messagebox.askyesno(
                "导入部署映射",
                f"导入 {len(ids)} 个关节的名称与策略顺序。{note}\n\n"
                "此映射必须与实际接线及训练模型一致。\n"
                "只更新部署名称和策略序号；零位、方向、限位、增益及三维绑定均保留。\n继续导入？",
                parent=self.root):
                self._show_action("已取消导入部署映射。")
                return
            if not self._deployment_idle():
                raise ValueError("设备操作状态已变化，请释放后重新导入。")
            self.profiles, self.dirty = updated, True
            self._show_profile()
            self._log(f"已导入 {len(ids)} 关节部署映射；请核对实际对应并保存配置。")
        except (ValueError, TypeError, OSError) as exc:
            self._error(exc)

    def _export_deployment(self):
        if not self._deployment_idle():
            self._error(ValueError("请先结束当前操作并释放舵机，再导出部署标定。"))
            return
        if not self._apply_profile(silent=True):
            return
        try:
            # 先校验完整性，再询问实际模型参考姿态；文件选择和导出不发送电机命令。
            document = build_calibration(self.profiles, simulation=self.demo.get(),
                                         baudrate=validate_baudrate(int(self.baudrate.get())),
                                         confirmed_reference=True)
            count = len(document["joints"])
            excluded = document["deployment"]["excluded_ids"]
            note = ("\n原例程固定14关节；此15关节文件需配套修改部署端后使用。"
                    if count == 15 else "\n本次不包含 ID 14；它的原始配置仍保留。")
            if excluded:
                note += "\n未导出的 ID：" + ", ".join(map(str, excluded))
            if document["deployment"]["baudrate"] != 6000000:
                note += "\n本文件记录4 Mbps；原例程仍固定6 Mbps，需同步调整部署程序。导出不会修改电机或部署端波特率。"
            if not messagebox.askyesno(
                "确认模型零位与方向",
                f"准备导出 {count} 个关节。{note}\n\n"
                "请确认：所选关节保存的0°就是训练XML的 q=0，控制正方向也与模型一致。\n"
                "CAD显示参考不等于模型零位；文件不能代替实物核验。\n已确认并继续导出？",
                parent=self.root):
                self._show_action("已取消部署标定导出。")
                return
            filename = filedialog.asksaveasfilename(
                title="另存部署标定（保留原软件配置）", defaultextension=".json",
                initialfile=f"s288_calibration_{count}dof.json",
                filetypes=[("部署标定 JSON", "*.json")], parent=self.root)
            if not filename:
                self._show_action("已取消部署标定导出。")
                return
            if not self._deployment_idle():
                raise ValueError("设备操作状态已变化，请释放后重新导出。")
            if Path(filename).resolve() == self.config_path.resolve():
                raise ValueError("部署标定请另存新文件，不能覆盖当前软件配置。")
            write_calibration(filename, document)
            self._log(f"已导出 {count} 关节部署标定：{filename}；原软件配置未改动。")
            if document["deployment"]["requires_runtime_adaptation"]:
                self._show_action("部署标定已导出；关节数量或波特率需要配套适配部署端，文件不代表可直接运行。", "warning")
        except (ValueError, TypeError, OSError) as exc:
            self._error(exc)

    def _joint_angle(self, motor_id, raw):
        p = self.profiles[motor_id]
        origin = self.snapshots.get(motor_id, {}).get("origin", raw)
        zero = p["zero_offset"] if p["zero_offset"] is not None else origin
        return math.degrees(p["direction"] * (raw - zero))

    def _arm(self):
        if not self._apply_profile(silent=True):
            return
        p = self.profiles[self.selected]
        self._arm_pending = True
        self._send("arm", id=self.selected, **{
            k: p[k] for k in ("kp", "kd", "zero_offset", "direction", "min_deg", "max_deg", "rate_deg_s", "direct_follow")
        })
        self._render()

    def _execute(self):
        self._cancel_slider()
        try:
            angle = float(self.target.get())
            if not math.isfinite(angle):
                raise ValueError("目标角度必须是有限数值。")
            p = self.profiles[self.selected]
            if not p["min_deg"] <= angle <= p["max_deg"]:
                raise ValueError(f"目标必须在 {p['min_deg']}° 到 {p['max_deg']}° 之间。")
            if self.armed_id != self.selected:
                raise ValueError("请先使能当前舵机。")
            self._send("target", id=self.selected, angle_deg=angle)
            self._sync_target(angle)
        except ValueError as exc:
            self._error(exc)

    def _jog(self, delta):
        self.target.set(f"{self.requested_deg + delta:.3f}")
        self._execute()

    def _control_busy(self):
        return (self._arm_pending or self.armed_id is not None or self.homing or bool(self.holding_ids)
                or self._capture_save_id is not None)

    def _capture_zero(self):
        if self._control_busy():
            return
        if not self._apply_profile():
            return
        self._capture_save_id = self.selected
        self._send("capture_zero", id=self.selected)

    def _home_current(self):
        self._begin_home([self.selected])

    def _home_all(self):
        self._begin_home([mid for mid, profile in self.profiles.items()
                          if profile["zero_offset"] is not None])

    def _begin_home(self, motor_ids):
        try:
            if not self.connected or self.faulted or self._control_busy():
                raise ValueError("请先连接设备并全部释放，再回零。")
            if not motor_ids:
                raise ValueError("还没有记录零位，请先手动摆好关节并记录。")
            missing = [mid for mid in motor_ids if mid not in self.monitored]
            if missing:
                raise ValueError(f"已标定 ID {missing} 未连接，请修改顶部 ID 后重连。")
            if any(self.profiles[mid]["zero_offset"] is None for mid in motor_ids):
                raise ValueError("当前舵机尚未记录零位。")
            self._cancel_slider()
            self.homing = True
            self.home_text.set("正在检查零位、限位和设备状态…")
            self._send("home", profiles={mid: dict(self.profiles[mid]) for mid in motor_ids})
            self._render()
        except ValueError as exc:
            self._error(exc)

    def _cancel_home(self):
        self._send("release_all")

    def _verified(self):
        p = self.profiles[self.selected]
        if p["zero_offset"] is None:
            return
        p["verified"] = True
        self.dirty = True
        self._show_profile()
        self._log(f"ID {self.selected}：已人工标记小角度往返验证完成。断电重启后仍需核对基准姿态。")

    def _clear_zero(self, all_joints=False):
        """Clear saved references only; never send a device command."""
        def blocked():
            return (self._control_busy() or self.connecting or self.closing
                    or self.connected and self.faulted)

        if blocked():
            self._error(ValueError("请先全部释放并等待操作完成；通信异常时先断开，再清除零位。"))
            return
        ids = [mid for mid, p in self.profiles.items()
               if p["zero_offset"] is not None and (all_joints or mid == self.selected)]
        if not ids:
            self._show_action("当前没有可清除的零位记录。", "warning")
            return
        scope = f"全部 {len(ids)} 台舵机（ID：{', '.join(map(str, ids))}）" if all_joints else f"ID {ids[0]}"
        if not messagebox.askyesno("清除软件零位", f"清除 {scope} 的软件零位和验证记录？\n\n"
                f"当前配置：{self.config_path.name}\n"
                "清除结果会立即保存。名称、方向、限位、Kp、Kd和三维绑定保留。\n"
                "已有配置保留上一版 .bak，可通过“恢复备份”载入。", parent=self.root):
            self._show_action("已取消清除零位。")
            return
        if blocked():
            self._error(ValueError("设备状态已变化，请全部释放并等待操作完成，再清除零位。"))
            return
        profiles = {mid: dict(p) for mid, p in self.profiles.items()}
        for mid in ids:
            profiles[mid].update(zero_offset=None, reference_expos=None, calibrated_at=None, verified=False)
        try:
            save_profiles(self.config_path, profiles, simulation=self.demo.get())
        except (OSError, ValueError) as exc:
            self._error(exc)
            return
        self.profiles, self.dirty = profiles, False
        self._cancel_slider()
        if self.selected in ids:
            self.zero_text.set("尚未标定 · 本次连接临时0°")
            snap = self.snapshots.get(self.selected)
            self._sync_target(self._joint_angle(self.selected, snap["feedback"]["raw_pos"]) if snap else 0)
        self.home_text.set("零位已清除，请手动摆好关节后重新记录。")
        self._log(f"已清除 {scope} 的软件零位并保存至 {self.config_path.name}；未下发运动指令。")
        self._render()

    def _save(self):
        if not self._control_busy() and not self._apply_profile(silent=True):
            return
        filename = filedialog.asksaveasfilename(parent=self.root, title="保存软件零位配置",
            initialdir=str(self.config_path.parent), initialfile=self.config_path.name,
            defaultextension=".json", filetypes=[("JSON 配置", "*.json")])
        if filename:
            try:
                save_profiles(filename, self.profiles, simulation=self.demo.get())
                self.config_path = Path(filename)
                self.path_text.set(f"配置：{self.config_path.name}")
                self.dirty = False
                self._log(f"配置已保存：{filename}")
            except (OSError, ValueError) as exc:
                self._error(exc)
        else:
            self._show_action("已取消保存配置。")

    def _load_file(self, path):
        if self._control_busy():
            raise ValueError("请先释放，再加载配置。")
        loaded = load_profiles(path, simulation=self.demo.get())
        self.profiles = loaded
        self.dirty = False
        self._show_profile()
        self._log(f"已加载：{path}。加载配置没有下发运动目标。")

    def _load(self):
        if self._control_busy():
            self._error(ValueError("请先释放，再加载配置。"))
            return
        if self.dirty and not messagebox.askyesno("加载配置", "放弃尚未保存的改动并加载？", parent=self.root):
            self._show_action("已取消加载配置。")
            return
        filename = filedialog.askopenfilename(parent=self.root, title="加载软件零位配置",
            initialdir=str(self.config_path.parent), filetypes=[("JSON 配置", "*.json")])
        if filename:
            try:
                self._load_file(filename)
                self.config_path = Path(filename)
                self.path_text.set(f"配置：{self.config_path.name}")
            except (OSError, ValueError) as exc:
                self._error(exc)
        else:
            self._show_action("已取消加载配置。")

    def _restore(self):
        if self._control_busy():
            self._error(ValueError("请先释放，再恢复备份。"))
            return
        backup = Path(str(self.config_path) + ".bak")
        if not backup.exists():
            self._error(ValueError("当前配置还没有上一次保存的 .bak 备份。"))
            return
        if not messagebox.askyesno("恢复备份", "将备份加载到界面？原配置文件会保留，保存后才覆盖。", parent=self.root):
            self._show_action("已取消恢复备份。")
            return
        try:
            self._load_file(backup)
            self.dirty = True
        except (OSError, ValueError) as exc:
            self._error(exc)

    def _handle(self, event):
        kind = event["type"]
        mid = event.get("id")
        if kind == "connected":
            self._cancel_slider()
            self.connected = True
            self.connecting = False
            self.monitored = event["ids"]
            self.snapshots.clear()
            self.armed_id = None
            self._arm_pending = False
            self.faulted = False
            self.connection_text.set("模拟设备 · 已连接" if event["demo"] else f"实机 · {self.port.get()}")
            if self.selected not in self.monitored:
                self.tree.selection_set(str(self.monitored[0]))
            self._send("select", id=self.selected if self.selected in self.monitored else self.monitored[0])
            self._log("模拟设备已连接，不使用串口。" if event["demo"] else
                      f"总线已连接。所有目标默认禁用。通信波特率 {event.get('baudrate', 6000000) / 1e6:g} Mbps。")
        elif kind == "snapshot":
            self.snapshots[mid] = event
            self.faulted = event.get("faulted", self.faulted)
            if self.faulted:
                self._cancel_slider()
            if event["armed"]:
                self.armed_id = mid
            elif self.armed_id == mid:
                self.armed_id = None
            if event.get("holding", False):
                self.holding_ids.add(mid)
            else:
                self.holding_ids.discard(mid)
        elif kind == "armed":
            self._arm_pending = False
            self.armed_id = mid
            self.requested_deg = self._joint_angle(mid, event["raw_pos"])
            if mid == self.selected:
                self._sync_target(self.requested_deg)
            self._log(f"ID {mid} 已从当前位置接管。")
        elif kind == "released":
            self._arm_pending = False
            # 释放会取消尚未执行的记录命令；此时不会再有 captured 结果返回。
            if self._capture_save_id == mid:
                self._capture_save_id = None
            self._cancel_slider()
            if self.armed_id == mid:
                self.armed_id = None
            self._log(f"ID {mid}：" + ("零力矩请求已获反馈。" if event["confirmed"] else "未确认释放，请检查设备状态。"))
            if not event["confirmed"]:
                self._show_action(f"ID {mid} 释放未获确认，请检查设备状态。", "error")
        elif kind == "held":
            self.requested_deg = self._joint_angle(mid, event["raw_pos"])
            if mid == self.selected:
                self._sync_target(self.requested_deg)
            self._show_action(f"ID {mid} 已切换为当前位置保持。", "success")
        elif kind == "captured":
            p = self.profiles[mid]
            p.update(zero_offset=event["zero_offset"], reference_expos=event["expos"],
                     calibrated_at=event["calibrated_at"], verified=False)
            self.dirty = True
            if mid == self.selected:
                self.requested_deg = 0.0
                self.target.set("0.0")
                self._show_profile()
            self._log(f"ID {mid} 软件零位已记录为 {event['zero_offset']:.7f} rad；尚未保存到文件。")
            if self._capture_save_id == mid:
                self._capture_save_id = None
                try:
                    # 磁盘写入失败时保留上面的内存记录和 dirty 标记，允许手动重试。
                    save_profiles(self.config_path, self.profiles, simulation=self.demo.get())
                    self.dirty = False
                    self._log(f"ID {mid} 的0°已保存至 {self.config_path.name}。")
                except (OSError, ValueError) as exc:
                    self._error(exc)
        elif kind == "home_state":
            self.homing = event["active"]
            self.holding_ids = set(event["holding_ids"])
            if not self.homing and self.selected in self.holding_ids:
                self._sync_target(0)
            self.home_text.set(event["message"])
            self._log(event["message"])
        elif kind == "disconnected":
            self._cancel_slider()
            self.connected = self.connecting = False
            self.armed_id = None
            self._arm_pending = False
            self.homing = False
            self.holding_ids.clear()
            self._capture_save_id = None
            self.connection_text.set("已断开" if event["confirmed"] else "已断开 · 释放未确认")
            self._log("连接已关闭。" if event["confirmed"] else "连接已关闭；设备是否释放未得到确认。")
            if self.closing and not event["confirmed"]:
                messagebox.showwarning("释放未确认", "串口已关闭，但没有确认全部舵机收到零力矩命令。请检查设备并按需要切断电机电源。", parent=self.root)
        elif kind == "error":
            self._arm_pending = False
            self._cancel_slider()
            self._capture_save_id = None
            self.connecting = False
            if not self.connected:
                self.connection_text.set("连接未完成")
            self._log("操作未完成：" + event["message"])
            self._show_action("操作未完成：" + event["message"], "error")
        elif kind == "status":
            self._log(event["message"])
        elif kind == "stopped":
            self._stopped = True

    def _poll(self):
        processed = False
        try:
            for _ in range(200):
                event = self.events.get_nowait()
                processed = True
                self._handle(event)
        except queue.Empty:
            pass
        if not self.root.winfo_exists():
            return
        if processed or self.connected or self.connecting or self.closing:
            self._render()
        if self.closing and (self._stopped or not self.controller.is_alive()):
            self.root.destroy()
            return
        self.root.after(50, self._poll)

    def _render(self):
        now = time.monotonic()
        control_busy = self._control_busy()
        for mid, p in self.profiles.items():
            snap = self.snapshots.get(mid)
            fresh = bool(self.connected and snap and now - snap["feedback"]["timestamp"] <= .3)
            state = "未连接" if mid not in self.monitored else "待反馈"
            angle = "—"
            if snap:
                state = "零位保持" if mid in self.holding_ids else "使能" if mid == self.armed_id else "已标定" if p["zero_offset"] is not None else "未标定"
                angle = f"{self._joint_angle(mid, snap['feedback']['raw_pos']):.2f}"
                if not fresh:
                    state = "反馈过期"
            row = (mid, p["name"], state, angle)
            if self._displayed_rows.get(mid) != row:
                self.tree.item(str(mid), values=row)
                self._displayed_rows[mid] = row
        calibrated = self.profiles[self.selected]["zero_offset"] is not None
        update_text(self.position_caption, "零位角度 / °" if calibrated else "临时角度 / °")
        snap = self.snapshots.get(self.selected)
        feedback = snap["feedback"] if snap else None
        fresh = bool(self.connected and feedback and now - feedback["timestamp"] <= .3)
        update_text(self.position_text, f"{self._joint_angle(self.selected, feedback['raw_pos']):+.2f}°" if fresh else "—")
        if self.connected and self.faulted:
            update_text(self.mode_text, "通信异常已锁止 · 检查设备后断开并重新连接")
        elif self.homing:
            update_text(self.mode_text, "正在按记录的零位逐个回零")
        elif self.holding_ids:
            update_text(self.mode_text, "零位保持中 · 全部释放后可手动摆动")
        elif self._arm_pending:
            update_text(self.mode_text, "正在使能当前舵机 · 等待设备反馈")
        elif self.armed_id == self.selected:
            update_text(self.mode_text, "直接跟随目标 · 角度限位已开启" if self.profiles[self.selected]["direct_follow"]
                        else "限速移动 · 按设定速度调整")
        elif not self.connected:
            update_text(self.mode_text, "未连接 · 配置可离线编辑")
        elif self.selected not in self.monitored:
            update_text(self.mode_text, f"ID {self.selected} 未连接 · 断开后添加该 ID")
        elif fresh:
            update_text(self.mode_text, "当前禁用位置控制 · 可读取并记录稳定零位" if calibrated
                        else "未标定 · 0°仅指本次连接姿态")
        else:
            update_text(self.mode_text, "等待新鲜反馈 · 运动操作暂不可用")
        for key, var in self.values.items():
            if not feedback:
                update_text(var, "—")
                continue
            if key == "raw": value = f"{feedback['raw_pos']:.6f}"
            elif key == "expos": value = f"{feedback['expos']:.6f}"
            elif key == "speed": value = f"{math.degrees(feedback['speed']):+.2f}"
            elif key == "torque": value = f"{feedback['torque']:+.3f}"
            elif key == "voltage": value = f"{feedback['voltage']:.1f}"
            elif key == "temp": value = f"{feedback['temp']:.0f}"
            elif key == "age": value = f"{(now-feedback['timestamp'])*1000:.0f}" + ("" if fresh else " · 过期")
            else: value = f"F{feedback['fault']} / W{feedback['warning']} / T{feedback['timeout']}"
            update_text(var, value)
        busy = self.connected or self.connecting or self.closing
        for w in (self.port_box, self.ids_entry, self.demo_check, self.refresh_button, self.connect_button):
            update_options(w, state="disabled" if busy else "normal")
        update_options(self.baudrate_box, state="disabled" if busy else "readonly")
        update_options(self.disconnect_button, state="normal" if self.connected or self.connecting else "disabled",
                       text="取消连接" if self.connecting else "断开")
        selected_armed = self.armed_id == self.selected
        batch = self.homing or bool(self.holding_ids)
        available = self.connected and not self.closing and not self.faulted
        moving = fresh and selected_armed and available and not batch
        update_options(self.arm_button, state="normal" if fresh and not control_busy and available else "disabled")
        for w in [self.target_entry, self.angle_slider, self.execute_button, self.hold_button, *self.jog_buttons]:
            update_options(w, state="normal" if moving else "disabled")
        update_options(self.release_button, state="normal" if self.connected and self.selected in self.monitored and not batch else "disabled")
        update_options(self.stop_button, state="normal" if self.connected else "disabled")
        update_options(self.capture_button, state="normal" if fresh and not control_busy and available else "disabled")
        update_options(self.verify_button, state="normal" if fresh and not control_busy and available and self.profiles[self.selected]["zero_offset"] is not None else "disabled")
        ready = available and not control_busy
        update_options(self.current_home_button, state="normal" if ready and fresh and self.profiles[self.selected]["zero_offset"] is not None else "disabled")
        update_options(self.home_all_button, state="normal" if ready and any(p["zero_offset"] is not None for p in self.profiles.values()) else "disabled")
        update_options(self.home_stop_button, state="normal" if self.connected and batch else "disabled")
        can_clear = not (control_busy or self.connecting or self.closing or self.connected and self.faulted)
        update_options(self.clear_zero_button, state="normal" if can_clear and self.profiles[self.selected]["zero_offset"] is not None else "disabled")
        update_options(self.clear_all_zeros_button, state="normal" if can_clear and any(p["zero_offset"] is not None for p in self.profiles.values()) else "disabled")
        for key, widget in self.settings_widgets:
            model_field = key in {"model_joint", "model_direction"}
            disabled = control_busy or model_field and not self._model_options
            update_options(widget, state="disabled" if disabled else "readonly"
                           if model_field or key in {"direction", "follow_mode"} else "normal")
        update_options(self.apply_button, state="disabled" if control_busy else "normal")
        deployment_idle = self._deployment_idle()
        update_options(self.import_deployment_button, state="normal" if deployment_idle else "disabled")
        update_options(self.export_deployment_button,
                       state="normal" if deployment_idle and not self.demo.get() else "disabled")
        count = sum(bool(p["deployment_joint"]) for p in self.profiles.values())
        update_text(self.deployment_text, f"部署 {count}/15" + (" · 模拟" if self.demo.get() else ""))
        refresh_chrome(self)

    def _close(self):
        if self.closing:
            return
        if self.dirty and not messagebox.askyesno("关闭工具", "零位或设置尚未保存。放弃改动并关闭？", parent=self.root):
            return
        self.closing = True
        if self.robot_view:
            self.robot_view.close()
        self._cancel_slider()
        self.connection_text.set("正在请求零力矩并关闭…")
        self.controller.send("shutdown")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true", help="启动不使用串口的模拟设备")
    parser.add_argument("--config", type=Path, help="指定配置文件，模拟与实机配置不可混用")
    args = parser.parse_args()
    root = tk.Tk()
    CalibratorApp(root, demo=args.demo, config_path=args.config)
    root.mainloop()


if __name__ == "__main__":
    main()

