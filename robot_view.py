"""只读机械预览：管理 CAD 联动与显示参考，不发送舵机指令或执行动力学仿真。"""
from __future__ import annotations

import json
import hashlib
import math
import os
from pathlib import Path
import queue
import tempfile
import threading
import time
import tkinter as tk
from tkinter import font as tkfont, messagebox, ttk
import xml.etree.ElementTree as ET
from ui import BODY_FONT, PALETTES, FeedbackButton, show_action, theme_widget, workspace_style
from zero_coordinates import wrap_pi

try:
    import numpy as np
except ImportError:
    np = None

ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT / "assets" / "robot" / "model.json"
BINDINGS_PATH = ROOT / "configs" / "model_bindings.json"

# 这些参数只影响三维显示，不改变串口频率、舵机速度或控制目标。
FRAME_INTERVAL_MS = 33
LOAD_POLL_MS = 30
RESIZE_DELAY_SECONDS = .2
MAX_RENDER_SIZE = (1000, 800)
CAMERA_DISTANCE_FACTOR = 1.6
TRANSPARENT_ALPHA = .35


def _array(value, size):
    result = np.asarray(value, dtype=float)
    if result.size != size or not np.isfinite(result).all():
        raise ValueError("模型包含错误尺寸或非有限数值")
    return result.reshape(-1)


def _rotation(value):
    rotation = _array(value, 9).reshape(3, 3)
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5):
        raise ValueError("模型旋转矩阵不合法")
    return rotation


def reference_pose_angles(scene):
    """CAD joint angles of the physical zero-jig pose, independent of motor feedback."""
    angles = {}
    for joint in scene.get("joints", ()):
        offset = joint.get("zero_pose_offset_rad", 0)
        if type(offset) not in (int, float) or not math.isfinite(offset):
            raise ValueError(f"关节支架参考角度必须为有限弧度值：{joint['key']}")
        angles[joint["key"]] = float(offset)
    return angles


def load_scene(path=MODEL_PATH):
    if np is None:
        raise RuntimeError("3D 预览需要当前 Python 环境中的 NumPy")
    path = Path(path)
    source = path.read_bytes()
    scene = json.loads(source.decode("utf-8-sig"))
    if scene.get("version") != 1 or scene.get("units") != "m":
        raise ValueError("模型必须使用 version=1、units=m")
    compatible = scene.get("compatible_bindings_sha256", [])
    if not isinstance(compatible, list) or any(
            not isinstance(value, str) or len(value) != 64
            or any(c not in "0123456789abcdef" for c in value) for value in compatible):
        raise ValueError("模型绑定兼容记录无效")
    _rotation(scene.get("display_rotation", np.eye(3).reshape(-1)))
    components, joints = scene.get("components", []), scene.get("joints", [])
    if not components or not joints:
        raise ValueError("模型没有组件或关节")
    names = [c["name"] for c in components]
    keys = [j["key"] for j in joints]
    if len(set(names)) != len(names) or len(set(keys)) != len(keys):
        raise ValueError("模型组件或关节名称重复")
    reference_pose_angles(scene)
    owned = set()
    for component in components:
        transform = _array(component["transform"], 16)
        _rotation(transform[:9])
        if transform[12] <= 0:
            raise ValueError("组件缩放必须为正")
        color = _array(component.get("color", [.65, .7, .75, 1]), 4)
        if ((color < 0) | (color > 1)).any():
            raise ValueError("组件颜色必须位于 0–1")
        mesh = (path.parent / component["mesh"]).resolve()
        if not mesh.is_relative_to(path.parent.resolve()) or not mesh.is_file():
            raise ValueError(f"组件网格缺失或路径无效：{component['name']}")
    lookup = {j["key"]: j for j in joints}
    for joint in joints:
        _array(joint["pivot"], 3)
        axis = _array(joint["axis"], 3)
        if not np.isclose(np.linalg.norm(axis), 1, atol=1e-5):
            raise ValueError(f"关节轴不是单位向量：{joint['key']}")
        parts = joint.get("parts", [])
        if len(set(parts)) != len(parts) or set(parts) - set(names) or owned.intersection(parts):
            raise ValueError("关节刚性组包含缺失或重复所属的组件")
        owned.update(parts)
        if joint.get("motor_component") not in set(names):
            raise ValueError("关节的舵机组件不存在")
        _ancestors(joint["key"], lookup)
    scene["_directory"] = path.parent
    scene["_fingerprint"] = hashlib.sha256(source).hexdigest()
    return scene


def _ancestors(key, joints):
    chain = []
    while key is not None:
        if key not in joints or key in chain:
            raise ValueError("关节父链缺失或存在循环")
        chain.append(key)
        key = joints[key].get("parent")
    return chain[::-1]


def component_poses(scene, angles):
    """World poses from reference-world screw axes; unassigned components stay fixed."""
    joints = {j["key"]: j for j in scene["joints"]}
    owners = {name: j["key"] for j in scene["joints"] for name in j["parts"]}
    display = np.eye(4)
    display[:3, :3] = _rotation(scene.get("display_rotation", np.eye(3).reshape(-1)))
    motions = {}
    for key, joint in joints.items():
        theta = float(angles.get(key, 0))
        if not math.isfinite(theta):
            raise ValueError("预览角度必须为有限数值")
        x, y, z = _array(joint["axis"], 3)
        cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        rotation = np.eye(3) + math.sin(theta) * cross + (1 - math.cos(theta)) * (cross @ cross)
        motion = np.eye(4)
        motion[:3, :3] = rotation
        pivot = _array(joint["pivot"], 3)
        motion[:3, 3] = pivot - rotation @ pivot
        motions[key] = motion
    world_motions = {}
    for key in joints:
        chain = np.eye(4)
        for ancestor in _ancestors(key, joints):
            chain = chain @ motions[ancestor]
        world_motions[key] = chain
    result = {}
    for component in scene["components"]:
        raw = _array(component["transform"], 16)
        pose = np.eye(4)
        pose[:3, :3] = raw[:9].reshape(3, 3).T
        pose[:3, 3] = raw[9:12]
        if component["name"] in owners:
            pose = world_motions[owners[component["name"]]] @ pose
        result[component["name"]] = display @ pose
    return result


def validate_bindings(bindings, joint_keys):
    if not isinstance(bindings, dict):
        raise ValueError("关节绑定必须是对象")
    result, used = {}, set()
    for motor, item in bindings.items():
        if str(motor) not in {str(i) for i in range(15)} or not isinstance(item, dict):
            raise ValueError("绑定 ID 必须为 0–14")
        key, direction, reference = item.get("joint"), item.get("direction"), item.get("reference_raw")
        if key not in joint_keys or key in used:
            raise ValueError("关节绑定缺失或重复")
        if type(direction) is not int or direction not in (-1, 1):
            raise ValueError("绑定方向必须为整数 +1 或 −1")
        if reference is not None and (type(reference) not in (float, int) or not math.isfinite(reference)):
            raise ValueError("模型参考角度必须为有限数值或空值")
        result[str(motor)] = {"joint": key, "direction": direction, "reference_raw": reference}
        used.add(key)
    return result


def _fresh_feedback(snapshots, motor, now=None, *, output_only=False):
    now = time.monotonic() if now is None else now
    snapshot = snapshots.get(int(motor), {})
    feedback = snapshot.get("feedback", {})
    raw = feedback.get("expos") if output_only else feedback.get("model_raw_pos", feedback.get("raw_pos"))
    stamp = feedback.get("timestamp")
    if (type(raw) not in (int, float) or type(stamp) not in (int, float)
            or not math.isfinite(raw) or not math.isfinite(stamp)
            or not -.05 <= now - stamp <= .6
            or snapshot.get("faulted", False)
            or output_only and not 0 <= raw < math.tau
            or not output_only and feedback.get("model_reference_error")
            or feedback.get("mode", 1) != 1
            or any(feedback.get(k, 0) for k in ("fault", "warning", "timeout"))):
        return None
    return dict(feedback, raw_pos=raw)


def feedback_angles(bindings, snapshots, previous=None, now=None, *, zero_references=None, zero_offsets=None):
    """Keep the last pose when feedback is absent, faulty or older than 0.6 seconds."""
    now = time.monotonic() if now is None else now
    angles, stale = dict(previous or {}), set()
    zero_references = zero_references or {}
    zero_offsets = zero_offsets or {}
    for motor, item in bindings.items():
        reference = item["reference_raw"]
        output_reference = zero_references.get(str(motor))
        if output_reference is not None and (type(output_reference) not in (int, float)
                or not math.isfinite(output_reference) or not 0 <= output_reference <= math.tau):
            stale.add(int(motor))
            continue
        if reference is None and output_reference is None:
            if snapshots.get(int(motor), {}).get("feedback", {}).get("model_reference_error"):
                stale.add(int(motor))
                continue
            angles.pop(item["joint"], None)
            continue
        feedback = _fresh_feedback(snapshots, motor, now, output_only=output_reference is not None)
        if feedback is None:
            stale.add(int(motor))
            continue
        if output_reference is not None:
            offset = zero_offsets.get(item["joint"], 0)
            if type(offset) not in (int, float) or not math.isfinite(offset):
                stale.add(int(motor))
                continue
            # 支架零位与 CAD 保存姿态不同；静态偏移仅用于显示，不参与舵机控制。
            angles[item["joint"]] = offset + item["direction"] * wrap_pi(feedback["expos"]-output_reference)
        else:
            # 未标定的旧独立模型参考仍使用原坐标，不叠加支架偏移。
            angles[item["joint"]] = item["direction"] * (feedback["raw_pos"]-reference)
    return angles, stale


def _replace_bindings_file(source, destination):
    """Briefly retry Windows sharing conflicts; keep permanent save failures visible."""
    for attempt in range(5):
        try:
            os.replace(source, destination)
            return
        except PermissionError as exc:
            if getattr(exc, "winerror", None) not in (5, 32, 33) or attempt == 4:
                raise
            time.sleep(.02 * (attempt + 1))


def load_bindings(path, joint_keys, *, simulation, fingerprint, compatible_fingerprints=()):
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("version") != 1:
        raise ValueError("3D 绑定配置版本不支持")
    if type(document.get("simulation")) is not bool or document["simulation"] != simulation:
        raise ValueError("模拟与实机的模型参考不能混用")
    saved_fingerprint = document.get("model_sha256")
    if not isinstance(saved_fingerprint, str) or saved_fingerprint not in (fingerprint, *compatible_fingerprints):
        raise ValueError("模型资产已变化，旧参考未载入；请重新核对绑定和参考姿态")
    return validate_bindings(document.get("bindings"), joint_keys)


class _Renderer:
    @staticmethod
    def compile(scene):
        """Build geometry without creating a window or an OpenGL context."""
        import mujoco
        document = ET.Element("mujoco")
        ET.SubElement(document, "compiler", angle="radian")
        visual = ET.SubElement(document, "visual")
        ET.SubElement(visual, "global", offwidth=str(MAX_RENDER_SIZE[0]), offheight=str(MAX_RENDER_SIZE[1]))
        ET.SubElement(visual, "headlight", ambient=".45 .45 .45", diffuse=".5 .5 .5")
        asset, world = ET.SubElement(document, "asset"), ET.SubElement(document, "worldbody")
        ET.SubElement(asset, "texture", type="skybox", builtin="gradient", width="256", height="256",
                      rgb1=".055 .075 .083", rgb2=".095 .13 .14")
        ET.SubElement(asset, "material", name="cad", specular=".12", shininess=".3")
        poses = component_poses(scene, reference_pose_angles(scene))
        mesh_names = {}
        mesh_assets = {}
        for index, component in enumerate(scene["components"]):
            scale = component["transform"][12]
            mesh_key = (component["mesh"], scale)
            if mesh_key not in mesh_names:
                mesh_names[mesh_key] = f"m{len(mesh_names)}"
                filename = mesh_names[mesh_key] + Path(component["mesh"]).suffix
                mesh_assets[filename] = (scene["_directory"] / component["mesh"]).read_bytes()
                ET.SubElement(asset, "mesh", name=mesh_names[mesh_key],
                    file=filename, scale=f"{scale} {scale} {scale}")
            pose, quat = poses[component["name"]], np.empty(4)
            mujoco.mju_mat2Quat(quat, pose[:3, :3].reshape(-1))
            body = ET.SubElement(world, "body", name=f"c{index}", mocap="true",
                pos=" ".join(map(str, pose[:3, 3])), quat=" ".join(map(str, quat)))
            ET.SubElement(body, "geom", type="mesh", mesh=mesh_names[mesh_key], contype="0", conaffinity="0",
                material="cad",
                rgba=" ".join(map(str, component.get("color", [.65, .7, .75, 1]))))
        return mujoco.MjModel.from_xml_string(ET.tostring(document, encoding="unicode"), assets=mesh_assets)

    def __init__(self, scene, model=None, size=(800, 500)):
        import mujoco
        self.mj, self.scene = mujoco, scene
        self.model = model if model is not None else self.compile(scene)
        self.data = mujoco.MjData(self.model)
        self.theme_mode = "dark"
        self._dark_sky = self.model.tex_data.copy()
        self.renderer = mujoco.Renderer(self.model, height=size[1], width=size[0])
        self.size = size
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self.camera.lookat[:] = self.model.stat.center
        self.camera.distance = max(.12, self.model.stat.extent * CAMERA_DISTANCE_FACTOR)
        self.camera.azimuth, self.camera.elevation = 225, -20
        self.home_distance = self.camera.distance

    def set_theme(self, mode):
        if mode == self.theme_mode:
            return False
        palette = PALETTES[mode]
        self.renderer.close()
        if palette["sky"] is None:
            self.model.tex_data[:] = self._dark_sky
        else:
            self.model.tex_data.reshape(-1, 3)[:] = palette["sky"]
        # Upload the new background once, reusing geometry, camera and pose data.
        self.renderer = self.mj.Renderer(self.model, height=self.size[1], width=self.size[0])
        self.theme_mode = mode
        return True

    def resize(self, size):
        if size == self.size:
            return False
        # Release GL resources in their own context before creating the next one.
        self.renderer.close()
        self.renderer = self.mj.Renderer(self.model, height=size[1], width=size[0])
        self.size = size
        return True

    def frame(self, poses, highlight=None, transparent=False):
        for index, component in enumerate(self.scene["components"]):
            pose, quat = poses[component["name"]], np.empty(4)
            self.mj.mju_mat2Quat(quat, pose[:3, :3].reshape(-1))
            self.data.mocap_pos[index] = pose[:3, 3]
            self.data.mocap_quat[index] = quat
            color = list(PALETTES[self.theme_mode]["highlight"] if component["name"] == highlight
                else component.get("color", [.65, .7, .75, 1]))
            if transparent and component["name"] != highlight:
                color[3] *= TRANSPARENT_ALPHA
            self.model.geom_rgba[index] = color
        self.mj.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, camera=self.camera)
        return self.renderer.render()

    def close(self):
        self.renderer.close()


class RobotView:
    def __init__(self, app, parent=None):
        self.app, self.angles, self.bindings = app, {}, {}
        self.embedded = parent is not None
        self.window = ttk.Frame(parent, style="Plain.TFrame") if self.embedded else tk.Toplevel(app.root)
        if self.embedded:
            self.window.pack(fill="both", expand=True)
        else:
            self.window.title("S288 Studio · 机械关节绑定与联动")
            self.window.geometry("960x740")
            self.window.minsize(900, 680)
            theme_widget(self.window, background="bg")
            self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.renderer = None
        self.binding_window = None
        self.after_id = None
        self._notify_id = None
        self.drag_origin = None
        self._frame_key = None
        self.transparent = False
        self._viewport_size = None
        self._resize_after = 0
        self.notice = ""
        self._binding_rows = {}
        self._demo_generated = set()
        self._reference_save_failed = False
        self._temporary_expos_references = {}
        self.closed = False
        self._show_binding_when_ready = False
        if self.embedded:
            self.loading = ttk.Label(self.window, text="正在载入机械结构…", anchor="center",
                                     style="Muted.TLabel")
            self.loading.pack(fill="both", expand=True)
            self._prepared = queue.Queue(maxsize=1)
            threading.Thread(target=self._prepare, args=(self._prepared,),
                             daemon=True, name="CAD preview loader").start()
            self.after_id = self.window.after(LOAD_POLL_MS, self._finish_prepare)
        else:
            try:
                self._initialize(load_scene())
            except Exception as exc:
                self.close()
                messagebox.showerror("3D 预览无法打开", str(exc), parent=app.root)

    @staticmethod
    def _prepare(result_queue):
        # 后台只持有结果队列，不持有窗口；关闭预览后不会在线程里释放 Tk 对象。
        try:
            scene = load_scene()
            result_queue.put((scene, _Renderer.compile(scene), None))
        except Exception as exc:
            result_queue.put((None, None, exc))

    def _finish_prepare(self):
        self.after_id = None
        if self.closed:
            return
        try:
            scene, model, error = self._prepared.get_nowait()
        except queue.Empty:
            self.after_id = self.window.after(LOAD_POLL_MS, self._finish_prepare)
            return
        try:
            if error:
                raise error
            self.loading.destroy()
            self._initialize(scene, model)
        except Exception as exc:
            renderer, self.renderer = self.renderer, None
            message = f"结构预览未能载入：{exc}"
            if renderer is not None:
                try:
                    renderer.close()
                except Exception as cleanup_error:
                    # 保留最初的载入错误；原生渲染清理失败不能再次中断 Tk 回调。
                    message += f"；渲染资源清理异常：{cleanup_error}"
            if self.loading.winfo_exists():
                self.loading.configure(text=message)
            else:
                ttk.Label(self.window, text=message,
                          style="Muted.TLabel", wraplength=450).pack(padx=20, pady=20)

    def _initialize(self, scene, model=None):
        self.scene = scene
        self.keys = {j["key"] for j in self.scene["joints"]}
        self.simulation = bool(self.app.demo.get())
        self.bindings_path = BINDINGS_PATH.with_name("demo_model_bindings.json") if self.simulation else BINDINGS_PATH
        if self.bindings_path.exists():
            try:
                self.bindings = load_bindings(self.bindings_path, self.keys,
                    simulation=self.simulation, fingerprint=self.scene["_fingerprint"],
                    compatible_fingerprints=self.scene.get("compatible_bindings_sha256", []))
            except (OSError, ValueError) as exc:
                self.notice = str(exc)
        if self.simulation and self.embedded:
            # Virtual examples only; preserve saved bindings and never infer real IDs.
            used = {item["joint"] for item in self.bindings.values()}
            available = [joint["key"] for joint in self.scene["joints"] if joint["key"] not in used]
            motors = [str(self.app.selected)] + [str(i) for i in range(15) if i != self.app.selected]
            missing = [motor for motor in motors if motor not in self.bindings]
            for motor, key in zip(missing, available):
                self.bindings[motor] = {"joint": key, "direction": 1, "reference_raw": None}
                self._demo_generated.add(motor)
        if self.embedded and getattr(self.app, "compact_robot_view", False):
            size = (640, 480) if getattr(self.app, "central_robot_view", False) else (480, 320)
            self.renderer = _Renderer(self.scene, model, size=size)
        else:
            self.renderer = _Renderer(self.scene, model) if model is not None else _Renderer(self.scene)
        self.set_theme()
        camera = self.renderer.camera
        self._home_camera = (camera.azimuth, camera.elevation, camera.distance, tuple(camera.lookat))
        self._build()
        responsive = getattr(self.app, "responsive_ui", None)
        if self.embedded and responsive:
            responsive.register(self.window)
        self._select_id()
        if self._show_binding_when_ready:
            self.show_bindings()
        self.after_id = self.window.after_idle(self._tick)
        self._notify_binding_changed()

    def _notify_binding_changed(self):
        # 多次修改合并成一次界面刷新，关窗时必须取消这个 Tk 全局队列任务。
        if self.closed or self._notify_id is not None:
            return
        callback = getattr(self.app, "_model_binding_changed", None)
        if callable(callback):
            self._notify_id = self.window.after_idle(self._run_binding_callback)

    def _run_binding_callback(self):
        self._notify_id = None
        callback = getattr(self.app, "_model_binding_changed", None)
        if not self.closed and callable(callback):
            callback()

    def binding_options(self):
        return dict(getattr(self, "labels", {}))

    def _software_references(self):
        callback = getattr(self.app, "model_zero_references", None)
        return dict(callback()) if callable(callback) else {}

    def _display_references(self):
        references = self._software_references()
        references.update(self._temporary_expos_references)
        return references

    def reset_temporary_references(self, motor_ids=None):
        if motor_ids is None:
            self._temporary_expos_references.clear()
        else:
            for motor in motor_ids:
                self._temporary_expos_references.pop(str(motor), None)
        self._frame_key = None
        if not self.closed:
            self._update_reference_button()

    def _update_reference_button(self):
        if self.closed or not hasattr(self, "reference_button"):
            return
        try:
            if not self.reference_button.winfo_exists():
                return
        except tk.TclError:
            return
        motor = self.motor.get()
        text = ("恢复软件零位显示" if motor in self._temporary_expos_references else "临时对齐模型参考"
                if motor in self._software_references() else "对齐模型参考")
        if str(self.reference_button.cget("text")) != text:
            self.reference_button.configure(text=text)

    @staticmethod
    def _motor_key(motor_id):
        if str(motor_id) not in {str(i) for i in range(15)}:
            raise ValueError("绑定 ID 必须为 0–14")
        return str(motor_id)

    def get_binding(self, motor_id):
        item = self.bindings.get(self._motor_key(motor_id))
        return dict(item) if item is not None else None

    def preview_joint(self, motor_id, joint_key):
        """Select a visual candidate without saving a binding or driving a motor."""
        motor = self._motor_key(motor_id)
        if not self.binding_options() or joint_key is not None and (type(joint_key) is not str or joint_key not in self.keys):
            raise ValueError("机械模型尚未就绪，或所选关节不存在")
        self.motor.set(motor)
        self.joint.set(next((label for label, key in self.labels.items() if key == joint_key), ""))
        item = self.bindings.get(motor)
        self.direction.set("−1" if item and item["joint"] == joint_key and item["direction"] == -1 else "+1")
        self._sync_joint_selection()
        self._frame_key = None

    def set_binding(self, motor_id, joint_key, display_direction=1):
        """Atomically save this ID's display mapping; do not access motor control."""
        motor = self._motor_key(motor_id)
        if not self.binding_options() or joint_key is not None and (type(joint_key) is not str or joint_key not in self.keys):
            raise ValueError("机械模型尚未就绪，或所选关节不存在")
        if type(display_direction) is not int or display_direction not in (-1, 1):
            raise ValueError("显示方向必须为整数 +1 或 −1")
        bindings = {mid: dict(item) for mid, item in self.bindings.items()}
        generated = set(self._demo_generated)
        old = bindings.get(motor)
        if joint_key is None:
            bindings.pop(motor, None)
        else:
            owner = next((mid for mid, item in bindings.items()
                          if item["joint"] == joint_key and mid != motor), None)
            if owner is not None:
                if owner not in generated:
                    raise ValueError("请先解除该关节已有的 ID 绑定。")
                bindings.pop(owner)
                generated.discard(owner)
            reference = old["reference_raw"] if old and old["joint"] == joint_key else None
            if reference is None and motor not in self._software_references() and getattr(self.app, "connected", True):
                feedback = _fresh_feedback(self.app.snapshots, motor)
                if feedback is not None:
                    reference = feedback["raw_pos"]
            bindings[motor] = {"joint": joint_key, "direction": display_direction, "reference_raw": reference}
        generated.discard(motor)
        if bindings != self.bindings or generated != self._demo_generated:
            self._persist(bindings, generated)
        self.preview_joint(motor, joint_key)
        self._notify_binding_changed()
        return True

    def set_theme(self):
        if self.renderer is not None:
            change = getattr(self.renderer, "set_theme", None)
            if change:
                change(getattr(self.app.root, "_theme_mode", "dark"))
            self._frame_key = None

    def _build(self):
        compact = self.embedded and getattr(self.app, "compact_robot_view", False)
        shell = ttk.Frame(self.window, padding=0 if self.embedded else 16, style="Plain.TFrame")
        shell.pack(fill="both", expand=True)
        top = ttk.Frame(shell, style="Plain.TFrame")
        top.pack(fill="x", padx=12 if compact else 16, pady=(10, 5) if compact else (14, 10))
        ttk.Label(top, text="机械结构", style="Section.TLabel").pack(side="left")
        self.view_mode = tk.StringVar(value="实际反馈")
        mode = ttk.Combobox(top, textvariable=self.view_mode,
                           values=("目标预览", "实际反馈"), width=8, state="readonly",
                           font=BODY_FONT)
        mode.pack(side="right")
        mode.bind("<<ComboboxSelected>>", lambda _: setattr(self, "_frame_key", None))
        if not self.embedded:
            FeedbackButton(top, text="关节绑定", style="Quiet.TButton",
                           command=self.toggle_bindings).pack(side="right", padx=(0, 8))

        central = self.embedded and getattr(self.app, "central_robot_view", False)
        viewport = ttk.Frame(shell, style="Viewport.TFrame", width=300,
                             height=300 if central else 130 if compact else 160 if self.embedded else 260)
        viewport.pack(fill="both", expand=True)
        viewport.pack_propagate(False)
        self.picture = theme_widget(ttk.Label(viewport, anchor="center"), background="bg")
        self.picture.place(x=6, y=6, relwidth=1, relheight=1, width=-12, height=-12)
        self.picture.bind("<ButtonPress-1>", lambda e: setattr(self, "drag_origin", (e.x, e.y)))
        self.picture.bind("<B1-Motion>", self._rotate)
        self.picture.bind("<MouseWheel>", self._zoom)
        camera_tools = ttk.Frame(viewport, style="Plain.TFrame")
        camera_tools.place(relx=.5, rely=1, anchor="s", y=-3)
        for key, label in (("front", "正面"), ("side", "侧面"), ("top", "俯视"), ("reset", "视角复位")):
            FeedbackButton(camera_tools, text=label, width=0, style="Quiet.TButton",
                       command=lambda choice=key: self._camera_preset(choice)).pack(side="left", padx=2)
        self.transparency_button = FeedbackButton(camera_tools, text="半透明", width=-8,
            style="Quiet.TButton", command=self._toggle_transparency)
        self.transparency_button.pack(side="left", padx=(8, 2))
        if compact:
            self.binding_window = theme_widget(tk.Toplevel(self.window), background="bg")
            self.binding_window.withdraw()
            self.binding_window.title("S288 Studio · 机械关节绑定")
            self.binding_window.geometry("520x430")
            self.binding_window.minsize(460, 380)
            self.binding_window.protocol("WM_DELETE_WINDOW", self.binding_window.withdraw)
        self.binding_panel = ttk.Frame(self.binding_window or shell, padding=(14, 10), style="Plain.TFrame")
        if compact:
            self.binding_panel.pack(fill="both", expand=True)
        elif not self.embedded:
            self.binding_panel.pack(fill="x")
        row = ttk.Frame(self.binding_panel, style="Plain.TFrame")
        row.pack(fill="x", pady=(0, 8))
        self.motor = tk.StringVar(value=str(self.app.selected))
        self.follow = tk.BooleanVar(value=True)
        self.joint = tk.StringVar()
        self.direction = tk.StringVar(value="+1")
        ttk.Label(row, text="ID", style="Muted.TLabel").pack(side="left")
        self.motor_box = ttk.Combobox(row, textvariable=self.motor, values=list(range(15)), width=4, state="disabled")
        self.motor_box.pack(side="left", padx=6)
        self.motor_box.bind("<<ComboboxSelected>>", lambda _: self._select_id())
        ttk.Checkbutton(row, text="跟随主界面ID", variable=self.follow, command=self._select_id).pack(side="left", padx=(0, 10))
        labels = [f"{j['label']} · M{j['motor_component'].rsplit('-', 1)[-1]}" for j in self.scene["joints"]]
        self.labels = dict(zip(labels, [j["key"] for j in self.scene["joints"]]))
        joint_row = ttk.Frame(self.binding_panel, style="Plain.TFrame")
        joint_row.pack(fill="x", pady=(0, 8))
        joint_box = ttk.Combobox(joint_row, textvariable=self.joint, values=labels, state="readonly", width=17)
        joint_box.pack(side="left", fill="x", expand=True)
        joint_box.bind("<<ComboboxSelected>>", lambda _: self._sync_joint_selection())
        ttk.Label(joint_row, text="显示方向", style="Muted.TLabel").pack(side="left", padx=(6, 0))
        ttk.Combobox(joint_row, textvariable=self.direction, values=("+1", "−1"), state="readonly", width=4).pack(side="left", padx=6)
        FeedbackButton(joint_row, text="绑定", style="Primary.TButton", command=self._bind).pack(side="left")
        row2 = ttk.Frame(self.binding_panel, style="Plain.TFrame")
        row2.pack(fill="x", pady=(0, 8))
        self.reference_button = FeedbackButton(row2, text="对齐模型参考", style="Quiet.TButton", command=self._capture)
        self.reference_button.pack(side="left")
        FeedbackButton(row2, text="解除绑定", style="Quiet.TButton", command=self._unbind).pack(side="left", padx=8)
        ttk.Label(row2, text="M 为装配编号", style="Muted.TLabel").pack(side="right")
        table = ttk.Frame(self.binding_panel, style="Plain.TFrame")
        table.pack(fill="both", expand=True)
        self.binding_table = ttk.Treeview(table, columns=("joint", "id"),
            show="headings", selectmode="browse", height=5 if self.embedded else 7)
        self.binding_table.heading("joint", text="模型关节")
        self.binding_table.heading("id", text="舵机 ID")
        self.binding_table.column("joint", width=145, stretch=True)
        self.binding_table.column("id", width=60, anchor="center", stretch=False)
        self.binding_table.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(table, orient="vertical", command=self.binding_table.yview)
        scrollbar.pack(side="right", fill="y")
        self.binding_table.configure(yscrollcommand=scrollbar.set)
        for joint in self.scene["joints"]:
            self.binding_table.insert("", "end", iid=joint["key"], values=(joint["label"], "未绑定"))
        self.binding_table.bind("<<TreeviewSelect>>", self._select_joint)
        self._refresh_bindings()
        self.footer = ttk.Frame(shell, padding=(12, 6) if compact else (16, 9), style="Plain.TFrame")
        self.footer.pack(fill="x")
        if compact:
            # A changing preview notice must not push the camera viewport up and down.
            self.footer._status_lines, self.footer._status_extra_height = 3, 12
            self.footer.configure(width=1, height=tkfont.Font(root=self.window, font=BODY_FONT).metrics("linespace") * 3 + 12)
            self.footer.pack_propagate(False)
        self.status = tk.StringVar()
        self.status_label = ttk.Label(self.footer, textvariable=self.status, style="Muted.TLabel",
                                      wraplength=330 if compact else 400 if self.embedded else 550,
                                      anchor="nw")
        self.status_label.pack(anchor="w", fill="both", expand=compact)
        if not compact:
            ttk.Label(self.footer, text="拖动旋转 · 滚轮缩放 · 显示相对动作", style="Muted.TLabel").pack(anchor="w", pady=(3, 0))

    def show_bindings(self):
        if not self.renderer:
            self._show_binding_when_ready = True
            show_action(self.app.root, "机械结构正在加载，完成后打开绑定窗口。")
        elif self.binding_window:
            self.binding_window.deiconify()
            self.binding_window.lift()
            show_action(self.app.root, "已打开关节绑定窗口。", "success")
        elif not self.binding_panel.winfo_manager():
            self.binding_panel.pack(fill="x", before=self.footer)
            show_action(self.app.root, "已展开关节绑定。", "success")

    def toggle_bindings(self):
        if self.binding_window:
            if self.binding_window.state() == "withdrawn":
                self.show_bindings()
            else:
                self.binding_window.withdraw()
                show_action(self.app.root, "已收起关节绑定窗口。")
        elif self.binding_panel.winfo_manager():
            self.binding_panel.pack_forget()
            show_action(self.app.root, "已收起关节绑定。")
        else:
            self.show_bindings()

    def _select_id(self):
        if self.follow.get():
            self.motor.set(str(self.app.selected))
        self.motor_box.configure(state="disabled" if self.follow.get() else "readonly")
        item = self.bindings.get(self.motor.get())
        key = item["joint"] if item else next(iter(self.labels.values()))
        self.joint.set(next(label for label, value in self.labels.items() if value == key))
        self.direction.set("−1" if item and item["direction"] == -1 else "+1")
        self._sync_joint_selection()
        self._update_reference_button()

    def _sync_joint_selection(self):
        key = self.labels.get(self.joint.get())
        if key and self.binding_table.selection() != (key,):
            self.binding_table.selection_set(key)
            self.binding_table.see(key)
        elif not key and self.binding_table.selection():
            self.binding_table.selection_remove(*self.binding_table.selection())

    def _select_joint(self, _=None):
        selected = self.binding_table.selection()
        if selected:
            key = selected[0]
            self.joint.set(next(label for label, value in self.labels.items() if value == key))

    def _refresh_bindings(self):
        owners = {item["joint"]: motor for motor, item in self.bindings.items()}
        for joint in self.scene["joints"]:
            key = joint["key"]
            row = (joint["label"], owners.get(key, "未绑定"))
            if self._binding_rows.get(key) != row:
                self.binding_table.item(key, values=row)
                self._binding_rows[key] = row

    def _save_bindings(self, bindings, generated):
        bindings = validate_bindings(bindings, self.keys)
        self.bindings_path.parent.mkdir(parents=True, exist_ok=True)
        fd, name = tempfile.mkstemp(dir=self.bindings_path.parent, suffix=".tmp")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump({"version": 1, "simulation": self.simulation,
                    "model_sha256": self.scene["_fingerprint"],
                    "bindings": {motor: item for motor, item in bindings.items() if motor not in generated}},
                    stream, ensure_ascii=False, indent=2, allow_nan=False)
            _replace_bindings_file(name, self.bindings_path)
        finally:
            if os.path.exists(name):
                os.unlink(name)
        return bindings

    def _persist(self, bindings, generated):
        """先保存候选副本，再提交内存；失败时保留原绑定、参考和画面。"""
        bindings = self._save_bindings(bindings, generated)
        self.bindings, self._demo_generated = bindings, set(generated)
        references = self._software_references()
        active = {item["joint"] for motor, item in bindings.items()
                  if item["reference_raw"] is not None or motor in references}
        self.angles = {key: value for key, value in self.angles.items() if key in active}
        self.notice = ""
        self._reference_save_failed = False
        self._refresh_bindings()

    def _bind(self):
        try:
            key = self.labels.get(self.joint.get())
            if key is None:
                raise ValueError("请先选择要绑定的模型关节。")
            self.set_binding(self.motor.get(), key,
                             -1 if self.direction.get() == "−1" else 1)
            show_action(self.app.root, f"ID {self.motor.get()} 的模型绑定已保存。", "success")
        except (ValueError, OSError) as exc:
            show_action(self.app.root, f"模型绑定未保存：{exc}", "error")
            messagebox.showerror("绑定未保存", str(exc), parent=self.window)

    def _initialize_references(self):
        if not getattr(self.app, "connected", True) or self._reference_save_failed:
            return
        bindings = {motor: dict(item) for motor, item in self.bindings.items()}
        changed = []
        for motor, item in bindings.items():
            if item["reference_raw"] is None and motor not in self._software_references():
                feedback = _fresh_feedback(self.app.snapshots, motor)
                if feedback is not None:
                    item["reference_raw"] = feedback["raw_pos"]
                    changed.append(motor)
        if not changed:
            return
        if all(motor in self._demo_generated for motor in changed):
            self.bindings = bindings  # 模拟示例参考只放在内存，不写成用户绑定。
            return
        try:
            self._persist(bindings, self._demo_generated)
        except (OSError, ValueError) as exc:
            # 自动保存失败只提示一次，避免每帧写盘或弹窗；手动绑定/对齐可重试。
            self._reference_save_failed = True
            self.notice = "模型参考未保存；检查目录后重新绑定或对齐参考。"
            log = getattr(self.app, "_log", None)
            if callable(log):
                log(f"模型参考未保存：{exc}")

    def _unbind(self):
        try:
            self.set_binding(self.motor.get(), None)
            show_action(self.app.root, f"ID {self.motor.get()} 的模型绑定已解除并保存。", "success")
        except (ValueError, OSError) as exc:
            show_action(self.app.root, f"模型绑定未保存：{exc}", "error")
            messagebox.showerror("绑定未保存", str(exc), parent=self.window)

    def _capture(self):
        motor = self.motor.get()
        item = self.bindings.get(motor)
        if motor in self._software_references():
            if motor in self._temporary_expos_references:
                self.reset_temporary_references([motor])
                show_action(self.app.root, f"ID {motor} 已恢复按保存的软件零位显示。", "success")
                return
            feedback = _fresh_feedback(self.app.snapshots, motor, output_only=True) if getattr(self.app, "connected", True) else None
            if not item or feedback is None:
                show_action(self.app.root, "无法临时对齐模型：请先绑定，并等待正常反馈。", "warning")
                messagebox.showinfo("无法临时对齐", "请先绑定关节，并等待新鲜正常反馈。", parent=self.window)
                return
            if messagebox.askyesno("临时对齐模型参考", "仅将当前姿态临时作为模型显示零位，不修改机械零位、方向或参数文件。\n"
                    "可用同一按钮恢复；断开、载入配置或重新记录零位后自动恢复软件零位显示。", parent=self.window):
                feedback = _fresh_feedback(self.app.snapshots, motor, output_only=True) if getattr(self.app, "connected", True) else None
                if feedback is None:
                    show_action(self.app.root, "临时模型参考未应用：反馈已过期或连接已断开。", "warning")
                    return
                self._temporary_expos_references[motor] = feedback["expos"]
                self._frame_key = None
                self._update_reference_button()
                show_action(self.app.root, f"ID {motor} 临时模型参考已应用；软件零位及参数文件未改。", "success")
            else:
                show_action(self.app.root, "已取消临时对齐模型。")
            return
        feedback = _fresh_feedback(self.app.snapshots, motor) if getattr(self.app, "connected", True) else None
        if not item or feedback is None:
            show_action(self.app.root, "无法记录模型参考：请先绑定，并等待正常反馈。", "warning")
            messagebox.showinfo("无法记录参考", "请先绑定关节，并等待该 ID 的新鲜正常反馈。", parent=self.window)
            return
        if messagebox.askyesno("确认模型参考姿态", "请确认实物关节已与模型参考姿态对齐。此操作只记录预览参考，不修改软件零位或电机控制。", parent=self.window):
            # 确认框期间反馈可能变化；点“是”后重新读取并校验。
            feedback = _fresh_feedback(self.app.snapshots, motor) if getattr(self.app, "connected", True) else None
            if feedback is None:
                show_action(self.app.root, "模型参考未记录：反馈已过期或连接已断开。", "warning")
                messagebox.showinfo("无法记录参考", "反馈已过期或连接已断开，请等待正常反馈后重试。", parent=self.window)
                return
            bindings = {mid: dict(binding) for mid, binding in self.bindings.items()}
            bindings[motor]["reference_raw"] = feedback["raw_pos"]
            generated = self._demo_generated - {motor}
            try:
                self._persist(bindings, generated)
                self.angles[item["joint"]] = 0
                show_action(self.app.root, f"ID {motor} 的模型参考已保存。", "success")
            except (OSError, ValueError) as exc:
                show_action(self.app.root, f"模型参考未保存：{exc}", "error")
                messagebox.showerror("绑定未保存", str(exc), parent=self.window)
        else:
            show_action(self.app.root, "已取消模型参考对齐。")

    def _rotate(self, event):
        if self.drag_origin and self.renderer:
            dx, dy = event.x-self.drag_origin[0], event.y-self.drag_origin[1]
            self.renderer.camera.azimuth += dx*.5
            self.renderer.camera.elevation = max(-89, min(89, self.renderer.camera.elevation+dy*.4))
            self.drag_origin = event.x, event.y

    def _zoom(self, event):
        if not self.renderer:
            return
        camera = self.renderer.camera
        camera.distance = max(self.renderer.home_distance*.1, min(self.renderer.home_distance*10,
            camera.distance * math.exp(-event.delta/120*.12)))

    def _camera_preset(self, name):
        """Change the viewing camera only; servo targets and reference angles stay intact."""
        if not self.renderer:
            return
        camera = self.renderer.camera
        if name == "reset":
            camera.azimuth, camera.elevation, camera.distance, lookat = self._home_camera
            camera.lookat[:] = lookat
        else:
            camera.azimuth, camera.elevation = {"front": (90, -5), "side": (0, -5), "top": (90, -89)}[name]
        self._frame_key = None
        label = {"front": "正面", "side": "侧面", "top": "俯视", "reset": "默认"}[name]
        show_action(self.app.root, f"已切换为{label}视角。", "success")

    def _toggle_transparency(self):
        self.transparent = not self.transparent
        self.transparency_button.configure(text="恢复实体" if self.transparent else "半透明",
            style=workspace_style(self.transparency_button, "Soft.TButton" if self.transparent else "Quiet.TButton"))
        self._frame_key = None
        show_action(self.app.root, "已开启半透明显示。" if self.transparent else "已恢复实体显示。", "success")

    def _tick(self):
        from PIL import Image, ImageOps, ImageTk
        started = time.monotonic()
        if self.closed or self.renderer is None:
            return
        # Wait for Tk layout before caching the first static preview frame.
        if not self.picture.winfo_ismapped() or min(self.picture.winfo_width(), self.picture.winfo_height()) <= 1:
            self.after_id = self.window.after(FRAME_INTERVAL_MS, self._tick)
            return
        if self.follow.get() and self.motor.get() != str(self.app.selected):
            self._select_id()
        self._initialize_references()
        references = self._display_references()
        zero_offsets = reference_pose_angles(self.scene)
        self._update_reference_button()
        self.angles, stale = feedback_angles(self.bindings, self.app.snapshots, self.angles,
            zero_references=references, zero_offsets=zero_offsets)
        item = self.bindings.get(self.motor.get())
        selected_key = self.labels.get(self.joint.get())
        selected_joint = next((j for j in self.scene["joints"] if j["key"] == selected_key), None)
        highlighted = selected_joint["motor_component"] if selected_joint else None
        try:
            self.display_angles = dict(zero_offsets)
            self.display_angles.update(self.angles)
            preview_motor = None
            target = getattr(self.app, "preview_target_expos", None)
            output_preview = callable(target)
            if not output_preview:
                # Read-only embedding clients may still supply an explicit model-coordinate preview.
                target = getattr(self.app, "preview_target_raw", None)
            if self.view_mode.get() == "目标预览" and callable(target):
                requested = target()
                if requested is not None:
                    motor, coordinate = requested
                    binding = self.bindings.get(str(motor))
                    output_reference = references.get(str(motor)) if output_preview else None
                    if (binding and (binding["reference_raw"] is not None or output_reference is not None)
                            and _fresh_feedback(self.app.snapshots, motor, output_only=output_reference is not None) is not None
                            and math.isfinite(coordinate)):
                        if output_reference is not None:
                            angle = wrap_pi(coordinate-output_reference)
                        else:
                            model_coordinate = self.app.model_position_from_expos(motor, coordinate) if output_preview else coordinate
                            angle = model_coordinate-binding["reference_raw"]
                        self.display_angles[binding["joint"]] = binding["direction"] * angle + (
                            zero_offsets.get(binding["joint"], 0) if output_reference is not None else 0)
                        preview_motor = motor
            width, height = max(50, self.picture.winfo_width()), max(50, self.picture.winfo_height())
            if self._viewport_size != (width, height):
                self._viewport_size = (width, height)
                self._resize_after = started + RESIZE_DELAY_SECONDS
            elif started >= self._resize_after:
                # Reuse the model; rebuild the render buffer only after resizing settles.
                scale = min(1, MAX_RENDER_SIZE[0] / width, MAX_RENDER_SIZE[1] / height)
                size = (max(1, round(width * scale)), max(1, round(height * scale)))
                resize = getattr(self.renderer, "resize", None)
                if resize and resize(size):
                    self._frame_key = None
            camera = self.renderer.camera
            frame_key = (tuple(sorted(self.display_angles.items())), highlighted, self.transparent, width, height,
                         camera.azimuth, camera.elevation, camera.distance, tuple(camera.lookat))
            if frame_key != self._frame_key:
                # 只有姿态、相机或显示状态变化才重绘；静止反馈继续复用图片。
                pixels = self.renderer.frame(component_poses(self.scene, self.display_angles), highlighted, self.transparent)
                image = ImageOps.fit(Image.fromarray(pixels), (width, height),
                                     method=Image.Resampling.BILINEAR)
                self.photo = ImageTk.PhotoImage(image, master=self.window)
                self.picture.configure(image=self.photo)
                self._frame_key = frame_key
            joint_label = next((j["label"] for j in self.scene["joints"] if item and j["key"] == item["joint"]), "")
            display_offset = zero_offsets.get(item["joint"], 0) if item and self.motor.get() in references else 0
            relative_actual = self.angles.get(item["joint"], display_offset)-display_offset if item else 0
            message = (f"ID {self.motor.get()} 未绑定，未参与三维联动；M 是装配编号，与通信 ID 无关" if not item else
                       f"ID {self.motor.get()} → {joint_label} · 已绑定，等待首次正常反馈设为联动起点" if item["reference_raw"] is None and self.motor.get() not in references else
                       f"ID {self.motor.get()} → {joint_label} · {'相对支架角度' if self.motor.get() in references else '相对角度'} {math.degrees(relative_actual):+.1f}°")
            if preview_motor is not None:
                binding = self.bindings[str(preview_motor)]
                preview_offset = zero_offsets.get(binding["joint"], 0) if str(preview_motor) in references else 0
                message = (f"ID {preview_motor} · 目标预览 {math.degrees(self.display_angles[binding['joint']]-preview_offset):+.1f}°"
                           f" · 实际反馈 {math.degrees(self.angles.get(binding['joint'], preview_offset)-preview_offset):+.1f}°")
            # 目标预览可能跟随主窗口 ID，而绑定编辑器停在另一个 ID。
            status_motor = str(preview_motor) if preview_motor is not None else self.motor.get()
            if status_motor in self._demo_generated:
                message += " · 模拟示例绑定，与实机 ID 无关"
            if status_motor in self._temporary_expos_references:
                message += " · 临时模型参考，可恢复软件零位显示"
            elif status_motor in references:
                message += " · 软件零位参考"
            if stale:
                message += " · 反馈过期／异常，冻结：" + ", ".join(map(str, sorted(stale)))
            if self.notice:
                message += " · " + self.notice
            if self.status.get() != message:
                self.status.set(message)
            wrap = max(180, min(440 if self.embedded else 850, width-30))
            if int(self.status_label.cget("wraplength")) != wrap:
                self.status_label.configure(wraplength=wrap)
            elapsed_ms = int((time.monotonic()-started)*1000)
            self.after_id = self.window.after(max(1, FRAME_INTERVAL_MS-elapsed_ms), self._tick)
        except Exception as exc:
            self.status.set(f"预览暂停：{exc}")

    def close(self):
        if self.closed:
            return
        self.closed = True
        for name in ("after_id", "_notify_id"):
            callback = getattr(self, name)
            setattr(self, name, None)
            if callback is not None:
                try:
                    self.window.after_cancel(callback)
                except tk.TclError:
                    pass
        renderer, self.renderer = self.renderer, None
        failure = None
        try:
            if renderer is not None:
                renderer.close()
        except Exception as exc:
            failure = exc
        finally:
            for window in (self.binding_window, self.window):
                try:
                    if window is not None and window.winfo_exists():
                        window.destroy()
                except tk.TclError:
                    pass
        if failure is not None:
            raise failure
