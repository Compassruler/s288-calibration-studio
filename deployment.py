"""部署布局和标定文件转换；只处理文件与数值，不访问串口。"""

from __future__ import annotations

import copy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile

from config import validate_profile
from protocol import RATIO, validate_baudrate


# 原例程固定的十四关节布局，不根据 CAD 编号或舵机 ID 猜测关节名称。
REFERENCE_JOINT_TO_MOTOR_ID = {
    "head_roll": 0, "head_yaw": 1, "head_pitch": 2, "neck_pitch": 3,
    "right_hip_yaw": 4, "left_hip_yaw": 5, "left_hip_roll": 6,
    "left_hip_pitch": 7, "left_knee": 8, "left_ankle": 9,
    "right_hip_roll": 10, "right_hip_pitch": 11, "right_knee": 12,
    "right_ankle": 13,
}
REFERENCE_POLICY_JOINT_ORDER = (
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll", "right_hip_yaw",
    "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
)
REFERENCE_POSE = "mujoco_reset_qpos_zero"


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"JSON 包含重复字段：{key}")
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError(f"JSON 不能包含 {value}")


def _read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8-sig"),
                          object_pairs_hook=_unique_object, parse_constant=_reject_constant)
    except json.JSONDecodeError as exc:
        raise ValueError(f"JSON 格式错误：第 {exc.lineno} 行") from exc


def _validated_layout(document):
    if not isinstance(document, dict):
        raise ValueError("部署布局必须是 JSON 对象")
    mapping, order = document.get("joint_to_motor_id"), document.get("policy_joint_order")
    if not isinstance(mapping, dict) or len(mapping) not in (14, 15):
        raise ValueError("部署布局必须完整包含 14 或 15 个关节")
    if any(not isinstance(name, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name)
           for name in mapping):
        raise ValueError("部署关节名只能使用英文字母、数字和下划线，且不能以数字开头")
    ids = list(mapping.values())
    if any(type(mid) is not int for mid in ids) or set(ids) != set(range(len(mapping))):
        raise ValueError("布局 ID 必须唯一且完整覆盖 0–13 或 0–14")
    if (not isinstance(order, (list, tuple)) or len(order) != len(mapping)
            or any(not isinstance(name, str) for name in order)
            or len(set(order)) != len(order) or set(order) != set(mapping)):
        raise ValueError("策略关节顺序必须恰好包含所有部署关节，不能缺失或重复")
    return {"joint_to_motor_id": dict(mapping), "policy_joint_order": list(order)}


def load_layout(path) -> dict:
    """读取参考/训练布局，仅使用名称、ID 和策略顺序，不导入增益或站立姿态。"""
    return _validated_layout(_read_json(path))


def apply_layout(profiles, layout) -> dict:
    """返回新的配置；导入十四关节时仅清空 ID14 的部署映射，保留其零位。"""
    layout = _validated_layout(layout)
    result = copy.deepcopy(profiles)
    mapping = layout["joint_to_motor_id"]
    if not set(mapping.values()) <= set(result):
        raise ValueError("当前配置缺少布局中的舵机 ID")
    for profile in result.values():
        profile["deployment_joint"], profile["policy_index"] = "", None
    for index, name in enumerate(layout["policy_joint_order"], 1):
        profile = result[mapping[name]]
        profile["deployment_joint"], profile["policy_index"] = name, index
    return result


def build_calibration(profiles, *, simulation, baudrate, confirmed_reference=False) -> dict:
    """把已验证的输出端零位转成部署格式；软件累计位置只用于检查是否已标定。"""
    if type(simulation) is not bool or simulation:
        raise ValueError("模拟配置不能导出为实机部署标定，请切换到实机配置")
    baudrate = validate_baudrate(baudrate)
    if confirmed_reference is not True:
        raise ValueError("请先确认所选软件零位对应模型 q=0，并已核对模型正方向")
    if not isinstance(profiles, dict) or not profiles:
        raise ValueError("没有可导出的关节配置")
    selected = {}
    for mid, raw in profiles.items():
        profile = validate_profile(raw)
        if type(mid) is not int or mid != profile["id"]:
            raise ValueError("关节索引与舵机 ID 不一致")
        if not profile["deployment_joint"]:
            if profile["policy_index"] is not None:
                raise ValueError(f"ID {mid} 已设置策略序号，但尚未填写部署关节名")
            continue
        selected[mid] = profile
    if len(selected) not in (14, 15) or set(selected) != set(range(len(selected))):
        raise ValueError(f"当前已映射 {len(selected)} 个关节；请完整设置 ID 0–13 或 0–14 的部署关节名和策略序号")
    names = [p["deployment_joint"] for p in selected.values()]
    indexes = [p["policy_index"] for p in selected.values()]
    if len(set(names)) != len(names):
        raise ValueError("部署关节名不能重复")
    if (any(type(i) is not int for i in indexes)
            or set(indexes) != set(range(1, len(selected) + 1))):
        raise ValueError(f"策略序号必须唯一且连续为 1–{len(selected)}，顺序应与训练策略一致")
    ordered = sorted(selected.values(), key=lambda p: p["policy_index"])
    mapping = {p["deployment_joint"]: mid for mid, p in sorted(selected.items())}
    order = [p["deployment_joint"] for p in ordered]
    if len(selected) == 14 and (mapping != REFERENCE_JOINT_TO_MOTOR_ID
                               or tuple(order) != REFERENCE_POLICY_JOINT_ORDER):
        raise ValueError("14 关节导出必须与原例程的关节映射和策略顺序一致；可先导入 reference_14dof.json")
    incomplete = [str(mid) for mid, p in sorted(selected.items())
                  if p["zero_offset"] is None or p["reference_expos"] is None or not p["verified"]]
    if incomplete:
        raise ValueError(f"以下 ID 尚未完成零位记录、ExPos 参考或验证：{', '.join(incomplete)}")
    joints = [{"joint_name": p["deployment_joint"], "motor_id": mid,
               "model_home_rad": 0.0, "encoder_home_rad": p["reference_expos"] % math.tau,
               "direction": p["direction"]} for mid, p in sorted(selected.items())]
    # 这里只说明标定工具使用过的增益，绝不把它写成训练/策略默认增益。
    gains = {p["deployment_joint"]: {
        "kp_output_nm_per_rad": p["kp"], "kd_output_nm_s_per_rad": p["kd"],
        "kp_sdk": p["kp"] / RATIO**2, "kd_sdk": p["kd"] / RATIO**2,
    } for p in ordered}
    custom = len(selected) == 15
    return {
        "format_version": 1, "robot": "s288", "simulation": False,
        "reference_pose": REFERENCE_POSE,
        "position_mapping": "q_model = model_home_rad + direction * wrap_pi(q_motor - encoder_home_rad)",
        "joints": joints,
        "deployment": {
            "exporter": "S288 Studio", "exported_at": datetime.now(timezone.utc).isoformat(),
            "layout_kind": "custom_15" if custom else "reference_14",
            "requires_runtime_adaptation": custom or baudrate != 6000000,
            "joint_to_motor_id": mapping, "policy_joint_order": order,
            "excluded_ids": sorted(set(profiles) - set(selected)),
            "baudrate": baudrate, "gear_ratio": RATIO,
            "reference_runtime_gear_ratio": 288.35,
            "joint_limits_rad": {p["deployment_joint"]: [math.radians(p["min_deg"]),
                                                          math.radians(p["max_deg"])] for p in ordered},
            "calibration_gains": gains,
            "calibration_gain_note": "仅记录标定用增益，不代表训练或部署推荐值；本工具按 round(物理增益 / 减速比平方 * 编码系数) 编码。",
            "target_speed_rad_s": 0.0, "feedforward_nm": 0.0,
            "reference_confirmed": True,
            "compatibility_note": (
                "15 关节需要同步适配部署程序与策略，原 14 关节例程会拒绝本文件。" if custom else
                "标定字段兼容原 14 关节加载器；通信波特率、减速比、训练增益和机械定义仍须核对。"
            ),
        },
    }


def _is_calibration(document):
    """仅允许备份覆盖同类有效标定文件，防止误选软件配置或训练布局。"""
    if (not isinstance(document, dict) or "schema_version" in document
            or document.get("format_version") != 1 or document.get("robot") != "s288"
            or document.get("reference_pose") != REFERENCE_POSE
            or document.get("simulation", False) is not False):
        return False
    joints = document.get("joints")
    if not isinstance(joints, list) or len(joints) not in (14, 15):
        return False
    seen, names = set(), set()
    for item in joints:
        if not isinstance(item, dict):
            return False
        mid, name, direction = item.get("motor_id"), item.get("joint_name"), item.get("direction")
        if (type(mid) is not int or mid in seen or not 0 <= mid <= 14
                or not isinstance(name, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name)
                or name in names or type(direction) is not int or direction not in (-1, 1)):
            return False
        for key in ("model_home_rad", "encoder_home_rad"):
            value = item.get(key)
            if type(value) not in (int, float) or not math.isfinite(value):
                return False
        seen.add(mid)
        names.add(name)
    return seen == set(range(len(joints)))


def write_calibration(path, document) -> None:
    """原子保存导出，上一版保留为 .bak；不覆盖软件零位配置或布局文件。"""
    if not _is_calibration(document):
        raise ValueError("不是完整的 S288 部署标定，未写入文件")
    target = Path(path)
    if target.suffix.lower() != ".json":
        raise ValueError("请将部署标定保存为独立的 .json 文件")
    payload = json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if target.exists() and not _is_calibration(_read_json(target)):
        raise ValueError("目标文件不是同类部署标定，不能覆盖；请使用新的文件名")
    target.parent.mkdir(parents=True, exist_ok=True)
    pending = []
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", newline="\n",
                                         dir=target.parent, prefix=".deployment-", suffix=".tmp",
                                         delete=False) as stream:
            temporary = Path(stream.name)
            pending.append(temporary)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        if target.exists():
            with tempfile.NamedTemporaryFile(dir=target.parent, prefix=".backup-", suffix=".tmp",
                                             delete=False) as stream:
                backup = Path(stream.name)
            pending.append(backup)
            shutil.copyfile(target, backup)
            os.replace(backup, Path(str(target) + ".bak"))
        os.replace(temporary, target)
    finally:
        for path in pending:
            if path.exists():
                path.unlink()
