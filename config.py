"""Software-zero profiles. All angles are radians unless the field ends in _deg."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile


MAX_RAW_POSITION = 1428.018898
PROFILE_FIELDS = {
    "id", "name", "zero_offset", "direction", "min_deg", "max_deg",
    "kp", "kd", "rate_deg_s", "direct_follow", "calibrated_at", "reference_expos",
    "verified", "notes", "deployment_joint", "policy_index",
}
DOCUMENT_FIELDS = {
    "schema_version", "units", "updated_at", "simulation", "joints",
}


def default_profiles() -> dict[int, dict]:
    """Create independent profiles; IDs do not imply physical joint names."""
    # 新建/缺失字段使用这里的默认值；读取已有配置时不会覆盖用户保存的参数。
    return {
        motor_id: {
            "id": motor_id,
            "name": f"舵机 ID {motor_id:02d}",
            "zero_offset": None,
            "direction": 1,
            "min_deg": -30.0,
            "max_deg": 30.0,
            "kp": 0.3,
            "kd": 0.02,
            "rate_deg_s": 15.0,
            "direct_follow": True,
            "calibrated_at": None,
            "reference_expos": None,
            "verified": False,
            "notes": "",
            # 训练/部署名称与 CAD 显示绑定独立；动作顺序不能从电机 ID 推断。
            "deployment_joint": "",
            "policy_index": None,
        }
        for motor_id in range(15)
    }


def _motor_id(value) -> int:
    if type(value) is not int or not 0 <= value <= 14:
        raise ValueError("舵机 ID 必须是 0–14 的整数；15 为广播地址，不用于标定")
    return value


def _number(value, label: str) -> float:
    if type(value) not in (int, float):
        raise ValueError(f"{label} 必须是有限数值")
    try:
        result = float(value)
    except OverflowError as exc:
        raise ValueError(f"{label} 必须是有限数值") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} 必须是有限数值")
    return result


def validate_profile(profile: dict) -> dict:
    """Validate one profile and return a normalized copy, without mutating it."""
    # 允许范围统一在此校验：零位用 rad，带 _deg 的范围和目标用度。
    if not isinstance(profile, dict):
        raise ValueError("关节设置必须是对象")
    unknown = set(profile) - PROFILE_FIELDS
    if unknown:
        raise ValueError(f"关节设置包含未知字段：{', '.join(sorted(unknown))}")
    motor_id = _motor_id(profile.get("id"))
    result = default_profiles()[motor_id]
    result.update(profile)
    if not isinstance(result["name"], str) or not result["name"].strip():
        raise ValueError("关节名称不能为空")
    result["name"] = result["name"].strip()
    if not isinstance(result["deployment_joint"], str):
        raise ValueError("部署关节名必须是文本")
    result["deployment_joint"] = result["deployment_joint"].strip()
    if result["deployment_joint"] and not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", result["deployment_joint"]):
        raise ValueError("部署关节名使用英文字母、数字和下划线，不能以数字开头")
    index = result["policy_index"]
    if index is not None and (type(index) is not int or not 1 <= index <= 15):
        raise ValueError("策略序号必须是 1–15 的整数，未设置时留空")
    if type(result["direction"]) is not int or result["direction"] not in (-1, 1):
        raise ValueError("正方向必须是整数 +1 或 -1")
    if result["zero_offset"] is not None:
        result["zero_offset"] = _number(result["zero_offset"], "零位偏移")
        if abs(result["zero_offset"]) > MAX_RAW_POSITION:
            raise ValueError("零位偏移超出 S288 位置协议范围")
    for field, label in (("min_deg", "角度下限"), ("max_deg", "角度上限"),
                         ("kp", "Kp"), ("kd", "Kd"),
                         ("rate_deg_s", "调节速度")):
        result[field] = _number(result[field], label)
    if not -180 <= result["min_deg"] < result["max_deg"] <= 180:
        raise ValueError("角度范围必须位于 -180°–180° 内，并且下限小于上限")
    if not 0 < result["kp"] <= 2.5:
        raise ValueError("标定用 Kp 必须大于 0 且不超过 2.5")
    if not 0 <= result["kd"] <= 0.2:
        raise ValueError("标定用 Kd 必须位于 0–0.2")
    if not 0.1 <= result["rate_deg_s"] <= 30:
        raise ValueError("调节速度必须位于 0.1–30 °/s")
    if type(result["direct_follow"]) is not bool:
        raise ValueError("直接跟随必须是 true 或 false")
    if result["reference_expos"] is not None:
        result["reference_expos"] = _number(result["reference_expos"], "输出编码器参考值")
        if not 0 <= result["reference_expos"] <= 2 * math.pi:
            raise ValueError("输出编码器参考值必须位于 0–2π rad")
    stamp = result["calibrated_at"]
    if stamp is not None:
        if not isinstance(stamp, str):
            raise ValueError("标定时间必须是 ISO 日期字符串或空值")
        try:
            datetime.fromisoformat(stamp)
        except ValueError as exc:
            raise ValueError("标定时间不是有效的 ISO 日期") from exc
    if type(result["verified"]) is not bool:
        raise ValueError("验证状态必须是 true 或 false")
    if result["verified"] and result["zero_offset"] is None:
        raise ValueError("未记录零位的关节不能标为已验证")
    if not isinstance(result["notes"], str):
        raise ValueError("备注必须是文本")
    return result


def parse_ids(text: str) -> list[int]:
    """Accept '14', '0-14', or '0, 2, 14'; deduplicate in entered order."""
    if not isinstance(text, str) or not text.strip():
        raise ValueError("请输入要连接的 ID，例如 14、0-14 或 0,2,14")
    ids = []
    for token in re.split(r"[,，\s]+", text.strip()):
        if not re.fullmatch(r"[0-9]+(?:-[0-9]+)?", token):
            raise ValueError(f"无法识别 ID：{token}；例如使用 0-14 或 0,2,14")
        endpoints = [int(part) for part in token.split("-")]
        first = _motor_id(endpoints[0])
        last = _motor_id(endpoints[-1])
        if first > last:
            raise ValueError("ID 范围的起点不能大于终点")
        for motor_id in range(first, last + 1):
            if motor_id not in ids:
                ids.append(motor_id)
    return ids


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"配置文件包含重复字段：{key}")
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError(f"配置文件不能包含 {value}")


def load_profiles(path, *, simulation: bool | None = None) -> dict[int, dict]:
    """Load profiles without changing files; optionally require the same device mode."""
    if simulation is not None and type(simulation) is not bool:
        raise ValueError("simulation 必须是布尔值或空值")
    try:
        with Path(path).open("r", encoding="utf-8-sig") as stream:
            document = json.load(stream, object_pairs_hook=_unique_object,
                                 parse_constant=_reject_constant)
    except json.JSONDecodeError as exc:
        raise ValueError(f"配置文件 JSON 格式错误：第 {exc.lineno} 行") from exc
    if not isinstance(document, dict):
        raise ValueError("配置文件顶层必须是对象")
    unknown = set(document) - DOCUMENT_FIELDS
    if unknown:
        raise ValueError(f"配置文件包含未知字段：{', '.join(sorted(unknown))}")
    if type(document.get("schema_version")) is not int or document["schema_version"] != 1:
        raise ValueError("不支持的配置版本，需要 schema_version = 1")
    if document.get("units") != "rad":
        raise ValueError("零位配置单位必须是 rad；角度限位字段单独使用 deg")
    mode = document.get("simulation", False)
    if type(mode) is not bool:
        raise ValueError("simulation 必须是 true 或 false")
    if simulation is not None and mode != simulation:
        raise ValueError("模拟配置与实机配置不能混用，请选择对应模式的配置文件")
    joints = document.get("joints")
    if not isinstance(joints, list) or not joints:
        raise ValueError("配置文件必须包含非空 joints 列表")
    result = default_profiles()
    seen = set()
    for item in joints:
        profile = validate_profile(item)
        motor_id = profile["id"]
        if motor_id in seen:
            raise ValueError(f"配置文件包含重复舵机 ID：{motor_id}")
        seen.add(motor_id)
        result[motor_id] = profile
    return result


def save_profiles(path, profiles: dict[int, dict], *, simulation: bool = False) -> None:
    """Atomically save JSON and preserve the previous file as <name>.bak."""
    if type(simulation) is not bool:
        raise ValueError("simulation 必须是布尔值")
    if not isinstance(profiles, dict) or not profiles:
        raise ValueError("没有可保存的关节设置")
    normalized = []
    for motor_id, profile in profiles.items():
        _motor_id(motor_id)
        item = validate_profile(profile)
        if item["id"] != motor_id:
            raise ValueError("关节索引与设置中的 ID 不一致")
        normalized.append(item)
    target = Path(path)
    if target.suffix.lower() == ".bak":
        raise ValueError("请将备份另存为 .json 文件，不要直接覆盖 .bak 备份")
    document = {
        "schema_version": 1,
        "units": "rad",
        "simulation": simulation,
        "updated_at": datetime.now(timezone.utc).isoformat(),
        "joints": sorted(normalized, key=lambda item: item["id"]),
    }
    payload = json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    backup_temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", newline="\n",
                                         dir=target.parent, prefix=".calibration-",
                                         suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        if target.exists():
            # 正式文件和备份分别原子替换；外层调用须在保存成功后才提交内存。
            # Copy to a temporary backup first, so a partial copy cannot damage .bak.
            with tempfile.NamedTemporaryFile(dir=target.parent, prefix=".backup-",
                                             suffix=".tmp", delete=False) as stream:
                backup_temporary = Path(stream.name)
            shutil.copyfile(target, backup_temporary)
            os.replace(backup_temporary, Path(str(target) + ".bak"))
        os.replace(temporary, target)
    finally:
        for pending in (temporary, backup_temporary):
            if pending is not None and pending.exists():
                pending.unlink()
