"""Output-encoder zero coordinates, independent of the volatile rotor counter."""

import math


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label}必须是有限数值")
    return float(value)


def wrap_pi(angle):
    """Return the shortest signed single-turn difference in [-pi, pi)."""
    angle = _finite(angle, "角度")
    return (angle + math.pi) % math.tau - math.pi


def circular_mean(values):
    """Return the mean in [0, 2pi) and circular standard deviation, in radians."""
    values = [_finite(value, "输出编码器角度") for value in values]
    if not values:
        raise ValueError("输出编码器采样不能为空")
    sine = sum(math.sin(value) for value in values) / len(values)
    cosine = sum(math.cos(value) for value in values) / len(values)
    resultant = min(1.0, math.hypot(sine, cosine))
    if resultant <= 1e-12:
        raise ValueError("输出编码器采样分散，无法确定单圈零位")
    return math.atan2(sine, cosine) % math.tau, math.sqrt(max(0.0, -2.0 * math.log(resultant)))


def joint_angle_rad(profile, expos, origin_expos=None):
    """Use saved ExPos zero, or this connection's ExPos origin if uncalibrated.

    A legacy accumulated raw zero cannot be substituted for an absent output
    reference: its coordinate can change after a power cycle.  This function
    only computes the control angle; it never changes the saved profile or
    applies the independent 3D display direction and reference-pose offsets.
    """
    expos = _finite(expos, "输出编码器角度")
    if not 0 <= expos < math.tau:
        raise ValueError("输出编码器角度必须位于 0–2π rad 内")
    reference = profile.get("reference_expos")
    if reference is None:
        if profile.get("zero_offset") is not None:
            raise ValueError("已记录的旧零位缺少 ExPos 输出编码器参考，请用零位支架重新记录零位")
        if origin_expos is None:
            raise ValueError("尚无本次连接的输出编码器参考，请等待设备反馈")
        reference = origin_expos
    reference = _finite(reference, "输出编码器零位参考")
    if not 0 <= reference <= math.tau:
        raise ValueError("输出编码器零位参考必须位于 0–2π rad 内")
    direction = profile.get("direction", 1)
    if type(direction) is not int or direction not in (-1, 1):
        raise ValueError("控制方向必须为 +1 或 -1")
    difference = wrap_pi(expos - reference)
    if math.isclose(abs(difference), math.pi, rel_tol=0.0, abs_tol=1e-10):
        raise ValueError("当前位置位于单圈编码器的 180° 歧义边界，请手动检查姿态")
    return direction * difference
