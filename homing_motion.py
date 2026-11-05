"""Pure home-only S-curve position targets with bounded velocity.

The controller advances elapsed time using its capped loop dt.  This module
does not use wall-clock time, read feedback, or change an encoder reference.
"""

from dataclasses import dataclass, field
import math


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name}必须是有限数值")
    return float(value)


def _ramp_integral(u):
    # Integral of velocity smoothstep 10u^3 - 15u^4 + 6u^5.
    return u ** 4 * (2.5 + u * (-3.0 + u))


@dataclass(frozen=True)
class HomingMotion:
    """Absolute continuous targets in output-equivalent radians; speed in rad/s."""
    start_raw: float
    target_raw: float
    max_speed: float
    distance: float = field(init=False)
    ramp_s: float = field(init=False)
    peak_speed: float = field(init=False)
    duration: float = field(init=False)
    cruise_s: float = field(init=False)

    def __post_init__(self):
        start = _number(self.start_raw, "起点")
        target = _number(self.target_raw, "终点")
        speed = _number(self.max_speed, "回零速度")
        if speed <= 0:
            raise ValueError("回零速度必须大于零")
        distance = abs(target - start)
        if not math.isfinite(distance):
            raise ValueError("回零行程超出数值范围")
        ramp = .6
        if distance == 0:
            ramp, peak, cruise, duration = 0.0, 0.0, 0.0, 0.0
        elif distance >= speed * ramp:
            peak = speed
            cruise = distance / speed - ramp
            duration = 2 * ramp + cruise
        else:
            # Scaling ramp time by the cube root bounds velocity, peak
            # acceleration and peak jerk by the long-move ramp values.
            ramp = max(.12, ramp * (distance / (speed * ramp)) ** (1.0 / 3.0))
            peak = distance / ramp
            cruise = 0.0
            duration = 2 * ramp
        if not all(math.isfinite(value) for value in (ramp, peak, cruise, duration)):
            raise ValueError("回零轨迹超出数值范围")
        for name, value in (("start_raw", start), ("target_raw", target), ("max_speed", speed),
                            ("distance", distance), ("ramp_s", ramp), ("peak_speed", peak),
                            ("cruise_s", cruise), ("duration", duration)):
            object.__setattr__(self, name, value)

    def position_at(self, elapsed):
        elapsed = _number(elapsed, "轨迹时间")
        if elapsed <= 0:
            return self.start_raw
        if elapsed >= self.duration:
            return self.target_raw
        if elapsed < self.ramp_s:
            distance = self.peak_speed * self.ramp_s * _ramp_integral(elapsed / self.ramp_s)
        elif elapsed < self.ramp_s + self.cruise_s:
            distance = self.peak_speed * (.5 * self.ramp_s + elapsed - self.ramp_s)
        else:
            remaining = (self.duration - elapsed) / self.ramp_s
            distance = self.distance - self.peak_speed * self.ramp_s * _ramp_integral(remaining)
        # Avoid a round-off overshoot of either endpoint.  There is no minimum
        # distance jump: arbitrarily small moves still follow the same curve.
        distance = max(0.0, min(self.distance, distance))
        return self.start_raw + math.copysign(distance, self.target_raw - self.start_raw)
