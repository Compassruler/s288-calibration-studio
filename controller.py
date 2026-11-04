"""后台通信与位置控制：串口只由此线程访问，界面通过队列收发命令和反馈。"""

from collections import deque
from datetime import datetime
import math
import queue
import threading
import time

from protocol import (Bus, DEFAULT_BAUDRATE, POSITION_LIMIT, ProtocolError, SimBus,
                      validate_baudrate, validate_id)
from config import validate_profile


FEEDBACK_MAX_AGE_S = 0.25  # 主机反馈有效期；超时后不能继续发送位置驱动。
ACTIVE_LIMIT_MARGIN_DEG = 2.0  # 仅容纳运行中的小幅跟随超调，不放宽目标角度限位。
HOME_TOLERANCE_DEG = 2.0  # 回零验收允许的实测位置偏差；控制目标仍是精确的已保存零位。


class HomeTimeout(Exception):
    """Healthy feedback was received, but the saved pose was not reached."""


class Controller(threading.Thread):
    """send() 接收命令，events 返回独立的反馈副本。

    界面刷新和保存文件不访问串口。停止、释放、断开和关闭会使旧运动命令失效，
    因而不能绕过这些入口直接改 armed/holding 或调用总线。
    """

    def __init__(self, events):
        super().__init__(name="S288 communication", daemon=True)
        self.events = events
        self.commands = queue.PriorityQueue()
        self._queue_lock = threading.Lock()
        self._generation = 0
        self._sequence = 0
        self._latest_targets = {}
        self._urgent = threading.Event()
        self.bus = None
        self.ids = []
        self.selected = None
        self.armed = None
        self.holding = {}
        self.homing = None
        self.feedback = {}
        self.origins = {}
        self.history = {}
        self.faulted = False
        self._other_index = 0
        self._running = True

    def send(self, action, **kwargs):
        urgent = action in {"hold", "release", "release_all", "disconnect", "shutdown"}
        with self._queue_lock:
            if urgent:
                self._generation += 1
                self._urgent.set()
            self._sequence += 1
            if action == "target":
                self._latest_targets[kwargs.get("id")] = self._sequence
            priority = 0 if urgent else 1
            self.commands.put((priority, self._sequence, self._generation, action, kwargs))

    def _emit(self, kind, **kwargs):
        self.events.put({"type": kind, **kwargs})

    def _snapshot(self, motor_id):
        feedback = self.feedback.get(motor_id)
        if feedback is None:
            return
        state = self.armed if self.armed and self.armed["id"] == motor_id else None
        held = self.holding.get(motor_id)
        self._emit(
            "snapshot", id=motor_id, feedback=dict(feedback),
            armed=state is not None, faulted=self.faulted, origin=self.origins[motor_id],
            target_raw=state["target_raw"] if state else None,
            requested_raw=state["requested_raw"] if state else None,
            holding=held is not None,
        )

    def _remember(self, motor_id, feedback):
        if feedback.get("id", motor_id) != motor_id:
            raise ProtocolError("设备反馈 ID 不匹配")
        for name in ("raw_pos", "expos", "speed", "torque", "voltage", "temp", "timestamp"):
            if not math.isfinite(feedback[name]):
                raise ProtocolError(f"设备反馈 {name} 无效")
        self.feedback[motor_id] = dict(feedback)
        self.origins.setdefault(motor_id, feedback["raw_pos"])
        self.history.setdefault(motor_id, deque(maxlen=100)).append(dict(feedback))

    @staticmethod
    def _healthy(feedback):
        return not (feedback["fault"] or feedback["warning"] or feedback["timeout"]) and feedback["mode"] == 1

    @staticmethod
    def _status_error(motor_id, feedback):
        return ProtocolError(
            f"ID {motor_id} 状态异常：故障={feedback['fault']}，警告={feedback['warning']}，"
            f"超时保护={feedback['timeout']}，模式={feedback['mode']}"
        )

    def _cancel_pending(self):
        # The flag may still describe an urgent command already handled in this
        # iteration; inspect the queue to cancel only for an unhandled request.
        with self._queue_lock:
            with self.commands.mutex:
                return any(item[0] == 0 for item in self.commands.queue)

    def _zero_exchange(self, motor_id, recover=True, cancelable=False):
        if cancelable and self._cancel_pending():
            raise ValueError("连接已取消")
        feedback = self.bus.exchange(motor_id)
        self._remember(motor_id, feedback)
        if feedback["fault"] or feedback["warning"] or feedback["mode"] not in (0, 1):
            raise self._status_error(motor_id, feedback)
        if recover and not self._healthy(feedback):
            # Protection clearing is only ever sent with zero gains/torque.
            for flag in (0, 1):
                for _ in range(10):
                    if cancelable and self._cancel_pending():
                        raise ValueError("连接已取消")
                    time.sleep(0.01)
                    feedback = self.bus.exchange(motor_id, timeout_flag=flag)
                    self._remember(motor_id, feedback)
                    if feedback["fault"] or feedback["warning"]:
                        raise self._status_error(motor_id, feedback)
                    if self._healthy(feedback):
                        break
                else:
                    raise self._status_error(motor_id, feedback)
        if not self._healthy(feedback):
            raise self._status_error(motor_id, feedback)
        return feedback

    def _require_connected(self, motor_id=None):
        if self.bus is None:
            raise ValueError("请先连接设备或进入模拟模式")
        if motor_id is not None:
            validate_id(motor_id)
            if motor_id not in self.ids:
                raise ValueError("该 ID 不在本次连接列表中")

    def _require_fresh(self, motor_id):
        feedback = self.feedback.get(motor_id)
        if feedback is None or not 0 <= time.monotonic() - feedback["timestamp"] <= FEEDBACK_MAX_AGE_S:
            raise ValueError("反馈已过期，请等待新的设备反馈")
        if not self._healthy(feedback):
            raise self._status_error(motor_id, feedback)
        return feedback

    @staticmethod
    def _number(value, name):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{name} 必须是有限数值")
        return float(value)

    def _connect(self, port, ids, demo=False, baudrate=DEFAULT_BAUDRATE):
        if self.bus is not None:
            raise ValueError("请先断开当前连接")
        baudrate = validate_baudrate(baudrate)
        if not isinstance(ids, (list, tuple)) or not ids or len(set(ids)) != len(ids):
            raise ValueError("连接 ID 列表必须非空且没有重复")
        for motor_id in ids:
            validate_id(motor_id)
        self.feedback.clear()
        self.origins.clear()
        self.history.clear()
        self.armed = None
        self.faulted = False
        self.ids = list(ids)
        self.selected = self.ids[0]
        # 保留默认构造方式；只在改用 4 Mbps 时覆盖总线默认值。
        bus_options = {} if baudrate == DEFAULT_BAUDRATE else {"baudrate": baudrate}
        self.bus = SimBus(**bus_options) if demo else Bus(port, **bus_options)
        try:
            for motor_id in self.ids:
                self._zero_exchange(motor_id, cancelable=True)
        except Exception:
            self._disconnect(recover=not self._cancel_pending())
            raise
        self._emit("connected", ids=list(self.ids), demo=bool(demo), baudrate=baudrate)
        for motor_id in self.ids:
            self._snapshot(motor_id)
        self._emit("status", message="模拟设备已连接" if demo else "已连接；当前均请求零驱动力矩")

    def _select(self, motor_id):
        self._require_connected(motor_id)
        self._require_manual_control()
        if self.armed:
            raise ValueError("请先释放当前舵机，再切换关节")
        self.selected = motor_id
        if not self.faulted:
            self._zero_exchange(motor_id)
        self._snapshot(motor_id)

    def _arm(self, motor_id, **settings):
        self._require_connected(motor_id)
        self._require_manual_control()
        if self.faulted:
            raise ValueError("通信异常已锁止，请断开后重新连接")
        if self.armed:
            raise ValueError("已有舵机使能，请先释放")
        if motor_id != self.selected:
            raise ValueError("只能使能当前选中的舵机")
        # 手动控制和批量回零共用配置校验，参数范围只在 config.py 维护。
        profile = validate_profile({"id": motor_id, "direct_follow": False, **settings})
        kp, kd, rate = profile["kp"], profile["kd"], profile["rate_deg_s"]
        low, high = profile["min_deg"], profile["max_deg"]
        direction = profile["direction"]
        zero = profile["zero_offset"]
        zero = self.origins[motor_id] if zero is None else zero
        if abs(zero) > POSITION_LIMIT:  # 未标定时的临时零位来自反馈，也要能编码为目标。
            raise ValueError("零位超出协议范围")
        feedback = self._zero_exchange(motor_id)
        angle = direction * math.degrees(feedback["raw_pos"] - zero)
        if not low <= angle <= high:
            raise ValueError(f"当前位置 {angle:.2f}° 已超出设定范围，不能使能")
        now = time.monotonic()
        # 第一个非零增益包必须保持刚读到的位置，避免使能瞬间跳到旧目标。
        self.armed = {
            "id": motor_id, "kp": kp, "kd": kd, "zero": zero,
            "direction": direction, "min_deg": low, "max_deg": high,
            "rate": math.radians(rate), "last_move": now,
            "direct_follow": profile["direct_follow"],
            "target_raw": feedback["raw_pos"], "requested_raw": feedback["raw_pos"],
        }
        self._poll_one(motor_id)
        self._emit("armed", id=motor_id, raw_pos=feedback["raw_pos"])

    def _set_target(self, motor_id, angle_deg):
        self._require_connected(motor_id)
        self._require_manual_control()
        if not self.armed or self.armed["id"] != motor_id:
            raise ValueError("请先使能当前舵机")
        self._require_fresh(motor_id)
        angle = self._number(angle_deg, "目标角度")
        state = self.armed
        if not state["min_deg"] <= angle <= state["max_deg"]:
            raise ValueError("目标角度超出本关节限位")
        raw = state["zero"] + state["direction"] * math.radians(angle)
        if abs(raw) > POSITION_LIMIT:
            raise ValueError("目标位置超出协议范围")
        state["requested_raw"] = raw
        self._snapshot(motor_id)

    def _hold(self, motor_id):
        self._require_connected(motor_id)
        if self.homing or self.holding:
            self._release_all()
            return
        if not self.armed or self.armed["id"] != motor_id:
            raise ValueError("当前舵机没有使能")
        feedback = self._require_fresh(motor_id)
        self.armed["target_raw"] = feedback["raw_pos"]
        self.armed["requested_raw"] = feedback["raw_pos"]
        self.armed["last_move"] = time.monotonic()
        self._snapshot(motor_id)
        self._emit("held", id=motor_id, raw_pos=feedback["raw_pos"])
        self._emit("status", message=f"ID {motor_id} 已取消剩余目标，保持当前位置")

    def _release(self, motor_id, recover=True):
        self._require_connected(motor_id)
        if self.armed and self.armed["id"] == motor_id:
            self.armed = None
        confirmed = False
        try:
            self._zero_exchange(motor_id, recover=recover)
            confirmed = True
        except Exception as exc:
            self.faulted = True
            self._emit("error", id=motor_id, message=f"零力矩请求未获确认：{exc}")
        self.history[motor_id] = deque(maxlen=100)
        self._snapshot(motor_id)
        self._emit("released", id=motor_id, confirmed=confirmed)
        return confirmed

    def _release_all(self, recover=True):
        batch = bool(self.homing or self.holding)
        self.homing = None
        self.holding.clear()
        if self.bus is None:
            return True
        active_id = self.armed["id"] if self.armed else None
        release_ids = sorted(self.ids, key=lambda motor_id: motor_id != active_id)
        self.armed = None
        confirmed = True
        for motor_id in release_ids:
            confirmed = self._release(motor_id, recover=recover) and confirmed
        status = "全部释放已确认" if confirmed else "释放未全部确认"
        self._home_status(("回零已停止，" if batch else "") + status)
        return confirmed

    def _disconnect(self, recover=True):
        confirmed = self._release_all(recover=recover)
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception as exc:
                confirmed = False
                self._emit("error", message=f"关闭串口失败：{exc}")
        self.bus = None
        self.ids = []
        self.selected = None
        self.armed = None
        self._emit("disconnected", confirmed=confirmed)

    def _capture_zero(self, motor_id):
        self._require_connected(motor_id)
        self._require_manual_control()
        if self.armed:
            raise ValueError("请先释放舵机，再记录软件零位")
        if self.faulted:
            raise ValueError("存在未解除的通信异常，不能记录零位")
        now = time.monotonic()
        samples = [sample for sample in self.history[motor_id]
                   if 0 <= now - sample["timestamp"] <= 0.65]
        capture = self._zero_record(motor_id, samples)
        self._emit("captured", **capture)
        self._emit("status", message=f"ID {motor_id} 的软件零位已记录；请保存配置")

    def _zero_record(self, motor_id, samples):
        latest = self._require_fresh(motor_id)
        if len(samples) < 10 or samples[-1]["timestamp"] - samples[0]["timestamp"] < 0.4:
            raise ValueError(f"ID {motor_id}：请保持关节静止至少约半秒，等待稳定采样")
        if any(not self._healthy(sample) or abs(sample["speed"]) >= 0.03 for sample in samples):
            raise ValueError(f"ID {motor_id}：关节仍在运动或状态异常，不能记录零位")
        positions = [sample["raw_pos"] for sample in samples]
        if max(positions) - min(positions) > math.radians(0.2):
            raise ValueError(f"ID {motor_id}：角度尚未稳定，不能记录零位")
        zero = sum(positions) / len(positions)
        if abs(zero) > POSITION_LIMIT or not 0 <= latest["expos"] <= 2 * math.pi:
            raise ValueError(f"ID {motor_id}：零位或输出编码器参考值超出协议范围")
        return dict(
            id=motor_id, zero_offset=zero, expos=latest["expos"],
            calibrated_at=datetime.now().astimezone().isoformat(timespec="seconds"),
        )

    def _capture_all_zero(self, ids=None):
        """Release and sample every connected motor; commit only one complete result."""
        self._require_connected()
        if ids is not None:
            if not isinstance(ids, (list, tuple)) or not ids:
                raise ValueError("全部零位记录需要完整的已连接 ID 列表")
            for motor_id in ids:
                self._require_connected(motor_id)
            if len(set(ids)) != len(ids) or set(ids) != set(self.ids):
                raise ValueError("连接 ID 列表已变化，请重新记录全部零位")
        if self.faulted:
            raise ValueError("存在未解除的通信异常，不能记录零位")
        generation = self._generation
        if not self._release_all():
            raise ValueError("未能确认全部舵机已释放，未记录任何零位")
        samples = {motor_id: [] for motor_id in self.ids}
        deadline = time.monotonic() + 3
        while True:
            for motor_id in self.ids:
                if self._cancel_pending():
                    raise ValueError("全部零位记录已取消，未记录任何零位")
                feedback = self._zero_exchange(motor_id, cancelable=True)
                self._require_fresh(motor_id)
                samples[motor_id].append(dict(feedback))
                self._snapshot(motor_id)
            if all(len(values) >= 10 and values[-1]["timestamp"] - values[0]["timestamp"] >= .4
                   for values in samples.values()):
                break
            if time.monotonic() >= deadline:
                raise ValueError("全部零位采样超时，未记录任何零位")
            time.sleep(.02)
        captures = [self._zero_record(motor_id, samples[motor_id]) for motor_id in self.ids]
        # An urgent stop that arrived during validation must invalidate the
        # entire result, just as it invalidates an older queued motion command.
        with self._queue_lock:
            if generation != self._generation:
                raise ValueError("全部零位记录已取消，未记录任何零位")
            self._emit("captured_all", captures=captures)
        self._emit("status", message=f"已记录全部 {len(captures)} 台舵机的软件零位；请保存配置")

    def _require_manual_control(self):
        if self.homing or self.holding:
            raise ValueError("正在回零或保持零位，请先全部释放")

    def _home_status(self, message):
        self._emit("home_state", active=self.homing is not None,
                   holding_ids=sorted(self.holding), message=message)

    def _start_home(self, profiles):
        """Validate the whole batch before enabling; move one joint at a time."""
        self._require_connected()
        self._require_manual_control()
        if self.faulted or self.armed:
            raise ValueError("请先全部释放并解除通信异常，再回零")
        if not isinstance(profiles, dict) or not profiles:
            raise ValueError("没有已记录零位的舵机")
        prepared = []
        for motor_id, source in sorted(profiles.items()):
            self._require_connected(motor_id)
            profile = validate_profile(source)
            if profile["id"] != motor_id or profile["zero_offset"] is None:
                raise ValueError(f"ID {motor_id} 尚未记录零位")
            if not profile["min_deg"] <= 0 <= profile["max_deg"]:
                raise ValueError(f"ID {motor_id} 的零位不在角度限位内")
            feedback = self._zero_exchange(motor_id, cancelable=True)
            self._check_home_range(profile, feedback)
            prepared.append(profile)
        if self._cancel_pending():
            raise ValueError("回零已取消")
        self.homing = {"pending": deque(prepared), "total": len(prepared), "done": 0,
                       "stable_since": None, "deadline": 0}
        try:
            self._next_home()
        except Exception:
            self._release_all(recover=False)
            raise

    @staticmethod
    def _check_home_range(profile, feedback):
        angle = profile["direction"] * math.degrees(feedback["raw_pos"]-profile["zero_offset"])
        if not profile["min_deg"] <= angle <= profile["max_deg"]:
            raise ValueError(f"ID {profile['id']} 当前位置 {angle:.2f}° 超出限位，不能回零")

    def _next_home(self):
        plan = self.homing
        if not plan or self._cancel_pending():
            return
        if not plan["pending"]:
            plan["stable_since"] = None
            plan["deadline"] = time.monotonic() + 5
            self._home_status("正在确认全部舵机的零位姿态…")
            return
        profile = plan["pending"].popleft()
        motor_id = profile["id"]
        # Uncontrolled joints may have moved while earlier joints returned home.
        feedback = self._zero_exchange(motor_id, cancelable=True)
        self._check_home_range(profile, feedback)
        now = time.monotonic()
        raw, zero = feedback["raw_pos"], profile["zero_offset"]
        self.armed = dict(id=motor_id, kp=profile["kp"], kd=profile["kd"],
            zero=zero, direction=profile["direction"], min_deg=profile["min_deg"],
            max_deg=profile["max_deg"], rate=math.radians(profile["rate_deg_s"]),
            last_move=now, direct_follow=False, target_raw=raw, requested_raw=raw)
        self._poll_one(motor_id)  # First nonzero-gain packet holds the measured pose.
        self.armed["requested_raw"] = zero
        plan["stable_since"] = None
        plan["deadline"] = now + abs(raw-zero)/self.armed["rate"] + 5
        self._home_status(f"正在回零：ID {motor_id} · {plan['done']+1}/{plan['total']}")

    def _check_home_progress(self):
        if not self.homing or self._cancel_pending():
            return
        plan, state, now = self.homing, self.armed, time.monotonic()
        if state is None:
            feedbacks = [(held, self._require_fresh(motor_id))
                         for motor_id, held in self.holding.items()]
            settled = all(self._at_home(held, feedback) for held, feedback in feedbacks)
            plan["stable_since"] = (plan["stable_since"] or now) if settled else None
            if plan["stable_since"] is not None and now-plan["stable_since"] >= .25:
                self.homing = None
                self._home_status(f"回零完成：{len(self.holding)} 台保持在已记录零位")
            elif now > plan["deadline"]:
                details = "；".join(self._home_deviation(held, feedback)
                                   for held, feedback in feedbacks
                                   if not self._at_home(held, feedback))
                raise HomeTimeout(f"整组回零超时，实际位置尚未稳定到位；{details}")
            return
        feedback = self._require_fresh(state["id"])
        settled = self._at_home(state, feedback)
        plan["stable_since"] = (plan["stable_since"] or now) if settled else None
        if plan["stable_since"] is not None and now-plan["stable_since"] >= .25:
            self.holding[state["id"]] = state
            self.armed = None
            plan["done"] += 1
            self._snapshot(state["id"])
            self._next_home()
        elif now > plan["deadline"]:
            raise HomeTimeout(f"回零超时，实际位置尚未稳定到位；{self._home_deviation(state, feedback)}")

    @staticmethod
    def _home_deviation(state, feedback):
        error = state["direction"] * math.degrees(feedback["raw_pos"] - state["zero"])
        speed = state["direction"] * math.degrees(feedback["speed"])
        return f"ID {state['id']}：位置偏差 {error:+.2f}°，速度 {speed:+.2f}°/s"

    @staticmethod
    def _at_home(state, feedback):
        return (abs(feedback["raw_pos"]-state["zero"]) <= math.radians(HOME_TOLERANCE_DEG) + 1e-12
                and abs(feedback["speed"]) <= math.radians(2)
                and abs(state["target_raw"]-state["zero"]) < 1e-9)

    def _check_active_feedback(self, state, feedback):
        """只保护非零增益控制；零增益释放不能被位置异常阻断。"""
        motor_id = state["id"]
        if feedback is None or not 0 <= time.monotonic() - feedback["timestamp"] <= FEEDBACK_MAX_AGE_S:
            raise ProtocolError(f"ID {motor_id} 活动反馈过期或时间异常，不能继续位置控制")
        if not self._healthy(feedback):
            raise self._status_error(motor_id, feedback)
        angle = state["direction"] * math.degrees(feedback["raw_pos"] - state["zero"])
        if not state["min_deg"] - ACTIVE_LIMIT_MARGIN_DEG <= angle <= state["max_deg"] + ACTIVE_LIMIT_MARGIN_DEG:
            raise ProtocolError(
                f"ID {motor_id} 实测角度 {angle:.2f}° 超出活动限位 "
                f"[{state['min_deg']:.1f}°, {state['max_deg']:.1f}°]"
                f"（保护容差 {ACTIVE_LIMIT_MARGIN_DEG:g}°）"
            )

    def _poll_one(self, motor_id):
        state = self.armed if self.armed and self.armed["id"] == motor_id else self.holding.get(motor_id)
        if state:
            if self._urgent.is_set():
                return
            self._check_active_feedback(state, self.feedback.get(motor_id))
            now = time.monotonic()
            dt = min(0.08, max(0.0, now - state["last_move"]))
            delta = state["requested_raw"] - state["target_raw"]
            # 限速仅逐步改变位置目标；协议的期望角速度仍为 0。
            if state["direct_follow"]:
                state["target_raw"] = state["requested_raw"]
            else:
                state["target_raw"] += max(-state["rate"] * dt, min(state["rate"] * dt, delta))
            state["last_move"] = now
            feedback = self.bus.exchange(motor_id, state["target_raw"], state["kp"], state["kd"])
            self._remember(motor_id, feedback)
            self._check_active_feedback(state, feedback)
        else:
            self._zero_exchange(motor_id)
        self._snapshot(motor_id)

    def _fault(self, exc):
        self.faulted = True
        with self._queue_lock:
            self._generation += 1
        self._release_all(recover=False)
        self._emit("error", message=f"已停止主动控制并锁止；请检查后重新连接。{exc}")

    def _home_failed(self, exc):
        # A missed pose is recoverable only after every zero-gain request is
        # confirmed. Discard older motion; never retry or resume automatically.
        with self._queue_lock:
            self._generation += 1
        confirmed = self._release_all(recover=False)
        if confirmed and not self.faulted:
            message = f"{exc}；全部舵机已释放，连接保留；检查后可手动重试。"
        else:
            self.faulted = True
            message = f"{exc}；释放未全部确认或设备状态异常，已锁止；请检查后重新连接。"
        self._home_status(message)
        self._emit("error", message=message)

    def _dispatch(self, action, data):
        motor_id = data.get("id")
        if action == "connect":
            self._connect(data.get("port", ""), data.get("ids"), data.get("demo", False),
                          data.get("baudrate", DEFAULT_BAUDRATE))
        elif action == "select":
            self._select(motor_id)
        elif action == "arm":
            self._arm(motor_id, **{key: value for key, value in data.items() if key != "id"})
        elif action == "target":
            self._set_target(motor_id, data.get("angle_deg"))
        elif action == "hold":
            self._hold(motor_id)
        elif action == "release":
            if self.homing or self.holding:
                self._release_all()
            else:
                self._release(motor_id)
        elif action == "release_all":
            self._release_all()
        elif action == "capture_zero":
            self._capture_zero(motor_id)
        elif action == "capture_all_zero":
            self._capture_all_zero(data.get("ids"))
        elif action == "home":
            self._start_home(data.get("profiles"))
        elif action == "disconnect":
            self._disconnect()
        elif action == "shutdown":
            self._disconnect()
            self._running = False
        else:
            raise ValueError(f"未知命令：{action}")

    def run(self):
        try:
            while self._running:
                started = time.monotonic()
                for _ in range(40):
                    try:
                        _, sequence, generation, action, data = self.commands.get_nowait()
                    except queue.Empty:
                        break
                    if generation != self._generation and action not in {"hold", "release", "release_all", "disconnect", "shutdown"}:
                        if action == "capture_all_zero":
                            self._emit("capture_cancelled", scope="all",
                                       message="全部零位记录已取消，未记录任何零位")
                        continue
                    if action == "target" and sequence != self._latest_targets.get(data.get("id")):
                        continue
                    try:
                        self._dispatch(action, data)
                    except ValueError as exc:
                        if action == "home":
                            self._home_status(f"回零未启动：{exc}")
                        self._emit("error", id=data.get("id"), message=str(exc))
                    except Exception as exc:
                        self._fault(exc)
                    if not self._running:
                        break
                with self._queue_lock:
                    with self.commands.mutex:
                        urgent_pending = any(item[0] == 0 for item in self.commands.queue)
                    if not urgent_pending:
                        self._urgent.clear()
                if self.bus is not None and not self.faulted and self._running:
                    try:
                        focus = self.armed["id"] if self.armed else self.selected
                        active = list(dict.fromkeys([focus, *self.holding]))
                        for motor_id in active:
                            if self._urgent.is_set():
                                break
                            self._poll_one(motor_id)
                        others = [motor_id for motor_id in self.ids if motor_id not in active]
                        if others and not self._urgent.is_set():
                            self._poll_one(others[self._other_index % len(others)])
                            self._other_index += 1
                        self._check_home_progress()
                    except HomeTimeout as exc:
                        self._home_failed(exc)
                    except Exception as exc:
                        self._fault(exc)
                time.sleep(max(0.001, 0.02 - (time.monotonic() - started)))
        finally:
            if self.bus is not None:
                self._disconnect()
            self._emit("stopped")
