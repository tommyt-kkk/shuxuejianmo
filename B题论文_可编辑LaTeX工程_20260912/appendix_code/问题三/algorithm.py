"""第三问：保守空间约束、时间代价调度与有限覆盖兜底。

纯算法模块，不连接或创建模拟器。无需第三方依赖；不读取案例真值。
方格表示整个闭合区域，而非一个假设目标位置。软评分仅用于调度。
"""

from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
import json
import math
import time

import hashlib
import os
import tempfile


Point = tuple[float, float]
EPS = 1e-7


@dataclass(frozen=True)
class StrategyConfig:
    region_radius: float = 1800.0
    minimum_range: float = 1000.0
    maximum_range: float = 1500.0
    direction_error_deg: float = 1.005001
    clear_radius: float = 20.0
    near_radius: float = 5.0
    speed: float = 5.0
    cell_size: float = 14.0
    # 下列参数只改变效率，不用于判定频道不存在。
    search_ring_radius: float = 1500.0
    lateral_offsets: tuple = (40.0, 100.0, 220.0)
    lookahead_samples: int = 9
    early_clear_mass: float = 0.72
    max_localization_measures: int = 6
    max_failed_clears: int = 2
    max_deferred_actions: int = 70
    real_time_reserve_s: float = 3.0
    clear_candidate_samples: int = 15
    failed_clear_penalty_s: float = 12.0
    no_signal_penalty_s: float = 150.0
    extra_measure_penalty_s: float = 18.0
    target_uncertainty_m: float = 12.0
    geometry_sine_floor: float = 0.015

    def __post_init__(self):
        physical = {"region_radius": 1800.0, "minimum_range": 1000.0,
                    "maximum_range": 1500.0, "direction_error_deg": 1.005001,
                    "clear_radius": 20.0, "near_radius": 5.0, "speed": 5.0}
        for name, value in physical.items():
            if getattr(self, name) != value:
                raise ValueError(f"题目常量和安全边界不能调参：{name}")
        for item in fields(self):
            value = getattr(self, item.name)
            if item.name == "lateral_offsets":
                if not isinstance(value, tuple) or not value:
                    raise ValueError("lateral_offsets 必须是非空序列")
                values = value
            else:
                values = (value,)
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                   not math.isfinite(v) or v < 0 for v in values):
                raise ValueError(f"参数必须是非负有限数值：{item.name}")
        for name in ("lookahead_samples", "clear_candidate_samples", "max_localization_measures",
                     "max_failed_clears", "max_deferred_actions"):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"参数必须是整数：{name}")
        if not 0 < self.early_clear_mass <= 1 or not 0 < self.geometry_sine_floor <= 1:
            raise ValueError("概率质量阈值和交会正弦下限必须在 (0,1] 内")
        if self.target_uncertainty_m <= 0 or self.clear_candidate_samples < 1:
            raise ValueError("精度目标和清除候选数必须为正")
        if not (0 < self.cell_size <= self.clear_radius * math.sqrt(2)):
            raise ValueError("兜底方格必须能被清除圆完全覆盖")
        # 中心圆与外围圆对 r in [minimum_range, region_radius] 的覆盖证明。
        for r in (self.minimum_range, self.region_radius):
            d2 = r*r + self.search_ring_radius**2 - 2*r*self.search_ring_radius*math.cos(math.pi/6)
            if d2 > self.minimum_range**2 + EPS:
                raise ValueError("搜索骨架不能覆盖目标区域")
        if self.lookahead_samples < 1 or self.max_deferred_actions < 1:
            raise ValueError("采样数和调度等待上限必须为正")


@dataclass(frozen=True)
class Cell:
    x0: float
    y0: float
    x1: float
    y1: float

    @property
    def center(self):
        return ((self.x0+self.x1)/2, (self.y0+self.y1)/2)

    @property
    def corners(self):
        return ((self.x0, self.y0), (self.x0, self.y1),
                (self.x1, self.y0), (self.x1, self.y1))

    def distance_bounds(self, p):
        dx = max(self.x0-p[0], 0, p[0]-self.x1)
        dy = max(self.y0-p[1], 0, p[1]-self.y1)
        lo = math.hypot(dx, dy)
        hi = math.hypot(max(abs(self.x0-p[0]), abs(self.x1-p[0])),
                        max(abs(self.y0-p[1]), abs(self.y1-p[1])))
        return lo, hi

    def children(self):
        x, y = self.center
        return (Cell(self.x0, self.y0, x, y), Cell(x, self.y0, self.x1, y),
                Cell(self.x0, y, x, self.y1), Cell(x, y, self.x1, self.y1))


@dataclass
class ChannelState:
    channel: int
    status: str = "UNKNOWN"
    directions: list = field(default_factory=list)
    no_signals: list = field(default_factory=list)
    failed_clears: list = field(default_factory=list)
    visited_bases: set = field(default_factory=set)
    measurements: set = field(default_factory=set)
    cells: list | None = None
    near_position: Point | None = None
    localization_measures: int = 0
    last_served: int = 0
    absence_reason: str | None = None
    geometry_cache: dict = field(default_factory=dict)

    @property
    def resolved(self):
        return self.status in ("CLEARED", "ABSENT")


@dataclass(frozen=True)
class Action:
    kind: str
    position: Point
    channel: int
    cost: float
    reason: str
    base_index: int | None = None


class ConstraintError(RuntimeError):
    """观测与保守模型矛盾；不得据此宣称目标不存在。"""


class Strategy:
    def __init__(self, config=None):
        self.cfg = config or StrategyConfig()
        self.channels = [ChannelState(c) for c in range(1, 21)]
        r = self.cfg.search_ring_radius
        self.bases = [(0.0, 0.0)] + [
            (r*math.cos(k*math.pi/3), r*math.sin(k*math.pi/3)) for k in range(6)]
        self.position = (0.0, 0.0)
        self.current_channel = 1
        self.virtual_time_s = 0.0
        self.steps = 0

    def cell_possible(self, cell, state):
        """只删除能证明不含目标的整格；保留边界不确定性。

        固定接收半径 R 满足 max(1000,所有有信号距离) <= R，
        且 R < 所有无信号距离。用距离区间保守判断是否矛盾。
        """
        cfg = self.cfg
        if cell.distance_bounds((0, 0))[0] > cfg.region_radius + EPS:
            return False
        radius_lower = cfg.minimum_range
        radius_upper = cfg.maximum_range
        if state.near_position is not None:
            if cell.distance_bounds(state.near_position)[0] > cfg.near_radius + EPS:
                return False
        for position, angle in state.directions:
            lo, hi = cell.distance_bounds(position)
            if lo > cfg.maximum_range + EPS or hi < cfg.near_radius - EPS:
                return False
            radius_lower = max(radius_lower, lo)
            # 方向扇区是两个半平面的交；整格在某半平面外才排除。
            for edge, sign in ((angle-cfg.direction_error_deg, 1),
                               (angle+cfg.direction_error_deg, -1)):
                ux, uy = math.cos(math.radians(edge)), math.sin(math.radians(edge))
                best = max(sign*(ux*(y-position[1])-uy*(x-position[0]))
                           for x, y in cell.corners)
                if best < -EPS:
                    return False
        for position in state.no_signals:
            _, hi = cell.distance_bounds(position)
            if hi < cfg.minimum_range - EPS:
                return False
            radius_upper = min(radius_upper, hi)
        if radius_lower > radius_upper + EPS:
            return False
        for position in state.failed_clears:
            if cell.distance_bounds(position)[1] <= cfg.clear_radius - EPS:
                return False
        return True

    def update_cells(self, state):
        if not state.directions and state.near_position is None:
            return
        r = self.cfg.region_radius
        stack = list(state.cells) if state.cells is not None else [Cell(-r, -r, r, r)]
        retained = []
        while stack:
            cell = stack.pop()
            if not self.cell_possible(cell, state):
                continue
            if cell.x1-cell.x0 > self.cfg.cell_size:
                stack.extend(cell.children())
            else:
                retained.append(cell)
        if not retained:
            raise ConstraintError(f"频道 {state.channel} 已发现，但可行域为空")
        state.cells = retained
        state.geometry_cache.clear()

    def action_cost(self, kind, position, channel, success=1.0):
        move = math.dist(self.position, position)/self.cfg.speed
        if kind == "measure":
            return move + 5 + (channel != self.current_channel)
        return move + 3 + 2*success

    @staticmethod
    def sample_positions(state, count):
        # 各叶格面积相同；这是几何采样，不宣称已知真实位置概率分布。
        if count in state.geometry_cache:
            return state.geometry_cache[count]
        ordered = sorted(state.cells, key=lambda c: c.center)
        n = min(count, len(ordered))
        points = [ordered[min(len(ordered)-1, int((i+.5)*len(ordered)/n))].center
                  for i in range(n)]
        state.geometry_cache[count] = points
        return points

    def clear_action(self, state):
        cfg = self.cfg
        if state.near_position is not None:
            p = state.near_position
            return Action("clear", p, state.channel,
                          self.action_cost("clear", p, state.channel), "near")
        cells = state.cells
        if not cells:
            return None
        if "clear_geometry" not in state.geometry_cache:
            center = ((min(c.x0 for c in cells)+max(c.x1 for c in cells))/2,
                      (min(c.y0 for c in cells)+max(c.y1 for c in cells))/2)
            guaranteed = all(c.distance_bounds(center)[1] <= cfg.clear_radius for c in cells)
            # 提前清除仅按几何质量估计；缓存避免调度其他频道时重复计算。
            positions = [center] + self.sample_positions(state, cfg.clear_candidate_samples)
            masses = [(p, sum(math.dist(c.center, p) <= cfg.clear_radius for c in cells)/len(cells))
                      for p in positions]
            state.geometry_cache["clear_geometry"] = center, guaranteed, masses
        center, guaranteed, masses = state.geometry_cache["clear_geometry"]
        if guaranteed:
            return Action("clear", center, state.channel,
                          self.action_cost("clear", center, state.channel), "guaranteed")
        # 有限兜底：每次选一格中心，失败至少删除该整格，成功则结束。
        fallback = (state.localization_measures >= cfg.max_localization_measures or
                    len(state.failed_clears) >= cfg.max_failed_clears)
        candidates = masses
        if fallback:
            p = min(cells, key=lambda c: math.dist(c.center, self.position)).center
            fraction = sum(math.dist(c.center, p) <= cfg.clear_radius for c in cells)/len(cells)
            candidates = [(p, fraction)]
        best = None
        for p, fraction in candidates:
            if not fallback and fraction < cfg.early_clear_mass:
                continue
            cost = self.action_cost("clear", p, state.channel, fraction)
            # 软质量只用于提前尝试；失败后继续，不据此宣布已清除。
            cost += (1-fraction)*cfg.failed_clear_penalty_s
            action = Action("clear", p, state.channel, cost,
                            "finite_cover" if fallback else "early_clear")
            if best is None or cost < best.cost:
                best = action
        return best

    def localization_action(self, state):
        cfg = self.cfg
        samples = self.sample_positions(state, cfg.lookahead_samples)
        center = (sum(p[0] for p in samples)/len(samples),
                  sum(p[1] for p in samples)/len(samples))
        _, angle = state.directions[-1]
        theta = math.radians(angle)
        side = (-math.sin(theta), math.cos(theta))
        candidates = [center]
        # 同时考虑靠近目标与横向改变视角；允许圆域外检测。
        for anchor in (center, self.position):
            for offset in cfg.lateral_offsets:
                for sign in (-1, 1):
                    candidates.append((anchor[0]+sign*offset*side[0],
                                       anchor[1]+sign*offset*side[1]))
        best = None
        for p in candidates:
            key = (round(p[0], 6), round(p[1], 6))
            if key in state.measurements:
                continue
            expected_tail = 0.0
            for target in samples:
                distance = math.dist(p, target)
                # 半径未知时采用最小接收半径，不虚构独立接收概率。
                if distance > cfg.minimum_range:
                    expected_tail += distance/cfg.speed + cfg.no_signal_penalty_s
                    continue
                if distance <= cfg.near_radius:
                    expected_tail += 5
                    continue
                # 两条含误差射线的局部误差传播仅作启发式耗时预测。
                # 对所有已有方向取最好的交会几何，不参与硬约束。
                uncertainty = float("inf")
                for old, _ in state.directions:
                    a = (target[0]-old[0], target[1]-old[1])
                    b = (target[0]-p[0], target[1]-p[1])
                    d1 = math.hypot(*a)
                    sine = abs(a[0]*b[1]-a[1]*b[0])/max(d1*distance, EPS)
                    bound = math.radians(cfg.direction_error_deg)*(d1+distance)/max(sine, cfg.geometry_sine_floor)
                    uncertainty = min(uncertainty, bound)
                extra = max(0.0, math.log2(max(1.0, uncertainty/cfg.target_uncertainty_m)))
                expected_tail += distance/cfg.speed + 5 + cfg.extra_measure_penalty_s*extra
            score = self.action_cost("measure", p, state.channel) + expected_tail/len(samples)
            action = Action("measure", p, state.channel, score, "localize")
            if best is None or score < best.cost:
                best = action
        if best is None:
            # 候选全被测过时转入有限覆盖，禁止原地重复积累虚假信息。
            state.localization_measures = cfg.max_localization_measures
            return self.clear_action(state)
        return best

    def channel_action(self, state):
        if state.directions or state.near_position is not None:
            clear = self.clear_action(state)
            if clear and clear.reason in ("near", "guaranteed", "finite_cover"):
                return clear
            measure = self.localization_action(state)
            return min((a for a in (clear, measure) if a), key=lambda a: a.cost)
        actions = []
        for i, p in enumerate(self.bases):
            if i in state.visited_bases:
                continue
            # 到点后的其他频道检测共享移动成本，鼓励原地完成有用扫描。
            pending = sum(not c.resolved and not c.directions and
                          c.near_position is None and i not in c.visited_bases
                          for c in self.channels)
            cost = math.dist(self.position, p)/self.cfg.speed/max(1, pending)
            cost += 5 + (state.channel != self.current_channel)
            actions.append(Action("measure", p, state.channel, cost, "search", i))
        if not actions:
            raise ConstraintError(f"频道 {state.channel} 的覆盖状态矛盾")
        return min(actions, key=lambda a: a.cost)

    def choose_action(self):
        # 题目明确上限为 16；已证实 16 个不同频道存在后，其余必不存在。
        known = sum(c.status == "CLEARED" or bool(c.directions) or
                    c.near_position is not None for c in self.channels)
        if known > 16:
            raise ConstraintError("检测到的不同干扰源超过题目上限 16")
        if known == 16:
            for c in self.channels:
                if c.status == "UNKNOWN":
                    c.status = "ABSENT"
                    c.absence_reason = "known_count_upper_bound"
        active = [c for c in self.channels if not c.resolved]
        if not active:
            return None
        # 近距离结果可立即兑现，正常情况下只有当前频道会有此状态。
        near = [c for c in active if c.near_position is not None]
        if near:
            return self.clear_action(near[0])
        oldest = min(active, key=lambda c: c.last_served)
        if self.steps-oldest.last_served >= self.cfg.max_deferred_actions:
            return self.channel_action(oldest)
        return min((self.channel_action(c) for c in active), key=lambda a: a.cost)

    def update(self, action, response):
        if response.get("accepted") is not True:
            raise RuntimeError("模拟器拒绝动作；位置和虚拟时间未更新")
        state = self.channels[action.channel-1]
        self.position = action.position
        self.virtual_time_s = float(response["virtual_time_s"])
        self.steps += 1
        state.last_served = self.steps
        if action.kind == "clear":
            result = response["clear_result"]
            if result == "success":
                state.status = "CLEARED"
                state.near_position = None
                state.cells = None
                return
            if result != "no_target_in_range":
                raise RuntimeError(f"未知清除结果：{result}")
            state.failed_clears.append(action.position)
            self.update_cells(state)
            return
        self.current_channel = action.channel
        state.measurements.add(tuple(round(v, 6) for v in action.position))
        if action.reason == "localize":
            state.localization_measures += 1
        result = response["measure_result"]
        if result == "direction":
            angle = float(response["svd_deg"])
            if not math.isfinite(angle) or not 0 <= angle < 360:
                raise RuntimeError("示向度不是有效角度")
            state.directions.append((action.position, angle))
            state.status = "LOCALIZING"
        elif result == "near":
            state.near_position = action.position
            state.status = "CLEAR_READY"
        elif result == "no_signal":
            state.no_signals.append(action.position)
            if action.base_index is not None:
                state.visited_bases.add(action.base_index)
            if len(state.visited_bases) == len(self.bases):
                if state.directions or state.near_position is not None:
                    raise ConstraintError("曾发现的频道不能因覆盖记录变为不存在")
                state.status = "ABSENT"
                state.absence_reason = "seven_point_coverage"
                return
        else:
            raise RuntimeError(f"未知检测结果：{result}")
        self.update_cells(state)


ROOT = Path(__file__).resolve().parent
DEFAULT_PARAMS = ROOT / "params.json"
DEFAULT_CANDIDATES = ROOT / "candidates.json"
DEFAULT_LOG = ROOT / "output" / "offline_runs.json"


def algorithm_version():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()[:16]


def config_from_dict(data):
    data = dict(data)
    if "lateral_offsets" in data:
        data["lateral_offsets"] = tuple(data["lateral_offsets"])
    return StrategyConfig(**data)


def config_id(config):
    return hashlib.sha256(json.dumps(asdict(config), sort_keys=True).encode()).hexdigest()[:16]


def load_config(path=DEFAULT_PARAMS):
    path = Path(path)
    if not path.exists():
        return StrategyConfig()
    data = json.loads(path.read_text(encoding="utf-8"))
    return config_from_dict(data.get("config", data))


def atomic_json(path, data):
    """同一时刻只运行一个批处理/迭代入口；原子替换避免半份 JSON。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=path.name+".", suffix=".tmp", delete=False) as stream:
            temporary = stream.name
            json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)


def run(interface, config=None, on_event=None, max_actions=10000):
    """两个环境共用的执行器；只调用四个接口，不读取真值或管理文件。"""
    strategy = Strategy(config)
    cfg = strategy.cfg
    start = time.monotonic()
    deadline = start
    outcome, error = "error", None
    entered = exit_attempted = False
    last_virtual = 0.0
    counts = {"move_distance_m": 0.0, "measure_count": 0, "switch_count": 0,
              "clear_attempts": 0, "failed_clear_count": 0, "fallback_count": 0}
    reasons = {}

    def emit(event):
        if on_event is not None:
            on_event(event)

    def perform_exit():
        nonlocal exit_attempted, last_virtual
        exit_attempted = True
        emit({"event": "request", "action": {"kind": "exit"}})
        response = interface.exit_robot()
        emit({"event": "response", "response": response})
        if response.get("accepted") is not True:
            raise RuntimeError("退出未被接受")
        last_virtual = float(response["virtual_time_s"])

    try:
        emit({"event": "request", "action": {"kind": "enter"}})
        response = interface.enter()
        emit({"event": "response", "response": response})
        if response.get("accepted") is not True:
            raise RuntimeError("进入失败，请检查模拟器就绪状态与参赛队号")
        entered = True
        deadline = start + float(response["remaining_real_duration_s"])
        virtual_limit = float(response["max_virtual_duration_s"])
        while True:
            if time.monotonic() >= deadline-cfg.real_time_reserve_s:
                outcome = "real_time_limit"
                break
            if strategy.steps >= max_actions:
                outcome = "action_limit"
                break
            action = strategy.choose_action()
            if action is None:
                outcome = "completed"
                break
            if time.monotonic() >= deadline-cfg.real_time_reserve_s:
                outcome = "real_time_limit"
                break
            if strategy.virtual_time_s + strategy.action_cost(
                    action.kind, action.position, action.channel) >= virtual_limit:
                outcome = "virtual_time_limit"
                break
            emit({"event": "request", "action": asdict(action)})
            function = interface.measure if action.kind == "measure" else interface.clear
            response = function(*action.position, action.channel)
            emit({"event": "response", "response": response})
            if response.get("accepted") is True:
                last_virtual = float(response["virtual_time_s"])
                counts["move_distance_m"] += math.dist(strategy.position, action.position)
                reasons[action.reason] = reasons.get(action.reason, 0)+1
                if action.kind == "measure":
                    counts["measure_count"] += 1
                    counts["switch_count"] += action.channel != strategy.current_channel
                else:
                    counts["clear_attempts"] += 1
                    counts["failed_clear_count"] += response.get("clear_result") == "no_target_in_range"
                    counts["fallback_count"] += action.reason == "finite_cover"
            strategy.update(action, response)
        if time.monotonic() < deadline:
            perform_exit()
    except Exception as exc:
        outcome = "error"
        error = {"type": type(exc).__name__, "message": str(exc)}
        emit({"event": "error", **error})
        # 网络异常不重发新动作；本地约束矛盾时可主动结束本局。
        if isinstance(exc, ConstraintError) and entered and not exit_attempted and time.monotonic() < deadline:
            try:
                perform_exit()
            except Exception as exit_exc:
                emit({"event": "exit_error", "message": str(exit_exc)})
    cleared = sum(c.status == "CLEARED" for c in strategy.channels)
    summary = {
        "outcome": outcome, "error": error, "cleared_count": cleared,
        "absent_count": sum(c.status == "ABSENT" for c in strategy.channels),
        "unresolved_channels": [c.channel for c in strategy.channels if not c.resolved],
        "virtual_time_s": last_virtual,
        "average_clear_time_s": last_virtual/cleared if cleared else None,
        "program_elapsed_s": time.monotonic()-start, "actions": strategy.steps,
        "action_reasons": reasons, **counts,
    }
    emit({"event": "summary", **summary})
    return summary

