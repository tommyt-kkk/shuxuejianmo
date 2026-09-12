"""离线环境、评价与批量运行。python sss.py --runs 10"""

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import random
import time
import uuid

import algorithm


def environment_version():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()[:16]


def load_log(path):
    path = Path(path)
    if not path.exists():
        return {"schema_version": 1, "batches": [], "runs": [], "iterations": []}
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError("不支持此日志格式")
    return data


def make_case(seed, profile="uniform", error_mode="spatial"):
    rng = random.Random(seed)
    targets = {}
    for c in rng.sample(range(1, 21), rng.randint(10, 16)):
        angle = rng.uniform(0, 2*math.pi)
        radius = 1800 if profile == "boundary" else 1800*math.sqrt(rng.random())
        if profile == "cluster":
            angle = rng.uniform(-.2, .2)
            radius = rng.uniform(700, 1100)
        reception = 1000 if profile == "boundary" else rng.uniform(1000, 1500)
        targets[c] = (radius*math.cos(angle), radius*math.sin(angle), reception)
    return {"seed": seed, "profile": profile, "error_mode": error_mode, "targets": targets}


class OfflineSimulator:
    """与 basic_sim 四个函数相同的调用/响应接口；不模拟 HTTP 传输故障。

    真值只供环境与局后评价使用。固定位置误差固定，接收半径每局固定。
    """
    def __init__(self, case, real_limit_s=1200):
        self._case = case
        self._remaining = case["targets"].copy()
        self._position = (0.0, 0.0)
        self._channel = 1
        self._virtual = 0.0
        self._started = None
        self._ended = False
        self._real_limit = real_limit_s

    def _response(self, accepted=True, **extra):
        return {"accepted": accepted, "real_timestamp_ms": int(time.time()*1000),
                "virtual_time_s": round(self._virtual, 6) if accepted else 0, **extra}

    def _active(self):
        if self._started is None or self._ended:
            return False
        if time.monotonic()-self._started >= self._real_limit or self._virtual >= 360000:
            self._ended = True
            raise ConnectionError("离线模拟器测试时间已到，接口关闭")
        return True

    def enter(self):
        if self._started is not None or self._ended:
            return self._response(False)
        self._started = time.monotonic()
        return self._response(max_virtual_duration_s=360000, max_real_duration_s=1200,
                              remaining_real_duration_s=self._real_limit)

    def _move(self, x, y, channel):
        if not self._active():
            return False
        if type(channel) is not int or not 1 <= channel <= 20:
            raise ValueError("频道必须是 1..20 的整数")
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(v) or abs(v) > 2000000 for v in (x, y)):
            raise ValueError("坐标必须有限且绝对值不超过 2000000")
        self._virtual += math.dist(self._position, (x, y))/5
        self._position = (x, y)
        return True

    def measure(self, x, y, channel):
        if not self._move(x, y, channel):
            return self._response(False)
        self._virtual += 5 + (self._channel != channel)
        self._channel = channel
        target = self._remaining.get(channel)
        distance = math.dist((x, y), target[:2]) if target else math.inf
        if target is None or distance > target[2]:
            return self._response(measure_result="no_signal")
        if distance <= 5:
            return self._response(measure_result="near")
        mode = self._case["error_mode"]
        phase = self._case["seed"]*.173 + channel*1.77
        error = .65*math.sin(x*.031+y*.023+phase) + .35*math.sin(x*.007-y*.017+phase)
        if mode == "positive":
            error = 1.0
        elif mode == "negative":
            error = -1.0
        angle = math.degrees(math.atan2(target[1]-y, target[0]-x))
        return self._response(measure_result="direction", svd_deg=round((angle+error) % 360, 2) % 360)

    def clear(self, x, y, channel):
        if not self._move(x, y, channel):
            return self._response(False)
        target = self._remaining.get(channel)
        success = target is not None and math.dist((x, y), target[:2]) <= 20
        self._virtual += 5 if success else 3
        if success:
            del self._remaining[channel]
        return self._response(clear_result="success" if success else "no_target_in_range")

    def exit_robot(self):
        if not self._active():
            return self._response(False)
        self._ended = True
        return self._response(exit_reason="user_exit")


def evaluate(simulator, summary):
    """只在执行结束后读取真值，不向算法提供真值。"""
    total = len(simulator._case["targets"])
    cleared = total-len(simulator._remaining)
    return {**summary, "target_count": total, "true_cleared_count": cleared,
            "clear_ratio": cleared/total,
            "complete": summary["outcome"] == "completed" and cleared == total,
            "remaining_channels": sorted(simulator._remaining)}


def case_plan(runs, seed):
    if runs < 1:
        raise ValueError("runs 必须为正")
    # 每第五个案例作为验证集；相同参数组合始终使用相同案例。
    profiles = ("uniform", "boundary", "cluster")
    errors = ("spatial", "positive", "negative")
    plan = []
    for i in range(runs):
        descriptor = {"seed": seed+i, "profile": profiles[i % 3],
                      "error_mode": errors[(i//3) % 3],
                      "split": "validation" if i % 5 == 4 else "train"}
        descriptor["case_id"] = hashlib.sha256(
            json.dumps(descriptor, sort_keys=True).encode()).hexdigest()[:16]
        plan.append(descriptor)
    return plan


def batch_configs(params_path, candidates_path):
    current = algorithm.load_config(params_path)
    configs = {algorithm.config_id(current): current}
    path = Path(candidates_path) if candidates_path else None
    if path and path.exists():
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("algorithm_version") != algorithm.algorithm_version():
            raise ValueError("候选参数属于其他算法版本，请先运行 iterator.py 重新生成")
        for item in data["candidates"]:
            config = algorithm.config_from_dict(item["config"])
            configs[algorithm.config_id(config)] = config
    return current, configs


def run_batch(runs=10, seed=2026, log_path=algorithm.DEFAULT_LOG,
              params_path=algorithm.DEFAULT_PARAMS, candidates_path=algorithm.DEFAULT_CANDIDATES,
              label=None):
    current, configs = batch_configs(params_path, candidates_path)
    plan = case_plan(runs, seed)
    data = load_log(log_path)
    batch = {"batch_id": uuid.uuid4().hex, "created_at": time.time(), "label": label,
             "algorithm_version": algorithm.algorithm_version(), "environment_version": environment_version(),
             "incumbent_id": algorithm.config_id(current),
             "configs": {key: asdict(value) for key, value in configs.items()},
             "cases": plan, "status": "running", "expected_runs": len(plan)*len(configs)}
    data["batches"].append(batch)
    algorithm.atomic_json(log_path, data)
    completed = 0
    try:
        for config_key, config in configs.items():
            for descriptor in plan:
                case = make_case(descriptor["seed"], descriptor["profile"], descriptor["error_mode"])
                simulator = OfflineSimulator(case)
                trace = []
                started = time.time()
                try:
                    summary = algorithm.run(simulator, config, trace.append)
                except KeyboardInterrupt:
                    data["runs"].append({"batch_id": batch["batch_id"], "config_id": config_key,
                                         "case_id": descriptor["case_id"], "split": descriptor["split"],
                                         "started_at": started, "status": "interrupted", "trace": trace})
                    raise
                record = {"run_id": uuid.uuid4().hex, "batch_id": batch["batch_id"],
                          "config_id": config_key, "config": asdict(config),
                          "case_id": descriptor["case_id"], "case": descriptor,
                          "split": descriptor["split"], "started_at": started, "status": "finished",
                          "metrics": evaluate(simulator, summary), "trace": trace}
                data["runs"].append(record)
                completed += 1
                # 每完成一局就保存；中途停止不丢失此前结果。
                algorithm.atomic_json(log_path, data)
                metrics = record["metrics"]
                print(f"[{completed}/{batch['expected_runs']}] {config_key[:6]} "
                      f"case={descriptor['seed']} {descriptor['split']} "
                      f"clear={metrics['true_cleared_count']}/{metrics['target_count']} "
                      f"time={metrics['virtual_time_s']:.2f}s {metrics['outcome']}", flush=True)
        batch["status"] = "completed"
    except BaseException:
        batch["status"] = "interrupted"
        raise
    finally:
        batch["finished_at"] = time.time()
        algorithm.atomic_json(log_path, data)
    return batch


def main():
    parser = argparse.ArgumentParser(description="离线测试当前参数和已有候选，日志汇总为一个 JSON")
    parser.add_argument("--runs", type=int, default=10, help="每组参数的案例数，默认 10")
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--log", type=Path, default=algorithm.DEFAULT_LOG)
    parser.add_argument("--params", type=Path, default=algorithm.DEFAULT_PARAMS)
    parser.add_argument("--candidates", type=Path, default=algorithm.DEFAULT_CANDIDATES)
    parser.add_argument("--current-only", action="store_true", help="只测试当前参数")
    args = parser.parse_args()
    if args.runs < 1:
        parser.error("--runs 必须为正")
    try:
        run_batch(args.runs, args.seed, args.log, args.params,
                  None if args.current_only else args.candidates)
    except KeyboardInterrupt:
        print("已停止，已完成的离线日志已保存。")
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
