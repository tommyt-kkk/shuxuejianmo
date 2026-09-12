"""读取离线 JSON，选择已验证参数并提出新候选；本模块不执行模拟。

采用离散坐标邻域搜索，辅以多参数随机变异。验证集仅用于检查训练
优胜者能否替换现有参数，不用验证分数对全部候选排序。
"""

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import random
import statistics
import time

import algorithm
from sss import environment_version, load_log


# 只允许效率参数变化，题目常量与保守误差边界不进入搜索空间。
SEARCH_SPACE = {
    "lateral_offsets": ((20.0, 60.0, 140.0), (40.0, 100.0, 220.0),
                        (60.0, 150.0, 300.0), (100.0, 220.0, 400.0)),
    "early_clear_mass": (0.55, 0.65, 0.72, 0.8, 0.9),
    "max_localization_measures": (3, 4, 6, 8),
    "max_failed_clears": (1, 2, 3, 4),
    "failed_clear_penalty_s": (5.0, 12.0, 25.0, 50.0),
    "extra_measure_penalty_s": (8.0, 18.0, 35.0, 60.0),
    "no_signal_penalty_s": (60.0, 150.0, 300.0),
    "target_uncertainty_m": (8.0, 12.0, 16.0),
    "max_deferred_actions": (30, 50, 70, 100),
    "search_ring_radius": (1450.0, 1500.0, 1550.0, 1600.0),
    # 四叉树实际叶格约为 7.03、14.06、28.125 米，均可被 20 米圆覆盖。
    "cell_size": (8.0, 16.0, 28.2),
    "lookahead_samples": (5, 9, 15, 21),
    "clear_candidate_samples": (9, 15, 25),
    "geometry_sine_floor": (0.01, 0.015, 0.03),
}


def aggregate(records):
    times = sorted(r["metrics"]["virtual_time_s"] for r in records)
    if not times:
        return None
    p90 = times[min(len(times)-1, math.ceil(.9*len(times))-1)]
    mean = statistics.mean(times)
    return {"cases": len(records), "failures": sum(not r["metrics"]["complete"] for r in records),
            "mean_virtual_time_s": mean, "p90_virtual_time_s": p90,
            "objective": .8*mean+.2*p90,
            "mean_real_time_s": statistics.mean(r["metrics"]["program_elapsed_s"] for r in records)}


def choose_tested(data, current, min_train_cases=3, min_validation_cases=2):
    version, env = algorithm.algorithm_version(), environment_version()
    batches = [b for b in data["batches"] if b["status"] == "completed" and
               b["algorithm_version"] == version and b["environment_version"] == env]
    if not batches:
        return current, {"status": "no_completed_batch", "validated": False}
    batch = batches[-1]
    incumbent = algorithm.config_id(current)
    if incumbent not in batch["configs"]:
        return current, {"status": "current_config_not_compared", "validated": False,
                         "batch_id": batch["batch_id"]}
    records = [r for r in data["runs"] if r["batch_id"] == batch["batch_id"] and
               r["status"] == "finished"]
    metrics = {}
    for config_key in batch["configs"]:
        metrics[config_key] = {}
        for split in ("train", "validation"):
            expected = {c["case_id"] for c in batch["cases"] if c["split"] == split}
            actual = [r for r in records if r["config_id"] == config_key and r["split"] == split]
            if len(actual) != len(expected) or {r["case_id"] for r in actual} != expected:
                return current, {"status": "incomplete_comparison", "validated": False}
            metrics[config_key][split] = aggregate(actual)
    report = {"batch_id": batch["batch_id"], "metrics": metrics, "validated": False}
    incumbent_metrics = metrics[incumbent]
    if (not incumbent_metrics["train"] or not incumbent_metrics["validation"] or
            incumbent_metrics["train"]["cases"] < min_train_cases or
            incumbent_metrics["validation"]["cases"] < min_validation_cases):
        return current, {**report, "status": "insufficient_train_or_validation_cases"}
    # 完成率优先；耗时目标兼顾平均和较差案例。
    key = lambda cid: (metrics[cid]["train"]["failures"], metrics[cid]["train"]["objective"],
                       cid != incumbent)
    winner = min(metrics, key=key)
    report["training_winner"] = winner
    if metrics[winner]["train"]["failures"]:
        return current, {**report, "status": "no_fully_cleared_training_config"}
    validation = metrics[winner]["validation"]
    baseline = incumbent_metrics["validation"]
    if validation["failures"] or (baseline["failures"] == 0 and
                                   validation["objective"] > baseline["objective"]*1.02):
        return current, {**report, "status": "validation_rejected"}
    selected = algorithm.config_from_dict(batch["configs"][winner])
    return selected, {**report, "status": "selected" if winner != incumbent else "incumbent_validated",
                      "validated": True, "selected_id": winner}


def propose(current, count, generation, seed, tested_ids=()):
    """轮换探索参数维度，先单参数邻域，再生成多参数变异；不声称新候选更优。"""
    if count < 1:
        raise ValueError("候选组数必须为正")
    rng = random.Random(seed + generation*1009)
    base = asdict(current)
    blocked = set(tested_ids) | {algorithm.config_id(current)}
    results = []

    def add(data):
        try:
            config = algorithm.config_from_dict(data)
        except (ValueError, TypeError):
            return
        identity = algorithm.config_id(config)
        if identity not in blocked:
            blocked.add(identity)
            results.append({"config_id": identity, "config": asdict(config)})

    names = list(SEARCH_SPACE)
    start = (generation*count) % len(names)
    names = names[start:]+names[:start]
    # 每一遍每个维度至多提出一个邻居，避免候选全落在同一维度。
    choices = {}
    for name in names:
        choices[name] = list(SEARCH_SPACE[name])
        rng.shuffle(choices[name])
    for index in range(max(map(len, choices.values()))):
        for name in names:
            if index < len(choices[name]):
                add({**base, name: choices[name][index]})
                if len(results) == count:
                    return results
    for _ in range(max(1000, count*20)):
        candidate = dict(base)
        for name in rng.sample(names, rng.randint(2, min(4, len(names)))):
            candidate[name] = rng.choice(SEARCH_SPACE[name])
        add(candidate)
        if len(results) == count:
            return results
    return results


def iterate(log_path=algorithm.DEFAULT_LOG, params_path=algorithm.DEFAULT_PARAMS,
            candidates_path=algorithm.DEFAULT_CANDIDATES, candidate_count=4, seed=2026):
    if candidate_count < 1:
        raise ValueError("candidate_count 必须为正")
    data = load_log(log_path)
    current = algorithm.load_config(params_path)
    selected, report = choose_tested(data, current)
    generation = len(data["iterations"])
    valid_batches = {b["batch_id"] for b in data["batches"]
                     if b["algorithm_version"] == algorithm.algorithm_version() and
                     b["environment_version"] == environment_version()}
    tested = {r["config_id"] for r in data["runs"] if r["batch_id"] in valid_batches and
              r["status"] == "finished"}
    candidates = propose(selected, candidate_count, generation, seed, tested)
    if report["validated"]:
        algorithm.atomic_json(params_path, {"schema_version": 1, "status": "offline_validated",
                              "algorithm_version": algorithm.algorithm_version(),
                              "environment_version": environment_version(),
                              "config_id": algorithm.config_id(selected), "config": asdict(selected),
                              "evidence_batch_id": report["batch_id"], "updated_at": time.time()})
    algorithm.atomic_json(candidates_path, {"schema_version": 1,
                          "algorithm_version": algorithm.algorithm_version(), "generation": generation,
                          "base_config_id": algorithm.config_id(selected), "candidates": candidates})
    data["iterations"].append({"created_at": time.time(), "generation": generation,
                               "seed": seed, "algorithm_version": algorithm.algorithm_version(),
                               "previous_config_id": algorithm.config_id(current),
                               "selected_config_id": algorithm.config_id(selected), "report": report,
                               "candidates": candidates})
    algorithm.atomic_json(log_path, data)
    print(f"参数迭代：{report['status']}；当前 {algorithm.config_id(selected)}；"
          f"下一轮候选 {len(candidates)} 组。", flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description="只读取已完成的离线日志，选参并生成下一批候选")
    parser.add_argument("--log", type=Path, default=algorithm.DEFAULT_LOG)
    parser.add_argument("--params", type=Path, default=algorithm.DEFAULT_PARAMS)
    parser.add_argument("--candidates", type=Path, default=algorithm.DEFAULT_CANDIDATES)
    parser.add_argument("--candidate-count", type=int, default=4)
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args()
    if args.candidate_count < 1:
        parser.error("--candidate-count 必须为正")
    iterate(args.log, args.params, args.candidates, args.candidate_count, args.seed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
