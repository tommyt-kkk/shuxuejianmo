"""自动执行离线测试和参数迭代循环；绝不连接官方模拟器。"""

import argparse
from pathlib import Path

import algorithm
import iterator
import sss


def run_cycles(rounds, runs, candidate_count=4, seed=2026,
               log_path=algorithm.DEFAULT_LOG, params_path=algorithm.DEFAULT_PARAMS,
               candidates_path=algorithm.DEFAULT_CANDIDATES):
    if rounds < 1 or runs < 10 or candidate_count < 1:
        raise ValueError("轮数与候选数必须为正；自动迭代每组至少 10 局（含 2 局验证）")
    total = rounds*runs*(candidate_count+1)
    print(f"计划 {rounds} 轮，每组 {runs} 局，每轮当前参数 + {candidate_count} 组候选，"
          f"预计 {total} 局。Ctrl+C 可停止。", flush=True)
    # 首轮前生成候选；这一步只读日志、写参数，不执行测试。
    iterator.iterate(log_path, params_path, candidates_path, candidate_count, seed)
    for index in range(rounds):
        print(f"\n第 {index+1}/{rounds} 轮", flush=True)
        # 各轮使用新的验证案例，降低反复查看同一验证集的过拟合风险。
        # 同一轮的全部参数组合则使用完全相同的案例。
        round_seed = seed+index*runs
        batch = sss.run_batch(runs, round_seed, log_path, params_path, candidates_path,
                              label=f"auto_round_{index+1}")
        if batch["status"] != "completed":
            raise RuntimeError("本轮未完成，不更新参数")
        iterator.iterate(log_path, params_path, candidates_path, candidate_count, seed)
    print(f"完成 {rounds} 轮。离线日志：{log_path}\n"
          f"通过验证的参数：{params_path}（无合格结果时不生成或不更新）。", flush=True)


def main():
    parser = argparse.ArgumentParser(description="自动循环：离线模拟 -> 读取 JSON 优化参数")
    parser.add_argument("--rounds", type=int, required=True, help="测试/迭代循环轮数")
    parser.add_argument("--runs", type=int, default=10, help="每组参数每轮的案例数，至少 10")
    parser.add_argument("--candidate-count", type=int, default=4, help="每轮待比较的候选参数组数")
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--log", type=Path, default=algorithm.DEFAULT_LOG)
    parser.add_argument("--params", type=Path, default=algorithm.DEFAULT_PARAMS)
    parser.add_argument("--candidates", type=Path, default=algorithm.DEFAULT_CANDIDATES)
    args = parser.parse_args()
    if args.rounds < 1 or args.runs < 10 or args.candidate_count < 1:
        parser.error("--rounds、--candidate-count 必须为正，--runs 至少为 10")
    try:
        run_cycles(args.rounds, args.runs, args.candidate_count, args.seed,
                   args.log, args.params, args.candidates)
    except KeyboardInterrupt:
        print("已停止自动循环，已完成的记录仍在统一 JSON 中。")
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
