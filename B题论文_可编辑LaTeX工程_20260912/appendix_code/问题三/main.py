"""官方模拟器入口；请先在模拟器界面启动测试并等待接口就绪。"""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

import algorithm
import basic_sim


def main():
    parser = argparse.ArgumentParser(description="使用当前参数执行一局官方测试，不自动开启测试")
    parser.add_argument("--params", type=Path, default=algorithm.DEFAULT_PARAMS)
    parser.add_argument("--log-dir", type=Path, default=algorithm.ROOT / "output" / "official")
    args = parser.parse_args()
    config = algorithm.load_config(args.params)
    if not args.params.exists():
        print("尚无已选参数文件，本次使用 algorithm.py 的默认参数。")
    args.log_dir.mkdir(parents=True, exist_ok=True)
    path = args.log_dir / f"run_{time.time_ns()}.jsonl"
    with path.open("w", encoding="utf-8") as stream:
        def record(event):
            stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False)+"\n")
            stream.flush()
        record({"algorithm_version": algorithm.algorithm_version(), "config": asdict(config)})
        result = algorithm.run(basic_sim, config, record)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"程序日志：{path}；正式测试加密日志仍需从官方模拟器导出。")
    return 0 if result["outcome"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
