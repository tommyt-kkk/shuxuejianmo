"""兼容旧入口；算法已移至 algorithm.py，官方执行入口为 main.py。"""
from algorithm import Action, Cell, ChannelState, ConstraintError, Strategy, StrategyConfig, run
from main import main

if __name__ == "__main__":
    raise SystemExit(main())
