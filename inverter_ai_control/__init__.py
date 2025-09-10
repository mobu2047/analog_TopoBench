"""Top-level package for inverter AI control framework.

该包聚焦于分层解耦：
- sim_env: 仅处理与 MATLAB/Simulink 的交互，不包含 RL/Gym 语义
- agents: 仅处理从观测到动作的策略，不依赖 MATLAB/Gym
- utils: 仅提供通用工具（日志、信号处理、可视化等）

注意：请通过 `main.py` 装配运行时依赖，避免在包初始化时做副作用操作。
"""

__all__ = [
    "sim_env",
    "agents",
    "utils",
]


