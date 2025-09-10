"""Simulation environment layer.

只负责：
- 与 MATLAB/Simulink 的交互
- 设参/写入控制量/读取观测/推进时间

不负责：
- 奖励/终止/episode 等训练语义（放在适配器或上层应用）
"""

__all__ = [
    "matlab_simulator",
]


