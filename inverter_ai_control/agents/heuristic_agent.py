"""简单的启发式智能体用于最小可运行样例。

目的：
- 在无 RL 框架与无 MATLAB 环境下，打通数据流与日志
- 保持与 BaseAgent 接口兼容
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import numpy as np

from inverter_ai_control.agents.base_agent import BaseAgent, Observation, Action, AgentConfig
from inverter_ai_control.utils.logger import get_logger


log = get_logger(__name__)


@dataclass(frozen=True)
class HeuristicAgentConfig(AgentConfig):
    step_gain: float = 0.0  # 演示参数：对某个参考的简单比例调整


class HeuristicAgent(BaseAgent):
    """最小实现：将 d_axis_ref 保持不变，q_axis_ref 恒为 0。

    说明：
    - 仅作为连通性验证，不包含控制策略
    """

    def __init__(self, config: HeuristicAgentConfig | None = None):
        super().__init__(config)
        self._cfg: HeuristicAgentConfig = config or HeuristicAgentConfig()

    def act(self, observation: Observation, deterministic: bool = True) -> Action:
        # 从观测中读取 d 轴/电流等（如果存在）
        d_ref = float(observation.signals.get("d_axis_ref", np.array([1.0]))[-1]) if observation.signals else 1.0
        action = Action(setpoints={
            "d_axis_ref": d_ref,  # 保持当前参考
            "q_axis_ref": 0.0,    # 简化：恒为 0
        })
        log.info("agent.act", deterministic=deterministic, keys=list(action.setpoints.keys()))
        return action


