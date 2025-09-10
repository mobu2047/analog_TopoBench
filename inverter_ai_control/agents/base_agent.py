"""智能体抽象基类：强调与仿真层解耦。

设计动机：
- BaseAgent 不依赖 MATLAB/Simulink/Gym，仅处理纯数据
- 具体智能体（RL/MCP/Heuristic）实现相同接口，便于热插拔

新增统一接口（关键）：
- get_action(state): 由任意“状态”生成“动作”，保持与环境解耦
- update(state, action, reward, next_state, done): 可选用于 RL 在线学习
- save(path)/load(path): 持久化/恢复智能体

兼容性说明：
- 为兼容旧代码，仍提供 act()/learn() 包装；但推荐未来统一使用 get_action()/update()
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np

from inverter_ai_control.utils.logger import get_logger


log = get_logger(__name__)


@dataclass(frozen=True)
class Observation:
    """上层应用传入的观测（纯数据）。"""

    signals: Dict[str, np.ndarray]
    timestamp_s: float


@dataclass(frozen=True)
class Action:
    """智能体输出的控制量（纯数据）。"""

    setpoints: Dict[str, float]


@dataclass(frozen=True)
class AgentConfig:
    """智能体超参数（纯数据）。"""

    seed: int = 0
    exploration_epsilon: float = 0.05
    extra: Dict[str, Any] = None


class BaseAgent(ABC):
    """智能体抽象基类：仅定义算法交互，不绑定任何仿真或框架细节。"""

    def __init__(self, config: Optional[AgentConfig] = None):
        self._config = config or AgentConfig()
        log.info("agent.init", cfg=dict(seed=self._config.seed, eps=self._config.exploration_epsilon))

    # ---------------------------
    # 新统一抽象接口（强制实现）
    # ---------------------------
    @abstractmethod
    def get_action(self, state: Any) -> Any:
        """根据输入状态返回一个动作（任意可序列化的纯数据）。

        为什么：
        - 将智能体与具体环境/观测结构解耦（state 可以是 ndarray/dict/namedtuple 等）
        - 便于在不同仿真/任务中复用同一智能体实现
        """
        raise NotImplementedError

    @abstractmethod
    def update(self, state: Any, action: Any, reward: float, next_state: Any, done: bool) -> None:
        """使用一次交互经验进行学习更新（可为空实现但需定义）。

        说明：
        - 对于无学习（启发式/规则）的智能体，可以留空实现
        - 对于 RL 智能体，通常会写入经验回放、执行一次优化步骤等
        """
        raise NotImplementedError

    @abstractmethod
    def save(self, path: str) -> None:
        """保存模型/参数到路径。格式由具体实现决定（如 .pt/.ckpt/.pkl）。"""
        raise NotImplementedError

    @abstractmethod
    def load(self, path: str) -> None:
        """从路径加载模型/参数。格式需与 save() 对齐。"""
        raise NotImplementedError

    # ---------------------------
    # 兼容接口与便捷方法
    # ---------------------------
    def act(self, observation: Observation, deterministic: bool = False) -> Action:
        """兼容旧接口：基于 Observation 生成 Action。

        默认将 observation 作为 state 直接转发给 get_action()；
        若返回为 dict[str, float]，则包装为 Action；否则抛出异常提示实现方。
        """
        action_obj = self.get_action(observation)
        if isinstance(action_obj, dict):
            # 约定：字典视为 setpoints
            return Action(setpoints={str(k): float(v) for k, v in action_obj.items()})
        raise TypeError("get_action should return a dict[str, float] when used via act().")

    def learn(
        self,
        last_observation: Observation,
        action: Action,
        reward: float,
        next_observation: Observation,
        done: bool,
    ) -> None:
        """兼容旧接口：转发到 update()。"""
        self.update(last_observation, action.setpoints, reward, next_observation, done)

    def reset(self) -> None:
        """可选：在 episode 开始时复位内部状态。"""

    def update_hyperparams(self, new_config: AgentConfig) -> None:
        """在不重建实例的情况下更新超参数。"""
        self._config = new_config
        log.info("agent.update_hparams", cfg=dict(seed=self._config.seed, eps=self._config.exploration_epsilon))


