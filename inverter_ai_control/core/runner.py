"""统一步进管线：Validator -> Adapter -> Simulator -> Observation -> Metrics。

用法：
- 在 main.py 中构造 Runner，并在循环中调用 step(action)
"""

from __future__ import annotations

from typing import Any, Dict

from inverter_ai_control.core.validator import ActionValidator
from inverter_ai_control.core.topology_adapter import TopologyAdapter
from inverter_ai_control.core.observation_reader import ObservationReader
from inverter_ai_control.core.metrics import MetricEvaluator


class Runner:
    def __init__(self, simulator, cfg: Dict[str, Any]):
        self._sim = simulator
        self._cfg = cfg
        self._validator = ActionValidator(cfg.get("action_space", []))
        self._adapter = TopologyAdapter(simulator, cfg.get("action_space", []))
        # v2 不再使用 topology.read_mode；默认从 workspace 读取
        self._metrics = MetricEvaluator(cfg.get("metrics", {}))

    def reset(self, initial_action: Dict[str, Any] | None = None) -> Dict[str, Any]:
        # v2: 支持从 presets.init_action 获取默认初始动作；避免重复写入，统一交由 simulator.reset 处理覆盖
        if initial_action is None:
            presets = self._cfg.get("presets", {}) or {}
            initial_action = presets.get("init_action")
        validated = self._validator.clip_and_validate(initial_action) if initial_action else None
        return self._sim.reset(initial_setpoints=validated)

    def step(self, action: Dict[str, Any], *, whole_duration: bool = True) -> Dict[str, Any]:
        # 恢复校验
        act = self._validator.clip_and_validate(action)
        # 先复位到初态
        self._sim.reset()
        # 写入本次动作
        self._adapter.apply_action(act)
        # 跑完整仿真（默认），或单步
        res = self._sim.run_step(act, whole_duration=whole_duration) if "whole_duration" in self._sim.run_step.__code__.co_varnames else self._sim.run_step(act)
        metrics = self._metrics
        return {"sim": res, "metrics": metrics}


