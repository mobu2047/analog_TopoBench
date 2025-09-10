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
        topo = cfg.get("topology", {})
        self._validator = ActionValidator(cfg.get("action_space", []))
        self._adapter = TopologyAdapter(simulator, cfg.get("action_space", []))
        self._observer = ObservationReader(simulator, cfg.get("observation_space", {}), topo.get("read_mode", "base_workspace"))
        self._metrics = MetricEvaluator(cfg.get("metrics", {}))

    def reset(self, initial_action: Dict[str, Any] | None = None) -> Dict[str, Any]:
        # 按 per_episode 的动作预写（若提供）
        if initial_action:
            self._adapter.apply_action(self._validator.clip_and_validate(initial_action))
        # 仿真复位
        return self._sim.reset()

    def step(self, action: Dict[str, Any], *, whole_duration: bool = True) -> Dict[str, Any]:
        # 恢复校验
        act = self._validator.clip_and_validate(action)
        # 先复位到初态
        self._sim.reset()
        # 写入本次动作
        self._adapter.apply_action(act)
        # 跑完整仿真（默认），或单步
        res = self._sim.run_step(act, whole_duration=whole_duration) if "whole_duration" in self._sim.run_step.__code__.co_varnames else self._sim.run_step(act)
        obs = self._observer.read()
        metrics = self._metrics.evaluate(obs)
        return {"sim": res, "obs": obs, "metrics": metrics}


