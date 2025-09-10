"""指标评估：基于表达式与聚合的配置化多目标评估。

实现策略：
- 提供一个安全的表达式求值上下文，仅暴露必要函数（abs、max、min、thd等）和观测键
- 支持 aggregator: mean/max/last
- 返回：每个指标的值、约束布尔、加权总目标（若提供权重）
"""

from __future__ import annotations

from typing import Any, Dict, List, Tuple
import numpy as np

from inverter_ai_control.utils import signal_processing as sp


def _safe_context(obs: Dict[str, Any]) -> Dict[str, Any]:
    ctx: Dict[str, Any] = {}
    # 基本函数
    ctx.update(
        abs=abs,
        max=max,
        min=min,
    )
    # 信号处理
    ctx.update(
        rms=sp.rms,
        thd=sp.thd,
        mean=sp.mean,
        last=sp.last,
        np=np,
    )
    # 观测变量注入
    ctx.update(obs)
    return ctx


def _aggregate(values: Any, method: str) -> float:
    if method == "mean":
        return float(sp.mean(values))
    if method == "max":
        arr = np.asarray(values).reshape(-1)
        return float(arr.max()) if arr.size else float("nan")
    # 默认 last
    return float(sp.last(values))


class MetricEvaluator:
    def __init__(self, metrics_cfg: Dict[str, Any]):
        self._cfg = metrics_cfg or {}
        self._defs = self._cfg.get("definitions", [])
        self._constraints = self._cfg.get("constraints", [])

    def evaluate(self, obs: Dict[str, Any]) -> Dict[str, Any]:
        ctx = _safe_context(obs)
        results: Dict[str, Any] = {"metrics": {}, "constraints": {}, "objective": None}
        objective = 0.0
        weight_sum = 0.0

        # 计算各指标
        for spec in self._defs:
            name = spec.get("name")
            expr = spec.get("expr")
            agg = spec.get("aggregator", "last")
            goal = spec.get("goal", "minimize")
            weight = float(spec.get("weight", 1.0))
            try:
                val_raw = eval(expr, {"__builtins__": {}}, ctx)
                val = _aggregate(val_raw, agg)
            except Exception:
                val = float("nan")
            results["metrics"][name] = val
            if np.isfinite(val):
                # 最小化则直接加，最大化取负号
                sign = 1.0 if goal == "minimize" else -1.0
                objective += sign * weight * val
                weight_sum += weight

        # 计算约束
        for c in self._constraints:
            name = c.get("name")
            expr = c.get("expr")
            try:
                ok = bool(eval(expr, {"__builtins__": {}}, ctx))
            except Exception:
                ok = False
            results["constraints"][name] = ok

        results["objective"] = objective if weight_sum > 0 else None
        return results


