"""观测读取器：从 MATLAB 读取信号并执行配置化变换。

支持两种来源：
- base_workspace: 直接读工作区变量
- simulation_output: 从 SimulationOutput 对象提取（后续可扩展）
"""

from __future__ import annotations

from typing import Any, Dict, List
import numpy as np

from inverter_ai_control.utils.logger import get_logger
from inverter_ai_control.utils import signal_processing as sp


log = get_logger(__name__)


class ObservationReader:
    def __init__(self, simulator, observation_cfg: Dict[str, Any], read_mode: str = "base_workspace"):
        self._sim = simulator
        self._cfg = observation_cfg or {}
        self._signals = self._cfg.get("signals", [])
        self._read_mode = read_mode

    def read(self) -> Dict[str, Any]:
        """读取并变换，返回键值观测。"""
        obs: Dict[str, Any] = {}
        for spec in self._signals:
            key = spec.get("key")
            src = spec.get("from", "workspace")
            path = spec.get("path", key)
            transforms = spec.get("transforms", [])

            raw = self._read_signal(src, path)
            val = self._apply_transforms(raw, transforms)
            obs[key] = val
        return obs

    def _read_signal(self, src: str, path: str):
        if src == "workspace":
            try:
                return self._sim._fetch_workspace_var(path)
            except Exception as e:
                log.error("observer.read_workspace_failed", path=path, error=str(e))
                return np.array([])
        elif src == "simulation_output":
            # 预留：从 SimulationOutput 读取（如 self._sim.last_out['V_out']）
            log.warning("observer.simulation_output_not_implemented", path=path)
            return np.array([])
        else:
            log.warning("observer.unknown_source", src=src, path=path)
            return np.array([])

    def _apply_transforms(self, x, transforms: List[str]):
        if not transforms:
            return x
        val = x
        for t in transforms:
            if t == "last":
                val = sp.last(val)
            elif t == "rms":
                val = sp.rms(val)
            elif t == "mean":
                val = sp.mean(val)
            elif t.startswith("thd"):
                # 需要在表达上提供 fs/f0；当前示例中由 metrics 进行计算更合适
                val = val
            else:
                # 未知变换，原样返回
                val = val
        return val


