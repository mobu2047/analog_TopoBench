"""Fake MATLAB session for local development and CI without MATLAB.

用途：
- 在无 MATLAB 环境下验证 `MatlabSimulator` 的数据/控制流与日志
- 模拟最小的 set_var/get_var/step/read_signals 行为

设计说明：
- 不追求电力电子的物理真实，仅提供可重复、可观测的信号
- 信号示例：
  - `logs.va`：50Hz 正弦电压，幅值约 311V（220Vrms*sqrt(2)）
  - `logs.ia`：围绕 d 轴参考的微小扰动
"""

from __future__ import annotations

import math
from typing import Any, Dict, Tuple

import numpy as np

from inverter_ai_control.utils.logger import get_logger


log = get_logger(__name__)


class FakeMatlabSession:
    """最小可用的 MATLAB 会话假实现，满足 MatlabSession 协议。"""

    def __init__(self) -> None:
        # 工作区变量存储（含设参与控制量）
        self._vars: Dict[str, Any] = {}
        # 当前仿真时间（秒）
        self._time_s: float = 0.0
        # 已打开的模型标识（可选）
        self._model_path: str | None = None

    # 会话/模型 -------------------------------------------------------------
    def start(self) -> None:
        log.info("fake_session.start")

    def stop(self) -> None:
        log.info("fake_session.stop")

    def open_model(self, model_path: str) -> None:
        self._model_path = model_path
        log.info("fake_session.open_model", model=model_path)

    def close_model(self, model_path: str) -> None:
        log.info("fake_session.close_model", model=model_path)
        self._model_path = None

    # 变量/参数 -------------------------------------------------------------
    def set_param(self, block: str, param: str, value: Any) -> None:
        # 占位：仅记录日志
        log.info("fake_session.set_param", block=block, param=param, value=value)

    def set_var(self, name: str, value: Any) -> None:
        self._vars[name] = value
        log.info("fake_session.set_var", name=name, value=value)

    def get_var(self, name: str) -> Any:
        return self._vars.get(name)

    # 推进/读取 -------------------------------------------------------------
    def step(self, step_size_s: float) -> float:
        # 简单推进时间
        self._time_s += float(step_size_s)
        return self._time_s

    def read_signals(self, names: Tuple[str, ...]) -> Dict[str, np.ndarray]:
        """返回与给定 MATLAB 名称对应的 numpy 数组（单点）。"""
        out: Dict[str, np.ndarray] = {}

        # 参考值（若未设置则使用默认）
        d_ref = float(self._vars.get("base.d_ref", 1.0))
        q_ref = float(self._vars.get("base.q_ref", 0.0))

        for n in names:
            if n.endswith(".va"):
                # 50Hz 电压，幅值约 311V
                val = 311.0 * math.sin(2.0 * math.pi * 50.0 * self._time_s)
                out[n] = np.array([val], dtype=float)
            elif n.endswith(".ia"):
                # 电流围绕 d_ref 的微小扰动
                val = d_ref + 0.05 * math.sin(2.0 * math.pi * 50.0 * self._time_s)
                out[n] = np.array([val], dtype=float)
            elif n in self._vars:
                # 直接返回工作区变量（包装为数组）
                out[n] = np.array([float(self._vars[n])], dtype=float)
            else:
                out[n] = np.array([0.0], dtype=float)

        log.info("fake_session.read_signals", t=self._time_s, names=list(names))
        return out


