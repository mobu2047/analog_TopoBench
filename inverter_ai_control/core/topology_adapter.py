"""拓扑适配器：把配置化的动作写入 MATLAB（工作区或块参数）。

说明：
- 独立于具体拓扑；通过 YAML 中 action_space 的 target/path/apply 进行映射
"""

from __future__ import annotations

from typing import Any, Dict, List
import numpy as np

from inverter_ai_control.utils.logger import get_logger


log = get_logger(__name__)


class TopologyAdapter:
    def __init__(self, simulator, action_space_cfg: List[Dict[str, Any]]):
        self._sim = simulator
        self._space = action_space_cfg
        self._name_to_spec = {item["name"]: item for item in self._space}

    def apply_action(self, action: Dict[str, Any]) -> None:
        """根据 action_space 配置将动作写入 MATLAB。"""
        for name, value in action.items():
            spec = self._name_to_spec.get(name)
            if spec is None:
                continue
            target = spec.get("target", "workspace")
            path = spec.get("path", name)
            apply = spec.get("apply", "assign")

            if target == "workspace":
                self._assign_workspace(path, value)
            elif target == "block_param":
                # 未来支持 set_param：path 形如 'model/Sub/Block:param'
                block, param = self._split_block_param(path)
                self._set_block_param(block, param, value)
            else:
                log.warning("adapter.unknown_target", target=target, name=name)

    def _assign_workspace(self, name: str, value: Any) -> None:
        try:
            self._sim._assign_in_base(name, value)  # 使用已有内部方法
        except Exception as e:
            log.error("adapter.assign_failed", name=name, error=str(e))
            raise

    def _set_block_param(self, block: str, param: str, value: Any) -> None:
        try:
            if isinstance(value, (list, tuple, np.ndarray)):
                val_str = str(list(np.asarray(value).flatten()))
            else:
                val_str = str(value)
            self._sim._eng.set_param(block, param, val_str, nargout=0)
        except Exception as e:
            log.error("adapter.set_param_failed", block=block, param=param, error=str(e))
            raise

    @staticmethod
    def _split_block_param(path: str):
        if ":" not in path:
            raise ValueError("block_param path must be 'block_path:param'")
        block, param = path.split(":", 1)
        return block, param


