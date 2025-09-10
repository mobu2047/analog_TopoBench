"""动作裁剪与校验模块。

功能：
- 根据配置的 bounds/dtype/shape/自定义规则，对动作进行标准化与裁剪
- 区分 per_step/per_episode 的更新频率（由上层调用时控制）
"""

from __future__ import annotations

from typing import Any, Dict, List
import numpy as np


class ActionValidator:
    def __init__(self, action_space_cfg: List[Dict[str, Any]]):
        self._space = action_space_cfg
        # 建立索引方便快速校验
        self._name_to_spec = {item["name"]: item for item in self._space}

    def clip_and_validate(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """按配置裁剪/校验并返回新动作字典。"""
        normalized: Dict[str, Any] = {}
        for name, value in action.items():
            spec = self._name_to_spec.get(name)
            if spec is None:
                # 未声明的动作键，直接忽略
                continue
            dtype = spec.get("dtype", "float")
            bounds = spec.get("bounds", {})
            if dtype == "float":
                v = float(value)
                vmin = float(bounds.get("min", v))
                vmax = float(bounds.get("max", v))
                v = float(np.clip(v, vmin, vmax))
                normalized[name] = v
            elif dtype == "array":
                arr = np.asarray(value, dtype=float)
                # 可选 shape 校验
                shape = spec.get("shape")
                if shape is not None and list(arr.shape) != shape and arr.size == int(np.prod(shape)):
                    arr = arr.reshape(shape)
                # 元素级裁剪
                emin = bounds.get("element_min")
                emax = bounds.get("element_max")
                if emin is not None:
                    arr = np.maximum(arr, float(emin))
                if emax is not None:
                    arr = np.minimum(arr, float(emax))
                normalized[name] = arr
            else:
                # 其他类型先原样返回
                normalized[name] = value
        return normalized


