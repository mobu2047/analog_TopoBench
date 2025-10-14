"""动作裁剪与校验模块。

功能：
- 根据配置的 bounds/dtype/shape/自定义规则，对动作进行标准化与裁剪
"""

from __future__ import annotations

from typing import Any, Dict, List
import numpy as np


class ActionValidator:
    def __init__(self, action_space_cfg: List[Dict[str, Any]]):
        """基于 action_space 定义进行动作的裁剪与类型校验。

        兼容两种声明方式：
        - 旧式：{ name, target, path, dtype, bounds, ... }
        - 新式：{ key: "block.param", dtype, bounds, ... } 或 { block, param, ... }
        """
        self._space: List[Dict[str, Any]] = []
        self._name_to_spec: Dict[str, Dict[str, Any]] = {}
        for item in (action_space_cfg or []):
            spec = dict(item)
            # 归一化 name/key：允许仅给 key 或 block+param
            key = spec.get("key")
            if not key and spec.get("block") and spec.get("param"):
                key = f"{spec['block']}.{spec['param']}"
                spec["key"] = key
            name = spec.get("name") or key or spec.get("path")
            if not name:
                # 无法确定唯一键，跳过
                continue
            spec["name"] = name
            self._space.append(spec)
            # 同时支持用 name 和 key 查询
            self._name_to_spec[name] = spec
            if key:
                self._name_to_spec[key] = spec

    def clip_and_validate(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """按配置裁剪/校验并返回新动作字典。"""
        normalized: Dict[str, Any] = {}
        for name, value in action.items():
            # 允许传入 name 或 key（block.param）
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


