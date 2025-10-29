"""拓扑适配器：把配置化的动作写入 MATLAB（工作区或块参数）。

说明：
- 新配置支持两种映射方式（二选一）
  1) 显式路径：target=block_param + path="<model>/<block>:<param>"（兼容旧格式）
  2) 极简键：key = "<block>.<param>"（推荐，与 auto_params 对齐）；运行时自动补全模型名前缀
"""

from __future__ import annotations

from typing import Any, Dict, List, Tuple
import numpy as np

from inverter_ai_control.utils.logger import get_logger


log = get_logger(__name__)


class TopologyAdapter:
    def __init__(self, simulator, action_space_cfg: List[Dict[str, Any]]):
        self._sim = simulator
        # 归一化 action_space：支持 {name,target,path} 或 {key} 两种写法
        self._space: List[Dict[str, Any]] = []
        self._name_to_spec: Dict[str, Dict[str, Any]] = {}
        for item in (action_space_cfg or []):
            spec = dict(item)
            # 允许省略 name：默认 name=key；允许省略 key：若有 block/param 则 key=block.param
            key = spec.get("key")
            if not key and spec.get("block") and spec.get("param"):
                key = f"{spec['block']}.{spec['param']}"
                spec["key"] = key
            name = spec.get("name") or key
            if not name:
                # 兼容旧格式：用 path 作为 name
                name = spec.get("path", "")
            spec["name"] = name
            self._space.append(spec)
            self._name_to_spec[name] = spec

    def apply_action(self, action: Dict[str, Any]) -> None:
        """根据 action_space 配置将动作写入 MATLAB。"""
        for name, value in action.items():
            spec = self._name_to_spec.get(name)
            if spec is None:
                # 未声明键：尝试智能解析 name
                # - 若形如 "block.param" → 作为块参数写入（自动补全模型名前缀）
                # - 若形如 "block_path:param" → 旧式块参数路径
                # - 否则 → 视为工作区变量名
                try:
                    if "." in name and ":" not in name:
                        block, param = name.split(".", 1)
                        model_name = getattr(self._sim, "_model_name", "")
                        block_full = f"{model_name}/{block}" if model_name and not block.startswith(model_name + "/") else str(block)
                        self._set_block_param(block_full, param, value)
                    elif ":" in name:
                        blk, prm = self._split_block_param(name)
                        model_name = getattr(self._sim, "_model_name", "")
                        blk_full = f"{model_name}/{blk}" if model_name and not blk.startswith(model_name + "/") else blk
                        self._set_block_param(blk_full, prm, value)
                    else:
                        self._assign_workspace(name, value)
                except Exception as e:
                    log.error("adapter.apply_action.unknown_key_failed", name=name, error=str(e))
                continue
            # 解析目标：优先使用极简键 key=block.param
            block_path, param_name, target = self._resolve_target(spec)

            if target == "workspace":
                self._assign_workspace(param_name, value)
            elif target == "block_param":
                self._set_block_param(block_path, param_name, value)
            else:
                log.warning("adapter.unknown_target", target=target, name=name)

    def _assign_workspace(self, name: str, value: Any) -> None:
        try:
            self._sim._assign_in_base(name, value)  # 使用已有内部方法
        except Exception as e:
            log.error("adapter.assign_failed", name=name, error=str(e))
            raise

    def _set_block_param(self, block: str, param: str, value: Any) -> None:
        """设置块参数；若直写失败，尝试基于叶子名的宽松匹配自动纠正大小写/空格。"""
        if isinstance(value, (list, tuple, np.ndarray)):
            val_str = str(list(np.asarray(value).flatten()))
        else:
            val_str = str(value)
        try:
            self._sim._eng.set_param(block, param, val_str, nargout=0)
            return
        except Exception as e:
            # 回退：基于最后一级块名做宽松匹配（忽略大小写和空白）
            try:
                leaf = block.split("/")[-1]
                norm = self._normalize_name(leaf)
                candidates = self._sim._eng.find_system(self._sim._model_name, "LookUnderMasks", "all", "FollowLinks", "on", nargout=1)
                fixed = None
                for p in (candidates or []):
                    try:
                        if self._normalize_name(str(p).split("/")[-1]) == norm:
                            fixed = str(p)
                            break
                    except Exception:
                        pass
                if fixed:
                    self._sim._eng.set_param(fixed, param, val_str, nargout=0)
                    log.info("adapter.set_param.resolved_block", orig=block, resolved=fixed, param=param)
                    return
            except Exception:
                pass
            log.error("adapter.set_param_failed", block=block, param=param, error=str(e))
            raise

    @staticmethod
    def _normalize_name(text: str) -> str:
        s = str(text).strip().lower()
        import re as _re
        s = _re.sub(r"\s+", " ", s)
        s = s.replace(" ", "_")
        s = _re.sub(r"[^a-z0-9_]+", "", s)
        return s

    @staticmethod
    def _split_block_param(path: str):
        if ":" not in path:
            raise ValueError("block_param path must be 'block_path:param'")
        block, param = path.split(":", 1)
        return block, param

    # --- 新增：从新老两种规范解析 block/param/target ---
    def _resolve_target(self, spec: Dict[str, Any]) -> Tuple[str, str, str]:
        """返回 (block_full_path, param_name, target)

        规则：
        - 若提供 key=block.param，则 target=block_param；block 自动补全模型名前缀
        - 若提供 block+param，则同上
        - 否则回退到旧格式 target+path（path='block:param' 或工作区变量名）
        """
        model_name = getattr(self._sim, "_model_name", "")
        # 新格式：key 或 block+param
        key = spec.get("key")
        block = spec.get("block")
        param = spec.get("param")
        if key and "." in key:
            block, param = key.split(".", 1)
        if block and param:
            block_full = f"{model_name}/{block}" if model_name and not block.startswith(model_name + "/") else str(block)
            return block_full, str(param), "block_param"

        # 旧格式
        target = spec.get("target", "workspace")
        path = spec.get("path", spec.get("name", ""))
        if target == "block_param":
            blk, prm = self._split_block_param(path)
            # 自动补全模型名前缀
            blk_full = f"{model_name}/{blk}" if model_name and not blk.startswith(model_name + "/") else blk
            return blk_full, prm, "block_param"
        else:
            # workspace：path 作为变量名
            return "", path, "workspace"


