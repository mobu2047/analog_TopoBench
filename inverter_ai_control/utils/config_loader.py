"""配置加载工具：集中读取与校验 YAML 配置。

为何需要：
- 避免在业务类中硬编码参数与路径，提升解耦与可测试性
- 通过统一加载入口，便于后续加入校验与默认值处理
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

import yaml

from inverter_ai_control.utils.logger import get_logger


log = get_logger(__name__)


def _read_yaml(p: Path) -> Dict[str, Any]:
    with p.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_config(path: str | Path) -> Dict[str, Any]:
    """从 YAML 文件加载配置为字典，并兼容 v2 结构。

    v2 关键点：
    - model.path / sim.step / sim.stop_time / sim.mode.*
    - matlab.output_map 作为唯一输出映射
    - parameters.manual（工作区变量）+ parameters.auto.file（外置自动参数）
    - presets.init_action（替代旧版 presets.episode/steps）
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"Config file not found: {p}")

    cfg = _read_yaml(p)

    # 合并 auto 参数（若定义了外置文件）
    params = cfg.get("parameters", {}) or {}
    auto = params.get("auto", {}) or {}
    auto_file = auto.get("file")
    if auto_file:
        auto_path = Path(auto_file)
        if not auto_path.is_absolute():
            auto_path = p.parent / auto_path
        if auto_path.exists():
            auto_data = _read_yaml(auto_path)
            params["auto_merged"] = auto_data
            cfg["parameters"] = params
            log.info("config.auto_params.merged", file=str(auto_path))
        else:
            log.warning("config.auto_params.missing", file=str(auto_path))

    log.info("config.loaded", path=str(p), version=str(cfg.get("version", "unknown")))
    return cfg


