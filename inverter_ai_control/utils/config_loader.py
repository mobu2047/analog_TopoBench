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


def load_config(path: str | Path) -> Dict[str, Any]:
    """从 YAML 文件加载配置为字典。

    设计：
    - 返回纯 dict，避免与具体库（omegaconf 等）耦合
    - 在此处可加入默认值注入/结构校验
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"Config file not found: {p}")

    with p.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    log.info("config.loaded", path=str(p))
    return cfg


