"""统一结构化日志封装。

为什么：
- Python 无法直接使用 Winston（Node.js 生态）；为达到类似能力，采用 logging + structlog
- 统一由本模块提供 get_logger，避免到处配置日志，减少耦合

如何使用：
- 在任何模块：from inverter_ai_control.utils.logger import get_logger; log = get_logger(__name__)
- 记录：log.info("message", key=value)
"""

from __future__ import annotations

import logging
import json
from typing import Optional

try:
    import structlog
except Exception:  # 单元测试或最小可运行场景下 fallback 到标准 logging
    structlog = None  # type: ignore


_CONFIGURED = False


def _configure_once() -> None:
    """全局一次性配置结构化日志。

    设计考量：
    - 延迟配置：避免模块导入时产生副作用
    - 幂等：重复调用不改变状态
    """
    global _CONFIGURED
    if _CONFIGURED:
        return

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s - %(message)s",
    )

    if structlog is not None:
        structlog.configure(
            processors=[
                structlog.processors.add_log_level,
                structlog.processors.TimeStamper(fmt="iso"),
                structlog.processors.StackInfoRenderer(),
                structlog.processors.format_exc_info,
                structlog.processors.JSONRenderer(),
            ],
            wrapper_class=structlog.make_filtering_bound_logger(logging.INFO),
            context_class=dict,
            cache_logger_on_first_use=True,
        )

    _CONFIGURED = True


class _StdLoggerProxy:
    """在未安装 structlog 时提供的轻量代理，支持 log.info("msg", key=value)。

    说明：标准 logging 不接受任意关键字参数；本代理将 kwargs 序列化为 JSON
    附加到消息末尾，以保持与 structlog 相近的调用体验。
    """

    def __init__(self, logger: logging.Logger) -> None:
        self._logger = logger

    def _format(self, msg: object, kwargs: dict) -> str:
        base = str(msg)
        if kwargs:
            try:
                payload = json.dumps(kwargs, ensure_ascii=False, default=str)
            except Exception:
                payload = str(kwargs)
            return f"{base} | {payload}"
        return base

    def debug(self, msg: object, *args, **kwargs) -> None:
        self._logger.debug(self._format(msg, kwargs), *args)

    def info(self, msg: object, *args, **kwargs) -> None:
        self._logger.info(self._format(msg, kwargs), *args)

    def warning(self, msg: object, *args, **kwargs) -> None:
        self._logger.warning(self._format(msg, kwargs), *args)

    def error(self, msg: object, *args, **kwargs) -> None:
        self._logger.error(self._format(msg, kwargs), *args)

    def exception(self, msg: object, *args, **kwargs) -> None:
        self._logger.exception(self._format(msg, kwargs), *args)

    def critical(self, msg: object, *args, **kwargs) -> None:
        self._logger.critical(self._format(msg, kwargs), *args)


def get_logger(name: Optional[str] = None):
    """获取统一 Logger 接口。

    - 若安装了 structlog：返回结构化 JSON Logger
    - 否则：降级为标准 logging.Logger
    """
    _configure_once()
    if structlog is not None:
        return structlog.get_logger(name or "app")
    # 未安装 structlog 时，返回代理以兼容 key=value 形式
    return _StdLoggerProxy(logging.getLogger(name or "app"))


