"""OllamaAgent：通过本地 Ollama 大模型按提示生成下一步动作（占位实现）。

设计动机：
- 让大模型基于历史评估与动作空间边界，直接产出 JSON 格式的动作键值对
- 避免依赖第三方 HTTP 客户端库，使用标准库实现请求/超时/解析

使用约定：
- 初始化需提供 host 与 model（均为字符串）；若任一缺失，建议上层回退到启发式/随机
- get_action(state) 期望 state 提供：
  - iteration: int          # 当前迭代索引
  - last_objective: float?  # 上一次目标值（可空）
  - best_objective: float?  # 目前最优目标值（可空）
  - numeric_space: List[Tuple[str, float, float]]  # (key, min, max)
  - last_action: Dict[str, float]?                 # 上一次动作（可空）

稳健性：
- 服务失败或解析失败时抛出异常，让上层回退到随机/其他策略
"""

from __future__ import annotations

import json
import socket
from typing import Any, Dict, List, Tuple
from urllib import request

from inverter_ai_control.agents.base_agent import BaseAgent, AgentConfig
from inverter_ai_control.utils.logger import get_logger


log = get_logger(__name__)


class OllamaAgent(BaseAgent):
    """基于 Ollama 本地服务的动作生成智能体（占位实现）。

    说明：
    - 仅实现最小可用功能：构造 prompt -> POST /api/generate -> 尝试解析 JSON 动作
    - 不做复杂对话记忆；需要时可切换到 /api/chat
    - 若返回的 JSON 不合法或越界，由上层进行二次裁剪与回退
    """

    def __init__(
        self,
        *,
        host: str,
        model: str,
        action_space: List[Tuple[str, float, float]],
        prompt_template: str | None = None,
        timeout_s: int = 30,
        config: AgentConfig | None = None,
    ) -> None:
        super().__init__(config=config)
        self._host = str(host or "").strip()
        self._model = str(model or "").strip()
        self._space = list(action_space or [])
        self._timeout_s = int(timeout_s)
        self._prompt_template = (
            prompt_template
            or (
                "You are optimizing simulation parameters.\n"
                "Given the numeric action space with ranges, propose the next action strictly as a JSON object mapping parameter names to numbers within their ranges.\n"
                "Only output the JSON, with no explanation.\n"
                "Action space: {space}.\n"
                "Iteration: {iteration}. Last objective: {last_objective}. Best objective: {best_objective}.\n"
                "Last action: {last_action}.\n"
            )
        )

        if not self._host or not self._model:
            log.warning("ollama.init.incomplete_config", host=self._host, model=self._model)

    # ---------------------------
    # 核心接口
    # ---------------------------
    def get_action(self, state: Any) -> Dict[str, float]:
        """向 Ollama 请求一次动作建议，并解析为 dict[str, float]。

        异常策略：
        - 网络/服务错误、解析失败时抛出异常，由上层回退备用策略
        """
        if not self._host or not self._model:
            raise RuntimeError("OllamaAgent: host/model 未配置")

        payload = self._build_payload(state)
        resp_text = self._post_json("/api/generate", payload, timeout_s=self._timeout_s)
        # Ollama /api/generate 返回 JSON，其中 'response' 字段是字符串
        try:
            resp_obj = json.loads(resp_text)
            content = str(resp_obj.get("response", "")).strip()
        except Exception as e:
            log.error("ollama.parse_top_json_failed", error=str(e))
            raise

        # 期望 content 是 JSON 文本；做宽松解析：提取首尾大括号
        try:
            start = content.find("{")
            end = content.rfind("}")
            if start == -1 or end == -1 or end <= start:
                raise ValueError("no_json_braces")
            obj = json.loads(content[start : end + 1])
            # 仅保留动作空间中出现的键
            valid_keys = {k for k, _, _ in self._space}
            action: Dict[str, float] = {}
            for k, v in dict(obj).items():
                if k in valid_keys:
                    try:
                        action[k] = float(v)
                    except Exception:
                        continue
            if not action:
                raise ValueError("empty_action")
            return action
        except Exception as e:
            log.error("ollama.parse_response_failed", error=str(e), preview=content[:200])
            raise

    def update(self, state, action, reward, next_state, done):  # noqa: D401 - 保留接口
        # 占位：当前不做在线学习
        return None

    def save(self, path: str) -> None:
        # 占位：后续若需要持久化 prompt/超参可实现
        return None

    def load(self, path: str) -> None:
        # 占位：后续若需要从磁盘恢复可实现
        return None

    # ---------------------------
    # 内部工具
    # ---------------------------
    def _build_payload(self, state: Any) -> Dict[str, Any]:
        """构造 /api/generate 请求体。"""
        iteration = getattr(state, "get", lambda k, d=None: d)("iteration", None) if hasattr(state, "get") else None
        last_obj = getattr(state, "get", lambda k, d=None: d)("last_objective", None) if hasattr(state, "get") else None
        best_obj = getattr(state, "get", lambda k, d=None: d)("best_objective", None) if hasattr(state, "get") else None
        last_action = getattr(state, "get", lambda k, d=None: d)("last_action", None) if hasattr(state, "get") else None
        numeric_space = getattr(state, "get", lambda k, d=None: d)("numeric_space", self._space) if hasattr(state, "get") else self._space

        # 将空间格式化为紧凑文本，便于模型约束输出范围
        space_txt = ", ".join(f"{k}:[{lo},{hi}]" for k, lo, hi in numeric_space)
        prompt = self._prompt_template.format(
            space=space_txt,
            iteration=iteration,
            last_objective=last_obj,
            best_objective=best_obj,
            last_action=json.dumps(last_action) if last_action is not None else None,
        )

        body = {
            "model": self._model,
            "prompt": prompt,
            "stream": False,
        }
        return body

    def _post_json(self, path: str, data: Dict[str, Any], *, timeout_s: int) -> str:
        """以 JSON 发送 POST 请求，返回响应文本。

        - 使用标准库，避免新增依赖
        - socket 超时用作兜底
        """
        url = self._host.rstrip("/") + path
        req = request.Request(url=url, method="POST")
        req.add_header("Content-Type", "application/json")
        payload = json.dumps(data).encode("utf-8")
        try:
            # 设置全局超时兜底
            old_to = socket.getdefaulttimeout()
            socket.setdefaulttimeout(timeout_s)
            with request.urlopen(req, data=payload, timeout=timeout_s) as resp:
                return resp.read().decode("utf-8", errors="ignore")
        finally:
            try:
                socket.setdefaulttimeout(old_to)
            except Exception:
                pass


