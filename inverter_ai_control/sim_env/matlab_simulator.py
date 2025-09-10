"""最小可用的 MATLAB 仿真抽象，强调解耦。

本文件提供：
- 纯数据 dataclass: SimConfig, Observation, Action
- MatlabSession 协议：隐藏具体 matlab.engine 依赖，便于 Mock/替换
- MatlabSimulator：只做设参/下发控制/读取信号/推进时间，不含 RL 语义

如何扩展：
- 真实会话：实现一个 MatlabEngineSession（独立文件/模块），在此处不直接依赖
- 单元测试：实现 FakeMatlabSession，返回可控的 numpy 数据
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Protocol, Tuple

import os
import logging
import numpy as np

from inverter_ai_control.utils.logger import get_logger


log = get_logger(__name__)


class MatlabSimulatorError(Exception):
    """通用仿真异常基类。用于汇总所有与仿真交互相关的错误。"""

    pass


class MatlabConnectionError(MatlabSimulatorError):
    """引擎启动、模型加载或连接过程中的错误。"""

    pass


class MatlabSimulationError(MatlabSimulatorError):
    """单步仿真(run_step)过程中发生的错误。"""

    pass


class MatlabResetError(MatlabSimulatorError):
    """模型复位(reset)过程中发生的错误。"""

    pass


class MatlabCloseError(MatlabSimulatorError):
    """引擎关闭(close)过程中发生的错误。"""

    pass


@dataclass(frozen=True)
class SimConfig:
    """仿真配置（纯数据）。"""

    model_path: str
    sample_time_s: float
    stop_time_s: float
    start_in_accelerator: bool = True
    extra_params: Mapping[str, Any] = None


@dataclass(frozen=True)
class Observation:
    """仿真输出（观测），仅包含 numpy 或基础类型。"""

    signals: Dict[str, np.ndarray]
    timestamp_s: float


@dataclass(frozen=True)
class Action:
    """仿真输入（动作），体现控制量，无 RL 语义。"""

    setpoints: Dict[str, float]


class MatlabSession(Protocol):
    """MATLAB 会话协议，隔离 matlab.engine 依赖。

    设计动机：
    - 允许用 Mock/Fake 在无 MATLAB 环境下进行开发与 CI
    - 未来可以替换为远程 RPC 服务
    """

    def start(self) -> None: ...
    def stop(self) -> None: ...

    def open_model(self, model_path: str) -> None: ...
    def close_model(self, model_path: str) -> None: ...

    def set_param(self, block: str, param: str, value: Any) -> None: ...
    def set_var(self, name: str, value: Any) -> None: ...
    def get_var(self, name: str) -> Any: ...

    def step(self, step_size_s: float) -> float: ...

    def read_signals(self, names: Tuple[str, ...]) -> Dict[str, np.ndarray]: ...


class SessionMatlabSimulator:
    """与 MATLAB/Simulink 交互的最小抽象。

    - 依赖倒置：通过 MatlabSession 交互
    - 数据边界：跨层仅传递纯数据 dataclass
    - 不包含 RL/Gym 语义，确保与智能体层解耦
    """

    def __init__(self, session: MatlabSession, config: SimConfig, io_map: Dict[str, str]):
        self._session = session
        self._config = config
        self._io_map = dict(io_map)
        self._connected = False
        self._current_time_s: float = 0.0

    def __enter__(self) -> "SessionMatlabSimulator":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    # 资源管理 --------------------------------------------------------------
    def connect(self) -> None:
        if self._connected:
            return
        log.info("sim.connect.start", model=self._config.model_path)
        self._session.start()
        self._session.open_model(self._config.model_path)
        # 运行模式可由具体 Session 内部处理
        self._connected = True
        self._current_time_s = 0.0
        log.info("sim.connect.ok", time_s=self._current_time_s)

    def disconnect(self) -> None:
        if not self._connected:
            return
        log.info("sim.disconnect.start")
        self._session.close_model(self._config.model_path)
        self._session.stop()
        self._connected = False
        log.info("sim.disconnect.ok")

    # 配置/复位 --------------------------------------------------------------
    def configure(self, params: Optional[Mapping[str, Any]] = None) -> None:
        merged = dict(self._config.extra_params or {})
        if params:
            merged.update(params)
        for k, v in merged.items():
            self._session.set_var(k, v)
        log.info("sim.configure", params=list(merged.keys()))

    def reset(self, initial_setpoints: Optional[Dict[str, float]] = None) -> Observation:
        if initial_setpoints:
            self.apply_action(Action(setpoints=initial_setpoints))
        self._current_time_s = 0.0
        signals = self._read_default_signals()
        log.info("sim.reset", t=self._current_time_s)
        return Observation(signals=signals, timestamp_s=self._current_time_s)

    # 主循环 API -------------------------------------------------------------
    def step(self, action: Action, read_signals: Optional[Tuple[str, ...]] = None) -> Observation:
        self.apply_action(action)
        self._current_time_s = self._session.step(self._config.sample_time_s)
        names = read_signals if read_signals is not None else tuple(self._io_map.keys())
        mapped = tuple(self._io_map[n] for n in names if n in self._io_map)
        values = self._session.read_signals(mapped)
        reverse = {v: k for k, v in self._io_map.items()}
        logical = {reverse[m]: arr for m, arr in values.items() if m in reverse}
        log.info("sim.step", t=self._current_time_s, signals=list(logical.keys()))
        return Observation(signals=logical, timestamp_s=self._current_time_s)

    def apply_action(self, action: Action) -> None:
        for logical_name, val in action.setpoints.items():
            if logical_name not in self._io_map:
                log.warning("sim.apply_action.unknown_key", key=logical_name)
                continue
            matlab_var = self._io_map[logical_name]
            self._session.set_var(matlab_var, float(val))
        log.info("sim.apply_action", keys=list(action.setpoints.keys()))

    # 内部工具 ---------------------------------------------------------------
    def _read_default_signals(self) -> Dict[str, np.ndarray]:
        names = tuple(self._io_map.keys())
        mapped = tuple(self._io_map[n] for n in names)
        values = self._session.read_signals(mapped)
        reverse = {v: k for k, v in self._io_map.items()}
        return {reverse[m]: arr for m, arr in values.items() if m in reverse}



class MatlabSimulator:
    """面向智能体无关的 Simulink 仿真接口（完整实现，基于 matlab.engine）。

    设计要点（解耦）：
    - 提供 reset / run_step / close 的最小交互面；输入输出均为纯 Python 字典
    - 不暴露 MATLAB/Simulink 句柄，便于 RL/Heuristic/MCP 等任意智能体复用
    - 通过 input_map/output_map 将逻辑键与 MATLAB 工作区变量名解耦

    日志：
    - 使用项目统一的结构化 logger（log.info/...），同时保持与标准 logging 兼容
    """

    def __init__(
        self,
        config: Mapping[str, Any],
        *,
        engine: Optional[Any] = None,
    ) -> None:
        """初始化并建立与 MATLAB 引擎的连接，载入 Simulink 模型。

        参数:
        - config: 配置字典（或类似映射），预期结构：
            matlab:
              model_path: str
              simulation_step: float
              stop_time: float
              start_in_accelerator: bool
              use_external_mode: bool
              extra_params: dict
              input_map: dict[str, str]
              output_map: dict[str, str]
            action_bounds: dict[str, {min: float, max: float}]  # 可选，供上层使用
        - engine: 可选，外部提供的 MATLAB 引擎实例（不由本类管理生命周期）

        行为：
        - 从 config 读取所需设置；通过 os.path 解析与验证模型路径；启动/复用 MATLAB 引擎；加载模型；配置初始参数

        异常:
        - MatlabConnectionError: 引擎启动失败、模型打开失败等
        """
        # 单行日志初始化提示（使用项目统一 logger，兼容标准 logging）
        logging.getLogger(__name__).debug("MatlabSimulator.__init__ starting")

        # 归一化并校验模型路径
        # 读取配置（含默认值）
        matlab_cfg = dict(config.get("matlab", {}))
        model_path = matlab_cfg.get("model_path", "inverter_model.slx")
        sample_time_s = float(matlab_cfg.get("simulation_step", 1e-3))
        stop_time_s = matlab_cfg.get("stop_time", None)
        start_in_accelerator = bool(matlab_cfg.get("start_in_accelerator", True))
        use_external_mode = bool(matlab_cfg.get("use_external_mode", False))
        extra_params = dict(matlab_cfg.get("extra_params", {}))
        input_map = dict(matlab_cfg.get("input_map", {}))
        output_map = dict(matlab_cfg.get("output_map", {}))

        normalized_path = os.path.abspath(os.path.expanduser(str(model_path)))
        if not os.path.exists(normalized_path):
            raise MatlabConnectionError(f"Model path does not exist: {normalized_path}")

        # 保存配置到实例（保持纯数据）
        self._model_path: str = normalized_path
        self._model_name: str = os.path.splitext(os.path.basename(normalized_path))[0]
        self._sample_time_s: float = float(sample_time_s)
        self._stop_time_s: Optional[float] = float(stop_time_s) if stop_time_s is not None else None
        self._input_map: Dict[str, str] = input_map
        self._output_map: Dict[str, str] = output_map
        self._extra_params: Dict[str, Any] = extra_params
        self._accelerator: bool = start_in_accelerator
        self._external_mode: bool = use_external_mode
        self._connected: bool = False
        self._current_time: float = 0.0

        # MATLAB 引擎：若未注入则在此处启动，并记录生命周期归属
        try:
            if engine is None:
                import matlab.engine  # 延迟导入，避免无引擎环境报错

                log.info("matlab.engine.start")
                self._eng = matlab.engine.start_matlab()
                self._engine_owned = True
            else:
                self._eng = engine
                self._engine_owned = False
        except Exception as e:
            raise MatlabConnectionError(f"Failed to start or acquire MATLAB engine: {e}") from e

        # 加载模型并进行基本配置（保持最小副作用）
        try:
            # 加载模型，不强制打开 UI，保持 headless
            self._eng.load_system(self._model_path, nargout=0)
            # 关键设置：统一关闭 Single simulation output，要求 To Workspace 直接写回 base 变量
            try:
                self._eng.set_param(self._model_name, "ReturnWorkspaceOutputs", "off", nargout=0)
                log.info("matlab.model.setting", key="ReturnWorkspaceOutputs", value="off")
            except Exception:
                # 某些版本/模型属性不可写入，忽略但记录
                log.warning("matlab.model.setting_failed", key="ReturnWorkspaceOutputs")

            # 运行模式可选配置
            if self._accelerator:
                try:
                    self._eng.set_param(self._model_name, "SimulationMode", "accelerator", nargout=0)
                except Exception:
                    # 某些模型/许可证不支持 accelerator，降级为 normal
                    self._eng.set_param(self._model_name, "SimulationMode", "normal", nargout=0)

            # 写入初始参数到工作区
            for var_name, var_value in self._extra_params.items():
                self._assign_in_base(var_name, var_value)

            # 确认连接就绪
            self._connected = True
            log.info("matlab.model.loaded", model=self._model_name, path=self._model_path)
        except Exception as e:
            raise MatlabConnectionError(f"Failed to load/configure model: {e}") from e

    def reset(self, initial_setpoints: Optional[Mapping[str, float]] = None) -> Dict[str, Any]:
        """复位模型到初始状态并返回初始观测（纯字典）。

        行为：
        - 停止当前仿真（若在运行）
        - 将 StartTime 置 0，StopTime 置 0，并执行一次 update 以回到初始条件
        - 注入 initial_setpoints 与 extra_params
        - 读取初始观测输出并返回
        """
        if not self._connected:
            raise MatlabResetError("Engine/model is not connected. Initialize first.")

        try:
            # 停止仿真并准备回到初始条件（注意：在“更新模型”之前必须保证工作区已有 PI 参数）
            self._eng.set_param(self._model_name, "SimulationCommand", "stop", nargout=0)
            self._eng.set_param(self._model_name, "StartTime", "0.0", nargout=0)
            self._eng.set_param(self._model_name, "StopTime", str(self._stop_time_s), nargout=0)

            # 完全解耦：仅按配置与调用者提供的变量写入，不推断/派生任何模型专属变量
            # 由外部 Profile（配置）/Adapter 决定需要哪些变量，如 rep_t/L_load/Kp/Ki 等
            base_vars: Dict[str, Any] = dict(self._extra_params)
            # input_vars: Dict[str, Any] = dict(self._input_map)
            for var_name, var_value in base_vars.items():
                self._assign_in_base(var_name, var_value)
            # for var_name, var_value in input_vars.items():
            #     self._eng.set_param(self._model_name+'/Series RLC Branch', var_name, var_value, nargout=0)
            if initial_setpoints:
                for logical_key, value in dict(initial_setpoints).items():
                    target = self._input_map.get(logical_key, logical_key)
                    # 按类型处理：标量->1x1 double；数组->double 向量；字符串跳过
                    self._assign_in_base(target, value)

            # 再执行一次模型更新，使初始条件与参数生效
            self._eng.set_param(self._model_name, "SimulationCommand", "update", nargout=0)

            # 重置当前时间
            self._current_time = 0.0

            # 方案A：reset 不做任何仿真与读数，只返回最小状态。
            # 说明：To Workspace 变量仅在发生仿真后才会生成；首帧观测由第一次 run_step 提供。
            log.info("matlab.reset.done", t=self._current_time, note="outputs available after first run_step")
            return {"time": float(self._current_time)}
        except Exception as e:
            raise MatlabResetError(f"Failed to reset model: {e}") from e

    def run_step(self, action: Mapping[str, float]) -> Dict[str, Any]:
        """执行一次仿真步并返回观测（纯字典）。

        行为：
        - 将 action 中的键按 input_map 写入 MATLAB 工作区（如 Kp/Ki 等控制参数）
        - 使用 set_param 设置 StopTime = current_time + sample_time_s，执行 eng.sim
        - 从工作区读取 output_map 对应的观测变量，转换为 Python/Numpy 类型
        - 返回结构化结果字典：{"time": x, <logical_key>: <array_or_value>, ...}
        """
        if not self._connected:
            raise MatlabSimulationError("Engine/model is not connected. Initialize first.")

        try:
            # 写入控制量/动作到工作区（按类型处理，避免将数组强制转换为标量）
            for logical_key, value in dict(action).items():
                target = self._input_map.get(logical_key, logical_key)
                self._assign_in_base(target, value)

            # 设置 StopTime 以推进一个控制步长
            next_t = self._current_time + self._stop_time_s
            self._eng.set_param(self._model_name, "StopTime", f"{next_t}", nargout=0)

            # 运行仿真。按要求使用 eng.sim；让模型自身的 To Workspace 块写出结果
            # 若需要捕获 SimulationOutput，可设 nargout=1 并进一步处理
            self._eng.sim(self._model_path, nargout=0)

            # 更新时间并读取输出
            self._current_time = next_t
            result = {"time": float(self._current_time)}
            result.update(self._read_outputs())

            log.info("matlab.step.done", t=self._current_time, keys=list(result.keys()))
            return result
        except Exception as e:
            raise MatlabSimulationError(f"Failed to run simulation step: {e}") from e

    def close(self) -> None:
        """优雅关闭模型/引擎并清理资源。"""
        if not getattr(self, "_eng", None):
            return
        try:
            try:
                # 先尝试停止仿真并卸载模型
                if self._connected:
                    self._eng.set_param(self._model_name, "SimulationCommand", "stop", nargout=0)
                self._eng.close_system(self._model_name, 0, nargout=0)  # 0=不保存
            except Exception:
                # 忽略模型关闭中的非致命错误，继续关闭引擎
                pass

            # 仅当引擎由本类创建时才停止它
            if getattr(self, "_engine_owned", False):
                self._eng.quit()
            self._connected = False
            log.info("matlab.engine.closed")
        except Exception as e:
            raise MatlabCloseError(f"Failed to close MATLAB engine/model: {e}") from e

    def __enter__(self) -> "MatlabSimulator":
        """上下文协议：进入时返回自身。"""
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """上下文协议：退出时调用 close()。"""
        self.close()

    @property
    def model_path(self) -> str:
        """返回模型路径（构造时提供）。"""
        return self._model_path

    @property
    def connected(self) -> bool:
        """返回当前连接/模型加载状态。"""
        return bool(self._connected)

    @property
    def current_time_s(self) -> float:
        """返回当前仿真时间（秒）。"""
        return float(self._current_time)

    # ---------------------------
    # 内部工具方法（封装交互细节）
    # ---------------------------
    def _assign_in_base(self, name: str, value: Any) -> None:
        """将 Python 值写入 MATLAB base 工作区。

        说明：将标量 float/int 转为 matlab.double 以便在 Simulink 中使用。
        """
        try:
            import matlab

            if isinstance(value, (float, int)):
                # 转成 1x1 double，避免维度歧义
                self._eng.workspace[name] = matlab.double([float(value)])
            elif isinstance(value, (list, tuple, np.ndarray)):
                arr = np.asarray(value, dtype=float).reshape(1, -1)
                self._eng.workspace[name] = matlab.double(arr.tolist())
            else:
                # 回退：尝试直接赋值（某些标量/结构体可能允许）
                self._eng.workspace[name] = value
            log.info("matlab.assign", name=name)
        except Exception as e:
            raise MatlabSimulationError(f"Failed to assign variable '{name}' to base workspace: {e}") from e

    def _read_outputs(self) -> Dict[str, Any]:
        """按 output_map 读取工作区变量并转为 Python/Numpy 类型。"""
        results: Dict[str, Any] = {}
        for logical_key, var_name in self._output_map.items():
            results[logical_key] = self._fetch_workspace_var(var_name)
        return results

    def _fetch_workspace_var(self, var_name: str) -> Any:
        """从工作区读取变量并进行尽量自然的类型转换。"""
        try:
            value = self._eng.workspace[var_name]
        except Exception as e:
            # 调试输出：打印当前 base 工作区的变量清单，辅助定位变量名映射问题
            try:
                # evalc 捕获命令窗口输出，比 who/whos 的返回更直观
                workspace_dump = self._eng.evalc("whos", nargout=1)
            except Exception as dump_err:
                workspace_dump = f"<failed to dump workspace via evalc('whos'): {dump_err}>"
            try:
                # 尝试获取变量名列表（有些 MATLAB 版本会返回 char 数组/元胞）
                who_list = self._eng.eval("who", nargout=1)
            except Exception:
                who_list = []
            # 直接打印，确保即使 logger 级别被调整也能看到
            try:
                print("=== MATLAB base workspace (whos) ===\n" + str(workspace_dump))
            except Exception:
                pass
            # 结构化日志记录，便于在文件/控制台查看
            log.error(
                "matlab.workspace.read_failed",
                var=var_name,
                error=str(e),
                who=who_list,
                whos=workspace_dump,
            )
            raise MatlabSimulationError(f"Failed to read workspace variable '{var_name}': {e}") from e

        # 类型转换：matlab.double -> numpy.ndarray；timeseries/dataset 尝试提取数据
        try:
            import matlab

            if isinstance(value, matlab.double):
                arr = np.array(value, dtype=float)
                # 常见情况：1xN 或 Nx1，返回一维向量
                return arr.flatten()

            # 处理 timeseries: 访问 Data/Time 字段（若存在）
            # 注意：在 Python 侧访问面向对象的 MATLAB 类型较麻烦，优先通过 To Workspace 保存为 double
            # 这里保守返回原值，交由上层处理或在模型中统一保存为 double
            return value
        except Exception:
            # 若无法识别类型，直接返回原值（可能是标量/字符串/结构体）
            return value

