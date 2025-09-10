### 项目概览

一个分层解耦的 Python⇄Simulink 控制仿真框架，面向后续接入强化学习（RL）、深度学习（DL）与 MCP/LLM Agent。核心原则：仿真层、智能体层、应用层彼此独立，通过清晰的数据接口（纯字典/数组）交互。

### 目录结构

```plaintext
analog TopoBench/
├── main.py                           # 演示脚本：无智能体，交互/预设两种控制模式
├── test_simulator.py                 # 独立测试：验证 MatlabSimulator 读写与绘图
├── config/
│   └── default.yaml                  # 顶层配置（推荐复制为 config.yaml 覆盖本地）
├── inverter_ai_control/
│   ├── sim_env/
│   │   ├── matlab_simulator.py       # 与 MATLAB/Simulink 交互的核心抽象
│   │   ├── fake_session.py           # 无 MATLAB 环境下的假会话（开发/CI）
│   │   └── inverter_gym_env.py       # 预留：Gym 适配层
│   ├── agents/
│   │   ├── base_agent.py             # 智能体 ABC：get_action/update/save/load
│   │   └── heuristic_agent.py        # 示例智能体（最小可运行）
│   └── utils/
│       ├── logger.py                 # 结构化日志（structlog；无则降级为标准 logging）
│       └── config_loader.py          # YAML 配置加载
├── simulink/
│   └── generate_inverter_model.m     # 生成示例模型（离散 PI + 一阶对象）
└── requirements.txt                  # Python 依赖
```

### 核心模块

- `MatlabSimulator`（仿真层）
  - 初始化：接收完整配置字典 `config`，内部启动/复用 `matlab.engine`、加载模型、写入初始参数。
  - `reset(initial_setpoints)`: 写入基础/初值，编译更新模型，不读取输出（首帧在第一次 `run_step` 后获得）。
  - `run_step(action)`: 将 `action`（如 `{Kp: x, Ki: y}`）写入 base 工作区 → 设置 `StopTime = t + Ts` → `sim()` → 从 base 工作区读取输出字典，返回如 `{time, V_out, V_ref, error}`。
  - 关键约束：
    - 框架会在加载模型时统一设置 `ReturnWorkspaceOutputs='off'`，要求 To Workspace 直接写回独立变量。
    - To Workspace 的 `VariableName` 须与 `config.matlab.output_map` 对应（推荐 `Array` 保存格式）。

- `BaseAgent`（智能体层）
  - 抽象方法：`get_action(state)`、`update(state, action, reward, next_state, done)`、`save(path)`、`load(path)`。
  - 向后兼容：保留了 `act/learn` 包装，便于逐步迁移。

- `main.py`（应用层）
  - 仅做装配与编排：加载 YAML → 构造 `MatlabSimulator(config)` → 交互或预设序列循环 `run_step(action)`。
  - 两种模式：
    - 交互：`--interactive`，命令行输入 `Kp Ki`；
    - 预设：`--steps/--kp-start/--kp-end/--ki` 等参数线性扫描。

### 环境准备

1) Python 依赖
```bash
pip install -r requirements.txt
```

2) MATLAB Engine for Python（必须）
- 使用 MATLAB 安装器为当前 Python 环境安装 `matlabengineforpython`；通常位于 `MATLABROOT\extern\engines\python`。

### 生成示例模型（可选）

若无现成模型，先在 MATLAB 里生成示例：
```matlab
addpath('C:/Users/tiany/PycharmProjects/analog TopoBench/simulink');
modelPath = generate_inverter_model();
```
这会创建 `C:/Users/tiany/PycharmProjects/analog TopoBench/simulink/inverter_model.slx`，模型包含：`V_ref → Sum → PI(Kp_val,Ki_val) → DiscretePlant → V_out`，并以 `To Workspace` 输出 `V_out/V_ref/error`。

### 配置文件（config/default.yaml）

```yaml
matlab:
  model_path: 'C:/Users/tiany/PycharmProjects/analog TopoBench/simulink/inverter_model.slx'
  simulation_step: 0.001
  stop_time: 0.02
  start_in_accelerator: true
  use_external_mode: false
  extra_params: { Ts: 0.001 }           # 可添加 Tau/V_ref_src 等；框架内部有默认
  input_map:  { Kp: 'Kp_val', Ki: 'Ki_val' }
  output_map: { V_out: 'V_out', V_ref: 'V_ref', error: 'error' }

action_bounds:
  Kp: { min: 0.0, max: 10.0 }
  Ki: { min: 0.0, max: 100.0 }
```

建议复制为 `config/config.yaml` 作为本地覆盖文件；`main.py` 会优先读取 `config/config.yaml`。

### 快速开始

1) 确认 `model_path` 为绝对路径（Windows 建议使用正斜杠 `/`）

2) 运行演示脚本
```bash
# 交互模式：手动输入 Kp Ki
python main.py --interactive

# 预设模式：线性扫描 Kp，Ki 固定
python main.py --steps 30 --kp-start 1.0 --kp-end 1.3 --ki 0.1
```

3) 运行独立测试
```bash
python test_simulator.py
```

### 使用要点与约束

- To Workspace 与输出读取
  - 框架在加载模型时执行 `set_param(model,'ReturnWorkspaceOutputs','off')`，因此 To Workspace 必须写出独立变量（非 `out.V_out`）。
  - `reset()` 不读取输出；第一次 `run_step()` 后方可在 base 工作区看到 `V_out/V_ref/error`。

- 关于 Kp/Ki 随时间变化
  - 本框架将 Kp/Ki 视为“动作”，每步 `run_step()` 之前写入 base，形成分段常值调度；适合在线调参/智能体控制。
  - 若需一次仿真中按时间序列连续变化，建议以 `From Workspace` 提供 `Kp(t)/Ki(t)` 或启用 Fast Restart/外部模式。

- 日志
  - 默认使用 `structlog` 输出结构化 JSON；若未安装，会降级为标准 logging，但仍支持 `log.info("msg", key=value)` 调用样式。

### 常见问题（FAQ）

- 报错 “Model path does not exist”
  - `config.matlab.model_path` 非绝对路径或文件不存在。请使用绝对路径，Windows 使用 `/`。

- 读取 `V_out` 失败或不存在
  - 确认模型设置：`get_param(model,'ReturnWorkspaceOutputs')` 返回 `off`；To Workspace 变量名与 `output_map` 一致，保存格式为 `Array`。
  - 记住：仅在 `run_step()` 后才能在 base workspace 看到输出变量。

- MATLAB 弹出 Scope 窗口
  - 框架尝试将所有 Scope 的 `OpenAtSimulationStart` 设为 `off`；如需可视化，请在模型中单独开启，或仿真结束后手动查看。

### 扩展：接入任意智能体

实现 `BaseAgent`：
```python
from inverter_ai_control.agents.base_agent import BaseAgent

class MyAgent(BaseAgent):
    def get_action(self, state):
        # state 可为 dict/ndarray，自由设计
        return {"Kp": 1.0, "Ki": 0.1}

    def update(self, state, action, reward, next_state, done):
        pass

    def save(self, path: str):
        pass

    def load(self, path: str):
        pass
```
应用层仅需把 `run_step()` 的返回组织为智能体所需 `state`，并调用 `agent.get_action(state)` 产生下一步动作即可。

### 结果存档（可选示例）

```python
# 在 main.py 的循环里收集结果
all_t, all_vout, all_vref, all_err = [], [], [], []
res = simulator.run_step(action)
all_t.append(res["time"])  # 记录时间
# 建议只取每步数组的最后一个采样点作为“当前步”的观测

# 结束后保存
import numpy as np, os, time
run_dir = os.path.join("runs", time.strftime("%Y%m%d-%H%M%S"))
os.makedirs(run_dir, exist_ok=True)
np.savez(os.path.join(run_dir, "result.npz"),
         time=np.array(all_t), V_out=np.array(all_vout), V_ref=np.array(all_vref), error=np.array(all_err))
```

### 许可证

内部项目示例，按需补充。


