# Analog TopoBench

## 项目概览

Analog TopoBench 是一个分层解耦的 Python⇄Simulink 控制仿真框架，专为电力电子拓扑的智能控制而设计。框架后续计划支持强化学习（RL）、深度学习（DL）与 MCP/LLM Agent 的接入，核心原则是仿真层、智能体层、应用层彼此独立，通过清晰的数据接口交互。

### 核心特性

- **分层架构**：仿真层、智能体层、应用层完全解耦
- **MATLAB/Simulink 集成**：无缝对接 MATLAB 仿真环境
- **灵活配置**：YAML 驱动的配置系统，支持复杂拓扑定义
- **智能体支持**：标准化的智能体接口，支持 RL/DL/启发式算法
- **可视化工具**：自动生成仿真结果图表和性能指标
- **扩展性强**：模块化设计，易于添加新的拓扑和控制算法

## 项目架构

### 目录结构

```
analog_TopoBench/
├── main.py                           # 主演示脚本：交互/预设控制模式
├── test_simulator.py                 # 独立测试：验证 MatlabSimulator 功能
├── config/
│   └── default.yaml                  # 默认配置文件（推荐复制为 config.yaml）
├── inverter_ai_control/              # 核心框架包
│   ├── sim_env/                      # 仿真环境层
│   │   ├── matlab_simulator.py       # MATLAB/Simulink 交互核心
│   │   ├── fake_session.py           # 无 MATLAB 环境下的模拟会话
│   │   └── inverter_gym_env.py       # Gym 环境适配器
│   ├── agents/                       # 智能体层
│   │   ├── base_agent.py             # 智能体抽象基类
│   │   └── heuristic_agent.py        # 启发式智能体示例
│   ├── core/                         # 核心处理模块
│   │   ├── runner.py                 # 统一步进管线
│   │   ├── validator.py              # 动作验证器
│   │   ├── topology_adapter.py       # 拓扑适配器
│   │   ├── observation_reader.py     # 观测读取器
│   │   └── metrics.py                # 性能指标评估器
│   └── utils/                        # 工具模块
│       ├── logger.py                 # 结构化日志系统
│       ├── config_loader.py          # YAML 配置加载器
│       ├── signal_processing.py      # 信号处理工具
│       └── visualization.py          # 可视化工具
├── simulink/                         # Simulink 模型文件
│   ├── generate_inverter_model.m     # 模型生成脚本
│   └── untitled.slx                 # 示例逆变器模型
├── runs/                            # 运行结果存储目录
└── requirements.txt                 # Python 依赖
```

### 核心模块说明

#### 1. 仿真层 (sim_env/)
- **MatlabSimulator**: 与 MATLAB/Simulink 交互的核心抽象
  - `reset(initial_setpoints)`: 初始化仿真环境
  - `run_step(action)`: 执行单步仿真并返回结果
  - 支持加速模式和外部模式

#### 2. 智能体层 (agents/)
- **BaseAgent**: 智能体抽象基类
  - `get_action(state)`: 根据状态生成动作
  - `update(state, action, reward, next_state, done)`: 学习更新
  - `save(path)/load(path)`: 模型持久化

#### 3. 核心处理层 (core/)
- **Runner**: 统一步进管线，协调各模块
- **ActionValidator**: 动作验证和范围限制
- **TopologyAdapter**: 拓扑特定的动作适配
- **ObservationReader**: 观测数据读取和预处理
- **MetricEvaluator**: 性能指标计算和评估

## 环境准备

### 1. Python 环境

```bash
# 安装 Python 依赖
pip install -r requirements.txt
```

### 2. MATLAB Engine for Python

```bash
# 在 MATLAB 中安装 Python 引擎
# 通常位于 MATLABROOT\extern\engines\python
cd "C:\Program Files\MATLAB\R2023a\extern\engines\python"
python setup.py install
```

### 3. 生成示例模型（可选）

```matlab
% 在 MATLAB 中运行
addpath('path/to/analog_TopoBench/simulink');
modelPath = generate_inverter_model();
```

## 使用方法

### 快速开始

#### 1. 交互模式
```bash
# 手动输入控制参数
python main.py --interactive
```

#### 2. 预设模式
```bash
# 按配置文件执行预设动作序列
python main.py
```

#### 3. 独立测试
```bash
# 验证仿真器功能
python test_simulator.py
```

### 编程接口

#### 基本使用流程

```python
from inverter_ai_control.utils.config_loader import load_config
from inverter_ai_control.sim_env.matlab_simulator import MatlabSimulator
from inverter_ai_control.core.runner import Runner

# 1. 加载配置
config = load_config("config/default.yaml")

# 2. 创建仿真器
simulator = MatlabSimulator(config=config)

# 3. 创建运行器
runner = Runner(simulator, config)

# 4. 执行仿真
with simulator:
    runner.reset()
    result = runner.step({"L_load": 10})
    print(result["metrics"])
```

#### 自定义智能体

```python
from inverter_ai_control.agents.base_agent import BaseAgent

class MyAgent(BaseAgent):
    def get_action(self, state):
        # 实现控制逻辑
        return {"Kp": 1.0, "Ki": 0.1}
    
    def update(self, state, action, reward, next_state, done):
        # 实现学习逻辑
        pass
    
    def save(self, path):
        # 保存模型
        pass
    
    def load(self, path):
        # 加载模型
        pass
```

## 配置项详细说明

### 1. MATLAB 仿真配置 (matlab)

```yaml
matlab:
  model_path: 'simulink/untitled.slx'    # Simulink 模型路径（相对或绝对）
  simulation_step: 0.001                 # 仿真步长 (s)
  stop_time: 0.1                         # 仿真停止时间 (s)
  start_in_accelerator: true             # 是否启用加速模式
  use_external_mode: false               # 是否使用外部模式
  extra_params:                          # 额外参数
    Ts: 0.001                            # 采样时间
    L_load: 0.001                        # 负载电感
  input_map:                             # 输入变量映射
    L_load: 'L_load'                     # 动作名 -> MATLAB 变量名
  output_map:                            # 输出变量映射
    ScopeData: 'ScopeData'               # 观测名 -> MATLAB 变量名
```

### 2. 拓扑配置 (topology)

```yaml
topology:
  id: single_phase_bridge                # 拓扑标识符
  description: 单相桥式逆变器（示例）      # 拓扑描述
  read_mode: base_workspace              # 数据读取模式
```

### 3. 动作空间配置 (action_space)

```yaml
action_space:
  - name: Kp                             # 动作名称
    target: workspace                    # 目标位置（workspace/model）
    path: Kp_val                         # 目标变量路径
    dtype: float                         # 数据类型
    bounds: { min: 0.0, max: 10.0 }     # 取值范围
    apply: assign                        # 应用方式（assign/update）
    step_rate: per_step                  # 更新频率（per_step/per_episode）
  - name: L_load
    target: workspace
    path: L_load
    dtype: float
    default: 0.001                       # 默认值
    bounds: { min: 0.0001, max: 0.01 }
    apply: assign
    step_rate: per_episode
```

### 4. 观测空间配置 (observation_space)

```yaml
observation_space:
  signals:
    - key: ScopeData                     # 观测键名
      from: workspace                    # 数据源
      path: ScopeData                    # 变量路径
      transforms: [ last ]               # 数据变换（last/mean/max等）
```

### 5. 性能指标配置 (metrics)

```yaml
metrics:
  evaluation_window: by_step             # 评估窗口（by_step/by_episode）
  definitions:
    - name: voltage_tracking             # 指标名称
      expr: abs(V_out - V_ref)          # 计算表达式
      aggregator: mean                   # 聚合方式
      goal: minimize                     # 优化目标
      weight: 0.5                        # 权重
  constraints:
    - name: thd_limit                    # 约束名称
      expr: thd(V_out, fs=10000, f0=50) <= 0.05  # 约束表达式
```

### 6. 预设动作配置 (presets)

```yaml
presets:
  episode:                              # 每回合初始化动作
    L_load: 0.001
  steps:                                # 逐步动作序列
    - { L_load: 10 }
    - { L_load: 20 }
```

## 扩展方向

### 1. 添加新的拓扑类型

1. 在 `config/default.yaml` 中定义新的拓扑配置
2. 在 `simulink/` 目录下创建对应的 Simulink 模型
3. 更新 `input_map` 和 `output_map` 配置

### 2. 实现新的智能体算法

```python
# 继承 BaseAgent 并实现必要方法
class RLAgent(BaseAgent):
    def __init__(self, config):
        super().__init__(config)
        # 初始化 RL 算法（如 PPO、SAC 等）
    
    def get_action(self, state):
        # 实现 RL 动作选择
        pass
    
    def update(self, state, action, reward, next_state, done):
        # 实现 RL 学习更新
        pass
```

### 3. 添加新的性能指标

在 `metrics.definitions` 中添加新的指标定义：

```yaml
- name: power_efficiency
  expr: P_out / P_in
  aggregator: mean
  goal: maximize
  weight: 0.3
```

### 4. 集成外部控制算法

```python
# 通过 MCP 协议集成外部 AI 服务
class MCPAgent(BaseAgent):
    def __init__(self, mcp_client):
        self.client = mcp_client
    
    def get_action(self, state):
        # 调用外部 AI 服务
        return self.client.predict(state)
```

### 5. 添加新的可视化功能

在 `utils/visualization.py` 中添加新的绘图函数：

```python
def plot_frequency_analysis(simulator, var_name, save_path=None):
    # 实现频域分析绘图
    pass
```

## 故障排除

### 常见问题

#### 1. MATLAB Engine 连接失败
```bash
# 检查 MATLAB 安装
matlab -batch "version"

# 重新安装 Python 引擎
cd "MATLABROOT\extern\engines\python"
python setup.py install
```

#### 2. 模型路径错误
- 检查 `model_path` 配置是否为有效路径
- 确保 Simulink 模型文件存在且可访问

#### 3. 变量映射错误
- 检查 `input_map` 和 `output_map` 中的变量名
- 确保 Simulink 模型中存在对应的变量

#### 4. 内存不足
- 减少 `stop_time` 或 `simulation_step`
- 启用 `start_in_accelerator: true`

### 调试技巧

1. **启用详细日志**：
```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

2. **检查中间结果**：
```python
# 在 runner.step() 后检查各阶段结果
result = runner.step(action)
print("Simulation:", result["sim"])
print("Observation:", result["obs"])
print("Metrics:", result["metrics"])
```

3. **使用假会话测试**：
```python
# 在无 MATLAB 环境下测试
from inverter_ai_control.sim_env.fake_session import FakeSession
simulator = MatlabSimulator(config=config, session=FakeSession())
```

## 开发指南

### 代码规范

1. **注释规范**：使用中文注释，说明"为什么"和"如何"，而非"做什么"
2. **日志规范**：使用结构化日志，记录关键工作流程
3. **类型提示**：为所有函数添加类型提示
4. **错误处理**：提供清晰的错误信息和恢复建议

### 测试策略

1. **单元测试**：为每个核心模块编写单元测试
2. **集成测试**：测试模块间的交互
3. **端到端测试**：测试完整的仿真流程

### 性能优化

1. **MATLAB 优化**：使用加速模式，避免频繁的模型编译
2. **内存管理**：及时清理大型数组，使用适当的数据类型
3. **并行处理**：考虑使用多进程进行批量仿真

## 许可证

内部项目示例，按需补充。

## 贡献指南

1. Fork 项目
2. 创建特性分支
3. 提交更改
4. 推送到分支
5. 创建 Pull Request

## 联系方式

如有问题或建议，请联系项目维护者。