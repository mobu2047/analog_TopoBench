"""可视化工具：从 MATLAB 工作区读取 ScopeData 并绘图。

使用说明：
- ScopeData 通常为 Simulink.SimulationData.Dataset，其中每个元素的 Values 为 timeseries
- 我们通过 matlab.engine 在 MATLAB 侧把 Time/Data 提取到 base workspace，再转为 numpy 绘图
"""

from __future__ import annotations

from typing import List, Tuple, Optional
import numpy as np


def extract_scope_dataset(simulator, var_name: str = "ScopeData") -> List[Tuple[np.ndarray, np.ndarray]]:
    """将工作区的 ScopeData 转成 [(t, y), ...] 列表。

    - 若数据集为空，返回空列表
    - 每个元素对应 Scope 的一个通道（或一个已记录信号）
    """
    eng = simulator._eng  # 使用现有 MATLAB 引擎
    try:
        n = int(eng.eval(f"numel({var_name})", nargout=1))
    except Exception:
        # 可能不是 Dataset，尝试按 timeseries 处理
        try:
            eng.eval(f"__t__ = {var_name}.Time; __y__ = {var_name}.Data;", nargout=0)
            t = np.array(eng.workspace["__t__"]).reshape(-1)
            y = np.array(eng.workspace["__y__"]).reshape(-1)
            return [(t, y)]
        except Exception:
            return []

    series: List[Tuple[np.ndarray, np.ndarray]] = []
    for i in range(1, n + 1):
        # t_i / y_i 放到 base workspace，便于取回
        eng.eval(f"__t__ = {var_name}{{{i}}}.Values.Time; __y__ = {var_name}{{{i}}}.Values.Data;", nargout=0)
        t = np.array(eng.workspace["__t__"]).reshape(-1)
        y_raw = np.array(eng.workspace["__y__"])  # 可能是列向量或多维
        y = y_raw.reshape(-1)
        series.append((t, y))
    return series


def plot_scope_dataset(simulator, var_name: str = "ScopeData", label_prefix: str = "sig", save_path: Optional[str] = None):
    import matplotlib.pyplot as plt

    series = extract_scope_dataset(simulator, var_name)
    if not series:
        print(f"[plot_scope_dataset] {var_name} 为空或无法识别。")
        return
    plt.figure(figsize=(8, 4))
    for idx, (t, y) in enumerate(series, start=1):
        plt.plot(t, y, label=f"{label_prefix}{idx}")
    plt.grid(True)
    plt.legend()
    plt.title(f"{var_name} signals")
    plt.xlabel("time [s]")
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=160)
    else:
        plt.show()


