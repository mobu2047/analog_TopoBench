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

    # 先做类型判断：Dataset（Simulink.SimulationData.Dataset）与 timeseries 处理不同
    # 这么做的原因：直接 print 会得到 matlab.object，需要通过 getElement/Values 才能取出数值矩阵
    try:
        cls = str(eng.eval(f"class({var_name})", nargout=1))
    except Exception:
        # 变量不存在或不可见，直接返回空
        return []

    # 情况1：Scope 使用 To Workspace 保存为 Dataset（推荐在模型端保持 Dataset，保留信号名等元信息）
    if "Simulink.SimulationData.Dataset" in cls:
        # 使用 MATLAB 的 numElements/getElement API，而不是 numel；后者对对象可能返回 1
        try:
            n = int(eng.eval(f"numElements({var_name})", nargout=1))
        except Exception:
            # 部分版本可用对象方法调用
            n = int(eng.eval(f"{var_name}.numElements", nargout=1))

        series: List[Tuple[np.ndarray, np.ndarray]] = []
        for i in range(1, n + 1):
            # 将 timeseries 的 Time/Data 写入 base workspace 临时变量，便于安全取回
            # 使用 getElement(var, i) 避免依赖花括号索引语法在 Python 引擎中的差异
            eng.eval(
                f"__t__ = getElement({var_name},{i}).Values.Time; __y__ = getElement({var_name},{i}).Values.Data;",
                nargout=0,
            )

            # 将 matlab.double 转为 numpy；保持时间为 1D
            t = np.array(eng.workspace["__t__"], dtype=float).reshape(-1)
            y_raw = np.array(eng.workspace["__y__"], dtype=float)

            # 多通道数据：按列拆分成多条曲线；单通道保持 1D
            if y_raw.ndim == 1:
                series.append((t, y_raw.reshape(-1)))
            else:
                y2d = y_raw.reshape(y_raw.shape[0], -1)
                for c in range(y2d.shape[1]):
                    series.append((t, y2d[:, c]))
        return series

    # 情况2：不是 Dataset，尝试按 timeseries（或等价结构）处理
    try:
        eng.eval(f"__t__ = {var_name}.Time; __y__ = {var_name}.Data;", nargout=0)
        t = np.array(eng.workspace["__t__"], dtype=float).reshape(-1)
        y_raw = np.array(eng.workspace["__y__"], dtype=float)
        if y_raw.ndim == 1:
            return [(t, y_raw.reshape(-1))]
        else:
            y2d = y_raw.reshape(y_raw.shape[0], -1)
            return [(t, y2d[:, c]) for c in range(y2d.shape[1])]
    except Exception:
        return []


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


