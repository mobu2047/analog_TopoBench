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

    - 优先通过 MATLAB 函数接口（非 eval 字符串）读取，避免非 ASCII 引起的解析错误
    - 先尝试 Dataset（numElements/getElement），失败则回退 timeseries（.Time/.Data）
    """
    eng = simulator._eng  # 使用现有 MATLAB 引擎

    # 变量是否存在
    try:
        scope = eng.workspace[var_name]
    except Exception:
        return []

    series: List[Tuple[np.ndarray, np.ndarray]] = []

    # 情况1：Dataset 读取（优先）
    try:
        n = int(eng.numElements(scope, nargout=1))
        for i in range(1, n + 1):
            el = eng.getElement(scope, float(i), nargout=1)
            vals = eng.getfield(el, 'Values', nargout=1)
            t = np.array(eng.getfield(vals, 'Time', nargout=1), dtype=float).reshape(-1)
            y_raw = np.array(eng.getfield(vals, 'Data', nargout=1), dtype=float)
            if y_raw.ndim == 1:
                series.append((t, y_raw.reshape(-1)))
            else:
                y2d = y_raw.reshape(y_raw.shape[0], -1)
                for c in range(y2d.shape[1]):
                    series.append((t, y2d[:, c]))
        return series
    except Exception:
        pass

    # 情况2：timeseries 回退
    try:
        t = np.array(eng.getfield(scope, 'Time', nargout=1), dtype=float).reshape(-1)
        y_raw = np.array(eng.getfield(scope, 'Data', nargout=1), dtype=float)
        if y_raw.ndim == 1:
            return [(t, y_raw.reshape(-1))]
        else:
            y2d = y_raw.reshape(y_raw.shape[0], -1)
            return [(t, y2d[:, c]) for c in range(y2d.shape[1])]
    except Exception:
        return []


def _format_actions(actions: Optional[dict]) -> str:
    if not actions:
        return ""
    parts = []
    for k, v in actions.items():
        if isinstance(v, (list, tuple, np.ndarray)):
            arr = np.asarray(v).reshape(-1)
            if arr.size <= 5:
                parts.append(f"{k}=[" + ",".join(f"{x:.4g}" for x in arr) + "]")
            else:
                head = ",".join(f"{x:.4g}" for x in arr[:3])
                tail = ",".join(f"{x:.4g}" for x in arr[-2:])
                parts.append(f"{k}=[{head},...,{tail}](n={arr.size})")
        elif isinstance(v, float):
            parts.append(f"{k}={v:.6g}")
        else:
            parts.append(f"{k}={v}")
    return "\n".join(parts)


def plot_scope_dataset(
    simulator,
    var_name: str = "ScopeData",
    label_prefix: str = "sig",
    save_path: Optional[str] = None,
    *,
    actions: Optional[dict] = None,
    xlabel: str = "Time [s]",
    ylabel: str = "Voltage [V]",
    sample_time: Optional[float] = None,
    time_mode: str = "native",   # native | reconstruct | scale_by_ts
    style: str = "line",         # line | stairs
    legend_from_actions: bool = False,  # 是否用动作值作为图例标签
):
    import matplotlib.pyplot as plt

    series = extract_scope_dataset(simulator, var_name)
    if not series:
        print(f"[plot_scope_dataset] {var_name} 为空或无法识别。")
        return
    plt.figure(figsize=(9, 4.8))
    ax = plt.gca()
    # 生成图例标签：优先使用动作的简短摘要
    def _legend_label(idx: int) -> str:
        if legend_from_actions and actions:
            # 仅保留简短的标量键，数组标记为 [...]
            items = []
            for k, v in actions.items():
                if isinstance(v, (int, float)):
                    items.append(f"{k}={v:.4g}")
                else:
                    items.append(f"{k}=[...]")
            label = ", ".join(items)
            # 避免过长
            return (label[:60] + "…") if len(label) > 60 else label
        return f"{label_prefix}{idx}"

    for idx, (t, y) in enumerate(series, start=1):
        # 根据需求修正时间轴：
        # - reconstruct: 用等间隔 sample_time 重建 t
        # - scale_by_ts: 若 t 为采样索引(0..N-1)，则乘以 sample_time
        if time_mode == "reconstruct" and (sample_time is not None):
            t = np.arange(len(y), dtype=float) * float(sample_time)
        elif time_mode == "scale_by_ts" and (sample_time is not None):
            t = np.asarray(t, dtype=float) * float(sample_time)
        else:
            t = np.asarray(t, dtype=float)

        if style == "stairs":
            ax.step(t, y, where="post", label=_legend_label(idx))
        else:
            ax.plot(t, y, label=_legend_label(idx))
    ax.grid(True)
    ax.legend()
    ax.set_title(f"{var_name}")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)

    # 在图中叠加当前动作参数（如 Kp/Ki/rep_t/L_load 等）
    text = _format_actions(actions)
    if text:
        ax.text(
            0.01,
            0.99,
            text,
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=9,
            family="monospace",
            bbox=dict(facecolor="white", alpha=0.7, edgecolor="gray"),
        )

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=160)
    else:
        plt.show()


