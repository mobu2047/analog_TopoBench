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
        # 使用 feval 获取元素数量/元素对象
        n = int(eng.feval('numElements', scope, nargout=1))
        for i in range(1, n + 1):
            el = eng.feval('getElement', scope, float(i), nargout=1)
            # 属性访问使用 getfield，避免类不支持 get 的报错
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
def plot_outputs_from_result(
    cfg: dict,
    result: dict,
    save_dir: str,
    *,
    label_prefix: str = "",
    actions: Optional[dict] = None,
    filename: str = "outputs.png",
    simulator=None,
    out_map: Optional[dict] = None,
    **kwargs,
) -> Optional[str]:
    """根据配置与 step 结果自动绘制输出变量，并保存到指定目录。

    约定：
    - cfg['matlab']['output_map'] 提供逻辑名 -> 基础工作区名的映射；此处使用逻辑名索引 result['sim']
    - result['sim'] 至少包含 'time'（标量）与可选 'tout'（时间向量）；若无 'tout'，则尝试用 sim.step 重建
    - 对除 'tout' 之外的数组信号逐一绘制
    """
    import os
    import matplotlib.pyplot as plt

    sim_cfg = dict(cfg.get("sim", {}))
    matlab_cfg = dict(cfg.get("matlab", {}))
    output_map = dict(matlab_cfg.get("output_map", {}))
    sim_out = dict(result.get("sim", {}))

    # 时间轴优先使用 'tout'
    t = None
    if "tout" in sim_out:
        t = np.asarray(sim_out.get("tout"), dtype=float).reshape(-1)
    step_ts = float(sim_cfg.get("step", 0.0) or 0.0)

    # 选择要绘制的信号（排除 'tout' 与时间标量），支持数组与 zip(通道迭代器)
    # 若调用方提供了 out_map（向后兼容），合并到配置映射中（out_map 优先）
    if out_map:
        output_map = {**output_map, **dict(out_map)}

    logical_names = [k for k in output_map.keys() if k not in {"tout", "time"}] or [k for k in sim_out.keys() if k not in {"tout", "time"}]
    def _is_array_like(v):
        return isinstance(v, (list, tuple, np.ndarray))
    def _is_zip(v):
        return hasattr(v, "__class__") and v.__class__.__name__ == 'zip'
    y_names = [name for name in logical_names if _is_array_like(sim_out.get(name)) or _is_zip(sim_out.get(name))]
    if not y_names:
        y_names = [k for k, v in sim_out.items() if k not in {"tout", "time"} and (_is_array_like(v) or _is_zip(v))]

    # 兜底1：若仅有 tout（时间向量）且为数组，则绘制 tout vs sample_index
    if not y_names and isinstance(sim_out.get("tout"), (list, tuple, np.ndarray)):
        import os
        import matplotlib.pyplot as plt
        os.makedirs(save_dir, exist_ok=True)
        tt = np.asarray(sim_out.get("tout"), dtype=float).reshape(-1)
        plt.figure(figsize=(9, 4.8))
        ax = plt.gca()
        ax.plot(np.arange(tt.size, dtype=float), tt, label="tout")
        ax.grid(True)
        ax.legend()
        ax.set_title("tout")
        ax.set_xlabel("Sample Index")
        ax.set_ylabel("Time [s]")
        plt.tight_layout()
        save_path = os.path.join(save_dir, filename)
        plt.savefig(save_path, dpi=160)
        return save_path

    # 兜底2：既没有数组信号，又提供了 simulator，则从工作区提取 ScopeData 直接绘制
    if not y_names:
        if simulator is not None:
            series = extract_scope_dataset(simulator, var_name=output_map.get("ScopeData", "ScopeData"))
            if series:
                import os
                import matplotlib.pyplot as plt
                os.makedirs(save_dir, exist_ok=True)
                plt.figure(figsize=(9, 4.8))
                ax = plt.gca()
                for idx, (tt, yy) in enumerate(series, start=1):
                    ax.plot(np.asarray(tt, dtype=float).reshape(-1), np.asarray(yy, dtype=float).reshape(-1), label=f"{label_prefix}{idx}" if label_prefix else f"sig{idx}")
                ax.grid(True)
                ax.legend()
                ax.set_title("ScopeData")
                ax.set_xlabel("Time [s]")
                ax.set_ylabel("Value")
                plt.tight_layout()
                save_path = os.path.join(save_dir, filename)
                plt.savefig(save_path, dpi=160)
                return save_path
        # 若仍无法绘制，则返回
        return None

    os.makedirs(save_dir, exist_ok=True)
    plt.figure(figsize=(9, 4.8))
    ax = plt.gca()

    def _align_time(tt_in, yy_in):
        import numpy as _np
        yy = _np.asarray(yy_in, dtype=float).reshape(-1)
        if tt_in is None:
            if step_ts and yy.size > 0:
                return _np.arange(yy.size, dtype=float) * step_ts, yy
            return _np.arange(yy.size, dtype=float), yy
        tt = _np.asarray(tt_in, dtype=float).reshape(-1)
        if tt.size == yy.size:
            return tt, yy
        # 尝试对齐：优先裁剪较长的序列；若差异过大且有步长，则重建
        if step_ts and abs(tt.size - yy.size) > max(10, int(0.001 * max(tt.size, yy.size))):
            return _np.arange(yy.size, dtype=float) * step_ts, yy
        if tt.size > yy.size:
            return tt[:yy.size], yy
        # tt 较短：线性插值为与 y 等长
        try:
            t_new = _np.linspace(tt[0] if tt.size else 0.0, tt[-1] if tt.size else (yy.size - 1) * (step_ts or 1.0), yy.size)
            return t_new, yy
        except Exception:
            return _np.arange(yy.size, dtype=float), yy

    for idx, name in enumerate(y_names, start=1):
        val = sim_out.get(name)
        if _is_zip(val):
            # zip 迭代器：每个元素是 (t, y) 或 (y)；优先使用提供的 t，否则用公共 t 或重建
            chan_idx = 0
            for item in list(val):
                chan_idx += 1
                if isinstance(item, (list, tuple)) and len(item) == 2:
                    tt, yy = _align_time(item[0], item[1])
                else:
                    tt, yy = _align_time(t, item)
                label = f"{name}_ch{chan_idx}"
                ax.plot(tt, yy, label=label)
        else:
            tt, y = _align_time(t, val)
            label = f"{label_prefix}{idx}" if label_prefix else name
            ax.plot(tt, y, label=label)

    ax.grid(True)
    ax.legend()
    ax.set_title("Outputs")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Value")
    plt.tight_layout()
    save_path = os.path.join(save_dir, filename)
    plt.savefig(save_path, dpi=160)
    return save_path
