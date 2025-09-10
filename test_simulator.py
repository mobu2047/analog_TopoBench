"""MatlabSimulator 独立测试脚本。

使用方法：
1) 确保已安装 matlabengineforpython，并能在本机启动 MATLAB。
2) 确保 Simulink 模型将以下变量映射到 base workspace：
   - 输入：Kp_val, Ki_val（或根据你的 input_map 修改）
   - 输出：V_out, V_ref, error（由 To Workspace 写入，Array 格式）
3) 根据你的模型路径修改 MODEL_PATH 变量。

脚本将：
- 实例化 MatlabSimulator
- reset 一次
- 循环调用 run_step，逐步改变 Kp/Ki
- 打印关键结果；若安装 matplotlib，也会绘图
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List

import numpy as np

try:
    import matplotlib.pyplot as plt
    HAS_PLT = True
except Exception:
    HAS_PLT = False

from inverter_ai_control.sim_env.matlab_simulator import MatlabSimulator


def main() -> None:
    # 根据实际路径调整
    MODEL_PATH = Path("inverter_model.slx").resolve()

    sim = MatlabSimulator(
        model_path=str(MODEL_PATH),
        sample_time_s=1e-3,
        stop_time_s=0.05,
        input_map={"Kp": "Kp_val", "Ki": "Ki_val"},
        output_map={"V_out": "V_out", "V_ref": "V_ref", "error": "error"},
        extra_params={"Ts": 1e-3},
        start_in_accelerator=True,
    )

    times: List[float] = []
    vout: List[float] = []
    vref: List[float] = []
    err: List[float] = []

    try:
        with sim:
            init = sim.reset({"Kp": 1.0, "Ki": 0.1})
            print("reset:", init)

            for i in range(20):
                action = {"Kp": 1.0 + 0.02 * i, "Ki": 0.1}
                res = sim.run_step(action)
                times.append(float(res.get("time", (i + 1) * 1e-3)))

                # 结果可能是标量或 1xN 数组，统一取最后一个值
                def last(x):
                    arr = np.asarray(x).flatten()
                    return float(arr[-1]) if arr.size > 0 else float(arr)

                vout.append(last(res.get("V_out", 0.0)))
                vref.append(last(res.get("V_ref", 0.0)))
                err.append(last(res.get("error", 0.0)))

                print(f"step {i:02d} t={times[-1]:.4f}s V_out={vout[-1]:.3f} V_ref={vref[-1]:.3f} err={err[-1]:.3f}")

    finally:
        # 确保引擎资源释放
        sim.close()

    if HAS_PLT:
        plt.figure(figsize=(8, 4))
        plt.plot(times, vout, label="V_out")
        plt.plot(times, vref, label="V_ref")
        plt.plot(times, err, label="error")
        plt.grid(True)
        plt.legend()
        plt.title("Simulation Outputs")
        plt.xlabel("time [s]")
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()


