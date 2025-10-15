"""独立演示脚本：无智能体，仅使用 MatlabSimulator 进行手动或预设控制。

要点：
- 从配置文件加载模型与变量映射，不在代码中硬编码
- 通过交互输入或预设序列生成 Kp/Ki，并调用 simulator.run_step(action)
- 实时打印结果；若安装 matplotlib，可选择在预设模式后绘制曲线
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np


try:
    import matplotlib.pyplot as plt
    HAS_PLT = True
except Exception:
    HAS_PLT = False

from inverter_ai_control.utils.logger import get_logger
from inverter_ai_control.utils.config_loader import load_config
from inverter_ai_control.sim_env.matlab_simulator import MatlabSimulator
from inverter_ai_control.core.runner import Runner



log = get_logger(__name__)


def find_config() -> Path:
    """查找配置文件：优先 config/config.yaml 其后 config/default.yaml；目标 v2。"""
    project_root = Path(__file__).parent
    for rel in ("config/config.yaml", "config/default.yaml"):
        p = project_root / rel
        if p.exists():
            return p
    return project_root / "config/default.yaml"


def clamp_action(action: Dict[str, float], bounds: Dict[str, Dict[str, float]]) -> Dict[str, float]:
    """按配置范围裁剪动作，防止越界。

    设计考量：
    - 将安全性约束前移到应用层，便于快速调试与保护模型
    """
    clamped = {}
    for k, v in action.items():
        b = bounds.get(k, {})
        vmin = float(b.get("min", v))
        vmax = float(b.get("max", v))
        clamped[k] = float(np.clip(float(v), vmin, vmax))
    return clamped


def run_interactive(sim: MatlabSimulator, cfg: Dict[str, Any]) -> None:
    """交互模式：从命令行输入 Kp/Ki，逐步推进仿真。"""
    bounds = dict(cfg.get("action_bounds", {}))
    print("进入交互模式。输入: Kp Ki ，或 'q' 退出。例如: 1.0 0.1")
    while True:
        try:
            raw = input("Kp Ki > ").strip()
        except EOFError:
            break
        if raw.lower() in {"q", "quit", "exit"}:
            break
        parts = raw.split()
        if len(parts) != 2:
            print("格式错误，请输入: Kp Ki，例如: 1.0 0.1")
            continue
        try:
            kp, ki = float(parts[0]), float(parts[1])
        except ValueError:
            print("解析失败，请输入数字，例如: 1.0 0.1")
            continue

        action = clamp_action({"Kp": kp, "Ki": ki}, bounds)
        res = sim.run_step(action)
        # 打印关键输出
        t = float(res.get("time", 0.0))
        v_out = np.asarray(res.get("V_out", [])).flatten()
        v_ref = np.asarray(res.get("V_ref", [])).flatten()
        err = np.asarray(res.get("error", [])).flatten()
        v_out_last = float(v_out[-1]) if v_out.size else float("nan")
        v_ref_last = float(v_ref[-1]) if v_ref.size else float("nan")
        err_last = float(err[-1]) if err.size else float("nan")
        print(f"t={t:.6f}s  Kp={action['Kp']:.4f} Ki={action['Ki']:.4f}  V_out={v_out_last:.4f} V_ref={v_ref_last:.4f} err={err_last:.4f}")


# def run_preset(sim: MatlabSimulator, cfg: Dict[str, Any], steps: int, kp_start: float, kp_end: float, ki: float) -> None:
#     """预设模式：按线性序列扫描 Kp，Ki 固定，循环推进仿真并可选绘图。"""
#     bounds = dict(cfg.get("action_bounds", {}))
#     times: List[float] = []
#     vout: List[float] = []
#     vref: List[float] = []
#     errs: List[float] = []

#     for i, kp in enumerate(np.linspace(kp_start, kp_end, num=steps)):
#         action = clamp_action({"Kp": float(kp), "Ki": float(ki)}, bounds)
#         res = sim.run_step(action)
#         times.append(float(res.get("time", (i + 1))))
#         def last(x: Any) -> float:
#             arr = np.asarray(x).flatten()
#             return float(arr[-1]) if arr.size else float("nan")
#         vout.append(last(res.get("V_out", 0.0)))
#         vref.append(last(res.get("V_ref", 0.0)))
#         errs.append(last(res.get("error", 0.0)))
#         log.info("preset.step", step=i, kp=action["Kp"], ki=action["Ki"], t=times[-1], err=errs[-1])

#     # 简单绘图（可选）
#     if HAS_PLT:
#         import matplotlib.pyplot as plt  # 局部导入以避免无环境时报错
#         plt.figure(figsize=(8, 4))
#         plt.plot(times, vout, label="V_out")
#         plt.plot(times, vref, label="V_ref")
#         plt.plot(times, errs, label="error")
#         plt.grid(True)
#         plt.legend()
#         plt.title("Simulation Outputs (Preset Mode)")
#         plt.xlabel("time [s]")
#         plt.tight_layout()
#         plt.show()


def main() -> None:
    # 1) 加载配置
    cfg_path = find_config()
    cfg = load_config(cfg_path)

    # 2) 构造 MatlabSimulator（基于 matlab.engine）并 reset 一次
    sim = MatlabSimulator(config=cfg)

    # 3) 解析命令行：选择交互或预设
    parser = argparse.ArgumentParser(description="MatlabSimulator demo (no agent)")
    parser.add_argument("--interactive", action="store_true", help="交互模式：根据 action_space 动态输入键值")
    args = parser.parse_args()

    runner = Runner(sim, cfg)
    with sim:
        # 使用新管线复位
        runner.reset()

        if args.interactive:
            # 动态交互：根据 action_space 逐步输入（v2 不再使用全局 action_bounds）
            action_space = cfg.get("action_space", [])
            print("交互模式：输入键=值（以空格分隔），例如：Kp=1.0 Ki=0.1；输入 q 退出")
            while True:
                raw = input("> ").strip()
                if raw.lower() in {"q", "quit", "exit"}:
                    break
                pairs = [p for p in raw.split() if "=" in p]
                action: Dict[str, Any] = {}
                for p in pairs:
                    k, v = p.split("=", 1)
                    try:
                        action[k] = float(v)
                    except ValueError:
                        # 简单数组支持: rep_t=[0,0.00005,0.0001]
                        if v.startswith("[") and v.endswith("]"):
                            nums = [float(x) for x in v.strip("[]").split(",") if x]
                            action[k] = nums
                out = runner.step(action)
                print(out["metrics"])  # 简要反馈
        else:
            # 批量整段仿真：按 experiments.cases 运行
            experiments = dict(cfg.get("experiments", {}))
            cases = list(experiments.get("cases", []) or [])
            if not cases:
                # 回退到单次：仅用 presets.init_action
                presets = cfg.get("presets", {}) or {}
                init_action = presets.get("init_action") or {}
                runner.reset(initial_action=init_action)
                out = runner.step({}, whole_duration=True)
                _save_and_plot(cfg, sim, out, case_name="single")
            else:
                for case in cases:
                    name = str(case.get("name", "case"))
                    action = dict(case.get("action", {}))
                    # 直接使用键为 name 或 block.param 的动作；Runner 内部会进行裁剪与映射
                    runner.reset(initial_action=(cfg.get("presets", {}) or {}).get("init_action"))
                    out = runner.step(action, whole_duration=True)
                    _save_and_plot(cfg, sim, out, case_name=name)


def _save_and_plot(cfg: Dict[str, Any], sim: MatlabSimulator, out: Dict[str, Any], *, case_name: str) -> None:
    """将结果保存到 runs/ 并尽量输出图与摘要。"""
    import os, time
    run_dir = os.path.join("runs", time.strftime("%Y%m%d-%H%M%S"), case_name)
    os.makedirs(run_dir, exist_ok=True)
    log.info("run.metrics", metrics=out.get("metrics", {}), case=case_name)
    from inverter_ai_control.utils.visualization import plot_outputs_from_result
    # 解析并打印 MATLAB Dataset/Timeseries 数据摘要（仅限定 ScopeData*/yout，避免误读 'time' 等标量）
    out_map = {}
    try:
        from inverter_ai_control.utils.visualization import extract_scope_dataset
        sim_out_keys = list(out.get("sim", {}).keys())
        dataset_vars = [k for k in sim_out_keys if k.lower().startswith("scopedata") or k == "yout"]
        for var_name in dataset_vars:
            series = extract_scope_dataset(sim, var_name)
            if not series:
                continue
            print(f"[Dataset] {var_name}: channels={len(series)}")
            for ch_idx, (tt, yy) in enumerate(series, start=1):
                import numpy as _np
                tt = _np.asarray(tt).reshape(-1)
                yy = _np.asarray(yy).reshape(-1)
                head_t = ", ".join(f"{float(x):.4g}" for x in tt[:5])
                head_y = ", ".join(f"{float(x):.4g}" for x in yy[:5])
                print(f"  - ch{ch_idx}: len={len(tt)}  t=[{head_t}]  y=[{head_y}] ...")
                out_map[var_name] = zip(tt, yy)
        plot_outputs_from_result(cfg, out, run_dir, label_prefix=case_name+" ", filename="outputs.png", simulator=sim, out_map=out_map)
    except Exception as _e:
        print(f"[warn] failed to extract dataset/timeseries: {_e}")
    # 打印 summary 键
    sim_out = dict(out.get("sim", {}))
    keys = sorted(sim_out.keys())
    print("\n[Simulation outputs summary]", case_name)
    print("keys:", keys)
            

if __name__ == "__main__":
    main()


