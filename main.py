"""独立演示脚本：无智能体，仅使用 MatlabSimulator 进行手动或预设控制。

要点：
- 从配置文件加载模型与变量映射，不在代码中硬编码
- 通过交互输入或预设序列生成 Kp/Ki，并调用 simulator.run_step(action)
- 实时打印结果；若安装 matplotlib，可选择在预设模式后绘制曲线
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

try:
    import matplotlib.pyplot as plt
    HAS_PLT = True
except Exception:
    HAS_PLT = False

from inverter_ai_control.utils.logger import get_logger
from inverter_ai_control.utils.config_loader import load_config
from inverter_ai_control.sim_env.matlab_simulator import MatlabSimulator


log = get_logger(__name__)


def find_config() -> Path:
    """查找配置文件：优先 config/config.yaml，其次 config/default.yaml，最后包内示例。"""
    project_root = Path(__file__).parent
    for rel in ("config/config.yaml", "config/default.yaml", "inverter_ai_control/config/default.yaml"):
        p = project_root / rel
        if p.exists():
            return p
    # 若都不存在，仍返回默认位置，load_config 会抛出清晰错误
    return project_root / "config/config.yaml"


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


def run_preset(sim: MatlabSimulator, cfg: Dict[str, Any], steps: int, kp_start: float, kp_end: float, ki: float) -> None:
    """预设模式：按线性序列扫描 Kp，Ki 固定，循环推进仿真并可选绘图。"""
    bounds = dict(cfg.get("action_bounds", {}))
    times: List[float] = []
    vout: List[float] = []
    vref: List[float] = []
    errs: List[float] = []

    for i, kp in enumerate(np.linspace(kp_start, kp_end, num=steps)):
        action = clamp_action({"Kp": float(kp), "Ki": float(ki)}, bounds)
        res = sim.run_step(action)
        times.append(float(res.get("time", (i + 1))))
        def last(x: Any) -> float:
            arr = np.asarray(x).flatten()
            return float(arr[-1]) if arr.size else float("nan")
        vout.append(last(res.get("V_out", 0.0)))
        vref.append(last(res.get("V_ref", 0.0)))
        errs.append(last(res.get("error", 0.0)))
        log.info("preset.step", step=i, kp=action["Kp"], ki=action["Ki"], t=times[-1], err=errs[-1])

    # 简单绘图（可选）
    if HAS_PLT:
        import matplotlib.pyplot as plt  # 局部导入以避免无环境时报错
        plt.figure(figsize=(8, 4))
        plt.plot(times, vout, label="V_out")
        plt.plot(times, vref, label="V_ref")
        plt.plot(times, errs, label="error")
        plt.grid(True)
        plt.legend()
        plt.title("Simulation Outputs (Preset Mode)")
        plt.xlabel("time [s]")
        plt.tight_layout()
        plt.show()


def main() -> None:
    # 1) 加载配置
    cfg_path = find_config()
    cfg = load_config(cfg_path)

    # 2) 构造 MatlabSimulator（基于 matlab.engine）并 reset 一次
    sim = MatlabSimulator(config=cfg)

    # 3) 解析命令行：选择交互或预设
    parser = argparse.ArgumentParser(description="MatlabSimulator demo (no agent)")
    parser.add_argument("--interactive", action="store_true", help="交互模式：从命令行输入 Kp/Ki")
    parser.add_argument("--steps", type=int, default=20, help="预设模式步数")
    parser.add_argument("--kp-start", type=float, default=1.0, help="预设模式 Kp 起始")
    parser.add_argument("--kp-end", type=float, default=1.2, help="预设模式 Kp 终止")
    parser.add_argument("--ki", type=float, default=0.1, help="预设模式 Ki 常量")
    args = parser.parse_args()

    with sim:
        # 按需注入初值（可从配置 action_bounds 推导或自定义）
        sim.reset(initial_setpoints={"Kp": args.kp_start, "Ki": args.ki})

        if args.interactive:
            run_interactive(sim, cfg)
        else:
            run_preset(sim, cfg, steps=args.steps, kp_start=args.kp_start, kp_end=args.kp_end, ki=args.ki)


if __name__ == "__main__":
    main()


