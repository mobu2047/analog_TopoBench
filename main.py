"""独立演示脚本：无智能体，仅使用 MatlabSimulator 进行手动或预设控制。

要点：
- 从配置文件加载模型与变量映射，不在代码中硬编码
- 通过交互输入或预设序列生成 Kp/Ki，并调用 simulator.run_step(action)
- 实时打印结果；若安装 matplotlib，可选择在预设模式后绘制曲线
"""

from __future__ import annotations
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


def main() -> None:
    # 1) 加载配置
    cfg_path = find_config()
    cfg = load_config(cfg_path)

    # 2) 构造 MatlabSimulator（基于 matlab.engine）并 reset 一次
    sim = MatlabSimulator(config=cfg)

    runner = Runner(sim, cfg)
    with sim:
        # 使用新管线复位
        runner.reset()
        # 运行模式选择：支持 agent/cases；默认 cases，保持兼容性
        # 为什么：允许通过配置切换为自动调参与搜索；在 agent 模式下后续接入智能体迭代回路
        run_cfg = dict(cfg.get("run", {}))
        run_mode = str(run_cfg.get("mode", "cases")).lower()

        if run_mode == "agent":
            # 占位实现：先按 presets.init_action 运行一次整段仿真
            # 作用：保证在未接好智能体前也能成功运行与产出结果
            presets = cfg.get("presets", {}) or {}
            init_action = presets.get("init_action") or {}
            runner.reset(initial_action=init_action)
            out = runner.step({}, whole_duration=True)
            _save_and_plot(cfg, sim, out, case_name="agent_single_placeholder")
        else:
            # 批量整段仿真：按 experiments.cases 运行（默认路径）
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


