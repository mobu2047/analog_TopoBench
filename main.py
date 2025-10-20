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
import os
import json
import yaml


try:
    import matplotlib.pyplot as plt
    HAS_PLT = True
except Exception:
    HAS_PLT = False

from inverter_ai_control.utils.logger import get_logger
from inverter_ai_control.utils.config_loader import load_config
from inverter_ai_control.sim_env.matlab_simulator import MatlabSimulator, MatlabSimulationError
from inverter_ai_control.core.runner import Runner
from inverter_ai_control.core.metrics import MetricEvaluator
from inverter_ai_control.agents.ollama_agent import OllamaAgent
import time

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
    # 运行模式选择：支持 agent/cases；默认 cases，保持兼容性
    run_cfg = dict(cfg.get("run", {}))
    run_mode = str(run_cfg.get("mode", "cases")).lower()

    probe_updated = False

    with sim:
        # 使用新管线复位
        runner.reset()

        if run_mode == "agent":
            # 第一次探测运行：不带任何参数，获取结果用于更新 output_map
            presets = cfg.get("presets", {}) or {}
            init_action = presets.get("init_action") or {}
            runner.reset(initial_action=init_action)
            out_probe = runner.step({}, whole_duration=True)
            _save_and_plot(cfg, sim, out_probe, case_name="agent_probe")

            # 基于探测结果，最小化更新 output_map：补充 ScopeData/tout（若缺失）
            add_map: Dict[str, str] = {}
            matlab_cfg = dict(cfg.get("matlab", {}))
            output_map = dict(matlab_cfg.get("output_map", {}))
            if "ScopeData" not in output_map:
                add_map["ScopeData"] = "ScopeData"
            if "tout" not in output_map:
                add_map["tout"] = "tout"

            if add_map:
                try:
                    _update_output_map_on_disk(cfg_path, add_map)
                    probe_updated = True
                    log.info("agent.output_map.updated", added=list(add_map.keys()))
                except Exception as _e_upd:
                    log.warning("agent.output_map.update_failed", error=str(_e_upd))
        else:
            # 批量整段仿真：按 experiments.cases 运行（默认路径），可选合并 experiments.grid
            experiments = dict(cfg.get("experiments", {}))
            cases = list(experiments.get("cases", []) or [])
            gridspec = experiments.get("grid")
            if gridspec:
                cases.extend(_expand_grid_cases(cfg, gridspec))
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
                    try:
                        out = runner.step(action, whole_duration=True)
                    except MatlabSimulationError as _e_case:
                        log.warning("case.failed", case=name, error=str(_e_case))
                        continue
                    _save_and_plot(cfg, sim, out, case_name=name)

    # 若为 agent 模式：探测后重新加载配置并执行智能仿真循环
    if run_mode == "agent":
        # 重新加载配置（若有更新）
        if probe_updated:
            cfg = load_config(cfg_path)

        sim2 = MatlabSimulator(config=cfg)
        runner = Runner(sim2, cfg)

        agent_cfg = dict(cfg.get("run", {})).get("agent", {}) or {}
        max_iterations = int(agent_cfg.get("max_iterations", 20))
        timeout_s = int(agent_cfg.get("timeout_s", 300))
        early_stop_no_improve = int(agent_cfg.get("early_stop_no_improve", 5))
        agent_type = str(agent_cfg.get("type", "ollama")).lower()
        host = str(agent_cfg.get("host", "") or "").strip()
        model = str(agent_cfg.get("model", "") or "").strip()
        prompt_template = agent_cfg.get("prompt_template", "")
        request_timeout_s = int(agent_cfg.get("request_timeout_s", 30))

        metric_evaluator = MetricEvaluator(cfg.get("metrics", {}))

        def _extract_numeric_space(conf: Dict[str, Any]) -> List[Tuple[str, float, float]]:
            numeric: List[Tuple[str, float, float]] = []
            for spec in conf.get("action_space", []) or []:
                key = spec.get("key")
                dt = str(spec.get("dtype", "float")).lower()
                if dt not in {"float", "int"}:
                    continue
                b = spec.get("bounds", {}) or {}
                if "min" in b and "max" in b:
                    try:
                        lo = float(b.get("min"))
                        hi = float(b.get("max"))
                        if hi > lo:
                            numeric.append((str(key), lo, hi))
                    except Exception:
                        continue
            return numeric

        numeric_space = _extract_numeric_space(cfg)
        if not numeric_space:
            log.warning("agent.no_numeric_space", msg="未找到可数值化的 action_space，回退到单次整段仿真")
            with sim2:
                presets = cfg.get("presets", {}) or {}
                init_action = presets.get("init_action") or {}
                runner.reset(initial_action=init_action)
                out = runner.step({}, whole_duration=True)
                _save_and_plot(cfg, sim2, out, case_name="agent_fallback_single")
            return

        ollama_agent = None
        if agent_type == "ollama" and host and model:
            try:
                ollama_agent = OllamaAgent(
                    host=host,
                    model=model,
                    action_space=numeric_space,
                    prompt_template=prompt_template or None,
                    timeout_s=request_timeout_s,
                )
            except Exception as _e_init:
                log.warning("agent.ollama_init_failed", error=str(_e_init))

        def _sample_action() -> Dict[str, float]:
            act: Dict[str, float] = {}
            for k, lo, hi in numeric_space:
                act[k] = float(np.random.uniform(lo, hi))
            return act

        with sim2:
            presets = cfg.get("presets", {}) or {}
            init_action = presets.get("init_action") or {}
            runner.reset(initial_action=init_action)

            start_ts = time.time()
            best_obj = float("inf")
            best_out: Dict[str, Any] = {}
            best_action: Dict[str, float] = {}
            no_improve = 0
            last_action: Dict[str, float] | None = None
            last_objective: float | None = None

            for i in range(max_iterations):
                if timeout_s > 0 and (time.time() - start_ts) > timeout_s:
                    log.warning("agent.timeout", iter=i, elapsed_s=time.time() - start_ts)
                    break

                action: Dict[str, float] | None = None
                if i == 0 and init_action:
                    action = dict(init_action)
                if action is None and ollama_agent is not None:
                    try:
                        state = {
                            "iteration": i,
                            "last_objective": last_objective,
                            "best_objective": best_obj if best_obj != float("inf") else None,
                            "numeric_space": numeric_space,
                            "last_action": last_action,
                        }
                        action = ollama_agent.get_action(state)
                    except Exception as _e_call:
                        log.warning("agent.ollama_call_failed", iter=i, error=str(_e_call))
                        action = None
                if action is None:
                    action = _sample_action()

                out = runner.step(action, whole_duration=True)
                eval_res = metric_evaluator.evaluate(out.get("sim", {}))
                obj = eval_res.get("objective")
                constraints = dict(eval_res.get("constraints", {}))
                all_ok = (len(constraints) == 0) or all(bool(v) for v in constraints.values())

                improved = False
                if obj is not None:
                    try:
                        import math as _math
                        if _math.isfinite(float(obj)) and float(obj) < best_obj:
                            best_obj = float(obj)
                            best_out = out
                            best_action = dict(action)
                            improved = True
                    except Exception:
                        pass

                if improved:
                    no_improve = 0
                else:
                    no_improve += 1

                last_action = dict(action)
                try:
                    last_objective = float(obj) if obj is not None else None
                except Exception:
                    last_objective = None

                log.info(
                    "agent.iter",
                    iter=i,
                    obj=float(obj) if obj is not None else None,
                    improved=improved,
                    no_improve=no_improve,
                    constraints_ok=all_ok,
                )

                if all_ok and (i >= 0):
                    log.info("agent.early_stop.constraints_ok", iter=i)
                    if improved:
                        best_out = out
                        best_action = dict(action)
                    break

                if early_stop_no_improve > 0 and no_improve >= early_stop_no_improve:
                    log.info("agent.early_stop.no_improve", iter=i, window=early_stop_no_improve)
                    break

            final_out = best_out if best_out else out
            save_dir = _save_and_plot(cfg, sim2, final_out, case_name="agent_best")
            try:
                final_eval = metric_evaluator.evaluate(final_out.get("sim", {}))
            except Exception:
                final_eval = {"metrics": {}, "constraints": {}, "objective": None}
            summary = {
                "agent": {
                    "type": agent_type,
                    "host": host,
                    "model": model,
                },
                "best_action": best_action if best_action else last_action,
                "evaluation": final_eval,
            }
            try:
                with open(os.path.join(save_dir or "runs", "summary.json"), "w", encoding="utf-8") as f:
                    json.dump(summary, f, ensure_ascii=False, indent=2)
            except Exception:
                pass


def _save_and_plot(cfg: Dict[str, Any], sim: MatlabSimulator, out: Dict[str, Any], *, case_name: str) -> str | None:
    """将结果保存到 runs/ 并尽量输出图与摘要；返回保存目录。"""
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
    # 将键与映射保存为 keys.json，便于后续补充指标/信号映射
    try:
        with open(os.path.join(run_dir, "keys.json"), "w", encoding="utf-8") as f:
            json.dump(
                {
                    "sim_keys": keys,
                    "output_map": dict(dict(cfg.get("matlab", {})).get("output_map", {})),
                },
                f,
                ensure_ascii=False,
                indent=2,
            )
    except Exception:
        pass
    return run_dir


def _expand_grid_cases(cfg: Dict[str, Any], grid_cfg: Dict[str, Any]) -> List[Dict[str, Any]]:
    """根据 action_space 的数值 bounds 生成笛卡儿积 cases。

    grid 配置：
      - divisions: int = 5            # 每变量划分份数，采样点数=divisions+1
      - variables: [key1, key2, ...]  # 可选：仅对这些变量做网格
    仅处理 dtype in {float,int} 且同时具备 bounds.min/max 的条目。
    """
    import itertools
    import numpy as _np

    divisions = int(grid_cfg.get("divisions", 5))
    var_whitelist = grid_cfg.get("variables")

    specs: List[Tuple[str, List[float]]] = []
    for item in (cfg.get("action_space", []) or []):
        key = item.get("key") or (f"{item.get('block')}.{item.get('param')}" if (item.get("block") and item.get("param")) else item.get("name"))
        if not key:
            continue
        dt = str(item.get("dtype", "float")).lower()
        if dt not in {"float", "int"}:
            continue
        b = item.get("bounds", {}) or {}
        if "min" not in b or "max" not in b:
            continue
        if var_whitelist and key not in var_whitelist:
            continue
        try:
            lo = float(b["min"])
            hi = float(b["max"])
            if hi <= lo:
                continue
            xs = _np.linspace(lo, hi, num=max(2, divisions + 1))
            vals: List[float] = []
            if dt == "float":
                # 避免取到精确 0（如电阻/电感等不允许为 0），将 0 调整为极小正数
                tiny = max(_np.finfo(float).eps, (hi - lo) / 1e9)
                for x in xs:
                    if float(x) == 0.0:
                        vals.append(float(tiny))
                    else:
                        vals.append(float(x))
            else:
                vals = [float(x) for x in xs]
            specs.append((str(key), vals))
        except Exception:
            continue

    if not specs:
        return []

    keys = [k for k, _ in specs]
    grids = [vals for _, vals in specs]
    cases: List[Dict[str, Any]] = []
    for combo in itertools.product(*grids):
        action = {k: float(v) for k, v in zip(keys, combo)}
        name_parts = [f"{k.replace(' ', '_').replace('.', '_')}={v:g}" for k, v in action.items()]
        cases.append({"name": "grid_" + "__".join(name_parts), "action": action})
    return cases


def _update_output_map_on_disk(cfg_path: Path, additions: Dict[str, str]) -> None:
    """将 additions 写入配置文件的 matlab.output_map 中（幂等合并）。

    - 若文件无 matlab/output_map，则创建对应层级
    - 仅添加缺失键，不覆盖已存在键
    """
    with open(cfg_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if "matlab" not in data or not isinstance(data["matlab"], dict):
        data["matlab"] = {}
    if "output_map" not in data["matlab"] or not isinstance(data["matlab"]["output_map"], dict):
        data["matlab"]["output_map"] = {}
    for k, v in additions.items():
        data["matlab"]["output_map"].setdefault(k, v)
    with open(cfg_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

if __name__ == "__main__":
    main()


