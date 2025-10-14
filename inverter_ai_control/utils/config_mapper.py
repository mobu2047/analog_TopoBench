"""配置映射器：将 MATLAB 导出的 JSON 参数映射到本项目 YAML 配置。

为什么做：
- MATLAB/Simulink 的参数导出格式不统一（按块、掩模、对话参数组织），而本项目的 YAML 配置结构相对稳定
- 通过“映射规则”将两者解耦，新增模型时仅需更新映射文件，而非修改代码

如何用：
- 生成全量参数快照（仅参数树）：
    python -m inverter_ai_control.utils.config_mapper --json simulink/export_model_graph/model_params.json --out config/auto_params.yaml
- 可选：基于快照生成 v2 主配置：
    python -m inverter_ai_control.utils.config_mapper --gen-default --model-path simulink/untitled3.slx --stop-time 0.1 --auto config/auto_params.yaml --config config/default.yaml

设计要点：
- 映射规则为 YAML：声明“目标 YAML 路径”和“源 JSON 选择器”，可选一系列 transform
- 支持静态值与动态生成混合：rule 可直接提供 value，或从 JSON 选择并应用 transforms
- 最小侵入：不改变现有 YAML 结构，仅按 target path 写入/更新对应键值
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import argparse
import json
import re
import yaml

# 兼容“作为模块运行”与“直接运行文件”两种方式：
# - 模块运行：python -m inverter_ai_control.utils.config_mapper
# - 直接运行：python inverter_ai_control/utils/config_mapper.py
try:
    from .logger import get_logger  # 包内相对导入（推荐）
except Exception:
    # 当用户直接运行本文件时，相对导入会失败；
    # 这里将仓库根目录加入 sys.path 再做绝对导入。
    import sys
    _THIS_FILE = Path(__file__).resolve()
    _REPO_ROOT = _THIS_FILE.parents[2]
    if str(_REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT))
    from inverter_ai_control.utils.logger import get_logger  # type: ignore


log = get_logger(__name__)


# ============================ 数据结构定义 ============================

@dataclass
class JsonSelector:
    """描述如何在 MATLAB 导出的 JSON 中定位一个参数。

    字段说明：
    - path:    块完整路径（优先使用精确匹配）。可选：regex=true 时使用正则。
    - block_type/mask_type: 作为 path 缺失时的备选过滤条件（与/或）。
    - param:   对话参数名称（DialogParams 内键名）。
    - regex:   path 是否作为正则匹配。
    """

    path: Optional[str] = None
    block_type: Optional[str] = None
    mask_type: Optional[str] = None
    param: Optional[str] = None
    regex: bool = False


@dataclass
class Transform:
    """单个变换步骤。

    支持类型：
    - to_float|to_int|to_str|to_bool
    - scale: 乘以 factor（先转 float）
    - eval_literal: 使用 Python literal_eval 解析字符串为基本类型/列表
    - pick: 从序列取索引 index
    """

    type: str
    factor: Optional[float] = None
    index: Optional[int] = None


@dataclass
class MappingRule:
    """从 JSON 选择/生成一个值，写入 YAML 的 target 路径。"""

    target: str
    value: Optional[Any] = None
    select: Optional[JsonSelector] = None
    transforms: Optional[List[Transform]] = None


# ============================ 基础工具函数 ============================

def _read_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _write_yaml(path: Path, data: Dict[str, Any]) -> None:
    # 说明：为避免依赖 ruamel.yaml 保留注释，这里使用标准 safe_dump；
    #       若需要完整保留注释/样式，可后续切换为 ruamel.yaml。
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


def _read_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _set_by_path(root: Dict[str, Any], path: str, value: Any) -> None:
    parts = [p for p in path.split(".") if p]
    if not parts:
        raise ValueError("empty target path")
    *parents, leaf = parts
    cur = root
    for key in parents:
        if key not in cur or not isinstance(cur[key], dict):
            cur[key] = {}
        cur = cur[key]  # type: ignore[assignment]
    cur[leaf] = value


def _literal_eval_safe(s: str) -> Any:
    # 轻量 literal_eval，避免引入 ast.literal_eval 的复杂错误处理
    import ast

    try:
        return ast.literal_eval(s)
    except Exception:
        return s


# ============================ JSON 选择与变换 ============================

def _iter_blocks(model_params: Dict[str, Any]) -> Iterable[Dict[str, Any]]:
    blocks = model_params.get("blocks", [])
    for b in blocks:
        if isinstance(b, dict):
            yield b


def _match_block(b: Dict[str, Any], sel: JsonSelector) -> bool:
    # path 精确/正则匹配优先
    if sel.path:
        bp = str(b.get("Path", ""))
        if sel.regex:
            import re

            if not re.search(sel.path, bp):
                return False
        else:
            if bp != sel.path:
                return False
    # 类型/掩模过滤
    if sel.block_type and str(b.get("BlockType", "")) != sel.block_type:
        return False
    if sel.mask_type and str(b.get("MaskType", "")) != sel.mask_type:
        return False
    return True


def _select_from_json(model_params: Dict[str, Any], sel: JsonSelector) -> Any:
    if sel is None:
        return None
    chosen: Optional[Dict[str, Any]] = None
    for b in _iter_blocks(model_params):
        if _match_block(b, sel):
            chosen = b
            break
    if chosen is None:
        raise KeyError(f"no block matched: {sel}")
    if not sel.param:
        # 未指定参数时，返回整块
        return chosen
    dp = chosen.get("DialogParams", {}) or {}
    if sel.param not in dp:
        raise KeyError(f"param not found: {sel.param} in block {chosen.get('Path')} ")
    return dp[sel.param]


def _apply_transforms(value: Any, transforms: Optional[List[Transform]]) -> Any:
    if not transforms:
        return value
    v = value
    for t in transforms:
        ttype = t.type.lower()
        if ttype == "to_float":
            v = float(v)
        elif ttype == "to_int":
            v = int(float(v))
        elif ttype == "to_str":
            v = str(v)
        elif ttype == "to_bool":
            s = str(v).strip().lower()
            v = s in {"1", "true", "on", "yes"}
        elif ttype == "scale":
            factor = float(t.factor or 1.0)
            v = float(v) * factor
        elif ttype == "eval_literal":
            v = _literal_eval_safe(str(v))
        elif ttype == "pick":
            idx = int(t.index or 0)
            v = list(v)[idx]
        else:
            raise ValueError(f"unknown transform: {t.type}")
    return v


# ============================ 自动发现与规则生成 ============================

def _normalize_key_segment(text: str) -> str:
    """将块路径或参数名规范化为 YAML 键段。

    设计动机：
    - MATLAB 路径包含空格、换行与符号，不宜直接作为 YAML 键
    - 将其转为小写、用下划线连接，去除不安全字符，保证可复用与可搜索
    """
    s = str(text).strip().lower()
    # 将常见分隔符转为下划线
    s = s.replace("\\n", " ")
    s = re.sub(r"[\s/\\]+", "_", s)
    # 去除非字母数字与下划线
    s = re.sub(r"[^a-z0-9_]+", "", s)
    # 去除首尾下划线与重复下划线
    s = re.sub(r"_+", "_", s).strip("_")
    return s or "key"


def _normalize_block_path(path_str: str) -> str:
    parts = [p for p in re.split(r"[/\\]", str(path_str)) if p]
    norm = ".".join(_normalize_key_segment(p) for p in parts)
    return norm or "block"


def _infer_transforms_for_value(v: Any) -> List[Dict[str, Any]]:
    """根据值的形态给出推荐的 transforms 序列。

    策略：
    - 数值（或可转浮点的字符串）→ to_float
    - 形如列表/元组的字面量字符串 → eval_literal
    - 其他 → to_str（保持信息不丢失）
    """
    # 直接为原生数字
    if isinstance(v, (int, float)):
        return [{"type": "to_float"}]

    # 字符串：尝试判断为数字或字面量容器
    if isinstance(v, str):
        sv = v.strip()
        # 容器字面量（列表、元组、字典）
        if re.match(r"^[\[\(\{].*[\]\)\}]$", sv):
            return [{"type": "eval_literal"}]
        # 简单数字
        try:
            float(sv)
            return [{"type": "to_float"}]
        except Exception:
            return [{"type": "to_str"}]

    # 其余非常见类型统一转字符串
    return [{"type": "to_str"}]


def discover_mappings(
    model_params: Dict[str, Any],
    prefix: str = "matlab.auto_params",
    include_block_types: Optional[List[str]] = None,
    exclude_block_types: Optional[List[str]] = None,
    include_params_regex: Optional[str] = None,
    exclude_params_regex: Optional[str] = None,
) -> Dict[str, Any]:
    """从 model_params.json 自动发现参数并生成映射规则。

    说明：
    - 只依赖 JSON 的 blocks[*].{Path, BlockType, DialogParams}
    - 将每个 DialogParams 键值映射到 prefix 下，保持可追溯性
    - 不直接写值，仍以 select+transforms 的形式输出，便于审阅/覆写
    """
    inc_types = set(t.lower() for t in (include_block_types or [])) or None
    exc_types = set(t.lower() for t in (exclude_block_types or [])) or None
    inc_re = re.compile(include_params_regex) if include_params_regex else None
    exc_re = re.compile(exclude_params_regex) if exclude_params_regex else None

    rules: List[Dict[str, Any]] = []
    for b in _iter_blocks(model_params):
        block_type = str(b.get("BlockType", "")).lower()
        if inc_types and block_type not in inc_types:
            continue
        if exc_types and block_type in exc_types:
            continue

        path_str = b.get("Path", "")
        dp = b.get("DialogParams", {}) or {}
        if not isinstance(dp, dict) or not dp:
            continue

        block_key = _normalize_block_path(path_str)
        for param_name, raw_value in dp.items():
            pname = str(param_name)
            if inc_re and not inc_re.search(pname):
                continue
            if exc_re and exc_re.search(pname):
                continue

            target = f"{prefix}.{block_key}.{_normalize_key_segment(pname)}"
            transforms = _infer_transforms_for_value(raw_value)
            rule = {
                "target": target,
                "select": {"path": path_str, "param": pname},
                "transforms": transforms,
            }
            rules.append(rule)

    # 保持确定性：按 target 排序
    rules.sort(key=lambda r: r.get("target", ""))
    return {"version": 1, "description": "auto discovered mappings", "mappings": rules}

# ============================ default.yaml 生成器 ============================

def _get_nested(d: Dict[str, Any], path: str, default: Any = None) -> Any:
    cur = d
    for part in [p for p in path.split(".") if p]:
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur





def generate_default_v2(
    *,
    model_path: str,
    sim_step: float | None,
    stop_time: float | None,
    auto_params_file: str,
    auto_tree_root_name: str | None = None,
    auto_tree: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    """生成 v2 精简版 default.yaml：去掉 manual，使用 auto+init+action。

    新格式要点：
    - action_space 使用 key= "<block>.<param>"（可直接从 auto_params 粘贴）
    - experiments 仅提供 cases 示例，便于用户批量整段仿真
    """
    manual = {}
    # 允许从 auto 中推断 sim.step（若启发式可用）
    if sim_step is None and auto_tree is not None:
        ts = _get_nested(auto_tree, "powergui.sampletime")
        if isinstance(ts, (int, float)):
            sim_step = float(ts)
    # 从 auto_tree 中挑选两个示例键（尽量选择可转为 float 的参数）
    demo_keys: List[str] = []
    demo_vals: List[float] = []
    if isinstance(auto_tree, dict):
        for bname, params in auto_tree.items():
            if not isinstance(params, dict):
                continue
            for pname, pval in params.items():
                try:
                    v = float(str(pval).strip())
                except Exception:
                    continue
                demo_keys.append(f"{bname}.{pname}")
                demo_vals.append(v)
                if len(demo_keys) >= 2:
                    break
            if len(demo_keys) >= 2:
                break

    # 兜底示例
    if not demo_keys:
        demo_keys = ["series_rlc_branch1.resistance", "dc_voltage_source.amplitude"]
        demo_vals = [0.03, 1000.0]

    cfg: Dict[str, Any] = {
        "version": 2,
        "model": {"path": model_path},
        "sim": {
            "step": float(sim_step) if sim_step is not None else 1.0e-6,
            "stop_time": float(stop_time) if stop_time is not None else 0.1,
            "mode": {"start_in_accelerator": True, "external_mode": False},
        },
        "matlab": {"output_map": {"tout": "tout"}},
        "parameters": {"auto": {"file": auto_params_file}},
        # 新格式：key = block.param，便于从 auto_params 直接复制
        "action_space": [
            {"key": demo_keys[0], "dtype": "float", "bounds": {"min": 0.0, "max": max(1.0, abs(float(demo_vals[0])) * 10)}},
            {"key": demo_keys[1], "dtype": "float", "bounds": {"min": 0.0, "max": max(1.0, abs(float(demo_vals[1])) * 2)}},
        ],
        "metrics": {
            "evaluation_window": "by_step",
            "definitions": [
                {"name": "voltage_tracking", "expr": "abs(V_out - V_ref)", "aggregator": "mean", "goal": "minimize", "weight": 0.5},
                {"name": "waveform_thd", "expr": "thd(V_out, fs=10000, f0=50)", "aggregator": "last", "goal": "minimize", "weight": 0.5},
            ],
            "constraints": [
                {"name": "thd_limit", "expr": "thd(V_out, fs=10000, f0=50) <= 0.05"},
                {"name": "saturation_guard", "expr": "V_out <= 400"},
            ],
        },
        "presets": {"init_action": {}},
        "experiments": {
            "cases": [
                {"name": f"case_{demo_keys[0].replace('.', '_')}_base", "action": {demo_keys[0]: float(demo_vals[0])}},
                {"name": f"case_{demo_keys[0].replace('.', '_')}_x2",   "action": {demo_keys[0]: float(demo_vals[0]) * 2}},
            ]
        },
    }
    return cfg


def _detect_model_root_name(model_params: Dict[str, Any]) -> Optional[str]:
    """从 blocks[*].Path 推断模型根名称（first segment）。"""
    for b in _iter_blocks(model_params):
        p = str(b.get("Path", ""))
        if p:
            root = p.split("/", 1)[0]
            return _normalize_key_segment(root) or root
    return None


# ============================ 直接导出参数树（无映射） ============================

def export_params_tree(model_params: Dict[str, Any]) -> Dict[str, Any]:
    """从 model_params.json 直接导出参数树：{root_name: {block_path: {param: value}}}。

    - 不使用映射规则；
    - 保留 JSON 中的原始值（字符串/数字均可），由运行时进行解析；
    - 路径规范：去除根模型段，仅保留子路径，并进行 normalize（空格->下划线，/->.）。
    """
    root = _detect_model_root_name(model_params) or "model"
    tree: Dict[str, Any] = {root: {}}
    for b in _iter_blocks(model_params):
        path_str = str(b.get("Path", ""))
        if not path_str:
            continue
        # 去掉根段
        if path_str.startswith(root + "/"):
            sub = path_str[len(root) + 1 :]
        else:
            sub = path_str
        sub_key = _normalize_block_path(sub)
        if not sub_key:
            continue
        dp = b.get("DialogParams", {}) or {}
        if not isinstance(dp, dict):
            continue
        block_dict = tree[root].setdefault(sub_key, {})
        for pname, pval in dp.items():
            block_dict[_normalize_key_segment(pname)] = pval
    return tree

# ============================ 核心流程 ============================

def apply_mapping(
    yaml_cfg: Dict[str, Any],
    model_params: Dict[str, Any],
    mapping_cfg: Dict[str, Any],
) -> Dict[str, Any]:
    """根据映射配置将 JSON 值写入 YAML 配置。

    规则格式（示例）：
    mappings:
      - target: matlab.extra_params.Ts
        select: { path: "untitled1/powergui", param: "Ts" }
        transforms:
          - { type: to_float }
          - { type: scale, factor: 1.0 }
      - target: matlab.stop_time
        value: 0.1
    """

    rules_raw = mapping_cfg.get("mappings", []) or []
    for i, r in enumerate(rules_raw, start=1):
        try:
            target = str(r["target"]).strip()
            if not target:
                raise ValueError("target is empty")

            if "value" in r and r["value"] is not None:
                val = r["value"]
            else:
                sraw = r.get("select", {}) or {}
                sel = JsonSelector(
                    path=sraw.get("path"),
                    block_type=sraw.get("block_type"),
                    mask_type=sraw.get("mask_type"),
                    param=sraw.get("param"),
                    regex=bool(sraw.get("regex", False)),
                )
                val = _select_from_json(model_params, sel)

            tlist = []
            for t in (r.get("transforms") or []):
                tlist.append(Transform(
                    type=t.get("type"),
                    factor=t.get("factor"),
                    index=t.get("index"),
                ))
            val2 = _apply_transforms(val, tlist)

            _set_by_path(yaml_cfg, target, val2)
            log.info("mapping.applied", idx=i, target=target, value=str(val2))
        except Exception as e:
            log.warning("mapping.skip", idx=i, reason=str(e))
    return yaml_cfg


def _default_paths() -> Dict[str, Path]:
    """推断默认路径，便于“一键运行”。

    规则：以当前文件两级父目录作为仓库根，回退到 CWD。
    """
    here = Path(__file__).resolve()
    repo_root = here.parents[2] if len(here.parents) >= 3 else Path.cwd()
    d = {
        "config": repo_root / "config" / "default.yaml",
        "json": repo_root / "simulink" / "export_model_graph" / "model_params.json",
        "auto": repo_root / "config" / "auto_params.yaml",
    }
    return d


def main(argv: Optional[List[str]] = None) -> int:
    """命令行入口：映射文件应用或自动发现参数并生成规则/直接写值。

    模式：
    - 传统映射：--mapping 映射规则文件 → 写入 --config
    - 自动发现：--discover 可配合 --emit-mapping 与 --apply-discovered
    """
    p = argparse.ArgumentParser(description="Export full parameter tree and optionally generate default.yaml")
    defaults = _default_paths()
    p.add_argument("--config", default=str(defaults["config"]), help="default.yaml path for generation")
    p.add_argument("--json", default=str(defaults["json"]), help="model_params.json path")
    p.add_argument("--out", default=None, help="auto_params.yaml output path (parameter tree)")

    # 可选：生成 default.yaml
    p.add_argument("--gen-default", action="store_true", help="generate minimal v2 default.yaml from auto_params")
    p.add_argument("--model-path", dest="model_path_opt", default=None, help="SLX model path for default.yaml (e.g. simulink/untitled3.slx)")
    p.add_argument("--stop-time", dest="stop_time_opt", type=float, default=None, help="stop time for default.yaml")
    p.add_argument("--auto", dest="auto_path_opt", default=str(defaults["auto"]), help="auto_params.yaml path for default generation")

    args = p.parse_args(argv)

    cfg_path = Path(args.config)
    json_path = Path(args.json)


    # 导出参数树（需要 JSON）
    if args.out:
        if not json_path.exists():
            log.error("export.input_missing", missing=str(json_path))
            return 2
        model_params = _read_json(json_path)
        tree = export_params_tree(model_params)
        _write_yaml(Path(args.out), tree)
        log.info("export.params_tree", out=args.out)

    # 生成 default.yaml（仅依赖 auto 文件）
    if args.gen_default:
        auto_path = Path(args.auto_path_opt)
        if not auto_path.is_absolute():
            auto_path = cfg_path.parent / auto_path
        if not auto_path.exists():
            log.error("gen_default.auto_missing", file=str(auto_path))
            return 2
        auto_data = _read_yaml(auto_path)
        # 自动选择根键
        if isinstance(auto_data, dict) and auto_data:
            auto_root_name = next(iter(auto_data.keys()))
            auto_tree = auto_data.get(auto_root_name, {})
        else:
            log.error("gen_default.auto_invalid", file=str(auto_path))
            return 2
        model_path_val = args.model_path_opt or f"simulink/{auto_root_name}.slx"
        cfg_v2 = generate_default_v2(
            model_path=model_path_val,
            sim_step=None,
            stop_time=args.stop_time_opt,
            auto_params_file=str(auto_path.relative_to(cfg_path.parent)),
            auto_tree_root_name=auto_root_name,
            auto_tree=auto_tree,
        )
        _write_yaml(cfg_path, cfg_v2)
        log.info("gen_default.written", path=str(cfg_path), auto_root=auto_root_name)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
# python -m inverter_ai_control.utils.config_mapper --json simulink/export_model_graph/model_params.json --out config/auto_params.yaml
# python -m inverter_ai_control.utils.config_mapper --gen-default --model-path simulink/untitled3.slx --stop-time 0.1 --auto auto_params.yaml --config config/default.yaml

