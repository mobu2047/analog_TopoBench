"""通用信号处理函数：rms、thd、统计聚合与窗口变换。

设计：
- 仅依赖 numpy；尽量保持与业务无关
- THD 采用简化实现：基于 FFT 估计基波与谐波幅值（窗口内）
"""

from __future__ import annotations

import numpy as np
from typing import Tuple, Optional


def to_1d(array_like) -> np.ndarray:
    arr = np.asarray(array_like, dtype=float).reshape(-1)
    return arr


def rms(x) -> float:
    a = to_1d(x)
    if a.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(a * a)))


def last(x) -> float:
    a = to_1d(x)
    return float(a[-1]) if a.size else float("nan")


def mean(x) -> float:
    a = to_1d(x)
    return float(np.mean(a)) if a.size else float("nan")


def compute_fft(x: np.ndarray, fs: float) -> Tuple[np.ndarray, np.ndarray]:
    n = x.size
    if n == 0:
        return np.array([]), np.array([])
    # 简单汉宁窗减少泄漏
    window = np.hanning(n)
    xw = x * window
    X = np.fft.rfft(xw)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    mag = np.abs(X) * 2.0 / np.sum(window)
    return freqs, mag


def thd(x, fs: float, f0: float, max_harmonics: int = 10) -> float:
    """基于 FFT 的简化 THD 估计。

    - fs: 采样频率
    - f0: 基波
    - max_harmonics: 估计到第 N 次谐波
    """
    a = to_1d(x)
    if a.size == 0:
        return float("nan")
    freqs, mag = compute_fft(a, fs)
    if freqs.size == 0:
        return float("nan")
    # 找到最接近各谐波频率的幅值
    def pick(freq):
        idx = (np.abs(freqs - freq)).argmin()
        return mag[idx]

    v1 = pick(f0)
    if v1 <= 1e-12:
        return float("inf")
    harm_sq = 0.0
    for k in range(2, max_harmonics + 1):
        vk = pick(k * f0)
        harm_sq += float(vk * vk)
    return float(np.sqrt(harm_sq) / v1)


def power_factor(x, fs: float, f0: float) -> float:
    """计算功率因数（简化版本，假设电压电流同相）"""
    a = to_1d(x)
    if a.size == 0:
        return float("nan")
    # 简化的功率因数计算，实际需要电压和电流信号
    # 这里假设为理想情况
    return 1.0


def efficiency(p_out, p_in) -> float:
    """计算效率"""
    p_out_val = float(p_out) if np.isfinite(p_out) else 0.0
    p_in_val = float(p_in) if np.isfinite(p_in) else 0.0
    if p_in_val <= 0:
        return 0.0
    return p_out_val / p_in_val


def settling_time(x, target, tolerance=0.02) -> float:
    """计算稳定时间（达到目标值±容差范围内的时间）"""
    a = to_1d(x)
    if a.size == 0:
        return float("nan")
    
    target_val = float(target)
    tol = float(tolerance)
    upper_bound = target_val * (1 + tol)
    lower_bound = target_val * (1 - tol)
    
    # 找到最后一个超出容差范围的点
    in_range = (a >= lower_bound) & (a <= upper_bound)
    if not np.any(in_range):
        return float("inf")
    
    # 找到最后一个超出范围的点
    last_out_of_range = np.where(~in_range)[0]
    if len(last_out_of_range) == 0:
        return 0.0
    
    return float(last_out_of_range[-1] / len(a))


def overshoot(x, target) -> float:
    """计算超调量（百分比）"""
    a = to_1d(x)
    if a.size == 0:
        return float("nan")
    
    target_val = float(target)
    max_val = np.max(a)
    
    if target_val == 0:
        return float("inf") if max_val > 0 else 0.0
    
    return float((max_val - target_val) / abs(target_val) * 100)


def steady_state_error(x, target) -> float:
    """计算稳态误差"""
    a = to_1d(x)
    if a.size == 0:
        return float("nan")
    
    # 取最后10%的数据作为稳态
    steady_start = int(len(a) * 0.9)
    steady_data = a[steady_start:]
    
    if len(steady_data) == 0:
        return float("nan")
    
    target_val = float(target)
    steady_mean = np.mean(steady_data)
    
    return float(steady_mean - target_val)


def harmonic_distortion(x, fs: float, f0: float, harmonic_order: int) -> float:
    """计算特定谐波失真"""
    a = to_1d(x)
    if a.size == 0:
        return float("nan")
    
    freqs, mag = compute_fft(a, fs)
    if freqs.size == 0:
        return float("nan")
    
    # 找到基波和谐波频率
    def pick(freq):
        idx = (np.abs(freqs - freq)).argmin()
        return mag[idx]
    
    v1 = pick(f0)
    vh = pick(harmonic_order * f0)
    
    if v1 <= 1e-12:
        return float("inf")
    
    return float(vh / v1)


def crest_factor(x) -> float:
    """计算峰值因子（峰值/有效值）"""
    a = to_1d(x)
    if a.size == 0:
        return float("nan")
    
    peak = np.max(np.abs(a))
    rms_val = rms(a)
    
    if rms_val <= 1e-12:
        return float("inf")
    
    return float(peak / rms_val)


def form_factor(x) -> float:
    """计算波形因子（有效值/平均值）"""
    a = to_1d(x)
    if a.size == 0:
        return float("nan")
    
    rms_val = rms(a)
    mean_val = np.mean(np.abs(a))
    
    if mean_val <= 1e-12:
        return float("inf")
    
    return float(rms_val / mean_val)


