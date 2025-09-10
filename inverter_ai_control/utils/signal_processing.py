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


