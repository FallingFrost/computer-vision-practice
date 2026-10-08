"""练习 7 前置估算：用已有数据估计时间常数 τ 的量级。

练习 3 的数据实际上就是"从静止到某个 PWM"的阶跃响应，只不过是以 50ms 采样的。
虽然太粗，但足以判断 τ 是 10ms 量级还是 500ms 量级——
这决定练习 7 的 10ms 采样能不能把瞬态解析出来。

只用标准库。
"""

import csv
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

BASE = Path(r"D:\work space\robotics\StampC3projects")
DATA = BASE / "数据"          # 所有测量 CSV 都在这里
MM_PER_COUNT = 0.280578


def read(path):
    with path.open(newline="", encoding="utf-8") as fh:
        rdr = csv.DictReader(fh)
        key = next(k for k in ("settled_speed_cps", "speed_cps") if k in rdr.fieldnames)
        return [{"PWM": int(r["PWM"]), "trial": int(r["trial_num"]),
                 "sample": int(r["sample_num"]), "t": float(r["time_ms"]),
                 "speed": float(r[key])} for r in rdr]


def fit_tau(samples, w0, w_inf):
    """对 ω(t) = w0 + (w∞−w0)(1−e^{−t/τ}) 做一维最小二乘，返回 τ（秒）。

    samples: [(t_s, speed)]，t_s 是**相对阶跃时刻**的秒数。
    """
    def sse(tau):
        return sum((w - (w0 + (w_inf - w0) * (1 - math.exp(-t / tau)))) ** 2
                   for t, w in samples)

    lo, hi = 0.005, 5.0
    grid = [lo * (hi / lo) ** (i / 300.0) for i in range(301)]
    best = min(grid, key=sse)
    # 三分细化
    a, b = max(lo, best / 1.3), min(hi, best * 1.3)
    for _ in range(60):
        m1, m2 = a + (b - a) / 3, b - (b - a) / 3
        if sse(m1) < sse(m2):
            b = m2
        else:
            a = m1
    return (a + b) / 2


def main():
    rows = read(DATA / "motor_exercise03_left_fwd.csv")
    by_trial = defaultdict(list)
    for r in rows:
        by_trial[(r["PWM"], r["trial"])].append(r)

    print("用练习 3 的「从静止起步」数据估 τ（每轮开头就是一次 0 → PWM 的阶跃）")
    print("⚠️ 练习 3 是 50ms 采样，瞬态里只有几个点，所以这是**量级估计**不是精确值。")
    print()
    print(f"{'PWM':>4} {'轮':>3} {'ω∞(实测尾段)':>13} {'τ (ms)':>8} {'3τ (ms)':>8}")
    taus = []
    for pwm in (60, 55, 50):
        for trial in (1, 2, 3, 4, 5):
            s = sorted(by_trial.get((pwm, trial), []), key=lambda r: r["sample"])
            if len(s) < 15:
                continue
            t0 = s[0]["t"]
            # 尾段（后 8 个样本）均值当作 ω∞
            w_inf = statistics.mean(r["speed"] for r in s[-8:])
            if w_inf < 100:          # 这一轮没起转，跳过
                continue
            pts = [((r["t"] - t0) / 1000.0, r["speed"]) for r in s[1:14]]
            tau = fit_tau(pts, 0.0, w_inf)
            taus.append(tau)
            print(f"{pwm:>4} {trial:>3} {w_inf:13.1f} {tau*1000:8.1f} {tau*3000:8.1f}")

    print()
    if taus:
        print(f"τ 中位数 : {statistics.median(taus)*1000:.1f} ms")
        print(f"τ 范围   : {min(taus)*1000:.1f} – {max(taus)*1000:.1f} ms")
        med = statistics.median(taus)
        print()
        print("对练习 7 的含义：")
        print(f"  10ms 采样下，瞬态约跨越 {med*1000*4/10:.0f} 个样本（4τ 算）")
        print(f"  50ms 采样下只有 {med*1000*4/50:.1f} 个点 —— 这就是练习 7 必须换到 10ms 的原因")
    else:
        print("没找到可用的起转轮次。")


if __name__ == "__main__":
    main()
