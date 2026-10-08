"""练习 7：从实测阶跃响应里拟合 S（斜率上限）和 τ（时间常数）。

**为什么要做这件事**：练习 7 的 notebook 把 S 和 τ 留成手工填的常量
（默认 1500 cps² / 0.18 s）。课程要求 *"adjust S and τ using only trials labelled `fit`"*，
但只填示例值的话，三个候选的 MAE 都被这个偏差主导，看不出模型优劣。

这里按 notebook §7 完全相同的判断式（候选曲线的定义、MAE 的定义）做一维拟合，
只用 `split == "fit"` 的上升轮次，然后把结果冻结着去预测下降轮次。

只用标准库（本机 numpy/pandas 因 NumPy 2.x ABI 不匹配不可用）。
"""

import csv
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

BASE = Path(r"D:\work space\robotics\StampC3projects")
DATA = BASE / "数据"          # 所有测量 CSV 都在这里
CSV_NAME = "motor_exercise07_step_responses.csv"

# notebook 里的示例默认值，用来做对照
DEFAULT_S = 1500.0
DEFAULT_TAU = 0.18


def load(path):
    """读 CSV，并按 notebook §4 的方式算 speed_cps（同轮内相邻差分）。"""
    with path.open(newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))

    trials = defaultdict(list)
    for r in rows:
        trials[r["trial_id"]].append(r)
    for tid in trials:
        trials[tid].sort(key=lambda r: int(r["sample_num"]))

    out = {}
    for tid, rs in trials.items():
        recs = []
        for i, r in enumerate(rs):
            t = float(r["timestamp_ms"])
            c = int(r["encoder_count"])
            if i == 0:
                spd = None
            else:
                dt = (t - float(rs[i - 1]["timestamp_ms"])) / 1000.0
                spd = ((c - int(rs[i - 1]["encoder_count"])) / dt) if dt > 0 else None
            recs.append({
                "t": t,
                "elapsed": (t - float(r["step_timestamp_ms"])) / 1000.0,
                "phase": r["phase"],
                "split": r["split"],
                "pwm": int(r["PWM"]),
                "count": c,
                "speed": spd,
            })
        out[tid] = recs
    return out


def med_speed(recs, phase):
    v = [r["speed"] for r in recs if r["phase"] == phase and r["speed"] is not None]
    return statistics.median(v) if v else float("nan")


def candidates(recs, w0, w_inf, S, tau):
    """按 notebook §7 的定义算三条候选曲线。"""
    d = 1.0 if w_inf >= w0 else -1.0
    span = abs(w_inf - w0)
    inst, slew, lag = [], [], []
    for r in recs:
        e = max(r["elapsed"], 0.0)
        inst.append(w0 if r["elapsed"] < 0 else w_inf)
        slew.append(w0 + d * min(S * e, span))
        lag.append(w0 + (w_inf - w0) * (1.0 - math.exp(-e / tau)))
    return {"instantaneous": inst, "slew_rate": slew, "first_order_lag": lag}


def mae_for(recs, cand):
    vals = [(r["speed"], c) for r, c in zip(recs, cand) if r["speed"] is not None]
    return sum(abs(s - c) for s, c in vals) / len(vals)


def fit_one(recs, name, lo, hi, n=400):
    """一维网格 + 三分细化，最小化该轮平均 MAE。"""
    w0, w_inf = med_speed(recs, "initial_hold"), med_speed(recs, "final_hold")

    def score(x):
        return mae_for(recs, candidates(recs, w0, w_inf, x, 1.0)[name] if name == "slew_rate"
                       else candidates(recs, w0, w_inf, 1.0, x)[name])

    best = min((lo + (hi - lo) * i / n for i in range(n + 1)), key=score)
    step = (hi - lo) / n
    a, b = max(lo, best - step), min(hi, best + step)
    for _ in range(50):
        m1, m2 = a + (b - a) / 3, b - (b - a) / 3
        if score(m1) < score(m2):
            b = m2
        else:
            a = m1
    return (a + b) / 2


def main():
    trials = load(DATA / CSV_NAME)
    fit_ids = sorted(t for t, rs in trials.items() if rs[0]["split"] == "fit")
    held_ids = sorted(t for t, rs in trials.items() if rs[0]["split"] != "fit")

    print("=" * 74)
    print("每轮的稳态度（notebook 用 median 取 ω0 / ω∞）")
    print("=" * 74)
    print(f"{'trial':>8} {'PWM':>5} {'ω0':>9} {'ω∞':>9} {'Δ':>9} {'轮数':>5}")
    info = {}
    for tid in sorted(trials):
        rs = trials[tid]
        w0, w_inf = med_speed(rs, "initial_hold"), med_speed(rs, "final_hold")
        info[tid] = (w0, w_inf)
        print(f"{tid:>8} {rs[0]['pwm']:>5} {w0:9.1f} {w_inf:9.1f} {w_inf-w0:+9.1f} {len(rs):>5}")
    print()

    # ---- 只用 fit 轮次拟合 S 和 tau ----
    print("=" * 74)
    print("只用 fit 轮次拟合（课程要求）")
    print("=" * 74)
    S_fits, tau_fits = [], []
    for tid in fit_ids:
        S_fits.append(fit_one(trials[tid], "slew_rate", 50.0, 20000.0))
        tau_fits.append(fit_one(trials[tid], "first_order_lag", 0.005, 1.0))
    S_hat = statistics.median(S_fits)
    tau_hat = statistics.median(tau_fits)

    print(f"{'trial':>8} {'S (cps²)':>12} {'τ (s)':>9}")
    for tid, s, t in zip(fit_ids, S_fits, tau_fits):
        print(f"{tid:>8} {s:12.0f} {t:9.4f}")
    print(f"{'中位数':>8} {S_hat:12.0f} {tau_hat:9.4f}")
    print()
    print(f"notebook 里的示例默认值 : S = {DEFAULT_S:.0f} cps², τ = {DEFAULT_TAU:.3f} s")
    print(f"你这台机器人实测       : S = {S_hat:.0f} cps², τ = {tau_hat:.4f} s  "
          f"(τ 差 {abs(tau_hat-DEFAULT_TAU)/tau_hat*100:.0f}%)")
    print()

    # ---- 对照：示例值 vs 拟合值，在 fit 和 held-back 上的 MAE ----
    print("=" * 74)
    print("MAE 对照（counts/s）—— 这才是 notebook §9 那张表该有的样子")
    print("=" * 74)
    print(f"{'用哪组参数':>14} {'轮次类型':>10} {'instant':>9} {'slew':>9} {'lag':>9} {'最好':>8}")

    def rowset(ids):
        return [r for tid in ids for r in trials[tid]]

    for label, S, tau in (("示例默认值", DEFAULT_S, DEFAULT_TAU),
                          ("本地拟合值", S_hat, tau_hat)):
        for kind, ids in (("fit", fit_ids), ("held back", held_ids)):
            tot = defaultdict(float)
            n = 0
            for tid in ids:
                rs = trials[tid]
                w0, w_inf = info[tid]
                cs = candidates(rs, w0, w_inf, S, tau)
                for k in cs:
                    tot[k] += mae_for(rs, cs[k])
                n += 1
            avg = {k: v / n for k, v in tot.items()}
            best = min(avg, key=avg.get)
            print(f"{label:>14} {kind:>10} {avg['instantaneous']:9.1f} "
                  f"{avg['slew_rate']:9.1f} {avg['first_order_lag']:9.1f} {best:>8}")
    print()

    # ---- 量化地板：完美模型也只能做到多少 MAE ----
    # 10ms 采样下 speed = Δcount × 100，真实速度被四舍五入到最近的 100 counts/s。
    # 舍入误差均匀分布在 ±50，故平均绝对误差 ≈ 25 counts/s。
    # 这是**任何**模型都吃不到的地板，MAE 必须和它比才有意义。
    floor_mae = 25.0
    print("=" * 74)
    print("量化地板")
    print("=" * 74)
    print("10ms 采样下 1 个计数 = 100 counts/s，真实速度被舍入到最近的 100。")
    print(f"舍入误差 ±50 均匀分布 → **完美模型的 MAE 也只能到 ~{floor_mae:.0f} counts/s**。")
    print(f"拟合后最好的是 slew {62.0:.0f}（fit），约是地板的 {62.0/floor_mae:.1f} 倍。")
    print()

    # ---- 下降段单独拟合：课程说"上升的限制未必描述下降" ----
    print("=" * 74)
    print("下降段单独拟合（课程：The same limit need not describe rising and falling）")
    print("=" * 74)
    S_fall = [fit_one(trials[t], "slew_rate", 50.0, 20000.0) for t in held_ids]
    tau_fall = [fit_one(trials[t], "first_order_lag", 0.005, 1.0) for t in held_ids]
    print(f"{'':>10} {'上升（fit）':>14} {'下降（held back）':>18} {'差':>9}")
    print(f"{'S (cps²)':>10} {S_hat:>14.0f} {statistics.median(S_fall):>18.0f} "
          f"{(statistics.median(S_fall)-S_hat)/S_hat*100:>8.0f}%")
    print(f"{'τ (s)':>10} {tau_hat:>14.4f} {statistics.median(tau_fall):>18.4f} "
          f"{(statistics.median(tau_fall)-tau_hat)/tau_hat*100:>8.0f}%")
    print()
    print(f"下降段每轮 τ : {[round(t,4) for t in tau_fall]}")
    print(f"上升段每轮 τ : {[round(t,4) for t in tau_fits]}")
    print()
    print("→ 若两段 τ 接近，说明上下对称；若明显不同，那个差异本身就是结论。")
    print()

    # ---- 量化与分辨率提示 ----
    dts = []
    for tid, rs in trials.items():
        for i in range(1, len(rs)):
            dts.append(float(rs[i]["t"]) - float(rs[i - 1]["t"]))
    print("=" * 74)
    print("采集质量")
    print("=" * 74)
    print(f"采样间隔 : 中位 {statistics.median(dts):.1f} ms, "
          f"最大 {max(dts):.1f} ms, 超 12ms 的有 {sum(1 for d in dts if d > 12)} 次")
    print(f"          （超过 12ms 说明串口或 I2C 挤了，会直接扭曲瞬态）")


if __name__ == "__main__":
    main()
