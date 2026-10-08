"""练习 5（Gain）本地预检 —— 只用标准库。

读 `build_gain_dataset.py` 装配好的 `motor_exercise05_gain.csv`，做 notebook §6–8 同样的事：

  1. 按 PWM 汇总（均值 + 重复标准差）
  2. 在拟合区间内做最小二乘拟合 ω̂ = G·u + c（**排除留出档位**）
  3. 算每个档位的预测与残余，看残余图该长什么样
  4. **留出点检验**：拿冻结的 G、c 预测 PWM 100 / 160，
     和 testGain 重新测的值比，并把误差和重复标准差比

本机 pandas 因 NumPy 2.x ABI 不匹配而不可用，所以只用标准库。
"""

import csv
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
GAIN_FILE = "motor_exercise05_gain.csv"
EX4_FILE = "motor_exercise04_saturation.csv"

FIT_PWM_MIN = 40
FIT_PWM_MAX = 170
HELD_BACK = [100, 160]

FIT_LO, FIT_HI = 40, 170


def read(path, speed_keys=("settled_speed_cps", "speed_cps")):
    with path.open(newline="", encoding="utf-8") as fh:
        rdr = csv.DictReader(fh)
        key = next((k for k in speed_keys if k in (rdr.fieldnames or [])), None)
        if key is None:
            raise SystemExit(f"{path.name} 找不到速度列：{rdr.fieldnames}")
        return [{
            "PWM": int(r["PWM"]),
            "trial_num": int(r["trial_num"]),
            "sample_num": int(r.get("sample_num", 0)),
            "speed": float(r[key]),
            "source": r.get("source", path.stem),
        } for r in rdr]


def summarise(rows):
    """按 PWM 汇总：均值、标准差、以及每轮的均值。"""
    buckets = defaultdict(list)
    for r in rows:
        buckets[r["PWM"]].append(r)
    out = {}
    for pwm, rs in buckets.items():
        speeds = [r["speed"] for r in rs]
        by_trial = defaultdict(list)
        for r in rs:
            by_trial[r["trial_num"]].append(r["speed"])
        out[pwm] = {
            "n": len(rs),
            "mean": statistics.mean(speeds),
            "sd": statistics.stdev(speeds) if len(speeds) > 1 else 0.0,
            "trial_means": [statistics.mean(v) for v in by_trial.values()],
            "sources": sorted(set(r["source"] for r in rs)),
        }
    return out


def polyfit(xs, ys):
    """一阶最小二乘，返回 (斜率, 截距, R^2)。"""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx
    inter = my - slope * mx
    ss_tot = sum((y - my) ** 2 for y in ys)
    ss_res = sum((y - (slope * x + inter)) ** 2 for x, y in zip(xs, ys))
    return slope, inter, (1 - ss_res / ss_tot if ss_tot else float("nan"))


def main():
    rows = read(DATA / GAIN_FILE)
    s = summarise(rows)
    pwms = sorted(s)

    print(f"数据集 : {GAIN_FILE}   共 {len(rows)} 行，{len(pwms)} 个档位")
    print(f"来源   : {sorted(set(r['source'] for r in rows))}")
    print()

    # ---- 拟合：区间内、排除留出档位 ----
    fit_pwms = [p for p in pwms if FIT_LO <= p <= FIT_HI and p not in HELD_BACK]
    G, c, r2 = polyfit([float(p) for p in fit_pwms],
                       [s[p]["mean"] for p in fit_pwms])

    print("=" * 68)
    print("拟合结果  ω̂ = G·u + c")
    print("=" * 68)
    print(f"拟合区间   : PWM {FIT_LO} – {FIT_HI}")
    print(f"参与档位   : {fit_pwms}   ({len(fit_pwms)} 个)")
    print(f"留出档位   : {HELD_BACK}   ← 未参与拟合")
    print()
    print(f"G (增益)   : {G:.4f} counts/s per PWM   =  {G*0.280578:.4f} mm/s per PWM")
    print(f"c (截距)   : {c:+.2f} counts/s           =  {c*0.280578:+.2f} mm/s")
    print(f"R^2        : {r2:.6f}")
    print(f"直线过零点 : PWM {-c/G:.2f}")
    print()

    # ---- 残余表 ----
    print("=" * 68)
    print("逐档：实测 vs 预测 vs 残余")
    print("=" * 68)
    print(f"{'PWM':>4} {'来源':>12} {'实测均值':>10} {'重复SD':>8} {'预测':>10} {'残余':>9} {'相对':>8}")
    for pwm in pwms:
        m = s[pwm]
        pred = G * pwm + c
        res = m["mean"] - pred
        tag = "留出" if pwm in HELD_BACK else ""
        src = ",".join(m["sources"])
        print(f"{pwm:>4} {src:>12} {m['mean']:10.1f} {m['sd']:8.1f} {pred:10.1f} "
              f"{res:+9.1f} {res/pred*100:+7.2f}%  {tag}")
    print()

    # 区间内残余的离散程度 —— 判断"直线是否够用"的关键
    in_res = [(s[p]["mean"] - (G * p + c)) / (G * p + c) * 100
              for p in fit_pwms]
    print(f"拟合区间内残余：min {min(in_res):+.2f}%  max {max(in_res):+.2f}%  "
          f"|均值| {abs(statistics.mean(in_res)):.2f}%")
    out_res = [(s[p]["mean"] - (G * p + c)) / (G * p + c) * 100
               for p in pwms if p not in fit_pwms and p not in HELD_BACK]
    if out_res:
        print(f"拟合区间外残余：{['%+.1f%%' % v for v in sorted(out_res)]}")
        print("  ↑ 系统性变负 = 饱和；这正是课程说的「残余图揭示线性拟合在哪失效」")
    print()

    # ---- 留出点检验 ----
    print("=" * 68)
    print("留出点检验（课程第 8 步）")
    print("=" * 68)
    print("冻结 G、c，预测从未参与拟合的档位；再和 testGain 重新测的值比。")
    print()

    # 练习 4 当时在这些档位测到的值，用于"重测是否复现"
    ex4_means = {}
    try:
        ex4 = summarise(read(DATA / EX4_FILE))
        ex4_means = {p: ex4[p]["mean"] for p in HELD_BACK if p in ex4}
    except FileNotFoundError:
        pass

    print(f"{'PWM':>4} {'预测':>10} {'重测实测':>10} {'误差':>9} {'相对':>8} "
          f"{'重复SD':>8} {'误差/SD':>8}")
    for pwm in HELD_BACK:
        m = s[pwm]
        pred = G * pwm + c
        err = m["mean"] - pred
        print(f"{pwm:>4} {pred:10.1f} {m['mean']:10.1f} {err:+9.1f} "
              f"{err/pred*100:+7.2f}% {m['sd']:8.1f} {abs(err)/m['sd']:8.2f}")
    print()
    print("课程要求的就是最后一列那个比较：误差相对于重复测量离散度有多大。")
    print("误差/SD < 1 表示预测误差落在测量噪声之内。")
    print()

    if ex4_means:
        print("重测复现性（练习 4 当时 vs 本次 testGain 重测）：")
        for pwm in HELD_BACK:
            if pwm in ex4_means:
                a, b = ex4_means[pwm], s[pwm]["mean"]
                print(f"  PWM {pwm:>3}: 练习 4 {a:8.1f}  本次 {b:8.1f}  差 {abs(a-b)/a*100:5.2f}%")


if __name__ == "__main__":
    main()
