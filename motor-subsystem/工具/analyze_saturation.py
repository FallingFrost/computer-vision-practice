"""练习 4（饱和）本地预检 —— 只用标准库。

目标：**找出响应在哪一档开始弯**，也就是练习 5（增益）可以用来做线性拟合的区间。
练习 3 的 40–60 段可以直接并进来，因为练习 4 的 ladder 从 60 起步，两者有一档重叠。

本机 pandas 因 NumPy 2.x ABI 不匹配而不可用，所以这里只用 csv + statistics。
"""

import csv
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
MM_PER_COUNT = 0.280578  # π × 32 / 358.3


def load(path):
    """读一份测量 CSV，把速度统一到 settled_speed_cps。

    练习 3 的速度列叫 speed_cps，练习 4 叫 settled_speed_cps —— 差一个词。
    这里两种都认，否则一合并就 KeyError。
    """
    with path.open(newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        return []
    speed_key = None
    for candidate in ("settled_speed_cps", "speed_cps"):
        if candidate in rows[0]:
            speed_key = candidate
            break
    if speed_key is None:
        raise KeyError(f"{path.name} 里找不到速度列：{list(rows[0].keys())}")

    for r in rows:
        r["PWM"] = int(r["PWM"])
        r["trial_num"] = int(r["trial_num"])
        r["settled_speed_cps"] = float(r[speed_key])
        r["time_ms"] = float(r["time_ms"])
    return rows


def per_pwm(rows):
    """按 PWM 汇总：均值、标准差、以及每轮(trial)的均值。"""
    buckets = defaultdict(list)
    for r in rows:
        buckets[r["PWM"]].append(r)

    out = {}
    for pwm, rs in buckets.items():
        speeds = [r["settled_speed_cps"] for r in rs]
        by_trial = defaultdict(list)
        for r in rs:
            by_trial[r["trial_num"]].append(r["settled_speed_cps"])
        out[pwm] = {
            "n": len(rs),
            "mean": statistics.mean(speeds),
            "sd": statistics.stdev(speeds) if len(speeds) > 1 else 0.0,
            "trial_means": [statistics.mean(v) for v in by_trial.values()],
            "times": [r["time_ms"] for r in rs],
        }
    return out


def main():
    ex4 = load(DATA / "motor_exercise04_saturation.csv")
    print(f"练习 4 : {len(ex4)} 行")
    print(f"档位   : {sorted(set(r['PWM'] for r in ex4))}")
    print(f"PWM 列取值 : {sorted(set(r['PWM'] for r in ex4))}")
    print(f"时间跨度 : {(max(r['time_ms'] for r in ex4) - min(r['time_ms'] for r in ex4))/1000:.1f} 秒")
    print()

    s4 = per_pwm(ex4)

    # 练习 3 的 40/45/50/55/60 档，用来和练习 4 的 60 档做跨练习交叉验证
    print("--- 练习 3 vs 练习 4 在重叠档位上的对比 ---")
    print("练习 3 从静止起步，所以整轮均值被启动瞬态拉低；练习 4 有 settle，全程都是稳态。")
    print(f"{'PWM':>4} {'练习3整轮均值':>13} {'练习3后段(11-20)':>17} {'练习4':>9} "
          f"{'整轮偏低估':>11} {'后段吻合度':>11}")
    try:
        ex3 = load(DATA / "motor_exercise03_left_fwd.csv")
        s3 = per_pwm(ex3)
        s3_late = per_pwm([r for r in ex3 if int(r["sample_num"]) >= 11])
        for pwm in (40, 45, 50, 55, 60):
            if pwm in s3 and pwm in s4:
                full = s3[pwm]["mean"]
                late = s3_late[pwm]["mean"]
                ref = s4[pwm]["mean"]
                print(f"{pwm:>4} {full:13.1f} {late:17.1f} {ref:9.1f} "
                      f"{(full-late)/late*100:10.1f}% {abs(late-ref)/ref*100:10.2f}%")
    except FileNotFoundError:
        print("  (找不到练习 3 的文件，跳过)")
    print()

    # 相邻档位的斜率 —— 饱和的证据
    pwms = sorted(s4)
    print("--- 相邻档位斜率（Δ速度 / ΔPWM）---")
    print(f"{'PWM':>4} {'均值cps':>9} {'mm/s':>7} {'sd':>7} {'斜率 cps/PWM':>13} {'相对首段':>9}")
    first_slope = None
    slopes = {}
    for i, pwm in enumerate(pwms):
        m = s4[pwm]
        if i == 0:
            slope_txt = "     -"
            ratio_txt = "    -"
        else:
            prev = s4[pwms[i - 1]]
            dpwm = pwm - pwms[i - 1]
            slope = (m["mean"] - prev["mean"]) / dpwm
            slopes[pwm] = slope
            if first_slope is None:
                first_slope = slope
            slope_txt = f"{slope:13.1f}"
            ratio_txt = f"{slope/first_slope*100:8.0f}%"
        print(f"{pwm:>4} {m['mean']:9.1f} {m['mean']*MM_PER_COUNT:7.1f} "
              f"{m['sd']:7.1f} {slope_txt} {ratio_txt}")
    print()

    # 斜率掉到首段一半的位置 —— 粗略的"拐点"
    if first_slope:
        knee = None
        for pwm in pwms[1:]:
            if slopes[pwm] < first_slope * 0.5:
                knee = pwm
                break
        print("--- 线性区间估计 ---")
        print(f"首段斜率（PWM {pwms[0]}→{pwms[1]}）: {first_slope:.1f} cps/PWM")
        if knee:
            print(f"斜率首次跌破首段一半的档位       : PWM {knee}")
            print(f"→ 建议的线性拟合区间上界约在 PWM {knee} 之前")
        else:
            print("斜率始终没跌破首段一半 → 在测过的范围内饱和不明显")
    print()

    # 用 40-60 的旧斜率做线性外推，看实测掉了多少
    print("--- 对照：练习 3 的线性外推 vs 本次实测 ---")
    ref_pwm, ref_speed = 60, s4.get(60, {}).get("mean")
    ref_slope = 14.2  # 练习 3 测到的 cps per PWM
    if ref_speed:
        print(f"{'PWM':>4} {'实测':>9} {'线性外推':>10} {'实测/外推':>10}")
        for pwm in pwms:
            if pwm < 60:
                continue
            predicted = ref_speed + (pwm - ref_pwm) * ref_slope
            m = s4[pwm]["mean"]
            print(f"{pwm:>4} {m:9.1f} {predicted:10.1f} {m/predicted*100:9.0f}%")
    print()

    # ---- 练习 5 预览：在候选线性区间上做最小二乘拟合 ----
    fit_pwms = [p for p in pwms if 60 <= p <= 170 and p not in (100, 160)]
    xs = [float(p) for p in fit_pwms]
    ys = [s4[p]["mean"] for p in fit_pwms]
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    G = sxy / sxx
    c = my - G * mx

    ss_tot = sum((y - my) ** 2 for y in ys)
    ss_res = sum((y - (G * x + c)) ** 2 for x, y in zip(xs, ys))
    r2 = 1 - ss_res / ss_tot

    print("--- 练习 5 预览：候选区间 60–170（已排除留出档位 100 / 160）---")
    print(f"参与拟合的档位 : {fit_pwms}")
    print(f"G (斜率)       : {G:.3f} counts/s per PWM  =  {G*MM_PER_COUNT:.3f} mm/s per PWM")
    print(f"c (截距)       : {c:+.3f} counts/s  =  {c*MM_PER_COUNT:+.3f} mm/s")
    print(f"R^2            : {r2:.6f}")
    print(f"直线过零点     : PWM {(-c/G):.2f}  ← 不是死区，直线在死区那段本来就无效")
    print()
    print(f"{'PWM':>4} {'实测':>9} {'预测':>9} {'残余':>8} {'相对':>7}")
    for pwm in pwms:
        m = s4[pwm]["mean"]
        pred = G * pwm + c
        mark = "  <- 留出，未参与拟合" if pwm in (100, 160) else ""
        print(f"{pwm:>4} {m:9.1f} {pred:9.1f} {m-pred:+8.1f} {(m-pred)/pred*100:+6.1f}%{mark}")
    print()
    print("线性区间内残余若都在 ±2% 内且无系统性走向，直线模型就站得住。")
    print()

    # 漂移检查：每个 PWM 的第 1 轮 vs 第 5 轮
    print("--- 漂移检查：每档第 1 轮 vs 第 5 轮均值 ---")
    print(f"{'PWM':>4} {'第1轮':>9} {'第5轮':>9} {'差':>8}")
    for pwm in pwms:
        tm = s4[pwm]["trial_means"]
        if len(tm) >= 2:
            print(f"{pwm:>4} {tm[0]:9.1f} {tm[-1]:9.1f} {(tm[-1]-tm[0])/tm[0]*100:7.1f}%")


if __name__ == "__main__":
    main()
