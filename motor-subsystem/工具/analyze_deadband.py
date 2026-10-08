"""练习 3（死区）本地预检 —— 只用标准库。

本机的 pandas / matplotlib 因 NumPy 2.x ABI 不匹配而 import 失败，
所以这里不用它们，只用 csv + statistics，先把数据验一遍再去 Colab。

用途：
  1. 硬校验（行数、列、rest_settled）
  2. 按 PWM 档位汇总「有没有持续运动」
  3. 用 PWM=0 档实测噪声地板，给出死区区间

判定规则见 电机练习03-代码说明.md §7，是本项目自己的提案，不是课程规定。
"""

import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path

# Windows 控制台默认是 GBK，输出中文/符号会炸。强制走 UTF-8。
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

BASE = Path(r"D:\work space\robotics\StampC3projects")
DATA = BASE / "数据"          # 所有测量 CSV 都在这里
CSV_NAME = "motor_exercise03_left_fwd.csv"

# --- 判定参数（改这里就能做敏感性分析）---
THETA_SPEED_CPS = 50.0   # 多快算「在动」：50 counts/s ≈ 14 mm/s
LATE_START = 11          # 后半段从第几个样本开始（1-based）
LATE_MOVING_MIN = 5      # 后半段至少几个样本在动
NULL_MARGIN = 2.0        # 噪声地板上加的余量（计数）


def load(path):
    with path.open(newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        for k in ("PWM", "trial_num", "sample_num", "encoder_count",
                  "other_encoder_count", "rest_residual_counts", "rest_settled"):
            r[k] = int(r[k])
        for k in ("speed_cps", "speed_mm_s", "dt_ms", "time_ms"):
            r[k] = float(r[k])
    return rows


def trial_metrics(samples):
    """samples: 一轮里按 sample_num 排好序的行。返回该轮的各统计量。"""
    # 每个样本的计数增量 = speed_cps * dt_ms / 1000
    deltas = [s["speed_cps"] * s["dt_ms"] / 1000.0 for s in samples]
    c_total = sum(deltas)

    late = deltas[LATE_START - 1:]
    late_dt = sum(s["dt_ms"] for s in samples[LATE_START - 1:])
    v_late = (sum(late) / late_dt * 1000.0) if late_dt > 0 else 0.0
    n_late_moving = sum(1 for d in late if abs(d) >= 0.5)

    tail = deltas[-3:]
    v_tail = (sum(tail) / 3.0) / (samples[-1]["dt_ms"] / 1000.0)

    return {
        "c_total": c_total,
        "v_mean": c_total / (sum(s["dt_ms"] for s in samples) / 1000.0),
        "v_late": v_late,
        "n_late_moving": n_late_moving,
        "v_tail": v_tail,
        "residual": samples[0]["rest_residual_counts"],
        "settled": samples[0]["rest_settled"],
    }


def classify(m, theta_null, theta_speed):
    if abs(m["c_total"]) <= theta_null:
        return "STALL"
    if m["n_late_moving"] == 0:
        return "TWITCH"
    if abs(m["v_late"]) < theta_speed or m["n_late_moving"] < LATE_MOVING_MIN:
        return "CREEP"
    return "SUSTAINED"


def main():
    rows = load(DATA / CSV_NAME)

    print(f"文件     : {CSV_NAME}")
    print(f"数据行   : {len(rows)}")
    print(f"列名     : {', '.join(rows[0].keys())}")
    print()

    # ---- 硬校验 ----
    problems = []
    if len(rows) != 1300:
        problems.append(f"行数 {len(rows)} != 1300")
    if set(r["rest_settled"] for r in rows) != {1}:
        problems.append("存在 rest_settled = 0 的轮次（没停稳）")
    dts = [r["dt_ms"] for r in rows]
    odd = sum(1 for d in dts if d > 55)
    if odd:
        problems.append(f"{odd} 个样本的 dt_ms > 55ms")

    print("--- 硬校验 ---")
    print(f"rest_settled 全为 1 : {set(r['rest_settled'] for r in rows) == {1}}")
    print(f"dt_ms 中位数        : {statistics.median(dts):.1f}")
    print(f"dt_ms 分布          : {dict(sorted((d, dts.count(d)) for d in set(dts)))}")
    if problems:
        print("[WARN] 问题：" + "；".join(problems))
    else:
        print("[OK] 全部通过")
    print()

    # ---- 按 (PWM, trial) 组装 ----
    grouped = defaultdict(list)
    for r in rows:
        grouped[(r["PWM"], r["trial_num"])].append(r)
    for k in grouped:
        grouped[k].sort(key=lambda s: s["sample_num"])

    # ---- 噪声地板：PWM=0 档实测 ----
    null_trials = [trial_metrics(v) for k, v in grouped.items() if k[0] == 0]
    theta_null = max(abs(m["c_total"]) for m in null_trials) + NULL_MARGIN
    print("--- 噪声地板（PWM = 0 档实测）---")
    print(f"各轮 |C_total|      : {[round(abs(m['c_total']), 1) for m in null_trials]}")
    print(f"θ_null              : {theta_null:.1f} 计数")
    print()

    # ---- 逐档汇总 ----
    print(f"--- 逐档结果（θ_speed = {THETA_SPEED_CPS:.0f} counts/s）---")
    print(f"{'PWM':>4} {'n':>2} {'STALL':>5} {'TWITCH':>6} {'CREEP':>5} {'SUST':>4} "
          f"{'p_sust':>6} {'中位速度':>10} {'最大速度':>10} {'范围':>16}")
    summary = {}
    for pwm in sorted(set(r["PWM"] for r in rows)):
        trials = [trial_metrics(grouped[k]) for k in grouped if k[0] == pwm]
        cls = [classify(m, theta_null, THETA_SPEED_CPS) for m in trials]
        n = len(cls)
        n_sust = cls.count("SUSTAINED")
        speeds = sorted(m["v_late"] for m in trials)
        all_spd = [s["speed_cps"] for k in grouped if k[0] == pwm for s in grouped[k]]
        summary[pwm] = {
            "n_sust": n_sust, "n": n, "cls": cls,
            "median_v": statistics.median(speeds),
            "max_v": max(all_spd),
            "min_v": min(all_spd),
        }
        print(f"{pwm:>4} {n:>2} {cls.count('STALL'):>5} {cls.count('TWITCH'):>6} "
              f"{cls.count('CREEP'):>5} {n_sust:>4} {n_sust/n:>6.2f} "
              f"{statistics.median(speeds):>10.1f} {max(all_spd):>10.1f} "
              f"{min(all_spd):>7.1f}..{max(all_spd):<7.1f}")
    print()

    # ---- 死区区间 ----
    zero = [p for p in summary if summary[p]["n_sust"] == 0]
    full = [p for p in summary if summary[p]["n_sust"] == summary[p]["n"]]
    print("--- 死区区间 ---")
    print(f"最大的「一轮都没持续运动」的 PWM : {max(zero) if zero else '无'}")
    print(f"最小的「每一轮都持续运动」的 PWM : {min(full) if full else '无（没有档位做到 5/5）'}")
    if zero and full and max(zero) < min(full):
        print(f"→ 阈值落在 ({max(zero)}, {min(full)}] 之间")
    print()

    # ---- 敏感性：θ_speed 扫一遍 ----
    print("--- θ_speed 敏感性（结论是否随阈值改变）---")
    print(f"{'θ_speed':>8} {'最后全 0 的 PWM':>15} {'首个全 SUST 的 PWM':>18}")
    for theta in (20, 35, 50, 80, 120, 200):
        z, f = [], []
        for pwm in summary:
            trials = [trial_metrics(grouped[k]) for k in grouped if k[0] == pwm]
            cls = [classify(m, theta_null, theta) for m in trials]
            if cls.count("SUSTAINED") == 0:
                z.append(pwm)
            if cls.count("SUSTAINED") == len(cls):
                f.append(pwm)
        print(f"{theta:>8} {max(z) if z else '-':>15} {min(f) if f else '-':>18}")
    print()

    # ---- 和旧数据的交叉验证 ----
    print("--- 交叉验证（与练习 2 的 PWM 40 / 60 稳态对比）---")
    mm_per_count = 0.280578
    for pwm in (40, 60):
        trials = [trial_metrics(grouped[k]) for k in grouped if k[0] == pwm]
        sust = [m for m in trials if classify(m, theta_null, THETA_SPEED_CPS) == "SUSTAINED"]
        if sust:
            v = statistics.median(m["v_late"] for m in sust) * mm_per_count
            print(f"  PWM {pwm}: 本次 {v:.1f} mm/s（{len(sust)}/{len(trials)} 轮持续运动）")
    print("  练习 2 参考：PWM 40 → 129.4 mm/s；PWM 60 → 210.2 mm/s")


if __name__ == "__main__":
    main()
