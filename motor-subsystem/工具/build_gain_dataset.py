"""把练习 3 / 4 / 5 的测量拼成练习 5（Gain）notebook 要的那一份 CSV。

为什么需要这个脚本，而不是手工拼：

1. **练习 3 的数据必须改用「后段窗口」。** 练习 3 每轮都从静止起步，
   整个 1.0 秒测量窗里前 ~300ms 在加速。整轮均值会把稳态速度低估约 15%
   （PWM 60 处实测 639.6 vs 稳态 752.0）。只用样本 11–20 之后，
   它与练习 4 的稳态值吻合到 0.10%。
2. **留出的两个档位必须从拟合集里剔掉。** 课程原文说留出点是
   "the PWM condition you held back (i.e., that you have not directly measured)"——
   所以练习 4 在 100 / 160 档测过的行不能进这份数据集，否则预测检验就是自己考自己。
   留出的档位由 testGain 重新测一次，那些行才进数据集。

输出 `motor_exercise05_gain.csv`，正好是 notebook 里 CSV_FILENAME 的默认值。
额外加一列 `source` 标明每行来自哪份数据（notebook 不读，但报告要用）。

只用标准库：本机 pandas 因 NumPy 2.x ABI 不匹配而不可用。
"""

import csv
import sys
from collections import Counter
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    # SystemExit 的文本走 stderr，不一起改的话中文会乱码。
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

BASE = Path(r"D:\work space\robotics\StampC3projects")
DATA = BASE / "数据"          # 所有测量 CSV 都在这里

EX3_FILE = "motor_exercise03_left_fwd.csv"
EX4_FILE = "motor_exercise04_saturation.csv"
GAIN_RAW_FILE = "motor_exercise05_gain_raw.csv"
OUT_FILE = "motor_exercise05_gain.csv"

# 练习 3 的测速窗从静止起步，前段是加速过程，不能算作稳态速度。
# 后 10 个样本（500ms 之后）电机已经稳住了。
EX3_LATE_SAMPLE_FROM = 11

# 拟合区间：死区上界 40（练习 3）/ 饱和拐点约 171（练习 4）。
FIT_PWM_MIN = 40
FIT_PWM_MAX = 170

# 留出、不参与拟合的档位。必须与固件里 GAIN_PWM_LEVEL_* 一致。
HELD_BACK = [100, 160]

OUT_COLUMNS = [
    "wheel", "direction", "PWM", "trial_num", "sample_num",
    "settled_speed_cps", "speed_mm_s", "dt_ms", "time_ms",
    "encoder_count", "other_encoder_count", "source",
]


def read_rows(path):
    with path.open(newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        if reader.fieldnames is None:
            raise SystemExit(f"{path.name} 是空文件")
        # 练习 3 的速度列叫 speed_cps，练习 4/5 叫 settled_speed_cps。
        speed_key = None
        for candidate in ("settled_speed_cps", "speed_cps"):
            if candidate in reader.fieldnames:
                speed_key = candidate
                break
        if speed_key is None:
            raise SystemExit(f"{path.name} 里找不到速度列：{reader.fieldnames}")

        rows = []
        for r in reader:
            rows.append({
                "wheel": r["wheel"],
                "direction": r["direction"],
                "PWM": int(r["PWM"]),
                "trial_num": int(r["trial_num"]),
                "sample_num": int(r["sample_num"]),
                "settled_speed_cps": float(r[speed_key]),
                "speed_mm_s": float(r["speed_mm_s"]),
                "dt_ms": float(r["dt_ms"]),
                "time_ms": float(r["time_ms"]),
                "encoder_count": int(r["encoder_count"]),
                "other_encoder_count": int(r["other_encoder_count"]),
            })
        return rows


def main():
    collected = []

    # ---- 练习 3：40–60 段，只用后段窗口 ----
    p = DATA / EX3_FILE
    if not p.exists():
        raise SystemExit(f"找不到 {EX3_FILE}")
    ex3 = read_rows(p)
    ex3_kept = [
        r for r in ex3
        if FIT_PWM_MIN <= r["PWM"] < 60 and r["sample_num"] >= EX3_LATE_SAMPLE_FROM
    ]
    for r in ex3_kept:
        r["source"] = "ex3_late"
    collected += ex3_kept
    print(f"练习 3 : 读入 {len(ex3):5d} 行 → 取 PWM {FIT_PWM_MIN}–55 的后段样本 "
          f"(sample_num >= {EX3_LATE_SAMPLE_FROM}) → 保留 {len(ex3_kept)} 行")

    # ---- 练习 4：60–170，剔除留出档位 ----
    p = DATA / EX4_FILE
    if not p.exists():
        raise SystemExit(f"找不到 {EX4_FILE}")
    ex4 = read_rows(p)
    ex4_kept = [
        r for r in ex4
        if 60 <= r["PWM"] <= FIT_PWM_MAX and r["PWM"] not in HELD_BACK
    ]
    ex4_dropped = [
        r for r in ex4
        if 60 <= r["PWM"] <= FIT_PWM_MAX and r["PWM"] in HELD_BACK
    ]
    for r in ex4_kept:
        r["source"] = "ex4"
    collected += ex4_kept
    print(f"练习 4 : 读入 {len(ex4):5d} 行 → 取 PWM 60–{FIT_PWM_MAX}，"
          f"剔除留出档位 {HELD_BACK} → 保留 {len(ex4_kept)} 行"
          f"（丢掉 {len(ex4_dropped)} 行）")

    # ---- 练习 5：重新测的留出档位 ----
    p = DATA / GAIN_RAW_FILE
    if not p.exists():
        raise SystemExit(
            f"找不到 {GAIN_RAW_FILE}。\n"
            f"先跑 testGain 抓一份原始数据：\n"
            f"  .\\capture-serial.ps1 -Port COM3 -Seconds 60 `\n"
            f"    -OutFile {GAIN_RAW_FILE} `\n"
            f'    -ExpectedHeader "wheel,direction,PWM,trial_num,sample_num,'
            f'settled_speed_cps,speed_mm_s,dt_ms,time_ms,encoder_count,other_encoder_count"'
        )
    gain = read_rows(p)
    gain_pwms = sorted(set(r["PWM"] for r in gain))
    if gain_pwms != sorted(HELD_BACK):
        raise SystemExit(
            f"{GAIN_RAW_FILE} 里的档位是 {gain_pwms}，"
            f"与本脚本的 HELD_BACK {sorted(HELD_BACK)} 不符。\n"
            f"检查固件里的 GAIN_PWM_LEVEL_* 是否改过。"
        )
    for r in gain:
        r["source"] = "ex5_testGain"
    collected += gain
    print(f"练习 5 : 读入 {len(gain):5d} 行，档位 {gain_pwms}（留出档位的重新测量）")

    # ---- 校验 ----
    print()
    print("--- 校验 ---")
    by_pwm = Counter(r["PWM"] for r in collected)
    fit_pwms = sorted(p for p in by_pwm if p not in HELD_BACK)
    print(f"拟合集档位 : {fit_pwms}")
    print(f"留出档位   : {sorted(HELD_BACK)}（在数据集里，但不参与拟合）")
    missing = [p for p in range(FIT_PWM_MIN, FIT_PWM_MAX + 1, 5)
               if p not in by_pwm]
    print(f"40–170 中的空档 : {missing if missing else '无'}")

    # 每个档位的轮数应当一致
    per_pwm_trials = {}
    for r in collected:
        per_pwm_trials.setdefault(r["PWM"], set()).add(r["trial_num"])
    uneven = {p: sorted(t) for p, t in sorted(per_pwm_trials.items()) if len(t) != 5}
    if uneven:
        print(f"[注意] 轮数不是 5 的档位（练习 3 的档位本来就只有 5 轮，正常）：")
        for p, t in uneven.items():
            print(f"         PWM {p}: {len(t)} 轮")
    print(f"总行数     : {len(collected)}")

    # ---- 写出 ----
    out = DATA / OUT_FILE
    with out.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=OUT_COLUMNS)
        w.writeheader()
        for r in sorted(collected, key=lambda r: (r["PWM"], r["source"], r["trial_num"], r["sample_num"])):
            w.writerow(r)

    print()
    print(f"已写出 : {out}")
    print(f"notebook 里 CSV_FILENAME 用 {OUT_FILE}，并把 {HELD_BACK} 从拟合中排除。")


if __name__ == "__main__":
    main()
