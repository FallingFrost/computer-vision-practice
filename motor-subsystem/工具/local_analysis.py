# -*- coding: utf-8 -*-
"""
本地复现 motor-exercise-02-encoder-speed.ipynb 的分析流程，用真实抓到的数据。

做三件事：
  1. 把多次抓取的 CSV 合并、重新编号 trial，写成 notebook 要的文件名
  2. 完全照搬 notebook 的计算逻辑（diff 求速度），出两张图
  3. 额外出一份"每轮叠加"的图，便于肉眼比对（notebook 的图横轴是绝对时间戳，
     跨 session 合并后会互相穿插，可读性差）
"""

import matplotlib
matplotlib.use("Agg")          # 无头环境出图，不弹窗

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

sns.set_theme(style="whitegrid", context="notebook")
pd.set_option("display.max_columns", 100)

BASE = Path(r"D:\work space\robotics\StampC3projects")
DATA = BASE / "数据"          # 所有测量 CSV 都在这里
OUT_CSV = DATA / "motor_exercise02_encoder_snapshots.csv"

# 要合并的来源，按时间顺序
SOURCES = [
    ("motor_exercise02_2026-10-01_handturn.csv", "手转"),
    ("motor_exercise02_sign-test.csv",           "符号测试"),
]

# ---------------------------------------------------------------- 1. 合并
frames, mapping = [], []
trial = 0
for name, label in SOURCES:
    df = pd.read_csv(DATA / name)
    if "trial_num" not in df.columns:
        raise SystemExit(f"{name} 第一行不是表头？列名是 {list(df.columns)[:3]}")
    for old in sorted(df["trial_num"].unique()):
        trial += 1
        sel = df[df["trial_num"] == old].copy()
        sel["trial_num"] = trial
        frames.append(sel)
        mapping.append((trial, label, int(old), len(sel)))

combined = pd.concat(frames, ignore_index=True)
combined = combined.sort_values(["trial_num", "sample_num"]).reset_index(drop=True)
combined.to_csv(OUT_CSV, index=False)

print("=" * 70)
print(f"合并写入 {OUT_CSV.name}：{len(combined)} 行，{trial} 轮")
for t, label, old, n in mapping:
    print(f"  trial {t}  <-  {label} 原第 {old} 轮   ({n} 行)")
print()

# ------------------------------------------------- 2. 照搬 notebook 的计算
data = combined.copy()
print("=" * 70)
print("data.head()")
print(data.head().to_string())
print()

data = data.sort_values(["trial_num", "sample_num"]).copy()
data["elapsed_s"] = data.groupby("trial_num")["timestamp_ms"].diff() / 1000
data["left_speed_cps"] = (
    data.groupby("trial_num")["left_encoder_count"].diff() / data["elapsed_s"]
)
data["right_speed_cps"] = (
    data.groupby("trial_num")["right_encoder_count"].diff() / data["elapsed_s"]
)

print("速度计算（前 8 行，首行必为 NaN——这是 diff 的固有行为）")
print(data[["timestamp_ms", "elapsed_s", "left_speed_cps", "right_speed_cps"]].head(8).to_string())
print()

# ------------------------------------------------------- 3. notebook 原版图
count_plot_data = data.melt(
    id_vars=["trial_num", "timestamp_ms"],
    value_vars=["left_encoder_count", "right_encoder_count"],
    var_name="wheel", value_name="encoder_count",
)
plt.figure(figsize=(11, 5))
sns.lineplot(data=count_plot_data, x="timestamp_ms", y="encoder_count",
             hue="wheel", estimator=None)
plt.title("Recorded encoder counts")
plt.xlabel("Timestamp (ms)")
plt.ylabel("Encoder count")
plt.tight_layout()
plt.savefig(BASE / "plot1_encoder_counts.png", dpi=110)
plt.close()

speed_plot_data = data.melt(
    id_vars=["trial_num", "timestamp_ms"],
    value_vars=["left_speed_cps", "right_speed_cps"],
    var_name="wheel", value_name="speed_cps",
).dropna()
plt.figure(figsize=(11, 5))
sns.lineplot(data=speed_plot_data, x="timestamp_ms", y="speed_cps",
             hue="wheel", estimator=None)
plt.axhline(0, color="black", linewidth=1)
plt.title("Wheel speed calculated from encoder snapshots")
plt.xlabel("Timestamp (ms)")
plt.ylabel("Encoder speed (counts/s)")
plt.tight_layout()
plt.savefig(BASE / "plot2_wheel_speed.png", dpi=110)
plt.close()

# ------------------------------------------- 4. 每轮叠加图（更好读，供自己看）
data["t_in_trial_s"] = data.groupby("trial_num")["timestamp_ms"].transform(
    lambda s: (s - s.iloc[0]) / 1000.0
)

fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
for t, g in data.groupby("trial_num"):
    axes[0].plot(g["t_in_trial_s"], g["left_encoder_count"], lw=1.2, label=f"trial {t} L")
    axes[1].plot(g["t_in_trial_s"], g["right_encoder_count"], lw=1.2, label=f"trial {t} R")
axes[0].set_ylabel("left encoder count"); axes[0].legend(fontsize=7, ncol=3)
axes[1].set_ylabel("right encoder count"); axes[1].legend(fontsize=7, ncol=3)
axes[1].set_xlabel("time since trial start (s)")
axes[0].set_title("Per-trial encoder counts (aligned to each trial's own start)")
plt.tight_layout()
plt.savefig(BASE / "plot3_per_trial_counts.png", dpi=110)
plt.close()

# --------------------------------------------------------- 5. 每轮数字摘要
print("=" * 70)
print("每轮摘要")
for t, g in data.groupby("trial_num"):
    lp = (g["left_speed_cps"] > 0).sum()
    ln = (g["left_speed_cps"] < 0).sum()
    rp = (g["right_speed_cps"] > 0).sum()
    rn = (g["right_speed_cps"] < 0).sum()
    lc0, lc1 = g["left_encoder_count"].iloc[0], g["left_encoder_count"].iloc[-1]
    rc0, rc1 = g["right_encoder_count"].iloc[0], g["right_encoder_count"].iloc[-1]
    print(f"  trial {t}: 左计数 {lc0:>6} -> {lc1:>6} ({lc1-lc0:+6})   "
          f"正 {lp:>3} / 负 {ln:>3}   |   右计数 {rc0:>6} -> {rc1:>6} ({rc1-rc0:+6})   "
          f"正 {rp:>3} / 负 {rn:>3}")

print()
print("=" * 70)
print("输出文件：")
for p in ["motor_exercise02_encoder_snapshots.csv",
          "plot1_encoder_counts.png",
          "plot2_wheel_speed.png",
          "plot3_per_trial_counts.png"]:
    f = DATA / p
    print(f"  {'OK ' if f.exists() else 'XX '} {f}")
