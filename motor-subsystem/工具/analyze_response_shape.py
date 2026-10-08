"""练习 6（Non-linearity）本地预检 —— 只用标准库。

复刻 `motor-exercise-06-model-comparison.ipynb` §6–8 的分析：

  1. 按 PWM 汇总，用**固定的**归一化边界算 x 和 v
     （x = (u − u_min)/(u_max − u_min)，v = (ω − ω_min)/(ω_max − ω_min)）
  2. 在**同一批拟合观测**上拟合三个候选模型：
       线性    v̂ = A + Bx
       二次    v̂ = A + Bx + Cx²
       幂律    v̂ = x^γ
  3. 用**同一批留出指令**做预测检验，报告 MAE 与逐点误差

课程的核心警告：二次模型参数更多、总能把拟合点跟得更紧，
**只有当它在留出点上也可重复地预测更好时，这点额外的灵活性才值得**。

本机 pandas / numpy 不可用（NumPy 2.x ABI 不匹配），所以全部手写。
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
SRC = "motor_exercise05_gain.csv"   # 练习 6 复用练习 5 的数据集

# ---- 归一化边界：固定，且同时用于拟合数据和留出数据 ----
COMMAND_MIN = 40
COMMAND_MAX = 170
HELD_BACK = [100, 160]
MM_PER_COUNT = 0.280578


def read(path):
    with path.open(newline="", encoding="utf-8") as fh:
        rdr = csv.DictReader(fh)
        key = next((k for k in ("settled_speed_cps", "speed_cps")
                    if k in (rdr.fieldnames or [])), None)
        if key is None:
            raise SystemExit(f"{path.name} 找不到速度列")
        return [{"PWM": int(r["PWM"]), "speed": float(r[key]),
                 "trial": int(r["trial_num"])} for r in rdr]


def summarise(rows):
    b = defaultdict(list)
    for r in rows:
        b[r["PWM"]].append(r)
    out = {}
    for p, rs in sorted(b.items()):
        speeds = [r["speed"] for r in rs]
        # 有效独立重复是**轮数**，不是样本数：同一轮里的 20 个样本是同一个稳态的
        # 连续采样，彼此高度相关，拿它们当独立观测会把标准误低估约 √20 倍。
        trials = len(set(r["trial"] for r in rs))
        out[p] = {"mean": statistics.mean(speeds),
                  "sd": statistics.stdev(speeds) if len(speeds) > 1 else 0.0,
                  "n": len(speeds), "trials": trials}
    return out


# ---------------- 三个模型 ----------------

def fit_linear(xs, ys):
    """v̂ = A + Bx，闭式解。返回 (A, B)。"""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    B = sxy / sxx
    return my - B * mx, B


def fit_quadratic(xs, ys):
    """v̂ = A + Bx + Cx²，解 3x3 正规方程。返回 (A, B, C)。"""
    # 构造 Σx^k 与 Σx^k·y
    s = [sum(x ** k for x in xs) for k in range(5)]        # s0..s4
    t = [sum((x ** k) * y for x, y in zip(xs, ys)) for k in range(3)]  # t0..t2
    # | s0 s1 s2 | |A|   |t0|
    # | s1 s2 s3 | |B| = |t1|
    # | s2 s3 s4 | |C|   |t2|
    M = [[s[0], s[1], s[2]],
         [s[1], s[2], s[3]],
         [s[2], s[3], s[4]]]
    rhs = [t[0], t[1], t[2]]

    # 高斯消元（选主元）
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-12:
            raise SystemExit("二次拟合的正规方程奇异，检查 x 是否退化")
        M[col], M[piv] = M[piv], M[col]
        rhs[col], rhs[piv] = rhs[piv], rhs[col]
        for r in range(col + 1, 3):
            f = M[r][col] / M[col][col]
            for c in range(col, 3):
                M[r][c] -= f * M[col][c]
            rhs[r] -= f * rhs[col]
    sol = [0.0, 0.0, 0.0]
    for r in (2, 1, 0):
        acc = rhs[r] - sum(M[r][c] * sol[c] for c in range(r + 1, 3))
        sol[r] = acc / M[r][r]
    return sol[0], sol[1], sol[2]


def fit_power(fit_pwms, norm, lo=0.5, hi=2.0, n=301):
    """v̂ = x^γ。

    **刻意照抄 notebook 的做法**：γ 从 `np.linspace(0.5, 2.0, 301)` 这个网格里挑，
    判据是归一化 MAE 最小（不是 SSE）。网格步长 0.005，所以结果只能落在
    1.000 / 1.005 / 1.010 ... 这样的值上——自己用连续优化算出 1.0047 是没意义的，
    会和对不上 notebook 的输出。
    """
    grid = [lo + (hi - lo) * i / (n - 1) for i in range(n)]

    def mae_of(g):
        return sum(abs(norm[p]["v"] - (norm[p]["x"] ** g)) for p in fit_pwms) / len(fit_pwms)

    return min(grid, key=mae_of)


def main():
    rows = read(DATA / SRC)
    s = summarise(rows)
    pwms = sorted(s)

    speed_min = s[COMMAND_MIN]["mean"]
    speed_max = s[COMMAND_MAX]["mean"]

    print("=" * 72)
    print("归一化边界（题目要求：取自死区 / 饱和 / 增益的证据，且全程固定）")
    print("=" * 72)
    print(f"u_min (PWM)      = {COMMAND_MIN}          ← 练习 3 死区上界 (30, 40]")
    print(f"u_max (PWM)      = {COMMAND_MAX}         ← 练习 4 饱和拐点 ~171")
    print(f"ω_min (counts/s) = {speed_min:.1f}   ← PWM {COMMAND_MIN} 的实测稳态速度")
    print(f"ω_max (counts/s) = {speed_max:.1f}  ← PWM {COMMAND_MAX} 的实测稳态速度")
    print(f"                   （{speed_min*MM_PER_COUNT:.1f} 和 {speed_max*MM_PER_COUNT:.1f} mm/s）")
    print()

    # 归一化
    norm = {}
    for p in pwms:
        x = (p - COMMAND_MIN) / (COMMAND_MAX - COMMAND_MIN)
        v = (s[p]["mean"] - speed_min) / (speed_max - speed_min)
        norm[p] = {"x": x, "v": v, "sd": s[p]["sd"], "mean": s[p]["mean"]}

    # 边界自检
    print("边界自检（课程要求：边界处的观测应当≈ 0 和 1）：")
    print(f"  PWM {COMMAND_MIN}: x = {norm[COMMAND_MIN]['x']:.3f}, v = {norm[COMMAND_MIN]['v']:.3f}")
    print(f"  PWM {COMMAND_MAX}: x = {norm[COMMAND_MAX]['x']:.3f}, v = {norm[COMMAND_MAX]['v']:.3f}")
    print()

    fit_pwms = [p for p in pwms if p not in HELD_BACK]
    xs = [norm[p]["x"] for p in fit_pwms]
    vs = [norm[p]["v"] for p in fit_pwms]

    print(f"拟合用档位（{len(fit_pwms)} 个，三个模型完全相同）: {fit_pwms}")
    print(f"留出档位（不参与任何拟合）        : {HELD_BACK}")
    print()

    # ---- 拟合 ----
    # 注意：notebook 的「线性参照」是**固定的 v̂ = x**，没有可拟合系数。
    # 二次和幂律才是拟合出来的。下面那个自由拟合的直线只作参照，不参与比较。
    A2, B2, C2 = fit_quadratic(xs, vs)
    gamma = fit_power(fit_pwms, norm)
    A1, B1 = fit_linear(xs, vs)

    models = {
        "线性":   (lambda x: x,                          "v = x          （固定，no fitted coefficient）"),
        "二次":   (lambda x: A2 + B2 * x + C2 * x * x,   f"v = {A2:+.4f} {B2:+.4f}·x {C2:+.4f}·x²"),
        "幂律":   (lambda x: x ** gamma,                 f"v = x^{gamma:.3f}"),
    }

    print("=" * 72)
    print("模型")
    print("=" * 72)
    for name, (_, eq) in models.items():
        print(f"  {name:4s}  {eq}")
    print()
    print(f"  幂律指数 γ = {gamma:.3f}   （γ=1 就是直线；γ>1 低端更平后段更陡；0<γ<1 反之）")
    print(f"      ↑ 网格 linspace(0.5, 2.0, 301)，步长 0.005 —— 与 notebook 一致")
    print(f"  参照：若把直线也拿去自由拟合，会得到 v = {A1:+.4f} {B1:+.4f}·x")
    print(f"        notebook 不用这个，它用固定的 v = x")
    print()

    # ---- MAE：拟合集 ----
    def mae(rows_pwms, fn):
        return sum(abs(norm[p]["v"] - fn(norm[p]["x"])) for p in rows_pwms) / len(rows_pwms)

    print("=" * 72)
    print("MAE（归一化单位）—— 课程要求三个模型用同样的行、同样的定义")
    print("=" * 72)
    print(f"{'模型':<6} {'拟合集 MAE':>12} {'留出点 MAE':>12} {'留出点1 误差':>14} {'留出点2 误差':>14}")
    results = {}
    for name, (fn, _) in models.items():
        m_fit = mae(fit_pwms, fn)
        m_held = mae(HELD_BACK, fn)
        e0 = norm[HELD_BACK[0]]["v"] - fn(norm[HELD_BACK[0]]["x"])
        e1 = norm[HELD_BACK[1]]["v"] - fn(norm[HELD_BACK[1]]["x"])
        results[name] = (m_fit, m_held, e0, e1, fn)
        print(f"{name:<6} {m_fit:12.5f} {m_held:12.5f} {e0:+14.5f} {e1:+14.5f}")
    print()

    # ---- 留出点换算回 counts/s，便于和重复离散度比 ----
    span = speed_max - speed_min
    print("=" * 72)
    print("留出点检验（换算回 counts/s，好和重复离散度比）")
    print("=" * 72)
    print(f"{'模型':<6} {'PWM':>5} {'预测':>10} {'实测':>10} {'误差':>9} {'重复SD':>8} {'误差/SD':>8}")
    for name, (fn, _) in models.items():
        for p in HELD_BACK:
            pred_cps = speed_min + fn(norm[p]["x"]) * span
            err = norm[p]["mean"] - pred_cps
            sd = norm[p]["sd"]
            print(f"{name:<6} {p:>5} {pred_cps:10.1f} {norm[p]['mean']:10.1f} "
                  f"{err:+9.1f} {sd:8.1f} {abs(err)/sd:8.2f}")
    print()

    # ---- 线性模型的残余走向：课程的关键诊断 ----
    print("=" * 72)
    print("线性模型的残余走向（课程：残差应当散在 0 两侧；系统性穿越 = 漏了曲率）")
    print("=" * 72)
    fn_lin = models["线性"][0]
    print(f"{'PWM':>4} {'x':>6} {'残余(归一)':>11} {'残余(counts/s)':>15} {'重复SD':>8} {'残余/SD':>8}")
    signs = []
    for p in pwms:
        e = norm[p]["v"] - fn_lin(norm[p]["x"])
        e_cps = e * span
        sd = norm[p]["sd"]
        tag = "  ← 留出" if p in HELD_BACK else ""
        signs.append("+" if e > 0 else "-")
        print(f"{p:>4} {norm[p]['x']:6.3f} {e:+11.5f} {e_cps:+15.1f} {sd:8.1f} "
              f"{e_cps/sd:+8.2f}{tag}")
    print()
    print(f"符号序列 : {''.join(signs)}")
    n_cross = sum(1 for i in range(1, len(signs)) if signs[i] != signs[i - 1])
    print(f"穿越次数 : {n_cross}  （0-1 次 = 随机散布；多次系统性穿越 = 有曲率结构）")
    print()

    # ---- 结论提示 ----
    print("=" * 72)
    print("读法")
    print("=" * 72)
    best_fit = min(results, key=lambda k: results[k][0])
    best_held = min(results, key=lambda k: results[k][1])
    print(f"拟合集 MAE 最小 : {best_fit}")
    print(f"留出点 MAE 最小 : {best_held}")
    if best_fit != best_held:
        print("→ 两者不一致。课程原话：更低的拟合 MAE 只有在「改善足够大、")
        print("  相对重复离散度有意义、且被未见过的预测支持」时才说明问题。")
    else:
        print("→ 两者一致。")
    print()
    lin_fit = results["线性"][0]
    for name in ("二次", "幂律"):
        imp = (lin_fit - results[name][0]) / lin_fit * 100
        print(f"  相对线性，{name} 的拟合 MAE 改善 {imp:+.1f}%")
    print()
    # 残差相对于重复离散度的量级
    mean_sd = statistics.mean(norm[p]["sd"] for p in pwms) / span
    print(f"重复离散度（归一化）约 {mean_sd:.5f}；线性模型拟合集 MAE 为 {lin_fit:.5f}")
    print(f"  → MAE / 重复离散度 = {lin_fit/mean_sd:.3f}")
    print("  < 1 表示线性模型的平均误差小于测量本身的散布。")
    print()

    # ---- 决定性问题：二次那点改善，比「拟合点本身的不确定度」大吗？----
    print("=" * 72)
    print("决定性问题：二次的改善，超过拟合点自身的不确定度了吗？")
    print("=" * 72)
    print("每个档位的均值是用 5 轮独立重复测出来的，所以该均值的标准误是")
    print("  SEM = 重复SD / √5")
    print("二次拟合把 MAE 降低的那点量，只有大于 SEM 才谈得上「真的更好」。")
    print()
    sems = [norm[p]["sd"] / math.sqrt(s[p]["trials"]) for p in fit_pwms]
    mean_sem = statistics.mean(sems)
    mean_sem_norm = mean_sem / span
    imp_norm = results["线性"][0] - results["二次"][0]
    imp_cps = imp_norm * span
    print(f"各档平均 SEM            : {mean_sem:.1f} counts/s  （归一化 {mean_sem_norm:.5f}）")
    print(f"二次相对线性的 MAE 改善  : {imp_cps:.1f} counts/s  （归一化 {imp_norm:.5f}）")
    print(f"  → 改善 / SEM = {imp_cps/mean_sem:.2f}")
    if imp_cps < mean_sem:
        print("  **改善小于 1 个 SEM** → 这点曲率没有从测量不确定度里浮现出来，")
        print("  不足以支撑「二次更好」这个结论。")
    print()

    # ---- 补充：同会话留出，把「模型形状」和「会话偏移」分开 ----
    print("=" * 72)
    print("补充检验：同会话留出（把模型形状与会话偏移分开）")
    print("=" * 72)
    print("上面那两个留出点来自练习 5 单独跑的会话，整体偏高约 +9 counts/s。")
    print("那点偏移（~9）比模型之间的差异（1–4）还大，所以上面那次留出检验")
    print("分辨不出模型好坏。这里改成：留出两个**同一会话内**的档位，")
    print("让三个模型在同等条件下比预测力。")
    print()
    held2 = [90, 140]          # 都来自练习 4 的那次会话
    fit2 = [p for p in pwms if p not in held2]
    xs2 = [norm[p]["x"] for p in fit2]
    vs2 = [norm[p]["v"] for p in fit2]

    a2, b2, c2 = fit_quadratic(xs2, vs2)
    g2 = fit_power(fit2, norm)
    m2 = {
        "线性": lambda x: x,                          # 同样用固定的 v = x
        "二次": lambda x: a2 + b2 * x + c2 * x * x,
        "幂律": lambda x: x ** g2,
    }
    print(f"留出 : {held2}   拟合用 : {len(fit2)} 档")
    print(f"幂律指数 γ = {g2:.3f}   二次的 C = {c2:+.5f}")
    print()
    print(f"{'模型':<6} {'PWM':>5} {'预测(cps)':>11} {'实测(cps)':>11} {'误差':>9} "
          f"{'重复SD':>8} {'误差/SD':>8}")
    for name, fn in m2.items():
        for p in held2:
            pred = speed_min + fn(norm[p]["x"]) * span
            err = norm[p]["mean"] - pred
            print(f"{name:<6} {p:>5} {pred:11.1f} {norm[p]['mean']:11.1f} "
                  f"{err:+9.1f} {norm[p]['sd']:8.1f} {abs(err)/norm[p]['sd']:8.2f}")
    print()
    held_mae = {n: sum(abs(norm[p]["v"] - fn(norm[p]["x"])) for p in held2) / len(held2)
                for n, fn in m2.items()}
    print("同会话留出 MAE：")
    for n in ("线性", "二次", "幂律"):
        print(f"  {n}: {held_mae[n]:.5f}")
    best2 = min(held_mae, key=held_mae.get)
    print(f"→ 同会话下留出最好的是：{best2}")
    print()
    print("若三个模型的留出 MAE 彼此接近、且都远小于重复离散度，")
    print("那就说明**这点曲率在测量分辨率之下**，应当保留更简单的直线模型。")


if __name__ == "__main__":
    main()
