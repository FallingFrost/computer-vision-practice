# SEMTM0043 电机子系统表征 — 项目导航

Bristol SEMTM0043/42 Robotics Science & Systems（CW2026，Paul O'Dowd）**Phase 1** 个人作业。
用户在 7 个子系统里**选了 Motors**，用 Pololu 3Pi+（Standard，29.86:1）+ M5Stack StampC3 做表征。

**七个练习全部完成。** 详细进度、全部实测结论、未决事项都在
→ **[`电机子系统-进度交接.md`](电机子系统-进度交接.md)**（主入口，先读它）

---

## 最终交付物：1 页技术报告

核心问题：**"What is an important part of the robotic system to model?"**
要回答三点：*Some measurements* / *What do they show?* / *What does it mean for the simulation?*

固定 5 节（原文见 [`课程参考/写作要求-AI.md`](课程参考/写作要求-AI.md)）：

| 节 | 要回答的问题 | 本项目的素材在 |
|---|---|---|
| **Intro** | 你的数字孪生系统被要求做什么？[Project Theme] | 课程页面 Phase 2 说明 |
| **Implementation** | 哪些要素是必须实现/明确定义的？ | 交接文档 §4（代码现状） |
| **Method** | 怎么测量并对比真机与仿真？ | 各练习文档 §5、§6 |
| **Results** | 从结果能看出什么？ | **下表** |
| **Conclusions** | 结果更泛化的意义是什么？ | 各练习文档的「附录：为什么这个练习重要」 |

---

## 七个练习的关键结论（写 Results 直接从这取）

| # | 练习 | 结论 | 数字 |
|---|---|---|---|
| 02 | 编码器测速 | 符号/单位/量级全部验证 | 680 counts/s × 0.280578 = **190.79** mm/s，与固件逐位相同 |
| 03 | **死区** | 阈值区间 | **PWM (30, 40]**；0–30 档 0/5 起转，40 档以上 5/5 |
| 04 | **饱和** | 硬拐点 | **170→180 斜率从 139.9 掉到 11.9**；平台 **2310 counts/s = 648 mm/s** |
| 05 | **增益** | 局部直线模型 | **G = 14.004 counts/s per PWM（3.94 mm/s per PWM）**，c = −90.89 counts/s |
| 06 | 非线性 | 保留直线 | **γ = 1.005**（幂律自己塌回直线）；二次的改善只有 **0.24 个 SEM** |
| 07 | **动态** | 斜率限制优于一阶滞后 | **τ = 0.1332 s**、S = 4636 cps²；留出检验两边都赢 |

**可用区间 = PWM 40 → 170。** 两端分别由死区（练习 3）和饱和（练习 4）定出来。

### 三条最有报告价值的发现

1. **静摩擦 > 动摩擦有直接证据**：PWM 30/35 起转失败的那些轮次**不是"转得慢"，是"根本没起转"**——
   起转成功时速度恰好落在 40–170 拟合直线的延长线上（误差 < 1%）。练习 3 和练习 6 两次独立复现。
2. **饱和是差速转向跑偏的直接机制**：一个轮子到顶、另一个没到，控制器却以为两边都听它的。
   真机有、仿真若漏掉，Phase 2 里所有控制器参数都会失效。
3. **上下行不对称**：下降的 τ 比上升小 **13%**（0.1157 vs 0.1332 s，逐轮不重叠）。
   *"The same limit need not describe rising and falling responses."*

---

## 目录

```
StampC3projects\
├── README.md                     ← 本文件
├── 电机子系统-进度交接.md          ← 主入口：全部结论、未决事项、命令速查
├── 电机练习01…07-代码说明.md      七份逐处讲解（每份含「实测结果」小节）
│
├── DemonstrationTest\            ★ Arduino 工作副本（⚠️ 不能移动）
├── StampC3_Template\             原版参考（⚠️ 不能移动）
├── capture-serial.ps1            串口抓取脚本
├── 数据\                          13 个测量 CSV（+ _损坏备份\）
├── 工具\                          9 个分析脚本（纯标准库）
└── 课程参考\                      Week2.pptx / 入门指南.html / 写作要求-AI.md
```

**Colab 存档**（练习 02–07 的执行版 notebook + 输出图）：
<https://github.com/FallingFrost/robotics-practice/tree/main/notebooks>

---

## 环境要点（踩过的坑）

- **板子枚举为 `USB-Enhanced-SERIAL CH9102 (COM3)`** → `CDCOnBoot=default`（IDE 菜单里的 "Disabled"）**才对**，别改
- **电机走 AAA 电池**（StampC3 走 USB）；**没电时电机完全不转但编码器照常读数** —— 表现为"整份数据零位移"，先查电
- **串口是独占的**：抓取开着时 Arduino IDE 上传会失败
- 无头编译（FQBN `m5stack:esp32:m5stack_stamp_c3`）命令见交接文档 §9
- 本机 Anaconda 的 pandas/matplotlib **因 NumPy 2.x ABI 不匹配全坏** → 用 `工具\` 里的纯标准库脚本

## 两条硬规则

> ⚠️ **每个练习的 notebook 要的 CSV 列都不一样**（03 是 `speed_cps`，04/05/06 是 `settled_speed_cps`，
> 07 是 `trial_id/split/phase` 且不报速度）。动手前先看对应 notebook 的 `Expected CSV columns`。
>
> ⚠️ **不要用 Excel 打开 `数据\` 里的 CSV**。曾经把 `motor_exercise04_saturation.csv` 改坏过
> （多出空列 + 一行垃圾，让所有脚本崩掉）。要看数据用文本编辑器或 pandas。
