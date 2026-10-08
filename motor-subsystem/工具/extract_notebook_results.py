"""从执行过的 .ipynb 里抽出**文字结果**，跳过 base64 图片。

用途：把 Colab 存档里的实测结果取回来，和本地预算的预期值核对，
再把结论写进对应的 `电机练习0N-代码说明.md`。

notebook 里内嵌的 PNG 会让文件膨胀到几百 KB，直接把 JSON 读进上下文会爆掉，
所以这里只取 stream / execute_result 的 text/plain，图只统计张数。

用法：
    python -I extract_notebook_results.py [notebook目录]
"""

import json
import re
import sys
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

# 这些名字开头的赋值行单独列出来——它们是"用户实际用了什么参数"的答案，
# 也是核对 Notebook 常量是否被我提示改过的依据。
KEY_ASSIGN = re.compile(
    r"^\s*(USE_EXAMPLE_DATA|CSV_FILENAME|SELECTED_\w+|FIT_\w+|COMMAND_\w+|"
    r"SPEED_\w+|HELD_\w+|SLEW_\w+|TIME_CONSTANT\w*|TRIAL_\w+|PHASE\w*)\s*="
)

MAX_OUT = 1200   # 单条文字输出最多打印多少字符


def text_of(output):
    """返回 (类型, 文字)。图片返回 (类型, None)。"""
    kind = output.get("output_type", "?")
    if kind == "stream":
        return kind, "".join(output.get("text", []))
    if kind in ("execute_result", "display_data"):
        data = output.get("data", {})
        if "image/png" in data:
            return "image", None
        if "text/plain" in data:
            return kind, "".join(data["text/plain"])
        return kind, None
    if kind == "error":
        return kind, "\n".join(output.get("traceback", []))
    return kind, None


def digest(path):
    nb = json.loads(path.read_text(encoding="utf-8"))
    lines = []
    n_img = 0
    n_out = 0

    for i, cell in enumerate(nb.get("cells", [])):
        if cell.get("cell_type") != "code":
            continue
        src = "".join(cell.get("source", []))

        keys = [ln.strip() for ln in src.splitlines() if KEY_ASSIGN.match(ln)]
        outs = []
        for o in cell.get("outputs", []):
            kind, txt = text_of(o)
            if kind == "image":
                n_img += 1
                continue
            if txt is None:
                continue
            txt = txt.strip()
            if not txt:
                continue
            if len(txt) > MAX_OUT:
                txt = txt[:MAX_OUT] + f"\n... [截断，原长 {len(txt)} 字符]"
            outs.append((kind, txt))

        if not keys and not outs:
            continue

        lines.append(f"\n--- cell {i} ---")
        if keys:
            lines.append("设置: " + " | ".join(keys))
        for kind, txt in outs:
            n_out += 1
            lines.append(f"[{kind}]\n{txt}")

    return "\n".join(lines), n_img, n_out


def main():
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else \
        Path(r"C:\Users\hbx\AppData\Local\Temp\frost_nb")
    if not root.is_dir():
        raise SystemExit(f"目录不存在：{root}")

    for path in sorted(root.glob("*.ipynb")):
        print("=" * 78)
        print(f"### {path.name}")
        print("=" * 78)
        body, n_img, n_out = digest(path)
        if not body:
            print("(没有文字输出——可能未执行，或输出全是图)")
        else:
            print(body)
        print(f"\n[图 {n_img} 张，文字输出 {n_out} 段]")
        print()


if __name__ == "__main__":
    main()
