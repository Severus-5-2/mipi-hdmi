#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bit_sim_osd_coord.py —— 坐标 OSD 的端到端核对：字形 → 屏幕
================================================================================
做的事
------
读 tb/tb_osd_coord.v 跑出来的 dump（被点亮的像素列表），
用【从 rtl/track_glyph_rom.v 原文解析出来的点阵】自行拼出
"X:xxxx" / "Y:yyyy" 应有的像素集合，逐像素比对，并把结果画成 ASCII
（直接就能用眼睛读出屏幕上写的是什么）。

★为什么要"从 RTL 解析字模"而不是"手抄一份字模"
    项目血泪教训（README 第 7 节 / 改动说明 2026-10-09）：
    Logo 那次事故就是验证脚本自己写了"应有"的公式，结果脚本和 RTL 一起错、
    还报 PASS。所以这里复用 glyph_rom_tool.parse_rtl()，只读 RTL 文本。

覆盖的链路
----------
    数值 → BCD 拆分 → 字符码选择 → 字模查表 → 位选 → 像素位置
这五步里任何一步错了（前导零、进位、bit 方向、行列序号、区域起点），
本脚本都会在对不上时报出来。

用法
----
    # 先跑 TB（三种数值各跑一遍，dump 会被覆盖，比对时用同一组数值）
    cd user_source/hdl_source/track_box
    iverilog -g2005 -o sim/tb_osd_coord.vvp tb/tb_osd_coord.v \
             rtl/osd_coord.v rtl/pix_coord_gen.v rtl/track_glyph_rom.v
    vvp sim/tb_osd_coord.vvp +xv=640 +yv=360

    python sim/bit_sim_osd_coord.py --xv 640 --yv 360
================================================================================
"""

import argparse
import os
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from glyph_rom_tool import parse_rtl, RTL_PATH          # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DUMP = os.path.join(HERE, "tb_osd_coord_out.txt")

# ---------------------------------------------------------------------------
# 布局：★必须与 tb/tb_osd_coord.v 里 osd_coord 实例化的参数一致
# ---------------------------------------------------------------------------
OSD_X0 = 0
OSD_Y1 = 0
OSD_Y2 = 16
CELL = 16
N_CHAR = 6


def compose(glyphs, text, x0, y0):
    """把 text（6 个字符）按 16x16 单元拼成像素集合。bit15 = 最左像素。"""
    pts = set()
    for idx, ch in enumerate(text):
        code = ord(ch)
        if code not in glyphs:
            raise KeyError(f"字模里没有字符 '{ch}' (0x{code:02x})")
        rows = glyphs[code]
        ox = x0 + idx * CELL
        for r in range(CELL):
            v = rows.get(r, 0)
            for c in range(CELL):
                if (v >> (15 - c)) & 1:
                    pts.add((ox + c, y0 + r))
    return pts


def render_zone(pts, x0, y0, w, h, lit="##", blank="  "):
    lines = []
    for y in range(y0, y0 + h):
        lines.append("".join(lit if (x, y) in pts else blank for x in range(x0, x0 + w)))
    return lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xv", type=int, default=640, help="要核对的 X 数值（与 TB 的 +xv 一致）")
    ap.add_argument("--yv", type=int, default=360, help="要核对的 Y 数值（与 TB 的 +yv 一致）")
    args = ap.parse_args()

    if not os.path.exists(DUMP):
        print(f"[FAIL] 找不到 {DUMP}，请先跑 tb_osd_coord")
        return 2

    glyphs = parse_rtl(RTL_PATH)
    if not glyphs:
        print(f"[FAIL] 从 {RTL_PATH} 解析不到字模")
        return 2

    dut = set()
    with open(DUMP, "r", encoding="utf-8") as f:
        for line in f:
            p = line.split()
            if len(p) == 2:
                dut.add((int(p[0]), int(p[1])))

    t1 = "X:%04d" % args.xv
    t2 = "Y:%04d" % args.yv
    exp = compose(glyphs, t1, OSD_X0, OSD_Y1) | compose(glyphs, t2, OSD_X0, OSD_Y2)

    print("=" * 74)
    print(f"osd_coord 端到端核对   X={args.xv}  Y={args.yv}")
    print(f"  期望屏幕文字 : {t1} / {t2}")
    print("=" * 74)
    print(f"  TB dump 点亮像素 : {len(dut)}")
    print(f"  字模拼字应有像素 : {len(exp)}")

    only_dut = sorted(dut - exp)
    only_exp = sorted(exp - dut)
    ok = not only_dut and not only_exp
    print(f"  dump 多出的像素 : {len(only_dut)}")
    print(f"  缺失的像素      : {len(only_exp)}")
    print(f"  逐像素一致      : [{'PASS' if ok else 'FAIL'}]")
    if only_dut[:6]:
        print(f"    多出样例: {only_dut[:6]}")
    if only_exp[:6]:
        print(f"    缺失样例: {only_exp[:6]}")
    print()

    print("-" * 74)
    print("  TB dump 渲染（用眼睛读一下是不是写着 %.6s / %.6s）：" % (t1, t2))
    print("-" * 74)
    for line in render_zone(dut, OSD_X0, OSD_Y1, N_CHAR * CELL, CELL):
        print("    " + line)
    print("    " + "-" * (N_CHAR * CELL * 2))
    for line in render_zone(dut, OSD_X0, OSD_Y2, N_CHAR * CELL, CELL):
        print("    " + line)
    print("-" * 74)
    print()

    print("=" * 74)
    print("结论：" + ("坐标 OSD 全链路（BCD / 字符码 / 字模 / 位选 / 位置）正确。"
                     if ok else "存在不一致，见上方明细。"))
    print("=" * 74)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
