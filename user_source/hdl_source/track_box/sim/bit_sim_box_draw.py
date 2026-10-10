#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bit_sim_box_draw.py —— box_draw 的第三方独立实现 + 交叉验证 + 上板预览
================================================================================
为什么要有第三份实现
--------------------
box_draw 的正确性目前由两份东西把关：
    · rtl/box_draw.v            —— RTL 本体（第 1 份实现）
    · tb/tb_box_draw.v 的 exp_color —— 仿真器里的期望模型（第 2 份实现）
本项目吃过"两份代码同错"的亏（README 第 7 节：验证脚本手写了"应有"的公式，
结果脚本和 RTL 一起错、还报 PASS）。所以再加第 3 份：本脚本。

三种模式
--------
    python bit_sim_box_draw.py check     # 读 TB 的 dump，与本脚本模型逐像素比对
    python bit_sim_box_draw.py preview   # 渲染 1280x720 SELF_TEST 固定框屏幕（ASCII）
    python bit_sim_box_draw.py both      # 两个都做（默认）

依赖
----
    check 模式需要先跑过 tb：
        iverilog -g2005 -o sim/tb_box_draw.vvp \\
                 tb/tb_box_draw.v rtl/box_draw.v rtl/pix_coord_gen.v
        vvp sim/tb_box_draw.vvp
    会在 sim/tb_box_draw_out.txt 产出 "x y hit color" 的逐像素 dump。
================================================================================
"""

import os
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

HERE = os.path.dirname(os.path.abspath(__file__))
DUMP = os.path.join(HERE, "tb_box_draw_out.txt")

# ---------------------------------------------------------------------------
# 一、TB 小画面的模型参数
#    ★必须与 tb/tb_box_draw.v 顶部的 localparam 一字不差
# ---------------------------------------------------------------------------
TB_H_ACTIVE, TB_V_ACTIVE = 64, 32
TB_BOXES = [
    # (x0, y0, x1, y1, color)
    (4,  4,  20, 14, 0xFF0000),
    (40, 4,  63, 14, 0x00FF00),
    (4,  20, 20, 31, 0x00A0FF),
    (28, 12, 44, 24, 0xFFF200),
]
TB_CR_X, TB_CR_Y, TB_CR_HALF = 36, 18, 5
TB_COLOR_CROSS = 0xFFFFFF
TB_BORDER_W = 2

# ---------------------------------------------------------------------------
# 二、上板（1280x720）SELF_TEST 固定框参数
#    ★必须与 rtl/box_draw.v 顶部注释里的固定框坐标一字不差
# ---------------------------------------------------------------------------
ST_W, ST_H = 1280, 720
ST_BOXES = [
    (64,   64,  256,  256, 0xFF0000, "B0 左上"),
    (1088, 64,  1279, 256, 0x00FF00, "B1 右上（右边贴屏边）"),
    (64,   464, 256,  656, 0x00A0FF, "B2 左下（与 B0 关于屏心对称）"),
    (510,  290, 770,  430, 0xFFF200, "B3 中央（框心 = 屏幕中心）"),
]
ST_CR_X, ST_CR_Y, ST_CR_HALF = 640, 360, 16
ST_COLOR_CROSS = 0xFFFFFF


# ---------------------------------------------------------------------------
# 三、独立实现的叠加模型（按规格朴素写，不与 RTL 的并行表达式同构）
# ---------------------------------------------------------------------------
def border_hit(x, y, x0, y0, x1, y1, bw=2):
    """某一框的边框是否覆盖像素 (x,y)。边框向内 bw 像素。

    ★每条边都必须有完整的区间上下界。第一版漏了 `x >= x0` / `y >= y0` 的下界，
      写成 `(iny and (x <= x0+1 or x >= x1-1)) or (inx and (y <= y0+1 or y >= y1-1))`，
      结果框**上方**与**左侧**整片被误判成边框，在 1280x720 上拖出一大块色斑。
      更糟的是 RTL / TB / Python 三份实现来自同一个错误心智模型，互相"验证通过"了。
      最后是 ASCII 还原一眼看出来的 ⇒ 空间图案必须出图肉眼过。
    """
    if (x1 - x0) <= 3 or (y1 - y0) <= 3:
        return False                       # 退化框不画
    in_x = x0 <= x <= x1
    in_y = y0 <= y <= y1
    edge_l = (x >= x0) and (x <= x0 + bw - 1)       # 左边 bw 列
    edge_r = (x >= x1 - bw + 1) and (x <= x1)       # 右边 bw 列
    edge_t = (y >= y0) and (y <= y0 + bw - 1)       # 上边 bw 行
    edge_b = (y >= y1 - bw + 1) and (y <= y1)       # 下边 bw 行
    return (in_y and (edge_l or edge_r)) or (in_x and (edge_t or edge_b))


def cross_hit(x, y, cx, cy, half, w=2):
    """十字准星是否覆盖像素 (x,y)。臂宽 w 像素，四臂半长 half。"""
    v = (cx <= x <= cx + w - 1) and (cy - half <= y <= cy + half)
    h = (cy <= y <= cy + w - 1) and (cx - half <= x <= cx + half)
    return v or h


def overlay_color(x, y, boxes, cr, border_w=2, color_cross=0xFFFFFF):
    """返回该像素应叠加的颜色，0 表示不叠加。优先级：准星 > B0 > B1 > B2 > B3。"""
    cx, cy, half = cr
    if cross_hit(x, y, cx, cy, half):
        return color_cross
    for (x0, y0, x1, y1, color) in boxes:
        if border_hit(x, y, x0, y0, x1, y1, border_w):
            return color
    return 0


def render(boxes, cr, w, h, color_cross=0xFFFFFF, scale_x=8, scale_y=16):
    """把叠加结果缩略成 ASCII：一个字符代表 scale_x x scale_y 的块，有叠加就画。"""
    cols = (w + scale_x - 1) // scale_x
    rows = (h + scale_y - 1) // scale_y
    grid = [[" "] * cols for _ in range(rows)]

    # 先铺框（按颜色区分字符），再铺准星（'+'），准星优先
    glyph = {0xFF0000: "0", 0x00FF00: "1", 0x00A0FF: "2", 0xFFF200: "3"}
    for (x0, y0, x1, y1, color) in boxes:
        ch = glyph.get(color, "#")
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                if border_hit(x, y, x0, y0, x1, y1):
                    grid[y // scale_y][x // scale_x] = ch
    cx, cy, half = cr
    for y in range(max(0, cy - half), min(h, cy + half + 1)):
        for x in range(max(0, cx - half), min(w, cx + half + 1)):
            if cross_hit(x, y, cx, cy, half):
                grid[y // scale_y][x // scale_x] = "+"

    header = "    " + "".join(str((c * scale_x // 100) % 10) for c in range(cols))
    lines = [header]
    for r, row in enumerate(grid):
        lines.append(f"{r * scale_y:4d}" + "".join(row))
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 四、check：读 TB dump 交叉验证
# ---------------------------------------------------------------------------
def check():
    if not os.path.exists(DUMP):
        print(f"[check] 找不到 {DUMP}")
        print("        请先在 track_box 目录下跑：")
        print("          iverilog -g2005 -o sim/tb_box_draw.vvp \\")
        print("                   tb/tb_box_draw.v rtl/box_draw.v rtl/pix_coord_gen.v")
        print("          vvp sim/tb_box_draw.vvp")
        return 2

    print("=" * 74)
    print("box_draw 交叉验证：TB dump  vs  本脚本独立模型")
    print("=" * 74)

    dut = {}
    with open(DUMP, "r", encoding="utf-8") as f:
        for line in f:
            parts = line.split()
            if len(parts) != 4:
                continue
            x, y, hit, color = int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3], 16)
            dut[(x, y)] = (hit, color)

    print(f"  dump 像素数 : {len(dut)}（去重后）")

    # ---- 逐像素比对 ----
    mismatches = []
    for y in range(TB_V_ACTIVE):
        for x in range(TB_H_ACTIVE):
            exp_c = overlay_color(x, y, TB_BOXES, (TB_CR_X, TB_CR_Y, TB_CR_HALF))
            exp_h = 1 if exp_c else 0
            got = dut.get((x, y))
            if got is None:
                mismatches.append((x, y, "dump 缺失", None, (exp_h, exp_c)))
            elif got != (exp_h, exp_c):
                mismatches.append((x, y, "不一致", got, (exp_h, exp_c)))

    ok = not mismatches
    print(f"  逐像素比对 : {TB_H_ACTIVE * TB_V_ACTIVE} 个像素, "
          f"不一致 {len(mismatches)}   [{'PASS' if ok else 'FAIL'}]")
    for m in mismatches[:10]:
        print(f"    (x={m[0]}, y={m[1]}) {m[2]}: DUT={m[3]} 期望={m[4]}")

    # ---- 结构检查：边框宽度必须恰好 2，且【不得越出框外】----
    print()
    print("  结构检查（边框恰好 2px、退化框不画、且不得越出框外）：")
    structural_ok = True
    for i, (x0, y0, x1, y1, color) in enumerate(TB_BOXES):
        w = x1 - x0 + 1
        h = y1 - y0 + 1
        # 上边 2 行全宽 + 下边 2 行全宽 + 左右各 2 列 x (h-4) 行
        exp_cnt = 2 * w * 2 + 2 * (h - 4) * 2
        # ★必须扫【全画面】。第一版只扫框内 [x0..x1]x[y0..y1]，
        #   于是"框外大片误判"这个 bug 被完整地盖住了，检查报 PASS。
        pts = [(x, y) for y in range(TB_V_ACTIVE) for x in range(TB_H_ACTIVE)
               if border_hit(x, y, x0, y0, x1, y1)]
        outside = [p for p in pts
                   if not (x0 <= p[0] <= x1 and y0 <= p[1] <= y1)]
        good = (len(pts) == exp_cnt) and (not outside)
        structural_ok &= good
        print(f"    框{i} ({x0},{y0})-({x1},{y1}) w={w} h={h}: "
              f"全画面命中 {len(pts)} / 解析式 {exp_cnt} / "
              f"越界 {len(outside)}  [{'PASS' if good else 'FAIL'}]")
        if outside:
            print(f"      越界样例(前 5): {outside[:5]}")
    print(f"    自洽性检查 [{'PASS' if structural_ok else 'FAIL'}]")
    print()

    print("-" * 74)
    print("TB dump 的 ASCII 还原（每个字符 = 1 像素，'#'=叠加，'.'=画面）：")
    for y in range(TB_V_ACTIVE):
        row = "".join("#" if dut.get((x, y), (0, 0))[0] else "." for x in range(TB_H_ACTIVE))
        print("    " + row)
    print("-" * 74)

    all_ok = ok and structural_ok
    print()
    print("=" * 74)
    print("结论：" + ("三份实现（RTL / TB 模型 / 本脚本）结果完全一致。"
                     if all_ok else "存在不一致，见上方明细。"))
    print("=" * 74)
    return 0 if all_ok else 1


# ---------------------------------------------------------------------------
# 五、preview：1280x720 SELF_TEST 屏幕
# ---------------------------------------------------------------------------
def preview():
    print("=" * 74)
    print("SELF_TEST 固定框屏幕预览（1280x720，box_draw.v 的 parameter 默认值）")
    print("=" * 74)
    print()
    print("  字符含义:  0/1/2/3 = B0..B3 的框边    + = 十字准星    ' ' = 画面")
    print("  缩略比例:  1 字符 = 8 x 16 像素")
    print()

    boxes = [(x0, y0, x1, y1, c) for (x0, y0, x1, y1, c, _n) in ST_BOXES]
    print(render(boxes, (ST_CR_X, ST_CR_Y, ST_CR_HALF), ST_W, ST_H))
    print()

    print("-" * 74)
    print("上板对照表（D1 验证坐标系统就靠这张表）：")
    print(f"  {'框':<4}{'xmin':>6}{'ymin':>6}{'xmax':>6}{'ymax':>6}"
          f"{'框心X':>8}{'框心Y':>8}   说明")
    for (x0, y0, x1, y1, color, name) in ST_BOXES:
        print(f"  {name[:2]:<4}{x0:>6}{y0:>6}{x1:>6}{y1:>6}"
              f"{(x0 + x1) / 2:>8.0f}{(y0 + y1) / 2:>8.0f}   {name[3:]}")
    print()
    print(f"  十字准星 : ({ST_CR_X}, {ST_CR_Y})，臂长 ±{ST_CR_HALF} px，臂宽 2 px")
    print(f"  osd_coord 默认显示 : X:{ST_CR_X:04d}  Y:{ST_CR_Y:04d}")
    print()
    print("  ★ 上板自检口诀：")
    print("    1) 十字准星是否正好落在 B3 的正中心？")
    print("    2) OSD 读数是 X:0640 / Y:0360 吗？")
    print("    3) B1 的右边框是否贴着屏幕最右列（x=1279）而不被裁掉？")
    print("    4) B0 与 B2 是否关于屏幕水平中线（y=360）对称？")
    print("    四条都过 → 坐标系统与画框对齐都对；任一条错 → 先查坐标源，再查延时拍数。")
    print("-" * 74)
    return 0


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "both"
    rc = 0
    if mode in ("check", "both"):
        rc |= check()
    if mode in ("preview", "both"):
        if mode == "both":
            print()
        rc |= preview()
    sys.exit(rc)
