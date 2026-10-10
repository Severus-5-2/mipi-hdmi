#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
glyph_rom_tool.py —— 画框 / 坐标 OSD 专用 16x16 字模的「生成 + 校验」工具
================================================================================
背景
----
`osd_char_lib.v` 在 2026-10-08 的中文 OSD 改版中**删掉了全部 26 个英文字母**，
只保留 0-9、':' 和 14 个汉字。而进阶2 的坐标 OSD 需要显示 `X:0640 Y:0360`，
必须用到字母 X / Y。

纪律：**不改动共享文件 `osd_char_lib.v`**（那是郭的顶层资产，且改动会影响
现有五处 OSD）。因此本模块自带一张只含所需字形的独立 ROM：
    rtl/track_glyph_rom.v

本脚本两个模式
--------------
    python glyph_rom_tool.py gen      # 由下方 ASCII 字形【生成】RTL
    python glyph_rom_tool.py check    # 【解析 RTL 原文】并渲染 ASCII 字形

为什么 check 要从 RTL 原文解析
------------------------------
这是本项目血泪教训（README 第 7 节 / 改动说明 2026-10-09）：Logo 地址那次事故
就是因为验证脚本自己手写了"应有"的公式，结果脚本和 RTL 一起错、还显示 PASS。
所以本脚本的 check 模式**只读 rtl/track_glyph_rom.v 的文本**，把里面的
16 进制常量还原成点阵再画出来，绝不使用 gen 模式里的字形表做对照。

字形坐标（与 osd_char_lib.v 保持同一点阵方向）
--------------------------------------------
    单元 16x16，点阵方向 **bit15 = 最左像素**
    数字 0-9 : 8 宽 x 12 高，占行 2..13、列 4..11
    X / Y    : 10 宽 x 12 高，占行 2..13、列 3..12
    ':'      : 2 宽，占列 7..8，点在第 5-6 行与第 9-10 行
================================================================================
"""

import os
import re
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

HERE = os.path.dirname(os.path.abspath(__file__))
RTL_PATH = os.path.normpath(os.path.join(HERE, "..", "rtl", "track_glyph_rom.v"))

CELL = 16          # 单元宽高
ROW0 = 2           # 字形起始行

# ---------------------------------------------------------------------------
# 字形表：ASCII art，'.' = 空，'#' = 亮
#   key = 字符码（直接用 ASCII 值）
# ---------------------------------------------------------------------------
GLYPHS = {
    0x30: ("0", 4, [
        "..####..",
        ".#....#.",
        "#......#",
        "#......#",
        "#......#",
        "#......#",
        "#......#",
        "#......#",
        "#......#",
        "#......#",
        ".#....#.",
        "..####..",
    ]),
    0x31: ("1", 4, [
        "...##...",
        "..###...",
        ".####...",
        "...##...",
        "...##...",
        "...##...",
        "...##...",
        "...##...",
        "...##...",
        "...##...",
        "...##...",
        ".######.",
    ]),
    0x32: ("2", 4, [
        "..####..",
        ".#....#.",
        "#......#",
        ".......#",
        "......#.",
        ".....#..",
        "....#...",
        "...#....",
        "..#.....",
        ".#......",
        "#.......",
        "########",
    ]),
    0x33: ("3", 4, [
        "..####..",
        ".#....#.",
        "#......#",
        ".......#",
        ".......#",
        "...####.",
        ".......#",
        ".......#",
        ".......#",
        "#......#",
        ".#....#.",
        "..####..",
    ]),
    0x34: ("4", 4, [
        ".....#..",
        "....##..",
        "...###..",
        "..#.##..",
        ".#..##..",
        "#...##..",
        "#...##..",
        "########",
        "....##..",
        "....##..",
        "....##..",
        "....##..",
    ]),
    0x35: ("5", 4, [
        "########",
        "#.......",
        "#.......",
        "#.......",
        "#.......",
        "#.#####.",
        "#......#",
        ".......#",
        ".......#",
        "#......#",
        ".#....#.",
        "..####..",
    ]),
    0x36: ("6", 4, [
        "...####.",
        "..#.....",
        ".#......",
        "#.......",
        "#.......",
        "#.####..",
        "##....#.",
        "#......#",
        "#......#",
        "#......#",
        ".#....#.",
        "..####..",
    ]),
    0x37: ("7", 4, [
        "########",
        ".......#",
        "......#.",
        ".....#..",
        "....#...",
        "...#....",
        "..#.....",
        "..#.....",
        ".#......",
        ".#......",
        ".#......",
        ".#......",
    ]),
    0x38: ("8", 4, [
        "..####..",
        ".#....#.",
        "#......#",
        "#......#",
        ".#....#.",
        "..####..",
        ".#....#.",
        "#......#",
        "#......#",
        "#......#",
        ".#....#.",
        "..####..",
    ]),
    0x39: ("9", 4, [
        "..####..",
        ".#....#.",
        "#......#",
        "#......#",
        "#......#",
        ".#.....#",
        "..#####.",
        ".......#",
        ".......#",
        "#......#",
        ".#....#.",
        "..####..",
    ]),
    0x58: ("X", 3, [
        "##......##",
        ".##....##.",
        "..##..##..",
        "...####...",
        "....##....",
        "....##....",
        "....##....",
        "....##....",
        "...####...",
        "..##..##..",
        ".##....##.",
        "##......##",
    ]),
    0x59: ("Y", 3, [
        "##......##",
        ".##....##.",
        "..##..##..",
        "...####...",
        "....##....",
        "....##....",
        "....##....",
        "....##....",
        "....##....",
        "....##....",
        "....##....",
        "....##....",
    ]),
}

# ':' 特殊：只有两个 2x2 点块（相对行号，绝对行 = ROW0 + 相对行）
COLON_ROWS = (3, 4, 7, 8)
GLYPHS[0x3A] = (":", 7, ["##" if r in COLON_ROWS else ".." for r in range(12)])


def art_to_hex(col0, art):
    """把 ASCII art 放到 16 宽单元里（bit15 = 最左），返回 16 行 16bit 整数。"""
    rows = [0] * CELL
    for i, line in enumerate(art):
        row = ROW0 + i
        assert 0 <= row < CELL, "字形超出单元高度"
        val = 0
        for j, ch in enumerate(line):
            if ch == "#":
                col = col0 + j
                assert 0 <= col < CELL, "字形超出单元宽度"
                val |= 1 << (15 - col)          # bit15 = 最左列
        rows[row] = val
    return rows


def gen():
    # 自检字形表本身
    for code, (name, col0, art) in GLYPHS.items():
        assert len(art) == 12, f"字形 {name} 行数 {len(art)} ≠ 12"
        w = len(art[0])
        assert all(len(l) == w for l in art), f"字形 {name} 列宽不齐"
        assert col0 + w <= CELL, f"字形 {name} 放不下"

    out = []
    out.append("//=====================================================================")
    out.append("//  track_glyph_rom.v  ——  画框 / 坐标 OSD 专用 16x16 字模")
    out.append("//---------------------------------------------------------------------")
    out.append("//  ★本文件由 sim/glyph_rom_tool.py 自动生成，**不要手改**。")
    out.append("//    改字形请编辑脚本里的 GLYPHS 表，然后：")
    out.append("//        python sim/glyph_rom_tool.py gen      # 重新生成")
    out.append("//        python sim/glyph_rom_tool.py check    # 从 RTL 原文渲染校验")
    out.append("//")
    out.append("//  为什么另起一张 ROM 而不复用 osd_char_lib.v")
    out.append("//    osd_char_lib.v 在 2026-10-08 的中文 OSD 改版中删掉了全部 26 个")
    out.append("//    英文字母（只留 0-9 + ':' + 14 个汉字）。坐标 OSD 要显示")
    out.append("//    「X:0640 Y:0360」，X / Y 无处可取。为不改动郭的顶层共享文件，")
    out.append("//    本模块自带只含所需字形的独立 ROM。")
    out.append("//    若后续郭愿意把 X / Y 两个字模并进 osd_char_lib.v，可直接删掉本文件。")
    out.append("//")
    out.append("//  接口    I_char(字符码) + I_row(行号 0..15)  ->  O_row_bits(该行 16bit 点阵)")
    out.append("//  点阵方向 bit15 = 最左像素（与 osd_char_lib.v 一致）")
    out.append("//  字形占位 数字 8x12 @ 列4..11、行2..13；X/Y 10x12 @ 列3..12；':' @ 列7..8")
    out.append("//  纯组合读（同 osd_char_lib.v 的写法，由调用方在第 2 拍取用）")
    out.append("//=====================================================================")
    out.append("module track_glyph_rom (")
    out.append("    input  wire [7:0]  I_char,")
    out.append("    input  wire [3:0]  I_row,")
    out.append("    output reg  [15:0] O_row_bits")
    out.append(");")
    out.append("")
    out.append("    // 字符码：0x30..0x39 = '0'..'9'，0x3a = ':'，0x58 = 'X'，0x59 = 'Y'")
    out.append("    always @(*) begin")
    out.append("        case(I_char)")

    for code in sorted(GLYPHS):
        name, col0, art = GLYPHS[code]
        rows = art_to_hex(col0, art)
        out.append(f"            //================= '{name}' (0x{code:02x}) =================")
        out.append(f"            8'h{code:02x}: begin")
        out.append("                case(I_row)")
        for r, v in enumerate(rows):
            if v:
                out.append(f"                    4'd{r}: O_row_bits = 16'h{v:04x};")
        out.append("                    default: O_row_bits = 16'h0000;")
        out.append("                endcase")
        out.append("            end")
        out.append("")

    out.append("            default: O_row_bits = 16'h0000;")
    out.append("        endcase")
    out.append("    end")
    out.append("")
    out.append("endmodule")

    with open(RTL_PATH, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    print(f"[gen] 已生成 {RTL_PATH}")
    print(f"[gen] 共 {len(GLYPHS)} 个字形："
          + " ".join(f"'{GLYPHS[c][0]}'" for c in sorted(GLYPHS)))


# ---------------------------------------------------------------------------
# check：只读 RTL 原文
# ---------------------------------------------------------------------------
RE_CHAR = re.compile(r"8'h([0-9a-fA-F]{2})\s*:\s*begin")
RE_ROW = re.compile(r"4'd(\d+)\s*:\s*O_row_bits\s*=\s*16'h([0-9a-fA-F]{4})")


def parse_rtl(path):
    """解析 RTL，返回 {字符码: {行号: 16bit 值}}（缺省行 = 0）。"""
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()

    glyphs = {}
    cur = None
    for line in text.splitlines():
        m = RE_CHAR.search(line)
        if m:
            cur = int(m.group(1), 16)
            glyphs[cur] = {}
            continue
        m = RE_ROW.search(line)
        if m and cur is not None:
            glyphs[cur][int(m.group(1))] = int(m.group(2), 16)
    return glyphs


def render(rows_map):
    """把 {行号: 值} 渲染成 ASCII，bit15 = 最左。返回 16 行字符串列表。"""
    lines = []
    for r in range(CELL):
        v = rows_map.get(r, 0)
        lines.append("".join("#" if (v >> (15 - c)) & 1 else "." for c in range(CELL)))
    return lines


def check():
    if not os.path.exists(RTL_PATH):
        print(f"[check] 找不到 {RTL_PATH}，先跑 gen")
        return 2

    glyphs = parse_rtl(RTL_PATH)
    print("=" * 60)
    print("track_glyph_rom.v  —— 从 RTL 原文解析并渲染（bit15 = 最左）")
    print("=" * 60)

    if not glyphs:
        print("[check] 解析不到任何字形，RTL 格式可能被改动了")
        return 2

    # 逐字形并列渲染，便于一眼比对
    order = sorted(glyphs)
    print(f"共解析到 {len(glyphs)} 个字形。逐个渲染：\n")

    for code in order:
        rows_map = glyphs[code]
        art = render(rows_map)
        ch = chr(code) if 32 <= code < 127 else "?"
        used = [r for r in range(CELL) if rows_map.get(r, 0)]
        lo, hi = (min(used), max(used)) if used else (None, None)

        # 有效列范围（按位）
        cols = []
        for r in range(CELL):
            v = rows_map.get(r, 0)
            for c in range(CELL):
                if (v >> (15 - c)) & 1:
                    cols.append(c)
        clo, chi = (min(cols), max(cols)) if cols else (None, None)

        print(f"--- 0x{code:02x} '{ch}' "
              f"行 {lo}..{hi}  列 {clo}..{chi}  亮像素 {sum(bin(v).count('1') for v in rows_map.values())} ---")
        for i, line in enumerate(art):
            mark = "*" if rows_map.get(i, 0) else " "
            print(f"  {i:2d}{mark}{line}")
        print()

    # 一致性检查：同一字符码不应出现两次 case 分支
    with open(RTL_PATH, "r", encoding="utf-8") as f:
        codes = [int(m.group(1), 16) for m in RE_CHAR.finditer(f.read())]
    dup = {c for c in codes if codes.count(c) > 1}
    print("=" * 60)
    if dup:
        print(f"[FAIL] 重复的 case 分支字符码: {[hex(c) for c in dup]}")
        return 1
    print("[PASS] 无重复 case 分支；解析完成。")
    return 0


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "check"
    if mode == "gen":
        gen()
        sys.exit(0)
    elif mode == "check":
        sys.exit(check())
    else:
        print(__doc__)
        sys.exit(2)
