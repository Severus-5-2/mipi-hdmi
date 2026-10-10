#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bit_sim_system.py —— track_box 系统级仿真的第三方独立核对 + 出图
================================================================================
输入
----
    sim/tb_system_frame.bin     ← tb/tb_track_system.v 跑出来的整帧位图
                                   （1280x720，每像素 1 字节颜色码）

做什么
------
1. **独立重算整帧**：不读任何 RTL 中间量，只用「几何 + 字模」从零算一遍
   1280x720 每个像素应有的颜色码，与 RTL dump 逐像素比对。
   —— 这是「第三方实现」，与 RTL / TB 都不同源。
2. 字模**从 rtl/track_glyph_rom.v 原文解析**（复用 glyph_rom_tool.parse_rtl），
   不手抄，避免「脚本和 RTL 一起错还报 PASS」（项目血泪教训）。
3. 输出 PNG：整帧 + 4 张局部放大（B0 左上角 / B1 贴屏右边 / B3+准星中心 /
   OSD 文字），供**肉眼过一遍**——空间图案只看计数是抓不住 bug 的。

颜色码约定（与 tb_track_system.v 一致）
    0=背景 1=框0红 2=框1绿 3=框2蓝 4=框3黄 5=准星白 6=坐标OSD

用法
----
    cd user_source/hdl_source/track_box
    iverilog -g2005 -o sim/tb_system.vvp tb/tb_track_system.v \
             rtl/pix_coord_gen.v rtl/box_draw.v rtl/osd_coord.v rtl/track_glyph_rom.v
    vvp sim/tb_system.vvp
    python sim/bit_sim_system.py
================================================================================
"""

import os
import struct
import sys
import zlib

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from glyph_rom_tool import parse_rtl, RTL_PATH          # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
BIN  = os.path.join(HERE, "tb_system_frame.bin")

W, H = 1280, 720

# ---------------------------------------------------------------------------
# 与 RTL 一字不差的几何常量
#   box_draw.v  SELF_TEST=1 的 4 个固定框 + 十字准星
#   osd_coord.v 默认布局（COORD_X0=16 / LINE1_Y=688 / LINE2_Y=704）
# ---------------------------------------------------------------------------
BOXES = [
    (64,   64,  256, 256, 1),     # B0 红
    (1088, 64,  1279, 256, 2),    # B1 绿（右边框贴屏边 x=1279）
    (64,   464, 256, 656, 3),     # B2 蓝
    (510,  290, 770, 430, 4),     # B3 黄（框心 = 屏幕中心）
]
BORDER_W  = 2
CROSS_X, CROSS_Y, CROSS_HALF = 640, 360, 16
CROSS_CODE = 5

OSD_X0, OSD_LINE1_Y, OSD_LINE2_Y = 16, 688, 704
OSD_CELL, OSD_NCHAR = 16, 6
OSD_TEXT1, OSD_TEXT2 = "X:0640", "Y:0360"


# ---------------------------------------------------------------------------
# 颜色码 → RGB（出图用，取上板真实观感色）
# ---------------------------------------------------------------------------
CODE_RGB = {
    0: (32, 32, 32),        # 背景
    1: (255, 0, 0),         # 红
    2: (0, 255, 0),         # 绿
    3: (0, 160, 255),       # 蓝
    4: (255, 242, 0),       # 黄
    5: (255, 255, 255),     # 白
    6: (255, 242, 0),       # OSD 黄（与框3同色，但位置不同区域）
}


# ---------------------------------------------------------------------------
# 独立几何模型
# ---------------------------------------------------------------------------
def border_hit(x, y, x0, y0, x1, y1, bw=BORDER_W):
    """某一框的边框是否覆盖 (x,y)。要求每条边都有完整上下界，退化框不画。"""
    if (x1 - x0) <= 3 or (y1 - y0) <= 3:
        return False
    in_x = x0 <= x <= x1
    in_y = y0 <= y <= y1
    on_v = in_y and (x0 <= x <= x0 + bw - 1 or x1 - bw + 1 <= x <= x1)
    on_h = in_x and (y0 <= y <= y0 + bw - 1 or y1 - bw + 1 <= y <= y1)
    return on_v or on_h


def cross_hit(x, y):
    v = (CROSS_X <= x <= CROSS_X + 1) and (CROSS_Y - CROSS_HALF <= y <= CROSS_Y + CROSS_HALF)
    h = (CROSS_Y <= y <= CROSS_Y + 1) and (CROSS_X - CROSS_HALF <= x <= CROSS_X + CROSS_HALF)
    return v or h


def build_osd_points(glyphs):
    """按 osd_coord.v 的布局与位选规则，算出 OSD 点亮的像素集合。"""
    pts = set()
    for text, y0 in ((OSD_TEXT1, OSD_LINE1_Y), (OSD_TEXT2, OSD_LINE2_Y)):
        for idx, ch in enumerate(text):
            code = ord(ch)
            if code not in glyphs:
                raise KeyError(f"字模里没有字符 {ch!r} (0x{code:02x})")
            rows = glyphs[code]
            ox = OSD_X0 + idx * OSD_CELL
            for r in range(OSD_CELL):                     # row = y 的低 4 位
                v = rows.get(r, 0)
                for c in range(OSD_CELL):                 # lx = x 的低 4 位
                    if (v >> (15 - c)) & 1:               # bit15 = 最左
                        pts.add((ox + c, y0 + r))
    return pts


def expected_codes(glyphs):
    """整帧独立重算，返回 bytearray(W*H)。优先级：准星 > B0 > B1 > B2 > B3 > OSD。"""
    osd_pts = build_osd_points(glyphs)

    # 先按「box 层」逐一绘制（低优先级先画、高优先级覆盖）
    codes = bytearray(W * H)
    for (x0, y0, x1, y1, code) in BOXES:                  # 绘制顺序即覆盖优先级：后画的赢
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                if border_hit(x, y, x0, y0, x1, y1):
                    codes[y * W + x] = code
    # 准星最高（覆盖所有框）
    for (x, y) in [(x, y) for y in range(H) for x in range(W) if cross_hit(x, y)]:
        codes[y * W + x] = CROSS_CODE

    # OSD 只在「未被任何 box/准星覆盖」处生效（顶层约定 box 优先于 osd）
    for (x, y) in osd_pts:
        if codes[y * W + x] == 0:
            codes[y * W + x] = 6
    return codes


# ---------------------------------------------------------------------------
# PNG 写入（标准库 zlib，无 PIL 依赖）
# ---------------------------------------------------------------------------
def write_png(path, w, h, rgb):
    raw = bytearray()
    stride = w * 3
    for y in range(h):
        raw.append(0)                                     # filter type 0
        raw += rgb[y * stride:(y + 1) * stride]

    def chunk(typ, data):
        return (struct.pack(">I", len(data)) + typ + data
                + struct.pack(">I", zlib.crc32(typ + data) & 0xffffffff))

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(bytes(raw), 9))
           + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(png)


def codes_to_rgb(codes, w, h):
    out = bytearray()
    for c in codes:
        r, g, b = CODE_RGB.get(c, (255, 0, 255))
        out += bytes((r, g, b))
    return bytes(out)


def crop_scale(codes, x0, y0, x1, y1, scale):
    """裁剪 [x0,x1]x[y0,y1] 并按 scale 最近邻放大，返回 (rgb, w, h)。"""
    cw, ch = x1 - x0 + 1, y1 - y0 + 1
    ow, oh = cw * scale, ch * scale
    rgb = bytearray(ow * oh * 3)
    for oy in range(oh):
        sy = y0 + oy // scale
        for ox in range(ow):
            sx = x0 + ox // scale
            c = codes[sy * W + sx]
            r, g, b = CODE_RGB.get(c, (255, 0, 255))
            p = (oy * ow + ox) * 3
            rgb[p] = r; rgb[p + 1] = g; rgb[p + 2] = b
    return bytes(rgb), ow, oh


# ---------------------------------------------------------------------------
def main():
    if not os.path.exists(BIN):
        print(f"[FAIL] 找不到 {BIN}")
        print("        请先在 track_box 目录下跑：")
        print("          iverilog -g2005 -o sim/tb_system.vvp tb/tb_track_system.v \\")
        print("                   rtl/pix_coord_gen.v rtl/box_draw.v rtl/osd_coord.v rtl/track_glyph_rom.v")
        print("          vvp sim/tb_system.vvp")
        return 2

    glyphs = parse_rtl(RTL_PATH)
    if not glyphs:
        print(f"[FAIL] 从 {RTL_PATH} 解析不到字模")
        return 2

    dut = open(BIN, "rb").read()
    print("=" * 78)
    print(" track_box 系统级仿真 —— 第三方独立核对（整帧 1280x720）")
    print("=" * 78)
    print(f"  dump 文件 : {os.path.basename(BIN)}  {len(dut)} 字节"
          f"（期望 {W*H}）  [{'PASS' if len(dut)==W*H else 'FAIL'}]")
    if len(dut) != W * H:
        return 1

    exp = expected_codes(glyphs)

    # ---- 逐像素比对 ----
    diff = [i for i in range(W * H) if dut[i] != exp[i]]
    print(f"  逐像素比对 : {W*H} 个像素, 不一致 {len(diff)}   "
          f"[{'PASS' if not diff else 'FAIL'}]")
    if diff[:8]:
        for i in diff[:8]:
            print(f"     ({i % W},{i // W})  DUT={dut[i]}  期望={exp[i]}")

    # ---- 分色统计（DUT 与期望必须逐项相等）----
    names = {0: "背景", 1: "框0红", 2: "框1绿", 3: "框2蓝", 4: "框3黄", 5: "准星白", 6: "OSD"}
    print()
    print("  分色统计（DUT / 期望）：")
    stat_ok = True
    for c in range(7):
        n_d = dut.count(c)
        n_e = exp.count(c)
        good = (n_d == n_e)
        stat_ok &= good
        print(f"    {names[c]:<6} : {n_d:>7} / {n_e:>7}   [{'PASS' if good else 'FAIL'}]")

    # ---- 结构硬指标 ----
    print()
    print("  结构检查：")
    cross_pts = sum(1 for y in range(H) for x in range(W) if cross_hit(x, y))
    print(f"    准星理论像素数            : {cross_pts}（竖 2x33 + 横 33x2 - 重叠 4 = 128）")
    # B1 右边框是否贴到 x=1279
    col1279 = [y for y in range(H) if dut[y * W + 1279] == 2]
    print(f"    B1 右边框在 x=1279 的行数 : {len(col1279)}（期望 {256-64+1}=193 行）")
    # B0 与 B2 是否关于 y=360 对称（顶上边 vs 底下边）
    top0 = [y for y in range(H) if dut[y * W + 64] == 1]
    bot2 = [y for y in range(H) if dut[y * W + 64] == 3]
    print(f"    B0 左边框 y 范围          : {min(top0)}..{max(top0)}")
    print(f"    B2 左边框 y 范围          : {min(bot2)}..{max(bot2)}")

    # ---- 出图 ----
    print()
    print("  生成 PNG（供肉眼过一遍）：")
    full_rgb = codes_to_rgb(dut, W, H)
    p1 = os.path.join(HERE, "system_full.png")
    write_png(p1, W, H, full_rgb)
    print(f"    整帧原尺寸 : {os.path.basename(p1)}")

    shots = [
        ("system_A_B0_topleft.png", 48, 48, 280, 280, 2, "B0 左上角框（红）+ B3 左半"),
        ("system_B_B1_right.png",   1060, 48, 1279, 280, 2, "B1 右边框贴屏（绿，x=1279）"),
        ("system_C_B3_cross.png",   600, 330, 690, 395, 6, "B3 框心 + 十字准星（白）"),
        ("system_D_osd.png",        8, 680, 120, 719, 6, "坐标 OSD 两行 X:0640 / Y:0360"),
    ]
    for (fn, x0, y0, x1, y1, sc, desc) in shots:
        rgb, ow, oh = crop_scale(dut, x0, y0, x1, y1, sc)
        p = os.path.join(HERE, fn)
        write_png(p, ow, oh, rgb)
        print(f"    {fn:<28} {ow}x{oh}   ← {desc}")

    print()
    print("=" * 78)
    ok = (not diff) and stat_ok and len(dut) == W * H
    print(" 结论：" + ("系统级整帧叠加与独立模型逐像素完全一致；位图已出，请肉眼复核。"
                       if ok else "存在不一致，见上方明细。"))
    print("=" * 78)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
