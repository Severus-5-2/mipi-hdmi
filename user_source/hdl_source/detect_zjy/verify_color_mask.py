#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_color_mask.py — detect_color_mask.v 的比特级对拍验证

设计原则（遵循工程 §7「验证脚本必须解析派」的约定）：
    本脚本 **不手抄任何系数**，而是从 RTL 原文用正则提取：
      · 9 个 YCbCr 常量系数（含符号）
      · 四舍五入常数、算术右移位数、色度偏置
      · 限幅上下界
      · 四个颜色各 4 个阈值 + 亮度门限
    然后用「补码 + 算术右移 + 四舍五入 + 限幅」的比特级语义复算，
    最后与 ModelSim 跑出来的 rtl_out.txt 逐像素比对。

    这样即使有人改了 RTL 里的系数而忘了同步验证脚本，本脚本也会自动跟着变，
    不会出现「脚本与 RTL 同错」的漏报。

用法：
    python verify_color_mask.py [--rtl detect_color_mask.v] [--out rtl_out.txt]
"""

import argparse
import os
import re
import sys

# ---------------------------------------------------------------------------
# 与 testbench 保持一致的激励构造（这是激励，不是 RTL 逻辑，允许镜像）
# ---------------------------------------------------------------------------
W = 8
H = 34
N = W * H


def build_vectors():
    vec = []
    for i in range(256):
        vec.append((i, i, i))                 # 灰阶扫描
    vec += [
        (0xFF, 0x00, 0x00),   # 纯红
        (0x00, 0xFF, 0x00),   # 纯绿
        (0x00, 0x00, 0xFF),   # 纯蓝
        (0x00, 0x00, 0x00),   # 纯黑
        (0xFF, 0xFF, 0xFF),   # 纯白
        (0xC8, 0x3C, 0x3C),   # 拟真红球
        (0x3C, 0x3C, 0xC8),   # 拟真蓝方块
        (0x3C, 0xC8, 0x3C),   # 拟真绿三角
        (0x80, 0x50, 0x30),   # 棕
        (0x80, 0x80, 0x80),   # 中灰
    ]
    while len(vec) < N:
        vec.append((0, 0, 0))
    return vec


# ---------------------------------------------------------------------------
# RTL 解析
# ---------------------------------------------------------------------------
class RtlParams(object):
    def __init__(self, path):
        with open(path, "r", encoding="utf-8", errors="replace") as fp:
            self.src = fp.read()

        # ---- 1. 9 个常量系数 ----
        #     形如： p_y_r  <=  18'sd77  * r_ext;
        #            p_cb_r <= -18'sd43  * r_ext;
        self.coef = {}   # coef[(comp, channel)] = signed value
        pat_c = re.compile(
            r"p_(y|cb|cr)_([rgb])\s*<=\s*(-?)\s*18'sd(\d+)\s*\*\s*([rgb])_ext")
        for m in pat_c.finditer(self.src):
            comp, reg_ch, sign, val, src_ch = m.groups()
            if reg_ch != src_ch:
                raise RuntimeError(
                    "乘积寄存器 %s_%s 的源通道是 %s_ext，疑似 RTL 写错" % (comp, reg_ch, src_ch))
            v = int(val)
            if sign == "-":
                v = -v
            self.coef[(comp, src_ch)] = v

        need = [(c, ch) for c in ("y", "cb", "cr") for ch in "rgb"]
        missing = [k for k in need if k not in self.coef]
        if missing:
            raise RuntimeError("未能从 RTL 解析出以下系数：%s" % missing)

        # ---- 2. 四舍五入常数与右移位数 ----
        m = re.search(r"y_sum\s*\+\s*18'sd(\d+)\s*\)\s*>>>\s*(\d+)", self.src)
        if not m:
            raise RuntimeError("未能解析 Y 的四舍五入/右移结构")
        self.round_const = int(m.group(1))
        self.shift = int(m.group(2))

        # ---- 3. 色度偏置 ----
        m = re.search(r">>>\s*\d+\s*\)\s*\+\s*10'sd(\d+)", self.src)
        if not m:
            raise RuntimeError("未能解析色度偏置 (+128)")
        self.chroma_offset = int(m.group(1))

        # ---- 4. 限幅上下界（以 Y 的限幅为准）----
        m_hi = re.search(r"q_y\s*>\s*10'sd(\d+)", self.src)
        m_lo = re.search(r"q_y\s*<\s*10'sd(\d+)", self.src)
        if not (m_hi and m_lo):
            raise RuntimeError("未能解析限幅结构")
        self.clamp_hi = int(m_hi.group(1))
        self.clamp_lo = int(m_lo.group(1))

        # ---- 5. 阈值参数（只在模块头部 parameter 区里找）----
        header = self.src.split(")(", 1)[0]
        self.param = {}
        for m in re.finditer(r"\b([A-Z][A-Z0-9_]*)\s*=\s*8'd(\d+)", header):
            self.param[m.group(1)] = int(m.group(2))

        for name in ("Y_MIN",
                     "RED_CB_LO", "RED_CB_HI", "RED_CR_LO", "RED_CR_HI",
                     "BLUE_CB_LO", "BLUE_CB_HI", "BLUE_CR_LO", "BLUE_CR_HI",
                     "GREEN_CB_LO", "GREEN_CB_HI", "GREEN_CR_LO", "GREEN_CR_HI"):
            if name not in self.param:
                raise RuntimeError("未能解析阈值参数 %s" % name)

    # -----------------------------------------------------------------
    def dump(self):
        print("[RTL 解析结果]  （系数表，符号已还原）")
        print("        R     G     B")
        for c, label in (("y", "Y "), ("cb", "Cb"), ("cr", "Cr")):
            print("   %s %5d %5d %5d" % (
                label, self.coef[(c, "r")], self.coef[(c, "g")], self.coef[(c, "b")]))
        print("   四舍五入 +%d, 算术右移 >>%d, 色度偏置 +%d, 限幅 %d~%d"
              % (self.round_const, self.shift, self.chroma_offset,
                 self.clamp_lo, self.clamp_hi))
        print("   阈值: " + ", ".join(
            "%s=%d" % (k, self.param[k]) for k in
            ("Y_MIN",
             "RED_CB_LO", "RED_CB_HI", "RED_CR_LO", "RED_CR_HI",
             "BLUE_CB_LO", "BLUE_CB_HI", "BLUE_CR_LO", "BLUE_CR_HI",
             "GREEN_CB_LO", "GREEN_CB_HI", "GREEN_CR_LO", "GREEN_CR_HI")))
        print()

    # -----------------------------------------------------------------
    def model(self, r, g, b):
        """比特级镜像：与 RTL 逐位等价。

        说明：RTL 里各中间量位宽都留足（18bit 有符号、10bit 有符号），
        实测值域不溢出，因此 Python 无界整数 + 算术右移（对负数向下取整，
        与 Verilog >>> 一致）与 RTL 结果完全相同。
        """
        c = self.coef
        y_sum = c[("y", "r")] * r + c[("y", "g")] * g + c[("y", "b")] * b
        cb_sum = c[("cb", "r")] * r + c[("cb", "g")] * g + c[("cb", "b")] * b
        cr_sum = c[("cr", "r")] * r + c[("cr", "g")] * g + c[("cr", "b")] * b

        q_y = (y_sum + self.round_const) >> self.shift
        q_cb = ((cb_sum + self.round_const) >> self.shift) + self.chroma_offset
        q_cr = ((cr_sum + self.round_const) >> self.shift) + self.chroma_offset

        def clamp(v):
            return self.clamp_hi if v > self.clamp_hi else (
                self.clamp_lo if v < self.clamp_lo else v)

        # ⚠ 掩膜必须用「限幅后」的值比较：
        #   q_cb/q_cr 值域可达 256，而阈值上限只能填 255，
        #   用未限幅值比较会把最饱和的纯红/纯蓝漏掉。
        cy, ccb, ccr = clamp(q_y), clamp(q_cb), clamp(q_cr)

        p = self.param
        gate = cy >= p["Y_MIN"]
        m_r = gate and (p["RED_CB_LO"] <= ccb <= p["RED_CB_HI"]) \
                    and (p["RED_CR_LO"] <= ccr <= p["RED_CR_HI"])
        m_b = gate and (p["BLUE_CB_LO"] <= ccb <= p["BLUE_CB_HI"]) \
                    and (p["BLUE_CR_LO"] <= ccr <= p["BLUE_CR_HI"])
        m_g = gate and (p["GREEN_CB_LO"] <= ccb <= p["GREEN_CB_HI"]) \
                    and (p["GREEN_CR_LO"] <= ccr <= p["GREEN_CR_HI"])

        return {"y": cy, "cb": ccb, "cr": ccr,
                "q_y": q_y, "q_cb": q_cb, "q_cr": q_cr,
                "m_r": int(m_r), "m_g": int(m_g), "m_b": int(m_b)}


# ---------------------------------------------------------------------------
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--rtl", default=os.path.join(here, "detect_color_mask.v"))
    ap.add_argument("--out", default=os.path.join(here, "rtl_out.txt"))
    args = ap.parse_args()

    rtl = RtlParams(args.rtl)
    rtl.dump()

    if not os.path.isfile(args.out):
        print("!! 找不到仿真输出 %s，请先跑 ModelSim" % args.out)
        return 2

    vec = build_vectors()

    rows = []
    with open(args.out, "r", encoding="utf-8", errors="replace") as fp:
        for line in fp:
            line = line.strip()
            if line:
                rows.append([int(t) for t in line.split()])

    print("[仿真输出] %s ：共 %d 行" % (os.path.basename(args.out), len(rows)))

    if len(rows) != N:
        print("!! 输出像素数 %d != 期望 %d（可能有时序/复位问题）" % (len(rows), N))
        return 2

    err = 0
    first_show = 0
    for idx, row in enumerate(rows):
        x, y, y_o, cb_o, cr_o, mr, mg, mb, msel, last = row
        r, g, b = vec[idx]
        exp = rtl.model(r, g, b)

        ex_x = idx % W
        ex_y = idx // W
        ex_last = 1 if (idx % W) == (W - 1) else 0

        problems = []
        if (x, y) != (ex_x, ex_y):
            problems.append("坐标 (%d,%d) != (%d,%d)" % (x, y, ex_x, ex_y))
        if y_o != exp["y"]:
            problems.append("Y %d != %d" % (y_o, exp["y"]))
        if cb_o != exp["cb"]:
            problems.append("Cb %d != %d" % (cb_o, exp["cb"]))
        if cr_o != exp["cr"]:
            problems.append("Cr %d != %d" % (cr_o, exp["cr"]))
        if mr != exp["m_r"]:
            problems.append("mask_r %d != %d" % (mr, exp["m_r"]))
        if mg != exp["m_g"]:
            problems.append("mask_g %d != %d" % (mg, exp["m_g"]))
        if mb != exp["m_b"]:
            problems.append("mask_b %d != %d" % (mb, exp["m_b"]))
        if msel != exp["m_r"]:
            problems.append("mask_sel %d != %d (color_sel=0 应为红)" % (msel, exp["m_r"]))
        if last != ex_last:
            problems.append("last %d != %d" % (last, ex_last))

        if problems:
            err += 1
            if first_show < 8:
                print("  第 %d 像素 RGB=(%3d,%3d,%3d): %s" % (idx, r, g, b, "; ".join(problems)))
                first_show += 1

    # ---- 灰阶零色偏专项检查（工程历史上最易踩的坑）----
    gray_bad = 0
    for idx in range(256):
        row = rows[idx]
        if not (row[3] == 128 and row[4] == 128):
            gray_bad += 1
            if gray_bad <= 5:
                print("  灰阶 %d 出现色偏：Cb=%d Cr=%d" % (idx, row[3], row[4]))

    # ---- 色卡断言：三类纯色必须各自命中自己的掩膜（防止阈值默认值被改坏）----
    #      行格式: x y Y Cb Cr m_r m_g m_b m_sel last  →  5/6/7 分别是 R/G/B 掩膜
    card_bad = 0
    for idx, name, col, label in ((256, "纯红", 5, "mask_r"),
                                  (257, "纯绿", 6, "mask_g"),
                                  (258, "纯蓝", 7, "mask_b")):
        if rows[idx][col] != 1:
            card_bad += 1
            print("  ✗ 色卡断言失败：%s 未命中 %s" % (name, label))
    # 黑 / 白 / 中灰 不得命中任何颜色（灰阶零色偏的功能面体现）
    for idx, name in ((259, "纯黑"), (260, "纯白"), (265, "中灰")):
        if any(rows[idx][c] for c in (5, 6, 7)):
            card_bad += 1
            print("  ✗ 色卡断言失败：%s 不应命中任何目标色" % name)

    print()
    print("=" * 66)
    print(" 像素级对拍 : %d/%d 通过, %d 处不一致" % (len(rows) - err, len(rows), err))
    print(" 灰阶零色偏 : %s（256 级灰的 Cb/Cr 必须恒为 128，错 %d 处）"
          % ("通过" if gray_bad == 0 else "失败", gray_bad))
    print(" 色卡断言   : %s（纯红→R掩膜、纯绿→G掩膜、纯蓝→B掩膜；黑白灰不命中）"
          % ("通过" if card_bad == 0 else "失败"))

    # 纯色系参照输出（便于人工核对，也方便 D7 标定看数）
    print("-" * 66)
    print(" 纯色/拟真目标 YCbCr 参照表（供 D7 阈值标定用）：")
    names = {256: "纯红", 257: "纯绿", 258: "纯蓝", 259: "纯黑", 260: "纯白",
             261: "拟真红球", 262: "拟真蓝方块", 263: "拟真绿三角",
             264: "棕", 265: "中灰"}
    for i in range(256, 266):
        row = rows[i]
        print("   %-12s RGB=(%3d,%3d,%3d)  Y=%3d Cb=%3d Cr=%3d   mask R/G/B=%d/%d/%d"
              % (names[i], vec[i][0], vec[i][1], vec[i][2],
                 row[2], row[3], row[4], row[5], row[6], row[7]))
    print("=" * 66)

    ok = (err == 0 and gray_bad == 0 and card_bad == 0)
    print(" 结论: %s" % ("✅ 全部通过" if ok else "❌ 存在不一致，需排查"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
