#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_color_mask.py — detect_color_mask.v 的比特级对拍验证（v2，支持新旧两套判据）

设计原则（遵循工程 §7「验证脚本必须解析派」的约定）：
    本脚本 **不手抄任何系数**，而是从 RTL 原文用正则提取：
      · 9 个 YCbCr 常量系数（含符号）
      · 四舍五入常数、算术右移位数、色度偏置
      · 限幅上下界
      · v2 自适应判据参数 MAG_TH / RATIO10
      · 旧固定窗口阈值 RED_*/BLUE_*/GREEN_* 与 USE_ABS_WINDOW
    然后用「补码 + 算术右移 + 四舍五入 + 限幅」的比特级语义复算，
    最后与 iverilog/ModelSim 跑出来的 rtl_out.txt 逐像素比对。

    ★ v2 变更（2026-10-11）：掩膜判据由「固定绝对窗口」换成「以 (128,128) 为原点的
      方向 + 幅度 + 比值」自适应判据。本脚本同时实现两条路径，按 RTL 里的
      USE_ABS_WINDOW 参数自动选择，因此：
        · USE_ABS_WINDOW=1 → 与 D1 基线行为逐位一致（回归用）
        · USE_ABS_WINDOW=0 → 校验 v2 曝光自适应判据

用法：
    python verify_color_mask.py [--rtl detect_color_mask.v] [--out rtl_out.txt]
                                [--window abs|auto|both]
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
        self.coef = {}
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

        # ---- 4. 限幅上下界 ----
        m_hi = re.search(r"q_y\s*>\s*10'sd(\d+)", self.src)
        m_lo = re.search(r"q_y\s*<\s*10'sd(\d+)", self.src)
        if not (m_hi and m_lo):
            raise RuntimeError("未能解析限幅结构")
        self.clamp_hi = int(m_hi.group(1))
        self.clamp_lo = int(m_lo.group(1))

        # ---- 5. 参数区 ----
        #  ⚠ 关键：必须把注释剥掉再解析。RTL 头部注释里有大量形如
        #    「（仅 USE_ABS_WINDOW=1 时生效）」的文字，若不剥注释，
        #    正则会把注释里的 `=1` 当成真实参数值，导致模型走错分支。
        header = self.src.split(")( ", 1)[0] if ")( " in self.src \
            else self.src.split(")(", 1)[0]
        # 剥 // 行注释 与 /* */ 块注释
        code = re.sub(r"/\*.*?\*/", " ", header, flags=re.S)
        code = re.sub(r"//[^\n]*", " ", code)

        self.param = {}
        # 8'dNNN 形式（Y_MIN / 颜色窗口 / MAG_TH）
        for m in re.finditer(r"\b([A-Z][A-Z0-9_]*)\s*=\s*8'd(\d+)", code):
            self.param[m.group(1)] = int(m.group(2))
        # 任意位宽 'd 形式（RATIO10 等）
        for m in re.finditer(r"\b([A-Z][A-Z0-9_]*)\s*=\s*\d+'d(\d+)", code):
            self.param[m.group(1)] = int(m.group(2))
        # integer 形式（USE_ABS_WINDOW）—— 只在「参数声明」里取，取最后一个
        for m in re.finditer(r"\b(USE_ABS_WINDOW)\s*=\s*(\d+)", code):
            self.param[m.group(1)] = int(m.group(2))

        for name in ("Y_MIN", "MAG_TH", "RATIO10", "USE_ABS_WINDOW",
                     "RED_CB_LO", "RED_CB_HI", "RED_CR_LO", "RED_CR_HI",
                     "BLUE_CB_LO", "BLUE_CB_HI", "BLUE_CR_LO", "BLUE_CR_HI",
                     "GREEN_CB_LO", "GREEN_CB_HI", "GREEN_CR_LO", "GREEN_CR_HI"):
            if name not in self.param:
                raise RuntimeError("未能解析参数 %s" % name)

        # ---- 6. 从 RTL 源码校验判据结构是否与模型一致（防止 RTL 被改而模型没跟）----
        #     自适应判据的四条方向/比值表达式必须在 RTL 里原样出现
        for frag in ("w_mag >= {1'b0, MAG_TH}",
                     "w_dcr > 9'sd0",
                     "w_dcb <= 9'sd0",
                     "w_rcr10 >= w_lim_cb",
                     "w_dcb > 9'sd0",
                     "w_dcr <= 9'sd0",
                     "w_rcb10 >= w_lim_cr",
                     "w_dcb < 9'sd0",
                     "w_dcr <  9'sd0"):
            if frag not in self.src:
                raise RuntimeError("RTL 中未找到判据片段 %r，模型可能已过期" % frag)

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
        print("   亮度门控 Y_MIN=%d" % self.param["Y_MIN"])
        print("   ★v2 自适应判据: MAG_TH=%d, RATIO10=%d（比值 ≤ %d/10）"
              % (self.param["MAG_TH"], self.param["RATIO10"], self.param["RATIO10"]))
        print("   兼容旧窗口 USE_ABS_WINDOW=%d" % self.param["USE_ABS_WINDOW"])
        print()

    # -----------------------------------------------------------------
    def ycbcr(self, r, g, b):
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

        # ⚠ 掩膜必须用「限幅后」的值比较（见 RTL 第 4 节注释）
        return clamp(q_y), clamp(q_cb), clamp(q_cr)

    # -----------------------------------------------------------------
    def model(self, r, g, b, mode=None):
        """比特级镜像。mode: 'auto' | 'abs' | None(按 RTL 的 USE_ABS_WINDOW)"""
        cy, ccb, ccr = self.ycbcr(r, g, b)
        p = self.param

        if mode is None:
            mode = "abs" if p["USE_ABS_WINDOW"] else "auto"

        gate = cy >= p["Y_MIN"]

        if mode == "abs":
            m_r = gate and (p["RED_CB_LO"] <= ccb <= p["RED_CB_HI"]) \
                        and (p["RED_CR_LO"] <= ccr <= p["RED_CR_HI"])
            m_b = gate and (p["BLUE_CB_LO"] <= ccb <= p["BLUE_CB_HI"]) \
                        and (p["BLUE_CR_LO"] <= ccr <= p["BLUE_CR_HI"])
            m_g = gate and (p["GREEN_CB_LO"] <= ccb <= p["GREEN_CB_HI"]) \
                        and (p["GREEN_CR_LO"] <= ccr <= p["GREEN_CR_HI"])
        else:
            # ---- v2：以 128 为原点的有符号色差 ----
            dcb = ccb - 128
            dcr = ccr - 128
            adcb = abs(dcb)
            adcr = abs(dcr)
            mag = adcb + adcr
            mag_ok = mag >= p["MAG_TH"]
            m_r = mag_ok and (dcr > 0) and (dcb <= 0) and (adcr * 10 >= adcb * p["RATIO10"])
            m_b = mag_ok and (dcb > 0) and (dcr <= 0) and (adcb * 10 >= adcr * p["RATIO10"])
            m_g = mag_ok and (dcb < 0) and (dcr < 0)

        return {"y": cy, "cb": ccb, "cr": ccr,
                "q_y": cy, "q_cb": ccb, "q_cr": ccr,
                "m_r": int(m_r), "m_g": int(m_g), "m_b": int(m_b)}


# ---------------------------------------------------------------------------
def compare(rtl, rows, vec, mode, verbose=True):
    """返回 (err, 明细列表)。"""
    err = 0
    details = []
    first_show = 0
    for idx, row in enumerate(rows):
        x, y, y_o, cb_o, cr_o, mr, mg, mb, msel, last = row
        r, g, b = vec[idx]
        exp = rtl.model(r, g, b, mode=mode)

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
            details.append((idx, (r, g, b), problems))
            if verbose and first_show < 8:
                print("  第 %d 像素 RGB=(%3d,%3d,%3d): %s"
                      % (idx, r, g, b, "; ".join(problems)))
                first_show += 1

    # ---- 灰阶零色偏专项检查 ----
    gray_bad = 0
    for idx in range(256):
        row = rows[idx]
        if not (row[3] == 128 and row[4] == 128):
            gray_bad += 1
            if verbose and gray_bad <= 5:
                print("  灰阶 %d 出现色偏：Cb=%d Cr=%d" % (idx, row[3], row[4]))

    # ---- 色卡断言（三纯色各自命中自己的掩膜；黑/白/中灰不得命中）----
    card_bad = 0
    for idx, name, col, label in ((256, "纯红", 5, "mask_r"),
                                  (257, "纯绿", 6, "mask_g"),
                                  (258, "纯蓝", 7, "mask_b")):
        if rows[idx][col] != 1:
            card_bad += 1
            if verbose:
                print("  ✗ 色卡断言失败：%s 未命中 %s" % (name, label))
    for idx, name in ((259, "纯黑"), (260, "纯白"), (265, "中灰")):
        if any(rows[idx][c] for c in (5, 6, 7)):
            card_bad += 1
            if verbose:
                print("  ✗ 色卡断言失败：%s 不应命中任何目标色" % name)

    return err, gray_bad, card_bad, details


# ---------------------------------------------------------------------------
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--rtl", default=os.path.join(here, "detect_color_mask.v"))
    ap.add_argument("--out", default=os.path.join(here, "rtl_out.txt"))
    ap.add_argument("--window", default="both",
                    choices=["abs", "auto", "both"],
                    help="只比对某一路，或两路都跑(both)")
    args = ap.parse_args()

    rtl = RtlParams(args.rtl)
    rtl.dump()

    if not os.path.isfile(args.out):
        print("!! 找不到仿真输出 %s，请先跑仿真" % args.out)
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

    modes = ["abs", "auto"] if args.window == "both" else [args.window]
    results = {}
    for mode in modes:
        label = "固定绝对窗口(旧)" if mode == "abs" else "曝光自适应(v2)"
        print()
        print("-" * 66)
        print(" 【%s】" % label)
        err, gray_bad, card_bad, details = compare(rtl, rows, vec, mode)
        results[mode] = (err, gray_bad, card_bad)
        print(" 像素级对拍 : %d/%d 通过, %d 处不一致" % (len(rows) - err, len(rows), err))
        print(" 灰阶零色偏 : %s（错 %d 处）"
              % ("通过" if gray_bad == 0 else "失败", gray_bad))
        print(" 色卡断言   : %s" % ("通过" if card_bad == 0 else "失败"))

    # 参照表
    print()
    print("-" * 66)
    print(" 纯色/拟真目标 YCbCr 参照表 + 两路判据对比（供现场标定用）：")
    names = {256: "纯红", 257: "纯绿", 258: "纯蓝", 259: "纯黑", 260: "纯白",
             261: "拟真红球", 262: "拟真蓝方块", 263: "拟真绿三角",
             264: "棕", 265: "中灰"}
    print("   %-12s %-16s %-8s %-8s  %-10s %-10s" %
          ("名称", "RGB", "Y", "Cb/Cr", "旧窗口R/G/B", "v2 R/G/B"))
    for i in range(256, 266):
        row = rows[i]
        m_abs = rtl.model(*vec[i], mode="abs")
        m_aut = rtl.model(*vec[i], mode="auto")
        print("   %-12s (%3d,%3d,%3d)  %3d  %3d/%3d   %d/%d/%d      %d/%d/%d"
              % (names[i], vec[i][0], vec[i][1], vec[i][2],
                 row[2], row[3], row[4],
                 m_abs["m_r"], m_abs["m_g"], m_abs["m_b"],
                 m_aut["m_r"], m_aut["m_g"], m_aut["m_b"]))
    print("=" * 66)

    # 结论：RTL 实际跑哪条路径，就以哪条路径为准
    active = "abs" if rtl.param["USE_ABS_WINDOW"] else "auto"
    err, gray_bad, card_bad = results[active]
    ok = (err == 0 and gray_bad == 0 and card_bad == 0)
    print(" RTL 当前生效路径: %s（USE_ABS_WINDOW=%d）"
          % ("固定绝对窗口(旧)" if active == "abs" else "曝光自适应(v2)",
             rtl.param["USE_ABS_WINDOW"]))
    print(" 结论: %s" % ("✅ 全部通过" if ok else "❌ 存在不一致，需排查"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
