#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_top_color_chain.py — 顶层新增链路位级对拍（三通道 mask → 颜色编码 → 总线）

与 verify_target_bus.py 的差别：颜色不由预置图样直接给，而是**按顶层
design_top_wrapper.v 的表达式现场编码**，因此把「红 > 蓝 > 绿」的优先级
也纳入对拍。链路：

    三通道掩膜 (R,G,B)
      │
      ├─ 合并  mask = R|G|B                 （镜像顶层 S_mask_hit = O_mask = r|b|g）
      └─ 编码  cls  = R?红 : B?蓝 : G?绿 : 未分类  （镜像顶层 assign S_det_color）
      │
    conn_label ─► conn_features ─► target_bus(384bit)

图样覆盖：单色目标 / 三色分离 / ★多种「同像素多通道命中」的重叠优先级用例。
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "sim")
sys.path.insert(0, HERE)

import verify_conn_label as V            # noqa: E402
import verify_target_bus as B            # noqa: E402

IVERILOG = r"C:\iverilog\bin\iverilog.exe"
VVP = r"C:\iverilog\bin\vvp.exe"

W, H = V.W, V.H
RED, BLUE, GREEN, UNK = V.RED, V.BLUE, V.GREEN, V.UNK


# ---------------------------------------------------------------------------
# 1. 三通道图样 → (mask, cls)  （镜像顶层编码）
# ---------------------------------------------------------------------------
def bm():
    return [[0] * W for _ in range(H)]


def rsq(g, y0, y1, x0, x1):
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            g[y][x] = 1


def encode(rm, gm, bm_):
    """镜像顶层：mask = R|G|B；cls = R?红 : B?蓝 : G?绿 : 未分类。"""
    ps = [[1 if (rm[y][x] or gm[y][x] or bm_[y][x]) else 0 for x in range(W)]
          for y in range(H)]
    cs = [[(RED if rm[y][x] else (BLUE if bm_[y][x] else (GREEN if gm[y][x] else UNK)))
           for x in range(W)] for y in range(H)]
    return ps, cs


def tri_patterns():
    """返回 [(rm, gm, bm, note), ...]，每帧三条独立掩膜。"""
    out = []

    # 0 空帧
    out.append((bm(), bm(), bm(), "空帧"))

    # 1 单色：红
    r = bm(); rsq(r, 3, 6, 5, 12); out.append((r, bm(), bm(), "红单目标"))

    # 2 单色：蓝
    b = bm(); rsq(b, 3, 6, 5, 12); out.append((bm(), bm(), b, "蓝单目标"))

    # 3 单色：绿
    g = bm(); rsq(g, 3, 6, 5, 12); out.append((bm(), g, bm(), "绿单目标"))

    # 4 红 + 蓝 分离两目标
    r = bm(); rsq(r, 2, 5, 2, 9)
    b = bm(); rsq(b, 9, 12, 20, 28)
    out.append((r, bm(), b, "红+蓝 分离"))

    # 5 三色分离
    r = bm(); rsq(r, 1, 4, 1, 8)
    g = bm(); rsq(g, 6, 9, 1, 8)
    b = bm(); rsq(b, 11, 14, 1, 8)
    out.append((r, g, b, "三色分离"))

    # 6 两色大块重叠（红块 raster 序更靠上）→ 连通成一块，颜色 = 首游程色 → 红
    r = bm(); rsq(r, 4, 9, 6, 20)
    b = bm(); rsq(b, 6, 11, 12, 25)          # 与 r 交叠
    out.append((r, bm(), b, "红/蓝大块重叠→整块取首游程色=红"))

    # 7 红/绿大块重叠（红在上）→ 红
    r = bm(); rsq(r, 3, 10, 8, 18)
    g = bm(); rsq(g, 5, 12, 4, 22)
    out.append((r, g, bm(), "红/绿大块重叠→整块取首游程色=红"))

    # 8 绿/蓝大块重叠（绿在上）→ 绿
    g = bm(); rsq(g, 3, 10, 4, 22)
    b = bm(); rsq(b, 5, 12, 8, 18)
    out.append((bm(), g, b, "绿/蓝大块重叠→整块取首游程色=绿"))

    # 9 三色大块重叠（绿最靠上）→ 绿
    r = bm(); rsq(r, 6, 11, 10, 22)
    g = bm(); rsq(g, 4, 13, 6, 26)
    b = bm(); rsq(b, 8, 9, 2, 29)
    out.append((r, g, b, "三色大块重叠→整块取首游程色=绿"))

    # ---- 以下 4 帧才是「编码优先级」的正主：**单像素**多通道命中 ----
    #      孤立像素 → 游程长 1 → 该连通域颜色 = 这个像素的编码色，
    #      于是直接检验顶层 assign：红 > 蓝 > 绿。
    r = bm(); b = bm(); r[8][10] = 1; b[8][10] = 1
    out.append((r, bm(), b, "★单像素 红∩蓝 → 编码判红"))

    g = bm(); b = bm(); g[5][7] = 1; b[5][7] = 1
    out.append((bm(), g, b, "★单像素 蓝∩绿 → 编码判蓝"))

    r = bm(); g = bm(); b = bm(); r[11][13] = 1; g[11][13] = 1; b[11][13] = 1
    out.append((r, g, b, "★单像素 红∩蓝∩绿 → 编码判红"))

    g = bm(); g[3][3] = 1
    out.append((bm(), g, bm(), "单像素 仅绿 → 判绿"))

    return out


# ---------------------------------------------------------------------------
# 2. 写图样（3*W bit/行）
# ---------------------------------------------------------------------------
def write_tri(trips):
    os.makedirs(SIM, exist_ok=True)
    nhex = (3 * W + 3) // 4
    with open(os.path.join(SIM, "pattern_rgb3.hex"), "w") as fh:
        fh.write("%x\n" % len(trips))
        for (rm, gm, bm_, _) in trips:
            for y in range(H):
                rv = gv = bv = 0
                for x in range(W):
                    if rm[y][x]:
                        rv |= (1 << x)
                    if gm[y][x]:
                        gv |= (1 << x)
                    if bm_[y][x]:
                        bv |= (1 << x)
                fh.write(("%0" + str(nhex) + "x\n") % ((rv << (2 * W)) | (gv << W) | bv))


# ---------------------------------------------------------------------------
# 3. 跑仿真
# ---------------------------------------------------------------------------
def run_sim():
    vvp_out = os.path.join(SIM, "tb_top_color_chain.vvp")
    r = subprocess.run(
        [IVERILOG, "-g2005", "-I", HERE,
         "-o", vvp_out,
         os.path.join(HERE, "conn_label.v"),
         os.path.join(HERE, "conn_features.v"),
         os.path.join(HERE, "target_bus.v"),
         os.path.join(HERE, "tb_top_color_chain.v")],
        capture_output=True, text=True)
    if r.returncode != 0:
        print("[FAIL] iverilog 编译失败:\n" + r.stdout + r.stderr)
        return False
    r = subprocess.run([VVP, vvp_out], capture_output=True, text=True, cwd=HERE)
    if "TB(top_color_chain) done" not in r.stdout:
        print("[FAIL] 仿真未正常结束:\n" + r.stdout + r.stderr)
        return False
    return True


def read_out():
    buses, cnts, ns = [], [], []
    with open(os.path.join(SIM, "bus_out_color.txt")) as fh:
        for ln in fh:
            p = ln.split()
            if not p:
                continue
            if p[0] == 'B':
                buses.append(int(p[1], 16))
                cnts.append(int(p[2]))
            elif p[0] == 'N':
                ns.append(int(p[1]))
    return buses, cnts, ns


# ---------------------------------------------------------------------------
# 4. 主流程
# ---------------------------------------------------------------------------
def main():
    trips = tri_patterns()
    write_tri(trips)

    ps_list, cs_list, notes = [], [], []
    for (rm, gm, bm_, note) in trips:
        ps, cs = encode(rm, gm, bm_)
        ps_list.append(ps)
        cs_list.append(cs)
        notes.append(note)

    exp = V.reference(ps_list, cs_list)
    print("图样 %d 帧（三通道）已生成 → sim/pattern_rgb3.hex" % len(trips))
    print("跑仿真（编码 → conn_label → conn_features → target_pack）...")
    if not run_sim():
        sys.exit(1)

    buses, cnts, ns = read_out()
    print("仿真输出：B 行 %d，N 行 %d" % (len(buses), len(ns)))
    print()
    print("--- 顶层链路 384bit 逐位对拍 ---")
    ok = B.compare(exp, buses, cnts)

    # 额外：把每帧的颜色分类结论打出来，方便人工核对优先级
    print()
    print("--- 每帧组件颜色（含重叠优先级用例）---")
    for k in range(len(trips)):
        tops = B.top4(exp[k][1])
        cls_s = ' '.join('%s(a=%d)' % (B.CLS_NAME.get(t['cls'], '?'), t['area'])
                         for t in tops) or '（无目标）'
        print("  帧 %d  %-22s  %s" % (k, notes[k], cls_s))

    print()
    print("=" * 60)
    print("总体：" + ("[PASS] 顶层链路位级对拍通过" if ok else "[FAIL] 存在不一致"))
    print("=" * 60)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
