#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_conn_label.py — conn_label.v 位级对拍（解析派）

流程：
  1. 生成 11 幅已知掩膜图样 → sim/pattern.hex
     + 对应的逐像素颜色图（2bit/像素）→ sim/pattern_cls.hex
  2. 用 Python 参考实现（严格镜像 RTL 的 4 连通 RLE-CCL 逻辑）算出期望结果
  3. 跑 iverilog + vvp
  4. 逐项对拍：
       · 每像素 label 流（O_label / O_label_valid / 坐标）
       · 帧末汇总（O_cmp_*：cls / area / bbox / 质心 / 紧凑编号 / 数量）

参考实现与 RTL 的对应关系（重要）：
  · eq[] 并查表，resolve3 = 3 级解析（像素流路径用）
  · resolve_full = 32 次迭代（帧末用）
  · 游程 label = min(重叠的上行游程 label)；无重叠则分配新 label（1 起，>32 则丢弃该游程）
  · 统计按「游程关闭时的 label」入桶，帧末再按 root 归并
  · 紧凑编号 = 根 label 升序（即组件首次出现的扫描顺序）
  · 颜色分类：每 label 取「首个被指派时的已知色」；等价合并时颜色沿并查链
    向根传播；全程未知 ⇒ 2'b11（未分类）。判决严格按 RTL 的非阻塞语义
    （同一拍内所有条件读**旧值**，多路写同一寄存器时以**最后调度者**为准）。
"""

import os
import subprocess
import sys

W, H = 32, 16
NEG = 32
MIN_AREA = 1

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "sim")

IVERILOG = r"C:\iverilog\bin\iverilog.exe"
VVP = r"C:\iverilog\bin\vvp.exe"


# ---------------------------------------------------------------------------
# 1. 图样生成
# ---------------------------------------------------------------------------
RED, BLUE, GREEN, UNK = 0, 1, 2, 3      # 与 common_defs.vh 的 CLS_* 一致


def blank():
    return [[0] * W for _ in range(H)]


def blank_c():
    return [[UNK] * W for _ in range(H)]


def rect(g, y0, y1, x0, x1):
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            g[y][x] = 1


def fill_col(c, mask, x0, x1, y0, y1, col):
    """在 mask 命中的像素上刷颜色（只在掩膜内刷）"""
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            if 0 <= y < H and 0 <= x < W and mask[y][x]:
                c[y][x] = col


def patterns():
    """返回 (masks, colors)；colors[k] 为 H×W 的 2bit 颜色图（UNK=未分类）。"""
    ps, cs = [], []

    # 0 空帧
    g = blank(); ps.append(g); cs.append(blank_c())

    # 1 单矩形 → 全红
    g = blank(); rect(g, 3, 6, 5, 12); ps.append(g)
    c = blank_c(); fill_col(c, g, 0, W - 1, 0, H - 1, RED); cs.append(c)

    # 2 两个水平分离矩形 → 左红 右蓝
    g = blank(); rect(g, 2, 4, 2, 8); rect(g, 9, 11, 20, 28); ps.append(g)
    c = blank_c()
    fill_col(c, g, 0, 12, 0, H - 1, RED); fill_col(c, g, 13, W - 1, 0, H - 1, BLUE)
    cs.append(c)

    # 3 两个垂直分离矩形 → 左绿 右红
    g = blank(); rect(g, 2, 7, 4, 9); rect(g, 2, 7, 20, 25); ps.append(g)
    c = blank_c()
    fill_col(c, g, 0, 12, 0, H - 1, GREEN); fill_col(c, g, 13, W - 1, 0, H - 1, RED)
    cs.append(c)

    # 4 L 形 → 蓝；但「raster 序首个掩膜像素」置 UNK，测游程内「首个已知色补齐」
    g = blank(); rect(g, 2, 10, 4, 6); rect(g, 8, 10, 4, 14); ps.append(g)
    c = blank_c(); fill_col(c, g, 0, W - 1, 0, H - 1, BLUE)
    done = False
    for y in range(H):
        for x in range(W):
            if g[y][x] and not done:
                c[y][x] = UNK; done = True
    cs.append(c)

    # 5 U 形 → 红；**左臂整条置 UNK**，测「根未定色时用合并游程色补齐」
    g = blank(); rect(g, 2, 12, 2, 4); rect(g, 2, 12, 12, 14); rect(g, 11, 13, 2, 14)
    ps.append(g)
    c = blank_c(); fill_col(c, g, 0, W - 1, 0, H - 1, RED)
    fill_col(c, g, 2, 4, 0, H - 1, UNK)          # 左臂 UNK
    cs.append(c)

    # 6 环形 → 绿（多游程等价合并）
    g = blank(); rect(g, 2, 12, 2, 14); rect(g, 4, 10, 4, 12)
    for y in range(4, 11):
        for x in range(4, 13):
            g[y][x] = 0
    ps.append(g)
    c = blank_c(); fill_col(c, g, 0, W - 1, 0, H - 1, GREEN); cs.append(c)

    # 7 对角线单像素点 → 颜色按点序循环（红蓝绿）
    g = blank()
    for k in range(9):
        g[2 + k][2 + k] = 1
    ps.append(g)
    c = blank_c()
    for k in range(9):
        c[2 + k][2 + k] = (RED, BLUE, GREEN)[k % 3]
    cs.append(c)

    # 8 对角相接的两块 → 上红 下蓝
    g = blank(); rect(g, 2, 5, 2, 6); rect(g, 6, 9, 7, 11); ps.append(g)
    c = blank_c()
    fill_col(c, g, 0, W - 1, 0, 5, RED); fill_col(c, g, 0, W - 1, 6, H - 1, BLUE)
    cs.append(c)

    # 9 满帧 → 全 UNK（期望 cls = 未分类）
    g = [[1] * W for _ in range(H)]; ps.append(g); cs.append(blank_c())

    # 10 40 个真正孤立的像素（隔行、隔列；4 连通互不相连）
    #    → 压到 label 上限 32，测「超出则丢弃游程」的降级路径；颜色循环
    g = blank()
    for k in range(40):
        yy = 2 * (k // 8)
        xx = (k % 8) * 4
        g[yy][xx] = 1
    ps.append(g)
    c = blank_c()
    for k in range(40):
        yy = 2 * (k // 8)
        xx = (k % 8) * 4
        c[yy][xx] = (RED, BLUE, GREEN)[k % 3]
    cs.append(c)

    return ps, cs


# ---------------------------------------------------------------------------
# 2. 参考实现
# ---------------------------------------------------------------------------
def rle_runs(row):
    runs = []
    x = 0
    while x < W:
        if row[x]:
            x0 = x
            while x < W and row[x]:
                x += 1
            runs.append((x0, x - 1))
        else:
            x += 1
    return runs


def reference(ps, cs):
    """返回 [(stream_dict, cmp_list), ...]，每个元素对应一帧。"""
    out = []
    for gi in range(len(ps)):
        grid = ps[gi]
        cgrid = cs[gi]
        eq = list(range(NEG + 1))            # eq[0..32]

        def resolve3(v):
            t = eq[v]; t = eq[t]; t = eq[t]
            return t

        def resolve_full(v):
            t = v
            for _ in range(NEG):
                t = eq[t]
            return t

        lbl_next = 1
        cls_lbl = [UNK] * (NEG + 1)          # 每 label 颜色（含 0 号占位）
        prev_runs = []                       # [(x0, x1, stored_label)]
        rows_runs = []
        acc = {i: dict(area=0, sx=0, sy=0,
                       xmin=2047, xmax=0, ymin=1023, ymax=0)
               for i in range(NEG + 1)}

        for y in range(H):
            cur = []
            for (x0, x1) in rle_runs(grid[y]):
                ovl = [(ps_, pe, pl) for (ps_, pe, pl) in prev_runs
                       if (x0 <= pe and x1 >= ps_)]
                resolved = sorted({resolve3(pl) for (_, _, pl) in ovl} - {0})
                if not resolved:
                    if lbl_next > NEG:       # 超 32 → 丢弃该游程（与 RTL 一致）
                        continue
                    lbl = lbl_next
                    lbl_next += 1
                    need_new = True
                else:
                    lbl = resolved[0]
                    for r in resolved[1:]:
                        if r != lbl:
                            eq[r] = lbl
                    need_new = False

                # ---- 颜色锁存（镜像 RTL：同拍条件一律读「旧值」）----
                rc = UNK                     # 本游程首个已知色
                for x in range(x0, x1 + 1):
                    if cgrid[y][x] != UNK:
                        rc = cgrid[y][x]
                        break
                old = cls_lbl[:]
                if need_new:
                    cls_lbl[lbl] = rc
                else:
                    # ① 根未定色 → 用本游程色
                    if old[lbl] == UNK and rc != UNK:
                        cls_lbl[lbl] = rc
                    # ② 被并入者未定色 → 继承根色（读旧值，保证链式传递）
                    for (_, _, pl) in ovl:
                        pr = resolve3(pl)
                        if pr != lbl and old[pr] == UNK:
                            cls_lbl[pr] = old[lbl]
                    # ③ 根与本游程都未知 → 取「最后一个」（= RTL 里 mi 最大者）满足者
                    if old[lbl] == UNK and rc == UNK:
                        pick = UNK
                        for (_, _, pl) in ovl:
                            pr = resolve3(pl)
                            if pr != lbl and old[pr] != UNK:
                                pick = old[pr]        # 顺序覆盖 → 末者胜
                        if pick != UNK:
                            cls_lbl[lbl] = pick

                cur.append((x0, x1, lbl))
                a = acc[lbl]
                a['area'] += (x1 - x0 + 1)
                a['sx'] += (x0 + x1) * (x1 - x0 + 1) // 2
                a['sy'] += y * (x1 - x0 + 1)
                a['xmin'] = min(a['xmin'], x0)
                a['xmax'] = max(a['xmax'], x1)
                a['ymin'] = min(a['ymin'], y)
                a['ymax'] = max(a['ymax'], y)
            rows_runs.append(cur)
            prev_runs = cur

        # ---- 每像素 label 流（行 0..H-2，晚一行回放）----
        stream = {}
        for y in range(H - 1):
            for (x0, x1, lbl) in rows_runs[y]:
                for x in range(x0, x1 + 1):
                    stream[(x, y)] = lbl

        # ---- 帧末：完全解析 → 归并 → 紧凑编号 ----
        root = [resolve_full(i) for i in range(NEG + 1)]
        cls_of = [cls_lbl[root[i]] for i in range(NEG + 1)]
        tp = {i: dict(area=0, sx=0, sy=0,
                      xmin=2047, xmax=0, ymin=1023, ymax=0)
              for i in range(NEG + 1)}
        for i in range(NEG + 1):
            if acc[i]['area'] == 0:
                continue
            r = root[i]
            d = tp[r]
            d['area'] += acc[i]['area']
            d['sx'] += acc[i]['sx']
            d['sy'] += acc[i]['sy']
            d['xmin'] = min(d['xmin'], acc[i]['xmin'])
            d['xmax'] = max(d['xmax'], acc[i]['xmax'])
            d['ymin'] = min(d['ymin'], acc[i]['ymin'])
            d['ymax'] = max(d['ymax'], acc[i]['ymax'])

        cmp_list = []
        sc = 0
        for i in range(NEG + 1):
            if root[i] != i:
                continue
            d = tp[i]
            if d['area'] < MIN_AREA:
                continue
            sc += 1
            cmp_list.append(dict(
                label=sc, cls=cls_of[i], area=d['area'],
                xmin=d['xmin'], xmax=d['xmax'],
                ymin=d['ymin'], ymax=d['ymax'],
                cx=d['sx'] // d['area'], cy=d['sy'] // d['area']))

        out.append((stream, cmp_list))
    return out


# ---------------------------------------------------------------------------
# 3. 生成 pattern.hex
# ---------------------------------------------------------------------------
def write_pattern(ps, cs):
    os.makedirs(SIM, exist_ok=True)
    with open(os.path.join(SIM, "pattern.hex"), "w") as fh:
        fh.write("%x\n" % len(ps))
        for g in ps:
            for y in range(H):
                v = 0
                for x in range(W):
                    if g[y][x]:
                        v |= (1 << x)
                fh.write("%08x\n" % v)
    with open(os.path.join(SIM, "pattern_cls.hex"), "w") as fh:
        fh.write("%x\n" % len(ps))
        for c in cs:
            for y in range(H):
                v = 0
                for x in range(W):
                    v |= (c[y][x] & 3) << (2 * x)
                fh.write("%016x\n" % v)


# ---------------------------------------------------------------------------
# 4. 跑仿真
# ---------------------------------------------------------------------------
def run_sim():
    vvp_out = os.path.join(SIM, "tb_conn_label.vvp")
    r = subprocess.run(
        [IVERILOG, "-g2005", "-I", os.path.join(HERE, "."),
         "-o", vvp_out,
         os.path.join(HERE, "conn_label.v"),
         os.path.join(HERE, "tb_conn_label.v")],
        capture_output=True, text=True)
    if r.returncode != 0:
        print("[FAIL] iverilog 编译失败:\n" + r.stdout + r.stderr)
        return False
    r = subprocess.run([VVP, vvp_out], capture_output=True, text=True, cwd=HERE)
    if "TB done" not in r.stdout:
        print("[FAIL] 仿真未正常结束:\n" + r.stdout + r.stderr)
        return False
    return True


# ---------------------------------------------------------------------------
# 5. 对拍
# ---------------------------------------------------------------------------
def read_outputs():
    stream = {}
    with open(os.path.join(SIM, "label_out.txt")) as fh:
        for ln in fh:
            p = ln.split()
            if len(p) == 5 and p[0] == 'P':
                _, x, y, lbl, lv = p
                stream[(int(x), int(y))] = (int(lbl), int(lv))   # 后出现者覆盖
    frames, cur = [], None
    with open(os.path.join(SIM, "cmp_out.txt")) as fh:
        for ln in fh:
            p = ln.split()
            if p[0] == 'C':
                if cur is None:
                    cur = []
                cur.append(dict(label=int(p[1]), cls=int(p[2]), area=int(p[3]),
                                xmin=int(p[4]), xmax=int(p[5]),
                                ymin=int(p[6]), ymax=int(p[7]),
                                cx=int(p[8]), cy=int(p[9])))
            elif p[0] == 'N':
                frames.append((cur or [], int(p[1])))
                cur = None
    return stream, frames


def compare(ps, expected, stream, frames):
    ok = True
    FIELDS = ('label', 'cls', 'area', 'xmin', 'xmax', 'ymin', 'ymax', 'cx', 'cy')
    CLS_NAME = {0: '红', 1: '蓝', 2: '绿', 3: '未分类'}

    # ---- 每像素 label 流 ----
    # 多帧共用 (x,y) 键，dump 顺序即写入顺序，后写的覆盖先写的 →
    # 最终 stream 里保留的就是**最后一帧**的值，与 expected[-1] 对拍。
    print("--- 每像素 label 流（末帧逐像素精确比对）---")
    exp_last = expected[-1][0]
    bad_lv, bad_lb = 0, 0
    first = None
    for y in range(H - 1):
        for x in range(W):
            e_lv = 1 if (x, y) in exp_last else 0
            e_lb = exp_last.get((x, y), 0)
            g_lb, g_lv = stream.get((x, y), (0, 0))
            if e_lv != g_lv:
                bad_lv += 1
                if first is None:
                    first = ('valid', x, y, e_lv, g_lv)
            if e_lb != g_lb:
                bad_lb += 1
                if first is None:
                    first = ('label', x, y, e_lb, g_lb)
    if bad_lv or bad_lb:
        ok = False
        print("  [FAIL] valid 不一致 %d 处 / label 值不一致 %d 处，首个：%s (x=%d,y=%d) 期望 %d 实得 %d"
              % (bad_lv, bad_lb, first[0], first[1], first[2], first[3], first[4]))
    else:
        print("  [OK]   valid 与 label 值逐像素完全一致：%d 像素" % (W * (H - 1)))

    # ---- 帧末汇总 ----
    # 组序：每次「fdone → 行 → 等待 → N」为一个组，帧 f 的汇总落在它自己的组里
    print("--- 帧末汇总（O_cmp_*）---")
    for k in range(len(ps)):
        if k >= len(frames):
            print("  [FAIL] 第 %d 帧汇总缺失" % k)
            ok = False
            continue
        got_c, got_n = frames[k]
        exp_c = expected[k][1]
        if got_n != len(exp_c):
            print("  [FAIL] 帧 %d 组件数：RTL %d vs 参考 %d" % (k, got_n, len(exp_c)))
            ok = False
        if len(got_c) != len(exp_c):
            print("  [FAIL] 帧 %d 组件条目数：RTL %d vs 参考 %d"
                  % (k, len(got_c), len(exp_c)))
            ok = False
            continue
        for i, (g, e) in enumerate(zip(got_c, exp_c)):
            for f in FIELDS:
                if g[f] != e[f]:
                    print("  [FAIL] 帧 %d 组件 %d 字段 %s：RTL %d vs 参考 %d"
                          % (k, i, f, g[f], e[f]))
                    ok = False
        if all(g[f] == e[f] for g, e in zip(got_c, exp_c) for f in FIELDS):
            cls_desc = ' '.join('L%d=%s' % (e['label'], CLS_NAME.get(e['cls'], '?'))
                                for e in exp_c)
            print("  [OK]   帧 %d：%d 个组件，cls/area/bbox/质心/编号全部一致  [%s]"
                  % (k, len(exp_c), cls_desc))

    return ok


def main():
    ps, cs = patterns()
    write_pattern(ps, cs)
    expected = reference(ps, cs)
    print("图样 %d 帧已生成 → sim/pattern.hex + sim/pattern_cls.hex" % len(ps))
    print("跑仿真 ...")
    if not run_sim():
        sys.exit(1)
    stream, frames = read_outputs()
    print("仿真输出：label 条目 %d，帧汇总 %d 组" % (len(stream), len(frames)))
    print()
    ok = compare(ps, expected, stream, frames)
    print()
    print("=" * 60)
    print("总体：" + ("[PASS] 位级对拍全部通过" if ok else "[FAIL] 存在不一致"))
    print("=" * 60)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
