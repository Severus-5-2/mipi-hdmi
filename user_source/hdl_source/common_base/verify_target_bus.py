#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_target_bus.py — conn_label + conn_features 全链路位级对拍

流程：
  1. 复用 verify_conn_label.py 的图样生成与 RLE-CCL 参考实现
     （掩膜 sim/pattern.hex + 颜色 sim/pattern_cls.hex）
  2. 参考实现给出每帧的连通域汇总 → 按 (area 降序, xmin 升序, label 升序) 取前 4
  3. 跑 tb_target_bus（conn_label + conn_features + target_pack）→ sim/bus_out.txt
  4. 把 RTL 的 384 bit 总线与参考实现打包结果逐位比对，并校验 valid_cnt

总线位域（独立于 RTL 的第二实现，用于交叉验证 target_bus.v）：
    槽 k 占 bit [96k +: 96]
      valid[0]  cls[2:1]  cx[13:3]  cy[23:14]
      xmin[34:24]  xmax[45:35]  ymin[55:46]  ymax[65:56]  area[85:66]
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "sim")
sys.path.insert(0, HERE)

import verify_conn_label as V          # noqa: E402

IVERILOG = r"C:\iverilog\bin\iverilog.exe"
VVP = r"C:\iverilog\bin\vvp.exe"

CLS_NAME = {0: '红', 1: '蓝', 2: '绿', 3: '未分类'}


# ---------------------------------------------------------------------------
# 1. 参考实现：前 4 排序 + 打包
# ---------------------------------------------------------------------------
def top4(cmp_list):
    """按 area 降序 / 同面积 xmin 升序 / 再同 label 升序，取前 4。"""
    return sorted(cmp_list, key=lambda d: (-d['area'], d['xmin'], d['label']))[:4]


def pack_slot(t):
    """单个目标 → 96 bit（空槽返回 0）。"""
    if t is None:
        return 0
    s = 0
    s |= 1 << 0
    s |= (t['cls'] & 0x3) << 1
    s |= (t['cx'] & 0x7FF) << 3
    s |= (t['cy'] & 0x3FF) << 14
    s |= (t['xmin'] & 0x7FF) << 24
    s |= (t['xmax'] & 0x7FF) << 35
    s |= (t['ymin'] & 0x3FF) << 46
    s |= (t['ymax'] & 0x3FF) << 56
    s |= (t['area'] & 0xFFFFF) << 66
    return s


def pack_bus(tops):
    b = 0
    for k, t in enumerate(tops[:4]):
        b |= pack_slot(t) << (96 * k)
    return b


# ---------------------------------------------------------------------------
# 2. 跑仿真
# ---------------------------------------------------------------------------
def run_sim():
    vvp_out = os.path.join(SIM, "tb_target_bus.vvp")
    r = subprocess.run(
        [IVERILOG, "-g2005", "-I", HERE,
         "-o", vvp_out,
         os.path.join(HERE, "conn_label.v"),
         os.path.join(HERE, "conn_features.v"),
         os.path.join(HERE, "target_bus.v"),
         os.path.join(HERE, "tb_target_bus.v")],
        capture_output=True, text=True)
    if r.returncode != 0:
        print("[FAIL] iverilog 编译失败:\n" + r.stdout + r.stderr)
        return False
    r = subprocess.run([VVP, vvp_out], capture_output=True, text=True, cwd=HERE)
    if "TB(target_bus) done" not in r.stdout:
        print("[FAIL] 仿真未正常结束:\n" + r.stdout + r.stderr)
        return False
    return True


def read_bus_out():
    buses, cnts, ns = [], [], []
    with open(os.path.join(SIM, "bus_out.txt")) as fh:
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
# 3. 比对
# ---------------------------------------------------------------------------
def unpack_slot(s):
    if (s & 1) == 0:
        return None
    return dict(cls=(s >> 1) & 0x3, cx=(s >> 3) & 0x7FF, cy=(s >> 14) & 0x3FF,
                xmin=(s >> 24) & 0x7FF, xmax=(s >> 35) & 0x7FF,
                ymin=(s >> 46) & 0x3FF, ymax=(s >> 56) & 0x3FF,
                area=(s >> 66) & 0xFFFFF)


def compare(exp, buses, cnts):
    ok = True
    nf = len(exp)

    # buses[0] 是「第 0 帧 fdone」时锁存的空擂台 → 必须全 0
    if buses:
        if buses[0] != 0 or cnts[0] != 0:
            print("  [FAIL] B#0（初始擂台）应为全 0，实得 %096x cnt=%d"
                  % (buses[0], cnts[0]))
            ok = False
        else:
            print("  [OK]   B#0 初始擂台全 0（未累积任何目标）")
    else:
        print("  [FAIL] 没有捕获到任何 B 行")
        return False

    if len(buses) < nf + 1:
        print("  [FAIL] B 行数不足：实得 %d，期望 ≥ %d" % (len(buses), nf + 1))
        return False

    for k in range(nf):
        got_bus, got_cnt = buses[k + 1], cnts[k + 1]
        tops = top4(exp[k][1])
        exp_bus = pack_bus(tops)

        tag = ' '.join('%s(a=%d,x0=%d,%d,%d,%d,%d)' %
                       (CLS_NAME.get(t['cls'], '?'), t['area'], t['xmin'], t['xmax'],
                        t['ymin'], t['ymax'], t['label'])
                       for t in tops)
        if got_bus != exp_bus or got_cnt != len(tops):
            print("  [FAIL] 帧 %d  bus 不一致" % k)
            print("         RTL  = %096x  cnt=%d" % (got_bus, got_cnt))
            print("         参考 = %096x  cnt=%d" % (exp_bus, len(tops)))
            # 逐槽定位
            for s in range(4):
                g = unpack_slot((got_bus >> (96 * s)) & ((1 << 96) - 1))
                e = unpack_slot((exp_bus >> (96 * s)) & ((1 << 96) - 1))
                if g != e:
                    print("         槽 %d: RTL=%s" % (s, g))
                    print("               参考=%s" % (e,))
            ok = False
        else:
            print("  [OK]   帧 %d：cnt=%d  槽序（最大→小）：%s"
                  % (k, len(tops), tag if tag else '（无目标，全 0）'))

    return ok


def main():
    ps, cs = V.patterns()
    V.write_pattern(ps, cs)
    exp = V.reference(ps, cs)
    print("图样 %d 帧已生成 → sim/pattern.hex + sim/pattern_cls.hex" % len(ps))
    print("跑仿真（conn_label + conn_features + target_pack）...")
    if not run_sim():
        sys.exit(1)
    buses, cnts, ns = read_bus_out()
    print("仿真输出：B 行 %d，N 行 %d" % (len(buses), len(ns)))
    print()
    print("--- target_bus 384bit 逐位对拍 ---")
    ok = compare(exp, buses, cnts)
    print()
    print("=" * 60)
    print("总体：" + ("[PASS] target_bus 全链路位级对拍通过" if ok else "[FAIL] 存在不一致"))
    print("=" * 60)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
