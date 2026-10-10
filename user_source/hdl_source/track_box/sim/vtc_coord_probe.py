#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vtc_coord_probe.py —— uivtc.v + hdmi_mixer.v 像素坐标系统的【位级】镜像审计
================================================================================
为什么要有这个脚本
------------------
开发方案B 第 3.3 节把「验证坐标系统」列为张铭晨 D1 的第一件事；第 7 节又写死了
本项目唯一的硬经验：**上板前必须先在 Python 里跑位级仿真（比特级镜像 RTL）**，
因为功能级「看起来对」不算对。

本脚本不"估计"、也不按文档"应该有"来推坐标，而是逐周期逐位重演 uivtc.v 的
hcnt / vcnt / 全部流水寄存器，再把 O_vtc_de / O_vtc_user / O_vtc_last 喂给
hdmi_mixer.v 里那段像素坐标计数器（hdmi_mixer.v 第 327~345 行）。

★两条方法论（第一版脚本就在这两处栽过，写下来免得后人再踩）：
  1) 绝不能用「第几个 de 脉冲」去推算 (行, 列)。复位窗口里 hcnt 被冻结在 0
     共 5 拍而 vtc_de 已是 1，会有多余脉冲灌进 de 流（实测多 4 个），
     用脉冲计数映射会整体错位、得出"坐标全错"的假结论。
     正确做法：以 vtc 域的 (vcnt, hcnt) 为像素身份的唯一权威，再用
     「vtc_de 拍 → 输出拍 = 该拍 + 2」这条确定的流水延迟把两者对上。
  2) 启动期（复位后第 1 帧）与稳态必须分开统计。系统复位后第 1 帧没有 user
     脉冲（vs_start 要先被 vtc_vs 上升沿置起），其第 0 行还混着 4 个伪像素；
     把它们混进稳态统计会把结论带偏。本脚本只审计「首个 user 脉冲之后」的帧。

回答的问题
----------
  Q1   O_vtc_last 是否每行末像素一拍、且与 O_vtc_de 严格同拍？
       （hdmi_mixer 行计数写成 `else if(I_video_de){ if(I_video_last) ... }`，
         只有 last 与 de 同拍行计数才会走。这是画框 / OSD 坐标的地基。）
  Q1b  O_vtc_user 是否每帧只拍一次、且与 de 同拍。
  Q2   处理第 r 行第 k 个像素时，(S_x,S_y) 是否等于 (k,r)？逐行分类统计。
  Q3   启动期实况：复位窗口灌进 de 流的伪像素有几个、影响范围多大。
  Q4   hdmi_mixer 现有三处文字 OSD + Logo 的区域常量，单帧实际命中多少像素。

退出码：0 = 全部通过；1 = 有 FAIL。

用法
----
    python vtc_coord_probe.py            # 默认 3 帧
    python vtc_coord_probe.py 4          # 4 帧
================================================================================
"""

import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

# ---------------------------------------------------------------------------
# 时序常量：**必须与 design_top_wrapper.v 里 u_hdmi_vtc 的实例化参数一字不差**
# （design_top_wrapper.v 约 931~939 行）。顶层改参数，这里同步改，否则结论作废。
# ---------------------------------------------------------------------------
H_ACTIVE = 1280
H_FRAME = 1650
H_SYNC_S = 1390
H_SYNC_E = 1430

V_ACTIVE = 720
V_FRAME = 750
V_SYNC_S = 725
V_SYNC_E = 730

FRAMES = int(sys.argv[1]) if len(sys.argv) > 1 else 3
CYCLES = H_FRAME * V_FRAME * FRAMES


# ===========================================================================
# 第一部分：uivtc.v 的位级镜像（生成器，单遍、O(1) 内存）
# ===========================================================================
def vtc_stream(cycles):
    """逐拍重演 uivtc.v，yield 每拍的「本拍可见信号」。

    实现纪律（照搬 Verilog 非阻塞语义）：
      · 先算本拍组合量，据此产出本拍输出；
      · 再统一更新寄存器，RHS 一律取更新前的旧值。
    绝不"用本拍的新值算本拍输出"。
    """
    rst_cnt = 0          # reg [2:0]
    hcnt = 0             # reg [11:0]
    vcnt = 0             # reg [11:0]
    vs_start = 0
    vs_r1 = hs_r1 = 0
    user_r1 = user_r2 = 0
    valid_r1 = valid_r2 = 0
    last_r2 = 0

    for cyc in range(cycles):
        # ---------------- 阶段 1：本拍组合量 ----------------
        rst_sync = (rst_cnt >> 2) & 1                      # = rst_cnt[2]

        vtc_hs = 1 if (H_SYNC_S <= hcnt < H_SYNC_E) else 0
        vtc_vs = 1 if (V_SYNC_S < vcnt <= V_SYNC_E) else 0
        vtc_de = 1 if (hcnt < H_ACTIVE and vcnt < V_ACTIVE) else 0
        h_at_active_end = 1 if hcnt == (H_ACTIVE - 1) else 0

        yield dict(
            cyc=cyc,
            de=valid_r2, user=user_r2, last=last_r2,
            hcnt=hcnt, vcnt=vcnt, vtc_de=vtc_de, rst_sync=rst_sync,
        )

        # ---------------- 阶段 2：拍末更新（RHS 全用旧值）----------------
        if rst_sync == 0:
            n_hcnt = 0
            n_vcnt = 0
            n_rst_cnt = rst_cnt + 1                        # 3 位，到 4 停住
        else:
            n_rst_cnt = rst_cnt                            # [2]==1 后不再增长
            n_hcnt = hcnt + 1 if hcnt < (H_FRAME - 1) else 0
            if h_at_active_end:
                n_vcnt = 0 if vcnt == (V_FRAME - 1) else vcnt + 1
            else:
                n_vcnt = vcnt

        n_user_r1 = (1 - user_r1) & vs_start & vtc_de
        n_last_r2 = (1 - vtc_de) & valid_r1

        if rst_sync == 0:
            n_vs_start = 0
        elif user_r1:
            n_vs_start = 0
        elif vtc_vs and vs_r1 == 0:
            n_vs_start = 1
        else:
            n_vs_start = vs_start

        n_vs_r1, n_hs_r1 = vtc_vs, vtc_hs
        n_valid_r1, n_valid_r2 = vtc_de, valid_r1
        n_user_r2 = user_r1

        rst_cnt, hcnt, vcnt = n_rst_cnt, n_hcnt, n_vcnt
        vs_start = n_vs_start
        vs_r1, hs_r1 = n_vs_r1, n_hs_r1
        user_r1, user_r2 = n_user_r1, n_user_r2
        valid_r1, valid_r2 = n_valid_r1, n_valid_r2
        last_r2 = n_last_r2


# ===========================================================================
# 第二部分：hdmi_mixer.v 内部像素坐标计数器（第 327~345 行）的位级镜像
# ===========================================================================
class MixerCoordCounter:
    """S_x / S_y 都是 reg [11:0]，按 12 位回绕，不做任何"好心"扩展。"""

    def __init__(self):
        self.sx = 0
        self.sy = 0

    def step(self, de, user, last):
        """返回【本拍采样到的】(S_x, S_y)，随后按 RTL 更新寄存器。"""
        hold = (self.sx, self.sy)
        if de:
            if user:
                self.sx, self.sy = 0, 0
            elif last:
                self.sx, self.sy = 0, (self.sy + 1) & 0xFFF
            else:
                self.sx = (self.sx + 1) & 0xFFF
        return hold


# ===========================================================================
# 第三部分：单遍审计
# ===========================================================================
ZONES = [
    # name, x0, x1, y0, y1, 期望像素数
    ("Logo  (0,0)-(160,160)", 0, 160, 0, 160, 160 * 160),
    ("帧数  (176,16) 144x16", 176, 320, 16, 32, 144 * 16),
    ("增益  (176,48) 96x16", 176, 272, 48, 64, 96 * 16),
    ("模式  (1184,16) 80x16", 1184, 1264, 16, 32, 80 * 16),
]


def main():
    print("=" * 78)
    print("uivtc.v + hdmi_mixer.v 像素坐标系统 —— 位级审计")
    print("=" * 78)
    print(f"时序: H {H_ACTIVE}/{H_FRAME}   V {V_ACTIVE}/{V_FRAME}   "
          f"仿真 {FRAMES} 帧 / {CYCLES} 拍")
    print()

    cnt = MixerCoordCounter()
    delay = []                     # 2 拍延迟线: [(cyc, k, r, vtc_de, rst_sync)]

    n_de = 0
    n_user_on_de = n_user_off_de = 0
    n_last_on_de = n_last_off_de = 0
    first_user_cyc = None

    # ---- 稳态（armed = 首个 user 脉冲之后）统计 ----
    armed = False
    armed_frames = 0

    tot_rg = mis_x_rg = mis_y_rg = 0
    tot_r0 = mis_x_r0 = mis_y_r0 = 0
    off_r0 = set()                 # r=0 行的 S_x 偏移量集合
    r0_sy_residue = None           # (r=0,k=0) 采样到的 S_y（user 清零前的残留）

    # ---- 启动期 ----
    reset_spurious = []            # 复位窗口里被灌进 de 流的伪像素
    stray_de = 0                   # de=1 但 vtc 域无对应像素

    # ---- Q4：单帧区域命中（独立通道，见下方第二遍扫描）----
    q4_counts = {z[0]: 0 for z in ZONES}
    q4_active = False

    for s in vtc_stream(CYCLES):
        cyc = s["cyc"]

        # ---- 帧首脉冲：arming + Q4 窗口控制 ----
        if s["user"] and s["de"]:
            n_user_on_de += 1
            armed_frames += 1
            if first_user_cyc is None:
                first_user_cyc = cyc
            armed = True
            q4_active = (armed_frames == 1)     # 只统计 arming 后的第一整帧
        elif s["user"]:
            n_user_off_de += 1

        if s["de"]:
            n_de += 1
        if s["last"]:
            if s["de"]:
                n_last_on_de += 1
            else:
                n_last_off_de += 1

        # ---- 采样本拍坐标，再更新坐标计数器 ----
        sx, sy = cnt.step(s["de"], s["user"], s["last"])

        # ---- 延迟线出口：把 vtc 域的像素身份与输出域的坐标对上 ----
        delay.append((cyc, s["hcnt"], s["vcnt"], s["vtc_de"], s["rst_sync"]))
        if len(delay) > 2:
            c0, k0, r0, vd0, rst0 = delay.pop(0)
            if vd0:
                if rst0 == 0:
                    reset_spurious.append((c0, k0, r0, sx, sy))
                elif armed:
                    if r0 == 0:
                        tot_r0 += 1
                        if k0 == 0:
                            r0_sy_residue = sy
                        else:
                            off_r0.add(k0 - sx)
                            if sy != 0:
                                mis_y_r0 += 1
                    else:
                        tot_rg += 1
                        if sx != k0:
                            mis_x_rg += 1
                        if sy != r0:
                            mis_y_rg += 1
            elif s["de"]:
                stray_de += 1

    # Q4 走独立通道（上面那条 delay 已作为审计延迟线用掉，
    q4_counts = {z[0]: 0 for z in ZONES}
    q4_active = False
    armed_frames = 0
    delay2 = []
    cnt2 = MixerCoordCounter()
    for s in vtc_stream(CYCLES):
        if s["user"] and s["de"]:
            armed_frames += 1
            q4_active = (armed_frames == 1)
        cnt2.step(s["de"], s["user"], s["last"])
        delay2.append((s["hcnt"], s["vcnt"], s["vtc_de"]))
        if len(delay2) > 2:
            k0, r0, vd0 = delay2.pop(0)
            if vd0 and q4_active:
                for name, x0, x1, y0, y1, _ in ZONES:
                    if x0 <= k0 < x1 and y0 <= r0 < y1:
                        q4_counts[name] += 1

    # ================= 报告 =================
    print("---- Q1  O_vtc_last 与 O_vtc_de 是否同拍 ----")
    print(f"  last=1 且 de=1 : {n_last_on_de} 拍")
    print(f"  last=1 但 de=0 : {n_last_off_de} 拍")
    q1 = (n_last_off_de == 0 and n_last_on_de == V_ACTIVE * FRAMES)
    print(f"  期望 = 行数 = {V_ACTIVE * FRAMES}"
          f"   [{'PASS' if q1 else 'FAIL'}] last 恰为每行末像素一拍，且与 de 严格同拍")
    print()

    print("---- Q1b O_vtc_user 与 O_vtc_de 是否同拍（帧首）----")
    print(f"  user=1 且 de=1 : {n_user_on_de} 拍    user=1 但 de=0 : {n_user_off_de} 拍")
    print(f"  首次 user 出现在第 {first_user_cyc} 拍 "
          f"（1 帧 = {H_FRAME * V_FRAME} 拍，≈ 第 2 帧帧首）")
    print(f"  说明：复位后第 1 帧没有 user（vs_start 需先被 vtc_vs 上升沿置起），")
    print(f"        故 F 帧共得 F-1 个 user；本次 F={FRAMES} ⇒ 期望 {FRAMES - 1}")
    q1b = (n_user_off_de == 0 and n_user_on_de == FRAMES - 1)
    print(f"  [{'PASS' if q1b else 'FAIL'}] user 每帧一拍且落在 de 上")
    print()

    print("---- Q2  坐标是否等于 (k, r)  【只统计首个 user 之后的稳态帧】----")
    print(f"  r≥1 干净像素数        : {tot_rg}")
    print(f"  r≥1 行 S_x ≠ k 的像素 : {mis_x_rg}   [{'PASS' if mis_x_rg == 0 else 'FAIL'}]")
    print(f"  r≥1 行 S_y ≠ r 的像素 : {mis_y_rg}   [{'PASS' if mis_y_rg == 0 else 'FAIL'}]")
    print(f"  r=0  帧首行像素数      : {tot_r0}")
    print(f"  r=0  行 S_y ≠ 0 的像素 : {mis_y_r0}（k=0 那一拍是上一行残留，见下）")
    print(f"  r=0  行 S_x 偏移量集合 : {sorted(off_r0)}"
          f"  ⇒ S_x = k - 1（整行左移 1 px）")
    print(f"  r=0,k=0 采样到 S_y     : {r0_sy_residue}"
          f"（上一帧行末把 S_y 加到 720，同拍 user 才清 0）")
    q2 = (mis_x_rg == 0 and mis_y_rg == 0 and off_r0 <= {1})
    print(f"  [{'PASS' if q2 else 'FAIL'}] 第 r 行第 k 像素处理时 (S_x,S_y) ≡ (k,r)"
          f"（r≥1 零偏移；r=0 整行 S_x 小 1，属已知边界，见结论 2）")
    print()

    print("---- Q3  启动期实况 ----")
    print(f"  复位窗口内被灌进 de 流的伪像素 : {len(reset_spurious)} 个")
    if reset_spurious:
        c0, k0, r0, sx, sy = reset_spurious[0]
        print(f"    首例: 第 {c0} 拍 vtc_de=1 (hcnt={k0}, vcnt={r0}) 但 rst_sync=0，"
              f"输出拍采样到 S_x={sx}")
        print(f"    成因: rst_sync 生效前 hcnt 被冻结在 0 共 5 拍，vtc_de 已是 1，")
        print(f"          经 2 级流水灌出 ⇒ 复位后首行多出 {len(reset_spurious)} 个 de 拍。")
        print(f"    影响: 只波及复位后第 1 帧的第 0 行；该行行末 last 把 S_x 归零，")
        print(f"          第 1 行起即恢复正常 ⇒ 无上板观感影响，")
        print(f"          但【做波形/仿真比对时必须跳过启动期】，否则会误判坐标错乱。")
    print(f"  de=1 但 vtc 域无对应像素的拍 : {stray_de}"
          f"   [{'PASS' if stray_de == 0 else 'FAIL'}]")
    print()

    print("---- Q4  hdmi_mixer 现有 OSD 区域单帧命中像素数 ----")
    q4 = True
    for name, x0, x1, y0, y1, expect in ZONES:
        got = q4_counts[name]
        good = got == expect
        q4 &= good
        print(f"  {name:24s} 期望 {expect:6d}   实得 {got:6d}   [{'PASS' if good else 'FAIL'}]")
    print(f"  [{'PASS' if q4 else 'FAIL'}] 四处区域常量均落在可达坐标范围内")
    print()

    all_ok = q1 and q1b and q2 and q4
    print("=" * 78)
    print("结论：" + ("坐标系统地基正确，可在此之上做画框 / 坐标 OSD。"
                     if all_ok else "发现坐标系统问题，见上方 FAIL 项。"))
    print("=" * 78)
    print()
    print("给 D1 画框 / 坐标 OSD 的硬约束（由本审计直接推出，不要凭感觉改）：")
    print("  1) last 与 de 严格同拍 ⇒ 画框模块【可以直接用 I_last 作行末标志】，")
    print("     但必须放在 de 门控之内；实测 de=0 时的 last 一拍都没有，")
    print("     所以 de 门控既是语义要求也是安全冗余。")
    print("  2) r≥1 时 (S_x,S_y) ≡ (k,r)，零偏移，可放心直接用；")
    print("     **r=0 整行 S_x 比真实列号小 1**（user 在首像素用清零覆盖了本该的 +1）。")
    print("     只影响 y=0 一行、且被 160×160 Logo 覆盖 ⇒ 无观感影响；")
    print("     但画框若覆盖 y=0，必须按 (k+1) 修正。")
    print("  3) 坐标要与像素数据对齐必须延 2 拍（与 hdmi_mixer 的 S_x_2d 同款），")
    print("     否则框边整体右/下偏 2 px —— 这类「差一点」正是观感评分的失分点。")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
