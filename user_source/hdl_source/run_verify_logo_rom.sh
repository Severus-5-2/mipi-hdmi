#!/usr/bin/env bash
# anlogic_logo_rom 位级等价对拍：
#   新「ERAM 数组版」 vs 旧「组合 case 版」
#   逐地址比对全部 20427 个地址。
#
# 参考模型 sim_hdl/logo_rom_ref_case.v 是「改造前的 case 版」，
# 体积 ~294KB，不入库（见 .gitignore）。本脚本会在缺失时自动从 git 历史重建。
set -e
cd "$(dirname "$0")"

IVERILOG="/c/iverilog/bin/iverilog.exe"
VVP="/c/iverilog/bin/vvp.exe"

REF="sim_hdl/logo_rom_ref_case.v"

# ---- 参考模型缺失 / 需要刷新时，从 git 历史重建 ----
#   取「本文件改造前」的版本：即 anlogic_logo_rom.v 最后一个 case 版提交。
#   这里用最稳妥的办法：拿 git log 里第一个包含 "case(I_addr)" 的历史版本。
if [ ! -f "$REF" ] || [ "$1" = "--regen" ]; then
    mkdir -p sim_hdl
    echo "[info] 生成参考模型 $REF ..."
    # 改造前的版本 = 本文件被改写前的 HEAD（最后一次提交）。
    # 注意：不能用 `git log | tail -1` —— 那会取到最老的版本（更早的纯组合 ROM，
    #       没有 I_clk 端口，端口对不上会报 "port I_clk is not a port of u_ref"）。
    REV="${LOGO_REF_REV:-HEAD}"
    (cd ../.. && git show "$REV:user_source/hdl_source/anlogic_logo_rom.v") \
        | sed 's/^module anlogic_logo_rom (/module logo_rom_ref_case (/' > "$REF"
    if ! grep -q "module logo_rom_ref_case" "$REF"; then
        echo "[FAIL] 参考模型生成失败，请设置 LOGO_REF_REV=<改造前commit> 后重跑"
        exit 1
    fi
    if ! grep -q "I_clk" "$REF"; then
        echo "[FAIL] 参考模型缺少 I_clk 端口 —— 该 commit 不是「同步读 case 版」。"
        echo "       请设置 LOGO_REF_REV=<改造前commit>（应含 I_clk + case(I_addr)）后重跑。"
        exit 1
    fi
    echo "[info] 参考模型已生成（来自 $REV）"
fi

echo "编译 ..."
"$IVERILOG" -g2005 -Wall -o /tmp/tb_logo_rom.vvp \
    tb_logo_rom_equiv.v \
    anlogic_logo_rom.v \
    "$REF"

echo "跑仿真 ..."
"$VVP" /tmp/tb_logo_rom.vvp
