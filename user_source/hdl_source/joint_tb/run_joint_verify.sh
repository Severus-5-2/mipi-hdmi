#!/usr/bin/env bash
#=============================================================================
#  run_joint_verify.sh — 合板联合验证一键脚本
#=============================================================================
#  目的：把「我方的检测链」与「张铭晨的跟踪/运动链」串起来跑一遍，
#        验证两边接上之后**跑通 + 跑稳**。
#
#  覆盖：mask_despeckle → conn_label → conn_features → target_bus
#        → target_assoc（D5 跟踪）→ box_draw（画框）
#
#  用法：
#      bash run_joint_verify.sh
#  退出码：0 = 通过；非 0 = 有失败项
#=============================================================================
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"          # user_source/hdl_source

# ---------- 1. 找 iverilog / vvp ----------
find_tool() {
    local name="$1"
    if command -v "$name" >/dev/null 2>&1; then command -v "$name"; return 0; fi
    for p in \
        "/c/iverilog/bin/${name}.exe" \
        "/c/Program Files/iverilog/bin/${name}.exe" \
        "/d/iverilog/bin/${name}.exe"; do
        [ -x "$p" ] && { echo "$p"; return 0; }
    done
    return 1
}
IVERILOG_BIN="${IVERILOG:-$(find_tool iverilog || true)}"
VVP_BIN="${VVP:-$(find_tool vvp || true)}"
if [ -z "$IVERILOG_BIN" ] || [ -z "$VVP_BIN" ]; then
    echo "!! 找不到 iverilog / vvp。可用 IVERILOG=... VVP=... 指定路径。"
    exit 2
fi
echo "iverilog : $IVERILOG_BIN"
echo "vvp      : $VVP_BIN"
echo

# ---------- 2. 准备临时仿真目录 ----------
#   ⚠ 必须保留目录层次：TB 里写的是 #include "../common_base/common_defs.vh"
#     以及各模块的相对路径。扁平拷贝会让相对 include 断掉。
SIMDIR="$(mktemp -d 2>/dev/null || echo "${HERE}/_joint_sim_tmp")"
mkdir -p "$SIMDIR/joint_tb" "$SIMDIR/common_base" "$SIMDIR/detect_zjy" "$SIMDIR/track_box/rtl"
trap 'rm -rf "$SIMDIR"' EXIT

cp "$HERE/tb_joint_chain.v"                              "$SIMDIR/joint_tb/"
cp "$ROOT/common_base/common_defs.vh"                    "$SIMDIR/common_base/"
cp "$ROOT/detect_zjy/mask_despeckle.v"                   "$SIMDIR/detect_zjy/"
cp "$ROOT/line_delay_ram.v"                              "$SIMDIR/"
cp "$ROOT/common_base/conn_label.v"                      "$SIMDIR/common_base/"
cp "$ROOT/common_base/conn_features.v"                   "$SIMDIR/common_base/"
cp "$ROOT/common_base/target_bus.v"                      "$SIMDIR/common_base/"
cp "$ROOT/track_box/rtl/target_assoc.v"                  "$SIMDIR/track_box/rtl/"
cp "$ROOT/track_box/rtl/box_draw.v"                      "$SIMDIR/track_box/rtl/"

cd "$SIMDIR/joint_tb" || exit 2

echo "== 编译联合 TB =="
"$IVERILOG_BIN" -g2005 -I../common_base -I../detect_zjy -I../track_box/rtl \
    -o joint.vvp \
    tb_joint_chain.v \
    ../detect_zjy/mask_despeckle.v \
    ../line_delay_ram.v \
    ../common_base/conn_label.v \
    ../common_base/conn_features.v \
    ../common_base/target_bus.v \
    ../track_box/rtl/target_assoc.v \
    ../track_box/rtl/box_draw.v 2>/dev/null || {
    echo "!! 联合 TB 编译失败"; exit 1; }
echo "   编译通过"
echo

echo "== 运行联合仿真（5 组场景 + 显示侧）=="
OUT="$("$VVP_BIN" joint.vvp 2>&1)"
echo "$OUT" | grep -E '^---|^  f|^  \[|^\[PASS\]|^\[FAIL\]|^  框|^  命中|^  消失后' | tail -40
echo
if echo "$OUT" | grep -q '\[PASS\]'; then
    echo "==> 联合验证通过 ✓"
    RC=0
else
    echo "==> 联合验证失败 ✗（见上方 !! 项）"
    RC=1
fi

# 把逐帧记录留在原地，便于事后比对
if [ -f joint_out.txt ]; then
    cp joint_out.txt "$HERE/joint_out.txt" 2>/dev/null || true
    echo "逐帧记录已保存：$( [ -w "$HERE" ] && echo "$HERE/joint_out.txt" || echo "（临时目录）" )"
fi
exit $RC
