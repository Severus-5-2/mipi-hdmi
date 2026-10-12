#!/usr/bin/env bash
#=============================================================================
#  run_verify.sh — detect_color_mask 一键验证：编译 → 仿真 → 位级对拍
#=============================================================================
#  依赖：Icarus Verilog（iverilog + vvp）与 Python 3。
#        脚本会自动在常见位置找 iverilog；也可用环境变量 IVERILOG 指定。
#
#  用法：
#      bash run_verify.sh
#  退出码：0 = 全部通过；非 0 = 有失败项
#=============================================================================
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"

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
PYTHON_BIN="${PYTHON:-$(command -v python3 || command -v python || true)}"

if [ -z "$IVERILOG_BIN" ] || [ -z "$VVP_BIN" ]; then
    echo "!! 找不到 iverilog / vvp。请安装 Icarus Verilog，或用 IVERILOG=... VVP=... 指定路径。"
    exit 2
fi
if [ -z "$PYTHON_BIN" ]; then
    echo "!! 找不到 python。"
    exit 2
fi

echo "iverilog : $IVERILOG_BIN"
echo "vvp      : $VVP_BIN"
echo "python   : $PYTHON_BIN"
echo

# ---------- 2. 临时仿真目录 ----------
SIMDIR="$(mktemp -d 2>/dev/null || echo "${HERE}/_sim_tmp")"
mkdir -p "$SIMDIR"
trap 'rm -rf "$SIMDIR"' EXIT
cp "$HERE/detect_color_mask.v" "$HERE/tb_detect_color_mask.v" "$SIMDIR/"
# 去斑模块自检需要行缓存（工程已有的公共模块）
cp "$HERE/mask_despeckle.v" "$HERE/tb_mask_despeckle.v" "$SIMDIR/" 2>/dev/null || true
cp "$HERE/../line_delay_ram.v" "$SIMDIR/" 2>/dev/null || true
cd "$SIMDIR"

# ---------- 3. 编译 ----------
echo "== 编译 =="
"$IVERILOG_BIN" -g2005 -Wall -o sim.out detect_color_mask.v tb_detect_color_mask.v || {
    echo "!! 编译失败"; exit 1; }

# ---------- 4. 仿真 ----------
echo
echo "== 仿真 =="
"$VVP_BIN" sim.out | tail -3

# ---------- 5. 位级对拍 ----------
echo
echo "== 对拍 =="
"$PYTHON_BIN" "$HERE/verify_color_mask.py" \
    --rtl "$HERE/detect_color_mask.v" \
    --out "$SIMDIR/rtl_out.txt"

# ---------- 6. 去斑模块自检（mask_despeckle）----------
#   验证：水平窗去斑 + 跨行垂直投票 + 列/行对齐
echo
echo "== 去斑模块自检（mask_despeckle）=="
if [ -f "$SIMDIR/mask_despeckle.v" ] && [ -f "$SIMDIR/tb_mask_despeckle.v" ]; then
    "$IVERILOG_BIN" -g2005 -o dspk.out mask_despeckle.v tb_mask_despeckle.v \
        line_delay_ram.v 2>/dev/null || {
        echo "!! 去斑模块编译失败"; exit 1; }
    DSPK_OUT="$("$VVP_BIN" dspk.out 2>&1)"
    echo "$DSPK_OUT" | grep -v '^\$dump' | sed '/^$/d' | tail -12
    if echo "$DSPK_OUT" | grep -q '\[PASS\]'; then
        : # 通过
    else
        echo "!! 去斑模块自检未通过"; exit 1
    fi
else
    echo "(跳过：未找到 mask_despeckle.v / tb_mask_despeckle.v)"
fi
