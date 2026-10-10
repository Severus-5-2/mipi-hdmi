#!/usr/bin/env bash
#=============================================================================
#  run_verify_common.sh  —  common_base 公共模块一键验证
#=============================================================================
#  做两件事：
#    ① 语法编译：pix_counter / ycbcr_convert / target_bus
#    ② 等价性对拍：ycbcr_convert 必须与 detect_zjy/detect_color_mask 逐位一致
#
#  用法（在 common_base 目录下）：
#      bash run_verify_common.sh
#
#  依赖：Icarus Verilog 12（C:\iverilog），免费无授权。
#        本机 ModelSim 授权已损坏，故不用 ModelSim。
#=============================================================================

set -u

IVERILOG="C:/iverilog/bin/iverilog.exe"
VVP="C:/iverilog/bin/vvp.exe"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

DETECT_DIR="../detect_zjy"
TMP="${TMPDIR:-/tmp}"
OUT_COMPILE="$TMP/cb_compile.out"
OUT_EQUIV="$TMP/cb_equiv.out"

echo "==================================================="
echo " common_base 公共模块验证"
echo " 目录: $SCRIPT_DIR"
echo "==================================================="

# -------- ① 语法编译 --------
echo ""
echo "[1/2] 语法编译 ..."
"$IVERILOG" -I . -o "$OUT_COMPILE" -tnull \
    pix_counter.v ycbcr_convert.v target_bus.v 2>&1
if [ $? -ne 0 ]; then
    echo "  ❌ 编译失败"
    exit 1
fi
echo "  ✅ 编译通过（pix_counter / ycbcr_convert / target_bus）"

# -------- ② 等价性对拍 --------
echo ""
echo "[2/2] 等价性对拍：ycbcr_convert vs detect_color_mask ..."
if [ ! -f "$DETECT_DIR/detect_color_mask.v" ]; then
    echo "  ⚠ 找不到 $DETECT_DIR/detect_color_mask.v，跳过等价性验证"
    exit 0
fi

"$IVERILOG" -I . -I "$DETECT_DIR" -o "$OUT_EQUIV" \
    tb_ycbcr_equiv.v pix_counter.v ycbcr_convert.v \
    "$DETECT_DIR/detect_color_mask.v" 2>&1
if [ $? -ne 0 ]; then
    echo "  ❌ 编译失败"
    exit 1
fi

"$VVP" "$OUT_EQUIV" 2>&1 | grep -E "PASS|FAIL|MISMATCH"
echo ""
echo "==================================================="
echo " 完成。若上面出现 [PASS]，则公共模块与 D1 基线逐位一致。"
echo "==================================================="
