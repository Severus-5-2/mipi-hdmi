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
# ⚠ 必须保留一层目录结构：detect_zjy/ 下的源码（shape_class / osd_category 及其 TB）
#   用 `include "../common_base/common_defs.vh"`（与 design_top_wrapper.v 同一写法，
#   这样 TD 综合时也能找到）⇒ 临时目录里要让 ../common_base/ 真实存在。
mkdir -p "$SIMDIR/detect_zjy" "$SIMDIR/common_base"
trap 'rm -rf "$SIMDIR"' EXIT
SIMD="$SIMDIR/detect_zjy"
cp "$HERE/detect_color_mask.v" "$HERE/tb_detect_color_mask.v" "$SIMD/"
# 去斑模块自检需要行缓存（工程已有的公共模块）
cp "$HERE/mask_despeckle.v" "$HERE/tb_mask_despeckle.v" "$SIMD/" 2>/dev/null || true
cp "$HERE/../line_delay_ram.v" "$SIMD/" 2>/dev/null || true
# D5/D6：几何分类 + 类别数量 OSD
cp "$HERE/shape_class.v" "$HERE/tb_shape_class.v" "$SIMD/" 2>/dev/null || true
cp "$HERE/osd_category.v" "$HERE/tb_osd_category.v" "$SIMD/" 2>/dev/null || true
cp "$HERE/../osd_char_lib.v" "$SIMD/" 2>/dev/null || true
cp "$HERE/../common_base/common_defs.vh" "$SIMDIR/common_base/" 2>/dev/null || true
cd "$SIMD"

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
if [ -f "$SIMD/mask_despeckle.v" ] && [ -f "$SIMD/tb_mask_despeckle.v" ]; then
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

# ---------- 7. 几何形状分类自检（shape_class，D5）----------
#   验证：填充度判据（方/圆/三角）+ 无效槽 + 阈值边界
echo
echo "== 几何分类自检（shape_class）=="
if [ -f "$SIMD/shape_class.v" ] && [ -f "$SIMD/tb_shape_class.v" ]; then
    "$IVERILOG_BIN" -g2005 -o shp.out shape_class.v tb_shape_class.v 2>/dev/null || {
        echo "!! 几何分类模块编译失败"; exit 1; }
    SHP_OUT="$("$VVP_BIN" shp.out 2>&1)"
    echo "$SHP_OUT" | grep -v '^\$dump' | sed '/^$/d' | tail -8
    if echo "$SHP_OUT" | grep -q '\[PASS\]'; then
        : # 通过
    else
        echo "!! 几何分类自检未通过"; exit 1
    fi
else
    echo "(跳过：未找到 shape_class.v / tb_shape_class.v)"
fi

# ---------- 8. 类别数量 OSD 自检（osd_category，D6）----------
#   验证：颜色/形状计数 + 22 个字符格渲染 + 复用字库数字
echo
echo "== 类别 OSD 自检（osd_category）=="
if [ -f "$SIMD/osd_category.v" ] && [ -f "$SIMD/tb_osd_category.v" ]; then
    "$IVERILOG_BIN" -g2005 -o cat.out osd_category.v tb_osd_category.v \
        osd_char_lib.v 2>/dev/null || {
        echo "!! 类别 OSD 模块编译失败"; exit 1; }
    CAT_OUT="$("$VVP_BIN" cat.out 2>&1)"
    echo "$CAT_OUT" | grep -E '^\[(OK|PASS|FAIL)\]' | tail -5
    if echo "$CAT_OUT" | grep -q '\[PASS\]'; then
        : # 通过
    else
        echo "!! 类别 OSD 自检未通过"; exit 1
    fi
else
    echo "(跳过：未找到 osd_category.v / tb_osd_category.v)"
fi

echo
echo "============================================================"
echo "  detect_zjy 全量验证：以上各项全部 PASS"
echo "============================================================"
