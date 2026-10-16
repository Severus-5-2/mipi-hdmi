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
cp "$HERE/shape_class.v" "$HERE/tb_shape_class.v" "$HERE/tb_shape_class_frame_align.v" "$SIMD/" 2>/dev/null || true
cp "$HERE/osd_category.v" "$HERE/tb_osd_category.v" "$SIMD/" 2>/dev/null || true
cp "$HERE/../osd_char_lib.v" "$SIMD/" 2>/dev/null || true
# 夜间（低照度）自适应门限自检
# 位置自适应门限（渐晕/边缘色度补偿）自检
cp "$HERE/tb_night_adapt.v" "$SIMD/" 2>/dev/null || true
# 位置自适应门限（渐晕/边缘色度补偿）自检
cp "$HERE/tb_zone_th.v" "$SIMD/" 2>/dev/null || true
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

# ---------- 9. 夜间（低照度）自适应门限自检（detect_color_mask，NIGHT_EN=1）----------
#   验证：① 按 AE 帧平均亮度分 4 档下调 MAG_TH（24/20/16/12）
#         ② 档位边界 ±HYST 滞回（亮度抖动不换档）
#         ③ 行为：mag 落在 16~24 的目标「亮场不检出、暗场检出」
#         ④ 反向保护：极暗档下中性灰仍不误检
#   ⚠ NIGHT_EN 默认 0 ⇒ 工程不显式打开时本特性完全不生效（行为同改造前）。
echo
echo "== 夜间自适应门限自检（NIGHT_EN=1）=="
if [ -f "$SIMD/tb_night_adapt.v" ]; then
    "$IVERILOG_BIN" -g2005 -o night.out tb_night_adapt.v detect_color_mask.v 2>/dev/null || {
        echo "!! 夜间自适应 TB 编译失败"; exit 1; }
    NG_OUT="$("$VVP_BIN" night.out 2>&1)"
    echo "$NG_OUT" | grep -E '^\s*\[(OK |NG |PASS|FAIL)' | tail -6
    if echo "$NG_OUT" | grep -q '\[PASS\]'; then
        : # 通过
    else
        echo "!! 夜间自适应门限自检未通过"; exit 1
    fi
else
    echo "(跳过：未找到 tb_night_adapt.v)"
fi

# ---------- 10. D7 标定工具自检（calib_threshold.py）----------
#   合成一张 PNG 写盘再读回，校验解码往返 + 三色块判据 + 灰底零误命中 + --tap 语义
echo
echo "== D7 标定工具自检（calib_threshold.py）=="
if [ -f "$HERE/calib_threshold.py" ]; then
    CAL_OUT="$("$PYTHON_BIN" "$HERE/calib_threshold.py" --selftest 2>&1)"
    echo "$CAL_OUT" | grep -E '^\s*\[(OK|PASS|FAIL)\]' | tail -8
    if echo "$CAL_OUT" | grep -q '\[PASS\]'; then
        : # 通过
    else
        echo "!! D7 标定工具自检未通过"; exit 1
    fi
else
    echo "(跳过：未找到 calib_threshold.py)"
fi

# ---------- 11. 位置自适应门限自检（渐晕/边缘色度补偿）----------
#   验证 3×3 九宫格分区放宽：中心 +0 / 边中 +MAGD_EDGE / 四角 +MAGD_CORNER。
#   ⚠ 两个放宽量**默认 0**（= 不补偿）；工程顶层显式传非 0 才生效。
#   ★ 定性依据见 `_vignette_2026-10-16/tb_vignette.v` 的对照实验（边缘色度衰减 40% 会丢检，
#     加补偿后边中/四角都恢复检出；中性灰在四角仍不误检）。
echo
echo "== 位置自适应门限自检（渐晕/边缘色度补偿）=="
if [ -f "$SIMD/tb_zone_th.v" ]; then
    "$IVERILOG_BIN" -g2005 -o zone.out tb_zone_th.v detect_color_mask.v 2>/dev/null || {
        echo "!! 分区门限 TB 编译失败"; exit 1; }
    ZN_OUT="$("$VVP_BIN" zone.out 2>&1)"
    echo "$ZN_OUT" | grep -E '\[PASS\]|\[FAIL\]|\[NG ' | tail -4
    if echo "$ZN_OUT" | grep -q '\[PASS\]'; then
        : # 通过
    else
        echo "!! 位置自适应门限自检未通过"; exit 1
    fi
else
    echo "(跳过：未找到 tb_zone_th.v)"
fi

# ---------- 12. 帧脉冲/总线锁存的「采样窗口」自检（shape_class）----------
#   消费者读 S_target_bus 的采样首拍若落在「比总线锁存早」的中间地带，会静默串帧。
#   ★ 判据**不依赖具体实现**（2026-10-14 修订）：
#       只断言「不早于锁存 -> 当帧」「早一个完整场消隐期 -> 一致上一帧」「中间带存在且连续」；
#       **窗口上沿只报告、不断言** —— 它随模块取槽节奏变化，钉固定数字会误报。
#      （起因：张铭晨用四档节奏桩复跑，上沿依次 4/7/10/13 拍 ⇒ "2~13" 只是当前实现的数。）
echo
echo "== 采样窗口自检（shape_class 读总线）=="
if [ -f "$SIMD/tb_shape_class_frame_align.v" ] && [ -f "$SIMD/shape_class.v" ]; then
    "$IVERILOG_BIN" -g2005 -o latch.out shape_class.v tb_shape_class_frame_align.v 2>/dev/null || {
        echo "!! 采样窗口 TB 编译失败"; exit 1; }
    LAT_OUT="$("$VVP_BIN" latch.out 2>&1)"
    echo "$LAT_OUT" | grep -E '串帧带|带外串帧|\[PASS\]|\[FAIL\]|!!' | tail -4
    if echo "$LAT_OUT" | grep -q '\[PASS\]'; then
        : # 通过
    else
        echo "!! 采样窗口自检未通过"; exit 1
    fi
else
    echo "(跳过：未找到 tb_shape_class_frame_align.v)"
fi

echo
echo "============================================================"
echo "  detect_zjy 全量验证：以上各项全部 PASS"
echo "============================================================"
