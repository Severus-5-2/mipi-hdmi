#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#=============================================================================
#  calib_threshold.py —— 方案B D7：用**真实目标物**标定 Cb/Cr 门限
#=============================================================================
#  项目：MIPI-HDMI 实时视频图像处理系统 · 方案B
#  负责人：张金艺   归属：user_source/hdl_source/detect_zjy/
#
#  依据：
#    · `doc/开发方案B-目标检测跟踪与运动检测.md` —— D7「用真实目标物标定 Cb/Cr 阈值」
#    · `common_base/common_defs.vh §5b`  —— MAG_TH / RATIO10 / G_AXIS_TH 宏
#    · `detect_color_mask.v`              —— v2 曝光自适应判据（本脚本严格复现）
#    · `sat_enhance.v`                    —— 取数点前的饱和度增强（1.5×）
#
#-----------------------------------------------------------------------------
#  一、它解决什么问题
#-----------------------------------------------------------------------------
#  D1–D6 的门限（MAG_TH=24 / RATIO10=13 / G_AXIS_TH=10）是**按仿真与经验**定的。
#  现场换一批目标物、换一种光照，最优门限会变。D7 就是"拿实物量一遍"。
#
#  本脚本把**硬件链路原样搬到 Python**，对一张实拍图算出：
#    · 目标区的 (Cb,Cr) 到底落在哪（幅度多少、两轴比例多少）
#    · 背景区的噪声/干扰色有多强
#    · 每组 `MAG_TH × G_AXIS_TH` 下的 **目标检出率** 与 **背景误命中率**
#    · 满足约束的最佳取值，并直接给出**可粘贴的 common_defs.vh 宏块**
#
#-----------------------------------------------------------------------------
#  二、★ 严格同源（这是本脚本的全部价值所在）
#-----------------------------------------------------------------------------
#  · YCbCr 系数：77/150/29、-43/-85/128、128/-107/-21，+128 四舍五入、>>8
#      —— 与 `detect_color_mask.v` / `common_base/common_defs.vh` 逐字一致
#  · 饱和度增强：Y=(77R+150G+29B)>>8，dC=C−Y，|dC|≤DEADZONE 不增强，
#      否则 C'=Y+(dC×SAT_GAIN)>>6（算术右移）  —— 与 `sat_enhance.v` 一致
#  · 判据：与 `detect_color_mask.v` §判据 一致（红/蓝用比值、绿用双轴）
#  · 门限默认值：**从 `common_defs.vh` 现场读取**（不在脚本里硬编码）
#
#-----------------------------------------------------------------------------
#  三、★ 取数点：选对 --tap（最容易搞错的一步）
#-----------------------------------------------------------------------------
#  检测的取数点是 `S_video_sat` —— **饱和度增强（1.5×）之后**。
#  所以要看你的输入图是什么：
#
#    --tap post （默认）
#        图**已经是**工程输出的画面（HDMI 屏截图 / 抓帧 / 手机拍屏）。
#        它已经含过 1.5× 增强了 ⇒ 本脚本**不再重复增强**。
#
#    --tap scene
#        图是**手机直拍实物**（完全没经过本工程的 ISP）。
#        本脚本会**补上** 1.5× 饱和度增强，模拟"进到检测器时"的样子。
#
#  两者结果可能差很多。拿不准就先跑 `--tap post`。
#
#-----------------------------------------------------------------------------
#  四、用法
#-----------------------------------------------------------------------------
#  1) 拿一张图（PNG；若是 JPEG 先用画图/看图的"另存为 PNG"转一下）
#  2) 找出目标所在的矩形区域（像素坐标 x0,y0,x1,y1，左上为原点）
#
#      python calib_threshold.py photo.png --tgt 400,250,760,520
#      python calib_threshold.py photo.png --tgt 400,250,760,520 --bg 0,0,1280,120
#      python calib_threshold.py photo.png --tgt 400,250,760,520 --tap scene
#      python calib_threshold.py photo.png --tgt 400,250,760,520 --mask-out mask.png
#
#  3) 看输出的「推荐参数」与「宏块」，改到 `common_defs.vh §5b` 一处即可（顶层引宏）
#
#  自检（不需要任何素材，会自己造一张 PNG 再读回来）：
#      python calib_threshold.py --selftest
#
#  依赖：**只用 Python 标准库**（zlib/struct/argparse）—— 不需要 PIL / numpy。
#=============================================================================

import argparse
import os
import struct
import sys
import zlib

#-----------------------------------------------------------------------------
#  0. 硬件链路常量 —— 全部来自 RTL，改 RTL 就要同步改这里
#-----------------------------------------------------------------------------
#  YCbCr（detect_color_mask.v / common_defs.vh §4）
YCBCR_ROUND = 128      # 四舍五入偏置
YCBCR_SHIFT = 8        # 算术右移位数
YCBCR_BIAS = 128       # Cb/Cr 零偏置

#  sat_enhance.v 默认参数（顶层例化值）
SAT_GAIN_DEFAULT = 96   # 6 位小数定点：64=1.0x / 96=1.5x / 112=1.75x / 128=2.0x
DEADZONE_DEFAULT = 3    # 色度死区：|dC| ≤ 该值视为灰色，不增强

#  判据门限的**兜底默认值**（正常情况下会从 common_defs.vh 现场读，读到就用读到的）
FALLBACK_MAG_TH = 24
FALLBACK_RATIO10 = 13
FALLBACK_G_AXIS = 10

HERE = os.path.dirname(os.path.abspath(__file__))


#-----------------------------------------------------------------------------
#  1. 纯标准库 PNG 解码 / 编码
#-----------------------------------------------------------------------------
def _paeth(a, b, c):
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    if pb <= pc:
        return b
    return c


def read_png(path):
    """解码 PNG → (宽, 高, 通道数, 像素字节串)。

    支持：8 bit/通道、真彩色（color type 2 = RGB / 6 = RGBA）、非隔行。
    不支持：调色板 / 灰度 / 16 bit / 隔行 —— 遇到会明确报错，不会静默出错。
    """
    with open(path, "rb") as fp:
        data = fp.read()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("不是 PNG 文件（若为 JPEG，请先另存为 PNG）")

    pos = 8
    idat = bytearray()
    w = h = bitdepth = colortype = interlace = None
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos:pos + 4])
        ctype = data[pos + 4:pos + 8]
        chunk = data[pos + 8:pos + 8 + length]
        pos += 12 + length
        if ctype == b"IHDR":
            w, h, bitdepth, colortype, _comp, _filt, interlace = struct.unpack(">IIBBBBB", chunk)
        elif ctype == b"IDAT":
            idat += chunk
        elif ctype == b"IEND":
            break

    if bitdepth != 8:
        raise ValueError("仅支持 8 bit/通道，本图为 %s bit" % bitdepth)
    if interlace != 0:
        raise ValueError("不支持隔行（Adam7）PNG，请另存为普通 PNG")
    if colortype not in (2, 6):
        raise ValueError("仅支持真彩色 PNG（RGB 或 RGBA），本图 color type = %s" % colortype)

    nch = 3 if colortype == 2 else 4
    raw = zlib.decompress(bytes(idat))
    stride = w * nch
    out = bytearray(h * stride)
    prev = bytearray(stride)
    p = 0
    for y in range(h):
        ft = raw[p]
        p += 1
        line = bytearray(raw[p:p + stride])
        p += stride
        if ft == 1:                                     # Sub
            for i in range(nch, stride):
                line[i] = (line[i] + line[i - nch]) & 0xFF
        elif ft == 2:                                   # Up
            for i in range(stride):
                line[i] = (line[i] + prev[i]) & 0xFF
        elif ft == 3:                                   # Average
            for i in range(stride):
                a = line[i - nch] if i >= nch else 0
                line[i] = (line[i] + ((a + prev[i]) >> 1)) & 0xFF
        elif ft == 4:                                   # Paeth
            for i in range(stride):
                a = line[i - nch] if i >= nch else 0
                c = prev[i - nch] if i >= nch else 0
                line[i] = (line[i] + _paeth(a, prev[i], c)) & 0xFF
        elif ft != 0:
            raise ValueError("未知 PNG 滤波类型 %d" % ft)
        out[y * stride:(y + 1) * stride] = line
        prev = line
    return w, h, nch, bytes(out)


def write_png(path, w, h, rgb):
    """写 8bit RGB PNG（用于输出掩膜预览图）。rgb 为长度 w*h*3 的字节串。"""
    raw = bytearray()
    stride = w * 3
    for y in range(h):
        raw.append(0)
        raw += rgb[y * stride:(y + 1) * stride]

    def _chunk(tag, payload):
        return (struct.pack(">I", len(payload)) + tag + payload
                + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    blob = (b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", ihdr)
            + _chunk(b"IDAT", zlib.compress(bytes(raw), 6)) + _chunk(b"IEND", b""))
    with open(path, "wb") as fp:
        fp.write(blob)


def get_px(buf, nch, w, x, y):
    i = (y * w + x) * nch
    return buf[i], buf[i + 1], buf[i + 2]


#-----------------------------------------------------------------------------
#  2. 硬件链路复现
#-----------------------------------------------------------------------------
def sat_enhance(r, g, b, gain=SAT_GAIN_DEFAULT, deadzone=DEADZONE_DEFAULT):
    """复现 `sat_enhance.v`：Y 保持不动，色度偏离量 dC 放大 gain/64 倍。

    · Y   = (77R + 150G + 29B) >> 8          （Rec.601，系数和 = 256）
    · dC  = C − Y                            （有符号）
    · |dC| ≤ deadzone  → 视为灰色，增益 1.0（原样还原）
    · 否则             → C' = Y + (dC × gain) >> 6     （算术右移，负数不错）
    · 限幅到 [0,255]
    """
    y_lum = (77 * r + 150 * g + 29 * b) >> 8
    out = []
    for c in (r, g, b):
        d = c - y_lum
        if abs(d) <= deadzone:
            v = y_lum + d                      # = c，灰度分支不做增强
        else:
            v = y_lum + ((d * gain) >> 6)      # Python >> 对负数是算术右移，同 Verilog >>>
        out.append(0 if v < 0 else (255 if v > 255 else v))
    return out[0], out[1], out[2]


def rgb2ycbcr(r, g, b):
    """复现 `detect_color_mask.v` 的 YCbCr 定点式（含 +128 四舍五入与 >>8）。"""
    y = (77 * r + 150 * g + 29 * b + YCBCR_ROUND) >> YCBCR_SHIFT
    cb = ((-43 * r - 85 * g + 128 * b + YCBCR_ROUND) >> YCBCR_SHIFT) + YCBCR_BIAS
    cr = ((128 * r - 107 * g - 21 * b + YCBCR_ROUND) >> YCBCR_SHIFT) + YCBCR_BIAS
    return y, cb, cr


def chroma_of(r, g, b, tap="post", gain=SAT_GAIN_DEFAULT, deadzone=DEADZONE_DEFAULT):
    """→ (dcb, dcr)，即检测器实际看到的色差（已含取数点差异）。"""
    if tap == "scene":
        r, g, b = sat_enhance(r, g, b, gain, deadzone)
    _y, cb, cr = rgb2ycbcr(r, g, b)
    return cb - YCBCR_BIAS, cr - YCBCR_BIAS


def judge(dcb, dcr, mag_th, ratio10=FALLBACK_RATIO10, g_axis=FALLBACK_G_AXIS):
    """复现 v2 判据（`detect_color_mask.v`）：返回 0=红 1=蓝 2=绿 3=未命中。

    红：mag ≥ MAG_TH 且 dcr > 0 且 dcb ≤ 0 且 |dcr|*10 ≥ |dcb|*RATIO10
    蓝：mag ≥ MAG_TH 且 dcb > 0 且 dcr ≤ 0 且 |dcb|*10 ≥ |dcr|*RATIO10
    绿：mag ≥ MAG_TH 且 dcb < 0 且 dcr < 0 且 |dcb| ≥ G_AXIS 且 |dcr| ≥ G_AXIS
    """
    a, b = abs(dcb), abs(dcr)
    if a + b < mag_th:
        return 3
    if dcr > 0 and dcb <= 0 and b * 10 >= a * ratio10:
        return 0
    if dcb > 0 and dcr <= 0 and a * 10 >= b * ratio10:
        return 1
    if dcb < 0 and dcr < 0:
        if g_axis and (a < g_axis or b < g_axis):
            return 3
        return 2
    return 3


#-----------------------------------------------------------------------------
#  3. 从 common_defs.vh 读当前门限（保证"同源"，不硬编码）
#-----------------------------------------------------------------------------
def read_macros(vh_path):
    """读取 `` `define MAG_TH 24 `` 等。读不到就用兜底值并提示。"""
    got = {}
    try:
        with open(vh_path, "r", encoding="utf-8", errors="replace") as fp:
            txt = fp.read()
    except IOError:
        return got
    import re
    for key in ("MAG_TH", "RATIO10", "G_AXIS_TH"):
        m = re.search(r"^`define\s+%s\s+(\d+)" % key, txt, re.M)
        if m:
            got[key] = int(m.group(1))
    return got


#-----------------------------------------------------------------------------
#  4. 统计与扫描
#-----------------------------------------------------------------------------
def region_chroma(buf, nch, w, h, box, tap, gain, deadzone, step=1):
    """把某矩形区域的像素转成 [(dcb,dcr,cls)]，用于统计。"""
    x0, y0, x1, y1 = box
    x0 = max(0, min(x0, w - 1)); x1 = max(x0, min(x1, w - 1))
    y0 = max(0, min(y0, h - 1)); y1 = max(y0, min(y1, h - 1))
    out = []
    for y in range(y0, y1 + 1, step):
        for x in range(x0, x1 + 1, step):
            r, g, b = get_px(buf, nch, w, x, y)
            dcb, dcr = chroma_of(r, g, b, tap, gain, deadzone)
            out.append((dcb, dcr))
    return out


def pct(vals, q):
    """简单分位数（0~100）。"""
    if not vals:
        return 0
    s = sorted(vals)
    k = int(round((len(s) - 1) * q / 100.0))
    return s[max(0, min(k, len(s) - 1))]


def stats(rows):
    """一组 (dcb,dcr) 的概览。"""
    if not rows:
        return None
    mags = [abs(a) + abs(b) for a, b in rows]
    dcb = [a for a, _b in rows]
    dcr = [b for _a, b in rows]
    return {
        "n": len(rows),
        "mag_p05": pct(mags, 5), "mag_p50": pct(mags, 50), "mag_p95": pct(mags, 95),
        "dcb_min": min(dcb), "dcb_max": max(dcb),
        "dcr_min": min(dcr), "dcr_max": max(dcr),
    }


def hit_rate(rows, mag_th, ratio10, g_axis):
    """命中率（返回命中像素比例，不含具体颜色）。"""
    if not rows:
        return 0.0
    n = 0
    for a, b in rows:
        if judge(a, b, mag_th, ratio10, g_axis) != 3:
            n += 1
    return n / float(len(rows))


def scan(tgt_rows, bg_rows, mag_list, gax_list, ratio10):
    """扫描 MAG_TH × G_AXIS_TH，返回 [(mag,gax,tp,fp)]。"""
    out = []
    for mt in mag_list:
        for ga in gax_list:
            tp = hit_rate(tgt_rows, mt, ratio10, ga)
            fp = hit_rate(bg_rows, mt, ratio10, ga) if bg_rows else 0.0
            out.append((mt, ga, tp, fp))
    return out


def recommend(scan_rows, min_tp, max_fp):
    """在约束内挑：优先 (1) 满足 tp≥min_tp 且 fp≤max_fp；(2) 其中 tp 最高、fp 最低、门限最低。"""
    ok = [r for r in scan_rows if r[2] >= min_tp and r[3] <= max_fp]
    if ok:
        ok.sort(key=lambda r: (-r[2], r[3], r[0], r[1]))
        return ok[0], True
    # 没有同时满足的：退而求其次 —— 在 fp 约束内取 tp 最大
    near = [r for r in scan_rows if r[3] <= max_fp]
    if near:
        near.sort(key=lambda r: (-r[2], r[3], r[0], r[1]))
        return near[0], False
    # 连 fp 都压不住：取 fp 最小
    scan_rows_sorted = sorted(scan_rows, key=lambda r: (r[3], -r[2], r[0]))
    return scan_rows_sorted[0], False


def macro_block(mag_th, g_axis, ratio10):
    return (
        "// ---------- 复制到 common_base/common_defs.vh §5b ----------\n"
        "`define MAG_TH           %d    // 饱和度幅度门限 |dcb|+|dcr|\n"
        "`define RATIO10          %d    // 比值判据系数（|a|*10 >= |b|*RATIO10，红/蓝用）\n"
        "`define G_AXIS_TH        %d    // 绿判据双轴门限（|dcb|、|dcr| 各自达标）\n"
        % (mag_th, ratio10, g_axis))


#-----------------------------------------------------------------------------
#  5. 主流程
#-----------------------------------------------------------------------------
def parse_box(s):
    try:
        parts = [int(v) for v in s.replace(" ", "").split(",")]
    except ValueError:
        raise argparse.ArgumentTypeError("区域格式应为 x0,y0,x1,y1（整数）")
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("区域格式应为 x0,y0,x1,y1（4 个整数）")
    x0, y0, x1, y1 = parts
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    return x0, y0, x1, y1


def run_selftest():
    """自检：造一张合成 PNG 写盘 → 读回 → 校验解码与判据链路。"""
    print("=" * 72)
    print("  自检：合成 PNG → 解码 → 链路 → 判据")
    print("=" * 72)
    W, H = 320, 240
    px = bytearray([100, 100, 100] * (W * H))          # 背景：中性灰
    boxes = {
        "红": (40, 40, 120, 120, (190, 60, 60)),       # 拟真红球
        "绿": (140, 40, 220, 120, (80, 170, 90)),      # 拟真绿三角
        "蓝": (60, 150, 140, 220, (90, 110, 180)),     # 拟真蓝方块
    }
    for _name, (bx0, by0, bx1, by1, rgb) in boxes.items():
        for y in range(by0, by1):
            for x in range(bx0, bx1):
                i = (y * W + x) * 3
                px[i:i + 3] = bytes(rgb)

    tmp = os.path.join(HERE, "_selftest_tmp.png")
    try:
        write_png(tmp, W, H, bytes(px))
        w, h, nch, buf = read_png(tmp)
        assert (w, h, nch) == (W, H, 3), "解码尺寸/通道不符"
        # 逐点校验：三个色块的第一个像素必须原样读回
        for name, (bx0, by0, _bx1, _by1, rgb) in boxes.items():
            got = get_px(buf, nch, w, bx0, by0)
            assert tuple(got) == rgb, "%s 色块解码不符：%r vs %r" % (name, got, rgb)
        # 背景像素
        assert tuple(get_px(buf, nch, w, 300, 230)) == (100, 100, 100), "背景解码不符"
        print("  [OK] PNG 编码/解码往返一致（3 个色块 + 背景）")

        # 判据链路：三个色块应分别命中山红/蓝/绿
        expect = {"红": 0, "绿": 2, "蓝": 1}
        for name, (bx0, by0, bx1, by1, _rgb) in boxes.items():
            cx = (bx0 + bx1) // 2
            cy = (by0 + by1) // 2
            r, g, b = get_px(buf, nch, w, cx, cy)
            dcb, dcr = chroma_of(r, g, b, tap="post")
            cls = judge(dcb, dcr, FALLBACK_MAG_TH, FALLBACK_RATIO10, FALLBACK_G_AXIS)
            tag = {0: "红", 1: "蓝", 2: "绿", 3: "未命中"}[cls]
            assert cls == expect[name], "%s 色块判成了 %s" % (name, tag)
            print("  [OK] %s 色块 → 判定 %s（dcb=%d dcr=%d）" % (name, tag, dcb, dcr))

        # 背景（中性灰）应当全部不命中
        bg = region_chroma(buf, nch, w, h, (250, 180, 310, 230), "post", SAT_GAIN_DEFAULT,
                           DEADZONE_DEFAULT)
        fp = hit_rate(bg, FALLBACK_MAG_TH, FALLBACK_RATIO10, FALLBACK_G_AXIS)
        assert fp == 0.0, "中性灰背景不应命中，实测 %.3f" % fp
        print("  [OK] 中性灰背景误命中率 0.000")

        # --tap scene 分支：应比 post 更强（1.5× 色度）
        r, g, b = boxes["绿"][4]
        d_post = chroma_of(r, g, b, tap="post")
        d_scene = chroma_of(r, g, b, tap="scene")
        assert abs(d_scene[0]) >= abs(d_post[0]) and abs(d_scene[1]) >= abs(d_post[1]), \
            "scene 分支应放大色度"
        print("  [OK] --tap scene 确实补上了 1.5× 增强：post=%r → scene=%r" % (d_post, d_scene))

        print()
        print("[PASS] calib_threshold.py 自检全部通过")
        return 0
    finally:
        try:
            os.remove(tmp)
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser(
        description="方案B D7：用真实目标物标定 Cb/Cr 门限（纯标准库，无需 PIL/numpy）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("img", nargs="?", help="输入 PNG 图（JPEG 请先另存为 PNG）")
    ap.add_argument("--tgt", type=parse_box, help="目标区域 x0,y0,x1,y1（必填）")
    ap.add_argument("--bg", type=parse_box, help="背景区域 x0,y0,x1,y1（可选；不给则用目标区以外的整幅图）")
    ap.add_argument("--tap", choices=("post", "scene"), default="post",
                    help="post=图已是工程输出（不再增强，默认）；scene=手机直拍实物（补 1.5× 增强）")
    ap.add_argument("--gain", type=int, default=SAT_GAIN_DEFAULT, help="饱和度增益（6 位小数，默认 96=1.5x）")
    ap.add_argument("--deadzone", type=int, default=DEADZONE_DEFAULT, help="色度死区（默认 3）")
    ap.add_argument("--ratio10", type=int, default=None, help="比值系数；默认从 common_defs.vh 读")
    ap.add_argument("--step", type=int, default=2, help="采样步长（默认 2，加速；=1 最精确）")
    ap.add_argument("--min-tp", type=float, default=0.80, help="目标检出率下限（默认 0.80）")
    ap.add_argument("--max-fp", type=float, default=0.001, help="背景误命中率上限（默认 0.1%%）")
    ap.add_argument("--mask-out", help="把「当前门限下的命中掩膜」写成 PNG 便于肉眼核对")
    ap.add_argument("--selftest", action="store_true", help="跑自检（不需要素材）")
    args = ap.parse_args()

    if args.selftest:
        return run_selftest()

    if not args.img or not args.tgt:
        ap.print_help()
        print("\n错误：需要 --tgt（目标区域）。例：python calib_threshold.py photo.png --tgt 400,250,760,520")
        return 2
    if not os.path.isfile(args.img):
        print("错误：找不到文件 %s" % args.img)
        return 2

    # 当前门限：优先命令行，其次 common_defs.vh，最后兜底
    macros = read_macros(os.path.join(HERE, "..", "common_base", "common_defs.vh"))
    cur_mag = macros.get("MAG_TH", FALLBACK_MAG_TH)
    cur_ratio = args.ratio10 if args.ratio10 is not None else macros.get("RATIO10", FALLBACK_RATIO10)
    cur_gax = macros.get("G_AXIS_TH", FALLBACK_G_AXIS)
    src = "common_defs.vh" if macros else "兜底默认值"

    print("=" * 72)
    print("  方案B D7 · Cb/Cr 门限标定")
    print("=" * 72)
    print("  输入图      : %s" % args.img)
    print("  取数点      : --tap %s%s" % (args.tap,
          "（图已是工程输出，不再增强）" if args.tap == "post"
          else "（补上 1.5× 饱和度增强）"))
    print("  饱和度增益  : %d/64 = %.2fx, 死区 %d" % (args.gain, args.gain / 64.0, args.deadzone))
    print("  当前门限    : MAG_TH=%d, RATIO10=%d, G_AXIS_TH=%d   （读自 %s）"
          % (cur_mag, cur_ratio, cur_gax, src))

    w, h, nch, buf = read_png(args.img)
    print("  图像尺寸    : %dx%d, %d 通道" % (w, h, nch))

    tgt_rows = region_chroma(buf, nch, w, h, args.tgt, args.tap, args.gain, args.deadzone, args.step)
    if args.bg:
        bg_rows = region_chroma(buf, nch, w, h, args.bg, args.tap, args.gain, args.deadzone, args.step)
        bg_src = "指定背景区"
    else:
        # 目标区以外的整幅图（四边环带采样，避免把目标算进背景）
        x0, y0, x1, y1 = args.tgt
        bg_rows = []
        band = 40
        for y in range(0, h, args.step):
            outside = (y < y0 - band) or (y > y1 + band)
            for x in range(0, w, args.step):
                if outside or x < x0 - band or x > x1 + band:
                    r, g, b = get_px(buf, nch, w, x, y)
                    bg_rows.append(chroma_of(r, g, b, args.tap, args.gain, args.deadzone))
        bg_src = "目标区以外的整幅图（自动）"

    st = stats(tgt_rows)
    sb = stats(bg_rows)
    print("  目标区      : (%d,%d)-(%d,%d)  采样 %d 点" % (args.tgt + (st["n"],)))
    print("  背景区      : %s  采样 %d 点" % (bg_src, sb["n"] if sb else 0))
    print()
    print("  --- 目标区色度分布 ---")
    print("    幅度 |dcb|+|dcr| : P5=%d  P50=%d  P95=%d" % (st["mag_p05"], st["mag_p50"], st["mag_p95"]))
    print("    dcb 范围          : %d ~ %d" % (st["dcb_min"], st["dcb_max"]))
    print("    dcr 范围          : %d ~ %d" % (st["dcr_min"], st["dcr_max"]))
    print("    当前门限下检出率  : %.1f%%" % (hit_rate(tgt_rows, cur_mag, cur_ratio, cur_gax) * 100))
    if sb:
        print("  --- 背景区色度分布 ---")
        print("    幅度 |dcb|+|dcr| : P5=%d  P50=%d  P95=%d" % (sb["mag_p05"], sb["mag_p50"], sb["mag_p95"]))
        print("    当前门限下误命中  : %.3f%%" % (hit_rate(bg_rows, cur_mag, cur_ratio, cur_gax) * 100))

    mag_list = [8, 12, 16, 20, 24, 28, 32, 40, 48, 56, 64]
    gax_list = [0, 6, 8, 10, 12, 16, 20, 24]
    rows = scan(tgt_rows, bg_rows, mag_list, gax_list, cur_ratio)

    print()
    print("  --- 扫描 MAG_TH × G_AXIS_TH（数值 = 目标检出率 / 背景误命中率）---")
    header = "    %-8s" % "MAG_TH"
    for ga in gax_list:
        header += " GA=%-8d" % ga
    print(header)
    for mt in mag_list:
        line = "    %-8d" % mt
        for ga in gax_list:
            rec = [r for r in rows if r[0] == mt and r[1] == ga][0]
            line += " %5.1f/%-5.3f" % (rec[2] * 100, rec[3] * 100)
        print(line)

    best, feasible = recommend(rows, args.min_tp, args.max_fp)
    print()
    print("  --- 推荐 ---")
    if feasible:
        print("    在「检出率 ≥ %.0f%% 且 误命中 ≤ %.2f%%」约束下："
              % (args.min_tp * 100, args.max_fp * 100))
    else:
        print("    ⚠ 没有同时满足约束（检出率 ≥ %.0f%% / 误命中 ≤ %.2f%%）的组合，"
              % (args.min_tp * 100, args.max_fp * 100))
        print("      给出的是**最接近**的一组 —— 说明这批目标/背景的可分性不够，")
        print("      建议：① 换个光照条件重拍；② 核对 --tap 是否选对；③ 缩小 --tgt 只框目标主体。")
    print("    MAG_TH = %d , G_AXIS_TH = %d  (RATIO10 保持 %d)" % (best[0], best[1], cur_ratio))
    print("    → 目标检出率 %.1f%% ，背景误命中率 %.3f%%" % (best[2] * 100, best[3] * 100))

    # 只按 MAG_TH 的最优（绿轴=0，兼容只看幅度的情况）
    flat = [r for r in rows if r[1] == 0]
    bf, ff = recommend(flat, args.min_tp, args.max_fp)
    print("    （若不想动 G_AXIS_TH：MAG_TH = %d 即可，检出 %.1f%% / 误命中 %.3f%%）"
          % (bf[0], bf[2] * 100, bf[3] * 100))

    print()
    print(macro_block(best[0], best[1], cur_ratio))

    if args.mask_out:
        rgb = bytearray(w * h * 3)
        for y in range(h):
            for x in range(w):
                r, g, b = get_px(buf, nch, w, x, y)
                dcb, dcr = chroma_of(r, g, b, args.tap, args.gain, args.deadzone)
                cls = judge(dcb, dcr, best[0], cur_ratio, best[1])
                col = ((220, 40, 40), (40, 80, 220), (40, 200, 40), (r, g, b))[cls]
                i = (y * w + x) * 3
                rgb[i:i + 3] = bytes(col)
        write_png(args.mask_out, w, h, bytes(rgb))
        print("  掩膜预览已写出：%s" % args.mask_out)
        print("    （红/蓝/绿 = 判定命中；其余保留原色。可肉眼核对目标是否被完整覆盖）")

    print()
    print("  下一步：把上面的宏块改到 `common_base/common_defs.vh §5b`（顶层例化引宏，只改这一处），")
    print("          然后 reset_run syn_1 + phy_1 重新综合。")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    sys.exit(main())
