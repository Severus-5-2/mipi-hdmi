#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_lowlight_mask.py  ——  低曝光场景下掩膜稳定性 / 检测完整性评估

目的（对应现场现象）：
  低曝光时「用来检测物体的小绿块有时有有时无」「目标检测不全」「框只在移动时显现」。

假设的因果链：
  低曝光 → 暗部像素 Cb/Cr 随机漂移
        → Y_MIN 门控关闭(time 关) ⇒ 噪点进掩膜
        → 每行游程数暴增 > NEG=8
        → 真实目标被挤出连通域表 ⇒ 检测不全 / 时有时无

做法（纯软件模型，不走 RTL，快速扫参数）：
  1. 造一帧 1280x720 的低曝光场景：
       - 背景：低亮度灰 + 暗部彩色噪声（模拟 gain 抬升后的底噪）
       - 目标：红球 / 蓝方块 / 绿三角，亮度也压低（模拟欠曝）
  2. 按 detect_color_mask 的**完全相同**的 YCbCr 系数与阈值算掩膜
  3. 统计：
       - 掩膜覆盖率、噪点游程数 vs 真实目标游程数
       - 真实目标是否还能形成「面积 >= MIN_AREA 的连通域」
  4. 扫 Y_MIN ∈ {0,16,24,32,48,64,80}，输出推荐值

用法：
    python sim_lowlight_mask.py            # 用默认参数扫 Y_MIN
    python sim_lowlight_mask.py --detail   # 附带逐目标明细
"""

import argparse
import random

# ---------------------------------------------------------------------------
# 与 detect_color_mask.v **逐字一致** 的系数（A3~A7 冻结项，不可改）
#   Y  =  77R + 150G +  29B            -> (x + 128) >>> 8
#   Cb = -43R -  85G + 128B  + 128     -> ((x + 128) >>> 8) + 128
#   Cr = 128R - 107G -  21B  + 128     -> ((x + 128) >>> 8) + 128
# ---------------------------------------------------------------------------
def rgb2ycbcr(r, g, b):
    y  = ( 77 * r + 150 * g +  29 * b + 128) >> 8
    cb = ((-43 * r -  85 * g + 128 * b + 128) >> 8) + 128
    cr = ((128 * r - 107 * g -  21 * b + 128) >> 8) + 128
    # 限幅（A9：先限幅再比较）
    y  = max(0, min(255, y))
    cb = max(0, min(255, cb))
    cr = max(0, min(255, cr))
    return y, cb, cr


# 阈值（与 RTL 默认值一致）
RED   = dict(cb=(50, 125),  cr=(155, 255))
BLUE  = dict(cb=(155, 255), cr=(40, 125))
GREEN = dict(cb=(20, 95),   cr=(0, 85))

CLS_NAME = {0: '红', 1: '蓝', 2: '绿'}


def classify(y, cb, cr, y_min):
    """返回 (cls, hit)。cls: 0=红 1=蓝 2=绿 3=未分类（与 RTL 优先级一致：红>蓝>绿）"""
    if y < y_min:
        return 3, False
    if RED['cb'][0] <= cb <= RED['cb'][1] and RED['cr'][0] <= cr <= RED['cr'][1]:
        return 0, True
    if BLUE['cb'][0] <= cb <= BLUE['cb'][1] and BLUE['cr'][0] <= cr <= BLUE['cr'][1]:
        return 1, True
    if GREEN['cb'][0] <= cb <= GREEN['cb'][1] and GREEN['cr'][0] <= cr <= GREEN['cr'][1]:
        return 2, True
    return 3, False


# ---------------------------------------------------------------------------
# 场景构造
# ---------------------------------------------------------------------------
W, H = 1280, 720

# 场景一：低曝光（模拟现场）
#   · 光不够 -> 整体亮度压低，目标像素 Y 落到 20~70
#   · AE 会往上抬增益 -> 底噪被同步放大，暗部出现彩色噪声
SCENES = {
    '低曝光(现场)': dict(
        bg_y=18, bg_noise=14,          # 背景亮度低 + 彩色噪声幅度大
        tgt_scale=0.28,                # 目标颜色向黑压缩（欠曝）
        tgt_noise=8,
    ),
    '正常曝光(对照)': dict(
        bg_y=90, bg_noise=4,
        tgt_scale=0.85,
        tgt_noise=3,
    ),
}

# 目标：(名称, 颜色, x0, y0, w, h)
TARGETS = [
    ('红球',   24, 0xFF0000,  180, 260, 120, 110),
    ('蓝方块', 0,  0x0000FF,  560, 300, 130, 130),
    ('绿三角', 2,  0x00FF00,  940, 240, 120, 120),
]


def make_frame(scene, seed):
    """生成一帧：返回 (mask_grid, cls_grid, hit_records)
       mask_grid[y][x] = True/False
       hit_records = [(cls, x, y)] 所有命中点（含噪点）
    """
    rnd = random.Random(seed)
    bg_y      = scene['bg_y']
    bg_noise  = scene['bg_noise']
    tgt_scale = scene['tgt_scale']
    tgt_noise = scene['tgt_noise']

    # 先把目标像素标记出来（供统计用）
    label = [[-1] * W for _ in range(H)]          # -1=背景, 0..2=第几个目标
    for ti, (_, _, _, x0, y0, w, h) in enumerate(TARGETS):
        for yy in range(y0, min(y0 + h, H)):
            for xx in range(x0, min(x0 + w, W)):
                if ti == 2:   # 绿三角：做个三角形状
                    cx = x0 + w // 2
                    if abs(xx - cx) > (yy - y0) * 0.6:
                        continue
                label[yy][xx] = ti

    mask = [[False] * W for _ in range(H)]
    hits = []
    for yy in range(H):
        for xx in range(W):
            ti = label[yy][xx]
            if ti >= 0:
                # 目标像素：颜色按比例压暗 + 少量噪声
                r, g, b = (TARGETS[ti][2] >> 16) & 0xFF, (TARGETS[ti][2] >> 8) & 0xFF, TARGETS[ti][2] & 0xFF
                r = int(r * tgt_scale) + rnd.randint(-tgt_noise, tgt_noise)
                g = int(g * tgt_scale) + rnd.randint(-tgt_noise, tgt_noise)
                b = int(b * tgt_scale) + rnd.randint(-tgt_noise, tgt_noise)
            else:
                # 背景：低亮度灰 + 暗部彩色噪声
                v = bg_y + rnd.randint(-bg_noise, bg_noise)
                r = g = b = v
                # 彩色噪声：三通道独立漂移（真实 sensor 底噪的简化模型）
                r += rnd.randint(-bg_noise, bg_noise)
                g += rnd.randint(-bg_noise, bg_noise)
                b += rnd.randint(-bg_noise, bg_noise)
            r = max(0, min(255, r)); g = max(0, min(255, g)); b = max(0, min(255, b))

            y, cb, cr = rgb2ycbcr(r, g, b)
            cls, hit = classify(y, cb, cr, CUR_Y_MIN)
            mask[yy][xx] = hit
            if hit:
                hits.append((cls, xx, yy, ti))
    return mask, label, hits


# ---------------------------------------------------------------------------
# 统计：每行游程数（这是 conn_label 的关键输入）
# ---------------------------------------------------------------------------
def run_count_per_row(mask):
    counts = []
    for yy in range(H):
        n, prev = 0, False
        for xx in range(W):
            cur = mask[yy][xx]
            if cur and not prev:
                n += 1
            prev = cur
        counts.append(n)
    counts.sort()
    return counts


MIN_AREA = 16


def eval_scene(name, scene, y_min, seeds=3):
    global CUR_Y_MIN
    CUR_Y_MIN = y_min

    tot_hit = 0
    tot_bg_hit = 0
    tot_tgt_hit = 0
    tot_tgt_px = 0
    all_runs = []
    tgt_ok = {t[0]: 0 for t in TARGETS}      # 各目标在多少帧里形成了合格连通域

    for s in range(seeds):
        mask, label, hits = make_frame(scene, seed=1000 + s)
        tot_hit += len(hits)
        tot_tgt_px += sum(1 for yy in range(H) for xx in range(W) if label[yy][xx] >= 0)

        # 噪声命中（背景处命中） vs 目标命中
        # 目标命中：只要落在目标 bbox 内即可（允许少量漏检）
        tgt_hit_cnt = 0
        for (cls, xx, yy, ti) in hits:
            if ti >= 0:
                tgt_hit_cnt += 1
            else:
                tot_bg_hit += 1
        tot_tgt_hit += tgt_hit_cnt

        all_runs.extend(run_count_per_row(mask))

        # 简化连通域：用 flood fill 找面积 >= MIN_AREA 的块
        seen = [[False] * W for _ in range(H)]
        for yy in range(H):
            for xx in range(W):
                if mask[yy][xx] and not seen[yy][xx]:
                    stack = [(xx, yy)]
                    seen[yy][xx] = True
                    area = 0
                    tgt_votes = {}
                    while stack:
                        cx, cy = stack.pop()
                        area += 1
                        ti = label[cy][cx]
                        if ti >= 0:
                            tgt_votes[ti] = tgt_votes.get(ti, 0) + 1
                        for nx, ny in ((cx+1,cy),(cx-1,cy),(cx,cy+1),(cx,cy-1)):
                            if 0 <= nx < W and 0 <= ny < H and mask[ny][nx] and not seen[ny][nx]:
                                seen[ny][nx] = True
                                stack.append((nx, ny))
                    if area >= MIN_AREA and tgt_votes:
                        # 该连通域主要落在哪个目标上
                        best_ti = max(tgt_votes, key=tgt_votes.get)
                        if tgt_votes[best_ti] >= area * 0.5:
                            tgt_ok[TARGETS[best_ti][0]] += 1

    all_runs.sort()
    n = len(all_runs)
    runs_med = all_runs[n // 2]
    runs_p95 = all_runs[int(n * 0.95)]
    runs_max = all_runs[-1]

    return dict(
        y_min=y_min,
        hits=tot_hit,
        bg_hits=tot_bg_hit,
        tgt_hits=tot_tgt_hit,
        tgt_px=tot_tgt_px,
        runs_med=runs_med, runs_p95=runs_p95, runs_max=runs_max,
        tgt_ok=tgt_ok, seeds=seeds,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--detail', action='store_true')
    args = ap.parse_args()

    Y_MIN_CANDIDATES = [0, 16, 24, 32, 40, 48, 64, 80]

    for sname, scene in SCENES.items():
        print('=' * 78)
        print('场景：%s   (目标压缩=%.2f  底噪=%d)' % (sname, scene['tgt_scale'], scene['bg_noise']))
        print('=' * 78)
        print('%6s | %10s | %10s | %8s | %8s | %8s | %s' %
              ('Y_MIN', '噪点命中', '目标命中', '游程中位', '游程P95', '游程MAX', '三目标成域(帧数/总)'))
        print('-' * 78)
        for ym in Y_MIN_CANDIDATES:
            r = eval_scene(sname, scene, ym, seeds=3)
            ok_str = ' '.join('%s=%d/%d' % (k, v, r['seeds']) for k, v in r['tgt_ok'].items())
            print('%6d | %10d | %10d | %8d | %8d | %8d | %s' %
                  (ym, r['bg_hits'], r['tgt_hits'], r['runs_med'], r['runs_p95'], r['runs_max'], ok_str))
        print()
        if args.detail:
            print('（明细：每帧为 %d 行，NEG 需 >= 游程MAX 才不漏目标）' % H)
            print()

    print('=' * 78)
    print('判读要点')
    print('=' * 78)
    print('· 「噪点命中」应随 Y_MIN 升高而大幅下降 → 这就是门控该起的作用')
    print('· 「目标命中」若随 Y_MIN 升高而明显下降 → Y_MIN 定太高，会误杀暗环境真目标')
    print('· 「游程MAX」必须 < NEG(=8)，否则真实目标会被噪点挤出去 → 检测不全')
    print('· 「三目标成域」是我们真正要的指标：越接近 帧数/总 越好')


if __name__ == '__main__':
    CUR_Y_MIN = 0
    main()
