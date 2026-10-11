//=============================================================================
//  conn_label.v  —  连通域标注（4 连通 RLE-CCL）+ 每像素 label 流 + 帧末汇总
//=============================================================================
//  项目  : MIPI-HDMI 实时视频图像处理系统（方案B §2.2）
//  归属  : user_source/hdl_source/common_base/   （艺 产出，晨 复用做运动检测）
//  依据  : doc/接口冻结表.md v1.4 §3
//          doc/开发方案B-目标检测跟踪与运动检测.md §2.2
//
//  算法（方案B §2.2 指定）：
//      ① 行内游程（RLE）：一行内连续的 "1" 合并成一个游程 [start, end]
//      ② 行间并查表    ：本行游程与上一行游程水平重叠 → 继承 label；
//                        与多个已有 label 重叠 → 记录等价对
//      ③ 等价表        ：深度 32（纯寄存器，不用 BRAM），帧末统一解析
//      ④ 帧末统计      ：每 label 的 area(Σ1) / Σx / Σy / xmin,xmax / ymin,ymax
//      ⑤ 消隐期质心除法：cx=Σx/area，移位-减法除法器，不在像素流里做
//      ⑥ 颜色分类      ：每 label 锁存「首个被指派时的已知色」，帧末随
//                        O_cmp_cls 输出（`CLS_*`；未知 2'b11）。**不参与连通判定**，
//                        I_color 接 2'b11 即退化为「未分类」（运动检测用）。
//
//  连通性：**4 连通**（判据 start<=prev_end && end>=prev_start）
//
//  ── 每像素 label 流的语义（如实说明，不藏）──────────────────────────────
//  流中输出的是「等价合并后、**当前已知**的最小编号」，即 provisional。
//  原因：连通域的最终编号要等整块区域扫完（可跨多行）才能定，而像素流无法等待未来行。
//  ⇒ 对**凸形目标**（红球/蓝方块/绿三角，单行单游程）两者完全一致；
//    只有环形 / V 形这类「同一行分裂成多游程、之后再合并」的形状，
//    流里会短暂出现两个编号。**完全确定的结果走 O_cmp_* 帧末汇总。**
//
//  ── 时序：label 流比输入**晚一行** ──────────────────────────────────────
//  行 y 的游程在行 y 期间检出并标注，在行 y+1 期间回放输出。
//  O_label_pix_x / O_label_pix_y 输出**该 label 真实所属的坐标**，故
//  消费者拿到的 (label, x, y) 三者自洽，只是时间上晚一行。
//  ⇒ 叠伪彩请把视频也延一行（line_delay_ram.v），或直接用 O_cmp_* 包围盒填色。
//  另：最后一行（y=719）的 label 在帧末清零前来不及回放，**不输出**（少 1 行）。
//=============================================================================

`timescale 1ns / 1ps
`include "common_defs.vh"

module conn_label #(
    parameter integer IMG_WIDTH  = `IMG_WIDTH,    // 1280
    parameter integer IMG_HEIGHT = `IMG_HEIGHT,   // 720
    parameter integer RUN_MAX    = `LABEL_MAX,    // 32（每行最多游程数）
    parameter integer LABEL_MAX  = `LABEL_MAX,    // 32（等价表深度）
    parameter integer MIN_AREA   = `MIN_AREA       // 最小面积门限（低于视为噪声；值同源于 common_defs.vh）
)(
    input  wire        clk,
    input  wire        rst_n,

    //---- 输入：接 detect_color_mask，六者同拍对齐 ----
    input  wire        I_de,
    input  wire        I_mask,
    // 该像素的颜色类别（与 I_mask 同拍；仅供分类，不参与连通判定）
    //   接法（高→低优先级 红 > 蓝 > 绿，见 doc 说明-conn_label.md §7）：
    //     I_color = O_mask_r ? `CLS_RED  :
    //               O_mask_b ? `CLS_BLUE :
    //               O_mask_g ? `CLS_GREEN : `CLS_UNKNOWN;
    //   运动检测不需要颜色 → 直接接 2'b11（CLS_UNKNOWN），行为退化为「未分类」。
    input  wire [1:0]  I_color,
    input  wire [10:0] I_pix_x,
    input  wire [9:0]  I_pix_y,
    input  wire        I_frame_done,      // 帧界脉冲（= 对齐后的 user）

    //---- 输出：每像素 label 流（比输入晚一行）----
    output reg  [5:0]  O_label,
    output reg         O_label_valid,
    output reg         O_label_de,
    output reg  [10:0] O_label_pix_x,
    output reg  [9:0]  O_label_pix_y,

    //---- 输出：帧末汇总（消隐期脉冲流，每连通域一拍，紧凑编号 1..O_cmp_cnt）----
    output reg         O_cmp_valid,
    output reg  [5:0]  O_cmp_label,
    output reg  [1:0]  O_cmp_cls,        // 颜色分类（CLS_*；未知 = 2'b11）
    output reg  [19:0] O_cmp_area,
    output reg  [10:0] O_cmp_xmin,
    output reg  [10:0] O_cmp_xmax,
    output reg  [10:0] O_cmp_cx,
    output reg  [9:0]  O_cmp_ymin,
    output reg  [9:0]  O_cmp_ymax,
    output reg  [9:0]  O_cmp_cy,
    output reg  [5:0]  O_cmp_cnt
);

    //=========================================================================
    //  0. 常量
    //=========================================================================
    localparam integer LW  = 6;
    localparam integer NEG = 32;                  // 展开深度（固定 32）

    localparam [LW-1:0]  LBMAX = LABEL_MAX[LW-1:0];
    localparam [19:0]    MINA  = MIN_AREA[19:0];
    localparam [10:0]    XINIT = 11'd2047;        // xmin 初始
    localparam [9:0]     YINIT = 10'd1023;        // ymin 初始
    localparam [9:0]     YLAST = IMG_HEIGHT[9:0] - 10'd1;   // 最后一行号

    // ── 帧末触发 ────────────────────────────────────────────────────────
    //  ⚠ 不能用 I_frame_done（VTC 的 user 落在**帧首有效像素**，不是消隐期）——
    //    若在帧首触发，汇总 FSM 会与下一帧的有效行抢时间、把累加器清掉。
    //  改为内部检测「最后一行结束」，汇总 FSM 天然落在**场消隐期**：
    //    720p60 场消隐约 49500 拍，而 FSM 最坏约 2200 拍（32 个连通域）。
    wire frame_end = I_de_d && !I_de && (I_pix_y == YLAST);

    //=========================================================================
    //  1. 等价表（并查表，纯寄存器） + 3 级展开解析
    //=========================================================================
    reg [LW-1:0] eq [0:NEG];

    // 像素流用：3 级展开（便宜；链长>3 时给的是 provisional，已文档说明）
    function [LW-1:0] resolve3;
        input [LW-1:0] x;
        reg   [LW-1:0] t;
        begin
            t = eq[x];        // 1 级
            t = eq[t];        // 2 级
            t = eq[t];        // 3 级
            resolve3 = t;
        end
    endfunction

    // 帧末用：完全解析（最多 32 次迭代，保证拿到真正的根）
    function [LW-1:0] resolve_full;
        input [LW-1:0] x;
        reg   [LW-1:0] t;
        integer       k;
        begin
            t = x;
            for (k = 0; k < NEG; k = k + 1) t = eq[t];
            resolve_full = t;
        end
    endfunction

    //=========================================================================
    //  2. 行内游程检测（RLE）
    //=========================================================================
    reg           in_run;
    reg [10:0]    run_x0;
    reg [19:0]    run_len;
    reg [20:0]    run_sumx;                       // 单游程 Σx（1279*1280 < 2^21）
    reg [1:0]     run_cls;                        // 本游程颜色（首个「已知色」像素）

    reg           ev_valid;                       // 游程关闭事件（单拍）
    reg [10:0]    ev_x0, ev_x1;
    reg [19:0]    ev_len;
    reg [20:0]    ev_sumx;
    reg [9:0]     ev_y;
    reg [1:0]     ev_cls;                         // 关闭游程的颜色（随事件携带）

    reg           I_de_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_run   <= 1'b0;
            run_x0   <= 11'd0;
            run_len  <= 20'd0;
            run_sumx <= 21'd0;
            run_cls  <= 2'b11;
            ev_valid <= 1'b0;
            ev_x0    <= 11'd0;
            ev_x1    <= 11'd0;
            ev_len   <= 20'd0;
            ev_sumx  <= 21'd0;
            ev_y     <= 10'd0;
            ev_cls   <= 2'b11;
            I_de_d   <= 1'b0;
        end else begin
            ev_valid <= 1'b0;                     // 默认单拍脉冲
            I_de_d   <= I_de;

            if (I_de) begin
                if (I_mask) begin
                    if (!in_run) begin
                        in_run   <= 1'b1;
                        run_x0   <= I_pix_x;
                        run_len  <= 20'd1;
                        run_sumx <= {10'd0, I_pix_x};
                        run_cls  <= I_color;               // 游程首像素颜色
                    end else begin
                        run_len  <= run_len  + 20'd1;
                        run_sumx <= run_sumx + {10'd0, I_pix_x};
                        // 「首个已知色」优先：首像素若为 UNKNOWN，则由本游程内
                        // 后续第一个已知色像素补齐（游程内按 红>蓝>绿 由上游定）。
                        if (run_cls == 2'b11) run_cls <= I_color;
                    end
                end else begin
                    if (in_run) begin             // 游程在 x-1 处结束
                        in_run   <= 1'b0;
                        ev_valid <= 1'b1;
                        ev_x0    <= run_x0;
                        ev_x1    <= I_pix_x - 11'd1;
                        ev_len   <= run_len;
                        ev_sumx  <= run_sumx;
                        ev_y     <= I_pix_y;
                        ev_cls   <= run_cls;
                    end
                end
            end else begin
                if (in_run) begin                 // 行末仍在游程中 → 收尾
                    in_run   <= 1'b0;
                    ev_valid <= 1'b1;
                    ev_x0    <= run_x0;
                    ev_x1    <= I_pix_x;
                    ev_len   <= run_len;
                    ev_sumx  <= run_sumx;
                    ev_y     <= I_pix_y;
                    ev_cls   <= run_cls;
                end
            end
        end
    end

    //=========================================================================
    //  3. 游程缓冲（双 bank 乒乓）
    //     bank0 = [0..31]，bank1 = [32..63]
    //     wr_bank = 正在写（当前行）；rd_bank = 正在回放（上一行）
    //=========================================================================
    reg [10:0]   rbuf_s [0:2*NEG-1];
    reg [10:0]   rbuf_e [0:2*NEG-1];
    reg [LW-1:0] rbuf_l [0:2*NEG-1];
    reg [LW-1:0] cnt_wr, cnt_rd;
    reg          wr_bank;

    wire [6:0] wr_base = wr_bank ? 7'd32 : 7'd0;
    wire [6:0] rd_base = wr_bank ? 7'd0  : 7'd32;

    //=========================================================================
    //  4. 行间重叠比对 + label 指派 + 等价记录
    //=========================================================================
    wire [LW-1:0] prv_lbl [0:NEG-1];
    wire          ovl_hit [0:NEG-1];

    genvar gj;
    generate
        for (gj = 0; gj < NEG; gj = gj + 1) begin : g_ovl
            wire j_valid = (gj < cnt_rd);
            wire hit_raw = j_valid
                         && (ev_x0 <= rbuf_e[rd_base + gj])
                         && (ev_x1 >= rbuf_s[rd_base + gj]);   // 4 连通水平重叠
            assign ovl_hit[gj] = hit_raw && ev_valid;
            assign prv_lbl[gj] = hit_raw ? resolve3(rbuf_l[rd_base + gj]) : {LW{1'b0}};
        end
    endgenerate

    function [LW-1:0] min2;
        input [LW-1:0] a;
        input [LW-1:0] b;
        begin
            if (a == {LW{1'b0}})      min2 = b;
            else if (b == {LW{1'b0}}) min2 = a;
            else                      min2 = (a < b) ? a : b;
        end
    endfunction

    wire [LW-1:0] t1 [0:15];
    wire [LW-1:0] t2 [0:7];
    wire [LW-1:0] t3 [0:3];
    wire [LW-1:0] t4 [0:1];
    wire [LW-1:0] min_lbl;

    generate
        for (gj = 0; gj < 16; gj = gj + 1)
            assign t1[gj] = min2(ovl_hit[2*gj]   ? prv_lbl[2*gj]   : {LW{1'b0}},
                                 ovl_hit[2*gj+1] ? prv_lbl[2*gj+1] : {LW{1'b0}});
        for (gj = 0; gj < 8; gj = gj + 1)
            assign t2[gj] = min2(t1[2*gj], t1[2*gj+1]);
        for (gj = 0; gj < 4; gj = gj + 1)
            assign t3[gj] = min2(t2[2*gj], t2[2*gj+1]);
        for (gj = 0; gj < 2; gj = gj + 1)
            assign t4[gj] = min2(t3[2*gj], t3[2*gj+1]);
        assign min_lbl = min2(t4[0], t4[1]);
    endgenerate

    //---- 新 label 分配（7 bit 便于判越界）----
    reg [6:0]     lbl_next;
    wire          need_new = (min_lbl == {LW{1'b0}});
    wire          lbl_ok   = !(need_new && (lbl_next > {1'b0, LBMAX}));
    wire [LW-1:0] fresh    = lbl_next[LW-1:0];
    wire [LW-1:0] lbl_use  = need_new ? fresh : min_lbl;
    wire          ev_ok    = ev_valid && lbl_ok;

    //---- 等价合并：与 min_lbl 不同且重叠的 label → 并入 min_lbl ----
    integer mi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (mi = 0; mi <= NEG; mi = mi + 1) eq[mi] <= mi[LW-1:0];
        end else if (ev_valid && lbl_ok && !need_new) begin
            for (mi = 0; mi < NEG; mi = mi + 1)
                if (ovl_hit[mi] && (prv_lbl[mi] != min_lbl))
                    eq[prv_lbl[mi]] <= min_lbl;
        end else if (I_frame_done) begin
            for (mi = 0; mi <= NEG; mi = mi + 1) eq[mi] <= mi[LW-1:0];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)            lbl_next <= 7'd1;
        else if (I_frame_done) lbl_next <= 7'd1;
        else if (ev_ok && need_new && (lbl_next <= {1'b0, LBMAX}))
            lbl_next <= lbl_next + 7'd1;
    end

    //=========================================================================
    //  4.1 每 label 颜色锁存（仅供 cls 分类，**不参与连通判定**）
    //      规则：取该连通域「首次被指派时的已知色」（游程内首个非 UNKNOWN 像素）；
    //            等价合并时把颜色沿并查链往根传播，保证链式合并（eq[5]=3、eq[3]=1）
    //            也能把颜色带到最终根；
    //            全程未知 ⇒ 2'b11（`CLS_UNKNOWN`）。
    //      ⚠ 前提：同一连通域内颜色一致（红球/蓝方块/绿三角各占一色）。
    //        若一块区域内混色，则取首个被指派游程的色，属已知的简化。
    //=========================================================================
    reg [1:0] cls_lbl [0:NEG];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (mi = 0; mi <= NEG; mi = mi + 1) cls_lbl[mi] <= 2'b11;
        end else if (I_frame_done) begin
            for (mi = 0; mi <= NEG; mi = mi + 1) cls_lbl[mi] <= 2'b11;
        end else if (ev_valid && lbl_ok) begin
            if (need_new) begin
                cls_lbl[fresh] <= ev_cls;                        // 新 label
            end else begin
                // ① 根未定色 → 用本游程色
                if ((cls_lbl[min_lbl] == 2'b11) && (ev_cls != 2'b11))
                    cls_lbl[min_lbl] <= ev_cls;
                // ② 被并入者未定色 → 继承根色（保证链式传递）
                for (mi = 0; mi < NEG; mi = mi + 1)
                    if (ovl_hit[mi] && (prv_lbl[mi] != min_lbl)
                        && (cls_lbl[prv_lbl[mi]] == 2'b11))
                        cls_lbl[prv_lbl[mi]] <= cls_lbl[min_lbl];
                // ③ 根与本游程都未知 → 从任一被并入者取色
                for (mi = 0; mi < NEG; mi = mi + 1)
                    if (ovl_hit[mi] && (prv_lbl[mi] != min_lbl)
                        && (cls_lbl[min_lbl] == 2'b11) && (ev_cls == 2'b11)
                        && (cls_lbl[prv_lbl[mi]] != 2'b11))
                        cls_lbl[min_lbl] <= cls_lbl[prv_lbl[mi]];
            end
        end
    end

    //=========================================================================
    //  5. 每 label 统计累加器
    //=========================================================================
    reg [19:0] acc_area [0:NEG];
    reg [30:0] acc_sumx [0:NEG];                  // 921600*1279 < 2^31
    reg [29:0] acc_sumy [0:NEG];                  // 921600*719  < 2^30
    reg [10:0] acc_xmin [0:NEG];
    reg [10:0] acc_xmax [0:NEG];
    reg [9:0]  acc_ymin [0:NEG];
    reg [9:0]  acc_ymax [0:NEG];

    reg        acc_clr;                           // 帧末清零脉冲

    integer ai;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (ai = 0; ai <= NEG; ai = ai + 1) begin
                acc_area[ai] <= 20'd0;
                acc_sumx[ai] <= 31'd0;
                acc_sumy[ai] <= 30'd0;
                acc_xmin[ai] <= XINIT;
                acc_xmax[ai] <= 11'd0;
                acc_ymin[ai] <= YINIT;
                acc_ymax[ai] <= 10'd0;
            end
        end else if (acc_clr) begin
            for (ai = 0; ai <= NEG; ai = ai + 1) begin
                acc_area[ai] <= 20'd0;
                acc_sumx[ai] <= 31'd0;
                acc_sumy[ai] <= 30'd0;
                acc_xmin[ai] <= XINIT;
                acc_xmax[ai] <= 11'd0;
                acc_ymin[ai] <= YINIT;
                acc_ymax[ai] <= 10'd0;
            end
        end else if (ev_ok) begin
            acc_area[lbl_use] <= acc_area[lbl_use] + ev_len;
            acc_sumx[lbl_use] <= acc_sumx[lbl_use] + {10'd0, ev_sumx};
            acc_sumy[lbl_use] <= acc_sumy[lbl_use] + ({10'd0, ev_y} * {12'd0, ev_len});
            if (ev_x0 < acc_xmin[lbl_use]) acc_xmin[lbl_use] <= ev_x0;
            if (ev_x1 > acc_xmax[lbl_use]) acc_xmax[lbl_use] <= ev_x1;
            if (ev_y  < acc_ymin[lbl_use]) acc_ymin[lbl_use] <= ev_y;
            if (ev_y  > acc_ymax[lbl_use]) acc_ymax[lbl_use] <= ev_y;
        end
    end

    //=========================================================================
    //  6. 游程写入 + bank 交换 + 帧首复位
    //=========================================================================
    reg [3:0] swap_wait;
    wire      de_fall = I_de_d && !I_de;
    wire      do_swap = (swap_wait == 4'd1) && !ev_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_wr    <= 6'd0;
            cnt_rd    <= 6'd0;
            wr_bank   <= 1'b0;
            swap_wait <= 4'd0;
        end else if (I_frame_done) begin
            cnt_wr    <= 6'd0;
            cnt_rd    <= 6'd0;
            wr_bank   <= 1'b0;
            swap_wait <= 4'd0;
        end else begin
            if (de_fall) swap_wait <= 4'd2;   // ★ 必须在 hblank 内完成交换，
                                              //   否则下一行 x=0 那一拍回放会用旧 cnt_rd
            else if (swap_wait != 4'd0) swap_wait <= swap_wait - 4'd1;

            if (ev_ok) begin
                if (cnt_wr < LBMAX) begin
                    rbuf_s[wr_base + cnt_wr] <= ev_x0;
                    rbuf_e[wr_base + cnt_wr] <= ev_x1;
                    rbuf_l[wr_base + cnt_wr] <= need_new ? fresh : min_lbl;
                    cnt_wr <= cnt_wr + 6'd1;
                end
                // cnt_wr 满：丢游程（只保留统计），已如实记录为降级行为
            end

            if (do_swap) begin
                cnt_rd  <= cnt_wr;
                cnt_wr  <= 6'd0;
                wr_bank <= ~wr_bank;
            end
        end
    end

    //=========================================================================
    //  7. 回放：输出每像素 label 流（晚一行）
    //=========================================================================
    reg [LW-1:0] rp_ptr;
    reg [9:0]    rp_y;

    wire rp_hit = (rp_ptr < cnt_rd)
               && (I_pix_x >= rbuf_s[rd_base + rp_ptr])
               && (I_pix_x <= rbuf_e[rd_base + rp_ptr]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rp_ptr <= {LW{1'b0}};
            rp_y   <= 10'd0;
        end else if (I_de) begin
            if ((rp_ptr < cnt_rd) && (I_pix_x > rbuf_e[rd_base + rp_ptr]))
                rp_ptr <= rp_ptr + 6'd1;
        end else begin
            rp_ptr <= {LW{1'b0}};
            rp_y   <= I_pix_y;                    // 行末锁存本行 y（de 无效时保持）
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            O_label       <= 6'd0;
            O_label_valid <= 1'b0;
            O_label_de    <= 1'b0;
            O_label_pix_x <= 11'd0;
            O_label_pix_y <= 10'd0;
        end else begin
            // 存放时 label 已解析到当时的最小值（根），此处直接输出，
            // 不在回放路径上再查 eq —— 保证结果与「按游程顺序离线建模」逐位一致。
            O_label       <= rp_hit ? rbuf_l[rd_base + rp_ptr] : 6'd0;
            O_label_valid <= rp_hit;
            O_label_de    <= I_de;
            O_label_pix_x <= I_pix_x;
            O_label_pix_y <= rp_y;
        end
    end

    //=========================================================================
    //  8. 帧末 FSM（消隐期运行，约 2200 拍 ≪ 49500 拍）
    //     完全解析 → 合并 → 紧凑编号 → 质心除法 → 输出 → 清零
    //=========================================================================
    localparam [2:0] S_IDLE = 3'd0,
                     S_ADDM = 3'd1,
                     S_SCAN = 3'd2,
                     S_DIV  = 3'd3,
                     S_OUT  = 3'd4,
                     S_CLR  = 3'd5;

    reg [2:0]    st;
    reg [LW-1:0] root  [0:NEG];
    reg [LW-1:0] cmap  [0:NEG];        // 根 → 紧凑编号（0 = 不输出）
    reg [5:0]    rofc  [0:NEG];        // 紧凑编号 → 根

    reg [19:0] tp_area [0:NEG];
    reg [30:0] tp_sumx [0:NEG];
    reg [29:0] tp_sumy [0:NEG];
    reg [10:0] tp_xmin [0:NEG];
    reg [10:0] tp_xmax [0:NEG];
    reg [9:0]  tp_ymin [0:NEG];
    reg [9:0]  tp_ymax [0:NEG];
    reg [10:0] cx_out  [0:NEG];
    reg [9:0]  cy_out  [0:NEG];

    reg [6:0]  fs;                     // 通用 0..32 计数器
    reg [5:0]  sc;                     // 紧凑计数
    reg [6:0]  cid;                    // 当前处理的 label（1..32）
    reg [2:0]  dsub;                   // 除法子状态 0..4
    reg [5:0]  dbit;                   // 除法位计数 30..0

    // 除法器
    reg [30:0] dv_val;
    reg [30:0] dv_rem;
    reg [19:0] dv_area;
    reg [10:0] dv_q;
    wire [31:0] dv_sh = {dv_rem, dv_val[dbit]};
    wire        dv_ge = (dv_sh >= {12'd0, dv_area});
    wire [31:0] dv_nx = dv_ge ? (dv_sh - {12'd0, dv_area}) : dv_sh;

    integer ii;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= S_IDLE;
            acc_clr     <= 1'b0;
            O_cmp_valid <= 1'b0;
            O_cmp_label <= 6'd0;
            O_cmp_cls   <= 2'b11;
            O_cmp_area  <= 20'd0;
            O_cmp_xmin  <= 11'd0;
            O_cmp_xmax  <= 11'd0;
            O_cmp_cx    <= 11'd0;
            O_cmp_ymin  <= 10'd0;
            O_cmp_ymax  <= 10'd0;
            O_cmp_cy    <= 10'd0;
            O_cmp_cnt   <= 6'd0;
            fs          <= 7'd0;
            sc          <= 6'd0;
            cid         <= 7'd1;
            dsub        <= 3'd0;
            dbit        <= 6'd30;
            dv_val      <= 31'd0;
            dv_rem      <= 31'd0;
            dv_area     <= 20'd0;
            dv_q        <= 11'd0;
            for (ii = 0; ii <= NEG; ii = ii + 1) begin
                root[ii]  <= ii[LW-1:0];
                cmap[ii]  <= 6'd0;
                rofc[ii]  <= 6'd0;
                tp_area[ii] <= 20'd0;
                tp_sumx[ii] <= 31'd0;
                tp_sumy[ii] <= 30'd0;
                tp_xmin[ii] <= XINIT;
                tp_xmax[ii] <= 11'd0;
                tp_ymin[ii] <= YINIT;
                tp_ymax[ii] <= 10'd0;
                cx_out[ii]  <= 11'd0;
                cy_out[ii]  <= 10'd0;
            end
        end else begin
            O_cmp_valid <= 1'b0;
            acc_clr     <= 1'b0;

            case (st)
            //-------------------------------------------------------------
            S_IDLE: begin
                if (frame_end) begin
                    for (ii = 0; ii <= NEG; ii = ii + 1) begin
                        root[ii] <= resolve_full(ii[LW-1:0]);   // ★ 完全解析
                        cmap[ii] <= 6'd0;
                        rofc[ii] <= 6'd0;
                        tp_area[ii] <= 20'd0;
                        tp_sumx[ii] <= 31'd0;
                        tp_sumy[ii] <= 30'd0;
                        tp_xmin[ii] <= XINIT;
                        tp_xmax[ii] <= 11'd0;
                        tp_ymin[ii] <= YINIT;
                        tp_ymax[ii] <= 10'd0;
                    end
                    fs <= 7'd0;
                    st <= S_ADDM;
                end
            end
            //-------------------------------------------------------------
            // 把 acc[i] 归入 tp[root[i]]
            S_ADDM: begin
                if (fs <= 7'd32) begin
                    if (acc_area[fs[5:0]] != 20'd0) begin
                        tp_area[root[fs[5:0]]] <=
                            tp_area[root[fs[5:0]]] + acc_area[fs[5:0]];
                        tp_sumx[root[fs[5:0]]] <=
                            tp_sumx[root[fs[5:0]]] + acc_sumx[fs[5:0]];
                        tp_sumy[root[fs[5:0]]] <=
                            tp_sumy[root[fs[5:0]]] + acc_sumy[fs[5:0]];
                        if (acc_xmin[fs[5:0]] < tp_xmin[root[fs[5:0]]])
                            tp_xmin[root[fs[5:0]]] <= acc_xmin[fs[5:0]];
                        if (acc_xmax[fs[5:0]] > tp_xmax[root[fs[5:0]]])
                            tp_xmax[root[fs[5:0]]] <= acc_xmax[fs[5:0]];
                        if (acc_ymin[fs[5:0]] < tp_ymin[root[fs[5:0]]])
                            tp_ymin[root[fs[5:0]]] <= acc_ymin[fs[5:0]];
                        if (acc_ymax[fs[5:0]] > tp_ymax[root[fs[5:0]]])
                            tp_ymax[root[fs[5:0]]] <= acc_ymax[fs[5:0]];
                    end
                    fs <= fs + 7'd1;
                end else begin
                    fs <= 7'd0;
                    sc <= 6'd0;
                    st <= S_SCAN;
                end
            end
            //-------------------------------------------------------------
            // 紧凑编号：根且 area>=MIN_AREA → 1,2,3,...
            S_SCAN: begin
                if (fs <= 7'd32) begin
                    if ((root[fs[5:0]] == fs[5:0]) && (tp_area[fs[5:0]] >= MINA)) begin
                        cmap[fs[5:0]]      <= sc + 6'd1;
                        rofc[sc + 6'd1]    <= fs[5:0];
                        sc                 <= sc + 6'd1;
                    end
                    fs <= fs + 7'd1;
                end else begin
                    O_cmp_cnt <= sc;
                    cid <= 7'd1;
                    dsub <= 2'd0;
                    st  <= S_DIV;
                end
            end
            //-------------------------------------------------------------
            // 质心除法：对每个紧凑组件算 cx、cy
            S_DIV: begin
                if (cid > 7'd32) begin
                    fs <= 7'd1;
                    st <= S_OUT;
                end else begin
                    // 跳过非根 / 面积不足
                    if (dsub == 3'd0) begin
                        if ((root[cid[5:0]] != cid[5:0]) || (tp_area[cid[5:0]] < MINA)) begin
                            cid <= cid + 7'd1;
                        end else begin
                            dv_val  <= tp_sumx[cid[5:0]];
                            dv_area <= tp_area[cid[5:0]];
                            dv_rem  <= 31'd0;
                            dv_q    <= 11'd0;
                            dbit    <= 6'd30;
                            dsub    <= 3'd1;
                        end
                    end else if (dsub == 3'd1) begin          // cx 31 步
                        dv_rem <= dv_nx[30:0];
                        dv_q   <= {dv_q[9:0], dv_ge};
                        if (dbit == 6'd0) begin
                            dsub <= 3'd2;
                        end else begin
                            dbit <= dbit - 6'd1;
                        end
                    end else if (dsub == 3'd2) begin          // 启动 cy
                        cx_out[cid[5:0]] <= dv_q;
                        dv_val  <= {1'b0, tp_sumy[cid[5:0]]};
                        dv_area <= tp_area[cid[5:0]];
                        dv_rem  <= 31'd0;
                        dv_q    <= 11'd0;
                        dbit    <= 6'd30;
                        dsub    <= 3'd3;
                    end else if (dsub == 3'd3) begin          // cy 31 步
                        dv_rem <= dv_nx[30:0];
                        dv_q   <= {dv_q[9:0], dv_ge};
                        if (dbit == 6'd0) begin
                            dsub <= 3'd4;                     // ★ 落到存结果状态
                        end else begin
                            dbit <= dbit - 6'd1;
                        end
                    end else begin                            // dsub == 4：存 cy，下一个组件
                        cy_out[cid[5:0]] <= dv_q;
                        cid  <= cid + 7'd1;
                        dsub <= 3'd0;
                    end
                end
            end
            //-------------------------------------------------------------
            // 输出脉冲流
            S_OUT: begin
                if (fs <= 6'd32) begin
                    if ((fs >= 7'd1) && (fs <= {1'b0, O_cmp_cnt}) &&
                        (rofc[fs] != 6'd0)) begin
                        O_cmp_valid <= 1'b1;
                        O_cmp_label <= fs[5:0];
                        O_cmp_cls   <= cls_lbl[rofc[fs]];
                        O_cmp_area  <= tp_area[rofc[fs]];
                        O_cmp_xmin  <= tp_xmin[rofc[fs]];
                        O_cmp_xmax  <= tp_xmax[rofc[fs]];
                        O_cmp_cx    <= cx_out[rofc[fs]];
                        O_cmp_ymin  <= tp_ymin[rofc[fs]];
                        O_cmp_ymax  <= tp_ymax[rofc[fs]];
                        O_cmp_cy    <= cy_out[rofc[fs]];
                    end
                    fs <= fs + 7'd1;
                end else begin
                    st <= S_CLR;
                end
            end
            //-------------------------------------------------------------
            S_CLR: begin
                acc_clr <= 1'b1;               // 下一拍清累加器
                st      <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
