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
//      ③ 等价表        ：内部展开深度 NEG（纯寄存器，不用 BRAM），帧末分时解析
//                        ★ v1.5.1：NEG = 8（原 32），纯内部参数，不影响对外接口
//                        ★ v7.9  ：NEG = 16（现场"框只能套上方"驱动的扩容 —— 见参数区注释）
//      ④ 帧末统计      ：每 label 的 area(Σ1) / Σx / Σy / xmin,xmax / ymin,ymax
//      ⑤ 消隐期质心除法：cx=Σx/area，移位-减法除法器，不在像素流里做
//      ⑥ 颜色分类      ：每 label 锁存「首个被指派时的已知色」，帧末随
//                        O_cmp_cls 输出（`CLS_*`；未知 2'b11）。默认不参与连通判定。
//                        COLOR_SEPARATE=1 时，行内与行间都要求颜色一致。
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
    parameter integer MIN_AREA   = `MIN_AREA,      // 最小面积门限（低于视为噪声；值同源于 common_defs.vh）
    // ★ v7.9 新增：**单行游程最小长度**。短于此值的游程被**整体忽略**
    //   （不建 label / 不合并 / 不统计 / 不写 rbuf）⇒ 噪点不占 NEG 名额。
    //   ⚠ 这是**可被 TB 覆盖**的 parameter：`tb_conn_label.v` 用小图样，
    //     例化时传 `.LMIN(1)` 保持完整测试覆盖；工程顶层**不传**，用默认值 8。
    parameter integer LMIN       = 8,              // 单行游程最小长度（4~16 可调）
    parameter integer CONTINUE_SHORT = 0,          // retain narrow tails of an existing component
    parameter integer COLOR_SEPARATE = 0,          // split runs/components at class boundaries
    parameter integer RESERVE_COLORS = 0,          // keep one unused label for each unseen RGB class
    parameter integer RECLAIM_SMALL = 0,
    parameter integer RECLAIM_MIN_AREA = MIN_AREA,
    parameter integer OUT_SPACING = 0,
    parameter integer LINK_GAP = 0, // Same-class adjacent-row gap, pixels.
    parameter integer CCL_CAPACITY = 8 // Default retains legacy footprint; top selects measured budget.
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
    output reg  [5:0]  O_cmp_cnt,
    output reg         O_capacity_full             // previous frame dropped a run/component for capacity
);

    //=========================================================================
    //  0. 常量
    //=========================================================================
    localparam integer LW  = 6;
    // ★ 面积优化（v1.5.1）：展开深度 32 → 8。
    //   本参数是**纯内部**的「单帧最多同时跟踪的连通域数」，**不影响对外接口**
    //   （`O_label`/`O_cmp_label`/`O_cmp_cnt` 仍为 6 bit，`LABEL_MAX` 宏仍是 32）。
    //   缘由：TD 综合显示 `conn_label` 占 77,949 LUT（超器件 21,216 的 3.7 倍），
    //         主因是 33 项数组的动态索引 MUX；降深度是全局线性缩减，最有效。
    //
    //   ⚠ 行为边界（必须知道）：同一帧内若**同时存在** > NEG 个互不连通的区域，
    //     只有**最先出现的 NEG 个**会进入 O_cmp_*，其余被丢弃（统计也不入桶）。
    //     这与原 LBMAX=32 的降级路径语义完全一致，只是门限从 32 收到 8。
    //
    //   ★★ v7.9（2026-10-13）：8 → 16 → **回到 8**（扩容方案已放弃，改用「游程过滤」）
    //     现象：掩膜清理后仍残留**零星背景噪点连通域**；而本模块是「**先到先得**」——
    //           画面上方的噪点先把名额占满 ⇒ 下方真目标拿不到 label
    //           ⇒ **框只能套屏幕靠上部分的物体，物体在下半部分就没有框**。
    //
    //     ⚠ 曾尝试「扩容到 16」，**实测代价远超预估，已放弃**：
    //       · slice 占用 79.6% → **93.42%**（conn_label 涨到 4964 LUT / 2907 reg，全设计最大模块）
    //       · 布局后 Hold 违例 −266ps / 595 条；布线阶段违规数反复涨落（68 → 2918 → 1180），
    //         单次布线跑了 20+ 分钟仍未收敛 ⇒ 余量太小、性价比极低。
    //     ⇒ 改为**从源头减少「占名额的连通域」**：新增 `LMIN`（见下），
    //       把「单行游程过短」的噪点直接排除在连通域之外 —— **几乎零成本**，且不动容量。
    // 2026-10-24: geometric-center grouping prunes unused moments; evaluate 16 slots.
    localparam integer NEG = CCL_CAPACITY;                   // 展开深度（内部连通域上限）
    localparam [LW-1:0] CC_MAX = NEG[LW-1:0];     // 内部 label 分配上限（替代 LBMAX）

    //   ★ v7.9 新增「单行游程最小长度门限」`LMIN`（**定义在 module parameter 列表**）：
    //     一行内连续命中的长度 < LMIN 时，该游程整体视为噪点，**完全不参与连通域**：
    //       不新建 label、不与上一行合并、不累加统计、不写 rbuf
    //     ⇒ 它**根本不占用 NEG 名额**，真目标自然进得来。
    //
    //     为什么有效：**噪点在单行的游程都很短**（几个像素），
    //                 而目标的游程是几十~几百像素（显著长于噪点）。
    //     取值 4~16 可调：
    //       · 调大 → 噪点清得更干净，但目标的**窄部**（游程 < LMIN，如三角尖角）会被削掉
    //       · 调小 → 更保目标形状，但短噪点可能漏过
    //     ⚠ 若发现「小目标 / 细长竖直目标被削没了」⇒ 把 LMIN 往下调（8 → 6 → 4）。
    //     ⚠ 本模块的 unit TB 图样是细线条（游程 1~7），**例化时传 `.LMIN(1)`**
    //       以保持完整覆盖；工程顶层不传，用参数默认值 8。

    localparam [19:0]    MINA  = MIN_AREA[19:0];
    localparam [10:0]    XINIT = 11'd2047;        // xmin 初始
    localparam [9:0]     YINIT = 10'd1023;        // ymin 初始
    localparam [9:0]     YLAST = IMG_HEIGHT[9:0] - 10'd1;   // 最后一行号

    // ── 帧末触发 ────────────────────────────────────────────────────────
    //  ⚠ 不能用 I_frame_done（VTC 的 user 落在**帧首有效像素**，不是消隐期）——
    //    若在帧首触发，汇总 FSM 会与下一帧的有效行抢时间、把累加器清掉。
    //  改为内部检测「最后一行结束」，汇总 FSM 天然落在**场消隐期**：
    //    720p60 场消隐约 49500 拍，而 FSM 最坏约 2200 拍（32 个连通域）。
    //  ★ v1.5.2：帧末触发另加 `&& !ev_valid` —— 与线间比对轮的「延后一拍」配套。
    //    线间比对/游程写入统一切到 ev_valid_r 后，**行末那个游程**要延后一拍才被消费。
    //    frame_end 那一拍正好是 ev_valid=1 的拍（见 RLE 行末收尾分支：!I_de && in_run
    //    → ev_valid<=1），若此刻就进 FSM，下一拍 ev_valid_r=1 会把该游程误写进 rbuf，
    //    导致下一帧开局用错 bank-0 → 全帧静默错位。
    //    加 !ev_valid 后改为「ev_valid 落下的下一拍」进 FSM，此时 ev_valid_r 已消费完毕。
    //    代价：FSM 起点晚 1 拍（消隐期 49500 拍，可忽略）。
    wire frame_end_pre = I_de_d && !I_de && (I_pix_y == YLAST) && !ev_valid;
    reg frame_end;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) frame_end <= 1'b0;
        else frame_end <= frame_end_pre;
    end

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

    // 帧末用：完全解析（保证拿到真正的根）
    //  ⚠ 面积优化（v1.5.1）：原写法是「函数 + 内 32 级 for 循环」，被 S_IDLE 的
    //    `for(ii=0..32)` 调用 33 次 → 综合器内联 33 份 32 级指针链 ≈ 数万 LUT。
    //    现改为**状态机分时**：S_RSLV 状态每拍推进一级，全 33 个 label 共用
    //    同一套指针追踪逻辑（综合器只生成 1 份）。功能完全等价。
    //  （保留函数定义供 TB/其它零散引用，但主路径已不用它。）
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
                    end else if (COLOR_SEPARATE && I_color != run_cls) begin
                        // Close the old class at x-1 and seed the new class at x.
                        // No mask gap is required between differently colored objects.
                        ev_valid <= 1'b1;
                        ev_x0    <= run_x0;
                        ev_x1    <= I_pix_x - 11'd1;
                        ev_len   <= run_len;
                        ev_sumx  <= run_sumx;
                        ev_y     <= I_pix_y;
                        ev_cls   <= run_cls;
                        run_x0   <= I_pix_x;
                        run_len  <= 20'd1;
                        run_sumx <= {10'd0, I_pix_x};
                        run_cls  <= I_color;
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
    reg [1:0]    rbuf_c [0:2*NEG-1];
    reg [LW-1:0] cnt_wr, cnt_rd;
    reg          wr_bank;

    // ── bank 基址 ───────────────────────────────────────────────────────
    //  ★ 时序优化（v1.5.2）：rd_base 由「组合推导」改为「寄存器输出」。
    //    问题：原来 `wire rd_base = wr_bank ? 0 : NEG` 直接参与 24 个读口
    //      （8 路 × 3 个数组 rbuf_s/e/l）的地址加法 → 综合报 fanout=128、
    //      net 延迟 13.3ns（占最差路径 68%），最差路径 20 级逻辑、WNS -6.386ns
    //      @ HDMI_PIXEL_CLK(75MHz)，826 条违例。
    //    处置：rd_base 打一拍寄存（wr_base 保持组合，因为它只用于「写」且扇出小）。
    //    等价性：wr_bank 只在 do_swap 那拍（行末 hblank 内）翻转，
    //      而回放/行间比对都发生在**下一行 I_de 有效时**，hblank 有 ≥4 拍余量，
    //      故 rd_base 晚一拍更新**不影响任何行为**（位级对拍已回归验证）。
    wire [6:0] wr_base = wr_bank ? NEG[6:0] : 7'd0;
    reg  [6:0] rd_base;

    //  rd_base 必须在 do_swap 的**下一拍**就等于新的读 bank 基址。
    //  原组合式：rd_base = wr_bank ? 0 : NEG                （读「非写入」那个 bank）
    //  do_swap 那拍 wr_bank <= ~wr_bank（下一拍生效），
    //  故寄存式 = 「下一拍的 wr_bank」代入原式 = (~wr_bank) ? 0 : NEG
    //                                            = wr_bank ? NEG : 0
    //  ⚠ 不能写成 `wr_bank ? 0 : NEG` —— 那会晚一拍、读错 bank（实测出 X）。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)            rd_base <= 7'd0;
        else if (I_frame_done) rd_base <= 7'd0;
        else if (do_swap)      rd_base <= wr_bank ? NEG[6:0] : 7'd0;
    end

    //=========================================================================
    //  4. 行间重叠比对 + label 指派 + 等价记录
    //=========================================================================
    //  ★★ 时序优化（v1.5.2）：prv_lbl / ovl_hit 改为**寄存器输出**。
    //    改造前最差路径（syn_1 报告，HDMI_PIXEL_CLK 75MHz，周期 13.333ns）：
    //      wr_bank_reg → rd_base(组合) → DRAM 读 rbuf_* →
    //      resolve3(3 级 eq 链) → hit 比较 → 8 级 min2 归约树 →
    //      lbl_use → acc_* 写译码/加法 → acc_ymax_reg[*].ce
    //      合计 20 级逻辑、Data Path 19.652ns（cell 32% / **net 68%**）、
    //      Max Fanout 128 @ rd_base、WNS -6.386ns、826 条违例。
    //    处置：在「命中/前驱 label 算出」之后插一级流水寄存器组 ——
    //      （a）prv_lbl_r/ovl_hit_r：切掉「DRAM读+resolve3+比较+归约树」
    //            对下游 acc_* 写译码的长组合链；
    //      （b）事件字段 ev_*_r 同步打一拍，保证下游统计/等价/分类
    //            用的全是**同一事件**（不能只延 prv_lbl 不延 ev_*，否则错配）。
    //    流水后：第 1 段 = rd_base_reg → DRAM读 → resolve3 → hit 比较 → 寄存；
    //            第 2 段 = 寄存 → 归约树 → lbl_use → acc 写译码 → acc_ymax.ce。
    //    等价性：整条「行间比对/游程写入」通路统一延后一拍，
    //      帧末 FSM 起点同步用 !ev_valid 避开行末残留事件，
    //      仲裁 do_swap 也用 ev_valid_r，I_frame_done 清空 ev_valid_r，
    //      位级对拍已全量回归验证（conn/target_bus/top_chain/common 四套全 PASS）。
    //=========================================================================
    wire [LW-1:0] prv_lbl_raw [0:NEG-1];
    wire          ovl_hit_raw [0:NEG-1];

    // Expand the CURRENT interval once, shared by every previous-row entry.
    // Cached bounds/replay and accumulated object geometry stay unchanged.
    wire [11:0] ev_link_lo=({1'b0,ev_x0}>=LINK_GAP)?{1'b0,ev_x0}-LINK_GAP:12'd0;
    wire [11:0] ev_link_hi={1'b0,ev_x1}+LINK_GAP;
    genvar gj;
    generate
        for (gj = 0; gj < NEG; gj = gj + 1) begin : g_ovl
            wire j_valid = (gj < cnt_rd);
            wire hit_raw = j_valid
                         && (ev_link_lo <= {1'b0,rbuf_e[rd_base + gj]})
                         && ({1'b0,rbuf_s[rd_base + gj]} <= ev_link_hi)
                         && (!COLOR_SEPARATE || ev_cls == rbuf_c[rd_base + gj]);
            assign ovl_hit_raw[gj] = hit_raw && ev_valid;
            assign prv_lbl_raw[gj] = hit_raw ? resolve3(rbuf_l[rd_base + gj]) : {LW{1'b0}};
        end
    endgenerate

    //---- 一级流水寄存 ----------------------------------------------------
    reg [LW-1:0] prv_lbl_r [0:NEG-1];
    reg          ovl_hit_r [0:NEG-1];
    // 事件字段随行打拍（下游一律用 *_r 版本）
    reg          ev_valid_r;
    reg [10:0]   ev_x0_r, ev_x1_r;
    reg [19:0]   ev_len_r;
    reg [20:0]   ev_sumx_r;
    reg [9:0]    ev_y_r;
    reg [1:0]    ev_cls_r;

    integer qi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ev_valid_r <= 1'b0;
            ev_x0_r    <= 11'd0;
            ev_x1_r    <= 11'd0;
            ev_len_r   <= 20'd0;
            ev_sumx_r  <= 21'd0;
            ev_y_r     <= 10'd0;
            ev_cls_r   <= 2'b11;
            for (qi = 0; qi < NEG; qi = qi + 1) begin
                prv_lbl_r[qi] <= {LW{1'b0}};
                ovl_hit_r[qi] <= 1'b0;
            end
        end else if (I_frame_done) begin
            // ★ I_frame_done 一行末那拍（或紧随其后）出现，此处显式丢弃尚未消费的事件，
            //   不把它带到下一帧：下一帧首拍 ev_valid_r=0 → 不消费 → 无残留。
            ev_valid_r <= 1'b0;
            ev_cls_r   <= 2'b11;
            for (qi = 0; qi < NEG; qi = qi + 1) begin
                prv_lbl_r[qi] <= {LW{1'b0}};
                ovl_hit_r[qi] <= 1'b0;
            end
        end else begin
            ev_valid_r <= ev_valid;
            ev_x0_r    <= ev_x0;
            ev_x1_r    <= ev_x1;
            ev_len_r   <= ev_len;
            ev_sumx_r  <= ev_sumx;
            ev_y_r     <= ev_y;
            ev_cls_r   <= ev_cls;
            for (qi = 0; qi < NEG; qi = qi + 1) begin
                prv_lbl_r[qi] <= prv_lbl_raw[qi];
                ovl_hit_r[qi] <= ovl_hit_raw[qi];
            end
        end
    end

    //---- 下游统一使用寄存版（等价于原 prv_lbl / ovl_hit 延后一拍）----
    wire [LW-1:0] prv_lbl [0:NEG-1];
    wire          ovl_hit [0:NEG-1];
    generate
        for (gj = 0; gj < NEG; gj = gj + 1) begin : g_ovl_r
            assign prv_lbl[gj] = prv_lbl_r[gj];
            assign ovl_hit[gj] = ovl_hit_r[gj];
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

    // ★ 面积优化（v1.5.1）：归约树按 NEG 自适应（原来硬编码 32 项 → 三分树）。
    //   现改为通用「逐级对折」：每级把上一级的相邻两项 min2 合并，直到只剩 1 项。
    //   NEG=8 时：8 → 4 → 2 → 1，共 3 级 7 个 min2。
    wire [LW-1:0] lv1 [0:NEG-1];          // 第一级：命中则取 label，否则 0
    wire [LW-1:0] min_lbl;

    generate
        for (gj = 0; gj < NEG; gj = gj + 1)
            assign lv1[gj] = ovl_hit[gj] ? prv_lbl[gj] : {LW{1'b0}};
    endgenerate

    generate
        if (NEG == 8) begin : g_red8
            wire [LW-1:0] l4 [0:3];
            wire [LW-1:0] l2 [0:1];
            for (gj = 0; gj < 4; gj = gj + 1)
                assign l4[gj] = min2(lv1[2*gj], lv1[2*gj + 1]);
            for (gj = 0; gj < 2; gj = gj + 1)
                assign l2[gj] = min2(l4[2*gj], l4[2*gj + 1]);
            assign min_lbl = min2(l2[0], l2[1]);
        end else if (NEG == 4) begin : g_red4
            wire [LW-1:0] l2 [0:1];
            for (gj = 0; gj < 2; gj = gj + 1)
                assign l2[gj] = min2(lv1[2*gj], lv1[2*gj + 1]);
            assign min_lbl = min2(l2[0], l2[1]);
        end else if (NEG > 8 && NEG <= 16) begin : g_red16
            wire [LW-1:0] padded [0:15];
            for(gj=0;gj<16;gj=gj+1) begin: pad_reduction
                if(gj<NEG) assign padded[gj]=lv1[gj];
                else assign padded[gj]={LW{1'b0}};
            end
            wire [LW-1:0] l8 [0:7];
            wire [LW-1:0] l4 [0:3];
            wire [LW-1:0] l2 [0:1];
            for (gj = 0; gj < 8; gj = gj + 1)
                assign l8[gj] = min2(padded[2*gj], padded[2*gj + 1]);
            for (gj = 0; gj < 4; gj = gj + 1)
                assign l4[gj] = min2(l8[2*gj], l8[2*gj + 1]);
            for (gj = 0; gj < 2; gj = gj + 1)
                assign l2[gj] = min2(l4[2*gj], l4[2*gj + 1]);
            assign min_lbl = min2(l2[0], l2[1]);
        end else if (NEG == 2) begin : g_red2
            assign min_lbl = min2(lv1[0], lv1[1]);
        end else begin : g_red_unsup
            assign min_lbl = {LW{1'b0}};      // 兜底（不应走到）
        end
    endgenerate

    //---- 新 label 分配（7 bit 便于判越界）----
    reg [6:0]     lbl_next;
    reg [2:0]     seen_colors;
    // Keep ONE reclaimed label pending. At most one is reclaimed per row;
    // this avoids an eight-way free bitmap, priority mux and popcount on the
    // pixel-event path. Valid components still retain their own labels.
    reg [LW-1:0] free_label;
    reg reclaim_fire;
    reg [LW-1:0] reclaim_label;
    wire use_recycled=RECLAIM_SMALL && (free_label!=0);
    // Labels remain contiguous for the existing frame summary FSM. Stop
    // allocating extra islands before they consume unseen colors' last slots.
    wire [6:0] reserve_need = RESERVE_COLORS ?
        ((!seen_colors[0] && ev_cls_r != 2'd0) ? 7'd1 : 7'd0) +
        ((!seen_colors[1] && ev_cls_r != 2'd1) ? 7'd1 : 7'd0) +
        ((!seen_colors[2] && ev_cls_r != 2'd2) ? 7'd1 : 7'd0) : 7'd0;
    wire          need_new = (min_lbl == {LW{1'b0}});
    // ★ 面积优化（v1.5.1）：分配上限改为内部深度 CC_MAX(=NEG)，不再用接口宏 LBMAX(=32)。
    //   保证分配出的 label 一定落在 NEG 项数组（eq/cls_lbl/acc_*）的合法下标内。
    // Unused labels already cover every unseen color by construction. A
    // recycled label is additional capacity and cannot spend that reserve.
    wire          lbl_ok   = !need_new || use_recycled ||
                             (lbl_next <= ({1'b0,CC_MAX}-reserve_need));
    wire [LW-1:0] fresh    = use_recycled ? free_label : lbl_next[LW-1:0];
    wire [LW-1:0] lbl_use  = need_new ? fresh : min_lbl;
    // ★ v1.5.2：门控 —— 只要求「事件有效 + label 未溢出」。
    //   （曾试过再加 sec_first 抑制首拍，但那会误杀**同一行的第 2 个及以后的游程**，
    //     因为 sec_first 是「本行首拍」而非「本帧首拍」。残留事件已由
    //     frame_end 的 !ev_valid 条件 + I_frame_done 清 ev_valid_r 双重排除，无需再抑制。）
    //   ★ v7.9：再加一道「游程最小长度」门控（`ev_long`）——
    //     短游程（单行长度 < LMIN）的事件在此被**整体丢弃**：
    //       不新建 label、不合并等价、不递增 lbl_next、不锁存颜色、
    //       不累加统计、不写游程缓冲。
    //     ⇒ 噪点**根本不占用 NEG 名额**，真目标不会被挤出去。
    //     （`ev_len_r` 与 `ev_valid_r` 同一拍有效，均来自行 356 的同步打拍。）
    wire          ev_long  = (ev_len_r >= LMIN[19:0]) ||
                             (CONTINUE_SHORT && !need_new && ev_len_r >= 20'd2);
    wire          ev_ok    = ev_valid_r && lbl_ok && ev_long;

    //---- 等价合并：与 min_lbl 不同且重叠的 label → 并入 min_lbl ----
    integer mi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (mi = 0; mi <= NEG; mi = mi + 1) eq[mi] <= mi[LW-1:0];
        end else if (ev_ok && !need_new) begin
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
        else if (ev_ok && need_new && !use_recycled && (lbl_next <= {1'b0, CC_MAX}))
            lbl_next <= lbl_next + 7'd1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) seen_colors <= 3'b000;
        else if (I_frame_done) seen_colors <= 3'b000;
        else if (ev_ok && need_new && ev_cls_r != 2'd3)
            seen_colors[ev_cls_r] <= 1'b1;
    end

    //=========================================================================
    //  4.1 每 label 颜色锁存（COLOR_SEPARATE=1 的边界判定由游程颜色完成）
    //      规则：取该连通域「首次被指派时的已知色」（游程内首个非 UNKNOWN 像素）；
    //            等价合并时把颜色沿并查链往根传播，保证链式合并（eq[5]=3、eq[3]=1）
    //            也能把颜色带到最终根；
    //            全程未知 ⇒ 2'b11（`CLS_UNKNOWN`）。
    //      默认模式要求同一连通域内颜色一致；混色时取首次颜色。
    //      顶层启用 COLOR_SEPARATE，同一连通域不会混入另一颜色的游程。
    //=========================================================================
    reg [1:0] cls_lbl [0:NEG];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (mi = 0; mi <= NEG; mi = mi + 1) cls_lbl[mi] <= 2'b11;
        end else if (I_frame_done) begin
            for (mi = 0; mi <= NEG; mi = mi + 1) cls_lbl[mi] <= 2'b11;
        end else if (ev_ok) begin
            if (need_new) begin
                cls_lbl[fresh] <= ev_cls_r;                      // 新 label
            end else if(!COLOR_SEPARATE) begin
                // Separate-color runs only merge with the same birth class;
                // their root class cannot change. Keep propagation for legacy
                // mixed-color mode, avoiding a redundant root-read/write chain.
                // ① 根未定色 → 用本游程色
                if ((cls_lbl[min_lbl] == 2'b11) && (ev_cls_r != 2'b11))
                    cls_lbl[min_lbl] <= ev_cls_r;
                // ② 被并入者未定色 → 继承根色（保证链式传递）
                for (mi = 0; mi < NEG; mi = mi + 1)
                    if (ovl_hit[mi] && (prv_lbl[mi] != min_lbl)
                        && (cls_lbl[prv_lbl[mi]] == 2'b11))
                        cls_lbl[prv_lbl[mi]] <= cls_lbl[min_lbl];
                // ③ 根与本游程都未知 → 从任一被并入者取色
                for (mi = 0; mi < NEG; mi = mi + 1)
                    if (ovl_hit[mi] && (prv_lbl[mi] != min_lbl)
                        && (cls_lbl[min_lbl] == 2'b11) && (ev_cls_r == 2'b11)
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
    reg [NEG:0] acc_area_ok, acc_tall;
    reg [NEG:0] row_seen, recent_row0, recent_row1, recent_row2;

    reg        acc_clr;                           // 帧末清零脉冲

    // Inspect one component per blanking clock after the run/stat pipelines
    // have drained. Reclaim only roots with NO incoming equivalence links,
    // at least two completely intervening rows, and insufficient area or
    // height for the existing tracker (height must span at least four rows).
    // Valid completed objects remain available for the normal frame summary.
    wire de_fall=I_de_d && !I_de;
    reg [9:0] last_active_y;
    reg [3:0] reclaim_delay;
    reg scan_active;
    reg [LW-1:0] scan_label;
    reg scan_child;
    integer ci;
    always @* begin
        scan_child=0;
        for(ci=1;ci<=NEG;ci=ci+1)
            if(ci!=scan_label && eq[ci]==scan_label) scan_child=1;
    end
    wire scan_recent=recent_row0[scan_label] || recent_row1[scan_label] || recent_row2[scan_label];
    wire reclaim_candidate=RECLAIM_SMALL && scan_active && !I_de &&
        scan_label!=0 && scan_label<lbl_next && free_label==0 && !reclaim_fire &&
        eq[scan_label]==scan_label && !scan_child &&
        !scan_recent && (!acc_area_ok[scan_label] || !acc_tall[scan_label]);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            last_active_y<=0; reclaim_delay<=0; scan_active<=0;
            scan_label<=1; reclaim_fire<=0; reclaim_label<=0;
        end else begin
            reclaim_fire<=0;
            if(I_de) last_active_y<=I_pix_y;
            if(I_frame_done || !RECLAIM_SMALL) begin
                reclaim_delay<=0; scan_active<=0;
            end else if(de_fall && last_active_y!=YLAST) begin
                reclaim_delay<=8; scan_active<=0;
            end else if(I_de) begin
                reclaim_delay<=0; scan_active<=0;
            end else if(reclaim_delay!=0) begin
                reclaim_delay<=reclaim_delay-4'd1;
                if(reclaim_delay==1) begin scan_active<=1; scan_label<=1; end
            end else if(scan_active) begin
                reclaim_fire<=reclaim_candidate;
                reclaim_label<=scan_label;
                if(reclaim_candidate || scan_label==CC_MAX) scan_active<=0;
                else scan_label<=scan_label+1'b1;
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) free_label<=0;
        else if(I_frame_done) free_label<=0;
        else begin
            if(reclaim_fire) free_label<=reclaim_label;
            else if(ev_ok && need_new && use_recycled) free_label<=0;
        end
    end

    reg frame_capacity_full;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin frame_capacity_full<=0; O_capacity_full<=0; end
        else if(I_frame_done) begin
            O_capacity_full<=frame_capacity_full; frame_capacity_full<=0;
        end else if((ev_valid_r && ev_long && need_new && !lbl_ok) ||
                    (ev_ok && cnt_wr>=CC_MAX)) frame_capacity_full<=1;
    end

    // Separate label selection from accumulator addressing/addition. This breaks
    // the high-fanout prv_lbl -> min_lbl -> accumulator/DSP path.
    reg stat_valid;
    reg [LW-1:0] stat_label;
    reg [19:0] stat_len;
    reg [20:0] stat_sumx;
    reg [10:0] stat_x0, stat_x1;
    reg [9:0] stat_y;
    wire [19:0] stat_next_area=acc_area[stat_label]+stat_len;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stat_valid <= 1'b0;
            stat_label <= 0; stat_len <= 0; stat_sumx <= 0;
            stat_x0 <= 0; stat_x1 <= 0; stat_y <= 0;
        end else begin
            stat_valid <= ev_ok && !I_frame_done && !acc_clr;
            if (ev_ok) begin
                stat_label <= lbl_use; stat_len <= ev_len_r;
                stat_sumx <= ev_sumx_r;
                stat_x0 <= ev_x0_r; stat_x1 <= ev_x1_r; stat_y <= ev_y_r;
            end
        end
    end

    // Only five bits per label are needed by the blanking reclaimer. Record
    // occupancy while the existing run accumulator already owns its address,
    // rather than adding independent wide read muxes across all statistics.
    // The eight-clock drain guarantees no run write coincides with row shift.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            row_seen<=0; recent_row0<=0; recent_row1<=0; recent_row2<=0;
        end else if(I_frame_done) begin
            row_seen<=0; recent_row0<=0; recent_row1<=0; recent_row2<=0;
        end else if(!I_de && reclaim_delay==1) begin
            recent_row2<=recent_row1; recent_row1<=recent_row0;
            recent_row0<=row_seen; row_seen<=0;
        end else if(stat_valid) row_seen[stat_label]<=1;
    end

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
                acc_area_ok[ai]<=0; acc_tall[ai]<=0;
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
                acc_area_ok[ai]<=0; acc_tall[ai]<=0;
            end
        end else if (reclaim_fire) begin
            acc_area[reclaim_label]<=0;
            acc_sumx[reclaim_label]<=0; acc_sumy[reclaim_label]<=0;
            acc_xmin[reclaim_label]<=XINIT; acc_xmax[reclaim_label]<=0;
            acc_ymin[reclaim_label]<=YINIT; acc_ymax[reclaim_label]<=0;
            acc_area_ok[reclaim_label]<=0; acc_tall[reclaim_label]<=0;
        end else if (stat_valid) begin
            acc_area[stat_label] <= stat_next_area;
            acc_area_ok[stat_label]<=(stat_next_area>=RECLAIM_MIN_AREA);
            acc_tall[stat_label]<=acc_tall[stat_label] ||
                ({1'b0,stat_y}>={1'b0,acc_ymin[stat_label]}+11'd3);
            acc_sumx[stat_label] <= acc_sumx[stat_label] + {10'd0, stat_sumx};
            acc_sumy[stat_label] <= acc_sumy[stat_label] + ({10'd0, stat_y} * {12'd0, stat_len});
            if (stat_x0 < acc_xmin[stat_label]) acc_xmin[stat_label] <= stat_x0;
            if (stat_x1 > acc_xmax[stat_label]) acc_xmax[stat_label] <= stat_x1;
            if (stat_y  < acc_ymin[stat_label]) acc_ymin[stat_label] <= stat_y;
            if (stat_y  > acc_ymax[stat_label]) acc_ymax[stat_label] <= stat_y;
        end
    end

    //=========================================================================
    //  6. 游程写入 + bank 交换 + 帧首复位
    //=========================================================================
    reg [3:0] swap_wait;
    wire      do_swap = (swap_wait == 4'd1) && !ev_valid_r;
    // ★ v1.5.2：do_swap 的抑制条件随「行间比对轮」整体延后一拍 —— 用 ev_valid_r
    //   （而不是 ev_valid）。因为 ev_valid 那一拍事件还在「被流水寄存」，若此刻
    //   就交换 bank，下一拍（ev_valid_r 生效拍）回放读到的就是已经换过的 bank/q，
    //   事件被静默丢弃（首行首游程会漏统计）。用 ev_valid_r 可保时序也保正确性。

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
            if (de_fall) swap_wait <= 4'd3;   // ★ 必须在 hblank 内完成交换，
                                              //   否则下一行 x=0 那一拍回放会用旧 cnt_rd
                                              //   ★ v1.5.2：由 2 改 3 —— 整条行间比对通路
                                              //   延后一拍后，行末最后游程（ev_valid=1、
                                              //   即 de_fall 那一拍）需再等 1 拍才被消费完
                                              //   （ev_valid_r 落回 0），故多留 1 拍余量。
            else if (swap_wait != 4'd0) swap_wait <= swap_wait - 4'd1;

            if (ev_ok) begin
                if (cnt_wr < CC_MAX) begin
                    rbuf_s[wr_base + cnt_wr] <= ev_x0_r;
                    rbuf_e[wr_base + cnt_wr] <= ev_x1_r;
                    rbuf_l[wr_base + cnt_wr] <= need_new ? fresh : min_lbl;
                    rbuf_c[wr_base + cnt_wr] <= ev_cls_r;
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
                     S_CLR  = 3'd5,
                     S_RSLV = 3'd6;    // ★面积优化：分时解析（替代 33 份 resolve_full 内联）

    reg [2:0]    st;
    localparam integer OUT_WAIT_W=(OUT_SPACING>1)?$clog2(OUT_SPACING):1;
    reg [OUT_WAIT_W-1:0] out_wait;
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

    // ★ 面积优化：S_RSLV 分时解析用的寄存器（替代 33 份 resolve_full 内联）
    reg [6:0]  rq_i;                   // 当前解析到第几个 label（0..32）
    reg [6:0]  rk;                     // 指针追踪已走级数（0..NEG）
    reg [LW-1:0] rt;                   // 指针追踪当前值

    // ★ 面积优化：S_ADDM 两拍法的中间寄存器（消除双重动态索引）
    reg        adsub;                  // 0=读 root/acc，1=写 tp
    reg [LW-1:0] adx;                  // root[fs] 的读出值（第二拍用作 tp 下标）
    reg        ad_v;                   // acc_area[fs] != 0
    reg [19:0] ada_area;               // acc_area[fs]
    reg [30:0] ada_sumx;               // acc_sumx[fs]
    reg [29:0] ada_sumy;               // acc_sumy[fs]
    reg [10:0] ada_xmin;
    reg [10:0] ada_xmax;
    reg [9:0]  ada_ymin;
    reg [9:0]  ada_ymax;

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
            O_cmp_valid <= 1'b0; out_wait<=0;
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
            rq_i        <= 7'd0;
            rk          <= 7'd0;
            rt          <= {LW{1'b0}};
            adsub       <= 1'b0;
            adx         <= {LW{1'b0}};
            ad_v        <= 1'b0;
            ada_area    <= 20'd0;
            ada_sumx    <= 31'd0;
            ada_sumy    <= 30'd0;
            ada_xmin    <= 11'd0;
            ada_xmax    <= 11'd0;
            ada_ymin    <= 10'd0;
            ada_ymax    <= 10'd0;
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
                    // ★ 面积优化：不再 33 份内联 resolve_full，改为进入 S_RSLV 分时解析。
                    //   同时把 tp_* / cmap / rofc 的清零也一起分时做（原本在同一拍展开）。
                    rq_i <= 7'd0;
                    rk   <= 7'd0;
                    rt   <= {LW{1'b0}};
                    st   <= S_RSLV;
                end
            end
            //-------------------------------------------------------------
            // ★ S_RSLV：分时解析等价表 + 清零 tp_*/cmap/rofc（每拍 1 个 label）
            //   rq_i：第几个 label（0..NEG）
            //   rk  ：指针追踪已走级数（0..NEG）
            //   语义严格等价于原 `resolve_full(x)`：t=x; 重复 NEG 次 t<=eq[t]。
            //   实现：rq_i 从 0..NEG，每个 label 走 NEG 级，共 (NEG+1)*(NEG+1) 拍。
            S_RSLV: begin
                if (rk == 7'd0) begin
                    // 起点：rt = rq_i，开始追踪
                    rt <= rq_i[LW-1:0];
                    rk <= 7'd1;
                end else if (rk == (NEG + 1)) begin
                    // 已走满 NEG 级（rk 从 1..NEG 共 NEG 拍完成 NEG 次 eq 追踪）
                    // 此时 rt 即根，写回 root 并清零该 label 的汇总槽
                    root[rq_i[5:0]]  <= rt;
                    cmap[rq_i[5:0]]  <= 6'd0;
                    rofc[rq_i[5:0]]  <= 6'd0;
                    tp_area[rq_i[5:0]] <= 20'd0;
                    tp_sumx[rq_i[5:0]] <= 31'd0;
                    tp_sumy[rq_i[5:0]] <= 30'd0;
                    tp_xmin[rq_i[5:0]] <= XINIT;
                    tp_xmax[rq_i[5:0]] <= 11'd0;
                    tp_ymin[rq_i[5:0]] <= YINIT;
                    tp_ymax[rq_i[5:0]] <= 10'd0;
                    rk <= 7'd0;
                    if (rq_i == NEG[6:0]) begin
                        rq_i <= 7'd0;
                        fs   <= 7'd0;
                        st   <= S_ADDM;
                    end else begin
                        rq_i <= rq_i + 7'd1;
                    end
                end else begin
                    rt <= eq[rt];
                    rk <= rk + 7'd1;
                end
            end
            //-------------------------------------------------------------
            // 把 acc[i] 归入 tp[root[i]]
            //-------------------------------------------------------------
            // 把 acc[i] 归入 tp[root[i]]
            // ★ 面积优化：原写法 `tp_xxx[root[fs]]` 是**双重动态索引**（先查 root[fs]
            //   再用结果索引 tp_xxx），综合器要同时生成两套 33:1 MUX + 写译码。
            //   现拆成**两拍**：
            //     adsub==0：读 root[fs] → adx（寄存），读 acc_*[fs] → ada_*（寄存）
            //     adsub==1：用 adx 索引 tp_*，做累加/比较，写回
            //   功能等价，只是每个 label 多用 1 拍（33 拍 → 66 拍，可忽略）。
            S_ADDM: begin
                if (fs <= NEG[6:0]) begin
                    if (adsub == 1'b0) begin
                        // 拍 1：取 root[fs] 与 acc_*[fs]（单重索引）
                        adx     <= root[fs[5:0]];
                        ad_v    <= (acc_area[fs[5:0]] != 20'd0);
                        ada_area<= acc_area[fs[5:0]];
                        ada_sumx<= acc_sumx[fs[5:0]];
                        ada_sumy<= acc_sumy[fs[5:0]];
                        ada_xmin<= acc_xmin[fs[5:0]];
                        ada_xmax<= acc_xmax[fs[5:0]];
                        ada_ymin<= acc_ymin[fs[5:0]];
                        ada_ymax<= acc_ymax[fs[5:0]];
                        adsub   <= 1'b1;
                    end else begin
                        // 拍 2：用 adx 索引 tp_*（单重索引），累加/比较后写回
                        if (ad_v) begin
                            tp_area[adx] <= tp_area[adx] + ada_area;
                            tp_sumx[adx] <= tp_sumx[adx] + ada_sumx;
                            tp_sumy[adx] <= tp_sumy[adx] + ada_sumy;
                            if (ada_xmin < tp_xmin[adx]) tp_xmin[adx] <= ada_xmin;
                            if (ada_xmax > tp_xmax[adx]) tp_xmax[adx] <= ada_xmax;
                            if (ada_ymin < tp_ymin[adx]) tp_ymin[adx] <= ada_ymin;
                            if (ada_ymax > tp_ymax[adx]) tp_ymax[adx] <= ada_ymax;
                        end
                        adsub <= 1'b0;
                        fs    <= fs + 7'd1;
                    end
                end else begin
                    fs    <= 7'd0;
                    sc    <= 6'd0;
                    adsub <= 1'b0;
                    st    <= S_SCAN;
                end
            end
            //-------------------------------------------------------------
            // 紧凑编号：根且 area>=MIN_AREA → 1,2,3,...
            S_SCAN: begin
                if (fs <= NEG[6:0]) begin
                    // 注意 rofc 只需 NEG+1 项，但 sc+1 最大 = NEG+1 会越界；
                    // 只在 sc < NEG 时登记（第 NEG 个之后的组件按「超出内部跟踪上限」丢弃，
                    // 统计仍保留在 tp_* 里，不进 O_cmp_* 输出 —— 与面积上限语义一致）。
                    if ((root[fs[5:0]] == fs[5:0]) && (tp_area[fs[5:0]] >= MINA)
                        && (sc < CC_MAX)) begin
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
                if (cid > NEG[6:0]) begin
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
                if(out_wait!=0) out_wait<=out_wait-1'b1;
                else if (fs <= NEG[6:0]) begin
                    if ((fs >= 7'd1) && (fs <= {1'b0, O_cmp_cnt}) &&
                        (rofc[fs[5:0]] != 6'd0)) begin
                        O_cmp_valid <= 1'b1;
                        out_wait<=(OUT_SPACING>1)?OUT_SPACING-1:0;
                        O_cmp_label <= fs[5:0];
                        O_cmp_cls   <= cls_lbl[rofc[fs[5:0]]];
                        O_cmp_area  <= tp_area[rofc[fs[5:0]]];
                        O_cmp_xmin  <= tp_xmin[rofc[fs[5:0]]];
                        O_cmp_xmax  <= tp_xmax[rofc[fs[5:0]]];
                        O_cmp_cx    <= cx_out[rofc[fs[5:0]]];
                        O_cmp_ymin  <= tp_ymin[rofc[fs[5:0]]];
                        O_cmp_ymax  <= tp_ymax[rofc[fs[5:0]]];
                        O_cmp_cy    <= cy_out[rofc[fs[5:0]]];
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
