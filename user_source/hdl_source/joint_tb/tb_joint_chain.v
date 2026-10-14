//=============================================================================
//  tb_joint_chain.v —— 合板联合仿真（两边的功能串起来跑）
//=============================================================================
//  目的（2026-10-14 交接版）：
//    我方的"检测链"与张铭晨的"跟踪链"在顶层接上之后，**合起来能不能跑通、跑稳**。
//    本 TB 把顶层真实链路的**核心段**原样串起来：
//
//        逐像素场景(程序生成)
//          │  三通道掩膜 + cls + de + 坐标 + 帧末脉冲
//          ▼
//        mask_despeckle      ← 与工程同参数（HALF=4/KMIN=5/VER_ROWS=3/VER_NEED=2）
//          │  对齐后的 mask / mask_rgb / color / de / 坐标 / 帧末
//          ▼
//        conn_label          ← 工程参数（MIN_AREA=16 / LMIN=8 / NEG=8）
//          │  每像素 label + 帧末汇总流
//          ▼
//        conn_features ──► target_bus(4×96) + valid_cnt
//          │
//          ▼
//        target_assoc        ← 张铭晨 D5（EMA 平滑 + 漏检保持 + 删框）
//          │  稳定框 4 槽
//          ▼
//        box_draw            ← 画框（按 cls 上色）
//
//  验证内容（"跑通" + "跑稳"）：
//    A. 静止双目标：框出现、位置正确、**连续帧之间不抖**
//    B. 目标逐帧平移：框**跟随**（不是卡死/不动）
//    C. 漏检（目标消失）：框**保持**（≤ MAX_MISS 帧），超时后**删框**
//    D. 空帧：**一个框都不该有**（这条是最重要的验收点）
//    E. 长跑：混合场景连续多帧，无 X、无锁死
//    F. 显示侧：box_draw 的命中像素包围盒 == 该帧框坐标（画框位置正确）
//    G. 采样时刻：target_assoc 与 conn_features 用**同一路**帧末脉冲（同源 ⇒ 早 0 拍），
//       符合冻结表 §1.1 铁律"中间地带一律不可用"
//
//  输出：joint_out.txt（逐帧记录），同时打印关键判据
//=============================================================================
`timescale 1ns/1ps
`include "../common_base/common_defs.vh"

module tb_joint_chain;

    //------------------------------------------------------------------
    //  画面与场景
    //------------------------------------------------------------------
    localparam integer W      = 128;      // 画面宽（行有效像素）
    localparam integer H      = 64;       // 画面高
    localparam integer HB     = 6;        // 行消隐
    localparam integer NTGT   = 4;        // 每帧最多 4 个目标（与总线槽数一致）

    localparam integer TOL    = 4;        // 框坐标容差（去斑边缘侵蚀 + EMA + 死区）
    //  ★ 运动场景容差：target_assoc 用一阶 EMA(a=1/4) 平滑位置，
    //    对匀速斜坡的稳态滞后 = v x (1-a)/a；本 TB 用 v=2px/帧 => 滞后约 6px，
    //    再留余量取 12。（容差给太小会把"正常的平滑滞后"误判成"没跟上"。）
    localparam integer TOL_MV = 12;
    localparam integer SETTLE = 600;      // 每帧settle拍数

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // ---- 场景描述 ----
    integer s_x0 [0:NTGT-1];
    integer s_y0 [0:NTGT-1];
    integer s_w  [0:NTGT-1];
    integer s_h  [0:NTGT-1];
    integer s_cls[0:NTGT-1];
    integer s_on [0:NTGT-1];        // 1 = 本帧检出；0 = 模拟漏检/消失
    integer s_n;                    // 本帧有效目标个数

    //------------------------------------------------------------------
    //  驱动
    //------------------------------------------------------------------
    reg         de;
    reg         mr, mg, mb;
    reg  [10:0] px;
    reg  [9:0]  py;
    reg         fdone;

    // ---- 去斑：与工程同参数 ----
    wire        d_de, d_msk, d_mr, d_mg, d_mb, d_fd;
    wire [10:0] d_px;
    wire [9:0]  d_py;

    mask_despeckle #(
        .HALF (4), .KMIN (5), .VER_ROWS (3), .VER_NEED (2), .DCH (2), .ENABLE (1)
    ) u_dspk (
        .clk          (clk),
        .rst_n        (rst_n),
        .I_de         (de),
        .I_mask       (mr | mg | mb),
        .I_mask_r     (mr),
        .I_mask_g     (mg),
        .I_mask_b     (mb),
        .I_frame_done (fdone),
        .I_pix_x      (px),
        .I_pix_y      (py),
        .O_de         (d_de),
        .O_mask       (d_msk),
        .O_mask_r     (d_mr),
        .O_mask_g     (d_mg),
        .O_mask_b     (d_mb),
        .O_frame_done (d_fd),
        .O_pix_x      (d_px),
        .O_pix_y      (d_py)
    );

    // ★ 与 design_top_wrapper.v 逐字一致的颜色编码（红 > 蓝 > 绿）
    wire [1:0] d_col = d_mr ? `CLS_RED :
                       d_mb ? `CLS_BLUE :
                       d_mg ? `CLS_GREEN : `CLS_UNKNOWN;

    //------------------------------------------------------------------
    //  conn_label → conn_features
    //------------------------------------------------------------------
    wire        cv;
    wire [5:0]  clb;
    wire [1:0]  ccls;
    wire [19:0] carea;
    wire [10:0] cxmin, cxmax, ccx;
    wire [9:0]  cymin, cymax, ccy;
    wire [5:0]  ccnt;

    conn_label #(
        .IMG_WIDTH  (W),
        .IMG_HEIGHT (H),
        .MIN_AREA   (16),            // ★ 工程值
        .LMIN       (8)              // ★ 工程值（目标宽 30px ≫ 8，可过）
    ) u_conn (
        .clk            (clk),
        .rst_n          (rst_n),
        .I_de           (d_de),
        .I_mask         (d_msk),
        .I_color        (d_col),
        .I_pix_x        (d_px),
        .I_pix_y        (d_py),
        .I_frame_done   (d_fd),
        .O_label        (),
        .O_label_valid  (),
        .O_label_de     (),
        .O_label_pix_x  (),
        .O_label_pix_y  (),
        .O_cmp_valid    (cv),
        .O_cmp_label    (clb),
        .O_cmp_cls      (ccls),
        .O_cmp_area     (carea),
        .O_cmp_xmin     (cxmin),
        .O_cmp_xmax     (cxmax),
        .O_cmp_cx       (ccx),
        .O_cmp_ymin     (cymin),
        .O_cmp_ymax     (cymax),
        .O_cmp_cy       (ccy),
        .O_cmp_cnt      (ccnt)
    );

    wire [`TGT_BUS_W-1:0] tbus;
    wire [2:0]            tcnt;
    wire                  tupd;

    conn_features #(
        .SLOT_N (`TARGET_MAX)
    ) u_feat (
        .clk                (clk),
        .rst_n              (rst_n),
        .I_frame_done       (d_fd),          // ★ 与 target_assoc 同源
        .I_cmp_valid        (cv),
        .I_cmp_label        (clb),
        .I_cmp_cls          (ccls),
        .I_cmp_area         (carea),
        .I_cmp_xmin         (cxmin),
        .I_cmp_xmax         (cxmax),
        .I_cmp_ymin         (cymin),
        .I_cmp_ymax         (cymax),
        .I_cmp_cx           (ccx),
        .I_cmp_cy           (ccy),
        .O_target_bus       (tbus),
        .O_target_valid_cnt (tcnt),
        .O_update           (tupd)
    );

    //------------------------------------------------------------------
    //  ★ 张铭晨 D5：目标关联（跟踪稳定化）
    //     I_frame_done 与 conn_features **同源**（d_fd）⇒ 采样首拍 = 锁存拍，早 0 拍
    //------------------------------------------------------------------
    wire [3:0]  b_valid;
    wire [43:0] b_xmin, b_xmax, b_cx;
    wire [39:0] b_ymin, b_ymax, b_cy;
    wire [7:0]  b_cls;
    wire [2:0]  trk_cnt;

    target_assoc u_assoc (
        .I_clk         (clk),
        .I_rst_n       (rst_n),
        .I_frame_done  (d_fd),
        .I_target_cnt  (tcnt),
        .I_target_bus  (tbus),
        .O_box_valid   (b_valid),
        .O_box_xmin    (b_xmin),
        .O_box_xmax    (b_xmax),
        .O_box_ymin    (b_ymin),
        .O_box_ymax    (b_ymax),
        .O_box_cls     (b_cls),
        .O_box_cx      (b_cx),
        .O_box_cy      (b_cy),
        .O_track_cnt   (trk_cnt)
    );

    //------------------------------------------------------------------
    //  显示侧：box_draw（用"显帧"驱动，统计命中像素的包围盒）
    //------------------------------------------------------------------
    reg  dsp_de;                    // 显帧的 de（与检帧分开驱动）
    reg  [10:0] dsp_x;
    reg  [9:0]  dsp_y;
    wire bx_hit;
    wire [23:0] bx_color;
    wire        bx_de;

    box_draw #(
        .SELF_TEST  (0),
        .COLOR_CLS0 (24'hff0000),
        .COLOR_CLS1 (24'h00a0ff),
        .COLOR_CLS2 (24'h00ff00),
        .COLOR_CLS3 (24'hfff200)
    ) u_box (
        .I_clk       (clk),
        .I_rst_n     (rst_n),
        .I_de        (dsp_de),
        .I_pix_x     (dsp_x),
        .I_pix_y     (dsp_y),
        .I_box_valid (b_valid),
        .I_box_cls   (b_cls),
        .I_box_xmin  (b_xmin),
        .I_box_xmax  (b_xmax),
        .I_box_ymin  (b_ymin),
        .I_box_ymax  (b_ymax),
        .I_cross_en  (1'b0),
        .I_cross_x   (11'd0),
        .I_cross_y   (10'd0),
        .O_hit       (bx_hit),
        .O_color     (bx_color),
        .O_de        (bx_de)
    );

    //------------------------------------------------------------------
    //  命中统计（显帧期间）
    //  ★ box_draw 的 O_hit / O_de **比 I_de 晚 2 拍**（模块头文件写明）
    //    ⇒ 统计时必须用"延 2 拍的像素坐标"，否则包围盒会整体错位。
    //    （这也是顶层要把 box_draw 接 pix_coord_gen 的 S_pix_x 而不是 _2d 的原因，
    //      box_draw 自己会补这 2 拍。）
    //------------------------------------------------------------------
    reg [10:0] dd_x [0:1];
    reg [9:0]  dd_y [0:1];
    reg        dd_de[0:1];
    integer hx_min, hx_max, hy_min, hy_max, h_cnt, h_any;

    always @(posedge clk) begin
        dd_x[0]  <= dsp_x;   dd_y[0]  <= dsp_y;   dd_de[0] <= dsp_de;
        dd_x[1]  <= dd_x[0]; dd_y[1]  <= dd_y[0]; dd_de[1] <= dd_de[0];
    end

    always @(posedge clk) begin
        if (rst_n && dd_de[1] && bx_hit) begin
            if (h_any == 0) begin
                hx_min = dd_x[1]; hx_max = dd_x[1];
                hy_min = dd_y[1]; hy_max = dd_y[1];
                h_any  = 1;
            end else begin
                if (dd_x[1] < hx_min) hx_min = dd_x[1];
                if (dd_x[1] > hx_max) hx_max = dd_x[1];
                if (dd_y[1] < hy_min) hy_min = dd_y[1];
                if (dd_y[1] > hy_max) hy_max = dd_y[1];
            end
            h_cnt = h_cnt + 1;
        end
    end

    //------------------------------------------------------------------
    //  工具函数
    //------------------------------------------------------------------
    function integer in_tgt;
        input integer x;
        input integer y;
        integer i;
        begin
            in_tgt = 0;
            for (i = 0; i < NTGT; i = i + 1)
                if (s_on[i] == 1)
                    if (x >= s_x0[i] && x < s_x0[i] + s_w[i] &&
                        y >= s_y0[i] && y < s_y0[i] + s_h[i]) in_tgt = 1;
        end
    endfunction

    function integer cls_of;
        input integer x;
        input integer y;
        integer i;
        begin
            cls_of = `CLS_UNKNOWN;
            for (i = 0; i < NTGT; i = i + 1)
                if (s_on[i] == 1)
                    if (x >= s_x0[i] && x < s_x0[i] + s_w[i] &&
                        y >= s_y0[i] && y < s_y0[i] + s_h[i]) cls_of = s_cls[i];
        end
    endfunction

    //------------------------------------------------------------------
    //  逐帧：检帧
    //------------------------------------------------------------------
    integer x, y, f, i;
    integer fLog;
    integer n_fail;

    task clear_scene;
        begin
            for (i = 0; i < NTGT; i = i + 1) begin
                s_on[i] = 0; s_x0[i] = 0; s_y0[i] = 0;
                s_w[i] = 0; s_h[i] = 0; s_cls[i] = 0;
            end
        end
    endtask

    task det_frame;
        begin
            // 帧末脉冲（放在帧首，与既有 TB 约定一致）
            @(negedge clk); fdone = 1'b1;
            @(negedge clk); fdone = 1'b0;
            repeat (HB) @(negedge clk);
            for (y = 0; y < H; y = y + 1) begin
                for (x = 0; x < W; x = x + 1) begin
                    de = 1'b1;
                    px = x[10:0];
                    py = y[9:0];
                    if (in_tgt(x, y)) begin
                        case (cls_of(x, y))
                            `CLS_RED  : begin mr = 1'b1; mg = 1'b0; mb = 1'b0; end
                            `CLS_BLUE : begin mr = 1'b0; mg = 1'b0; mb = 1'b1; end
                            default   : begin mr = 1'b0; mg = 1'b1; mb = 1'b0; end
                        endcase
                    end else begin
                        mr = 1'b0; mg = 1'b0; mb = 1'b0;
                    end
                    @(negedge clk);
                end
                de = 1'b0; mr = 0; mg = 0; mb = 0;
                repeat (HB) @(negedge clk);
            end
            de = 1'b0; mr = 0; mg = 0; mb = 0;
            repeat (SETTLE) @(negedge clk);
        end
    endtask

    // 空转若干帧（无目标）—— 让上一场景的轨迹按 MAX_MISS 自然删除，隔离场景
    task idle_frames;
        input integer n;
        integer q;
        begin
            for (q = 0; q < n; q = q + 1) begin
                clear_scene; s_n = 0;
                det_frame;
            end
        end
    endtask

    // 显帧：不送掩膜，只扫一遍画面统计 box_draw 命中
    task disp_frame;
        begin
            hx_min = W; hx_max = 0; hy_min = H; hy_max = 0; h_cnt = 0; h_any = 0;
            for (y = 0; y < H; y = y + 1) begin
                for (x = 0; x < W; x = x + 1) begin
                    dsp_de = 1'b1; dsp_x = x[10:0]; dsp_y = y[9:0];
                    @(negedge clk);
                end
                dsp_de = 1'b0;
                repeat (2) @(negedge clk);
            end
            dsp_de = 1'b0;
            repeat (8) @(negedge clk);
        end
    endtask

    // 槽 i 的框坐标
    function integer bxs0;  input integer s; begin bxs0 = b_xmin[11*s +: 11]; end endfunction
    function integer bxs1;  input integer s; begin bxs1 = b_xmax[11*s +: 11]; end endfunction
    function integer bys0;  input integer s; begin bys0 = b_ymin[10*s +: 10]; end endfunction
    function integer bys1;  input integer s; begin bys1 = b_ymax[10*s +: 10]; end endfunction

    // ★ 有效框个数（Verilog 要求函数至少一个输入端口 ⇒ 加哑元）
    function integer n_valid;
        input integer dummy;
        integer s, n;
        begin
            n = 0;
            for (s = 0; s < 4; s = s + 1)
                if (b_valid[s] === 1'b1) n = n + 1;
            n_valid = n;
        end
    endfunction

    // ★ 在 4 个槽里找"左上角与给定目标矩形匹配"的框（±tol），返回槽号；-1 = 没找到
    //   ⚠ 必须是**顺序无关**的：conn_features 按 area 降序排槽，槽号 ≠ 场景顺序
    function integer find_box;
        input integer rx0;
        input integer ry0;
        input integer tol;
        integer s;
        begin
            find_box = -1;
            for (s = 0; s < 4; s = s + 1)
                if (b_valid[s] === 1'b1)
                    if (bxs0(s) >= rx0 - tol && bxs0(s) <= rx0 + tol &&
                        bys0(s) >= ry0 - tol && bys0(s) <= ry0 + tol) find_box = s;
        end
    endfunction

    // 逐帧临时量
    integer k0, k1;            // 匹配到的槽号
    integer pv_x, pv_y, pv_ok; // 上一帧的框坐标（查静止稳定性）
    integer exp_n;             // 本帧期望目标数
    integer dv;

    initial begin
        fLog = $fopen("joint_out.txt", "w");
        n_fail = 0;
        fdone = 0; de = 0; mr = 0; mg = 0; mb = 0; px = 0; py = 0;
        dsp_de = 0; dsp_x = 0; dsp_y = 0;
        clear_scene; s_n = 0;
        pv_x = 0; pv_y = 0; pv_ok = 0;

        $display("");
        $display("==================================================================");
        $display("  合板联合仿真：检测链(我方) × 跟踪链(张铭晨)");
        $display("  画面 %0dx%0d | 去斑 HALF=4/KMIN=5/2-of-3 | conn_label MIN_AREA=16/LMIN=8", W, H);
        $display("  ★ 校验是**顺序无关**的：conn_features 按 area 降序排槽，槽号 != 场景顺序");
        $display("==================================================================");

        rst_n = 0;
        repeat (8) @(negedge clk);
        rst_n = 1;
        repeat (4) @(negedge clk);

        //=============================================================
        //  A. 静止双目标（红块小 / 绿块大），连续 12 帧 → 框出现、位置对、不抖
        //=============================================================
        $display("");
        $display("--- A. 静止双目标（12 帧）---");
        $fwrite(fLog, "# A static\n");
        for (f = 0; f < 12; f = f + 1) begin
            clear_scene;
            s_on[0] = 1; s_x0[0] = 16; s_y0[0] = 12; s_w[0] = 30; s_h[0] = 20; s_cls[0] = `CLS_RED;
            s_on[1] = 1; s_x0[1] = 70; s_y0[1] = 30; s_w[1] = 32; s_h[1] = 22; s_cls[1] = `CLS_GREEN;
            det_frame;
            k0 = find_box(16, 12, TOL);      // 小目标（红）
            k1 = find_box(70, 30, TOL);      // 大目标（绿）
            $display("  f%0d  tcnt=%0d trk=%0d 有效框=%0d  目标红→槽%0d  目标绿→槽%0d  v=%b",
                     f, tcnt, trk_cnt, n_valid(0), k0, k1, b_valid);
            $fwrite(fLog, "A %0d tcnt=%0d trk=%0d nv=%0d k0=%0d k1=%0d v=%b\n",
                    f, tcnt, trk_cnt, n_valid(0), k0, k1, b_valid);
            if ((^tbus === 1'bx) || (^b_valid === 1'bx) || (^b_xmin === 1'bx)) begin
                n_fail = n_fail + 1; $display("  !! X 出现在总线/框信号");
            end
            if (f >= 4) begin
                if (n_valid(0) != 2) begin
                    n_fail = n_fail + 1; $display("  !! f%0d 有效框数 %0d != 2", f, n_valid(0));
                end
                if (k0 < 0) begin
                    n_fail = n_fail + 1; $display("  !! f%0d 找不到红色目标的框", f);
                end
                if (k1 < 0) begin
                    n_fail = n_fail + 1; $display("  !! f%0d 找不到绿色目标的框", f);
                end
                // 静止稳定性：与上一帧比，坐标漂移不得超过死区 +1
                if (f > 4 && k0 >= 0 && pv_ok == 1) begin
                    dv = bxs0(k0) - pv_x;
                    if (dv < 0) dv = -dv;
                    if (dv > 3) begin
                        n_fail = n_fail + 1;
                        $display("  !! f%0d 静止目标的框在漂（xmin %0d→%0d）", f, pv_x, bxs0(k0));
                    end
                end
                if (k0 >= 0) begin pv_x = bxs0(k0); pv_y = bys0(k0); pv_ok = 1; end
            end
        end

        //=============================================================
        //  B. 单目标逐帧右移 3px → 框跟随（不是卡死）
        //=============================================================
        $display("");
        $display("--- B. 单目标逐帧右移 2px（14 帧，查框是否跟随）---");
        $fwrite(fLog, "# B moving\n");
        $display("  (先空转 8 帧，清掉上一场景遗留的轨迹)");
        idle_frames(8);
        for (f = 0; f < 14; f = f + 1) begin
            clear_scene;
            s_on[0] = 1; s_x0[0] = 12 + 2*f; s_y0[0] = 20; s_w[0] = 28; s_h[0] = 18; s_cls[0] = `CLS_RED;
            det_frame;
            k0 = find_box(12 + 2*f, 20, TOL_MV);
            $display("  f%0d  目标 x0=%0d  有效框=%0d  匹配槽=%0d  框x=[%0d..%0d]  平滑滞后=%0d",
                     f, 12+2*f, n_valid(0), k0,
                     (k0 >= 0 ? bxs0(k0) : -1), (k0 >= 0 ? bxs1(k0) : -1),
                     (k0 >= 0 ? bxs0(k0) - (12+2*f) : -999));
            $fwrite(fLog, "B %0d tx0=%0d nv=%0d k=%0d xmin=%0d xmax=%0d v=%b\n",
                    f, 12+2*f, n_valid(0), k0,
                    (k0 >= 0 ? bxs0(k0) : -1), (k0 >= 0 ? bxs1(k0) : -1), b_valid);
            if (f >= 2) begin
                if (n_valid(0) < 1) begin
                    n_fail = n_fail + 1; $display("  !! f%0d 运动目标丢框", f);
                end
                if (k0 < 0) begin
                    n_fail = n_fail + 1;
                    $display("  !! f%0d 框没跟上（目标 x0=%0d）", f, 12+2*f);
                end
            end
        end

        //=============================================================
        //  C. 漏检保持：目标先出现 6 帧，然后"消失"
        //=============================================================
        $display("");
        $display("--- C. 漏检保持（出现 6 帧 → 消失）---");
        $fwrite(fLog, "# C miss-hold\n");
        for (f = 0; f < 6; f = f + 1) begin
            clear_scene;
            s_on[0] = 1; s_x0[0] = 40; s_y0[0] = 24; s_w[0] = 30; s_h[0] = 20; s_cls[0] = `CLS_BLUE;
            det_frame;
        end
        $display("  (6 帧后：有效框=%0d，框=[%0d..%0d,%0d..%0d])",
                 n_valid(0), bxs0(0), bxs1(0), bys0(0), bys1(0));
        for (f = 0; f < 9; f = f + 1) begin
            clear_scene; s_n = 0;
            det_frame;
            $display("  消失后第 %0d 帧: tcnt=%0d 有效框=%0d%s",
                     f+1, tcnt, n_valid(0), (n_valid(0)>0 ? "  ← 保持中" : "  ← 已删除"));
            $fwrite(fLog, "C %0d tcnt=%0d nv=%0d\n", f+1, tcnt, n_valid(0));
            if (f < 3) begin
                if (n_valid(0) < 1) begin
                    n_fail = n_fail + 1;
                    $display("  !! 消失后第 %0d 帧就丢框（应保持几帧）", f+1);
                end
            end
            if (f >= 7) begin
                if (n_valid(0) != 0) begin
                    n_fail = n_fail + 1;
                    $display("  !! 消失后第 %0d 帧仍有框（应已删除）", f+1);
                end
            end
        end

        //=============================================================
        //  D. 空帧：一个框都不该有（最重要验收点）
        //=============================================================
        $display("");
        $display("--- D. 空帧（连续 6 帧无目标）---");
        $fwrite(fLog, "# D empty\n");
        for (f = 0; f < 6; f = f + 1) begin
            clear_scene; s_n = 0;
            det_frame;
            $display("  f%0d  tcnt=%0d  有效框=%0d  box_valid=%b", f, tcnt, n_valid(0), b_valid);
            $fwrite(fLog, "D %0d tcnt=%0d nv=%0d\n", f, tcnt, n_valid(0));
            if (b_valid !== 4'b0000) begin
                n_fail = n_fail + 1;
                $display("  !! 空帧仍有框 valid=%b", b_valid);
            end
        end

        //=============================================================
        //  E. 长跑：混合场景 30 帧（目标增减 + 缓慢移动），查稳定性
        //=============================================================
        $display("");
        $display("--- E. 长跑混合场景（30 帧）---");
        $fwrite(fLog, "# E longrun\n");
        for (f = 0; f < 30; f = f + 1) begin
            clear_scene;
            s_on[0] = 1; s_x0[0] = 10 + (f < 15 ? 2*f : 2*(29-f));
            s_y0[0] = 10; s_w[0] = 26; s_h[0] = 16; s_cls[0] = `CLS_RED;
            if (f >= 8) begin
                s_on[1] = 1; s_x0[1] = 80; s_y0[1] = 36; s_w[1] = 30; s_h[1] = 20; s_cls[1] = `CLS_GREEN;
            end
            if (f >= 20) begin
                s_on[2] = 1; s_x0[2] = 20; s_y0[2] = 40; s_w[2] = 28; s_h[2] = 18; s_cls[2] = `CLS_BLUE;
            end
            det_frame;
            exp_n = (f >= 20) ? 3 : ((f >= 8) ? 2 : 1);
            $display("  f%0d  tcnt=%0d trk=%0d 有效框=%0d  本帧场景目标数=%0d",
                     f, tcnt, trk_cnt, n_valid(0), exp_n);
            $fwrite(fLog, "E %0d tcnt=%0d trk=%0d nv=%0d\n", f, tcnt, trk_cnt, n_valid(0));
            if ((^tbus === 1'bx) || (^b_valid === 1'bx)) begin
                n_fail = n_fail + 1; $display("  !! X 出现");
            end
            // 允许 1 帧流水滞后：有效框数 ∈ {上一帧期望, 本帧期望}
            if (f >= 10) begin
                if (n_valid(0) != exp_n && n_valid(0) != (exp_n == 1 ? 1 : exp_n-1)) begin
                    n_fail = n_fail + 1;
                    $display("  !! f%0d 有效框数 %0d != 期望 %0d", f, n_valid(0), exp_n);
                end
            end
        end

        //=============================================================
        //  F. 显示侧：box_draw 画出来的包围盒 == 框坐标
        //=============================================================
        $display("");
        $display("--- F. 显示侧（box_draw 命中包围盒 vs 框坐标）---");
        $fwrite(fLog, "# F display\n");
        $display("  (先空转 8 帧清轨迹，再让新目标收敛 24 帧给 EMA 时间)");
        idle_frames(8);
        for (f = 0; f < 24; f = f + 1) begin
            clear_scene;
            s_on[0] = 1; s_x0[0] = 30; s_y0[0] = 20; s_w[0] = 40; s_h[0] = 24; s_cls[0] = `CLS_GREEN;
            det_frame;
        end
        $display("  有效框=%0d  v=%b", n_valid(0), b_valid);
        for (i = 0; i < 4; i = i + 1)
            if (b_valid[i] === 1'b1)
                $display("    槽%0d: x=[%0d..%0d] y=[%0d..%0d]", i, bxs0(i), bxs1(i), bys0(i), bys1(i));
        k0 = find_box(30, 20, TOL);
        disp_frame;
        $display("  命中包围盒 x=[%0d..%0d] y=[%0d..%0d]  命中像素=%0d",
                 hx_min, hx_max, hy_min, hy_max, h_cnt);
        $fwrite(fLog, "F box0=%0d,%0d,%0d,%0d hit=%0d,%0d,%0d,%0d n=%0d\n",
                bxs0(k0), bxs1(k0), bys0(k0), bys1(k0), hx_min, hx_max, hy_min, hy_max, h_cnt);
        if (k0 < 0) begin
            n_fail = n_fail + 1; $display("  !! 显示侧：没找到该目标的框");
        end
        if (h_cnt == 0) begin
            n_fail = n_fail + 1; $display("  !! 框有效但一个像素都没画出来");
        end
        if (k0 >= 0) begin
            if (hx_min < bxs0(k0)-6 || hx_max > bxs1(k0)+6 ||
                hy_min < bys0(k0)-6 || hy_max > bys1(k0)+6) begin
                n_fail = n_fail + 1;
                $display("  !! 命中包围盒与框坐标不符（超过 ±6 像素）");
            end
        end

        //=============================================================
        //  结论
        //=============================================================
        $display("");
        $display("==================================================================");
        if (n_fail == 0)
            $display("[PASS] 合板联合仿真：5 组场景 + 显示侧 全部通过（框位置/跟随/保持/删除/空帧/长跑）");
        else
            $display("[FAIL] 合板联合仿真：%0d 处不符合预期（见上方 !! 项）", n_fail);
        $display("==================================================================");
        $fclose(fLog);
        $finish;
    end

endmodule
