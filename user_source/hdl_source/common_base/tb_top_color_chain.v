//=============================================================================
//  tb_top_color_chain.v  —  顶层新增链路位级仿真（三通道 mask → 颜色编码 → 总线）
//=============================================================================
//  目的：验证 design_top_wrapper.v 里 D2/D4 新增的那一段接线语义，即
//
//      detect_color_mask.O_mask_r/g/b   （三通道独立掩膜，同拍）
//        │
//        ├─（合并）O_mask = r|g|b  ……… 本 tb 里等价于 msk = r|g|b
//        └─（编码）I_color = r?RED : b?BLUE : g?GREEN : UNK   ← ★重点验证「红>蓝>绿」
//        │
//      conn_label ──► conn_features ──► target_bus(384bit)
//
//  与 tb_target_bus.v 的唯一区别：颜色**不由预置图样直接给**，而是由三通道
//  掩膜按顶层表达式现场编码，从而把「编码优先级」也纳入位级对拍。
//
//  图样文件：sim/pattern_rgb3.hex
//      第 1 行        : 帧数 NF（十六进制）
//      之后 NF×H 行   : 每行一个 3*W bit 十六进制数，高位→低位 = {R, G, B} 掩膜
//                       （位 x 对应像素列 x）
//
//  输出：sim/bus_out_color.txt
//      B <384bit 十六进制> <valid_cnt>
//      N <组件数>
//=============================================================================

`timescale 1ns / 1ps
`include "common_defs.vh"

module tb_top_color_chain;

    localparam integer W  = 32;
    localparam integer H  = 16;
    localparam integer HB = 4;
    localparam integer VB = 8;

    reg clk, rst_n;

    // ---- 桩 detect_color_mask：只给三通道掩膜 + 时序（同拍）----
    reg         de, fdone;
    reg         mr, mg, mb;            // 三通道独立掩膜
    reg  [10:0] px;
    reg  [9:0]  py;

    // ---- 顶层同款编码（★ 与 design_top_wrapper.v 逐字一致）----
    wire        msk = mr | mg | mb;                       // = detect 的 O_mask(r|g|b)
    wire [1:0]  col = mr ? `CLS_RED   :
                      mb ? `CLS_BLUE  :
                      mg ? `CLS_GREEN : `CLS_UNKNOWN;

    // ---- conn_label ←→ conn_features ----
    wire        cv;
    wire [5:0]  clb;
    wire [1:0]  ccls;
    wire [19:0] carea;
    wire [10:0] cxmin, cxmax, ccx;
    wire [9:0]  cymin, cymax, ccy;
    wire [5:0]  ccnt;

    wire [`TGT_BUS_W-1:0] tbus;
    wire [2:0]            tcnt;
    wire                  tupd;

    conn_label #(
        .IMG_WIDTH  (W),
        .IMG_HEIGHT (H),
        .MIN_AREA   (1),
        .LMIN       (1)                 // ★ 测试图样是细线条，设 1 保持完整覆盖（工程用默认 8）
    ) u_conn (
        .clk            (clk),
        .rst_n          (rst_n),
        .I_de           (de),
        .I_mask         (msk),
        .I_color        (col),
        .I_pix_x        (px),
        .I_pix_y        (py),
        .I_frame_done   (fdone),
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

    conn_features #(
        .SLOT_N (`TARGET_MAX)
    ) u_feat (
        .clk                (clk),
        .rst_n              (rst_n),
        .I_frame_done       (fdone),
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

    initial clk = 1'b0;
    always #5 clk = ~clk;

    reg [3*W-1:0] tmem [0:2047];        // {R, G, B} 各 W bit
    integer       nf, f, y, x;
    integer       fBus;
    reg [W-1:0]   rm, gm, bm;

    initial begin
        $readmemh("sim/pattern_rgb3.hex", tmem);
        nf   = tmem[0];
        fBus = $fopen("sim/bus_out_color.txt", "w");

        rst_n = 1'b0;
        de = 1'b0; fdone = 1'b0; mr = 1'b0; mg = 1'b0; mb = 1'b0;
        px = 11'd0; py = 10'd0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);

        for (f = 0; f < nf; f = f + 1) begin
            fdone = 1'b1;
            @(negedge clk);
            fdone = 1'b0;
            repeat (VB) @(negedge clk);

            for (y = 0; y < H; y = y + 1) begin
                rm = tmem[f*H + y + 1][3*W-1 -: W];
                gm = tmem[f*H + y + 1][2*W-1 -: W];
                bm = tmem[f*H + y + 1][  W-1 -: W];
                for (x = 0; x < W; x = x + 1) begin
                    de = 1'b1;
                    px = x[10:0];
                    py = y[9:0];
                    mr = rm[x];
                    mg = gm[x];
                    mb = bm[x];
                    @(negedge clk);
                end
                de = 1'b0;
                repeat (HB) @(negedge clk);
            end

            de = 1'b0;
            repeat (3000) @(negedge clk);
            $fwrite(fBus, "N %0d\n", ccnt);
        end

        fdone = 1'b1; @(negedge clk); fdone = 1'b0;
        repeat (3000) @(negedge clk);
        $fwrite(fBus, "N %0d\n", ccnt);

        repeat (20) @(negedge clk);
        $fclose(fBus);
        $display("TB(top_color_chain) done: frames=%0d", nf);
        $finish;
    end

    always @(posedge clk) begin
        if (tupd)
            $fwrite(fBus, "B %096x %0d\n", tbus, tcnt);
    end

endmodule
