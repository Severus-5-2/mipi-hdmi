//=============================================================================
//  tb_target_bus.v  —  conn_label + conn_features 全链路位级仿真
//=============================================================================
//  说明：
//    用与 tb_conn_label 相同的图样（sim/pattern.hex + sim/pattern_cls.hex），
//    把 detect_color_mask 的位置（掩膜 + 颜色）打给 conn_label，
//    再把 conn_label 的帧末汇总流接给 conn_features，
//    最后把 384 bit target_bus 与 valid_cnt dump 出来，
//    交给 verify_target_bus.py 与 Python 参考实现逐项对拍。
//
//  输出：sim/bus_out.txt
//      B <384bit 十六进制> <valid_cnt>
//      N <组件数>            （跟随 conn_label 的帧汇总数，便于人工核对）
//
//  注意：第 1 个 B 出现在「第 0 帧的 fdone」（此时擂台还是空的）→ 应全 0。
//        第 k+1 个 B 才是第 k 帧的结果。
//=============================================================================

`timescale 1ns / 1ps
`include "common_defs.vh"

module tb_target_bus;

    localparam integer W  = 32;
    localparam integer H  = 16;
    localparam integer HB = 4;
    localparam integer VB = 8;

    reg clk, rst_n;

    reg         de, msk, fdone;
    reg  [1:0]  col;
    reg  [10:0] px;
    reg  [9:0]  py;

    //---- conn_label ←→ conn_features 之间的汇总流线 ----
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

    //------------------------------------------------------------------
    //  DUT 1：连通域
    //------------------------------------------------------------------
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

    //------------------------------------------------------------------
    //  DUT 2：目标列表打包
    //------------------------------------------------------------------
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

    reg [W-1:0]   mem  [0:2047];
    reg [2*W-1:0] cmem [0:2047];
    integer       nf, f, y, x;

    integer fBus;

    initial begin
        $readmemh("sim/pattern.hex",     mem);
        $readmemh("sim/pattern_cls.hex", cmem);
        nf   = mem[0];
        fBus = $fopen("sim/bus_out.txt", "w");

        rst_n = 1'b0;
        de = 1'b0; msk = 1'b0; fdone = 1'b0; col = 2'b11; px = 11'd0; py = 10'd0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);

        for (f = 0; f < nf; f = f + 1) begin
            fdone = 1'b1;
            @(negedge clk);
            fdone = 1'b0;
            repeat (VB) @(negedge clk);

            for (y = 0; y < H; y = y + 1) begin
                for (x = 0; x < W; x = x + 1) begin
                    de  = 1'b1;
                    px  = x[10:0];
                    py  = y[9:0];
                    msk = mem[f*H + y + 1][x];
                    col = cmem[f*H + y + 1][2*x +: 2];
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
        $display("TB(target_bus) done: frames=%0d", nf);
        $finish;
    end

    always @(posedge clk) begin
        if (tupd)
            $fwrite(fBus, "B %096x %0d\n", tbus, tcnt);
    end

endmodule
