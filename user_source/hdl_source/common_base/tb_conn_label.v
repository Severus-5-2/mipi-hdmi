//=============================================================================
//  tb_conn_label.v  —  conn_label 位级仿真 testbench
//=============================================================================
//  说明：
//    用小画面（默认 32×16）跑多帧，每帧读入一幅已知掩膜图样，
//    把「行末汇总」与「每像素 label 流」全部 dump 成文本，
//    交给 verify_conn_label.py 与 Python 参考实现逐项对拍。
//
//  图样文件：sim/pattern.hex（掩膜）
//      第 1 行        : 帧数 NF（十六进制）
//      之后 NF×H 行   : 每行一个 W bit 十六进制数（位 x 对应像素列 x）
//  图样文件：sim/pattern_cls.hex（逐像素颜色，2 bit/像素）
//      第 1 行        : 帧数 NF（十六进制）
//      之后 NF×H 行   : 每行一个 2W bit 十六进制数（位对 [2x+1:2x] 对应列 x）
//                       0=红 1=蓝 2=绿 3=未分类（`CLS_*`）
//
//  输出：sim/label_out.txt（每像素）、sim/cmp_out.txt（每帧汇总）
//=============================================================================

`timescale 1ns / 1ps
`include "common_defs.vh"

module tb_conn_label;

    //---- 小画面（加速仿真）----
    localparam integer W  = 32;
    localparam integer H  = 16;

    localparam integer HB = 4;          // 行消隐拍数
    localparam integer VB = 8;          // 场消隐拍数

    reg clk, rst_n;

    //---- DUT 接口 ----
    reg         de, msk, fdone;
    reg  [1:0]  col;                    // 与 msk 同拍的颜色类别
    reg  [10:0] px;
    reg  [9:0]  py;

    wire [5:0]  lbl;
    wire        lv, lde;
    wire [10:0] lpx;
    wire [9:0]  lpy;

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
        .MIN_AREA   (1),                // 测试时不滤小目标
        .LMIN       (1)                 // ★ 测试图样是细线条（游程 1~7），设 1 保持完整覆盖；
                                        //   工程顶层不传此参数，用默认值 8
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .I_de           (de),
        .I_mask         (msk),
        .I_color        (col),
        .I_pix_x        (px),
        .I_pix_y        (py),
        .I_frame_done   (fdone),
        .O_label        (lbl),
        .O_label_valid  (lv),
        .O_label_de     (lde),
        .O_label_pix_x  (lpx),
        .O_label_pix_y  (lpy),
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

    //---- 时钟：74.25MHz → 周期 13.468ns，仿真取整数 10ns ----
    initial clk = 1'b0;
    always #5 clk = ~clk;

    //---- 图样存储器 ----
    reg [W-1:0]   mem  [0:2047];        // mem[0]=NF，其后为掩膜各行
    reg [2*W-1:0] cmem [0:2047];        // cmem[0]=NF，其后为颜色各行
    integer       nf, f, y, x;

    //---- 输出文件 ----
    integer fLbl, fCmp;

    initial begin
        $readmemh("sim/pattern.hex",     mem);
        $readmemh("sim/pattern_cls.hex", cmem);
        nf = mem[0];
        fLbl = $fopen("sim/label_out.txt", "w");
        fCmp = $fopen("sim/cmp_out.txt",   "w");

        rst_n = 1'b0;
        de = 1'b0; msk = 1'b0; fdone = 1'b0; col = 2'b11; px = 11'd0; py = 10'd0;
        repeat (4) @(negedge clk);          // ★ 激励一律在 negedge 驱动
        rst_n = 1'b1;
        @(negedge clk);

        //----------------------------------------------------------------
        // 逐帧激励
        //----------------------------------------------------------------
        for (f = 0; f < nf; f = f + 1) begin
            // 帧首：场消隐 + frame_done 脉冲
            fdone = 1'b1;
            @(negedge clk);
            fdone = 1'b0;
            repeat (VB) @(negedge clk);

            // 有效行
            for (y = 0; y < H; y = y + 1) begin
                for (x = 0; x < W; x = x + 1) begin
                    de  = 1'b1;
                    px  = x[10:0];
                    py  = y[9:0];
                    msk = mem[f*H + y + 1][x];
                    col = cmem[f*H + y + 1][2*x +: 2];
                    @(negedge clk);
                end
                // 行消隐
                de = 1'b0;
                repeat (HB) @(negedge clk);
            end

            // 帧末：模拟场消隐，让汇总 FSM（最坏约 2200 拍）跑完
            de = 1'b0;
            repeat (3000) @(negedge clk);
            $fwrite(fCmp, "N %0d\n", ccnt);
        end

        // 多跑一轮 fdone，把最后一帧的汇总也冲出来
        fdone = 1'b1; @(negedge clk); fdone = 1'b0;
        repeat (3000) @(negedge clk);
        $fwrite(fCmp, "N %0d\n", ccnt);

        repeat (20) @(negedge clk);
        $fclose(fLbl);
        $fclose(fCmp);
        $display("TB done: frames=%0d", nf);
        $finish;
    end

    //--------------------------------------------------------------------
    //  每像素 label 流 dump（用模块给出的真实坐标 O_label_pix_x/y）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (lde)
            $fwrite(fLbl, "P %0d %0d %0d %0d\n", lpx, lpy, lbl, lv);
    end

    //--------------------------------------------------------------------
    //  帧末汇总 dump（label cls area xmin xmax ymin ymax cx cy）
    //--------------------------------------------------------------------
    always @(posedge clk) begin
        if (cv)
            $fwrite(fCmp, "C %0d %0d %0d %0d %0d %0d %0d %0d %0d\n",
                    clb, ccls, carea, cxmin, cxmax, cymin, cymax, ccx, ccy);
        if (fdone)
            $fwrite(fCmp, "F %0d\n", ccnt);
    end

endmodule
