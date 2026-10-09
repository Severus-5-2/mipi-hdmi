//=============================================================================
//  tb_detect_color_mask.v  —  detect_color_mask 的位级仿真对拍 testbench
//=============================================================================
//  用法（ModelSim）：
//      vlog ../detect_color_mask.v tb_detect_color_mask.v
//      vsim -c -do "run -all; quit" tb_detect_color_mask
//  产物：rtl_out.txt（每行一个 O_de 有效像素），由 verify_color_mask.py 解析比对。
//
//  激励设计：
//    · 帧结构：1 拍 user 脉冲 → 34 行 × 8 像素（每行末 last 有效）
//    · 前 256 个像素 = 灰阶扫描 R=G=B=0..255
//      —— 灰度必须得到 Cb=Cr=128 且三色掩膜全 0，这是检验取整/偏置的最强用例
//    · 之后 10 个像素 = 纯红/绿/蓝/黑/白 + 拟真目标色
//    · 其余补 0
//=============================================================================
`timescale 1ns / 1ps

module tb_detect_color_mask;

    localparam integer W = 8;      // 每行像素数
    localparam integer H = 34;     // 行数
    localparam integer N = W * H;  // 总像素数 = 272

    reg         clk = 1'b0;
    reg         rst_n = 1'b0;

    reg         I_de    = 1'b0;
    reg         I_vsync = 1'b0;
    reg         I_hsync = 1'b0;
    reg         I_user  = 1'b0;
    reg         I_last  = 1'b0;
    reg  [23:0] I_rgb   = 24'd0;
    reg  [1:0]  I_color_sel = 2'd0;

    wire        O_mask, O_mask_r, O_mask_g, O_mask_b;
    wire [7:0]  O_y, O_cb, O_cr;
    wire        O_de, O_vsync, O_hsync, O_user, O_last, O_frame_done;
    wire [10:0] O_pix_x;
    wire [9:0]  O_pix_y;

    reg [23:0] vec [0:N-1];
    integer    i, f, fd_cnt;

    //-------------------------------------------------------------------------
    detect_color_mask #(
        .IMG_WIDTH  (W),
        .IMG_HEIGHT (H),
        .PIPE_DELAY (0)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .I_de        (I_de),
        .I_vsync     (I_vsync),
        .I_hsync     (I_hsync),
        .I_user      (I_user),
        .I_last      (I_last),
        .I_rgb       (I_rgb),
        .I_color_sel (I_color_sel),
        .O_mask      (O_mask),
        .O_mask_r    (O_mask_r),
        .O_mask_g    (O_mask_g),
        .O_mask_b    (O_mask_b),
        .O_y         (O_y),
        .O_cb        (O_cb),
        .O_cr        (O_cr),
        .O_de        (O_de),
        .O_vsync     (O_vsync),
        .O_hsync     (O_hsync),
        .O_user      (O_user),
        .O_last      (O_last),
        .O_pix_x     (O_pix_x),
        .O_pix_y     (O_pix_y),
        .O_frame_done(O_frame_done)
    );

    always #5 clk = ~clk;   // 100 MHz（仿真用，功能与时序无关）

    //-------------------------------------------------------------------------
    // 输出采集：每有一个有效像素就落一行
    //-------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst_n && O_de) begin
            $fdisplay(f, "%0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
                      O_pix_x, O_pix_y, O_y, O_cb, O_cr,
                      O_mask_r, O_mask_g, O_mask_b, O_mask, O_last);
        end
        if (rst_n && O_frame_done)
            fd_cnt = fd_cnt + 1;
    end

    //-------------------------------------------------------------------------
    // 激励
    //-------------------------------------------------------------------------
    initial begin
        f = $fopen("rtl_out.txt", "w");
        fd_cnt = 0;

        // ---- 构造测试向量 ----
        for (i = 0; i < 256; i = i + 1)
            vec[i] = {i[7:0], i[7:0], i[7:0]};          // 灰阶 0..255

        vec[256] = 24'hFF0000;   // 纯红
        vec[257] = 24'h00FF00;   // 纯绿
        vec[258] = 24'h0000FF;   // 纯蓝
        vec[259] = 24'h000000;   // 纯黑
        vec[260] = 24'hFFFFFF;   // 纯白
        vec[261] = 24'hC83C3C;   // 拟真「红球」（受 AWB/光照影响后的红）
        vec[262] = 24'h3C3CC8;   // 拟真「蓝方块」
        vec[263] = 24'h3CC83C;   // 拟真「绿三角」
        vec[264] = 24'h805030;   // 棕（应不命中任何目标色）
        vec[265] = 24'h808080;   // 中灰
        for (i = 266; i < N; i = i + 1)
            vec[i] = 24'd0;

        // ---- 复位 ----
        rst_n = 1'b0;
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);

        // ---- 帧起始：user 一拍 ----
        I_user = 1'b1; I_de = 1'b0; I_last = 1'b0;
        @(negedge clk);
        I_user = 1'b0;

        // ---- 逐行送像素 ----
        for (i = 0; i < N; i = i + 1) begin
            I_de   = 1'b1;
            I_rgb  = vec[i];
            I_last = ((i % W) == (W - 1)) ? 1'b1 : 1'b0;
            @(negedge clk);
        end
        I_de   = 1'b0;
        I_last = 1'b0;

        // ---- 排空流水线（3 拍）并留足余量 ----
        repeat (12) @(negedge clk);

        $display("TB_DONE: vectors=%0d  frame_done_pulses=%0d", N, fd_cnt);
        $fclose(f);
        $finish;
    end

    // 超时保护
    initial begin
        #200000;
        $display("TB_TIMEOUT");
        $finish;
    end

endmodule
