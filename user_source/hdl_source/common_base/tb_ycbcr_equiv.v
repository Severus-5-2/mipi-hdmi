//=============================================================================
//  tb_ycbcr_equiv.v  —  证明 common_base/ycbcr_convert.v 与
//                        detect_zjy/detect_color_mask.v 的 YCbCr 输出**逐位一致**
//=============================================================================
//  目的：D1 的 detect_color_mask.v 已经过位级对拍（272/272 通过）。
//        现在把 YCbCr 部分抽成公共模块，必须证明**抽出后行为不变**——
//        否则公共模块不可信，两人共用反而放大错误。
//
//  方法：同一套激励同时喂给两个模块，逐像素比较 O_y/O_cb/O_cr/O_de/O_pix_x/O_pix_y。
//=============================================================================

`timescale 1ns / 1ps
`include "common_defs.vh"

module tb_ycbcr_equiv;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;          // 100MHz，够用

    //------------------------------------------------------------------
    // 激励：256 级灰阶 + 纯色 + 拟真目标色（与 D1 同一套）
    //------------------------------------------------------------------
    integer i;
    reg [23:0] rgb_mem [0:271];
    initial begin
        for (i = 0; i < 256; i = i + 1)
            rgb_mem[i] = {i[7:0], i[7:0], i[7:0]};       // 灰阶
        rgb_mem[256] = 24'hFF0000;   // 纯红
        rgb_mem[257] = 24'h00FF00;   // 纯绿
        rgb_mem[258] = 24'h0000FF;   // 纯蓝
        rgb_mem[259] = 24'h000000;   // 纯黑
        rgb_mem[260] = 24'hFFFFFF;   // 纯白
        rgb_mem[261] = 24'hC83C3C;   // 拟真红球 (200,60,60)
        rgb_mem[262] = 24'h3C3CC8;   // 拟真蓝方块 (60,60,200)
        rgb_mem[263] = 24'h3CC83C;   // 拟真绿三角 (60,200,60)
        rgb_mem[264] = 24'h808080;   // 中灰 (128,128,128)
        rgb_mem[265] = 24'hFF8000;   // 橙
        rgb_mem[266] = 24'h8000FF;   // 紫
        rgb_mem[267] = 24'h00FFFF;   // 青
        rgb_mem[268] = 24'hFFFF00;   // 黄
        rgb_mem[269] = 24'h0A0A0A;   // 近黑
        rgb_mem[270] = 24'hF5F5F5;   // 近白
        rgb_mem[271] = 24'h123456;   // 随机
    end

    //------------------------------------------------------------------
    // 视频时序激励（自建，模拟 1 行 16 像素、多帧）
    //------------------------------------------------------------------
    localparam PW = 16;            // 本 tb 用一行 16 像素，方便观察
    localparam PH = 4;             // 每帧 4 行

    reg        v_de, v_user, v_last, v_vsync;
    reg [23:0] v_rgb;
    reg [`PIX_X_W-1:0] v_px;
    reg [`PIX_Y_W-1:0] v_py;

    integer idx, frame, row, col;
    integer fout;
    integer fail_cnt;

    // 公共坐标计数器（模拟顶层 pix_counter）
    wire [`PIX_X_W-1:0] cc_px;
    wire [`PIX_Y_W-1:0] cc_py;
    wire cc_fd;

    pix_counter #(.IMG_WIDTH(PW), .IMG_HEIGHT(PH)) u_cnt (
        .clk(clk), .rst_n(rst_n),
        .I_de(v_de), .I_user(v_user), .I_last(v_last), .I_vsync(v_vsync),
        .O_pix_x(cc_px), .O_pix_y(cc_py), .O_frame_done(cc_fd)
    );

    //------------------------------------------------------------------
    // DUT-A：common_base/ycbcr_convert.v
    //------------------------------------------------------------------
    wire [7:0] a_y, a_cb, a_cr;
    wire       a_gate, a_de;
    wire [`PIX_X_W-1:0] a_px;
    wire [`PIX_Y_W-1:0] a_py;

    ycbcr_convert #(.PIPE_DELAY(0), .Y_MIN(8'd0)) u_a (
        .clk(clk), .rst_n(rst_n),
        .I_de(v_de), .I_vsync(v_vsync), .I_hsync(1'b0),
        .I_rgb(v_rgb), .I_pix_x(cc_px), .I_pix_y(cc_py),
        .O_y(a_y), .O_cb(a_cb), .O_cr(a_cr), .O_gate(a_gate),
        .O_de(a_de), .O_vsync(), .O_hsync(),
        .O_pix_x(a_px), .O_pix_y(a_py)
    );

    //------------------------------------------------------------------
    // DUT-B：detect_zjy/detect_color_mask.v（D1 参考实现，取其 YCbCr 输出）
    //------------------------------------------------------------------
    wire [7:0] b_y, b_cb, b_cr;
    wire       b_de;
    wire [`PIX_X_W-1:0] b_px;
    wire [`PIX_Y_W-1:0] b_py;

    detect_color_mask #(
        .IMG_WIDTH(PW), .IMG_HEIGHT(PH), .PIPE_DELAY(0), .Y_MIN(8'd0)
    ) u_b (
        .clk(clk), .rst_n(rst_n),
        .I_de(v_de), .I_vsync(v_vsync), .I_hsync(1'b0),
        .I_user(v_user), .I_last(v_last),
        .I_rgb(v_rgb), .I_color_sel(`COLOR_SEL_OR),
        .O_mask(), .O_mask_r(), .O_mask_g(), .O_mask_b(),
        .O_y(b_y), .O_cb(b_cb), .O_cr(b_cr),
        .O_de(b_de), .O_vsync(), .O_hsync(), .O_user(), .O_last(),
        .O_pix_x(b_px), .O_pix_y(b_py),
        .O_frame_done()
    );

    //------------------------------------------------------------------
    // 比对：两者 O_de 同拍时逐位比较
    //------------------------------------------------------------------
    initial begin
        fout = $fopen("ycbcr_equiv_result.txt", "w");
        fail_cnt = 0;

        rst_n = 0; v_de = 0; v_user = 0; v_last = 0; v_vsync = 0; v_rgb = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        idx = 0;
        for (frame = 0; frame < 18; frame = frame + 1) begin
            // ---- 帧起始 ----
            @(negedge clk); v_user = 1; v_vsync = 1; v_de = 0;
            @(negedge clk); v_user = 0; v_vsync = 0;
            // ---- 逐行 ----
            for (row = 0; row < PH; row = row + 1) begin
                for (col = 0; col < PW; col = col + 1) begin
                    @(negedge clk);
                    v_de   = 1;
                    v_last = (col == PW-1);
                    v_rgb  = rgb_mem[idx % 272];
                    idx = idx + 1;
                end
            end
            @(negedge clk); v_de = 0; v_last = 0;
            // ---- 消隐 ----
            repeat (6) @(negedge clk);
        end

        repeat (20) @(negedge clk);
        $fclose(fout);

        if (fail_cnt == 0)
            $display("\n  [PASS] ycbcr_convert vs detect_color_mask: 逐位完全一致\n");
        else
            $display("\n  [FAIL] 共 %0d 处不一致\n", fail_cnt);

        $finish;
    end

    // 在每个 de 有效的输出拍比较
    always @(posedge clk) begin
        if (rst_n && a_de && b_de) begin
            if (a_y !== b_y || a_cb !== b_cb || a_cr !== b_cr ||
                a_px !== b_px || a_py !== b_py) begin
                fail_cnt = fail_cnt + 1;
                if (fail_cnt <= 20)
                    $display("  MISMATCH @(%0d,%0d): A(Y=%0d,Cb=%0d,Cr=%0d)  B(Y=%0d,Cb=%0d,Cr=%0d)",
                             a_px, a_py, a_y, a_cb, a_cr, b_y, b_cb, b_cr);
            end
            $fwrite(fout, "%0d %0d %0d %0d %0d %0d\n", a_px, a_py, a_y, a_cb, a_cr, a_y^b_y);
        end
    end

endmodule
