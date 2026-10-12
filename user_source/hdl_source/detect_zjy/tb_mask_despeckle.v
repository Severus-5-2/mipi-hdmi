//=============================================================================
//  tb_mask_despeckle.v —— mask_despeckle 自检（水平去斑 + 垂直投票 + 对齐）
//=============================================================================
//  验证三件事：
//    ① **水平去斑**：孤立单点被删；足够宽的实心块保留。
//    ② **垂直投票**：只出现 1 行的横条被删；连续多行的块保留。
//    ③ **对齐**：输出保留像素的包围盒必须与输入吻合（列/行都不能偏），
//              且 O_mask_r 必须是同一像素的颜色。
//
//  用法：
//    iverilog -g2005 -o t.vvp tb_mask_despeckle.v mask_despeckle.v \
//             ../line_delay_ram.v && vvp t.vvp
//=============================================================================

`timescale 1ns / 1ps

module tb_mask_despeckle;

    localparam integer HALF     = 4;
    localparam integer KMIN     = 5;
    localparam integer VER_ROWS = 3;
    localparam integer VER_NEED = 2;
    localparam integer DCH      = 2;

    // ---- 画面尺寸（小画面便于核对）----
    localparam integer NW = 64;      // 一行有效像素
    localparam integer NH = 10;      // 行数
    localparam integer BLK_X0 = 20, BLK_X1 = 39;   // 实心块列范围
    localparam integer BLK_Y0 = 3,  BLK_Y1 = 7;    // 实心块行范围

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg        I_de;
    reg        I_mask, I_mask_r, I_mask_g, I_mask_b, I_frame_done;
    reg [10:0] I_pix_x;
    reg [9:0]  I_pix_y;

    wire        O_de, O_mask, O_mask_r, O_mask_g, O_mask_b, O_frame_done;
    wire [10:0] O_pix_x;
    wire [9:0]  O_pix_y;

    mask_despeckle #(
        .HALF(HALF), .KMIN(KMIN), .VER_ROWS(VER_ROWS), .VER_NEED(VER_NEED),
        .DCH(DCH), .ENABLE(1)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .I_de(I_de), .I_mask(I_mask),
        .I_mask_r(I_mask_r), .I_mask_g(I_mask_g), .I_mask_b(I_mask_b),
        .I_frame_done(I_frame_done),
        .I_pix_x(I_pix_x), .I_pix_y(I_pix_y),
        .O_de(O_de), .O_mask(O_mask),
        .O_mask_r(O_mask_r), .O_mask_g(O_mask_g), .O_mask_b(O_mask_b),
        .O_frame_done(O_frame_done),
        .O_pix_x(O_pix_x), .O_pix_y(O_pix_y)
    );

    //------------------------------------------------------------------
    // 输入图样（整帧二维）
    //   ① 实心块      rows 3..7, cols 20..39      → 中央应保留
    //   ② 孤立单点    (row 2, col 55)             → 水平窗应删除
    //   ③ 单行横条    (row 1, cols 30..36)        → 垂直投票应删除
    //      ⚠ 特意放在**最上面一行**：它上下两行都空
    //        （若放在紧挨实心块隔一行处，2/3 投票会合理地把它当成块的一部分而保留）
    //      ⚠ 特意做成 **7 像素宽**（< WID-1=8）：更宽的话会触发 RTL 里的
    //        「水平窗几乎全满就无条件放行」保险条款，测不到垂直投票本身。
    //   ④ 孤立单点    (row 5, col 5)              → 应删除
    //------------------------------------------------------------------
    function automatic in_hit(input integer x, input integer y);
        begin
            in_hit = 1'b0;
            if (y >= BLK_Y0 && y <= BLK_Y1 && x >= BLK_X0 && x <= BLK_X1) in_hit = 1'b1;
            if (y == 5 && x == 5)  in_hit = 1'b1;
            if (y == 1 && x >= 30 && x <= 36) in_hit = 1'b1;
            if (y == 2 && x == 55) in_hit = 1'b1;
        end
    endfunction

    function automatic in_red(input integer x, input integer y);
        begin
            in_red = (y >= BLK_Y0 && y <= BLK_Y1 && x >= BLK_X0 && x <= BLK_X1);
        end
    endfunction

    //------------------------------------------------------------------
    // 采集
    //------------------------------------------------------------------
    integer kx_min, kx_max, ky_min, ky_max;
    integer n_keep, n_de;
    integer err_align, err_lone, err_row9, err_blk_core, err_coord;

    //------------------------------------------------------------------
    // 行缓存初始化（仿真专用）
    //   line_delay_ram 内部 mem 未初始化 ⇒ 前两行会读到 X，
    //   这里显式清零，避免 X 传播干扰判读。
    //------------------------------------------------------------------
    integer mi;
    initial begin
        for (mi = 0; mi < 1280; mi = mi + 1) begin
            tb_mask_despeckle.dut.u_lb_keep1.mem[mi] = 1'b0;
            tb_mask_despeckle.dut.u_lb_keep2.mem[mi] = 1'b0;
        end
    end

    integer x, y;
    integer sA_keep, sB_keep;

    task do_reset;
        begin
            rst_n = 0; I_de = 0; I_mask = 0; I_mask_r = 0; I_mask_g = 0;
            I_mask_b = 0; I_frame_done = 0; I_pix_x = 0; I_pix_y = 0;
            repeat (4) @(posedge clk);
            rst_n = 1;
            repeat (2) @(posedge clk);
        end
    endtask

    task clear_stats;
        begin
            kx_min = 100000; kx_max = 0; ky_min = 100000; ky_max = 0;
            n_keep = 0; n_de = 0;
            err_align = 0; err_lone = 0; err_row9 = 0; err_blk_core = 0; err_coord = 0;
        end
    endtask

    task drive_frame;
        begin
            for (y = 0; y < NH; y = y + 1) begin
                for (x = 0; x < NW; x = x + 1) begin
                    @(negedge clk);
                    I_de       = 1'b1;
                    I_pix_x    = x[10:0];
                    I_pix_y    = y[9:0];
                    I_mask     = in_hit(x, y);
                    I_mask_r   = in_red(x, y);
                    I_mask_g   = ~in_red(x, y);
                    I_mask_b   = 1'b0;
                    I_frame_done = (x == 0 && y == 0);
                end
                for (x = 0; x < 8; x = x + 1) begin
                    @(negedge clk);
                    I_de = 1'b0; I_mask = 1'b0; I_mask_r = 0; I_mask_g = 0;
                    I_pix_x = 11'd0; I_frame_done = 1'b0;
                end
            end
            @(negedge clk);
            I_de = 1'b0; I_mask = 1'b0; I_mask_r = 0; I_mask_g = 0;
        end
    endtask

    task report(input [255:0] tag);
        begin
            $display("");
            $display("  === 场景 %s ===", tag);
            $display("  O_de 有效拍数        : %0d  (期望 %0d)", n_de, NW*NH);
            $display("  保留像素数           : %0d", n_keep);
            $display("  保留像素包围盒       : x[%0d..%0d]  y[%0d..%0d]", kx_min, kx_max, ky_min, ky_max);
            $display("  实心块输入范围       : x[%0d..%0d]  y[%0d..%0d]", BLK_X0, BLK_X1, BLK_Y0, BLK_Y1);
            $display("  孤立点被保留(应0)    : %0d", err_lone);
            $display("  单行横条被保留(应0)  : %0d", err_row9);
            $display("  实心块核心被删(应0)  : %0d", err_blk_core);
            $display("  颜色错拍(应0)        : %0d", err_align);
            $display("  坐标越界(应0)        : %0d", err_coord);
        end
    endtask

    initial begin
        $dumpfile("tb_mask_despeckle.vcd");
        $dumpvars(0, tb_mask_despeckle);

        //==================================================================
        //  场景 A：正常工作
        //==================================================================
        clear_stats;
        do_reset;
        drive_frame;
        repeat (3 * (NW + 10) + 60) @(posedge clk);
        sA_keep = n_keep;
        report("A-NORMAL");
        if (!(err_lone == 0 && err_row9 == 0 && err_blk_core == 0 &&
              err_align == 0 && err_coord == 0 && n_keep > 0 &&
              kx_min >= BLK_X0 && kx_max <= BLK_X1 &&
              ky_min >= BLK_Y0 && ky_max <= BLK_Y1))
            $display("  [FAIL] 场景 A 未通过");

        //==================================================================
        //  场景 B：★ 强行把两个行缓存的输出拉 0，模拟"行缓存失效"
        //          验证「水平窗几乎全满就放行」的保险条款确实兜得住 ——
        //          目标核心必须仍在（不能出现"整片掩膜被清空"的全崩）。
        //==================================================================
        force tb_mask_despeckle.dut.K1_raw = 1'b0;
        force tb_mask_despeckle.dut.K2_raw = 1'b0;
        clear_stats;
        do_reset;
        drive_frame;
        repeat (3 * (NW + 10) + 60) @(posedge clk);
        sB_keep = n_keep;
        report("B-LINEBUF-FORCED-0");
        release tb_mask_despeckle.dut.K1_raw;
        release tb_mask_despeckle.dut.K2_raw;

        //==================================================================
        //  总结
        //==================================================================
        $display("");
        if (sA_keep > 0 && sB_keep > 0)
            $display("[PASS] mask_despeckle：A 正常 %0d px / B 行缓存失效仍保留 %0d px —— 全部通过",
                     sA_keep, sB_keep);
        else
            $display("[FAIL] mask_despeckle：A=%0d px  B=%0d px（B 应为 >0，否则保险条款没起作用）",
                     sA_keep, sB_keep);
        $finish;
    end

    always @(negedge clk) begin
        if (rst_n && O_de) begin
            n_de = n_de + 1;
            if (O_pix_x >= NW || O_pix_y >= NH) err_coord = err_coord + 1;
            if (O_mask) begin
                n_keep = n_keep + 1;
                if (O_pix_x < kx_min) kx_min = O_pix_x;
                if (O_pix_x > kx_max) kx_max = O_pix_x;
                if (O_pix_y < ky_min) ky_min = O_pix_y;
                if (O_pix_y > ky_max) ky_max = O_pix_y;

                if (O_pix_y == 5 && O_pix_x == 5)  err_lone = err_lone + 1;
                if (O_pix_y == 2 && O_pix_x == 55) err_lone = err_lone + 1;
                if (O_pix_y == 1) err_row9 = err_row9 + 1;

                if (O_mask_r !== (in_red(O_pix_x, O_pix_y) ? 1'b1 : 1'b0))
                    err_align = err_align + 1;
            end else begin
                if (O_pix_y >  BLK_Y0 + (HALF-1) && O_pix_y <  BLK_Y1 - (HALF-1) &&
                    O_pix_x >  BLK_X0 + (HALF+2) && O_pix_x <  BLK_X1 - (HALF+2))
                    err_blk_core = err_blk_core + 1;
            end
        end
    end

endmodule
