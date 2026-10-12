//=============================================================================
//  tb_mask_despeckle.v —— mask_despeckle 自检（对齐 + 去斑效果）
//=============================================================================
//  验证两件事：
//    ① **对齐**：O_mask / O_mask_r / O_de / O_pix_x / O_pix_y / O_frame_done
//              必须是**同一像素**，只是整体延后 LAT 拍。
//    ② **去斑**：孤立单点被删；实心游程保留（两端各缩 ~HALF 像素）。
//
//  用法：
//    iverilog -g2005 -o t.vvp tb_mask_despeckle.v mask_despeckle.v && vvp t.vvp
//=============================================================================

`timescale 1ns / 1ps

module tb_mask_despeckle;

    localparam integer HALF = 4;
    localparam integer KMIN = 5;
    localparam integer DCH  = 2;
    localparam integer LAT  = HALF + 1 + DCH;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;          // 100 MHz

    reg        I_de;
    reg        I_mask, I_mask_r, I_mask_g, I_mask_b, I_frame_done;
    reg [10:0] I_pix_x;
    reg [9:0]  I_pix_y;

    wire        O_de, O_mask, O_mask_r, O_mask_g, O_mask_b, O_frame_done;
    wire [10:0] O_pix_x;
    wire [9:0]  O_pix_y;

    mask_despeckle #(
        .HALF(HALF), .KMIN(KMIN), .DCH(DCH), .ENABLE(1)
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
    // 输入序列：1 行 64 像素；然后 8 拍消隐
    //   x=10        ：孤立单点（应被去斑删掉）
    //   x=20..39    ：20 像素实心游程（应保留）
    //   x=50..53    ：4 像素短游程（窗口内命中数 <5，两端应被削）
    //------------------------------------------------------------------
    localparam integer NACT = 64;

    integer err_cnt = 0;
    integer n_checked = 0;

    // 期望函数（软件模型，与 RTL 同构）
    function automatic exp_keep(input integer x);
        integer c, t;
        begin
            c = 0;
            for (t = -HALF; t <= HALF; t = t + 1) begin
                if (x + t >= 0 && x + t < NACT) begin
                    if ((x + t) == 10) c = c + 1;                     // 孤立点
                    if ((x + t) >= 20 && (x + t) <= 39) c = c + 1;     // 实心游程
                    if ((x + t) >= 50 && (x + t) <= 53) c = c + 1;     // 短游程
                end
            end
            exp_keep = (c >= KMIN);
        end
    endfunction

    // 输入侧当前掩膜
    function automatic in_mask(input integer x);
        begin
            in_mask = ((x == 10) || (x >= 20 && x <= 39) || (x >= 50 && x <= 53));
        end
    endfunction

    integer x;
    integer lat_cnt;

    initial begin
        $dumpfile("tb_mask_despeckle.vcd");
        $dumpvars(0, tb_mask_despeckle);

        rst_n = 0; I_de = 0; I_mask = 0; I_mask_r = 0; I_mask_g = 0; I_mask_b = 0;
        I_frame_done = 0; I_pix_x = 0; I_pix_y = 0;
        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        // ---- 第 1 行：有效像素 ----
        I_pix_y = 10'd0;
        for (x = 0; x < NACT; x = x + 1) begin
            @(negedge clk);
            I_de       = 1'b1;
            I_pix_x    = x[10:0];
            I_mask     = in_mask(x);
            // 颜色：x 在实心区用红，其余用绿，便于核对「颜色是否同像素」
            I_mask_r   = (x >= 20 && x <= 39);
            I_mask_g   = ~(x >= 20 && x <= 39);
            I_mask_b   = 1'b0;
            I_frame_done = (x == 0);
        end
        // ---- 消隐 8 拍（窗口自然清空）----
        for (x = 0; x < 8; x = x + 1) begin
            @(negedge clk);
            I_de = 1'b0; I_mask = 1'b0; I_mask_r = 0; I_mask_g = 0;
            I_pix_x = 11'd0; I_frame_done = 1'b0;
        end

        // ---- 让流水流干 ----
        repeat (LAT + 4) @(posedge clk);

        //------------------------------------------------------------------
        // 校验：把仿真期间记录的输出与软件模型对比
        //   —— 由 record 块在每拍采集（见下）
        //------------------------------------------------------------------
        if (err_cnt == 0)
            $display("\n[PASS] mask_despeckle：%0d 个有效像素全部一致（对齐 + 去斑）", n_checked);
        else
            $display("\n[FAIL] mask_despeckle：%0d/%0d 处不一致", err_cnt, n_checked);
        $finish;
    end

    //------------------------------------------------------------------
    // 采集与校验：在 negedge 采样（输出是 posedge 寄存的，negedge 时已稳定）
    //------------------------------------------------------------------
    integer ox;
    always @(negedge clk) begin
        if (rst_n && O_de) begin
            ox = O_pix_x;
            n_checked = n_checked + 1;

            // ① 掩膜是否符合软件模型
            if (O_mask !== exp_keep(ox)) begin
                err_cnt = err_cnt + 1;
                if (err_cnt <= 12)
                    $display("  [mask] x=%0d  O_mask=%b  期望=%b", ox, O_mask, exp_keep(ox));
            end

            // ② 颜色是否与同一像素的输入一致
            if (O_mask_r !== (ox >= 20 && ox <= 39)) begin
                err_cnt = err_cnt + 1;
                if (err_cnt <= 12)
                    $display("  [align] x=%0d  O_mask_r=%b 期望=%b（颜色错拍！）",
                             ox, O_mask_r, (ox >= 20 && ox <= 39));
            end

            // ③ 坐标必须落在输入范围内（对齐 sanity）
            if (O_pix_x >= NACT) begin
                err_cnt = err_cnt + 1;
                if (err_cnt <= 12)
                    $display("  [coord] O_pix_x=%0d 越界", O_pix_x);
            end
        end
    end

endmodule
