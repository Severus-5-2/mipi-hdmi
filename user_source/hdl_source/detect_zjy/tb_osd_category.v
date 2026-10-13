//=============================================================================
//  tb_osd_category.v —— osd_category 位级自检（含点阵渲染）
//=============================================================================
//  构造一组目标（颜色：红1 蓝2 绿1 ；形状：方1 圆2 三角1），
//  发帧末脉冲后，把 OSD 区域的 O_hit 收集成点阵并打印成 ASCII，
//  同时以字符串逐格比对字符码。
//=============================================================================
`timescale 1ns / 1ps
`include "common_defs.vh"

module tb_osd_category;

    localparam integer SW  = `TGT_SLOT_W;    // 96
    localparam integer X0  = 16;
    localparam integer Y0  = 16;
    localparam integer COLS= 11;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [`TGT_BUS_W-1:0] I_bus;
    reg  [7:0]            I_shapes;
    reg  [2:0]            I_valid_cnt;
    reg                   I_frame_done;
    reg                   I_de;
    reg  [10:0]           I_pix_x;
    reg  [9:0]            I_pix_y;

    wire O_hit;
    wire [23:0] O_color;

    osd_category dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .I_de         (I_de),
        .I_pix_x      (I_pix_x),
        .I_pix_y      (I_pix_y),
        .I_bus        (I_bus),
        .I_shapes     (I_shapes),
        .I_valid_cnt  (I_valid_cnt),
        .I_frame_done (I_frame_done),
        .O_hit        (O_hit),
        .O_color      (O_color)
    );

    //---- 位图（2 行 × COLS 格 × 16 像素）----
    reg bmp [0:1][0:COLS*16-1][0:15];
    integer cx, cy, px, py, ii;
    integer err_cnt;

    task set_slot_cls;
        input integer k;
        input [1:0]   c;
        begin
            I_bus[`TGT_O_VALID + k*SW]     = 1'b1;
            I_bus[`TGT_O_CLS + k*SW +: 2]  = c;
        end
    endtask

    //---- 逐像素扫描区域，收集 O_hit ----
    task scan_area;
        integer yy, xx;
        begin
            for (yy = 0; yy < 32; yy = yy + 1) begin
                for (xx = 0; xx < COLS*16; xx = xx + 1) begin
                    @(negedge clk);
                    I_de    = 1'b1;
                    I_pix_x = X0 + xx;
                    I_pix_y = Y0 + yy;
                    @(negedge clk);     // O_hit 相对 I_de 晚 2 拍：再等 2 拍后采样
                    @(negedge clk);
                    bmp[yy/16][xx][yy%16] = O_hit;
                end
            end
            @(negedge clk);
            I_de = 1'b0;
        end
    endtask

    task render;
        integer r, c, i;
        begin
            for (r = 0; r < 2; r = r + 1) begin
                for (i = 0; i < 16; i = i + 1) begin
                    $write("  ");
                    for (c = 0; c < COLS; c = c + 1) begin
                        for (px = 0; px < 16; px = px + 1)
                            $write("%s", bmp[r][c*16+px][i] ? "#" : ".");
                        $write(" ");
                    end
                    $write("\n");
                end
                $write("\n");
            end
        end
    endtask

    initial begin
        $dumpfile("tb_osd_category.vcd");
        $dumpvars(0, tb_osd_category);

        err_cnt = 0;
        I_bus = {`TGT_BUS_W{1'b0}};
        I_shapes = 8'hFF;            // 默认全"未知"(3)
        I_valid_cnt = 3'd0;
        I_frame_done = 1'b0;
        I_de = 1'b0; I_pix_x = 11'd0; I_pix_y = 10'd0;

        for (cy = 0; cy < 2; cy = cy + 1)
            for (cx = 0; cx < COLS*16; cx = cx + 1)
                for (py = 0; py < 16; py = py + 1)
                    bmp[cy][cx][py] = 1'b0;

        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        //---- 构造目标：cls = 红/蓝/蓝/绿 ；shape = 方/圆/圆/三角 ----
        set_slot_cls(0, `CLS_RED);
        set_slot_cls(1, `CLS_BLUE);
        set_slot_cls(2, `CLS_BLUE);
        set_slot_cls(3, `CLS_GREEN);
        // shape：槽0=方(0) 槽1=圆(1) 槽2=圆(1) 槽3=三角(2)
        I_shapes = {2'd2, 2'd1, 2'd1, 2'd0};
        I_valid_cnt = 3'd4;

        //---- 帧末脉冲 → 触发统计 ----
        @(negedge clk);
        I_frame_done = 1'b1;
        @(negedge clk);
        I_frame_done = 1'b0;
        repeat (4) @(posedge clk);

        //---- 扫描 OSD 区域 ----
        scan_area();

        $display("");
        $display("== osd_category 点阵渲染 ==");
        $display("（期望：第 1 行 R:1 B:2 G:1 ；第 2 行 S:1 C:2 T:1）");
        $display("");
        render();

        //---- 校验几个关键字符的字符码是否正确（用 DUT 内部信号 ch 抽样）----
        if (dut.cnt_red !== 4'd1 || dut.cnt_blue !== 4'd2 ||
            dut.cnt_green !== 4'd1 || dut.cnt_sq !== 4'd1 ||
            dut.cnt_cir !== 4'd2 || dut.cnt_tri !== 4'd1) begin
            err_cnt = err_cnt + 1;
            $display("[FAIL] 计数错误：R=%0d B=%0d G=%0d / S=%0d C=%0d T=%0d（期望 1 2 1 / 1 2 1）",
                     dut.cnt_red, dut.cnt_blue, dut.cnt_green,
                     dut.cnt_sq, dut.cnt_cir, dut.cnt_tri);
        end else begin
            $display("[OK]   计数：R=%0d B=%0d G=%0d / S=%0d C=%0d T=%0d",
                     dut.cnt_red, dut.cnt_blue, dut.cnt_green,
                     dut.cnt_sq, dut.cnt_cir, dut.cnt_tri);
        end

        //---- 检查每格命中数：第 3 / 7 格是空白分隔，其余格必须有内容 ----
        for (cy = 0; cy < 2; cy = cy + 1) begin
            for (cx = 0; cx < COLS; cx = cx + 1) begin
                px = 0;
                for (py = 0; py < 16; py = py + 1)
                    for (ii = 0; ii < 16; ii = ii + 1)
                        if (bmp[cy][cx*16+ii][py]) px = px + 1;
                if (cx == 3 || cx == 7) begin
                    if (px != 0) begin
                        err_cnt = err_cnt + 1;
                        $display("[FAIL] 行%0d 第%0d 格（应为空白分隔）却有 %0d 个点", cy, cx, px);
                    end
                end else if (px == 0) begin
                    err_cnt = err_cnt + 1;
                    $display("[FAIL] 行%0d 第%0d 格（应有字符）却是空的", cy, cx);
                end
            end
        end

        $display("");
        if (err_cnt == 0)
            $display("[PASS] osd_category：计数与 22 个字符格全部正确");
        else
            $display("[FAIL] osd_category：%0d 处不一致", err_cnt);
        $display("");
        $finish;
    end

endmodule
