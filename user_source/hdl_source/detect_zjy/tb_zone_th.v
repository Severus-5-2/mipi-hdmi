//=============================================================================
//  tb_zone_th.v —— 位置自适应门限（渐晕/边缘色度补偿）自检
//=============================================================================
//  验证对象：detect_color_mask 的 3×3 九宫格门限放宽（MAGD_EDGE / MAGD_CORNER）
//
//  规则（本 TB 参数：MAG_TH=24, MAGD_EDGE=6, MAGD_CORNER=10）：
//    中心区（384≤x<896 且 216≤y<504）：命中条件 mag ≥ 24
//    边中区（x 在中心带 或 y 在中心带） ：命中条件 mag ≥ 24−6  = 18
//    四角区（x、y 都在中心带外）      ：命中条件 mag ≥ 24−10 = 14
//
//  做法：先量出该目标色的真实 mag（从 O_cb/O_cr 反算），再对三个区分别断言 ——
//        "模型 ↔ RTL"同源，不手算 YCbCr。
//
//  运行： iverilog -g2005 -o tb_zone_th.vvp tb_zone_th.v detect_color_mask.v
//         vvp tb_zone_th.vvp
//=============================================================================
`timescale 1ns/1ps

module tb_zone_th;

    localparam integer MAG_TH_X    = 24;
    localparam integer MAGD_EDGE   = 6;
    localparam integer MAGD_CORNER = 10;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [23:0] rgb;
    reg         de;
    reg  [10:0] px;
    reg  [9:0]  py;

    wire        o_mask, o_mr, o_mg, o_mb;
    wire [7:0]  o_y, o_cb, o_cr;
    wire        o_de, o_vsync, o_hsync, o_user, o_last;
    wire [10:0] o_px;
    wire [9:0]  o_py;
    wire        o_fd;

    detect_color_mask #(
        .IMG_WIDTH   (1280       ),
        .IMG_HEIGHT  (720        ),
        .PIPE_DELAY  (0          ),
        .PIX_DLY     (0          ),
        .MAG_TH      (MAG_TH_X   ),
        .NIGHT_EN    (0          ),          // 关掉夜间档，单独验分区
        .MAGD_EDGE   (8'd6       ),
        .MAGD_CORNER (8'd10      )
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .I_de(de), .I_vsync(1'b0), .I_hsync(1'b0), .I_user(1'b0), .I_last(1'b0),
        .I_rgb(rgb), .I_pix_x(px), .I_pix_y(py),
        .I_color_sel(2'd0),                  // 红
        .I_y_avg(8'd145), .I_frame_toggle(1'b0),
        .O_mask(o_mask), .O_mask_r(o_mr), .O_mask_g(o_mg), .O_mask_b(o_mb),
        .O_y(o_y), .O_cb(o_cb), .O_cr(o_cr),
        .O_de(o_de), .O_vsync(o_vsync), .O_hsync(o_hsync), .O_user(o_user), .O_last(o_last),
        .O_pix_x(o_px), .O_pix_y(o_py), .O_frame_done(o_fd)
    );

    reg cap_en = 0;
    reg m_obs;  reg [7:0] cb_obs, cr_obs;
    always @(posedge clk) begin
        if (cap_en && o_de) begin
            m_obs <= o_mask; cb_obs <= o_cb; cr_obs <= o_cr;
        end
    end

    integer n_ok = 0, n_bad = 0;
    integer mag_obs, exp_ok;

    task probe;
        input [10:0] X;
        input [9:0]  Y;
        input [23:0] C;
        begin
            de = 1; rgb = C; px = X; py = Y;
            repeat (6) @(negedge clk);
            cap_en = 1;
            repeat (3) @(negedge clk);
            cap_en = 0; de = 0;
            repeat (2) @(negedge clk);
        end
    endtask

    task calc_mag;
        begin
            mag_obs = (cb_obs > 8'd128 ? cb_obs - 8'd128 : 8'd128 - cb_obs) +
                      (cr_obs > 8'd128 ? cr_obs - 8'd128 : 8'd128 - cr_obs);
        end
    endtask

    task expect;
        input integer relax;
        begin
            exp_ok = (mag_obs + relax) >= MAG_TH_X;
            if (m_obs === exp_ok[0]) n_ok = n_ok + 1;
            else begin n_bad = n_bad + 1;
                $display("       NG: mag=%0d relax=%0d 期望=%b 实际=%b", mag_obs, relax, exp_ok[0], m_obs);
            end
        end
    endtask

    integer i;

    initial begin
        de = 0; rgb = 24'h000000; px = 0; py = 0;
        repeat (4) @(negedge clk); rst_n = 1'b1; repeat (4) @(negedge clk);

        $display("============================================================");
        $display(" tb_zone_th —— 位置自适应门限自检");
        $display("   MAG_TH=%0d  MAGD_EDGE=%0d  MAGD_CORNER=%0d", MAG_TH_X, MAGD_EDGE, MAGD_CORNER);
        $display("   中心带 x=[384,896)  y=[216,504)");
        $display("============================================================");

        probe(11'd640, 10'd360, 24'h9C8080);
        calc_mag;
        $display("\n  目标色 R=156 G=128 B=128 → Cb=%0d Cr=%0d  mag=%0d", cb_obs, cr_obs, mag_obs);
        if ((mag_obs >= (MAG_TH_X - MAGD_CORNER)) && (mag_obs < MAG_TH_X))
            begin n_ok = n_ok + 1;
                  $display("  [OK ] 该 mag=%0d 落在 [%0d,%0d)，正好能区分三个区（用例有效）",
                           mag_obs, MAG_TH_X - MAGD_CORNER, MAG_TH_X); end
        else
            begin n_bad = n_bad + 1;
                  $display("  [NG ] mag=%0d 不在 [%0d,%0d)，本用例无法区分三区",
                           mag_obs, MAG_TH_X - MAGD_CORNER, MAG_TH_X); end

        $display("\n  [1] 中心区 (640,360) —— 放宽 0");
        probe(11'd640, 10'd360, 24'h9C8080); expect(0);
        $display("  [OK ] 中心区：mag=%0d → mask=%b（期望 %b）", mag_obs, m_obs, exp_ok[0]);

        $display("\n  [2] 边中区 —— 水平边 (100,360) / 垂直边 (640,80) —— 放宽 %0d", MAGD_EDGE);
        probe(11'd100, 10'd360, 24'h9C8080); expect(MAGD_EDGE);
        probe(11'd640, 10'd80,  24'h9C8080); expect(MAGD_EDGE);
        $display("  [OK ] 边中区两处：mag=%0d → mask=%b（期望 %b）", mag_obs, m_obs, exp_ok[0]);

        $display("\n  [3] 四角区 (60,60)/(1220,60)/(60,660)/(1220,660) —— 放宽 %0d", MAGD_CORNER);
        probe(11'd60,   10'd60,  24'h9C8080); expect(MAGD_CORNER);
        probe(11'd1220, 10'd60,  24'h9C8080); expect(MAGD_CORNER);
        probe(11'd60,   10'd660, 24'h9C8080); expect(MAGD_CORNER);
        probe(11'd1220, 10'd660, 24'h9C8080); expect(MAGD_CORNER);
        $display("  [OK ] 四角区四处：mag=%0d → mask=%b（期望 %b）", mag_obs, m_obs, exp_ok[0]);

        $display("\n  [4] 边界内侧 —— 中心区第一列 (384,360) 与中心区外一列 (383,360)");
        probe(11'd384, 10'd360, 24'h9C8080); expect(0);              // 384 属中心区
        probe(11'd383, 10'd360, 24'h9C8080); expect(MAGD_EDGE);      // 383 属边中区
        $display("  [OK ] x=384 归中心、x=383 归边中（边界正确）");

        $display("\n  [5] 中性灰在四角也不该被检出（放宽不等于放行灰色）");
        probe(11'd60, 10'd660, 24'h8C8C8C); calc_mag;
        if ((m_obs === 1'b0) && (mag_obs < (MAG_TH_X - MAGD_CORNER)))
            begin n_ok = n_ok + 1;
                  $display("  [OK ] 四角灰 mag=%0d 仍不误检（mask=%b）", mag_obs, m_obs); end
        else
            begin n_bad = n_bad + 1;
                  $display("  [NG ] 四角灰 mag=%0d mask=%b", mag_obs, m_obs); end

        $display("\n============================================================");
        if (n_bad == 0) $display("  [PASS] tb_zone_th：%0d 项全部通过", n_ok);
        else            $display("  [FAIL] tb_zone_th：%0d 项中 %0d 项不通过", n_ok + n_bad, n_bad);
        $display("============================================================");
        $finish;
    end

endmodule
