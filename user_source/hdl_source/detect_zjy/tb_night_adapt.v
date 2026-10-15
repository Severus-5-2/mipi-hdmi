//=============================================================================
//  tb_night_adapt.v —— 夜间（低照度）自适应门限 自检
//=============================================================================
//  验证对象：detect_color_mask 的 ★2026-10-15 夜间自适应（NIGHT_EN=1）
//
//  覆盖：
//    ① 档位表 —— 按 ae_meter 帧平均亮度 I_y_avg 分 4 档，MAG_TH 24/20/16/12
//    ② 滞回   —— 亮度在档位边界 ±HYST 内抖动时**不换档**
//    ③ 行为   —— 拿一个「mag 落在 16~24 之间」的目标：
//                 亮场（档 0，th=24）不检出；暗场（档 1/2/3）检出
//                 ⇒ 直接证明"暗场目标不再因为够不着固定门限而丢检"
//    ④ 反向保护 —— 极暗档下中性灰（mag≈0）仍不误检
//
//  ⚠ 注意：档位状态机**一帧只走一档**（有意为之，防止亮度突变时门限跳变）；
//    所以用例里要先空跑几帧把档位"接力"到目标档，再验行为。
//  ⚠ iverilog 的 $display 把**参数里的中文**会乱码（字面量里的正常）——
//    故断言文案用 case(id) 分支直接 $display 字面量。
//
//  运行： iverilog -g2005 -o tb_night_adapt.vvp tb_night_adapt.v detect_color_mask.v
//         vvp tb_night_adapt.vvp
//=============================================================================
`timescale 1ns/1ps

module tb_night_adapt;

    localparam integer W = 8, H = 8;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [7:0]  y_avg;
    reg         tog;
    reg  [23:0] rgb;
    reg         de;
    reg  [10:0] px;
    reg  [9:0]  py;

    wire        o_mask, o_mask_r, o_mask_g, o_mask_b;
    wire [7:0]  o_y, o_cb, o_cr;
    wire        o_de, o_vsync, o_hsync, o_user, o_last;
    wire [10:0] o_px;
    wire [9:0]  o_py;
    wire        o_fdone;

    detect_color_mask #(
        .IMG_WIDTH   (W        ),
        .IMG_HEIGHT  (H        ),
        .PIPE_DELAY  (0        ),
        .PIX_DLY     (0        ),
        .NIGHT_EN    (1        )        // ★ 打开夜间自适应
    ) dut (
        .clk            (clk      ),
        .rst_n          (rst_n    ),
        .I_de           (de       ),
        .I_vsync        (1'b0     ),
        .I_hsync        (1'b0     ),
        .I_user         (1'b0     ),
        .I_last         (1'b0     ),
        .I_rgb          (rgb      ),
        .I_pix_x        (px       ),
        .I_pix_y        (py       ),
        .I_color_sel    (2'd0     ),     // 红
        .I_y_avg        (y_avg    ),
        .I_frame_toggle (tog      ),
        .O_mask         (o_mask   ),
        .O_mask_r       (o_mask_r ),
        .O_mask_g       (o_mask_g ),
        .O_mask_b       (o_mask_b ),
        .O_y            (o_y      ),
        .O_cb           (o_cb     ),
        .O_cr           (o_cr     ),
        .O_de           (o_de     ),
        .O_vsync        (o_vsync  ),
        .O_hsync        (o_hsync  ),
        .O_user         (o_user   ),
        .O_last         (o_last   ),
        .O_pix_x        (o_px     ),
        .O_pix_y        (o_py     ),
        .O_frame_done   (o_fdone  )
    );

    //------------------------------------------------------------------
    //  采样器：在 O_de 有效期内抓一个像素的 CB/CR/mask
    //------------------------------------------------------------------
    reg        cap_en = 0;
    reg [7:0]  m_obs, cb_obs, cr_obs;

    always @(posedge clk) begin
        if (cap_en && o_de) begin
            m_obs  <= o_mask;
            cb_obs <= o_cb;
            cr_obs <= o_cr;
        end
    end

    integer n_ok = 0, n_bad = 0;
    integer id;                                  // 断言编号（用于中文文案分派）

    task chk;
        input integer i_id;
        input         cond;
        begin
            if (cond) n_ok = n_ok + 1; else begin n_bad = n_bad + 1; $display("       ^^^ 本条 NG"); end
            case (i_id)
              1: $display("  [%0s] Y=145 → 档0 / th=24",              cond?"OK ":"NG ");
              2: $display("  [%0s] Y=100 → 档1 / th=20（100<106）",     cond?"OK ":"NG ");
              3: $display("  [%0s] Y=110 边界内 → 仍档1 / th=20（滞回）", cond?"OK ":"NG ");
              4: $display("  [%0s] Y=118 → 升回档0 / th=24",            cond?"OK ":"NG ");
              5: $display("  [%0s] Y=107 边界内 → 仍档0 / th=24（滞回）", cond?"OK ":"NG ");
              6: $display("  [%0s] Y=105 → 档1 / th=20",                cond?"OK ":"NG ");
              7: $display("  [%0s] Y=73  → 档2 / th=16（73<74）",       cond?"OK ":"NG ");
              8: $display("  [%0s] Y=70 边界内 → 仍档2 / th=16（滞回）", cond?"OK ":"NG ");
              9: $display("  [%0s] Y=41  → 档3 / th=12（41<42）",       cond?"OK ":"NG ");
             10: $display("  [%0s] Y=48 边界内 → 仍档3 / th=12（滞回）", cond?"OK ":"NG ");
             11: $display("  [%0s] Y=54  → 升回档2 / th=16",            cond?"OK ":"NG ");
             12: $display("  [%0s] Y=86  → 升回档1 / th=20",            cond?"OK ":"NG ");
             13: $display("  [%0s] Y=0（自档1）→ 一帧只降一档到档2",     cond?"OK ":"NG ");
             14: $display("  [%0s] Y=0（再一帧）→ 档3 / th=12",         cond?"OK ":"NG ");
             15: $display("  [%0s] 该目标 mag 确实落在 (16,24)，用例有效", cond?"OK ":"NG ");
             16: $display("  [%0s] 亮场档0(th=%0d)：该目标【不检出】",   cond?"OK ":"NG ", th_obs);
             17: $display("  [%0s] 暗场档1(th=%0d)：该目标【检出】",     cond?"OK ":"NG ", th_obs);
             18: $display("  [%0s] 暗场档2(th=%0d)：该目标【检出】",     cond?"OK ":"NG ", th_obs);
             19: $display("  [%0s] 暗场档3(th=%0d)：该目标【检出】",     cond?"OK ":"NG ", th_obs);
             20: $display("  [%0s] 极暗档3 中性灰(mag=%0d)仍不误检",    cond?"OK ":"NG ", mag_obs);
             default: $display("  [%0s] 断言 %0d", cond?"OK ":"NG ", i_id);
            endcase
        end
    endtask

    //------------------------------------------------------------------
    //  档位读数 / 跑帧
    //------------------------------------------------------------------
    reg [7:0] band_obs;
    reg [7:0] th_obs;
    integer   i;
    integer   mag_obs;

    task run_frame_band;                  // 只关心档位表：不送像素
        input [7:0] y;
        begin
            y_avg = y;
            @(negedge clk);
            tog = ~tog;                   // 帧握手翻转（模块检沿后延 1 拍采样）
            @(negedge clk);
            repeat (4) @(negedge clk);    // 留足时间让档位状态机更新
            band_obs = dut.r_band;
            th_obs   = dut.r_mag_th;
        end
    endtask

    task run_frame;                       // 档位 + 送一帧同色像素
        input [7:0]  y;
        input [23:0] c;
        begin
            y_avg = y;
            @(negedge clk);
            tog = ~tog;
            @(negedge clk);
            repeat (4) @(negedge clk);
            band_obs = dut.r_band;
            th_obs   = dut.r_mag_th;

            cap_en = 1;
            de = 1; rgb = c; py = 10'd0;
            for (i = 0; i < W + 8; i = i + 1) begin
                px = i[10:0];
                @(negedge clk);
            end
            de = 0; cap_en = 0;
            @(negedge clk);
        end
    endtask

    task calc_mag;
        begin
            mag_obs = (cb_obs > 8'd128 ? cb_obs - 8'd128 : 8'd128 - cb_obs) +
                      (cr_obs > 8'd128 ? cr_obs - 8'd128 : 8'd128 - cr_obs);
        end
    endtask

    initial begin
        y_avg = 8'd145; tog = 1'b0; rgb = 24'h000000; de = 0; px = 0; py = 0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (4) @(negedge clk);

        $display("============================================================");
        $display(" tb_night_adapt —— 夜间自适应门限自检（NIGHT_EN=1）");
        $display("  档位边界：Y_avg>=112->0 / >=80->1 / >=48->2 / else->3    HYST=6");
        $display("============================================================");

        $display("\n[1] 档位表 / 滞回");
        run_frame_band(8'd145); chk( 1, (band_obs == 0) && (th_obs == 24));
        run_frame_band(8'd100); chk( 2, (band_obs == 1) && (th_obs == 20));
        run_frame_band(8'd110); chk( 3, (band_obs == 1) && (th_obs == 20));
        run_frame_band(8'd118); chk( 4, (band_obs == 0) && (th_obs == 24));
        run_frame_band(8'd107); chk( 5, (band_obs == 0) && (th_obs == 24));
        run_frame_band(8'd105); chk( 6, (band_obs == 1) && (th_obs == 20));
        run_frame_band(8'd73);  chk( 7, (band_obs == 2) && (th_obs == 16));
        run_frame_band(8'd70);  chk( 8, (band_obs == 2) && (th_obs == 16));
        run_frame_band(8'd41);  chk( 9, (band_obs == 3) && (th_obs == 12));
        run_frame_band(8'd48);  chk(10, (band_obs == 3) && (th_obs == 12));
        run_frame_band(8'd54);  chk(11, (band_obs == 2) && (th_obs == 16));
        run_frame_band(8'd86);  chk(12, (band_obs == 1) && (th_obs == 20));
        run_frame_band(8'd0);   chk(13, (band_obs == 2) && (th_obs == 16));
        run_frame_band(8'd0);   chk(14, (band_obs == 3) && (th_obs == 12));

        //  把档位"接力"回档 0（一帧一档：3->2->1->0）
        run_frame_band(8'd145);
        run_frame_band(8'd145);
        run_frame_band(8'd145);
        $display("  （已把档位接力回档0：当前 band=%0d th=%0d）", band_obs, th_obs);

        //--------------------------------------------------------------
        //  ② 行为：mag 落在 16~24 之间的目标
        //--------------------------------------------------------------
        $display("\n[2] 行为验证 —— 固定目标色 R=160 G=128 B=128");
        run_frame(8'd145, 24'hA08080); calc_mag;
        $display("  目标实测：Cb=%0d Cr=%0d -> mag = %0d  （当前档%0d / th=%0d）",
                 cb_obs, cr_obs, mag_obs, band_obs, th_obs);
        chk(15, (mag_obs > 16) && (mag_obs < 24));
        $display("  该帧 mask=%b", m_obs);
        chk(16, (m_obs === 1'b0));

        run_frame(8'd100, 24'hA08080); calc_mag;
        $display("  该帧 mask=%b  （档%0d / th=%0d）", m_obs, band_obs, th_obs);
        chk(17, (m_obs === 1'b1));

        run_frame(8'd73, 24'hA08080);
        $display("  该帧 mask=%b  （档%0d / th=%0d）", m_obs, band_obs, th_obs);
        chk(18, (m_obs === 1'b1));

        run_frame(8'd41, 24'hA08080);
        $display("  该帧 mask=%b  （档%0d / th=%0d）", m_obs, band_obs, th_obs);
        chk(19, (m_obs === 1'b1));

        //--------------------------------------------------------------
        //  ③ 反向保护：中性灰在极暗档也不该被检出
        //--------------------------------------------------------------
        $display("\n[3] 反向保护 —— 近中性灰 R=G=B=140（mag 很小）");
        run_frame(8'd41, 24'h8C8C8C); calc_mag;
        $display("  灰实测：Cb=%0d Cr=%0d -> mag=%0d   mask=%b  （档%0d / th=%0d）",
                 cb_obs, cr_obs, mag_obs, m_obs, band_obs, th_obs);
        chk(20, (m_obs === 1'b0) && (mag_obs < 12));

        $display("\n============================================================");
        if (n_bad == 0)
            $display("  [PASS] tb_night_adapt：%0d 项全部通过（档位表/滞回/行为/反向保护）", n_ok);
        else
            $display("  [FAIL] tb_night_adapt：%0d 项中 %0d 项不通过", n_ok + n_bad, n_bad);
        $display("============================================================");
        $finish;
    end

endmodule
