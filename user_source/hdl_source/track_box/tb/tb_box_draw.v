//=====================================================================
//  tb_box_draw.v  ——  pix_coord_gen + box_draw 自检 testbench
//---------------------------------------------------------------------
//  仿真器：Icarus Verilog 12.0（本机 D:\iverilog\bin\iverilog 实测通过）
//
//      cd user_source/hdl_source/track_box
//      iverilog -g2005 -o sim/tb_box_draw.vvp \
//               tb/tb_box_draw.v rtl/box_draw.v rtl/pix_coord_gen.v
//      vvp sim/tb_box_draw.vvp
//
//  为什么用小画面
//    box_draw 的逻辑只依赖传入的坐标，与画面大小无关。用 64x32 有效区 /
//    80x40 整帧，仿真压到毫秒级、便于反复跑。1280x720 的 SELF_TEST 固定框
//    屏幕另有 sim/bit_sim_box_draw.py 出 ASCII 预览。
//
//  测什么
//    1) pix_coord_gen：de 有效时 O_pix_x/O_pix_y 是否恒等于真实 (列, 行)
//       —— 这是 D1「验证坐标系统」的可执行证据；
//    2) box_draw：O_hit/O_color 是否与 TB 内独立写成的期望模型逐像素一致；
//       并把整帧叠加 dump 到 sim/tb_box_draw_out.txt，
//       交由 sim/bit_sim_box_draw.py 做第三方比对（三份实现互证）。
//
//  关于"期望模型"的写法
//    本 TB 不手写任何常量期望值，全部由 exp_color() 现算。exp_color 用
//    朴素的条件分支按【规格】写，刻意不与 RTL 的并行 wire 表达式同构，
//    避免"两份代码同错"。项目教训（README 第 7 节）是不要手写"应有"的
//    公式去对账 RTL —— 这里既没有手写常量，另外还有 Python 侧独立实现。
//=====================================================================
`timescale 1ns / 1ps

module tb_box_draw;

    //==================================================================
    // 0. 画面参数（H_ACTIVE / V_ACTIVE 必须与 pix_coord_gen 实例化一致）
    //==================================================================
    localparam integer H_ACTIVE = 64;
    localparam integer H_FRAME  = 80;
    localparam integer V_ACTIVE = 32;
    localparam integer V_FRAME  = 40;
    localparam integer N_FRAMES = 3;

    // ---- 目标框（SELF_TEST=0，走总线通路）----
    localparam integer B0_X0 = 4,  B0_Y0 = 4,  B0_X1 = 20, B0_Y1 = 14;
    localparam integer B1_X0 = 40, B1_Y0 = 4,  B1_X1 = 63, B1_Y1 = 14;  // 贴右边
    localparam integer B2_X0 = 4,  B2_Y0 = 20, B2_X1 = 20, B2_Y1 = 31;  // 贴下边
    localparam integer B3_X0 = 28, B3_Y0 = 12, B3_X1 = 44, B3_Y1 = 24;  // 框心 (36,18)
    localparam integer CR_X  = 36, CR_Y  = 18;
    localparam integer CR_HALF = 5;

    localparam [23:0] C_BOX0  = 24'hff0000;
    localparam [23:0] C_BOX1  = 24'h00ff00;
    localparam [23:0] C_BOX2  = 24'h00a0ff;
    localparam [23:0] C_BOX3  = 24'hfff200;
    localparam [23:0] C_CROSS = 24'hffffff;

    //==================================================================
    // 1. 时钟 / 复位
    //==================================================================
    reg S_clk   = 0;
    reg S_rst_n = 0;
    always #5 S_clk = ~S_clk;

    initial begin
        S_rst_n = 0;
        repeat(4) @(posedge S_clk);
        S_rst_n = 1;
    end

    //==================================================================
    // 2. 视频时序（直接生成干净的 de/last/user/vsync）
    //    ★计数器与 DUT 同步复位，保证两边从同一起点开始计数，
    //      这样就不会像 uivtc 那样在复位窗口里多灌出伪像素。
    //==================================================================
    reg [11:0] c_hcnt = 12'd0;
    reg [11:0] c_vcnt = 12'd0;

    always @(posedge S_clk) begin
        if(!S_rst_n) begin
            c_hcnt <= 12'd0;
            c_vcnt <= 12'd0;
        end
        else if(c_hcnt == H_FRAME - 1) begin
            c_hcnt <= 12'd0;
            c_vcnt <= (c_vcnt == V_FRAME - 1) ? 12'd0 : (c_vcnt + 12'd1);
        end
        else begin
            c_hcnt <= c_hcnt + 12'd1;
        end
    end

    wire W_de    = (c_hcnt < H_ACTIVE) && (c_vcnt < V_ACTIVE);
    wire W_last  = (c_hcnt == H_ACTIVE - 1) && (c_vcnt < V_ACTIVE);
    wire W_user  = (c_hcnt == 0) && (c_vcnt == 0);
    // vsync 高电平落在有效行之后的消隐区（供 pix_coord_gen 取上升沿归零）
    wire W_vsync = (c_vcnt > (V_ACTIVE + 2)) && (c_vcnt <= (V_ACTIVE + 6));

    wire [10:0] W_true_x = c_hcnt[10:0];
    wire [9:0]  W_true_y = c_vcnt[9:0];

    //==================================================================
    // 3. DUT-A：pix_coord_gen
    //==================================================================
    wire [10:0] S_pix_x, S_pix_x_1d, S_pix_x_2d;
    wire [9:0]  S_pix_y, S_pix_y_1d, S_pix_y_2d;
    wire        S_frame_done;

    pix_coord_gen #(
        .H_ACTIVE ( H_ACTIVE ),
        .V_ACTIVE ( V_ACTIVE )
    ) u_pix_coord_gen (
        .I_clk        ( S_clk        ),
        .I_rst_n      ( S_rst_n      ),
        .I_de         ( W_de         ),
        .I_last       ( W_last       ),
        .I_vsync      ( W_vsync      ),
        .O_pix_x      ( S_pix_x      ),
        .O_pix_y      ( S_pix_y      ),
        .O_pix_x_1d   ( S_pix_x_1d   ),
        .O_pix_x_2d   ( S_pix_x_2d   ),
        .O_pix_y_1d   ( S_pix_y_1d   ),
        .O_pix_y_2d   ( S_pix_y_2d   ),
        .O_frame_done ( S_frame_done )
    );

    //==================================================================
    // 4. DUT-B：box_draw
    //    I_de 用 W_de（与 S_pix_x 同拍），与顶层里 pix_coord_gen 的
    //    使用方式一致。
    //==================================================================
    reg [3:0]      S_box_valid = 4'b1111;
    reg [4*11-1:0] S_box_xmin  = {11'd28, 11'd4,  11'd40, 11'd4 };
    reg [4*11-1:0] S_box_xmax  = {11'd44, 11'd20, 11'd63, 11'd20};
    reg [4*10-1:0] S_box_ymin  = {10'd12, 10'd20, 10'd4,  10'd4 };
    reg [4*10-1:0] S_box_ymax  = {10'd24, 10'd31, 10'd14, 10'd14};

    wire        S_hit;
    wire [23:0] S_color;
    wire        S_de_out;

    box_draw #(
        .SELF_TEST   ( 0       ),
        .BORDER_W    ( 2       ),
        .CROSS_HALF  ( CR_HALF ),
        .COLOR_BOX0  ( C_BOX0  ),
        .COLOR_BOX1  ( C_BOX1  ),
        .COLOR_BOX2  ( C_BOX2  ),
        .COLOR_BOX3  ( C_BOX3  ),
        .COLOR_CROSS ( C_CROSS )
    ) u_box_draw (
        .I_clk       ( S_clk       ),
        .I_rst_n     ( S_rst_n     ),
        .I_de        ( W_de        ),
        .I_pix_x     ( S_pix_x     ),
        .I_pix_y     ( S_pix_y     ),
        .I_box_valid ( S_box_valid ),
        .I_box_xmin  ( S_box_xmin  ),
        .I_box_xmax  ( S_box_xmax  ),
        .I_box_ymin  ( S_box_ymin  ),
        .I_box_ymax  ( S_box_ymax  ),
        .I_cross_en  ( 1'b1        ),
        .I_cross_x   ( CR_X[10:0]  ),
        .I_cross_y   ( CR_Y[9:0]   ),
        .O_hit       ( S_hit       ),
        .O_color     ( S_color     ),
        .O_de        ( S_de_out    )
    );

    //==================================================================
    // 5. 期望模型：按规格朴素实现（BORDER_W=2，准星优先）
    //==================================================================
    // 单个框的边框命中。★每条边都必须有完整的区间上下界：
    //   竖边：y ∈ [y0,y1] 且 ( x ∈ [x0,x0+1] 或 x ∈ [x1-1,x1] )
    //   横边：x ∈ [x0,x1] 且 ( y ∈ [y0,y0+1] 或 y ∈ [y1-1,y1] )
    // 第一版这里漏了 `x >= x0` / `y >= y0` 的下界，导致框上方与左侧拖出大片色块，
    // 而且 RTL / TB / Python 三份实现犯的是同一个错、互相对得上——见 rtl/box_draw.v
    // 第 3 节的坑记录。
    function border_hit;
        input integer x, y, x0, y0, x1, y1;
        integer inx, iny, el, er, et, eb;
        begin
            inx = (x >= x0) && (x <= x1);
            iny = (y >= y0) && (y <= y1);
            el  = (x >= x0)     && (x <= x0 + 1);      // 左边 2 列
            er  = (x >= x1 - 1) && (x <= x1);          // 右边 2 列
            et  = (y >= y0)     && (y <= y0 + 1);      // 上边 2 行
            eb  = (y >= y1 - 1) && (y <= y1);          // 下边 2 行
            if((x1 - x0) <= 3 || (y1 - y0) <= 3)
                border_hit = 0;                        // 退化框不画
            else
                border_hit = (iny && (el || er)) || (inx && (et || eb));
        end
    endfunction

    function [23:0] exp_color;
        input integer x, y;
        integer cv, ch;
        begin
            cv = ((x >= CR_X) && (x <= CR_X + 1)
                  && (y >= CR_Y - CR_HALF) && (y <= CR_Y + CR_HALF));
            ch = ((y >= CR_Y) && (y <= CR_Y + 1)
                  && (x >= CR_X - CR_HALF) && (x <= CR_X + CR_HALF));

            if(cv || ch)                                      exp_color = C_CROSS;
            else if(border_hit(x, y, B0_X0, B0_Y0, B0_X1, B0_Y1)) exp_color = C_BOX0;
            else if(border_hit(x, y, B1_X0, B1_Y0, B1_X1, B1_Y1)) exp_color = C_BOX1;
            else if(border_hit(x, y, B2_X0, B2_Y0, B2_X1, B2_Y1)) exp_color = C_BOX2;
            else if(border_hit(x, y, B3_X0, B3_Y0, B3_X1, B3_Y1)) exp_color = C_BOX3;
            else                                              exp_color = 24'd0;
        end
    endfunction

    //==================================================================
    // 6. 检查 + dump
    //==================================================================
    integer f_out;
    integer n_coord_ck = 0, n_coord_err = 0;
    integer n_opx_ck   = 0, n_opx_err   = 0;
    integer n_frame_done = 0;
    integer n_dump     = 0;
    integer n_last_on_de = 0, n_last_off_de = 0;
    integer n_vsync_rise = 0;
    reg     armed = 0;                   // 首个 vsync 上升沿之后才开始统计

    initial begin
        f_out = $fopen("sim/tb_box_draw_out.txt", "w");
        if(f_out == 0) begin
            $display("ERROR: 无法打开 sim/tb_box_draw_out.txt（请在 track_box 目录下运行）");
            $finish;
        end
    end

    always @(posedge S_clk) begin
        if(!S_rst_n) begin
            n_vsync_rise = 0;
            armed        = 0;
        end
        else begin
            // ---- vsync 上升沿：解除 armed + 计帧 ----
            if(W_vsync && (S_vsync_r == 1'b0)) begin
                n_vsync_rise = n_vsync_rise + 1;
                armed        = 1'b1;
            end

            // ---- 检查 0：last 必须与 de 同拍（pix_coord_gen 的硬约束）----
            if(W_last) begin
                if(W_de) n_last_on_de  = n_last_on_de  + 1;
                else     n_last_off_de = n_last_off_de + 1;
            end

            // ---- 检查 1：坐标系统 ----
            if(W_de) begin
                n_coord_ck = n_coord_ck + 1;
                if((S_pix_x !== W_true_x) || (S_pix_y !== W_true_y)) begin
                    n_coord_err = n_coord_err + 1;
                    if(n_coord_err < 8)
                        $display("  [坐标错] t=%0t 真实(%0d,%0d) 但 O_pix=(%0d,%0d)",
                                 $time, W_true_x, W_true_y, S_pix_x, S_pix_y);
                end
            end

            if(S_frame_done) n_frame_done = n_frame_done + 1;

            // ---- 检查 2：叠加输出（输出级像素 = S_pix_x_2d / S_pix_y_2d）----
            if(S_de_out) begin
                n_opx_ck = n_opx_ck + 1;
                begin : chk
                    reg [23:0] e;
                    e = exp_color(S_pix_x_2d, S_pix_y_2d);
                    if((S_hit !== (e != 24'd0)) || (S_color !== e)) begin
                        n_opx_err = n_opx_err + 1;
                        if(n_opx_err < 12)
                            $display("  [叠加错] t=%0t 像素(%0d,%0d) DUT(hit=%b,color=%06x) 期望(color=%06x)",
                                     $time, S_pix_x_2d, S_pix_y_2d, S_hit, S_color, e);
                    end

                    if(armed) begin
                        $fwrite(f_out, "%0d %0d %0d %06x\n",
                                S_pix_x_2d, S_pix_y_2d, S_hit, S_color);
                        n_dump = n_dump + 1;
                    end
                end
            end

            // ---- 收尾：跑够 N_FRAMES 个 vsync 上升沿就结束 ----
            if(n_vsync_rise >= N_FRAMES) begin
                $fclose(f_out);
                report;
                $finish;
            end
        end
    end

    reg S_vsync_r = 1'b0;
    always @(posedge S_clk) begin
        if(!S_rst_n) S_vsync_r <= 1'b0;
        else         S_vsync_r <= W_vsync;
    end

    //==================================================================
    // 7. 报告
    //==================================================================
    task report;
        begin
            $display("");
            $display("============================================================");
            $display(" tb_box_draw 结果");
            $display("============================================================");
            $display(" last 与 de: 同拍 %0d 次 / 不同拍 %0d 次  %s",
                     n_last_on_de, n_last_off_de,
                     (n_last_off_de == 0) ? "[PASS]" : "[FAIL]");
            $display(" 坐标检查  : %0d 拍, 错 %0d  %s",
                     n_coord_ck, n_coord_err,
                     (n_coord_err == 0) ? "[PASS]" : "[FAIL]");
            $display(" 叠加检查  : %0d 拍, 错 %0d  %s",
                     n_opx_ck, n_opx_err,
                     (n_opx_err == 0) ? "[PASS]" : "[FAIL]");
            $display(" frame_done 脉冲数 : %0d（期望 %0d）",
                     n_frame_done, N_FRAMES - 1);
            $display(" dump 像素数 : %0d（已写入 sim/tb_box_draw_out.txt）", n_dump);
            $display("============================================================");
            if(n_coord_err == 0 && n_opx_err == 0 && n_last_off_de == 0)
                $display(" 全部 PASS");
            else
                $display(" 存在 FAIL");
            $display("============================================================");
        end
    endtask

endmodule
