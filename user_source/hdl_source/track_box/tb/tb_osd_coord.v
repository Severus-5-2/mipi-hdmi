//=====================================================================
//  tb_osd_coord.v  ——  osd_coord + pix_coord_gen 自检 testbench
//---------------------------------------------------------------------
//  仿真器：Icarus Verilog 12.0
//
//      cd user_source/hdl_source/track_box
//      iverilog -g2005 -o sim/tb_osd_coord.vvp \
//               tb/tb_osd_coord.v rtl/osd_coord.v rtl/pix_coord_gen.v \
//               rtl/track_glyph_rom.v
//      vvp sim/tb_osd_coord.vvp +xv=640 +yv=360          # 默认值
//      vvp sim/tb_osd_coord.vvp +xv=1279 +yv=719         # 边界值（最大坐标）
//      vvp sim/tb_osd_coord.vvp +xv=0 +yv=0              # 最小值
//
//  为什么用「数值 via plusargs」
//    要验的是「X/Y 数值 → 屏幕字形」这条完整链路，必须换若干组数值各跑一遍：
//    0000 这种前导零、1279 这种最大值的进位，都是最容易出错的位。
//    用 plusargs 就不必为每组值改代码重编译。
//
//  为什么用 112x32 的小画面
//    osd_coord 的两行文字各 6 格 = 96 px 宽，56 px 高用不到；
//    112x32 刚好装下两行，仿真快。
//    通过参数把文字区挪到 (0,0) 与 (0,16)，即可在画面内直接观察。
//
//  产出
//    把叠加被点亮的像素 dump 到 sim/tb_osd_coord_out.txt（格式 "x y"）。
//    再由 sim/bit_sim_osd_coord.py 读【从 RTL 解析出来的字模】自行拼出
//    "X:0640"/"Y:0360" 并逐位比对——注意是解析 RTL 字模，不是手抄字模。
//
//  ★这里的期望值仍然不是手写的常量表：Python 侧是从 track_glyph_rom.v
//    解析出点阵再拼字，属于真正的端到端核对。
//=====================================================================
`timescale 1ns / 1ps

module tb_osd_coord;

    localparam integer H_ACTIVE = 112;
    localparam integer H_FRAME  = 140;
    localparam integer V_ACTIVE = 32;
    localparam integer V_FRAME  = 40;
    localparam integer N_FRAMES = 3;

    // osd_coord 的文字区被挪到画面左上，便于在 112x32 里观察
    localparam integer OSD_X0 = 0;
    localparam integer OSD_Y1 = 0;
    localparam integer OSD_Y2 = 16;

    localparam [23:0] C_COORD = 24'hfff200;

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
    // 2. 数值（来自 plusargs，默认 640 / 360 —— D1 标定参考点）
    //    注意用 reg 而不是 integer：需要对它做位选 arg_xv[10:0]，
    //    且 $value$plusargs 的结果用一个临时变量接住，避免 void' 这种
    //    SystemVerilog 专有写法（本 TB 按 -g2005 编译）。
    //==================================================================
    reg [31:0] arg_xv = 32'd640;
    reg [31:0] arg_yv = 32'd360;
    integer    plus_ok;

    initial begin
        plus_ok = $value$plusargs("xv=%d", arg_xv);
        plus_ok = $value$plusargs("yv=%d", arg_yv);
    end

    wire [10:0] W_xv = arg_xv[10:0];
    wire [9:0]  W_yv = arg_yv[9:0];

    //==================================================================
    // 3. 视频时序
    //==================================================================
    reg [11:0] c_hcnt = 12'd0;
    reg [11:0] c_vcnt = 12'd0;
    reg        S_vsync_r = 1'b0;

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
    wire W_vsync = (c_vcnt > (V_ACTIVE + 2)) && (c_vcnt <= (V_ACTIVE + 6));

    //==================================================================
    // 4. DUT
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

    wire        S_hit;
    wire [23:0] S_color;

    osd_coord #(
        .COORD_X0    ( OSD_X0   ),
        .LINE1_Y     ( OSD_Y1   ),
        .LINE2_Y     ( OSD_Y2   ),
        .COLOR_COORD ( C_COORD  )
    ) u_osd_coord (
        .I_clk        ( S_clk         ),
        .I_rst_n      ( S_rst_n       ),
        .I_de         ( W_de          ),
        .I_pix_x      ( S_pix_x       ),
        .I_pix_y      ( S_pix_y       ),
        .I_x_val      ( W_xv          ),
        .I_y_val      ( W_yv          ),
        .I_frame_done ( S_frame_done  ),
        .O_hit        ( S_hit         ),
        .O_color      ( S_color       )
    );

    //==================================================================
    // 5. 检查 + dump
    //==================================================================
    integer f_out;
    integer n_vsync_rise = 0;
    integer n_dump = 0;
    integer n_hit  = 0;
    integer n_color_err = 0;

    // 只有落在这两个矩形内的像素才允许被点亮
    wire W_in_zone = ((S_pix_x_2d < OSD_X0 + 96) &&
                      ((S_pix_y_2d >= OSD_Y1) && (S_pix_y_2d < OSD_Y1 + 16))) ||
                     ((S_pix_x_2d < OSD_X0 + 96) &&
                      ((S_pix_y_2d >= OSD_Y2) && (S_pix_y_2d < OSD_Y2 + 16)));

    initial begin
        f_out = $fopen("sim/tb_osd_coord_out.txt", "w");
        if(f_out == 0) begin
            $display("ERROR: 无法打开 sim/tb_osd_coord_out.txt（请在 track_box 目录下运行）");
            $finish;
        end
        $display("tb_osd_coord:  X=%0d  Y=%0d（期望屏幕显示 X:%04d  Y:%04d）",
                 arg_xv, arg_yv, arg_xv, arg_yv);
    end

    always @(posedge S_clk) begin
        if(!S_rst_n) begin
            n_vsync_rise = 0;
            S_vsync_r    = 1'b0;
        end
        else begin
            // ★先判边沿、再更新边沿寄存器。反过来写会变成
            //   "W_vsync && (W_vsync == 0)" 这种恒假条件，边沿永远检测不到，
            //   仿真直接跑死（第一版就是这么挂的，20s 超时才发现）。
            if(W_vsync && (S_vsync_r == 1'b0))
                n_vsync_rise = n_vsync_rise + 1;
            S_vsync_r = W_vsync;

            // 输出级检查：命中区域之外绝不允许点亮；颜色必须是 C_COORD
            if(S_hit) begin
                n_hit = n_hit + 1;
                if(!W_in_zone) begin
                    if(n_hit < 10)
                        $display("  [区域错] t=%0t 像素(%0d,%0d) 被点亮但在文字区之外",
                                 $time, S_pix_x_2d, S_pix_y_2d);
                    n_color_err = n_color_err + 1;
                end
                if(S_color !== C_COORD) begin
                    if(n_color_err < 10)
                        $display("  [颜色错] t=%0t 像素(%0d,%0d) color=%06x 期望 %06x",
                                 $time, S_pix_x_2d, S_pix_y_2d, S_color, C_COORD);
                    n_color_err = n_color_err + 1;
                end

                // 第 2 帧（首个 vsync 之后的一整帧）才 dump
                if(n_vsync_rise == 1) begin
                    $fwrite(f_out, "%0d %0d\n", S_pix_x_2d, S_pix_y_2d);
                    n_dump = n_dump + 1;
                end
            end

            if(n_vsync_rise >= N_FRAMES) begin
                $fclose(f_out);
                $display("");
                $display("============================================================");
                $display(" tb_osd_coord 结果   X=%0d Y=%0d", arg_xv, arg_yv);
                $display("============================================================");
                $display(" 点亮像素 %0d 个，dump %0d 个", n_hit, n_dump);
                $display(" 区域/颜色错误 : %0d  %s", n_color_err,
                         (n_color_err == 0) ? "[PASS]" : "[FAIL]");
                $display(" 已写 sim/tb_osd_coord_out.txt");
                $display("============================================================");
                $finish;
            end
        end
    end

endmodule
