//=====================================================================
//  osd_coord.v  ——  坐标数字 OSD 叠加（开发方案B 第 3.1 节 进阶2）
//---------------------------------------------------------------------
//  归属：张铭晨（进阶2 目标跟踪与信息标注）
//  D1 目标（方案B 3.3 节）：「画框模块 + OSD 坐标显示（先画固定框，验证坐标系统）」
//
//  显示内容（方案B 3.1 节原文样式）：
//        X:0640
//        Y:0360
//    两块文字区各 6 个 16x16 字符格 = 96 px 宽，位于屏幕左下角：
//        X 行 y = 688..703 ，Y 行 y = 704..719 ，两行都从 x = 16 起
//    （1280x720 下 688 与 704 都是 16 的整数倍 ⇒ 行号 = y[3:0]、无除法）
//
//  设计沿用 hdmi_mixer.v 已被团队验证的 OSD 写法（改动说明 2026-10-09 第 46 节）：
//    · 区域起点取 16 的整数倍 ⇒ 字符序号 = x[11:4] - 常数，**零除法**；
//    · 两级流水：第 1 拍「区域判定 + 字符码查表」寄存，第 2 拍「字模查表 + 位选」；
//    · 取字模某位用变址 O_row_bits[~x_2d[3:0]]（4 位数据下 15-lx 恒等于 ~lx）。
//
//  字符来源：track_glyph_rom.v（自带 ROM，含 X / Y / ':' / 0-9）
//    为什么不直接用 osd_char_lib.v：它已在中文 OSD 改版中删掉了全部英文字母，
//    而本模块要显示 X / Y。为不动郭的共享文件，自带一张小 ROM。
//    若郭后续愿意把 X / Y 并进 osd_char_lib.v，把这里的子模块换掉即可。
//
//  位级验证：tb/tb_osd_coord.v（iverilog）
//=====================================================================
module osd_coord #(
    //---------------- 布局（单位：像素，1280x720）----------------
    parameter integer COORD_X0 = 16,      // 文字区左边（必须为 16 的整数倍）
    parameter integer LINE1_Y  = 688,     // 「X:xxxx」行（必须为 16 的整数倍）
    parameter integer LINE2_Y  = 704,     // 「Y:xxxx」行（必须为 16 的整数倍）
    parameter [23:0]  COLOR_COORD = 24'hfff200   // 与现有 OSD 文字同色（黄）
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    // ---- 像素流（与 I_de 同拍）----
    input  wire        I_de,
    input  wire [10:0] I_pix_x,
    input  wire [9:0]  I_pix_y,

    // ---- 要显示的两个坐标值（静态量，通常接十字准星 / 目标中心）----
    input  wire [10:0] I_x_val,
    input  wire [9:0]  I_y_val,

    // ---- 一帧结束脉冲（pix_coord_gen.O_frame_done），BCD 每帧刷一次 ----
    input  wire        I_frame_done,

    // ---- 输出（比 I_de 晚 2 拍；O_hit 已用延 2 拍的 de 门控）----
    output wire        O_hit,
    output wire [23:0] O_color
);

    //==================================================================
    // 1. 布局常量
    //==================================================================
    localparam [11:0] COORD_W = 12'd96;      // 6 格 x 16 px
    // ★字符序号基准必须从 COORD_X0 派生，不能写死：COORD_X0 一定是 16 的整数倍，
    //   所以这里的 /16 是常量除法，综合期直接折叠成移位，零成本。
    //   （第一版写死成 8'd1 = 16/16，一旦把区域挪到别处就会整体错字符。）
    localparam [6:0]  COL0    = COORD_X0 / 16;

    localparam [7:0]  CHAR_X     = 8'h58;    // 'X'
    localparam [7:0]  CHAR_Y     = 8'h59;    // 'Y'
    localparam [7:0]  CHAR_COLON = 8'h3a;    // ':'
    localparam [7:0]  CHAR_ZERO  = 8'h30;    // '0'，配合 +digit

    //==================================================================
    // 2. 坐标值的 BCD（每帧刷新一次，**不在像素流里做除法**）
    //    方案B 1.2 节：除法一律挪到帧消隐期。这里更彻底——一帧只算一次。
    //==================================================================
    reg [3:0] S_xd3, S_xd2, S_xd1, S_xd0;    // X 的 千 百 十 个
    reg [3:0] S_yd3, S_yd2, S_yd1, S_yd0;    // Y 的 千 百 十 个

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_xd3 <= 4'd0; S_xd2 <= 4'd0; S_xd1 <= 4'd0; S_xd0 <= 4'd0;
            S_yd3 <= 4'd0; S_yd2 <= 4'd0; S_yd1 <= 4'd0; S_yd0 <= 4'd0;
        end
        else if(I_frame_done) begin
            // 全部是「除以常数」，TD 综合为乘法+移位网络（hdmi_mixer 第 13 节
            // 显示 ag 的 BCD 就是这么写的，已验证可用）。一帧一次，不进关键路径。
            S_xd3 <= I_x_val / 11'd1000;
            S_xd2 <= (I_x_val % 11'd1000) / 11'd100;
            S_xd1 <= (I_x_val % 11'd100) / 11'd10;
            S_xd0 <= I_x_val % 11'd10;

            S_yd3 <= I_y_val / 10'd1000;
            S_yd2 <= (I_y_val % 10'd1000) / 10'd100;
            S_yd1 <= (I_y_val % 10'd100) / 10'd10;
            S_yd0 <= I_y_val % 10'd10;
        end
    end

    //==================================================================
    // 3. 坐标两级流水（与 hdmi_mixer 的 S_x_1d/S_x_2d 同款）
    //==================================================================
    reg [10:0] S_x_1d, S_x_2d;
    reg [9:0]  S_y_1d, S_y_2d;
    reg        S_de_2d;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x_1d <= 11'd0; S_x_2d <= 11'd0;
            S_y_1d <= 10'd0; S_y_2d <= 10'd0;
            S_de_2d <= 1'b0;
        end
        else begin
            S_x_1d <= I_pix_x;  S_x_2d <= S_x_1d;
            S_y_1d <= I_pix_y;  S_y_2d <= S_y_1d;
            S_de_2d <= I_de;
        end
    end

    //==================================================================
    // 4. 第 1 拍：区域判定 + 字符码查表（结果拍末寄存）
    //    ★用 _1d 而不是 _2d：这两步（比较器 + 8 位减法 + 字符码 case）
    //      在拍末被寄存，使第 2 拍只剩「字模查表 + 位选」。
    //      时序与 hdmi_mixer 第 8 节注释完全同构，可直接对照。
    //==================================================================
    wire W_line1_area = (S_x_1d >= COORD_X0) && (S_x_1d < (COORD_X0 + COORD_W)) &&
                        (S_y_1d >= LINE1_Y)  && (S_y_1d < (LINE1_Y + 16));
    wire W_line2_area = (S_x_1d >= COORD_X0) && (S_x_1d < (COORD_X0 + COORD_W)) &&
                        (S_y_1d >= LINE2_Y)  && (S_y_1d < (LINE2_Y + 16));

    // 字符序号 = 全局列号 - 区域起始列号（区域起点是 16 的倍数）
    //
    // ★★ 位选越界坑（2026-10-09，务必记住）★★
    //   hdmi_mixer.v 里写的是 S_x_1d[11:4]，因为**那边 S_x 是 12 位**。
    //   本模块的 S_x_1d 是 11 位（配 pixel_coord_gen 的 O_pix_x[10:0]），
    //   照抄 [11:4] 会去选不存在的 bit11 ⇒ Verilog 返回 1'bx ⇒
    //   整个 W_col 变成 xxxx ⇒ 字符码 case 全部落到 default ⇒
    //   屏幕上 6 个格子全画成同一个字符（实测全变成 '0'）。
    //   正确写法是 [10:4]（7 位，装得下 0..127 的格子号）。
    //   教训：**抄写法之前先核对被抄对象的位宽**。
    wire [6:0] W_col = S_x_1d[10:4] - COL0;

    // 第 1 行： idx 0..5 = 'X' ':' d3 d2 d1 d0
    function [7:0] F_line1_char;
        input [3:0] I_idx;
        begin
            case(I_idx)
                4'd0:    F_line1_char = CHAR_X;
                4'd1:    F_line1_char = CHAR_COLON;
                4'd2:    F_line1_char = CHAR_ZERO + S_xd3;
                4'd3:    F_line1_char = CHAR_ZERO + S_xd2;
                4'd4:    F_line1_char = CHAR_ZERO + S_xd1;
                default: F_line1_char = CHAR_ZERO + S_xd0;
            endcase
        end
    endfunction

    // 第 2 行： idx 0..5 = 'Y' ':' d3 d2 d1 d0
    function [7:0] F_line2_char;
        input [3:0] I_idx;
        begin
            case(I_idx)
                4'd0:    F_line2_char = CHAR_Y;
                4'd1:    F_line2_char = CHAR_COLON;
                4'd2:    F_line2_char = CHAR_ZERO + S_yd3;
                4'd3:    F_line2_char = CHAR_ZERO + S_yd2;
                4'd4:    F_line2_char = CHAR_ZERO + S_yd1;
                default: F_line2_char = CHAR_ZERO + S_yd0;
            endcase
        end
    endfunction

    reg [7:0] S_char_r;      // 送给字库的字符码（已寄存）
    reg [3:0] S_row_r;       // 字模行号 0..15（已寄存）
    reg       S_hit_r;       // 该像素是否落在文字区内（已寄存）

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_char_r <= 8'd0;
            S_row_r  <= 4'd0;
            S_hit_r  <= 1'b0;
        end
        else begin
            S_hit_r <= W_line1_area || W_line2_area;
            S_row_r <= S_y_1d[3:0];

            if(W_line1_area)
                S_char_r <= F_line1_char(W_col[3:0]);
            else if(W_line2_area)
                S_char_r <= F_line2_char(W_col[3:0]);
            // 未命中时保持原值（省一个 mux，与 hdmi_mixer 同款写法）
        end
    end

    //==================================================================
    // 5. 字库（纯组合，与 osd_char_lib 用法一致：输入已寄存）
    //==================================================================
    wire [15:0] S_row_bits;

    track_glyph_rom u_track_glyph_rom(
        .I_char     ( S_char_r    ),
        .I_row      ( S_row_r     ),
        .O_row_bits ( S_row_bits  )
    );

    //==================================================================
    // 6. 第 2 拍：位选 + 输出
    //    S_hit_r 由 _1d 坐标算出，与本拍的 S_x_2d 严格同像素
    //    （_2d 比 _1d 晚 1 拍，正好补上这一级寄存）。
    //    位选：bit15 = 最左，取第 (15 - lx) 位；4 位数据下恒等于 ~lx。
    //==================================================================
    assign O_hit   = S_hit_r && S_row_bits[~S_x_2d[3:0]] && S_de_2d;
    assign O_color = COLOR_COORD;

endmodule
