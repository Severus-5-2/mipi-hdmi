//=============================================================================
//  osd_category.v —— 目标类别与数量 OSD（开发方案B §2.4，D6「OSD 文字」）
//=============================================================================
//  项目：MIPI-HDMI 实时视频图像处理系统 · 方案B
//  负责人：张金艺   归属：user_source/hdl_source/detect_zjy/
//  依据：`doc/开发方案B-目标检测跟踪与运动检测.md` §2.4
//
//-----------------------------------------------------------------------------
//  一、显示内容
//-----------------------------------------------------------------------------
//  屏幕左上角两行（区域起点取 16 的整数倍 ⇒ **零除法**）：
//
//      R:1 B:0 G:2      ← 颜色计数（红/蓝/绿，来自 target_bus 的 cls）
//      S:1 C:0 T:2      ← 形状计数（方/圆/三角，来自 shape_class）
//
//  R=红 B=蓝 G=绿 ；S=方(Square) C=圆(Circle) T=三角(Triangle)
//
//-----------------------------------------------------------------------------
//  二、为什么自带上字母 ROM
//-----------------------------------------------------------------------------
//  共享字库 `osd_char_lib.v` 在 2026-10-08 的中文 OSD 改版中**删掉了全部英文字母**，
//  只剩数字 0-9、冒号与 14 个汉字。而本模块要显示 R/B/G/S/C/T。
//
//  纪律：**不改动共享文件 `osd_char_lib.v`**（那是郭的顶层资产，现有五处 OSD 在用它）。
//  做法（与张铭晨 `track_glyph_rom.v` 同一思路）：
//      · 数字与冒号 → **例化 `osd_char_lib` 复用**（不分叉）
//      · 六个字母   → **本模块自带小 ROM**
//      · 按字符码分派：`I_char <= 8'h08` 走自带 ROM，否则走 `osd_char_lib`
//
//  字模规格与 `osd_char_lib.v` 保持一致：
//      · 16x16 单元，**bit15 = 最左像素**
//      · 有效区 **bits[12:3]**（10 列，居中）
//      · 字形占 **row 1..12**（row 0/13/14/15 空）
//      · 笔画 2 行/2 列粗（与既有数字同风格）
//
//-----------------------------------------------------------------------------
//  三、时序（沿用 osd_coord.v 已被验证的两级流水）
//-----------------------------------------------------------------------------
//   第 1 拍：区域判定 + 字符码查表 → 寄存
//   第 2 拍：字模查表 + 位选 → O_hit
//   ⇒ O_hit 相对 I_de 晚 2 拍，用延 2 拍的 de 门控（与 osd_coord 完全一致）
//=============================================================================

`timescale 1ns / 1ps
`include "../common_base/common_defs.vh"   // ★ 相对本文件目录（同 design_top_wrapper.v 的写法）

module osd_category #(
    //---------------- 布局（必须为 16 的整数倍）----------------
    parameter integer X0 = 16,               // 区域左边
    parameter integer Y0 = 16,               // 第一行上边
    parameter integer COLS = 11,             // 每行字符格数（"R:1 B:0 G:2" = 11）
    parameter [23:0]  COLOR = 24'hfff200     // 与现有 OSD 文字同色（黄）
)(
    input  wire        clk,
    input  wire        rst_n,

    //---- 像素流（与 I_de 同拍）----
    input  wire        I_de,
    input  wire [10:0] I_pix_x,
    input  wire [9:0]  I_pix_y,

    //---- 待显示的数据（静态量，帧末更新）----
    input  wire [`TGT_BUS_W-1:0] I_bus,      // 4 槽目标总线（取 cls）
    input  wire [7:0]            I_shapes,   // shape_class 输出（每槽 2 bit）
    input  wire [2:0]            I_valid_cnt,
    input  wire                  I_frame_done,

    //---- 输出（相对 I_de 晚 2 拍）----
    output wire        O_hit,
    output wire [23:0] O_color
);

    //=========================================================================
    //  0. 字符码（本模块自定义，≤ 8'h08 表示"自带 ROM"）
    //=========================================================================
    localparam [7:0] CH_SP = 8'h00;     // 空白
    localparam [7:0] CH_R  = 8'h01;
    localparam [7:0] CH_B  = 8'h02;
    localparam [7:0] CH_G  = 8'h03;
    localparam [7:0] CH_S  = 8'h04;
    localparam [7:0] CH_C  = 8'h05;
    localparam [7:0] CH_T  = 8'h06;
    localparam [7:0] CH_COLON = 8'h3a;  // 复用 osd_char_lib 的冒号
    localparam [7:0] CH_DIG0  = 8'h30;  // 复用 osd_char_lib 的数字基址

    //=========================================================================
    //  1. 帧末统计：每色 / 每形状的目标数（0~4，4 bit）
    //=========================================================================
    localparam integer SW = `TGT_SLOT_W;   // 96

    reg [3:0] cnt_red, cnt_blue, cnt_green;
    reg [3:0] cnt_sq,  cnt_cir,  cnt_tri;

    // 槽 k 的 cls / shape
    wire [1:0] c0 = I_bus[`TGT_O_CLS + 0*SW +: 2];
    wire [1:0] c1 = I_bus[`TGT_O_CLS + 1*SW +: 2];
    wire [1:0] c2 = I_bus[`TGT_O_CLS + 2*SW +: 2];
    wire [1:0] c3 = I_bus[`TGT_O_CLS + 3*SW +: 2];
    wire [1:0] s0 = I_shapes[1:0];
    wire [1:0] s1 = I_shapes[3:2];
    wire [1:0] s2 = I_shapes[5:4];
    wire [1:0] s3 = I_shapes[7:6];

    localparam [1:0] SHP_SQ = 2'd0, SHP_CIR = 2'd1, SHP_TRI = 2'd2;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_red <= 4'd0; cnt_blue <= 4'd0; cnt_green <= 4'd0;
            cnt_sq  <= 4'd0; cnt_cir  <= 4'd0; cnt_tri   <= 4'd0;
        end else if (I_frame_done) begin
            cnt_red   <= (c0 == `CLS_RED)   + (c1 == `CLS_RED)   + (c2 == `CLS_RED)   + (c3 == `CLS_RED);
            cnt_blue  <= (c0 == `CLS_BLUE)  + (c1 == `CLS_BLUE)  + (c2 == `CLS_BLUE)  + (c3 == `CLS_BLUE);
            cnt_green <= (c0 == `CLS_GREEN) + (c1 == `CLS_GREEN) + (c2 == `CLS_GREEN) + (c3 == `CLS_GREEN);
            cnt_sq    <= (s0 == SHP_SQ)     + (s1 == SHP_SQ)     + (s2 == SHP_SQ)     + (s3 == SHP_SQ);
            cnt_cir   <= (s0 == SHP_CIR)    + (s1 == SHP_CIR)    + (s2 == SHP_CIR)    + (s3 == SHP_CIR);
            cnt_tri   <= (s0 == SHP_TRI)    + (s1 == SHP_TRI)    + (s2 == SHP_TRI)    + (s3 == SHP_TRI);
        end
    end

    //=========================================================================
    //  2. 区域判定（第 1 拍）
    //=========================================================================
    //  区域：x ∈ [X0, X0+COLS*16) ，y ∈ [Y0, Y0+32)
    //  起点是 16 的整数倍 ⇒ 字符列号 = x[11:4] - X0/16 （常量减法）
    localparam integer COL0 = X0 / 16;
    localparam integer ROW0 = Y0 / 16;

    //  ⚠ I_pix_x 只有 11 位（0..1279）⇒ 列号取 [10:4]（7 位，0..79），不能写 [11:4]（越界出 x）
    wire [6:0] col_now = I_pix_x[10:4] - COL0[6:0];   // 当前列号（区域内 0..COLS-1）
    wire [5:0] row_now = I_pix_y[9:4] - ROW0[5:0];    // 当前行号（区域内 0 或 1）

    //  区域命中：列在 range、行在 0..1、且在有效像素期
    wire in_area = I_de
                && (I_pix_x >= X0[10:0]) && (I_pix_x < (X0 + COLS*16))
                && (I_pix_y >= Y0[9:0])  && (I_pix_y < (Y0 + 32));

    //-------------------------------------------------------------------------
    //  字符码查找（按列号）
    //-------------------------------------------------------------------------
    //  第 0 行：R:红 B:蓝 G:绿
    //  第 1 行：S:方 C:圆 T:三
    //  每 4 格一组（字母、冒号、数字、空白）
    wire [7:0] dig_top = {4'd0, cnt_red}   + CH_DIG0;   // 数字 ASCII（0~4）
    wire [7:0] dig_mid = {4'd0, cnt_blue}  + CH_DIG0;
    wire [7:0] dig_bot = {4'd0, cnt_green} + CH_DIG0;
    wire [7:0] dig_sq  = {4'd0, cnt_sq}    + CH_DIG0;
    wire [7:0] dig_cir = {4'd0, cnt_cir}   + CH_DIG0;
    wire [7:0] dig_tri = {4'd0, cnt_tri}   + CH_DIG0;

    reg [7:0] ch;        // 第 1 拍选出的字符码
    reg       in_r1;     // 第 1 拍的 in_area（打拍）
    reg [3:0] lx_r1;     // 第 1 拍的列内像素号 x[3:0]

    wire [7:0] ch_r0 = (col_now == 7'd0 ) ? CH_R :
                       (col_now == 7'd1 ) ? CH_COLON :
                       (col_now == 7'd2 ) ? dig_top : CH_SP;
    wire [7:0] ch_r1 = (col_now == 7'd4 ) ? CH_B :
                       (col_now == 7'd5 ) ? CH_COLON :
                       (col_now == 7'd6 ) ? dig_mid : CH_SP;
    wire [7:0] ch_r2 = (col_now == 7'd8 ) ? CH_G :
                       (col_now == 7'd9 ) ? CH_COLON :
                       (col_now == 7'd10) ? dig_bot : CH_SP;

    wire [7:0] ch_s0 = (col_now == 7'd0 ) ? CH_S :
                       (col_now == 7'd1 ) ? CH_COLON :
                       (col_now == 7'd2 ) ? dig_sq  : CH_SP;
    wire [7:0] ch_s1 = (col_now == 7'd4 ) ? CH_C :
                       (col_now == 7'd5 ) ? CH_COLON :
                       (col_now == 7'd6 ) ? dig_cir : CH_SP;
    wire [7:0] ch_s2 = (col_now == 7'd8 ) ? CH_T :
                       (col_now == 7'd9 ) ? CH_COLON :
                       (col_now == 7'd10) ? dig_tri : CH_SP;

    wire [7:0] ch_sel = (row_now == 6'd0) ? (ch_r0 | ch_r1 | ch_r2)
                                           : (ch_s0 | ch_s1 | ch_s2);

    //  第 1 拍：寄存字符码 + 列内像素号 + 行内像素号 + in_area
    reg [3:0] ly_r1;     // 行内像素号 y[3:0]
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ch <= CH_SP; in_r1 <= 1'b0; lx_r1 <= 4'd0; ly_r1 <= 4'd0;
        end else begin
            ch    <= ch_sel;
            in_r1 <= in_area;
            lx_r1 <= I_pix_x[3:0];
            ly_r1 <= I_pix_y[3:0];
        end
    end

    //=========================================================================
    //  3. 字模查表（第 2 拍）
    //=========================================================================
    //  · I_char <= 8'h08 → 自带字母 ROM
    //  · 否则（数字 0x30-0x39 / 冒号 0x3a）→ 复用 osd_char_lib
    wire        is_letter = (ch <= 8'h08) && (ch != CH_SP);

    wire [15:0] rom_letter;
    wire [15:0] rom_lib;

    //---- 自带六个字母的 ROM（16 行；与 osd_char_lib 同风格：10 列宽、bit15 左）----
    reg [15:0] ltr;
    always @(*) begin
        ltr = 16'h0000;
        case (ch)
            //-------------------------------------------------- R
            CH_R: case (ly_r1)
                4'd1,  4'd2:  ltr = 16'h1ff8;   // 顶横
                4'd3,  4'd4, 4'd5: ltr = 16'h300c;   // 左竖 + 右竖
                4'd6,  4'd7:  ltr = 16'h1ff8;   // 中横
                4'd8:  ltr = 16'h3030;          // 斜腿起点
                4'd9:  ltr = 16'h3060;
                4'd10: ltr = 16'h30c0;
                4'd11: ltr = 16'h3000;          // 只有左竖
                4'd12: ltr = 16'h3000;
                default: ltr = 16'h0000;
            endcase
            //-------------------------------------------------- B（上环小、下环大）
            CH_B: case (ly_r1)
                4'd1,  4'd2:  ltr = 16'h1ff8;   // 顶横
                4'd3,  4'd4:  ltr = 16'h300c;
                4'd5:  ltr = 16'h3000;          // 上环收口（右竖结束）
                4'd6,  4'd7:  ltr = 16'h1ff8;   // 中横
                4'd8,  4'd9, 4'd10: ltr = 16'h300c;
                4'd11, 4'd12: ltr = 16'h1ff8;   // 底横
                default: ltr = 16'h0000;
            endcase
            //-------------------------------------------------- G（右侧有横杠）
            CH_G: case (ly_r1)
                4'd1,  4'd2:  ltr = 16'h1ff8;   // 顶横
                4'd3,  4'd4, 4'd5, 4'd6: ltr = 16'h3000;   // 左竖
                4'd7:  ltr = 16'h30f8;          // 中横杠（右侧）
                4'd8,  4'd9, 4'd10: ltr = 16'h300c;
                4'd11, 4'd12: ltr = 16'h1ff8;
                default: ltr = 16'h0000;
            endcase
            //-------------------------------------------------- S（方：上下弧 + 中横）
            CH_S: case (ly_r1)
                4'd1,  4'd2:  ltr = 16'h1ff8;
                4'd3,  4'd4, 4'd5: ltr = 16'h3000;
                4'd6,  4'd7:  ltr = 16'h1ff8;
                4'd8,  4'd9, 4'd10: ltr = 16'h000c;
                4'd11, 4'd12: ltr = 16'h1ff8;
                default: ltr = 16'h0000;
            endcase
            //-------------------------------------------------- C（圆）
            CH_C: case (ly_r1)
                4'd1,  4'd2:  ltr = 16'h1ff8;
                4'd3,  4'd4:  ltr = 16'h300c;
                4'd5,  4'd6, 4'd7, 4'd8: ltr = 16'h3000;
                4'd9,  4'd10: ltr = 16'h300c;
                4'd11, 4'd12: ltr = 16'h1ff8;
                default: ltr = 16'h0000;
            endcase
            //-------------------------------------------------- T（三角）
            CH_T: case (ly_r1)
                4'd1,  4'd2:  ltr = 16'h1ff8;   // 顶横
                4'd3,  4'd4, 4'd5, 4'd6, 4'd7,
                4'd8,  4'd9, 4'd10, 4'd11, 4'd12: ltr = 16'h0180;  // 中竖
                default: ltr = 16'h0000;
            endcase
            default: ltr = 16'h0000;
        endcase
    end
    assign rom_letter = ltr;

    //---- 复用共享字库（数字 / 冒号）----
    osd_char_lib u_char_lib (
        .I_char     (ch),
        .I_row      (ly_r1),
        .O_row_bits (rom_lib)
    );

    wire [15:0] row_bits = is_letter ? rom_letter : rom_lib;

    //  位选：bit15 = 最左像素 ⇒ 取 `15 - lx` 位。lx 为 4 位，15-lx ≡ ~lx。
    wire pix_on = row_bits[~lx_r1];

    assign O_hit   = in_r1 & pix_on;
    assign O_color = COLOR;

endmodule
