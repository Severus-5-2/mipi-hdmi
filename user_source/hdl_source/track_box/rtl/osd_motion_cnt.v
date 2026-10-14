//=====================================================================
//  osd_motion_cnt.v  ——  「MOTION: N」运动区域计数 OSD（方案B §3.2 进阶4）
//---------------------------------------------------------------------
//  归属：张铭晨（进阶4 运动检测）
//  依据：《开发方案B》§3.2 —— 输出规格「运动区域红框 + MOTION: N 计数」
//
//  显示内容（8 个 16x16 字符格 = 128 px 宽，默认贴屏幕左上角）
//        MOTION: 3
//    区域 x = 16..143 、y = 16..31（起点都取 16 的整数倍 ⇒ 字符序号 =
//    x[10:4] - 常数、行号 = y[3:0]，**全程零除法**）
//
//  字符码（本模块 7 个固定字符 + 1 个数字）
//        idx 0..6 = 'M' 'O' 'T' 'I' 'O' 'N' ':'      0x4d 0x4f 0x54 0x49 0x4f 0x4e 0x3a
//        idx 7    = '0' + N（N = I_mot_cnt，0..4）
//
//  颜色
//        N = 0（无运动） → COLOR_IDLE  （默认黄 0xfff200，与坐标 OSD 同色）
//        N > 0（有运动） → COLOR_ACTIVE（默认红 0xff0000，与运动区域框同色）
//        ⇒ 上板时一眼就能看出「有没有运动」，不用去数数字。
//
//  字模来源：track_glyph_rom.v
//    该 ROM 原本只有 0-9 / ':' / 'X' / 'Y'；2026-10-13 为「MOTION」新增
//    I/M/N/O/T 五个字母，**纯新增字符码、原有码位一字未动**
//    （`git diff rtl/track_glyph_rom.v` = 95 插入 / 0 删除）。
//    字形由 sim/glyph_rom_tool.py 生成与校验（check 模式只读 RTL 原文渲染）。
//
//  流水（与 osd_coord.v / hdmi_mixer.v 同款，逐拍对齐关系已在 D1 位级验证过）
//        第 1 拍：区域判定 + 字符码查表（用 _1d 坐标）→ 拍末寄存
//        第 2 拍：字模查表 + 位选（用 _2d 坐标，正好补上这一级寄存）
//        ⇒ O_hit 比 I_de 晚 2 拍；O_hit 已用延 2 拍的 de 门控。
//
//  ⚠ 位宽坑（本项目已踩过两次，勿照抄别的模块的位选）
//    本模块 S_x_1d 是 **11 位**（配 pix_coord_gen 的 O_pix_x[10:0]），
//    字符序号必须写 S_x_1d[10:4]（7 位）。若照抄 hdmi_mixer 的 12 位写法
//    S_x_1d[11:4]，会去选不存在的 bit11 ⇒ 整个序号变 x ⇒ 6 个格子全画成
//    同一个字符。**抄写法前先核对被抄对象的位宽。**
//
//  位级验证
//    tb/tb_osd_motion_cnt.v         iverilog 逐像素扫描（只扫文字带 + 边界）
//    sim/bit_sim_osd_motion_cnt.py  独立渲染「MOTION: N」并逐像素对拍 + ASCII 预览
//=====================================================================
module osd_motion_cnt #(
    //---------------- 布局（单位：像素，1280x720）----------------
    parameter integer MC_X0 = 16,          // 文字区左边（必须为 16 的整数倍）
    parameter integer MC_Y0 = 16,          // 文字区顶边（必须为 16 的整数倍）
    parameter [23:0]  COLOR_IDLE   = 24'hfff200,   // 计数 = 0：黄
    parameter [23:0]  COLOR_ACTIVE = 24'hff0000    // 计数 > 0：红
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    // ---- 像素流（与 I_de 同拍）----
    input  wire        I_de,
    input  wire [10:0] I_pix_x,
    input  wire [9:0]  I_pix_y,

    // ---- 要显示的运动区域数（3bit，0..4；静态量，帧首刷新）----
    input  wire [2:0]  I_mot_cnt,

    // ---- 帧界脉冲（pix_coord_gen.O_frame_done，落在帧首有效像素）----
    input  wire        I_frame_done,

    // ---- 输出（比 I_de 晚 2 拍）----
    output wire        O_hit,
    output wire [23:0] O_color
);

    //==================================================================
    // 1. 布局常量
    //==================================================================
    localparam integer NCHAR = 8;                       // 「MOTION: N」
    localparam [11:0]  MC_W  = NCHAR * 16;              // 128 px
    // ★字符序号基准从 MC_X0 派生（MC_X0 是 16 的倍数 ⇒ 常量除法，综合期折叠）
    localparam [6:0]   COL0  = MC_X0 / 16;

    localparam [7:0] CHAR_M     = 8'h4d;    // 'M'
    localparam [7:0] CHAR_O     = 8'h4f;    // 'O'
    localparam [7:0] CHAR_T     = 8'h54;    // 'T'
    localparam [7:0] CHAR_I     = 8'h49;    // 'I'
    localparam [7:0] CHAR_N     = 8'h4e;    // 'N'
    localparam [7:0] CHAR_COLON = 8'h3a;    // ':'
    localparam [7:0] CHAR_ZERO  = 8'h30;    // '0'，配合 +digit

    //==================================================================
    // 2. 数字（每帧刷新一次，**不在像素流里做任何运算**）
    //==================================================================
    reg [3:0] S_dig;                        // 当前显示的数字 0..4

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)          S_dig <= 4'd0;
        else if(I_frame_done) S_dig <= {1'b0, I_mot_cnt};   // 3bit → 4bit（<=4，恒为个位）
    end

    //==================================================================
    // 3. 坐标两级流水（与 hdmi_mixer / osd_coord 同款）
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
    //==================================================================
    wire W_area = (S_x_1d >= MC_X0) && (S_x_1d < (MC_X0 + MC_W)) &&
                  (S_y_1d >= MC_Y0) && (S_y_1d < (MC_Y0 + 16));

    // 字符序号 = 全局列号 - 区域起始列号（区域起点是 16 的倍数）
    // ★ 必须 [10:4]（11 位坐标），写 [11:4] 会选到不存在的 bit11（见文件头警告）
    wire [6:0] W_col = S_x_1d[10:4] - COL0;

    // idx 0..7 = 'M' 'O' 'T' 'I' 'O' 'N' ':' digit
    function [7:0] F_char;
        input [3:0] I_idx;
        begin
            case(I_idx)
                4'd0:    F_char = CHAR_M;
                4'd1:    F_char = CHAR_O;
                4'd2:    F_char = CHAR_T;
                4'd3:    F_char = CHAR_I;
                4'd4:    F_char = CHAR_O;
                4'd5:    F_char = CHAR_N;
                4'd6:    F_char = CHAR_COLON;
                default: F_char = CHAR_ZERO + {4'b0, S_dig};
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
            S_hit_r <= W_area;
            S_row_r <= S_y_1d[3:0];
            if(W_area)
                S_char_r <= F_char(W_col[3:0]);
            // 未命中时保持原值（省一个 mux，与 hdmi_mixer 同款写法）
        end
    end

    //==================================================================
    // 5. 字库（纯组合，输入已寄存；与 osd_char_lib.v 用法一致）
    //==================================================================
    wire [15:0] S_row_bits;

    track_glyph_rom u_track_glyph_rom(
        .I_char     ( S_char_r    ),
        .I_row      ( S_row_r     ),
        .O_row_bits ( S_row_bits  )
    );

    //==================================================================
    // 6. 第 2 拍：位选 + 输出
    //    S_hit_r 由 _1d 坐标算出，与本拍 S_x_2d 是同一像素（_2d 比 _1d 晚 1 拍，
    //    正好补上第 4 节那一级寄存）。位选 bit15 = 最左，取第 (15-lx) 位；
    //    4 位数据下恒等于 ~lx。
    //==================================================================
    assign O_hit   = S_hit_r && S_row_bits[~S_x_2d[3:0]] && S_de_2d;
    assign O_color = (S_dig == 4'd0) ? COLOR_IDLE : COLOR_ACTIVE;

endmodule
