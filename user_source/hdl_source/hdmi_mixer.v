//=====================================================================
//  hdmi_mixer.v  ——  OSD 叠加 + HDMI 输出
//---------------------------------------------------------------------
//  ★2026-10-08 改版（中文 OSD，比赛硬性要求：切换模式须在屏上显示模式名）
//
//  一、移除
//    · 旧「安路 CNT: xxxxxx」DYN_OSD 块（位置 375,120，其内容已被新布局取代）
//    · 6 个从未被调用的字模 function（死代码）：
//        F_logo_text_bits / F_hex7seg / F_cnt_label_bits /
//        F_cn16_row / F_dyn_char_bits / F_osd_char16_row
//    · 与之配套的死寄存器（S_lbl_* / S_dyn_char / S_dyn_seg / S_cn_* / S_char_* 等）
//
//  二、新增三处【常驻】中文 OSD（全部 16x16 点阵、1:1 映射、零除法）
//        ① Logo 右侧 第 1 行： 「帧数:000123」   —— 帧计数器（6 位十进制）
//        ② Logo 右侧 第 2 行： 「增益:187」      —— AE 增益控制字 ag（3 位十进制）
//        ③ 右上角：            「模式:彩色 / 灰度 / 二值 / 边缘」
//
//  三、布局（1280x720，单位像素）
//        ┌──────────────────────────────────────────────────────────┐
//        │ [安路 Logo 160x160]   帧数:000123       模式:彩色         │
//        │                       增益:187                            │
//        │                                                           │
//        │                     （视频画面）                          │
//        └──────────────────────────────────────────────────────────┘
//     · 字符格 16x16；所有区域 X/Y 起点都是 16 的整数倍
//       ⇒ 局部列 = S_x_2d[3:0]、字符序号 = S_x_2d[11:4] - 常数，
//         **完全不需要常数除法**（旧实现有 /24、/15、/8 三处）
//     · 取字模某一位用变址 S_pf_row_bits[~lx]（lx = S_x_2d[3:0]；
//       4 位数据下 15-lx 恒等于 ~lx）⇒ 取代旧的 16 路 case 展开
//     · 字符渲染分两级流水：第 1 拍做「区域判定 + 字符码查表」（拍末寄存），
//       第 2 拍只做「字模查表 + 位选」⇒ 关键路径约 7 级（合并非流水约 15 级）
//
//  五、★2026-10-09 时序修复（Logo 也做两级流水；上板 WNS -0.083ns 的那 5 条）
//     · 旧写法：Logo 的「区域比较 + 坐标减法 + S_logo_y*LOGO_W（DSPMULT）
//       + 加法 + anlogic_logo_rom 的 6400 条组合 case（~13 级 LUT mux 树）」
//       全塞在第 2 拍，直通 O_hdmi_data_reg ⇒ Logic-Level = 15、扇出 113。
//     · 现写法：第 1 拍（S_x_1d/S_y_1d）算区域与地址并寄存命中标志，
//       地址送【同步 ROM】（anlogic_logo_rom 增加 I_clk，输出打 1 拍），
//       第 2 拍只剩「ROM 输出 + alpha + 叠加 mux」≈ 3 级。
//       **总延迟不变（仍是与像素对齐的 2 拍）**，理由见 17 节注释。
//     · 顺带：LOGO_W = 160 = 128 + 32 ⇒ 地址改用两次左移相加，
//       去掉 1 个 DSP 乘法器（同类教训见 image_enhance v3.3 / lsc_shading v2）。
//     · 另有 8 条违例（WNS -0.283ns）属跨时钟域，已在 camera_to_dsi_display.sdc
//       用反向 set_false_path 解耦（见 sdc 注释）。
//     · 字模查表在 osd_char_lib.v 内完成，本模块只输出「字符码 + 行号」
//
//  四、还原方法
//     · .OSD_INFO_EN(1'b0) ⇒ 只留 Logo，三处信息全部不显示
//     · 端口 I_ae_y 保留但当前不显示（Y_avg 诊断量，需要时可直接接进来）
//=====================================================================
module hdmi_mixer #(
    parameter H_OFFSET = 128,
    parameter V_OFFSET = 60,
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720,
    parameter integer DEBUG_MODE = 0,
    //------------------------------------------------------------------
    //  ★2026-10-08 OSD 信息叠加总开关
    //    1 = 常驻显示三处中文 OSD（帧计数 / AE 增益 / 当前模式）
    //    0 = 仅保留安路 Logo，不叠加任何信息（交付/演示可关）
    //------------------------------------------------------------------
    parameter OSD_INFO_EN = 1'b1
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_video_vsync,
    input  wire        I_video_hsync,
    input  wire        I_video_de,
    input  wire        I_video_user,
    input  wire        I_video_last,

    input  wire [3:0]  I_debug_status,

    // ★2026-10-08 当前显示模式（顶层 S_mode_sel 已在像素时钟域两级同步）
    input  wire [1:0]  I_mode,

    // ★AE 诊断量：Y_avg 预留给后续使用（当前不显示）；ag 用于第 2 行显示
    input  wire [7:0]  I_ae_y,
    input  wire [15:0] I_ae_ag,

    output wire        O_video_rd_en,
    input  wire [23:0] I_video_rd_data,

    output reg         O_hdmi_vsync,
    output reg         O_hdmi_hsync,
    output reg         O_hdmi_de,
    output reg  [23:0] O_hdmi_data
);

    //==================================================================
    // 1. 输出时序整形寄存器（像素数据延时 2 拍，坐标同步延时 2 拍）
    //==================================================================
    reg        S_video_vsync_1d;
    reg        S_video_vsync_2d;
    reg        S_video_hsync_1d;
    reg        S_video_hsync_2d;
    reg        S_video_de_1d;
    reg        S_video_de_2d;
    reg [23:0] S_video_data_1d;
    reg [23:0] S_video_data_2d;

    reg [11:0] S_x;
    reg [11:0] S_y;
    reg [11:0] S_x_1d;
    reg [11:0] S_x_2d;
    reg [11:0] S_y_1d;
    reg [11:0] S_y_2d;

    //==================================================================
    // 2. 帧计数器（32 位二进制）+ 6 位十进制 BCD
    //==================================================================
    reg [31:0] S_frame_cnt;
    reg [3:0]  S_cnt_d0;   // 个位
    reg [3:0]  S_cnt_d1;   // 十位
    reg [3:0]  S_cnt_d2;   // 百位
    reg [3:0]  S_cnt_d3;   // 千位
    reg [3:0]  S_cnt_d4;   // 万位
    reg [3:0]  S_cnt_d5;   // 十万位

    //==================================================================
    // 3. AE 增益 ag（24 MHz 域 → 本域三级同步）+ 3 位十进制 BCD
    //==================================================================
    reg [15:0] S_ag_s0;
    reg [15:0] S_ag_s1;
    reg [15:0] S_ag_s2;
    reg [3:0]  S_ag_d2;    // 百位
    reg [3:0]  S_ag_d1;    // 十位
    reg [3:0]  S_ag_d0;    // 个位

    //==================================================================
    // 4. OSD 渲染中间量
    //==================================================================
    reg        S_osd_hit;
    reg [23:0] S_osd_color;

    // ---- Logo（安路 160x160 @ 0,0）同步 ROM 的两级流水 ----
    //   ★2026-10-09：Logo 也改为「两级流水」，与字符渲染完全同款：
    //     第 1 拍（用 S_x_1d/S_y_1d）：区域判定 + 地址计算 -> 拍末由 ROM 寄存
    //     第 2 拍（用 ROM 已寄存的输出 + S_x_2d）：alpha 判定 + 叠加
    //   旧写法把「比较 + 减法 + 乘法 + 6400 条 case 的 13 级 mux 树」全塞在
    //   第 2 拍直通输出寄存器，Logic-Level = 15 ⇒ setup 违例（WNS -0.083ns）。
    reg        S_logo_hit_r;      // ★第 1 拍寄存：该像素是否落在 Logo 区域内
    wire[24:0] S_logo_pixel;      // ★ROM 同步输出（第 2 拍取用，与 S_x_2d 同像素）

    // ---- 字符渲染两级流水 ----
    //   第 1 拍（用 S_x_1d/S_y_1d）：区域判定 + 字符码查表  → 拍末寄存
    //   第 2 拍（用已寄存值 + S_x_2d[3:0]）：字模查表 + 位选 → 与像素同拍
    //   ⇒ 关键路径从「判定+查表+查表+位选」约 15 级压缩到「查表+位选」约 7 级
    reg [7:0]  S_pf_char_r;       // 送给字库的字符码（已寄存）
    reg [3:0]  S_pf_row_r;        // 送给字库的行号 0..15（已寄存）
    reg        S_pf_hit_r;        // 该像素是否落在 OSD 文字区域内（已寄存）
    wire[15:0] S_pf_row_bits;     // 字库返回的该行点阵

    reg [7:0]  S_mode_c0;         // 模式名首字
    reg [7:0]  S_mode_c1;         // 模式名次字

    //==================================================================
    // 5. 布局常量
    //==================================================================
    localparam LOGO_X = 12'd0;
    localparam LOGO_Y = 12'd0;
    localparam LOGO_W = 12'd160;
    localparam LOGO_H = 12'd160;

    localparam CELL_W = 12'd16;    // 字符格宽
    localparam CELL_H = 12'd16;    // 字符格高

    // ---- 信息区（Logo 右侧，两行）----
    //   行1「帧数:」+6 位数字 = 9 格 = 144 px
    //   行2「增益:」+3 位数字 = 6 格 =  96 px
    localparam INFO_X  = 12'd176;   // = 16 x 11（必须为 16 的整数倍）
    localparam INFO_Y1 = 12'd16;
    localparam INFO_Y2 = 12'd48;
    localparam INFO_W1 = 12'd144;
    localparam INFO_W2 = 12'd96;

    // ---- 模式区（右上角）----
    //   「模式:」+2 个汉字 = 5 格 = 80 px，右对齐（右边距 1280-1264 = 16 px）
    localparam MODE_X  = 12'd1184;  // = 16 x 74（必须为 16 的整数倍）
    localparam MODE_Y  = 12'd16;
    localparam MODE_W  = 12'd80;

    // ---- 字符序号换算基准（= 区域起点 / 16）----
    localparam [7:0] INFO_COL0 = 8'd11;   // 176 / 16
    localparam [7:0] MODE_COL0 = 8'd74;   // 1184 / 16

    localparam COLOR_LOGO_BLUE = 24'h1f315c;
    localparam COLOR_LOGO_RED  = 24'hec1c2d;
    localparam COLOR_DYN_TEXT  = 24'hfff200;   // OSD 文字颜色（黄）

    //==================================================================
    // 6. 字符码常量（与 osd_char_lib.v 保持一致）
    //==================================================================
    localparam CHAR_0     = 8'h30;   // '0'，配合 +digit 生成 0..9
    localparam CHAR_COLON = 8'h3a;   // ':'

    localparam CHAR_ZHEN  = 8'h80;   // 帧
    localparam CHAR_SHU   = 8'h81;   // 数
    localparam CHAR_ZENG  = 8'h82;   // 增
    localparam CHAR_YI    = 8'h83;   // 益
    localparam CHAR_MO    = 8'h84;   // 模
    localparam CHAR_SHI   = 8'h85;   // 式
    localparam CHAR_CAI   = 8'h86;   // 彩
    localparam CHAR_SE    = 8'h87;   // 色
    localparam CHAR_HUI   = 8'h88;   // 灰
    localparam CHAR_DU    = 8'h89;   // 度
    localparam CHAR_ER    = 8'h8a;   // 二
    localparam CHAR_ZHI   = 8'h8b;   // 值
    localparam CHAR_BIAN  = 8'h8c;   // 边
    localparam CHAR_YUAN  = 8'h8d;   // 缘

    //==================================================================
    // 7. 文本查表函数（只做「字符序号 → 字符码」的一级映射）
    //==================================================================
    //   第 1 行： idx 0..8  =  帧 数 : d5 d4 d3 d2 d1 d0
    function [7:0] F_row1_char;
        input [3:0] I_idx;
        begin
            case(I_idx)
                4'd0:    F_row1_char = CHAR_ZHEN;
                4'd1:    F_row1_char = CHAR_SHU;
                4'd2:    F_row1_char = CHAR_COLON;
                4'd3:    F_row1_char = CHAR_0 + S_cnt_d5;
                4'd4:    F_row1_char = CHAR_0 + S_cnt_d4;
                4'd5:    F_row1_char = CHAR_0 + S_cnt_d3;
                4'd6:    F_row1_char = CHAR_0 + S_cnt_d2;
                4'd7:    F_row1_char = CHAR_0 + S_cnt_d1;
                default: F_row1_char = CHAR_0 + S_cnt_d0;
            endcase
        end
    endfunction

    //   第 2 行： idx 0..5  =  增 益 : d2 d1 d0
    function [7:0] F_row2_char;
        input [3:0] I_idx;
        begin
            case(I_idx)
                4'd0:    F_row2_char = CHAR_ZENG;
                4'd1:    F_row2_char = CHAR_YI;
                4'd2:    F_row2_char = CHAR_COLON;
                4'd3:    F_row2_char = CHAR_0 + S_ag_d2;
                4'd4:    F_row2_char = CHAR_0 + S_ag_d1;
                default: F_row2_char = CHAR_0 + S_ag_d0;
            endcase
        end
    endfunction

    //   模式区： idx 0..4  =  模 式 : c0 c1
    function [7:0] F_mode_char;
        input [3:0] I_idx;
        begin
            case(I_idx)
                4'd0:    F_mode_char = CHAR_MO;
                4'd1:    F_mode_char = CHAR_SHI;
                4'd2:    F_mode_char = CHAR_COLON;
                4'd3:    F_mode_char = S_mode_c0;
                default: F_mode_char = S_mode_c1;
            endcase
        end
    endfunction

    //==================================================================
    // 8. 组合逻辑：区域判定 + 字符序号（第 1 拍，用 S_x_1d / S_y_1d）
    //    ★用 1d 而不是 2d 的原因：这两步（比较器 + 8 位减法 + 字符码 case）
    //      在拍末被寄存，使第 2 拍只剩「字模查表 + 位选」，关键路径减半。
    //      时序上完全等价：t 边沿采样 S_x_1d = X0，同时 S_x_2d 更新为 X0，
    //      所以「寄存后的字符码」与「第 2 拍的 S_x_2d / 像素数据」严格同像素。
    //==================================================================
    wire W_row1_area = (S_x_1d >= INFO_X) && (S_x_1d < (INFO_X + INFO_W1)) &&
                       (S_y_1d >= INFO_Y1) && (S_y_1d < (INFO_Y1 + CELL_H));
    wire W_row2_area = (S_x_1d >= INFO_X) && (S_x_1d < (INFO_X + INFO_W2)) &&
                       (S_y_1d >= INFO_Y2) && (S_y_1d < (INFO_Y2 + CELL_H));
    wire W_mode_area = (S_x_1d >= MODE_X) && (S_x_1d < (MODE_X + MODE_W)) &&
                       (S_y_1d >= MODE_Y) && (S_y_1d < (MODE_Y + CELL_H));

    // 字符序号 = 全局列号 - 区域起始列号（两个区域起点都是 16 的倍数）
    wire [7:0] W_info_col = S_x_1d[11:4] - INFO_COL0;
    wire [7:0] W_mode_col = S_x_1d[11:4] - MODE_COL0;

    //-------------------- Logo 区域与地址（第 1 拍，用 S_x_1d / S_y_1d）----------
    //  ★地址 = S_y*LOGO_W + S_x，LOGO_W = 160 = 128 + 32
    //    ⇒ 用两次左移相加（{y,7'b0} + {y,5'b0}），**不引入乘法器**。
    //    旧写法 `S_logo_y * LOGO_W` 被综合成 DSPMULT（34/40 个 DSP 之一），
    //    DSP 的 c2q + 乘法 + 输出寄存共 ~2.1ns，直接吃掉本就很紧的 13.33ns 预算。
    //  ★不做「地址门外加 mux 归零」：越界时地址虽任意，但 S_logo_hit_r（已寄存的
    //    区域判定）在第 2 拍会否掉它，逻辑等价且少一级、少一片 mux。
    wire W_logo_area = (S_x_1d >= LOGO_X) && (S_x_1d < (LOGO_X + LOGO_W)) &&
                       (S_y_1d >= LOGO_Y) && (S_y_1d < (LOGO_Y + LOGO_H));

    //  ★★★ 拼接铁律：【字段里尾随 0 的个数 = 左移位数】。
    //      y*128 ⇒ 尾随 7 个 0 ； y*32 ⇒ 尾随 5 个 0 ； y*160 = y*128 + y*32。
    //  ★★★ 2026-10-09 事故（安路 Logo 整块消失，上板实拍确认）：
    //      此处曾误写成 `{3'b000,y,4'd0}` + `{5'b00000,y,2'd0}`，
    //      实际 = y*16 + y*4 = y*20 ⇒ Logo 区内最大地址仅 159*20+159 = 3339，
    //      而 ROM 第一个非默认项在 15'd5494 ⇒ 读地址全落在 default(=0)
    //      ⇒ O_pixel[24]=0 ⇒ 不叠加 ⇒ Logo 不显示（文字 OSD 不受影响）。
    //      教训：验证脚本绝不能手写「应有」的公式，必须从 RTL 原文解析。
    wire [14:0] W_logo_addr =
                     ( {S_y_1d[7:0],       7'd0}       // y * 128  (= y << 7)
                     + {2'b00, S_y_1d[7:0], 5'd0}     // y *  32  (= y << 5)
                     + {4'd0,  S_x_1d[7:0]} );        // x

    // AE 增益钳位到 8 位后送显示（ag 正常范围 16..239，此钳位只是保险）
    wire [7:0] W_ae_ag = (S_ag_s2 > 16'd255) ? 8'd255 : S_ag_s2[7:0];

    //==================================================================
    // 9. 子模块
    //==================================================================
    assign O_video_rd_en = I_video_de;

    anlogic_logo_rom u_anlogic_logo_rom(
        .I_clk   ( I_clk         ),   // ★同步读（地址已在第 1 拍算好）
        .I_addr  ( W_logo_addr   ),
        .O_pixel ( S_logo_pixel  )
    );

    osd_char_lib u_osd_char_lib(
        .I_char     ( S_pf_char_r   ),
        .I_row      ( S_pf_row_r    ),
        .O_row_bits ( S_pf_row_bits )
    );

    //==================================================================
    // 10. 像素坐标计数器（0 起，每行/每帧回绕）
    //==================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_user) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_de) begin
            if(I_video_last) begin
                S_x <= 12'd0;
                S_y <= S_y + 12'd1;
            end
            else begin
                S_x <= S_x + 12'd1;
            end
        end
    end

    //==================================================================
    // 11. ag 三级同步（24 MHz → 74.25 MHz；仅显示用）
    //     多比特直接同步不保证一致性，但 ag 最快 4 帧才变一次，
    //     偶发取到过渡值只影响一帧显示，无功能影响。
    //==================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_ag_s0 <= 16'd0;
            S_ag_s1 <= 16'd0;
            S_ag_s2 <= 16'd0;
        end
        else begin
            S_ag_s0 <= I_ae_ag;
            S_ag_s1 <= S_ag_s0;
            S_ag_s2 <= S_ag_s1;
        end
    end

    //==================================================================
    // 12. 帧计数器 + 帧计数 BCD（每帧 I_video_user 走一次）
    //==================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_frame_cnt <= 32'd0;
            S_cnt_d0 <= 4'd0;
            S_cnt_d1 <= 4'd0;
            S_cnt_d2 <= 4'd0;
            S_cnt_d3 <= 4'd0;
            S_cnt_d4 <= 4'd0;
            S_cnt_d5 <= 4'd0;
        end
        else if(I_video_user) begin
            S_frame_cnt <= S_frame_cnt + 32'd1;
            if(S_cnt_d0 == 4'd9) begin
                S_cnt_d0 <= 4'd0;
                if(S_cnt_d1 == 4'd9) begin
                    S_cnt_d1 <= 4'd0;
                    if(S_cnt_d2 == 4'd9) begin
                        S_cnt_d2 <= 4'd0;
                        if(S_cnt_d3 == 4'd9) begin
                            S_cnt_d3 <= 4'd0;
                            if(S_cnt_d4 == 4'd9) begin
                                S_cnt_d4 <= 4'd0;
                                if(S_cnt_d5 == 4'd9)
                                    S_cnt_d5 <= 4'd0;
                                else
                                    S_cnt_d5 <= S_cnt_d5 + 4'd1;
                            end
                            else
                                S_cnt_d4 <= S_cnt_d4 + 4'd1;
                        end
                        else
                            S_cnt_d3 <= S_cnt_d3 + 4'd1;
                    end
                    else
                        S_cnt_d2 <= S_cnt_d2 + 4'd1;
                end
                else
                    S_cnt_d1 <= S_cnt_d1 + 4'd1;
            end
            else
                S_cnt_d0 <= S_cnt_d0 + 4'd1;
        end
    end

    //==================================================================
    // 13. AE 增益 BCD（每帧刷新一次，与帧计数同节拍）
    //     8 位 ÷ 100 与 ÷ 10 都是「8 位除以常数」，TD 综合为
    //     乘法+移位网络，深度与旧版 OSD 相同（旧版即用同款写法）。
    //==================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_ag_d2 <= 4'd0;
            S_ag_d1 <= 4'd0;
            S_ag_d0 <= 4'd0;
        end
        else if(I_video_user) begin
            S_ag_d2 <= W_ae_ag / 8'd100;
            S_ag_d1 <= (W_ae_ag % 8'd100) / 8'd10;
            S_ag_d0 <= W_ae_ag % 8'd10;
        end
    end

    //==================================================================
    // 14. 模式名查表（1 级 mux，浅）
    //==================================================================
    always @(*) begin
        case(I_mode)
            2'b00:   begin S_mode_c0 = CHAR_CAI;  S_mode_c1 = CHAR_SE;   end  // 彩色
            2'b01:   begin S_mode_c0 = CHAR_HUI;  S_mode_c1 = CHAR_DU;   end  // 灰度
            2'b10:   begin S_mode_c0 = CHAR_ER;   S_mode_c1 = CHAR_ZHI;  end  // 二值
            default: begin S_mode_c0 = CHAR_BIAN; S_mode_c1 = CHAR_YUAN; end  // 边缘
        endcase
    end

    //==================================================================
    // 15. 第 1 拍预计算：区域判定 + 字符码/行号 → 拍末寄存
    //     命中区域时更新；未命中时保持（省一个 mux，且字库输入更稳定）。
    //     命中与否本身也寄存为 S_pf_hit_r，第 2 拍与 S_video_de_2d 一起门控。
    //==================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_pf_char_r  <= 8'd0;
            S_pf_row_r   <= 4'd0;
            S_pf_hit_r   <= 1'b0;
            S_logo_hit_r <= 1'b0;
        end
        else begin
            S_pf_hit_r <= OSD_INFO_EN &&
                          (W_row1_area || W_row2_area || W_mode_area);
            S_pf_row_r <= S_y_1d[3:0];

            // ★Logo 命中判定同拍寄存：本拍地址送给 ROM，ROM 输出下一拍才有效，
            //   而下一拍的 S_x_2d 正好也是这个像素 ⇒ 命中标志与像素严格对齐。
            //   注意：Logo 是常驻图层，不受 OSD_INFO_EN 门控（置 0 只关三处文字）。
            S_logo_hit_r <= W_logo_area;

            if(W_row1_area)
                S_pf_char_r <= F_row1_char(W_info_col[3:0]);
            else if(W_row2_area)
                S_pf_char_r <= F_row2_char(W_info_col[3:0]);
            else if(W_mode_area)
                S_pf_char_r <= F_mode_char(W_mode_col[3:0]);
        end
    end

    //==================================================================
    // 16. 坐标延时（与像素数据同拍对齐）
    //==================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x_1d <= 12'd0;
            S_x_2d <= 12'd0;
            S_y_1d <= 12'd0;
            S_y_2d <= 12'd0;
        end
        else begin
            S_x_1d <= S_x;
            S_x_2d <= S_x_1d;
            S_y_1d <= S_y;
            S_y_2d <= S_y_1d;
        end
    end

    //==================================================================
    // 17. OSD 叠加判定（第 2 拍，纯组合）
    //     ★2026-10-09：Logo 也已流到第 1 拍，这里只剩「ROM 输出 + alpha + mux」
    //       与「字模位选 + mux」，关键路径从 15 级降到约 3 级。
    //     优先级：Logo > 文字信息。Logo 区 (0,0)-(160,160)，文字区 X >= 176，
    //     两区在几何上永不重叠 ⇒ 用 else-if 与旧「logo 命中则跳过文字」完全等价。
    //==================================================================
    always @(*) begin
        S_osd_hit   = 1'b0;
        S_osd_color = 24'd0;

        if(S_video_de_2d) begin

            //------------------ 17.1 安路 Logo（160x160 @ 0,0）------------------
            //   S_logo_hit_r : 第 1 拍已判定「该像素落在 Logo 区域内」
            //   S_logo_pixel : 同步 ROM 的输出，与本拍 S_x_2d / 像素数据同像素
            if(S_logo_hit_r && S_logo_pixel[24]) begin
                S_osd_hit   = 1'b1;
                S_osd_color = S_logo_pixel[23:0];
            end

            //------------------ 17.2 文字信息（帧计数 / AE 增益 / 模式）----------
            //   S_pf_hit_r  : 第 1 拍已判定「该像素落在某个文字区域内」
            //   S_pf_char_r : 第 1 拍已查出字符码（多路选一，已寄存）
            //   S_pf_row_r  : 第 1 拍已取出字模行号（= 行内 y 的低 4 位）
            //   位选        : 取第 15-lx 位；4 位数据下 15-lx 恒等于 ~lx
            else if(S_pf_hit_r && S_pf_row_bits[~S_x_2d[3:0]]) begin
                S_osd_hit   = 1'b1;
                S_osd_color = COLOR_DYN_TEXT;
            end
        end
    end

    //==================================================================
    // 17. 输出级（时序整形 + OSD 叠加 / 调试模式）
    //==================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_video_vsync_1d <= 1'b0;
            S_video_vsync_2d <= 1'b0;
            S_video_hsync_1d <= 1'b0;
            S_video_hsync_2d <= 1'b0;
            S_video_de_1d    <= 1'b0;
            S_video_de_2d    <= 1'b0;
            O_hdmi_vsync     <= 1'b0;
            O_hdmi_hsync     <= 1'b0;
            O_hdmi_de        <= 1'b0;
            O_hdmi_data      <= 24'd0;
        end
        else begin
            S_video_vsync_1d <= I_video_vsync;
            S_video_vsync_2d <= S_video_vsync_1d;
            O_hdmi_vsync     <= S_video_vsync_2d;

            S_video_hsync_1d <= I_video_hsync;
            S_video_hsync_2d <= S_video_hsync_1d;
            O_hdmi_hsync     <= S_video_hsync_2d;

            S_video_de_1d    <= I_video_de;
            S_video_de_2d    <= S_video_de_1d;
            O_hdmi_de        <= S_video_de_2d;

            S_video_data_1d  <= I_video_rd_data;
            S_video_data_2d  <= S_video_data_1d;

            if(DEBUG_MODE == 1) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd128)
                        O_hdmi_data <= 24'hff0000;
                    else if(S_x < 12'd256)
                        O_hdmi_data <= 24'h00ff00;
                    else if(S_x < 12'd384)
                        O_hdmi_data <= 24'h0000ff;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= 24'hffff00;
                    else if(S_x < 12'd640)
                        O_hdmi_data <= 24'h00ffff;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= 24'hff00ff;
                    else if(S_x < 12'd896)
                        O_hdmi_data <= 24'hffffff;
                    else
                        O_hdmi_data <= 24'h202020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 2) begin
                if(S_video_de_2d) begin
                    case (I_debug_status)
                        4'b0000: O_hdmi_data <= 24'h000000;
                        4'b0001: O_hdmi_data <= 24'hff0000;
                        4'b0011: O_hdmi_data <= 24'hffff00;
                        4'b0111: O_hdmi_data <= 24'h00ff00;
                        4'b1111: O_hdmi_data <= I_video_rd_data;
                        default: O_hdmi_data <= 24'h0000ff;
                    endcase
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 3) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd256)
                        O_hdmi_data <= I_debug_status[0] ? 24'hff8000 : 24'h201000;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= I_debug_status[1] ? 24'hffff00 : 24'h202000;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= I_debug_status[2] ? 24'hff0000 : 24'h200000;
                    else
                        O_hdmi_data <= I_debug_status[3] ? 24'h00ffff : 24'h002020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else begin
                if(S_osd_hit)
                    O_hdmi_data <= S_osd_color;
                else
                    O_hdmi_data <= S_video_data_2d;
            end
        end
    end

endmodule
