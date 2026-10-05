//========================================================================
// 文件名称：gamma_lut.v
// 模块功能：8bit RGB Gamma 校正（线性域 -> sRGB 显示域）
//
// 为什么需要这个模块（问题定位）：
//   1) SC520CS 输出的 RAW 是线性数据，像素值正比于入射光强；
//   2) HDMI 显示器/电视按 sRGB（约 gamma=2.2）的非线性编码显示图像；
//   3) 现有通路 RAW10 -> Demosaic -> AWB -> DDR -> HDMI 中没有任何 Gamma
//      环节，线性数据直接上屏，导致：
//        - 中间调被严重压暗（画面"发暗"）
//        - 暗部细节挤在一起、黑位不黑（画面"雾感"）
//        - 暗部亮度被压缩，色彩显得"闷 / 不鲜艳"
//   4) 手机拍照正常，是因为手机 ISP 内部做了同样的 Gamma 校正。
//
// 校正公式：
//   out = round( 255 * (in / 255) ** (1 / 2.2) )
//   例：in=128（线性 50% 亮度）-> out=186，屏幕观感才是"中间灰"。
//
// 实现方式：
//   - 256 项常量查找表（ROM），R/G/B 三通道各查一次表，同一拍完成；
//   - 纯组合逻辑 ROM + 一级输出寄存，总延迟固定 1 个时钟周期，
//     与原 brightness_gain 模块延迟完全一致，因此不会破坏
//     灰度 -> 二值化 / Sobel 的数据对齐关系。
//
// 时钟域：S_hdmi_pixel_clk（74.25MHz，720p60），与 brightness_gain 相同
// 端口：  与 brightness_gain 完全兼容，可直接替换
//
// 【本文件由 gen_gamma_lut.py 自动生成，请勿手工修改 case 表】
//========================================================================

//------------------------------------------------------------------------
// 单通道 256x8 Gamma 查找表（组合逻辑 ROM，综合为分布式 LUTRAM）
//------------------------------------------------------------------------
module gamma_rom_256x8 (
    input  wire [7:0] addr,     // 输入线性像素值 0~255
    output reg  [7:0] dout      // 输出 gamma 校正后的像素值 0~255
);

    always @(*) begin
        case (addr)
            8'd0  : dout = 8'd0;
            8'd1  : dout = 8'd21;
            8'd2  : dout = 8'd28;
            8'd3  : dout = 8'd34;
            8'd4  : dout = 8'd39;
            8'd5  : dout = 8'd43;
            8'd6  : dout = 8'd46;
            8'd7  : dout = 8'd50;
            8'd8  : dout = 8'd53;
            8'd9  : dout = 8'd56;
            8'd10 : dout = 8'd59;
            8'd11 : dout = 8'd61;
            8'd12 : dout = 8'd64;
            8'd13 : dout = 8'd66;
            8'd14 : dout = 8'd68;
            8'd15 : dout = 8'd70;
            8'd16 : dout = 8'd72;
            8'd17 : dout = 8'd74;
            8'd18 : dout = 8'd76;
            8'd19 : dout = 8'd78;
            8'd20 : dout = 8'd80;
            8'd21 : dout = 8'd82;
            8'd22 : dout = 8'd84;
            8'd23 : dout = 8'd85;
            8'd24 : dout = 8'd87;
            8'd25 : dout = 8'd89;
            8'd26 : dout = 8'd90;
            8'd27 : dout = 8'd92;
            8'd28 : dout = 8'd93;
            8'd29 : dout = 8'd95;
            8'd30 : dout = 8'd96;
            8'd31 : dout = 8'd98;
            8'd32 : dout = 8'd99;
            8'd33 : dout = 8'd101;
            8'd34 : dout = 8'd102;
            8'd35 : dout = 8'd103;
            8'd36 : dout = 8'd105;
            8'd37 : dout = 8'd106;
            8'd38 : dout = 8'd107;
            8'd39 : dout = 8'd109;
            8'd40 : dout = 8'd110;
            8'd41 : dout = 8'd111;
            8'd42 : dout = 8'd112;
            8'd43 : dout = 8'd114;
            8'd44 : dout = 8'd115;
            8'd45 : dout = 8'd116;
            8'd46 : dout = 8'd117;
            8'd47 : dout = 8'd118;
            8'd48 : dout = 8'd119;
            8'd49 : dout = 8'd120;
            8'd50 : dout = 8'd122;
            8'd51 : dout = 8'd123;
            8'd52 : dout = 8'd124;
            8'd53 : dout = 8'd125;
            8'd54 : dout = 8'd126;
            8'd55 : dout = 8'd127;
            8'd56 : dout = 8'd128;
            8'd57 : dout = 8'd129;
            8'd58 : dout = 8'd130;
            8'd59 : dout = 8'd131;
            8'd60 : dout = 8'd132;
            8'd61 : dout = 8'd133;
            8'd62 : dout = 8'd134;
            8'd63 : dout = 8'd135;
            8'd64 : dout = 8'd136;
            8'd65 : dout = 8'd137;
            8'd66 : dout = 8'd138;
            8'd67 : dout = 8'd139;
            8'd68 : dout = 8'd140;
            8'd69 : dout = 8'd141;
            8'd70 : dout = 8'd142;
            8'd71 : dout = 8'd143;
            8'd72 : dout = 8'd144;
            8'd73 : dout = 8'd144;
            8'd74 : dout = 8'd145;
            8'd75 : dout = 8'd146;
            8'd76 : dout = 8'd147;
            8'd77 : dout = 8'd148;
            8'd78 : dout = 8'd149;
            8'd79 : dout = 8'd150;
            8'd80 : dout = 8'd151;
            8'd81 : dout = 8'd151;
            8'd82 : dout = 8'd152;
            8'd83 : dout = 8'd153;
            8'd84 : dout = 8'd154;
            8'd85 : dout = 8'd155;
            8'd86 : dout = 8'd156;
            8'd87 : dout = 8'd156;
            8'd88 : dout = 8'd157;
            8'd89 : dout = 8'd158;
            8'd90 : dout = 8'd159;
            8'd91 : dout = 8'd160;
            8'd92 : dout = 8'd160;
            8'd93 : dout = 8'd161;
            8'd94 : dout = 8'd162;
            8'd95 : dout = 8'd163;
            8'd96 : dout = 8'd164;
            8'd97 : dout = 8'd164;
            8'd98 : dout = 8'd165;
            8'd99 : dout = 8'd166;
            8'd100: dout = 8'd167;
            8'd101: dout = 8'd167;
            8'd102: dout = 8'd168;
            8'd103: dout = 8'd169;
            8'd104: dout = 8'd170;
            8'd105: dout = 8'd170;
            8'd106: dout = 8'd171;
            8'd107: dout = 8'd172;
            8'd108: dout = 8'd173;
            8'd109: dout = 8'd173;
            8'd110: dout = 8'd174;
            8'd111: dout = 8'd175;
            8'd112: dout = 8'd175;
            8'd113: dout = 8'd176;
            8'd114: dout = 8'd177;
            8'd115: dout = 8'd178;
            8'd116: dout = 8'd178;
            8'd117: dout = 8'd179;
            8'd118: dout = 8'd180;
            8'd119: dout = 8'd180;
            8'd120: dout = 8'd181;
            8'd121: dout = 8'd182;
            8'd122: dout = 8'd182;
            8'd123: dout = 8'd183;
            8'd124: dout = 8'd184;
            8'd125: dout = 8'd184;
            8'd126: dout = 8'd185;
            8'd127: dout = 8'd186;
            8'd128: dout = 8'd186;
            8'd129: dout = 8'd187;
            8'd130: dout = 8'd188;
            8'd131: dout = 8'd188;
            8'd132: dout = 8'd189;
            8'd133: dout = 8'd190;
            8'd134: dout = 8'd190;
            8'd135: dout = 8'd191;
            8'd136: dout = 8'd192;
            8'd137: dout = 8'd192;
            8'd138: dout = 8'd193;
            8'd139: dout = 8'd194;
            8'd140: dout = 8'd194;
            8'd141: dout = 8'd195;
            8'd142: dout = 8'd195;
            8'd143: dout = 8'd196;
            8'd144: dout = 8'd197;
            8'd145: dout = 8'd197;
            8'd146: dout = 8'd198;
            8'd147: dout = 8'd199;
            8'd148: dout = 8'd199;
            8'd149: dout = 8'd200;
            8'd150: dout = 8'd200;
            8'd151: dout = 8'd201;
            8'd152: dout = 8'd202;
            8'd153: dout = 8'd202;
            8'd154: dout = 8'd203;
            8'd155: dout = 8'd203;
            8'd156: dout = 8'd204;
            8'd157: dout = 8'd205;
            8'd158: dout = 8'd205;
            8'd159: dout = 8'd206;
            8'd160: dout = 8'd206;
            8'd161: dout = 8'd207;
            8'd162: dout = 8'd207;
            8'd163: dout = 8'd208;
            8'd164: dout = 8'd209;
            8'd165: dout = 8'd209;
            8'd166: dout = 8'd210;
            8'd167: dout = 8'd210;
            8'd168: dout = 8'd211;
            8'd169: dout = 8'd212;
            8'd170: dout = 8'd212;
            8'd171: dout = 8'd213;
            8'd172: dout = 8'd213;
            8'd173: dout = 8'd214;
            8'd174: dout = 8'd214;
            8'd175: dout = 8'd215;
            8'd176: dout = 8'd215;
            8'd177: dout = 8'd216;
            8'd178: dout = 8'd217;
            8'd179: dout = 8'd217;
            8'd180: dout = 8'd218;
            8'd181: dout = 8'd218;
            8'd182: dout = 8'd219;
            8'd183: dout = 8'd219;
            8'd184: dout = 8'd220;
            8'd185: dout = 8'd220;
            8'd186: dout = 8'd221;
            8'd187: dout = 8'd221;
            8'd188: dout = 8'd222;
            8'd189: dout = 8'd223;
            8'd190: dout = 8'd223;
            8'd191: dout = 8'd224;
            8'd192: dout = 8'd224;
            8'd193: dout = 8'd225;
            8'd194: dout = 8'd225;
            8'd195: dout = 8'd226;
            8'd196: dout = 8'd226;
            8'd197: dout = 8'd227;
            8'd198: dout = 8'd227;
            8'd199: dout = 8'd228;
            8'd200: dout = 8'd228;
            8'd201: dout = 8'd229;
            8'd202: dout = 8'd229;
            8'd203: dout = 8'd230;
            8'd204: dout = 8'd230;
            8'd205: dout = 8'd231;
            8'd206: dout = 8'd231;
            8'd207: dout = 8'd232;
            8'd208: dout = 8'd232;
            8'd209: dout = 8'd233;
            8'd210: dout = 8'd233;
            8'd211: dout = 8'd234;
            8'd212: dout = 8'd234;
            8'd213: dout = 8'd235;
            8'd214: dout = 8'd235;
            8'd215: dout = 8'd236;
            8'd216: dout = 8'd236;
            8'd217: dout = 8'd237;
            8'd218: dout = 8'd237;
            8'd219: dout = 8'd238;
            8'd220: dout = 8'd238;
            8'd221: dout = 8'd239;
            8'd222: dout = 8'd239;
            8'd223: dout = 8'd240;
            8'd224: dout = 8'd240;
            8'd225: dout = 8'd241;
            8'd226: dout = 8'd241;
            8'd227: dout = 8'd242;
            8'd228: dout = 8'd242;
            8'd229: dout = 8'd243;
            8'd230: dout = 8'd243;
            8'd231: dout = 8'd244;
            8'd232: dout = 8'd244;
            8'd233: dout = 8'd245;
            8'd234: dout = 8'd245;
            8'd235: dout = 8'd246;
            8'd236: dout = 8'd246;
            8'd237: dout = 8'd247;
            8'd238: dout = 8'd247;
            8'd239: dout = 8'd248;
            8'd240: dout = 8'd248;
            8'd241: dout = 8'd249;
            8'd242: dout = 8'd249;
            8'd243: dout = 8'd249;
            8'd244: dout = 8'd250;
            8'd245: dout = 8'd250;
            8'd246: dout = 8'd251;
            8'd247: dout = 8'd251;
            8'd248: dout = 8'd252;
            8'd249: dout = 8'd252;
            8'd250: dout = 8'd253;
            8'd251: dout = 8'd253;
            8'd252: dout = 8'd254;
            8'd253: dout = 8'd254;
            8'd254: dout = 8'd255;
            8'd255: dout = 8'd255;
            default: dout = 8'd0;
        endcase
    end

endmodule

//------------------------------------------------------------------------
// 顶层：24bit RGB888 = 黑电平扣除 + Gamma 校正
//   rgb_in  = {R[7:0], G[7:0], B[7:0]}（线性域）
//   rgb_out = {R'[7:0], G'[7:0], B'[7:0]}（显示域，相对输入延迟 1 拍）
//
// 处理顺序：先扣黑电平（线性域偏移）-> 再做 Gamma（非线性映射）
//
// 参数说明（都可以在不重新连线的情况下改，方便上板 A/B 对比）：
//   EN_GAMMA = 1 -> 启用 Gamma 校正；0 -> 只做黑电平扣除
//   EN_BLC   = 1 -> 启用黑电平扣除；0 -> 关闭
//   BL_LEVEL = 8bit 域的黑电平偏移量。SC520CS 数据手册未给出确定值，
//              思特威 10bit RAW 的典型值为 64，右移 2 位后 = 16。
//              调试方法：遮住镜头看画面，若发灰就调大，若暗部出现
//              "死黑块 + 亮噪点"说明减过头了，就调小；设为 0 即关闭。
//------------------------------------------------------------------------
module gamma_lut #(
    parameter       EN_GAMMA = 1'b1,         // 1: 启用 Gamma；0: 旁路
    parameter       EN_BLC   = 1'b1,         // 1: 启用黑电平扣除；0: 旁路
    parameter [7:0] BL_LEVEL = 8'd16         // 8bit 域黑电平（= RAW10 域 64>>2）
) (
    input  wire        clk,                 // 像素时钟
    input  wire        rst_n,               // 低电平复位
    input  wire [23:0] rgb_in,              // 输入 RGB888（线性域）
    output reg  [23:0] rgb_out              // 输出 RGB888（显示域）
);

    // R/G/B 三通道拆分
    wire [7:0] r_in = rgb_in[23:16];
    wire [7:0] g_in = rgb_in[15:8];
    wire [7:0] b_in = rgb_in[7:0];

    // 第一步：扣除黑电平
    //   注意一定要"限幅"再减：无符号数下溢会翻转成大数，
    //   在暗部变成刺眼的亮噪点，这比不扣黑电平还糟。
    wire [7:0] r_bl = EN_BLC ? ((r_in > BL_LEVEL) ? (r_in - BL_LEVEL) : 8'd0) : r_in;
    wire [7:0] g_bl = EN_BLC ? ((g_in > BL_LEVEL) ? (g_in - BL_LEVEL) : 8'd0) : g_in;
    wire [7:0] b_bl = EN_BLC ? ((b_in > BL_LEVEL) ? (b_in - BL_LEVEL) : 8'd0) : b_in;

    // 第二步：Gamma 查表。三通道输入值不同，因此需要 3 份独立 ROM
    wire [7:0] r_out;
    wire [7:0] g_out;
    wire [7:0] b_out;

    gamma_rom_256x8 u_rom_r (.addr(r_bl), .dout(r_out));
    gamma_rom_256x8 u_rom_g (.addr(g_bl), .dout(g_out));
    gamma_rom_256x8 u_rom_b (.addr(b_bl), .dout(b_out));

    // 输出寄存一拍，保持与原 brightness_gain 完全相同的流水线延迟
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rgb_out <= 24'd0;
        else if (EN_GAMMA)
            rgb_out <= {r_out, g_out, b_out};   // 黑电平 + Gamma
        else
            rgb_out <= {r_bl, g_bl, b_bl};      // 只扣黑电平（A/B 对比用）
    end

endmodule
