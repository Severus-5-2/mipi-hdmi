//=====================================================================
//  osd_char_lib.v  ——  OSD 16x16 字符点阵库
//---------------------------------------------------------------------
//  接口：I_char(字符码) + I_row(行号 0..15) -> O_row_bits(该行 16 位点阵)
//        点阵方向：bit15 = 最左像素，bit0 = 最右像素
//
//  ★2026-10-08 改版（配合中文 OSD）：
//    ① 新增 14 个 16x16 汉字字模（0x80..0x8d）：
//       帧 数 增 益 模 式 彩 色 灰 度 二 值 边 缘
//       来源：Windows 宋体内嵌点阵 16px 逐像素提取（tools/gen_cn16.py），
//       保证字形正确、笔画清晰，非手工绘制。
//    ② 删除全部 26 个英文字母（A-Z）与旧「安/路」汉字字模。
//       依据：全仓库检索确认 osd_char_lib 只被 hdmi_mixer 一处实例化，
//       而改版后的 hdmi_mixer 只使用【数字 0-9 + 冒号 + 14 个新汉字】，
//       字母与「安/路」已无任何引用 ⇒ 删掉可显著减少 LUT（原字库 39 个
//       字形 → 现 25 个字形），是一条零风险的资源优化。
//    ③ 数字 0-9 与冒号的字模【保持原样不动】，避免改变既有显示风格。
//
//  说明：数字字模有效区为 bit13..bit2（约 12 列居中），新汉字为满 16 列，
//        两者行号都用 0..15 直接用，显示时 1:1 映射（无缩放、无除法）。
//=====================================================================
module osd_char_lib (
    input  wire [7:0]  I_char,
    input  wire [3:0]  I_row,
    output reg  [15:0] O_row_bits
);

    //------------------------------------------------------------------
    // 字符码定义
    //   0x30..0x39 = '0'..'9'（十进制数字，配合 CHAR_0 + digit 使用）
    //   0x3a       = ':'
    //   0x80..0x8d = 汉字
    //------------------------------------------------------------------
    localparam CHAR_0     = 8'h30;
    localparam CHAR_1     = 8'h31;
    localparam CHAR_2     = 8'h32;
    localparam CHAR_3     = 8'h33;
    localparam CHAR_4     = 8'h34;
    localparam CHAR_5     = 8'h35;
    localparam CHAR_6     = 8'h36;
    localparam CHAR_7     = 8'h37;
    localparam CHAR_8     = 8'h38;
    localparam CHAR_9     = 8'h39;
    localparam CHAR_COLON = 8'h3a;

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

    always @(*) begin
        case(I_char)
            //====================== 数字 0-9 ======================
            CHAR_0: begin
                case(I_row)
                    4'd1:  O_row_bits = 16'h1ff8;
                    4'd2:  O_row_bits = 16'h1ff8;
                    4'd3:  O_row_bits = 16'h300c;
                    4'd4:  O_row_bits = 16'h300c;
                    4'd5:  O_row_bits = 16'h300c;
                    4'd8:  O_row_bits = 16'h300c;
                    4'd9:  O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h300c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_1: begin
                case(I_row)
                    4'd1:  O_row_bits = 16'h000c;
                    4'd2:  O_row_bits = 16'h000c;
                    4'd3:  O_row_bits = 16'h000c;
                    4'd4:  O_row_bits = 16'h000c;
                    4'd5:  O_row_bits = 16'h000c;
                    4'd8:  O_row_bits = 16'h000c;
                    4'd9:  O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h000c;
                    4'd12: O_row_bits = 16'h000c;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_2: begin
                case(I_row)
                    4'd1:  O_row_bits = 16'h1ff8;
                    4'd2:  O_row_bits = 16'h1ff8;
                    4'd3:  O_row_bits = 16'h000c;
                    4'd4:  O_row_bits = 16'h000c;
                    4'd5:  O_row_bits = 16'h000c;
                    4'd6:  O_row_bits = 16'h1ff8;
                    4'd7:  O_row_bits = 16'h1ff8;
                    4'd8:  O_row_bits = 16'h3000;
                    4'd9:  O_row_bits = 16'h3000;
                    4'd10: O_row_bits = 16'h3000;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_3: begin
                case(I_row)
                    4'd1:  O_row_bits = 16'h1ff8;
                    4'd2:  O_row_bits = 16'h1ff8;
                    4'd3:  O_row_bits = 16'h000c;
                    4'd4:  O_row_bits = 16'h000c;
                    4'd5:  O_row_bits = 16'h000c;
                    4'd6:  O_row_bits = 16'h1ff8;
                    4'd7:  O_row_bits = 16'h1ff8;
                    4'd8:  O_row_bits = 16'h000c;
                    4'd9:  O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_4: begin
                case(I_row)
                    4'd1:  O_row_bits = 16'h300c;
                    4'd2:  O_row_bits = 16'h300c;
                    4'd3:  O_row_bits = 16'h300c;
                    4'd4:  O_row_bits = 16'h300c;
                    4'd5:  O_row_bits = 16'h300c;
                    4'd6:  O_row_bits = 16'h1ff8;
                    4'd7:  O_row_bits = 16'h1ff8;
                    4'd8:  O_row_bits = 16'h000c;
                    4'd9:  O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h000c;
                    4'd12: O_row_bits = 16'h000c;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_5: begin
                case(I_row)
                    4'd1:  O_row_bits = 16'h1ff8;
                    4'd2:  O_row_bits = 16'h1ff8;
                    4'd3:  O_row_bits = 16'h3000;
                    4'd4:  O_row_bits = 16'h3000;
                    4'd5:  O_row_bits = 16'h3000;
                    4'd6:  O_row_bits = 16'h1ff8;
                    4'd7:  O_row_bits = 16'h1ff8;
                    4'd8:  O_row_bits = 16'h000c;
                    4'd9:  O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_6: begin
                case(I_row)
                    4'd1:  O_row_bits = 16'h1ff8;
                    4'd2:  O_row_bits = 16'h1ff8;
                    4'd3:  O_row_bits = 16'h3000;
                    4'd4:  O_row_bits = 16'h3000;
                    4'd5:  O_row_bits = 16'h3000;
                    4'd6:  O_row_bits = 16'h1ff8;
                    4'd7:  O_row_bits = 16'h1ff8;
                    4'd8:  O_row_bits = 16'h300c;
                    4'd9:  O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h300c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_7: begin
                case(I_row)
                    4'd1:  O_row_bits = 16'h1ff8;
                    4'd2:  O_row_bits = 16'h1ff8;
                    4'd3:  O_row_bits = 16'h000c;
                    4'd4:  O_row_bits = 16'h000c;
                    4'd5:  O_row_bits = 16'h000c;
                    4'd8:  O_row_bits = 16'h000c;
                    4'd9:  O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h000c;
                    4'd12: O_row_bits = 16'h000c;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_8: begin
                case(I_row)
                    4'd1:  O_row_bits = 16'h1ff8;
                    4'd2:  O_row_bits = 16'h1ff8;
                    4'd3:  O_row_bits = 16'h300c;
                    4'd4:  O_row_bits = 16'h300c;
                    4'd5:  O_row_bits = 16'h300c;
                    4'd6:  O_row_bits = 16'h1ff8;
                    4'd7:  O_row_bits = 16'h1ff8;
                    4'd8:  O_row_bits = 16'h300c;
                    4'd9:  O_row_bits = 16'h300c;
                    4'd10: O_row_bits = 16'h300c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_9: begin
                case(I_row)
                    4'd1:  O_row_bits = 16'h1ff8;
                    4'd2:  O_row_bits = 16'h1ff8;
                    4'd3:  O_row_bits = 16'h300c;
                    4'd4:  O_row_bits = 16'h300c;
                    4'd5:  O_row_bits = 16'h300c;
                    4'd6:  O_row_bits = 16'h1ff8;
                    4'd7:  O_row_bits = 16'h1ff8;
                    4'd8:  O_row_bits = 16'h000c;
                    4'd9:  O_row_bits = 16'h000c;
                    4'd10: O_row_bits = 16'h000c;
                    4'd11: O_row_bits = 16'h1ff8;
                    4'd12: O_row_bits = 16'h1ff8;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_COLON: begin
                case(I_row)
                    4'd6:  O_row_bits = 16'h0180;
                    4'd7:  O_row_bits = 16'h0180;
                    4'd10: O_row_bits = 16'h0180;
                    4'd11: O_row_bits = 16'h0180;
                    default: O_row_bits = 16'h0000;
                endcase
            end

            //====================== 汉字 16x16 ====================
            CHAR_ZHEN: begin   // 帧
                case(I_row)
                    4'd0:  O_row_bits = 16'h1020;
                    4'd1:  O_row_bits = 16'h1020;
                    4'd2:  O_row_bits = 16'h103e;
                    4'd3:  O_row_bits = 16'h7c20;
                    4'd4:  O_row_bits = 16'h5420;
                    4'd5:  O_row_bits = 16'h55fc;
                    4'd6:  O_row_bits = 16'h5504;
                    4'd7:  O_row_bits = 16'h5524;
                    4'd8:  O_row_bits = 16'h5524;
                    4'd9:  O_row_bits = 16'h5524;
                    4'd10: O_row_bits = 16'h5524;
                    4'd11: O_row_bits = 16'h5d24;
                    4'd12: O_row_bits = 16'h1050;
                    4'd13: O_row_bits = 16'h1048;
                    4'd14: O_row_bits = 16'h1084;
                    4'd15: O_row_bits = 16'h1104;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_SHU: begin    // 数
                case(I_row)
                    4'd0:  O_row_bits = 16'h0820;
                    4'd1:  O_row_bits = 16'h4920;
                    4'd2:  O_row_bits = 16'h2a20;
                    4'd3:  O_row_bits = 16'h083e;
                    4'd4:  O_row_bits = 16'hff44;
                    4'd5:  O_row_bits = 16'h2a44;
                    4'd6:  O_row_bits = 16'h4944;
                    4'd7:  O_row_bits = 16'h88a4;
                    4'd8:  O_row_bits = 16'h1028;
                    4'd9:  O_row_bits = 16'hfe28;
                    4'd10: O_row_bits = 16'h2210;
                    4'd11: O_row_bits = 16'h4210;
                    4'd12: O_row_bits = 16'h6428;
                    4'd13: O_row_bits = 16'h1828;
                    4'd14: O_row_bits = 16'h3444;
                    4'd15: O_row_bits = 16'hc282;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_ZENG: begin   // 增
                case(I_row)
                    4'd0:  O_row_bits = 16'h2208;
                    4'd1:  O_row_bits = 16'h2110;
                    4'd3:  O_row_bits = 16'h27fc;
                    4'd4:  O_row_bits = 16'h2444;
                    4'd5:  O_row_bits = 16'hfd54;
                    4'd6:  O_row_bits = 16'h24e4;
                    4'd7:  O_row_bits = 16'h2444;
                    4'd8:  O_row_bits = 16'h27fc;
                    4'd10: O_row_bits = 16'h23f8;
                    4'd11: O_row_bits = 16'h3a08;
                    4'd12: O_row_bits = 16'he3f8;
                    4'd13: O_row_bits = 16'h4208;
                    4'd14: O_row_bits = 16'h03f8;
                    4'd15: O_row_bits = 16'h0208;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_YI: begin     // 益
                case(I_row)
                    4'd0:  O_row_bits = 16'h1010;
                    4'd1:  O_row_bits = 16'h0820;
                    4'd3:  O_row_bits = 16'hfffe;
                    4'd5:  O_row_bits = 16'h0820;
                    4'd6:  O_row_bits = 16'h1010;
                    4'd7:  O_row_bits = 16'h2008;
                    4'd8:  O_row_bits = 16'h4004;
                    4'd9:  O_row_bits = 16'h3ff8;
                    4'd10: O_row_bits = 16'h2448;
                    4'd11: O_row_bits = 16'h2448;
                    4'd12: O_row_bits = 16'h2448;
                    4'd13: O_row_bits = 16'h2448;
                    4'd14: O_row_bits = 16'hfffe;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_MO: begin     // 模
                case(I_row)
                    4'd0:  O_row_bits = 16'h1110;
                    4'd1:  O_row_bits = 16'h1110;
                    4'd2:  O_row_bits = 16'h17fc;
                    4'd3:  O_row_bits = 16'h1110;
                    4'd4:  O_row_bits = 16'hfc00;
                    4'd5:  O_row_bits = 16'h13f8;
                    4'd6:  O_row_bits = 16'h3208;
                    4'd7:  O_row_bits = 16'h3bf8;
                    4'd8:  O_row_bits = 16'h5608;
                    4'd9:  O_row_bits = 16'h53f8;
                    4'd10: O_row_bits = 16'h9040;
                    4'd11: O_row_bits = 16'h17fc;
                    4'd12: O_row_bits = 16'h10a0;
                    4'd13: O_row_bits = 16'h1110;
                    4'd14: O_row_bits = 16'h1208;
                    4'd15: O_row_bits = 16'h1406;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_SHI: begin    // 式
                case(I_row)
                    4'd0:  O_row_bits = 16'h0048;
                    4'd1:  O_row_bits = 16'h0044;
                    4'd2:  O_row_bits = 16'h0044;
                    4'd3:  O_row_bits = 16'h0040;
                    4'd4:  O_row_bits = 16'hfffe;
                    4'd5:  O_row_bits = 16'h0040;
                    4'd6:  O_row_bits = 16'h0040;
                    4'd7:  O_row_bits = 16'h3e40;
                    4'd8:  O_row_bits = 16'h0840;
                    4'd9:  O_row_bits = 16'h0840;
                    4'd10: O_row_bits = 16'h0820;
                    4'd11: O_row_bits = 16'h0822;
                    4'd12: O_row_bits = 16'h0f12;
                    4'd13: O_row_bits = 16'h780a;
                    4'd14: O_row_bits = 16'h2006;
                    4'd15: O_row_bits = 16'h0002;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_CAI: begin    // 彩
                case(I_row)
                    4'd0:  O_row_bits = 16'h0100;
                    4'd1:  O_row_bits = 16'h0784;
                    4'd2:  O_row_bits = 16'h7804;
                    4'd3:  O_row_bits = 16'h0888;
                    4'd4:  O_row_bits = 16'h4490;
                    4'd5:  O_row_bits = 16'h2522;
                    4'd6:  O_row_bits = 16'h2002;
                    4'd7:  O_row_bits = 16'h0404;
                    4'd8:  O_row_bits = 16'h7f88;
                    4'd9:  O_row_bits = 16'h0c10;
                    4'd10: O_row_bits = 16'h1622;
                    4'd11: O_row_bits = 16'h1502;
                    4'd12: O_row_bits = 16'h2484;
                    4'd13: O_row_bits = 16'h4408;
                    4'd14: O_row_bits = 16'h8410;
                    4'd15: O_row_bits = 16'h0460;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_SE: begin     // 色
                case(I_row)
                    4'd0:  O_row_bits = 16'h0800;
                    4'd1:  O_row_bits = 16'h0800;
                    4'd2:  O_row_bits = 16'h1fe0;
                    4'd3:  O_row_bits = 16'h2020;
                    4'd4:  O_row_bits = 16'h4040;
                    4'd5:  O_row_bits = 16'hbff8;
                    4'd6:  O_row_bits = 16'h2108;
                    4'd7:  O_row_bits = 16'h2108;
                    4'd8:  O_row_bits = 16'h2108;
                    4'd9:  O_row_bits = 16'h3ff8;
                    4'd10: O_row_bits = 16'h2000;
                    4'd11: O_row_bits = 16'h2002;
                    4'd12: O_row_bits = 16'h2002;
                    4'd13: O_row_bits = 16'h2002;
                    4'd14: O_row_bits = 16'h1ffe;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_HUI: begin    // 灰
                case(I_row)
                    4'd0:  O_row_bits = 16'h0400;
                    4'd1:  O_row_bits = 16'h0400;
                    4'd2:  O_row_bits = 16'h0400;
                    4'd3:  O_row_bits = 16'hfffe;
                    4'd4:  O_row_bits = 16'h0800;
                    4'd5:  O_row_bits = 16'h0880;
                    4'd6:  O_row_bits = 16'h0884;
                    4'd7:  O_row_bits = 16'h1284;
                    4'd8:  O_row_bits = 16'h1288;
                    4'd9:  O_row_bits = 16'h2490;
                    4'd10: O_row_bits = 16'h2940;
                    4'd11: O_row_bits = 16'h4140;
                    4'd12: O_row_bits = 16'h8220;
                    4'd13: O_row_bits = 16'h0410;
                    4'd14: O_row_bits = 16'h1808;
                    4'd15: O_row_bits = 16'h6006;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_DU: begin     // 度
                case(I_row)
                    4'd0:  O_row_bits = 16'h0100;
                    4'd1:  O_row_bits = 16'h0080;
                    4'd2:  O_row_bits = 16'h3ffe;
                    4'd3:  O_row_bits = 16'h2220;
                    4'd4:  O_row_bits = 16'h2220;
                    4'd5:  O_row_bits = 16'h3ffc;
                    4'd6:  O_row_bits = 16'h2220;
                    4'd7:  O_row_bits = 16'h2220;
                    4'd8:  O_row_bits = 16'h23e0;
                    4'd9:  O_row_bits = 16'h2000;
                    4'd10: O_row_bits = 16'h2ff0;
                    4'd11: O_row_bits = 16'h2410;
                    4'd12: O_row_bits = 16'h4220;
                    4'd13: O_row_bits = 16'h41c0;
                    4'd14: O_row_bits = 16'h8630;
                    4'd15: O_row_bits = 16'h380e;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_ER: begin     // 二
                case(I_row)
                    4'd3:  O_row_bits = 16'h3ff8;
                    4'd12: O_row_bits = 16'hfffe;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_ZHI: begin    // 值
                case(I_row)
                    4'd0:  O_row_bits = 16'h0840;
                    4'd1:  O_row_bits = 16'h0840;
                    4'd2:  O_row_bits = 16'h0ffc;
                    4'd3:  O_row_bits = 16'h1040;
                    4'd4:  O_row_bits = 16'h1040;
                    4'd5:  O_row_bits = 16'h33f8;
                    4'd6:  O_row_bits = 16'h3208;
                    4'd7:  O_row_bits = 16'h53f8;
                    4'd8:  O_row_bits = 16'h9208;
                    4'd9:  O_row_bits = 16'h13f8;
                    4'd10: O_row_bits = 16'h1208;
                    4'd11: O_row_bits = 16'h13f8;
                    4'd12: O_row_bits = 16'h1208;
                    4'd13: O_row_bits = 16'h1208;
                    4'd14: O_row_bits = 16'h1ffe;
                    4'd15: O_row_bits = 16'h1000;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_BIAN: begin   // 边
                case(I_row)
                    4'd0:  O_row_bits = 16'h0040;
                    4'd1:  O_row_bits = 16'h2040;
                    4'd2:  O_row_bits = 16'h1040;
                    4'd3:  O_row_bits = 16'h13fc;
                    4'd4:  O_row_bits = 16'h0044;
                    4'd5:  O_row_bits = 16'h0044;
                    4'd6:  O_row_bits = 16'hf044;
                    4'd7:  O_row_bits = 16'h1084;
                    4'd8:  O_row_bits = 16'h1084;
                    4'd9:  O_row_bits = 16'h1104;
                    4'd10: O_row_bits = 16'h1104;
                    4'd11: O_row_bits = 16'h1228;
                    4'd12: O_row_bits = 16'h1410;
                    4'd13: O_row_bits = 16'h2800;
                    4'd14: O_row_bits = 16'h47fe;
                    default: O_row_bits = 16'h0000;
                endcase
            end
            CHAR_YUAN: begin   // 缘
                case(I_row)
                    4'd0:  O_row_bits = 16'h1080;
                    4'd1:  O_row_bits = 16'h10fc;
                    4'd2:  O_row_bits = 16'h2104;
                    4'd3:  O_row_bits = 16'h21f8;
                    4'd4:  O_row_bits = 16'h4808;
                    4'd5:  O_row_bits = 16'hfbfe;
                    4'd6:  O_row_bits = 16'h1040;
                    4'd7:  O_row_bits = 16'h20a2;
                    4'd8:  O_row_bits = 16'h4334;
                    4'd9:  O_row_bits = 16'hf858;
                    4'd10: O_row_bits = 16'h4094;
                    4'd11: O_row_bits = 16'h0334;
                    4'd12: O_row_bits = 16'h1852;
                    4'd13: O_row_bits = 16'he090;
                    4'd14: O_row_bits = 16'h4350;
                    4'd15: O_row_bits = 16'h0020;
                    default: O_row_bits = 16'h0000;
                endcase
            end

            default: O_row_bits = 16'h0000;
        endcase
    end

endmodule
