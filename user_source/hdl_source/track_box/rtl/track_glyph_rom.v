//=====================================================================
//  track_glyph_rom.v  ——  画框 / 坐标 OSD 专用 16x16 字模
//---------------------------------------------------------------------
//  ★本文件由 sim/glyph_rom_tool.py 自动生成，**不要手改**。
//    改字形请编辑脚本里的 GLYPHS 表，然后：
//        python sim/glyph_rom_tool.py gen      # 重新生成
//        python sim/glyph_rom_tool.py check    # 从 RTL 原文渲染校验
//
//  为什么另起一张 ROM 而不复用 osd_char_lib.v
//    osd_char_lib.v 在 2026-10-08 的中文 OSD 改版中删掉了全部 26 个
//    英文字母（只留 0-9 + ':' + 14 个汉字）。坐标 OSD 要显示
//    「X:0640 Y:0360」，X / Y 无处可取。为不改动郭的顶层共享文件，
//    本模块自带只含所需字形的独立 ROM。
//    若后续郭愿意把 X / Y 两个字模并进 osd_char_lib.v，可直接删掉本文件。
//
//  接口    I_char(字符码) + I_row(行号 0..15)  ->  O_row_bits(该行 16bit 点阵)
//  点阵方向 bit15 = 最左像素（与 osd_char_lib.v 一致）
//  字形占位 数字 8x12 @ 列4..11、行2..13；X/Y 10x12 @ 列3..12；':' @ 列7..8
//  纯组合读（同 osd_char_lib.v 的写法，由调用方在第 2 拍取用）
//=====================================================================
module track_glyph_rom (
    input  wire [7:0]  I_char,
    input  wire [3:0]  I_row,
    output reg  [15:0] O_row_bits
);

    // 字符码：0x30..0x39 = '0'..'9'，0x3a = ':'，0x58 = 'X'，0x59 = 'Y'
    always @(*) begin
        case(I_char)
            //================= '0' (0x30) =================
            8'h30: begin
                case(I_row)
                    4'd2: O_row_bits = 16'h03c0;
                    4'd3: O_row_bits = 16'h0420;
                    4'd4: O_row_bits = 16'h0810;
                    4'd5: O_row_bits = 16'h0810;
                    4'd6: O_row_bits = 16'h0810;
                    4'd7: O_row_bits = 16'h0810;
                    4'd8: O_row_bits = 16'h0810;
                    4'd9: O_row_bits = 16'h0810;
                    4'd10: O_row_bits = 16'h0810;
                    4'd11: O_row_bits = 16'h0810;
                    4'd12: O_row_bits = 16'h0420;
                    4'd13: O_row_bits = 16'h03c0;
                    default: O_row_bits = 16'h0000;
                endcase
            end

            //================= '1' (0x31) =================
            8'h31: begin
                case(I_row)
                    4'd2: O_row_bits = 16'h0180;
                    4'd3: O_row_bits = 16'h0380;
                    4'd4: O_row_bits = 16'h0780;
                    4'd5: O_row_bits = 16'h0180;
                    4'd6: O_row_bits = 16'h0180;
                    4'd7: O_row_bits = 16'h0180;
                    4'd8: O_row_bits = 16'h0180;
                    4'd9: O_row_bits = 16'h0180;
                    4'd10: O_row_bits = 16'h0180;
                    4'd11: O_row_bits = 16'h0180;
                    4'd12: O_row_bits = 16'h0180;
                    4'd13: O_row_bits = 16'h07e0;
                    default: O_row_bits = 16'h0000;
                endcase
            end

            //================= '2' (0x32) =================
            8'h32: begin
                case(I_row)
                    4'd2: O_row_bits = 16'h03c0;
                    4'd3: O_row_bits = 16'h0420;
                    4'd4: O_row_bits = 16'h0810;
                    4'd5: O_row_bits = 16'h0010;
                    4'd6: O_row_bits = 16'h0020;
                    4'd7: O_row_bits = 16'h0040;
                    4'd8: O_row_bits = 16'h0080;
                    4'd9: O_row_bits = 16'h0100;
                    4'd10: O_row_bits = 16'h0200;
                    4'd11: O_row_bits = 16'h0400;
                    4'd12: O_row_bits = 16'h0800;
                    4'd13: O_row_bits = 16'h0ff0;
                    default: O_row_bits = 16'h0000;
                endcase
            end

            //================= '3' (0x33) =================
            8'h33: begin
                case(I_row)
                    4'd2: O_row_bits = 16'h03c0;
                    4'd3: O_row_bits = 16'h0420;
                    4'd4: O_row_bits = 16'h0810;
                    4'd5: O_row_bits = 16'h0010;
                    4'd6: O_row_bits = 16'h0010;
                    4'd7: O_row_bits = 16'h01e0;
                    4'd8: O_row_bits = 16'h0010;
                    4'd9: O_row_bits = 16'h0010;
                    4'd10: O_row_bits = 16'h0010;
                    4'd11: O_row_bits = 16'h0810;
                    4'd12: O_row_bits = 16'h0420;
                    4'd13: O_row_bits = 16'h03c0;
                    default: O_row_bits = 16'h0000;
                endcase
            end

            //================= '4' (0x34) =================
            8'h34: begin
                case(I_row)
                    4'd2: O_row_bits = 16'h0040;
                    4'd3: O_row_bits = 16'h00c0;
                    4'd4: O_row_bits = 16'h01c0;
                    4'd5: O_row_bits = 16'h02c0;
                    4'd6: O_row_bits = 16'h04c0;
                    4'd7: O_row_bits = 16'h08c0;
                    4'd8: O_row_bits = 16'h08c0;
                    4'd9: O_row_bits = 16'h0ff0;
                    4'd10: O_row_bits = 16'h00c0;
                    4'd11: O_row_bits = 16'h00c0;
                    4'd12: O_row_bits = 16'h00c0;
                    4'd13: O_row_bits = 16'h00c0;
                    default: O_row_bits = 16'h0000;
                endcase
            end

            //================= '5' (0x35) =================
            8'h35: begin
                case(I_row)
                    4'd2: O_row_bits = 16'h0ff0;
                    4'd3: O_row_bits = 16'h0800;
                    4'd4: O_row_bits = 16'h0800;
                    4'd5: O_row_bits = 16'h0800;
                    4'd6: O_row_bits = 16'h0800;
                    4'd7: O_row_bits = 16'h0be0;
                    4'd8: O_row_bits = 16'h0810;
                    4'd9: O_row_bits = 16'h0010;
                    4'd10: O_row_bits = 16'h0010;
                    4'd11: O_row_bits = 16'h0810;
                    4'd12: O_row_bits = 16'h0420;
                    4'd13: O_row_bits = 16'h03c0;
                    default: O_row_bits = 16'h0000;
                endcase
            end

            //================= '6' (0x36) =================
            8'h36: begin
                case(I_row)
                    4'd2: O_row_bits = 16'h01e0;
                    4'd3: O_row_bits = 16'h0200;
                    4'd4: O_row_bits = 16'h0400;
                    4'd5: O_row_bits = 16'h0800;
                    4'd6: O_row_bits = 16'h0800;
                    4'd7: O_row_bits = 16'h0bc0;
                    4'd8: O_row_bits = 16'h0c20;
                    4'd9: O_row_bits = 16'h0810;
                    4'd10: O_row_bits = 16'h0810;
                    4'd11: O_row_bits = 16'h0810;
                    4'd12: O_row_bits = 16'h0420;
                    4'd13: O_row_bits = 16'h03c0;
                    default: O_row_bits = 16'h0000;
                endcase
            end

            //================= '7' (0x37) =================
            8'h37: begin
                case(I_row)
                    4'd2: O_row_bits = 16'h0ff0;
                    4'd3: O_row_bits = 16'h0010;
                    4'd4: O_row_bits = 16'h0020;
                    4'd5: O_row_bits = 16'h0040;
                    4'd6: O_row_bits = 16'h0080;
                    4'd7: O_row_bits = 16'h0100;
                    4'd8: O_row_bits = 16'h0200;
                    4'd9: O_row_bits = 16'h0200;
                    4'd10: O_row_bits = 16'h0400;
                    4'd11: O_row_bits = 16'h0400;
                    4'd12: O_row_bits = 16'h0400;
                    4'd13: O_row_bits = 16'h0400;
                    default: O_row_bits = 16'h0000;
                endcase
            end

            //================= '8' (0x38) =================
            8'h38: begin
                case(I_row)
                    4'd2: O_row_bits = 16'h03c0;
                    4'd3: O_row_bits = 16'h0420;
                    4'd4: O_row_bits = 16'h0810;
                    4'd5: O_row_bits = 16'h0810;
                    4'd6: O_row_bits = 16'h0420;
                    4'd7: O_row_bits = 16'h03c0;
                    4'd8: O_row_bits = 16'h0420;
                    4'd9: O_row_bits = 16'h0810;
                    4'd10: O_row_bits = 16'h0810;
                    4'd11: O_row_bits = 16'h0810;
                    4'd12: O_row_bits = 16'h0420;
                    4'd13: O_row_bits = 16'h03c0;
                    default: O_row_bits = 16'h0000;
                endcase
            end

            //================= '9' (0x39) =================
            8'h39: begin
                case(I_row)
                    4'd2: O_row_bits = 16'h03c0;
                    4'd3: O_row_bits = 16'h0420;
                    4'd4: O_row_bits = 16'h0810;
                    4'd5: O_row_bits = 16'h0810;
                    4'd6: O_row_bits = 16'h0810;
                    4'd7: O_row_bits = 16'h0410;
                    4'd8: O_row_bits = 16'h03e0;
                    4'd9: O_row_bits = 16'h0010;
                    4'd10: O_row_bits = 16'h0010;
                    4'd11: O_row_bits = 16'h0810;
                    4'd12: O_row_bits = 16'h0420;
                    4'd13: O_row_bits = 16'h03c0;
                    default: O_row_bits = 16'h0000;
                endcase
            end

            //================= ':' (0x3a) =================
            8'h3a: begin
                case(I_row)
                    4'd5: O_row_bits = 16'h0180;
                    4'd6: O_row_bits = 16'h0180;
                    4'd9: O_row_bits = 16'h0180;
                    4'd10: O_row_bits = 16'h0180;
                    default: O_row_bits = 16'h0000;
                endcase
            end

            //================= 'X' (0x58) =================
            8'h58: begin
                case(I_row)
                    4'd2: O_row_bits = 16'h1818;
                    4'd3: O_row_bits = 16'h0c30;
                    4'd4: O_row_bits = 16'h0660;
                    4'd5: O_row_bits = 16'h03c0;
                    4'd6: O_row_bits = 16'h0180;
                    4'd7: O_row_bits = 16'h0180;
                    4'd8: O_row_bits = 16'h0180;
                    4'd9: O_row_bits = 16'h0180;
                    4'd10: O_row_bits = 16'h03c0;
                    4'd11: O_row_bits = 16'h0660;
                    4'd12: O_row_bits = 16'h0c30;
                    4'd13: O_row_bits = 16'h1818;
                    default: O_row_bits = 16'h0000;
                endcase
            end

            //================= 'Y' (0x59) =================
            8'h59: begin
                case(I_row)
                    4'd2: O_row_bits = 16'h1818;
                    4'd3: O_row_bits = 16'h0c30;
                    4'd4: O_row_bits = 16'h0660;
                    4'd5: O_row_bits = 16'h03c0;
                    4'd6: O_row_bits = 16'h0180;
                    4'd7: O_row_bits = 16'h0180;
                    4'd8: O_row_bits = 16'h0180;
                    4'd9: O_row_bits = 16'h0180;
                    4'd10: O_row_bits = 16'h0180;
                    4'd11: O_row_bits = 16'h0180;
                    4'd12: O_row_bits = 16'h0180;
                    4'd13: O_row_bits = 16'h0180;
                    default: O_row_bits = 16'h0000;
                endcase
            end

            default: O_row_bits = 16'h0000;
        endcase
    end

endmodule
