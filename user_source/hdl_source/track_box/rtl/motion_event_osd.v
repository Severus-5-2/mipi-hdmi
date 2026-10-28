// Read-only display of the motion monitor. Two pixel clocks of latency,
// matching box_draw and the external overlay input of hdmi_mixer.
module motion_event_osd #(
    parameter integer DISPLAY_NR_AB=0,
    parameter integer ROI_X0=320, ROI_X1=960,
    parameter integer ROI_Y0=192, ROI_Y1=576
)(
    input wire I_clk,I_rst_n,I_de,
    input wire [10:0] I_pix_x,
    input wire [9:0] I_pix_y,
    input wire I_color_mode, // SW4: color status / regional alarm
    input wire I_features_enable, // Only true for the RGB display mode
    input wire [1:0] I_display_mode, // 00 RGB, 01 gray, 10 binary, 11 edge
    input wire [11:0] I_counts,
    input wire [1:0] I_nr_level,
    input wire I_scene_busy,
    input wire [3:0] I_tnr_pages,input wire I_tnr_fault,
    input wire [7:0] I_tnr_statistics, // Frame-held source, synchronized by TOP
    input wire I_display_nr_active, // Previous complete display epoch had any averaging
    input wire [27:0] I_ae_quality_status,
    input wire I_cal_enable,
    input wire [359:0] I_cal_bcd,
    input wire [1:0] I_cal_ready,
    input wire I_alarm,I_ready,
    input wire [7:0] I_events_bcd,
    input wire [11:0] I_roi_bcd,
    input wire I_det_full,
    input wire [1:0] I_red_state,I_blue_state,
    output wire O_hit,
    output wire [23:0] O_color
);
    // Slow frame-held MIPI status crosses only into the read-only overlay.
    // Two synchronizers plus stable-sample/frame-boundary acceptance avoid
    // a changing NR label during one displayed frame; level3 is invalid.
    reg [1:0] nr_meta,nr_sync,nr_prev,nr_frame;
    reg [27:0] qs_meta,qs_sync,qs_prev,qs_frame;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin nr_meta<=0;nr_sync<=0;nr_prev<=0;nr_frame<=0;qs_meta<=0;qs_sync<=0;qs_prev<=0;qs_frame<=0;end
        else begin
            nr_meta<=I_nr_level;nr_sync<=nr_meta;nr_prev<=nr_sync;
            qs_meta<=I_ae_quality_status;qs_sync<=qs_meta;qs_prev<=qs_sync;
            if(I_de && I_pix_x==0 && I_pix_y==0 && qs_sync==qs_prev) qs_frame<=qs_sync;
            if(I_de && I_pix_x==0 && I_pix_y==0 && nr_sync==nr_prev && nr_sync!=3)
                nr_frame<=nr_sync;
        end
    end
    // Accept stable multi-bit statistics only at the visible frame boundary.
    reg [7:0] statistics_prev,statistics_frame;
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin statistics_prev<=0;statistics_frame<=0;end
        else begin
            statistics_prev<=I_tnr_statistics;
            if(I_de && I_pix_x==0 && I_pix_y==0 && statistics_prev==I_tnr_statistics)
                statistics_frame<=I_tnr_statistics;
        end
    end
    wire [7:0] valid_digit=8'h30+{4'd0,statistics_frame[7:4]};
    wire [7:0] average_digit=8'h30+{4'd0,statistics_frame[3:0]};
    wire [7:0] tnr_page_hex=(I_tnr_pages<10)?8'h30+I_tnr_pages:8'h41+(I_tnr_pages-10);
    // Private 8-bit glyph IDs; every display cell remains exactly 16 pixels.
    wire [7:0] tnr_state=I_tnr_fault?8'h8c:I_display_nr_active?8'h8a:8'h8b;
    function [31:0] state_word;
        input [1:0] state;
        begin case(state)
            0:state_word={8'h8b,8'h20,8'h20,8'h20};
            1:state_word={8'h8f,8'h90,8'h20,8'h20};
            2:state_word={8'h91,8'h92,8'h20,8'h20};
            default:state_word={8'h93,8'h94,8'h20,8'h20};
        endcase end
    endfunction
    wire [31:0] zone_word=I_alarm?{8'ha1,8'ha2,8'h20,8'h20}:I_ready?{8'ha3,8'ha4,8'h20,8'h20}:{8'h93,8'h94,8'h20,8'h20};
    reg [31:0] display_word;
    always @* case(I_display_mode)
        2'd0:display_word={8'h80,8'h81,8'h20,8'h20};
        2'd1:display_word={8'h82,8'h83,8'h20,8'h20};
        2'd2:display_word={8'h84,8'h85,8'h20,8'h20};
        2'd3:display_word={8'h86,8'h87,8'h20,8'h20};
        default:display_word={8'h80,8'h81,8'h20,8'h20};
    endcase
    wire [7:0] event_tens={4'd0,I_events_bcd[7:4]}+8'h30;
    wire [7:0] event_ones={4'd0,I_events_bcd[3:0]}+8'h30;
    reg [127:0] line_text;
    always @* begin
        case(I_pix_y[6:4])
            3'd5:line_text=DISPLAY_NR_AB?{display_word,8'h88,8'h89,8'h3a,tnr_state,8'h20,8'h8a,8'hb4,valid_digit,8'hb5,8'h89,average_digit,8'h20}:{display_word,8'hb2,8'hb3,8'h3a,(8'h30+{6'd0,nr_frame}),8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20};
            3'd6:begin
                if(!I_features_enable) line_text={8'h95,8'h96,8'h3a,8'hb0,8'hb1,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20};
                else if(I_color_mode) line_text={8'h8d,8'h81,8'h3a,state_word(I_red_state),8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20};
                else line_text={8'h9f,8'ha0,8'h3a,zone_word,8'h20,8'ha5,8'ha6,8'h3a,event_tens,event_ones,8'h20,8'h20,8'h20};
            end
            3'd7:begin
                if(!I_features_enable) line_text={8'haf,8'ha2,8'h3a,8'hb0,8'hb1,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20};
                else if(I_color_mode) line_text={8'h8e,8'h81,8'h3a,state_word(I_blue_state),8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20};
                else line_text={8'ha7,8'h92,8'h3a,({4'd0,I_roi_bcd[11:8]}+8'h30),({4'd0,I_roi_bcd[7:4]}+8'h30),({4'd0,I_roi_bcd[3:0]}+8'h30),8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20};
            end
            3'd0:begin
                if(!I_features_enable) line_text=128'h20202020202020202020202020202020;
                else if(I_color_mode) line_text={8'h95,8'h96,8'h3a,(I_det_full?{8'h99,8'h9a}:{8'h97,8'h98}),8'h20,8'h9b,8'h9c,8'h3a,(I_scene_busy?"1":"0"),8'h20,8'h20,8'h20,8'h20,8'h20,8'h20};
                else line_text={8'ha8,8'h92,8'h3a,8'ha9,8'haa,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20};
            end
            3'd1:begin
                if(!I_features_enable) line_text={8'h9e,8'h3a,tnr_page_hex,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20};
                else if(I_color_mode) line_text={8'h53,8'h57,8'h34,8'h3a,8'h31,8'h20,8'h8d,(8'h30+{4'd0,I_counts[0 +:4]}),8'h8e,(8'h30+{4'd0,I_counts[4 +:4]}),8'h9d,(8'h30+{4'd0,I_counts[8 +:4]}),8'h9e,8'h3a,tnr_page_hex,8'h20};
                else line_text={8'h53,8'h57,8'h34,8'h3a,8'h30,8'h20,8'h9e,8'h3a,tnr_page_hex,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20};
            end
            default:line_text={8'hab,8'hac,8'h3a,(8'h30+{4'd0,qs_frame[15:12]}),(8'h30+{4'd0,qs_frame[11:8]}),(8'h30+{4'd0,qs_frame[7:4]}),(8'h30+{4'd0,qs_frame[3:0]}),8'h20,8'had,8'hae,8'h3a,(8'h30+{4'd0,qs_frame[27:24]}),(8'h30+{4'd0,qs_frame[23:20]}),(8'h30+{4'd0,qs_frame[19:16]}),8'h20,8'h20};
        endcase
    end
    wire [3:0] col=I_pix_x[7:4]-4'd1;
    wire [7:0] normal_char=line_text[127-{col,3'd0} -:8];
    wire cal_bank=I_pix_x>=1008;
    wire [3:0] cal_col=(I_pix_x-(cal_bank?11'd1008:11'd736))>>4;
    wire [2:0] cal_row=(I_pix_y-10'd80)>>4;
    wire cal_panel=I_cal_enable && I_de && I_pix_y>=64 && I_pix_y<160 &&
        ((I_pix_x>=736 && I_pix_x<992) || (I_pix_x>=1008 && I_pix_x<1264));
    reg [35:0] cal_values;
    always @* begin
        case({cal_bank,cal_row})
            4'd0:cal_values=I_cal_bcd[0 +:36];
            4'd1:cal_values=I_cal_bcd[36 +:36];
            4'd2:cal_values=I_cal_bcd[72 +:36];
            4'd3:cal_values=I_cal_bcd[108 +:36];
            4'd4:cal_values=I_cal_bcd[144 +:36];
            4'd8:cal_values=I_cal_bcd[180 +:36];
            4'd9:cal_values=I_cal_bcd[216 +:36];
            4'd10:cal_values=I_cal_bcd[252 +:36];
            4'd11:cal_values=I_cal_bcd[288 +:36];
            4'd12:cal_values=I_cal_bcd[324 +:36];
            default:cal_values=36'd0;
        endcase
    end
    reg [15:0] cal_label;
    always @* case(cal_row)
        0:cal_label="TL";1:cal_label="TR";2:cal_label="C ";
        3:cal_label="BL";4:cal_label="BR";default:cal_label="  ";
    endcase
    reg [127:0] cal_text;
    always @* begin
        if(I_pix_y<80) cal_text=cal_bank?"D  R   G   B    ":"L  R   G   B    ";
        else if(!I_cal_ready[cal_bank]) cal_text="WAIT            ";
        else cal_text={cal_label," ",
            (8'h30+{4'd0,cal_values[11:8]}),(8'h30+{4'd0,cal_values[7:4]}),(8'h30+{4'd0,cal_values[3:0]})," ",
            (8'h30+{4'd0,cal_values[23:20]}),(8'h30+{4'd0,cal_values[19:16]}),(8'h30+{4'd0,cal_values[15:12]})," ",
            (8'h30+{4'd0,cal_values[35:32]}),(8'h30+{4'd0,cal_values[31:28]}),(8'h30+{4'd0,cal_values[27:24]}),"  "};
    end
    wire [7:0] selected_char=cal_panel?cal_text[127-{cal_col,3'd0} -:8]:normal_char;
    wire panel=I_de && I_pix_x>=11'd16 && I_pix_x<11'd272 &&
        I_pix_y>=10'd592 && I_pix_y<10'd688;
    // Five by seven glyphs scaled by two, centered in a 16 by 16 cell.
    wire glyph_area=I_pix_x[3:0]>=3 && I_pix_x[3:0]<13 && I_pix_y[3:0]<14;
    wire roi_border=I_features_enable && !I_color_mode && I_de && I_pix_x>=ROI_X0 && I_pix_x<ROI_X1 &&
        I_pix_y>=ROI_Y0 && I_pix_y<ROI_Y1 &&
        (I_pix_x<ROI_X0+2 || I_pix_x>=ROI_X1-2 || I_pix_y<ROI_Y0+2 || I_pix_y>=ROI_Y1-2);
    reg [7:0] char_d;
    reg [3:0] cn_col_d;
    reg [2:0] row_d,col_d;
    reg panel_d,roi_d;
    reg cal_bg_d,cal_bg_dd;
    reg [1:0] color_d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            char_d<=0; cn_col_d<=0; row_d<=0; col_d<=0; panel_d<=0; roi_d<=0; color_d<=0;cal_bg_d<=0;cal_bg_dd<=0;
        end else begin
            char_d<=selected_char;
            cn_col_d<=I_pix_x[3:0];
            row_d<=I_pix_y[3:1];
            col_d<=(I_pix_x[3:0]-4'd3)>>1;
            panel_d<=(panel || cal_panel) && (selected_char[7] || glyph_area);
            cal_bg_d<=cal_panel;cal_bg_dd<=cal_bg_d;
            roi_d<=roi_border;
            if(!I_features_enable) color_d<=2'd3;
            else if(I_color_mode) begin
                if(I_pix_y>=608 && I_pix_y<624) color_d<=2'd1;
                else if(I_pix_y>=624 && I_pix_y<640) color_d<=2'd2;
                else if(I_pix_y>=640 && I_pix_y<656 && I_det_full) color_d<=2'd1;
                else color_d<=2'd3;
            end else begin
                if(I_pix_y>=608 && I_pix_y<624) color_d<=I_alarm?2'd1:I_ready?2'd0:2'd3;
                else color_d<=2'd3;
            end
        end
    end

    // CN_ROW_ROM: one 1024x16 synchronous block ROM, no output reset.
    // Existing ASCII/calibration font below is unchanged. bit15 is left.
    (* rom_style = "bram" *) reg [15:0] cn_rows [0:1023];
    reg [15:0] cn_row_d;
    initial begin
        // CN_GLYPH 00 U+5F69 彩
        cn_rows[0] = 16'h0100;
        cn_rows[1] = 16'h0784;
        cn_rows[2] = 16'h7804;
        cn_rows[3] = 16'h0888;
        cn_rows[4] = 16'h4490;
        cn_rows[5] = 16'h2522;
        cn_rows[6] = 16'h2002;
        cn_rows[7] = 16'h0404;
        cn_rows[8] = 16'h7f88;
        cn_rows[9] = 16'h0c10;
        cn_rows[10] = 16'h1622;
        cn_rows[11] = 16'h1502;
        cn_rows[12] = 16'h2484;
        cn_rows[13] = 16'h4408;
        cn_rows[14] = 16'h8410;
        cn_rows[15] = 16'h0460;
        // CN_GLYPH 01 U+8272 色
        cn_rows[16] = 16'h0800;
        cn_rows[17] = 16'h0800;
        cn_rows[18] = 16'h1fe0;
        cn_rows[19] = 16'h2020;
        cn_rows[20] = 16'h4040;
        cn_rows[21] = 16'hbff8;
        cn_rows[22] = 16'h2108;
        cn_rows[23] = 16'h2108;
        cn_rows[24] = 16'h2108;
        cn_rows[25] = 16'h3ff8;
        cn_rows[26] = 16'h2000;
        cn_rows[27] = 16'h2002;
        cn_rows[28] = 16'h2002;
        cn_rows[29] = 16'h2002;
        cn_rows[30] = 16'h1ffe;
        cn_rows[31] = 16'h0000;
        // CN_GLYPH 02 U+7070 灰
        cn_rows[32] = 16'h0400;
        cn_rows[33] = 16'h0400;
        cn_rows[34] = 16'h0400;
        cn_rows[35] = 16'hfffe;
        cn_rows[36] = 16'h0800;
        cn_rows[37] = 16'h0880;
        cn_rows[38] = 16'h0884;
        cn_rows[39] = 16'h1284;
        cn_rows[40] = 16'h1288;
        cn_rows[41] = 16'h2490;
        cn_rows[42] = 16'h2940;
        cn_rows[43] = 16'h4140;
        cn_rows[44] = 16'h8220;
        cn_rows[45] = 16'h0410;
        cn_rows[46] = 16'h1808;
        cn_rows[47] = 16'h6006;
        // CN_GLYPH 03 U+5EA6 度
        cn_rows[48] = 16'h0100;
        cn_rows[49] = 16'h0080;
        cn_rows[50] = 16'h3ffe;
        cn_rows[51] = 16'h2220;
        cn_rows[52] = 16'h2220;
        cn_rows[53] = 16'h3ffc;
        cn_rows[54] = 16'h2220;
        cn_rows[55] = 16'h2220;
        cn_rows[56] = 16'h23e0;
        cn_rows[57] = 16'h2000;
        cn_rows[58] = 16'h2ff0;
        cn_rows[59] = 16'h2410;
        cn_rows[60] = 16'h4220;
        cn_rows[61] = 16'h41c0;
        cn_rows[62] = 16'h8630;
        cn_rows[63] = 16'h380e;
        // CN_GLYPH 04 U+4E8C 二
        cn_rows[64] = 16'h0000;
        cn_rows[65] = 16'h0000;
        cn_rows[66] = 16'h0000;
        cn_rows[67] = 16'h3ff8;
        cn_rows[68] = 16'h0000;
        cn_rows[69] = 16'h0000;
        cn_rows[70] = 16'h0000;
        cn_rows[71] = 16'h0000;
        cn_rows[72] = 16'h0000;
        cn_rows[73] = 16'h0000;
        cn_rows[74] = 16'h0000;
        cn_rows[75] = 16'h0000;
        cn_rows[76] = 16'hfffe;
        cn_rows[77] = 16'h0000;
        cn_rows[78] = 16'h0000;
        cn_rows[79] = 16'h0000;
        // CN_GLYPH 05 U+503C 值
        cn_rows[80] = 16'h0840;
        cn_rows[81] = 16'h0840;
        cn_rows[82] = 16'h0ffc;
        cn_rows[83] = 16'h1040;
        cn_rows[84] = 16'h1040;
        cn_rows[85] = 16'h33f8;
        cn_rows[86] = 16'h3208;
        cn_rows[87] = 16'h53f8;
        cn_rows[88] = 16'h9208;
        cn_rows[89] = 16'h13f8;
        cn_rows[90] = 16'h1208;
        cn_rows[91] = 16'h13f8;
        cn_rows[92] = 16'h1208;
        cn_rows[93] = 16'h1208;
        cn_rows[94] = 16'h1ffe;
        cn_rows[95] = 16'h1000;
        // CN_GLYPH 06 U+8FB9 边
        cn_rows[96] = 16'h0040;
        cn_rows[97] = 16'h2040;
        cn_rows[98] = 16'h1040;
        cn_rows[99] = 16'h13fc;
        cn_rows[100] = 16'h0044;
        cn_rows[101] = 16'h0044;
        cn_rows[102] = 16'hf044;
        cn_rows[103] = 16'h1084;
        cn_rows[104] = 16'h1084;
        cn_rows[105] = 16'h1104;
        cn_rows[106] = 16'h1104;
        cn_rows[107] = 16'h1228;
        cn_rows[108] = 16'h1410;
        cn_rows[109] = 16'h2800;
        cn_rows[110] = 16'h47fe;
        cn_rows[111] = 16'h0000;
        // CN_GLYPH 07 U+7F18 缘
        cn_rows[112] = 16'h1080;
        cn_rows[113] = 16'h10fc;
        cn_rows[114] = 16'h2104;
        cn_rows[115] = 16'h21f8;
        cn_rows[116] = 16'h4808;
        cn_rows[117] = 16'hfbfe;
        cn_rows[118] = 16'h1040;
        cn_rows[119] = 16'h20a2;
        cn_rows[120] = 16'h4334;
        cn_rows[121] = 16'hf858;
        cn_rows[122] = 16'h4094;
        cn_rows[123] = 16'h0334;
        cn_rows[124] = 16'h1852;
        cn_rows[125] = 16'he090;
        cn_rows[126] = 16'h4350;
        cn_rows[127] = 16'h0020;
        // CN_GLYPH 08 U+5E27 帧
        cn_rows[128] = 16'h1020;
        cn_rows[129] = 16'h1020;
        cn_rows[130] = 16'h103e;
        cn_rows[131] = 16'h7c20;
        cn_rows[132] = 16'h5420;
        cn_rows[133] = 16'h55fc;
        cn_rows[134] = 16'h5504;
        cn_rows[135] = 16'h5524;
        cn_rows[136] = 16'h5524;
        cn_rows[137] = 16'h5524;
        cn_rows[138] = 16'h5524;
        cn_rows[139] = 16'h5d24;
        cn_rows[140] = 16'h1050;
        cn_rows[141] = 16'h1048;
        cn_rows[142] = 16'h1084;
        cn_rows[143] = 16'h1104;
        // CN_GLYPH 09 U+5747 均
        cn_rows[144] = 16'h1040;
        cn_rows[145] = 16'h1040;
        cn_rows[146] = 16'h1080;
        cn_rows[147] = 16'h10fc;
        cn_rows[148] = 16'h1104;
        cn_rows[149] = 16'hfe04;
        cn_rows[150] = 16'h1084;
        cn_rows[151] = 16'h1044;
        cn_rows[152] = 16'h1044;
        cn_rows[153] = 16'h1014;
        cn_rows[154] = 16'h1024;
        cn_rows[155] = 16'h1c44;
        cn_rows[156] = 16'he184;
        cn_rows[157] = 16'h4004;
        cn_rows[158] = 16'h0028;
        cn_rows[159] = 16'h0010;
        // CN_GLYPH 10 U+6709 有
        cn_rows[160] = 16'h0200;
        cn_rows[161] = 16'h0200;
        cn_rows[162] = 16'hfffe;
        cn_rows[163] = 16'h0400;
        cn_rows[164] = 16'h0400;
        cn_rows[165] = 16'h0ff0;
        cn_rows[166] = 16'h0810;
        cn_rows[167] = 16'h1810;
        cn_rows[168] = 16'h2ff0;
        cn_rows[169] = 16'h4810;
        cn_rows[170] = 16'h8810;
        cn_rows[171] = 16'h0ff0;
        cn_rows[172] = 16'h0810;
        cn_rows[173] = 16'h0810;
        cn_rows[174] = 16'h0850;
        cn_rows[175] = 16'h0820;
        // CN_GLYPH 11 U+65E0 无
        cn_rows[176] = 16'h0000;
        cn_rows[177] = 16'h3ff0;
        cn_rows[178] = 16'h0200;
        cn_rows[179] = 16'h0200;
        cn_rows[180] = 16'h0200;
        cn_rows[181] = 16'h0200;
        cn_rows[182] = 16'h7ffc;
        cn_rows[183] = 16'h0480;
        cn_rows[184] = 16'h0480;
        cn_rows[185] = 16'h0480;
        cn_rows[186] = 16'h0880;
        cn_rows[187] = 16'h0880;
        cn_rows[188] = 16'h1084;
        cn_rows[189] = 16'h2084;
        cn_rows[190] = 16'h407c;
        cn_rows[191] = 16'h8000;
        // CN_GLYPH 12 U+9519 错
        cn_rows[192] = 16'h2110;
        cn_rows[193] = 16'h2110;
        cn_rows[194] = 16'h3910;
        cn_rows[195] = 16'h27fc;
        cn_rows[196] = 16'h4110;
        cn_rows[197] = 16'h7910;
        cn_rows[198] = 16'haffe;
        cn_rows[199] = 16'h2000;
        cn_rows[200] = 16'hfbf8;
        cn_rows[201] = 16'h2208;
        cn_rows[202] = 16'h2208;
        cn_rows[203] = 16'h23f8;
        cn_rows[204] = 16'h2a08;
        cn_rows[205] = 16'h3208;
        cn_rows[206] = 16'h23f8;
        cn_rows[207] = 16'h0208;
        // CN_GLYPH 13 U+7EA2 红
        cn_rows[208] = 16'h1000;
        cn_rows[209] = 16'h1000;
        cn_rows[210] = 16'h21fc;
        cn_rows[211] = 16'h2420;
        cn_rows[212] = 16'h4420;
        cn_rows[213] = 16'hf820;
        cn_rows[214] = 16'h1020;
        cn_rows[215] = 16'h2020;
        cn_rows[216] = 16'h4020;
        cn_rows[217] = 16'hfc20;
        cn_rows[218] = 16'h4020;
        cn_rows[219] = 16'h0020;
        cn_rows[220] = 16'h1c20;
        cn_rows[221] = 16'he020;
        cn_rows[222] = 16'h43fe;
        cn_rows[223] = 16'h0000;
        // CN_GLYPH 14 U+84DD 蓝
        cn_rows[224] = 16'h0820;
        cn_rows[225] = 16'h0820;
        cn_rows[226] = 16'hfffe;
        cn_rows[227] = 16'h0820;
        cn_rows[228] = 16'h0480;
        cn_rows[229] = 16'h2480;
        cn_rows[230] = 16'h24fc;
        cn_rows[231] = 16'h24a0;
        cn_rows[232] = 16'h2510;
        cn_rows[233] = 16'h0000;
        cn_rows[234] = 16'h3ff8;
        cn_rows[235] = 16'h2448;
        cn_rows[236] = 16'h2448;
        cn_rows[237] = 16'h2448;
        cn_rows[238] = 16'hfffe;
        cn_rows[239] = 16'h0000;
        // CN_GLYPH 15 U+9759 静
        cn_rows[240] = 16'h1040;
        cn_rows[241] = 16'h1040;
        cn_rows[242] = 16'hfe78;
        cn_rows[243] = 16'h1088;
        cn_rows[244] = 16'h7c10;
        cn_rows[245] = 16'h11fc;
        cn_rows[246] = 16'hfe24;
        cn_rows[247] = 16'h0024;
        cn_rows[248] = 16'h7dfe;
        cn_rows[249] = 16'h4424;
        cn_rows[250] = 16'h7c24;
        cn_rows[251] = 16'h45fc;
        cn_rows[252] = 16'h7c24;
        cn_rows[253] = 16'h4420;
        cn_rows[254] = 16'h54a0;
        cn_rows[255] = 16'h4840;
        // CN_GLYPH 16 U+6B62 止
        cn_rows[256] = 16'h0100;
        cn_rows[257] = 16'h0100;
        cn_rows[258] = 16'h0100;
        cn_rows[259] = 16'h0100;
        cn_rows[260] = 16'h1100;
        cn_rows[261] = 16'h1100;
        cn_rows[262] = 16'h11f8;
        cn_rows[263] = 16'h1100;
        cn_rows[264] = 16'h1100;
        cn_rows[265] = 16'h1100;
        cn_rows[266] = 16'h1100;
        cn_rows[267] = 16'h1100;
        cn_rows[268] = 16'h1100;
        cn_rows[269] = 16'h1100;
        cn_rows[270] = 16'hfffe;
        cn_rows[271] = 16'h0000;
        // CN_GLYPH 17 U+8FD0 运
        cn_rows[272] = 16'h0000;
        cn_rows[273] = 16'h23f8;
        cn_rows[274] = 16'h1000;
        cn_rows[275] = 16'h1000;
        cn_rows[276] = 16'h0000;
        cn_rows[277] = 16'h07fc;
        cn_rows[278] = 16'hf040;
        cn_rows[279] = 16'h1080;
        cn_rows[280] = 16'h1110;
        cn_rows[281] = 16'h1208;
        cn_rows[282] = 16'h17fc;
        cn_rows[283] = 16'h1204;
        cn_rows[284] = 16'h1000;
        cn_rows[285] = 16'h2800;
        cn_rows[286] = 16'h47fe;
        cn_rows[287] = 16'h0000;
        // CN_GLYPH 18 U+52A8 动
        cn_rows[288] = 16'h0040;
        cn_rows[289] = 16'h0040;
        cn_rows[290] = 16'h7c40;
        cn_rows[291] = 16'h0040;
        cn_rows[292] = 16'h01fc;
        cn_rows[293] = 16'h0044;
        cn_rows[294] = 16'hfe44;
        cn_rows[295] = 16'h2044;
        cn_rows[296] = 16'h2044;
        cn_rows[297] = 16'h2084;
        cn_rows[298] = 16'h4884;
        cn_rows[299] = 16'h4484;
        cn_rows[300] = 16'hfd04;
        cn_rows[301] = 16'h4504;
        cn_rows[302] = 16'h0228;
        cn_rows[303] = 16'h0410;
        // CN_GLYPH 19 U+7B49 等
        cn_rows[304] = 16'h2040;
        cn_rows[305] = 16'h3f7e;
        cn_rows[306] = 16'h4890;
        cn_rows[307] = 16'h8508;
        cn_rows[308] = 16'h0100;
        cn_rows[309] = 16'h3ff8;
        cn_rows[310] = 16'h0100;
        cn_rows[311] = 16'h0100;
        cn_rows[312] = 16'hfffe;
        cn_rows[313] = 16'h0000;
        cn_rows[314] = 16'h0020;
        cn_rows[315] = 16'h7ffc;
        cn_rows[316] = 16'h0820;
        cn_rows[317] = 16'h0420;
        cn_rows[318] = 16'h04a0;
        cn_rows[319] = 16'h0040;
        // CN_GLYPH 20 U+5F85 待
        cn_rows[320] = 16'h0840;
        cn_rows[321] = 16'h0840;
        cn_rows[322] = 16'h1040;
        cn_rows[323] = 16'h23fc;
        cn_rows[324] = 16'h4840;
        cn_rows[325] = 16'h0840;
        cn_rows[326] = 16'h17fe;
        cn_rows[327] = 16'h3010;
        cn_rows[328] = 16'h5010;
        cn_rows[329] = 16'h97fe;
        cn_rows[330] = 16'h1010;
        cn_rows[331] = 16'h1210;
        cn_rows[332] = 16'h1110;
        cn_rows[333] = 16'h1110;
        cn_rows[334] = 16'h1050;
        cn_rows[335] = 16'h1020;
        // CN_GLYPH 21 U+68C0 检
        cn_rows[336] = 16'h1040;
        cn_rows[337] = 16'h1040;
        cn_rows[338] = 16'h10a0;
        cn_rows[339] = 16'h10a0;
        cn_rows[340] = 16'hfd10;
        cn_rows[341] = 16'h1208;
        cn_rows[342] = 16'h35f6;
        cn_rows[343] = 16'h3800;
        cn_rows[344] = 16'h5488;
        cn_rows[345] = 16'h5048;
        cn_rows[346] = 16'h9248;
        cn_rows[347] = 16'h1150;
        cn_rows[348] = 16'h1110;
        cn_rows[349] = 16'h1020;
        cn_rows[350] = 16'h17fe;
        cn_rows[351] = 16'h1000;
        // CN_GLYPH 22 U+6D4B 测
        cn_rows[352] = 16'h0004;
        cn_rows[353] = 16'h27c4;
        cn_rows[354] = 16'h1444;
        cn_rows[355] = 16'h1454;
        cn_rows[356] = 16'h8554;
        cn_rows[357] = 16'h4554;
        cn_rows[358] = 16'h4554;
        cn_rows[359] = 16'h1554;
        cn_rows[360] = 16'h1554;
        cn_rows[361] = 16'h2554;
        cn_rows[362] = 16'he554;
        cn_rows[363] = 16'h2104;
        cn_rows[364] = 16'h2284;
        cn_rows[365] = 16'h2244;
        cn_rows[366] = 16'h2414;
        cn_rows[367] = 16'h0808;
        // CN_GLYPH 23 U+6B63 正
        cn_rows[368] = 16'h0000;
        cn_rows[369] = 16'h7ffc;
        cn_rows[370] = 16'h0100;
        cn_rows[371] = 16'h0100;
        cn_rows[372] = 16'h0100;
        cn_rows[373] = 16'h0100;
        cn_rows[374] = 16'h1100;
        cn_rows[375] = 16'h11f8;
        cn_rows[376] = 16'h1100;
        cn_rows[377] = 16'h1100;
        cn_rows[378] = 16'h1100;
        cn_rows[379] = 16'h1100;
        cn_rows[380] = 16'h1100;
        cn_rows[381] = 16'h1100;
        cn_rows[382] = 16'hfffe;
        cn_rows[383] = 16'h0000;
        // CN_GLYPH 24 U+5E38 常
        cn_rows[384] = 16'h0100;
        cn_rows[385] = 16'h1110;
        cn_rows[386] = 16'h0920;
        cn_rows[387] = 16'h7ffe;
        cn_rows[388] = 16'h4002;
        cn_rows[389] = 16'h9ff4;
        cn_rows[390] = 16'h1010;
        cn_rows[391] = 16'h1ff0;
        cn_rows[392] = 16'h0100;
        cn_rows[393] = 16'h3ff8;
        cn_rows[394] = 16'h2108;
        cn_rows[395] = 16'h2108;
        cn_rows[396] = 16'h2128;
        cn_rows[397] = 16'h2110;
        cn_rows[398] = 16'h0100;
        cn_rows[399] = 16'h0100;
        // CN_GLYPH 25 U+6EE1 满
        cn_rows[400] = 16'h0108;
        cn_rows[401] = 16'h2108;
        cn_rows[402] = 16'h17fe;
        cn_rows[403] = 16'h1108;
        cn_rows[404] = 16'h8000;
        cn_rows[405] = 16'h47fe;
        cn_rows[406] = 16'h4090;
        cn_rows[407] = 16'h1090;
        cn_rows[408] = 16'h17fe;
        cn_rows[409] = 16'h2492;
        cn_rows[410] = 16'he492;
        cn_rows[411] = 16'h256a;
        cn_rows[412] = 16'h2646;
        cn_rows[413] = 16'h2402;
        cn_rows[414] = 16'h240a;
        cn_rows[415] = 16'h0404;
        // CN_GLYPH 26 U+8F7D 载
        cn_rows[416] = 16'h0820;
        cn_rows[417] = 16'h0828;
        cn_rows[418] = 16'h7f24;
        cn_rows[419] = 16'h0820;
        cn_rows[420] = 16'hfffe;
        cn_rows[421] = 16'h1020;
        cn_rows[422] = 16'h1024;
        cn_rows[423] = 16'hff24;
        cn_rows[424] = 16'h2024;
        cn_rows[425] = 16'h4828;
        cn_rows[426] = 16'h7f28;
        cn_rows[427] = 16'h0810;
        cn_rows[428] = 16'h0f12;
        cn_rows[429] = 16'hf82a;
        cn_rows[430] = 16'h0846;
        cn_rows[431] = 16'h0882;
        // CN_GLYPH 27 U+573A 场
        cn_rows[432] = 16'h2000;
        cn_rows[433] = 16'h23f0;
        cn_rows[434] = 16'h2020;
        cn_rows[435] = 16'h2040;
        cn_rows[436] = 16'h2080;
        cn_rows[437] = 16'hf900;
        cn_rows[438] = 16'h23fc;
        cn_rows[439] = 16'h2124;
        cn_rows[440] = 16'h2124;
        cn_rows[441] = 16'h2124;
        cn_rows[442] = 16'h2224;
        cn_rows[443] = 16'h3a44;
        cn_rows[444] = 16'he444;
        cn_rows[445] = 16'h4884;
        cn_rows[446] = 16'h0128;
        cn_rows[447] = 16'h0210;
        // CN_GLYPH 28 U+53D8 变
        cn_rows[448] = 16'h0200;
        cn_rows[449] = 16'h0100;
        cn_rows[450] = 16'hfffe;
        cn_rows[451] = 16'h0440;
        cn_rows[452] = 16'h1450;
        cn_rows[453] = 16'h2448;
        cn_rows[454] = 16'h4444;
        cn_rows[455] = 16'h0000;
        cn_rows[456] = 16'h3ff0;
        cn_rows[457] = 16'h0820;
        cn_rows[458] = 16'h0440;
        cn_rows[459] = 16'h0280;
        cn_rows[460] = 16'h0100;
        cn_rows[461] = 16'h06c0;
        cn_rows[462] = 16'h1830;
        cn_rows[463] = 16'he00e;
        // CN_GLYPH 29 U+7EFF 绿
        cn_rows[464] = 16'h1000;
        cn_rows[465] = 16'h11f8;
        cn_rows[466] = 16'h2008;
        cn_rows[467] = 16'h2008;
        cn_rows[468] = 16'h49f8;
        cn_rows[469] = 16'hf808;
        cn_rows[470] = 16'h1008;
        cn_rows[471] = 16'h23fe;
        cn_rows[472] = 16'h4020;
        cn_rows[473] = 16'hfa22;
        cn_rows[474] = 16'h4174;
        cn_rows[475] = 16'h00a8;
        cn_rows[476] = 16'h1924;
        cn_rows[477] = 16'he222;
        cn_rows[478] = 16'h40a0;
        cn_rows[479] = 16'h0040;
        // CN_GLYPH 30 U+9875 页
        cn_rows[480] = 16'h0000;
        cn_rows[481] = 16'h7ffc;
        cn_rows[482] = 16'h0200;
        cn_rows[483] = 16'h0400;
        cn_rows[484] = 16'h1ff0;
        cn_rows[485] = 16'h1010;
        cn_rows[486] = 16'h1110;
        cn_rows[487] = 16'h1110;
        cn_rows[488] = 16'h1110;
        cn_rows[489] = 16'h1110;
        cn_rows[490] = 16'h1110;
        cn_rows[491] = 16'h1290;
        cn_rows[492] = 16'h0240;
        cn_rows[493] = 16'h0420;
        cn_rows[494] = 16'h1810;
        cn_rows[495] = 16'h6008;
        // CN_GLYPH 31 U+533A 区
        cn_rows[496] = 16'h0000;
        cn_rows[497] = 16'h7ffc;
        cn_rows[498] = 16'h4000;
        cn_rows[499] = 16'h4010;
        cn_rows[500] = 16'h4410;
        cn_rows[501] = 16'h4220;
        cn_rows[502] = 16'h4140;
        cn_rows[503] = 16'h4080;
        cn_rows[504] = 16'h4140;
        cn_rows[505] = 16'h4220;
        cn_rows[506] = 16'h4410;
        cn_rows[507] = 16'h4810;
        cn_rows[508] = 16'h4000;
        cn_rows[509] = 16'h4000;
        cn_rows[510] = 16'h7ffe;
        cn_rows[511] = 16'h0000;
        // CN_GLYPH 32 U+57DF 域
        cn_rows[512] = 16'h2014;
        cn_rows[513] = 16'h2012;
        cn_rows[514] = 16'h2010;
        cn_rows[515] = 16'h27fe;
        cn_rows[516] = 16'h2010;
        cn_rows[517] = 16'hf810;
        cn_rows[518] = 16'h23d2;
        cn_rows[519] = 16'h2252;
        cn_rows[520] = 16'h2252;
        cn_rows[521] = 16'h2254;
        cn_rows[522] = 16'h23d4;
        cn_rows[523] = 16'h3808;
        cn_rows[524] = 16'he0ea;
        cn_rows[525] = 16'h471a;
        cn_rows[526] = 16'h0226;
        cn_rows[527] = 16'h0042;
        // CN_GLYPH 33 U+62A5 报
        cn_rows[528] = 16'h1000;
        cn_rows[529] = 16'h11fc;
        cn_rows[530] = 16'h1104;
        cn_rows[531] = 16'h1104;
        cn_rows[532] = 16'hfd14;
        cn_rows[533] = 16'h1108;
        cn_rows[534] = 16'h1100;
        cn_rows[535] = 16'h15fc;
        cn_rows[536] = 16'h1944;
        cn_rows[537] = 16'h3144;
        cn_rows[538] = 16'hd128;
        cn_rows[539] = 16'h1128;
        cn_rows[540] = 16'h1110;
        cn_rows[541] = 16'h1128;
        cn_rows[542] = 16'h5144;
        cn_rows[543] = 16'h2182;
        // CN_GLYPH 34 U+8B66 警
        cn_rows[544] = 16'h2420;
        cn_rows[545] = 16'hff20;
        cn_rows[546] = 16'h247e;
        cn_rows[547] = 16'h7ec4;
        cn_rows[548] = 16'h8228;
        cn_rows[549] = 16'h7a10;
        cn_rows[550] = 16'h4a28;
        cn_rows[551] = 16'h7ac6;
        cn_rows[552] = 16'h0500;
        cn_rows[553] = 16'hfffe;
        cn_rows[554] = 16'h0000;
        cn_rows[555] = 16'h3ff8;
        cn_rows[556] = 16'h0000;
        cn_rows[557] = 16'h3ff8;
        cn_rows[558] = 16'h2008;
        cn_rows[559] = 16'h3ff8;
        // CN_GLYPH 35 U+5B89 安
        cn_rows[560] = 16'h0200;
        cn_rows[561] = 16'h0100;
        cn_rows[562] = 16'h3ffc;
        cn_rows[563] = 16'h2004;
        cn_rows[564] = 16'h4208;
        cn_rows[565] = 16'h0200;
        cn_rows[566] = 16'h0200;
        cn_rows[567] = 16'hfffe;
        cn_rows[568] = 16'h0420;
        cn_rows[569] = 16'h0820;
        cn_rows[570] = 16'h1840;
        cn_rows[571] = 16'h0640;
        cn_rows[572] = 16'h0180;
        cn_rows[573] = 16'h0260;
        cn_rows[574] = 16'h0c10;
        cn_rows[575] = 16'h7008;
        // CN_GLYPH 36 U+5168 全
        cn_rows[576] = 16'h0100;
        cn_rows[577] = 16'h0100;
        cn_rows[578] = 16'h0280;
        cn_rows[579] = 16'h0440;
        cn_rows[580] = 16'h0820;
        cn_rows[581] = 16'h1010;
        cn_rows[582] = 16'h2fe8;
        cn_rows[583] = 16'hc106;
        cn_rows[584] = 16'h0100;
        cn_rows[585] = 16'h0100;
        cn_rows[586] = 16'h1ff0;
        cn_rows[587] = 16'h0100;
        cn_rows[588] = 16'h0100;
        cn_rows[589] = 16'h0100;
        cn_rows[590] = 16'h7ffc;
        cn_rows[591] = 16'h0000;
        // CN_GLYPH 37 U+6B21 次
        cn_rows[592] = 16'h0080;
        cn_rows[593] = 16'h4080;
        cn_rows[594] = 16'h2080;
        cn_rows[595] = 16'h20fc;
        cn_rows[596] = 16'h0104;
        cn_rows[597] = 16'h0908;
        cn_rows[598] = 16'h0a40;
        cn_rows[599] = 16'h1440;
        cn_rows[600] = 16'h1040;
        cn_rows[601] = 16'he0a0;
        cn_rows[602] = 16'h20a0;
        cn_rows[603] = 16'h2110;
        cn_rows[604] = 16'h2110;
        cn_rows[605] = 16'h2208;
        cn_rows[606] = 16'h2404;
        cn_rows[607] = 16'h0802;
        // CN_GLYPH 38 U+6570 数
        cn_rows[608] = 16'h0820;
        cn_rows[609] = 16'h4920;
        cn_rows[610] = 16'h2a20;
        cn_rows[611] = 16'h083e;
        cn_rows[612] = 16'hff44;
        cn_rows[613] = 16'h2a44;
        cn_rows[614] = 16'h4944;
        cn_rows[615] = 16'h88a4;
        cn_rows[616] = 16'h1028;
        cn_rows[617] = 16'hfe28;
        cn_rows[618] = 16'h2210;
        cn_rows[619] = 16'h4210;
        cn_rows[620] = 16'h6428;
        cn_rows[621] = 16'h1828;
        cn_rows[622] = 16'h3444;
        cn_rows[623] = 16'hc282;
        // CN_GLYPH 39 U+6D3B 活
        cn_rows[624] = 16'h0010;
        cn_rows[625] = 16'h2078;
        cn_rows[626] = 16'h13c0;
        cn_rows[627] = 16'h1040;
        cn_rows[628] = 16'h8040;
        cn_rows[629] = 16'h47fe;
        cn_rows[630] = 16'h4040;
        cn_rows[631] = 16'h1040;
        cn_rows[632] = 16'h1040;
        cn_rows[633] = 16'h23f8;
        cn_rows[634] = 16'he208;
        cn_rows[635] = 16'h2208;
        cn_rows[636] = 16'h2208;
        cn_rows[637] = 16'h2208;
        cn_rows[638] = 16'h23f8;
        cn_rows[639] = 16'h0208;
        // CN_GLYPH 40 U+81EA 自
        cn_rows[640] = 16'h0100;
        cn_rows[641] = 16'h0200;
        cn_rows[642] = 16'h0400;
        cn_rows[643] = 16'h1ff0;
        cn_rows[644] = 16'h1010;
        cn_rows[645] = 16'h1010;
        cn_rows[646] = 16'h1010;
        cn_rows[647] = 16'h1ff0;
        cn_rows[648] = 16'h1010;
        cn_rows[649] = 16'h1010;
        cn_rows[650] = 16'h1ff0;
        cn_rows[651] = 16'h1010;
        cn_rows[652] = 16'h1010;
        cn_rows[653] = 16'h1010;
        cn_rows[654] = 16'h1ff0;
        cn_rows[655] = 16'h1010;
        // CN_GLYPH 41 U+89E3 解
        cn_rows[656] = 16'h1000;
        cn_rows[657] = 16'h10fc;
        cn_rows[658] = 16'h3c24;
        cn_rows[659] = 16'h2424;
        cn_rows[660] = 16'h4854;
        cn_rows[661] = 16'hbe88;
        cn_rows[662] = 16'h2a10;
        cn_rows[663] = 16'h2a50;
        cn_rows[664] = 16'h3e7c;
        cn_rows[665] = 16'h2a90;
        cn_rows[666] = 16'h2a10;
        cn_rows[667] = 16'h3efe;
        cn_rows[668] = 16'h2a10;
        cn_rows[669] = 16'h4a10;
        cn_rows[670] = 16'h4210;
        cn_rows[671] = 16'h8610;
        // CN_GLYPH 42 U+9664 除
        cn_rows[672] = 16'h0040;
        cn_rows[673] = 16'h7840;
        cn_rows[674] = 16'h48a0;
        cn_rows[675] = 16'h5110;
        cn_rows[676] = 16'h5208;
        cn_rows[677] = 16'h65f6;
        cn_rows[678] = 16'h5040;
        cn_rows[679] = 16'h4840;
        cn_rows[680] = 16'h4ffc;
        cn_rows[681] = 16'h4840;
        cn_rows[682] = 16'h6a50;
        cn_rows[683] = 16'h5248;
        cn_rows[684] = 16'h4444;
        cn_rows[685] = 16'h4844;
        cn_rows[686] = 16'h4140;
        cn_rows[687] = 16'h4080;
        // CN_GLYPH 43 U+66DD 曝
        cn_rows[688] = 16'h03f8;
        cn_rows[689] = 16'h0208;
        cn_rows[690] = 16'hf3f8;
        cn_rows[691] = 16'h9208;
        cn_rows[692] = 16'h93f8;
        cn_rows[693] = 16'h9110;
        cn_rows[694] = 16'h97fc;
        cn_rows[695] = 16'hf110;
        cn_rows[696] = 16'h9ffe;
        cn_rows[697] = 16'h9110;
        cn_rows[698] = 16'h9248;
        cn_rows[699] = 16'h9554;
        cn_rows[700] = 16'hf0e0;
        cn_rows[701] = 16'h9150;
        cn_rows[702] = 16'h0248;
        cn_rows[703] = 16'h00c0;
        // CN_GLYPH 44 U+5149 光
        cn_rows[704] = 16'h0100;
        cn_rows[705] = 16'h2108;
        cn_rows[706] = 16'h1108;
        cn_rows[707] = 16'h0910;
        cn_rows[708] = 16'h0920;
        cn_rows[709] = 16'h0100;
        cn_rows[710] = 16'hfffe;
        cn_rows[711] = 16'h0440;
        cn_rows[712] = 16'h0440;
        cn_rows[713] = 16'h0440;
        cn_rows[714] = 16'h0440;
        cn_rows[715] = 16'h0842;
        cn_rows[716] = 16'h0842;
        cn_rows[717] = 16'h1042;
        cn_rows[718] = 16'h203e;
        cn_rows[719] = 16'hc000;
        // CN_GLYPH 45 U+76EE 目
        cn_rows[720] = 16'h0000;
        cn_rows[721] = 16'h3ff8;
        cn_rows[722] = 16'h2008;
        cn_rows[723] = 16'h2008;
        cn_rows[724] = 16'h2008;
        cn_rows[725] = 16'h3ff8;
        cn_rows[726] = 16'h2008;
        cn_rows[727] = 16'h2008;
        cn_rows[728] = 16'h2008;
        cn_rows[729] = 16'h3ff8;
        cn_rows[730] = 16'h2008;
        cn_rows[731] = 16'h2008;
        cn_rows[732] = 16'h2008;
        cn_rows[733] = 16'h2008;
        cn_rows[734] = 16'h3ff8;
        cn_rows[735] = 16'h2008;
        // CN_GLYPH 46 U+6807 标
        cn_rows[736] = 16'h1000;
        cn_rows[737] = 16'h11fc;
        cn_rows[738] = 16'h1000;
        cn_rows[739] = 16'h1000;
        cn_rows[740] = 16'hfc00;
        cn_rows[741] = 16'h13fe;
        cn_rows[742] = 16'h3020;
        cn_rows[743] = 16'h3820;
        cn_rows[744] = 16'h5528;
        cn_rows[745] = 16'h5524;
        cn_rows[746] = 16'h9224;
        cn_rows[747] = 16'h1222;
        cn_rows[748] = 16'h1422;
        cn_rows[749] = 16'h1020;
        cn_rows[750] = 16'h10a0;
        cn_rows[751] = 16'h1040;
        // CN_GLYPH 47 U+544A 告
        cn_rows[752] = 16'h0100;
        cn_rows[753] = 16'h1100;
        cn_rows[754] = 16'h1100;
        cn_rows[755] = 16'h1ff8;
        cn_rows[756] = 16'h2100;
        cn_rows[757] = 16'h4100;
        cn_rows[758] = 16'h0100;
        cn_rows[759] = 16'hfffe;
        cn_rows[760] = 16'h0000;
        cn_rows[761] = 16'h0000;
        cn_rows[762] = 16'h1ff0;
        cn_rows[763] = 16'h1010;
        cn_rows[764] = 16'h1010;
        cn_rows[765] = 16'h1010;
        cn_rows[766] = 16'h1ff0;
        cn_rows[767] = 16'h1010;
        // CN_GLYPH 48 U+5173 关
        cn_rows[768] = 16'h1010;
        cn_rows[769] = 16'h0810;
        cn_rows[770] = 16'h0820;
        cn_rows[771] = 16'h0000;
        cn_rows[772] = 16'h3ff8;
        cn_rows[773] = 16'h0100;
        cn_rows[774] = 16'h0100;
        cn_rows[775] = 16'h0100;
        cn_rows[776] = 16'hfffe;
        cn_rows[777] = 16'h0100;
        cn_rows[778] = 16'h0280;
        cn_rows[779] = 16'h0280;
        cn_rows[780] = 16'h0440;
        cn_rows[781] = 16'h0820;
        cn_rows[782] = 16'h3018;
        cn_rows[783] = 16'hc006;
        // CN_GLYPH 49 U+95ED 闭
        cn_rows[784] = 16'h2000;
        cn_rows[785] = 16'h17fc;
        cn_rows[786] = 16'h0004;
        cn_rows[787] = 16'h4084;
        cn_rows[788] = 16'h4084;
        cn_rows[789] = 16'h5ff4;
        cn_rows[790] = 16'h4084;
        cn_rows[791] = 16'h4184;
        cn_rows[792] = 16'h4284;
        cn_rows[793] = 16'h4484;
        cn_rows[794] = 16'h4884;
        cn_rows[795] = 16'h5084;
        cn_rows[796] = 16'h4284;
        cn_rows[797] = 16'h4104;
        cn_rows[798] = 16'h4014;
        cn_rows[799] = 16'h4008;
        // CN_GLYPH 50 U+964D 降
        cn_rows[800] = 16'h0080;
        cn_rows[801] = 16'h7c80;
        cn_rows[802] = 16'h44fc;
        cn_rows[803] = 16'h4908;
        cn_rows[804] = 16'h4a90;
        cn_rows[805] = 16'h5060;
        cn_rows[806] = 16'h4998;
        cn_rows[807] = 16'h4e26;
        cn_rows[808] = 16'h4420;
        cn_rows[809] = 16'h45fc;
        cn_rows[810] = 16'h4420;
        cn_rows[811] = 16'h6a20;
        cn_rows[812] = 16'h53fe;
        cn_rows[813] = 16'h4020;
        cn_rows[814] = 16'h4020;
        cn_rows[815] = 16'h4020;
        // CN_GLYPH 51 U+566A 噪
        cn_rows[816] = 16'h03f8;
        cn_rows[817] = 16'h0208;
        cn_rows[818] = 16'hf208;
        cn_rows[819] = 16'h93f8;
        cn_rows[820] = 16'h9000;
        cn_rows[821] = 16'h97bc;
        cn_rows[822] = 16'h94a4;
        cn_rows[823] = 16'h94a4;
        cn_rows[824] = 16'h97bc;
        cn_rows[825] = 16'h9040;
        cn_rows[826] = 16'hfffe;
        cn_rows[827] = 16'h9160;
        cn_rows[828] = 16'h0250;
        cn_rows[829] = 16'h0448;
        cn_rows[830] = 16'h1846;
        cn_rows[831] = 16'h0040;
        // CN_GLYPH 52 U+6548 效
        cn_rows[832] = 16'h1020;
        cn_rows[833] = 16'h0820;
        cn_rows[834] = 16'h0020;
        cn_rows[835] = 16'hff3e;
        cn_rows[836] = 16'h0044;
        cn_rows[837] = 16'h2444;
        cn_rows[838] = 16'h4244;
        cn_rows[839] = 16'h81a4;
        cn_rows[840] = 16'h2428;
        cn_rows[841] = 16'h1428;
        cn_rows[842] = 16'h0810;
        cn_rows[843] = 16'h1410;
        cn_rows[844] = 16'h2228;
        cn_rows[845] = 16'h4248;
        cn_rows[846] = 16'h8084;
        cn_rows[847] = 16'h0102;
        // CN_GLYPH 53 U+5E73 平
        cn_rows[848] = 16'h0000;
        cn_rows[849] = 16'h7ffc;
        cn_rows[850] = 16'h0100;
        cn_rows[851] = 16'h0100;
        cn_rows[852] = 16'h1110;
        cn_rows[853] = 16'h0910;
        cn_rows[854] = 16'h0920;
        cn_rows[855] = 16'h0100;
        cn_rows[856] = 16'hfffe;
        cn_rows[857] = 16'h0100;
        cn_rows[858] = 16'h0100;
        cn_rows[859] = 16'h0100;
        cn_rows[860] = 16'h0100;
        cn_rows[861] = 16'h0100;
        cn_rows[862] = 16'h0100;
        cn_rows[863] = 16'h0100;
        cn_rows[864] = 16'h0000;
        cn_rows[865] = 16'h0000;
        cn_rows[866] = 16'h0000;
        cn_rows[867] = 16'h0000;
        cn_rows[868] = 16'h0000;
        cn_rows[869] = 16'h0000;
        cn_rows[870] = 16'h0000;
        cn_rows[871] = 16'h0000;
        cn_rows[872] = 16'h0000;
        cn_rows[873] = 16'h0000;
        cn_rows[874] = 16'h0000;
        cn_rows[875] = 16'h0000;
        cn_rows[876] = 16'h0000;
        cn_rows[877] = 16'h0000;
        cn_rows[878] = 16'h0000;
        cn_rows[879] = 16'h0000;
        cn_rows[880] = 16'h0000;
        cn_rows[881] = 16'h0000;
        cn_rows[882] = 16'h0000;
        cn_rows[883] = 16'h0000;
        cn_rows[884] = 16'h0000;
        cn_rows[885] = 16'h0000;
        cn_rows[886] = 16'h0000;
        cn_rows[887] = 16'h0000;
        cn_rows[888] = 16'h0000;
        cn_rows[889] = 16'h0000;
        cn_rows[890] = 16'h0000;
        cn_rows[891] = 16'h0000;
        cn_rows[892] = 16'h0000;
        cn_rows[893] = 16'h0000;
        cn_rows[894] = 16'h0000;
        cn_rows[895] = 16'h0000;
        cn_rows[896] = 16'h0000;
        cn_rows[897] = 16'h0000;
        cn_rows[898] = 16'h0000;
        cn_rows[899] = 16'h0000;
        cn_rows[900] = 16'h0000;
        cn_rows[901] = 16'h0000;
        cn_rows[902] = 16'h0000;
        cn_rows[903] = 16'h0000;
        cn_rows[904] = 16'h0000;
        cn_rows[905] = 16'h0000;
        cn_rows[906] = 16'h0000;
        cn_rows[907] = 16'h0000;
        cn_rows[908] = 16'h0000;
        cn_rows[909] = 16'h0000;
        cn_rows[910] = 16'h0000;
        cn_rows[911] = 16'h0000;
        cn_rows[912] = 16'h0000;
        cn_rows[913] = 16'h0000;
        cn_rows[914] = 16'h0000;
        cn_rows[915] = 16'h0000;
        cn_rows[916] = 16'h0000;
        cn_rows[917] = 16'h0000;
        cn_rows[918] = 16'h0000;
        cn_rows[919] = 16'h0000;
        cn_rows[920] = 16'h0000;
        cn_rows[921] = 16'h0000;
        cn_rows[922] = 16'h0000;
        cn_rows[923] = 16'h0000;
        cn_rows[924] = 16'h0000;
        cn_rows[925] = 16'h0000;
        cn_rows[926] = 16'h0000;
        cn_rows[927] = 16'h0000;
        cn_rows[928] = 16'h0000;
        cn_rows[929] = 16'h0000;
        cn_rows[930] = 16'h0000;
        cn_rows[931] = 16'h0000;
        cn_rows[932] = 16'h0000;
        cn_rows[933] = 16'h0000;
        cn_rows[934] = 16'h0000;
        cn_rows[935] = 16'h0000;
        cn_rows[936] = 16'h0000;
        cn_rows[937] = 16'h0000;
        cn_rows[938] = 16'h0000;
        cn_rows[939] = 16'h0000;
        cn_rows[940] = 16'h0000;
        cn_rows[941] = 16'h0000;
        cn_rows[942] = 16'h0000;
        cn_rows[943] = 16'h0000;
        cn_rows[944] = 16'h0000;
        cn_rows[945] = 16'h0000;
        cn_rows[946] = 16'h0000;
        cn_rows[947] = 16'h0000;
        cn_rows[948] = 16'h0000;
        cn_rows[949] = 16'h0000;
        cn_rows[950] = 16'h0000;
        cn_rows[951] = 16'h0000;
        cn_rows[952] = 16'h0000;
        cn_rows[953] = 16'h0000;
        cn_rows[954] = 16'h0000;
        cn_rows[955] = 16'h0000;
        cn_rows[956] = 16'h0000;
        cn_rows[957] = 16'h0000;
        cn_rows[958] = 16'h0000;
        cn_rows[959] = 16'h0000;
        cn_rows[960] = 16'h0000;
        cn_rows[961] = 16'h0000;
        cn_rows[962] = 16'h0000;
        cn_rows[963] = 16'h0000;
        cn_rows[964] = 16'h0000;
        cn_rows[965] = 16'h0000;
        cn_rows[966] = 16'h0000;
        cn_rows[967] = 16'h0000;
        cn_rows[968] = 16'h0000;
        cn_rows[969] = 16'h0000;
        cn_rows[970] = 16'h0000;
        cn_rows[971] = 16'h0000;
        cn_rows[972] = 16'h0000;
        cn_rows[973] = 16'h0000;
        cn_rows[974] = 16'h0000;
        cn_rows[975] = 16'h0000;
        cn_rows[976] = 16'h0000;
        cn_rows[977] = 16'h0000;
        cn_rows[978] = 16'h0000;
        cn_rows[979] = 16'h0000;
        cn_rows[980] = 16'h0000;
        cn_rows[981] = 16'h0000;
        cn_rows[982] = 16'h0000;
        cn_rows[983] = 16'h0000;
        cn_rows[984] = 16'h0000;
        cn_rows[985] = 16'h0000;
        cn_rows[986] = 16'h0000;
        cn_rows[987] = 16'h0000;
        cn_rows[988] = 16'h0000;
        cn_rows[989] = 16'h0000;
        cn_rows[990] = 16'h0000;
        cn_rows[991] = 16'h0000;
        cn_rows[992] = 16'h0000;
        cn_rows[993] = 16'h0000;
        cn_rows[994] = 16'h0000;
        cn_rows[995] = 16'h0000;
        cn_rows[996] = 16'h0000;
        cn_rows[997] = 16'h0000;
        cn_rows[998] = 16'h0000;
        cn_rows[999] = 16'h0000;
        cn_rows[1000] = 16'h0000;
        cn_rows[1001] = 16'h0000;
        cn_rows[1002] = 16'h0000;
        cn_rows[1003] = 16'h0000;
        cn_rows[1004] = 16'h0000;
        cn_rows[1005] = 16'h0000;
        cn_rows[1006] = 16'h0000;
        cn_rows[1007] = 16'h0000;
        cn_rows[1008] = 16'h0000;
        cn_rows[1009] = 16'h0000;
        cn_rows[1010] = 16'h0000;
        cn_rows[1011] = 16'h0000;
        cn_rows[1012] = 16'h0000;
        cn_rows[1013] = 16'h0000;
        cn_rows[1014] = 16'h0000;
        cn_rows[1015] = 16'h0000;
        cn_rows[1016] = 16'h0000;
        cn_rows[1017] = 16'h0000;
        cn_rows[1018] = 16'h0000;
        cn_rows[1019] = 16'h0000;
        cn_rows[1020] = 16'h0000;
        cn_rows[1021] = 16'h0000;
        cn_rows[1022] = 16'h0000;
        cn_rows[1023] = 16'h0000;
    end
    always @(posedge I_clk)
        cn_row_d <= cn_rows[{selected_char[5:0], I_pix_y[3:0]}];
    // END_CN_ROW_ROM

    // STATUS_FONT
    function [34:0] font;
        input [7:0] char;
        begin case(char)
            8'h30:font=35'b01110100011001110101110011000101110; // 0
            8'h31:font=35'b00100011000010000100001000010001110; // 1
            8'h32:font=35'b01110100010000100010001000100011111; // 2
            8'h33:font=35'b11110000010000101110000010000111110; // 3
            8'h34:font=35'b00010001100101010010111110001000010; // 4
            8'h35:font=35'b11111100001000011110000010000111110; // 5
            8'h36:font=35'b01110100001000011110100011000101110; // 6
            8'h37:font=35'b11111000010001000100010000100001000; // 7
            8'h38:font=35'b01110100011000101110100011000101110; // 8
            8'h39:font=35'b01110100011000101111000010000101110; // 9
            8'h3a:font=35'b00000001000010000000001000010000000; // :
            8'h41:font=35'b01110100011000111111100011000110001; // A
            8'h42:font=35'b11110100011000111110100011000111110; // B
            8'h43:font=35'b01110100011000010000100001000101110; // C
            8'h44:font=35'b11110100011000110001100011000111110; // D
            8'h45:font=35'b11111100001000011110100001000011111; // E
            8'h46:font=35'b11111100001000011110100001000010000; // F
            8'h47:font=35'b01110100001000010111100011000101110; // G
            8'h49:font=35'b01110001000010000100001000010001110; // I
            8'h4c:font=35'b10000100001000010000100001000011111; // L
            8'h4d:font=35'b10001110111010110101100011000110001; // M
            8'h4e:font=35'b10001110011010110011100011000110001; // N
            8'h4f:font=35'b01110100011000110001100011000101110; // O
            8'h50:font=35'b11110100011000111110100001000010000; // P
            8'h52:font=35'b11110100011000111110101001001010001; // R
            8'h53:font=35'b01111100001000001110000010000111110; // S
            8'h54:font=35'b11111001000010000100001000010000100; // T
            8'h55:font=35'b10001100011000110001100011000101110; // U
            8'h56:font=35'b10001100011000110001100010101000100; // V
            8'h57:font=35'b10001100011000110101101011101110001; // W
            8'h58:font=35'b10001100010101000100010101000110001; // X
            8'h5a:font=35'b11111000010001000100010001000011111; // Z
            default:font=35'd0;
        endcase end
    endfunction
    // END_STATUS_FONT

    wire [34:0] glyph=font(char_d);
    wire [5:0] row_offset={row_d,2'b00}+{3'b000,row_d};
    wire glyph_bit=char_d[7]?cn_row_d[4'd15-cn_col_d]:
        ((row_d<7 && col_d<5)?glyph[6'd34-row_offset-{3'b000,col_d}]:1'b0);
    reg text_hit,roi_hit;
    reg [1:0] color_dd;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin text_hit<=0; roi_hit<=0; color_dd<=0; end
        else begin
            text_hit<=panel_d && glyph_bit;
            roi_hit<=roi_d;
            color_dd<=color_d;
        end
    end
    wire [23:0] text_color=color_dd==0?24'h40ff40:color_dd==1?24'hff4040:
        color_dd==2?24'h40b0ff:24'hfff200;
    wire [23:0] zone_color=I_alarm?24'hff4040:I_ready?24'h40ff40:24'hfff200;
    assign O_hit=text_hit || roi_hit || cal_bg_dd;
    assign O_color=text_hit?text_color:cal_bg_dd?24'h101010:zone_color;
endmodule


// Five neutral-field windows,128x128 pixels each,1280x720 geometry.
// L=DDR linear RGB after AWB; D=post gamma/NR/saturation before all overlays.
// Alternate four-frame windows, so both displays update every eight frames.
// Sample every8pixels/rows:256samples/frame,1024/window;18bit sums fit261120.
// Read-only observer. No pixel outputs,feedback,frame RAM,DSP or camera writes.
module color_field_meter(
    input wire clk,rst_n,I_de,I_frame,
    input wire [10:0] I_x,
    input wire [9:0] I_y,
    input wire [23:0] I_linear,I_display,
    output reg [359:0] O_bcd,
    output reg [1:0] O_ready
);
    reg collect_phase,finished_phase,frame_seen;
    reg [1:0] complete_frames;
    reg means_ready;
    reg [23:0] p_rgb;
    reg [10:0] p_x;
    reg [9:0] p_y;
    reg p_de,p_frame;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin p_rgb<=0;p_x<=0;p_y<=0;p_de<=0;p_frame<=0;end
        else begin
            p_rgb<=collect_phase?I_display:I_linear;
            p_x<=I_x;p_y<=I_y;p_de<=I_de;p_frame<=I_frame;
        end
    end
    wire window_end=p_frame && frame_seen && complete_frames==3;
    wire window_start=p_frame && (!frame_seen || complete_frames==3);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            collect_phase<=0;finished_phase<=0;frame_seen<=0;complete_frames<=0;means_ready<=0;
        end else begin
            means_ready<=0;
            if(p_frame) begin
                if(!frame_seen) begin frame_seen<=1;complete_frames<=0;end
                else if(complete_frames==3) begin
                    complete_frames<=0;finished_phase<=collect_phase;
                    collect_phase<=!collect_phase;means_ready<=1;
                end else complete_frames<=complete_frames+1'b1;
            end
        end
    end
    wire left=p_x>=16 && p_x<144;
    wire right=p_x>=1136 && p_x<1264;
    wire top=p_y>=16 && p_y<144;
    wire bottom=p_y>=576 && p_y<704;
    wire center=p_x>=576 && p_x<704 && p_y>=296 && p_y<424;
    wire [2:0] region=(left && top)?0:(right && top)?1:center?2:
                      (left && bottom)?3:(right && bottom)?4:7;
    wire sample_en=frame_seen && p_de && p_x[2:0]==3 && p_y[2:0]==3;
    wire [119:0] means;
    genvar z;
    generate for(z=0;z<5;z=z+1) begin:g_zone
        reg [17:0] sum_r,sum_g,sum_b;
        reg [23:0] mean_rgb;
        assign means[z*24 +:24]=mean_rgb;
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin sum_r<=0;sum_g<=0;sum_b<=0;mean_rgb<=0;end
            else begin
                if(window_end) mean_rgb<={sum_b[17:10],sum_g[17:10],sum_r[17:10]};
                if(window_start) begin sum_r<=0;sum_g<=0;sum_b<=0;end
                else if(sample_en && region==z) begin
                    sum_r<=sum_r+{10'd0,p_rgb[23:16]};
                    sum_g<=sum_g+{10'd0,p_rgb[15:8]};
                    sum_b<=sum_b+{10'd0,p_rgb[7:0]};
                end
            end
        end
    end endgenerate
    // One shared sequential decimal converter,8clocks per channel.
    // The~120clock update occurs atframe start,well before panel y64 is scanned.
    reg converting,convert_phase;
    reg [3:0] channel_index,bits_left;
    reg [7:0] binary;
    reg [11:0] decimal;
    reg [11:0] adjusted;
    integer j;
    always @* begin
        adjusted=decimal;
        for(j=0;j<3;j=j+1)
            if(decimal[j*4 +:4]>=5) adjusted[j*4 +:4]=decimal[j*4 +:4]+4'd3;
    end
    wire [11:0] decimal_next={adjusted[10:0],binary[7]};
    wire [6:0] next_byte_offset=({3'd0,channel_index}+7'd1)<<3;
    // Constant output slices avoid a wide variable-write barrel shifter.
    genvar c;
    generate for(c=0;c<15;c=c+1) begin:g_bcd
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin O_bcd[c*12 +:12]<=0;O_bcd[180+c*12 +:12]<=0;end
            else if(converting && bits_left==1 && channel_index==c) begin
                if(!convert_phase) O_bcd[c*12 +:12]<=decimal_next;
                else O_bcd[180+c*12 +:12]<=decimal_next;
            end
        end
    end endgenerate
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            O_ready<=0;converting<=0;convert_phase<=0;
            channel_index<=0;bits_left<=0;binary<=0;decimal<=0;
        end else if(means_ready) begin
            converting<=1;convert_phase<=finished_phase;channel_index<=0;
            binary<=means[7:0];decimal<=0;bits_left<=8;
        end else if(converting) begin
            if(bits_left==1) begin
                if(channel_index==14) begin converting<=0;O_ready[convert_phase]<=1;end
                else begin
                    channel_index<=channel_index+1'b1;binary<=means[next_byte_offset +:8];
                    decimal<=0;bits_left<=8;
                end
            end else begin
                decimal<=decimal_next;binary<={binary[6:0],1'b0};bits_left<=bits_left-1'b1;
            end
        end
    end
endmodule
