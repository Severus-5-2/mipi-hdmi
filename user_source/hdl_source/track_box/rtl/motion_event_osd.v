// Read-only display of the motion monitor. Two pixel clocks of latency,
// matching box_draw and the external overlay input of hdmi_mixer.
module motion_event_osd #(
    parameter integer ROI_X0=320, ROI_X1=960,
    parameter integer ROI_Y0=192, ROI_Y1=576
)(
    input wire I_clk,I_rst_n,I_de,
    input wire [10:0] I_pix_x,
    input wire [9:0] I_pix_y,
    input wire I_color_mode,
    input wire [11:0] I_counts,
    input wire [1:0] I_nr_level,
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
    function [39:0] state_word;
        input [1:0] state;
        begin
            case(state)
                0:state_word="NONE "; 1:state_word="STILL";
                2:state_word="MOVE "; default:state_word="WAIT ";
            endcase
        end
    endfunction
    wire [39:0] zone_word=I_alarm?"ALARM":I_ready?"SAFE ":"WAIT ";
    wire [7:0] event_tens={4'd0,I_events_bcd[7:4]}+8'h30;
    wire [7:0] event_ones={4'd0,I_events_bcd[3:0]}+8'h30;
    reg [127:0] line_text;
    always @* begin
        if(I_color_mode) begin
            case(I_pix_y[6:4])
                3'd5:line_text={"MODE:COLOR NR:",(8'h30+{6'd0,nr_frame})," "}; // y=592
                3'd6:line_text={"RED :",state_word(I_red_state),"      "};
                3'd7:line_text={"BLUE:",state_word(I_blue_state),"      "};
                3'd0:line_text={"DET:",(I_det_full?"FULL":"NORM"),"        "};
                3'd1:line_text={"SW4:1 R",(8'h30+{4'd0,I_counts[3:0]})," B",(8'h30+{4'd0,I_counts[7:4]})," G",(8'h30+{4'd0,I_counts[11:8]}),"  "};
                default:line_text={"AE:",(8'h30+{4'd0,qs_frame[15:12]}),(8'h30+{4'd0,qs_frame[11:8]}),
                    (8'h30+{4'd0,qs_frame[7:4]}),(8'h30+{4'd0,qs_frame[3:0]})," MAX:",
                    (8'h30+{4'd0,qs_frame[27:24]}),(8'h30+{4'd0,qs_frame[23:20]}),(8'h30+{4'd0,qs_frame[19:16]})," "};
            endcase
        end else begin
            case(I_pix_y[6:4])
                3'd5:line_text={"MODE:ALARM NR:",(8'h30+{6'd0,nr_frame})," "};
                3'd6:line_text={"ZONE:",zone_word," N:",event_tens,event_ones," "};
                3'd7:line_text={"ACT:",({4'd0,I_roi_bcd[11:8]}+8'h30),
                    ({4'd0,I_roi_bcd[7:4]}+8'h30),({4'd0,I_roi_bcd[3:0]}+8'h30),"         "};
                3'd0:line_text="AUTO:CLEAR      ";
                3'd1:line_text="SW4:0           ";
                default:line_text={"AE:",(8'h30+{4'd0,qs_frame[15:12]}),(8'h30+{4'd0,qs_frame[11:8]}),
                    (8'h30+{4'd0,qs_frame[7:4]}),(8'h30+{4'd0,qs_frame[3:0]})," MAX:",
                    (8'h30+{4'd0,qs_frame[27:24]}),(8'h30+{4'd0,qs_frame[23:20]}),(8'h30+{4'd0,qs_frame[19:16]})," "};
            endcase
        end
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
    wire roi_border=!I_color_mode && I_de && I_pix_x>=ROI_X0 && I_pix_x<ROI_X1 &&
        I_pix_y>=ROI_Y0 && I_pix_y<ROI_Y1 &&
        (I_pix_x<ROI_X0+2 || I_pix_x>=ROI_X1-2 || I_pix_y<ROI_Y0+2 || I_pix_y>=ROI_Y1-2);
    reg [7:0] char_d;
    reg [2:0] row_d,col_d;
    reg panel_d,roi_d;
    reg cal_bg_d,cal_bg_dd;
    reg [1:0] color_d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            char_d<=0; row_d<=0; col_d<=0; panel_d<=0; roi_d<=0; color_d<=0;cal_bg_d<=0;cal_bg_dd<=0;
        end else begin
            char_d<=selected_char;
            row_d<=I_pix_y[3:1];
            col_d<=(I_pix_x[3:0]-4'd3)>>1;
            panel_d<=(panel || cal_panel) && glyph_area;
            cal_bg_d<=cal_panel;cal_bg_dd<=cal_bg_d;
            roi_d<=roi_border;
            if(I_color_mode) begin
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
    wire glyph_bit=(row_d<7 && col_d<5)?glyph[6'd34-row_offset-{3'b000,col_d}]:1'b0;
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
