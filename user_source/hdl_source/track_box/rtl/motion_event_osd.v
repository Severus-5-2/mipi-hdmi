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
    wire [7:0] selected_char=line_text[127-{col,3'd0} -:8];
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
    reg [1:0] color_d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            char_d<=0; row_d<=0; col_d<=0; panel_d<=0; roi_d<=0; color_d<=0;
        end else begin
            char_d<=selected_char;
            row_d<=I_pix_y[3:1];
            col_d<=(I_pix_x[3:0]-4'd3)>>1;
            panel_d<=panel && glyph_area;
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
    assign O_hit=text_hit || roi_hit;
    assign O_color=text_hit?text_color:zone_color;
endmodule
