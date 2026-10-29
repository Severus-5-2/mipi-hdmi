// Qualified weak-gradient Sobel; black background and white contours.
// Gray/DE/frame-first must be same-phase. Two line RAMs, never reset.
// Window sampled at clock c produces rgb_out at c+3, centered on the
// previous input row and column. Frame-first flushes pending old responses.
module sobel_3x3 #(
    parameter integer IMG_WIDTH=1280, IMG_HEIGHT=720,
    parameter integer EDGE_TH=63, DARK_TH=31, MID_TH=47,
    parameter integer SUPPORT_TH=8, BRIGHT_SUPPORT_TH=12
)(
    input wire clk,rst_n,I_de,I_frame,
    input wire [7:0] gray_in,
    output reg [23:0] rgb_out
);
    (* ram_style="dram" *) reg [7:0] line_buf0[0:IMG_WIDTH-1];
    (* ram_style="bram" *) reg [7:0] line_buf1[0:IMG_WIDTH-1];
    reg [10:0] h_cnt;
    reg [9:0] v_cnt;
    wire [10:0] write_x=I_frame?11'd0:h_cnt;
    wire [9:0] write_y=I_frame?10'd0:v_cnt;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin h_cnt<=0;v_cnt<=0;end
        else if(I_de) begin
            if(write_x==IMG_WIDTH-1) begin
                h_cnt<=0;v_cnt<=(write_y==IMG_HEIGHT-1)?10'd0:write_y+1'b1;
            end else begin h_cnt<=write_x+1'b1;v_cnt<=write_y;end
        end else if(I_frame) begin h_cnt<=0;v_cnt<=0;end
    end
    always @(posedge clk) if(I_de) begin
        line_buf1[write_x]<=line_buf0[write_x];
        line_buf0[write_x]<=gray_in;
    end
    reg [7:0] p00,p01,p02,p10,p11,p12,p20,p21,p22;
    always @(posedge clk) if(I_de) p02<=line_buf1[write_x];
    reg window_valid;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            {p00,p01,p10,p11,p12,p20,p21,p22}<=0;window_valid<=0;
        end else begin
            window_valid<=I_de && write_y>=2 && write_x>=2;
            if(I_de) begin
                if(write_x==0) begin
                    p00<=0;p01<=0;p10<=0;p11<=0;p20<=0;p21<=0;
                end else begin
                    p00<=p01;p01<=p02;p10<=p11;p11<=p12;p20<=p21;p21<=p22;
                end
                p12<=line_buf0[write_x];p22<=gray_in;
            end else if(I_frame) {p00,p01,p10,p11,p12,p20,p21,p22}<=0;
        end
    end
    wire signed [8:0] dx0=$signed({1'b0,p02})-$signed({1'b0,p00});
    wire signed [8:0] dx1=$signed({1'b0,p12})-$signed({1'b0,p10});
    wire signed [8:0] dx2=$signed({1'b0,p22})-$signed({1'b0,p20});
    wire signed [8:0] dy0=$signed({1'b0,p20})-$signed({1'b0,p00});
    wire signed [8:0] dy1=$signed({1'b0,p21})-$signed({1'b0,p01});
    wire signed [8:0] dy2=$signed({1'b0,p22})-$signed({1'b0,p02});
    wire signed [8:0] h00=$signed({1'b0,p00})-$signed({1'b0,p01});
    wire signed [8:0] h01=$signed({1'b0,p01})-$signed({1'b0,p02});
    wire signed [8:0] h10=$signed({1'b0,p10})-$signed({1'b0,p11});
    wire signed [8:0] h11=$signed({1'b0,p11})-$signed({1'b0,p12});
    wire signed [8:0] h20=$signed({1'b0,p20})-$signed({1'b0,p21});
    wire signed [8:0] h21=$signed({1'b0,p21})-$signed({1'b0,p22});
    wire signed [8:0] support_now=(p11>=160)?BRIGHT_SUPPORT_TH:SUPPORT_TH;
    reg signed [8:0] x0_s1,x1_s1,x2_s1,y0_s1,y1_s1,y2_s1,support_s1;
    reg [7:0] threshold_s1,threshold_s2;
    reg [5:0] hp_s1,hn_s1;
    reg valid_s1,valid_s2;
    reg signed [10:0] gx_s2,gy_s2;
    reg x_support_s2,y_support_s2,diagonal_s2;
    function coherent;
        input signed [8:0] a,b,c,floor;
        input negative_gradient;
        begin
            coherent=(!negative_gradient && (((a>=floor)&&(b>=floor))||
                       ((a>=floor)&&(c>=floor))||((b>=floor)&&(c>=floor))))||
                     (negative_gradient && (((a<=-floor)&&(b<=-floor))||
                       ((a<=-floor)&&(c<=-floor))||((b<=-floor)&&(c<=-floor))));
        end
    endfunction
    function signed [10:0] wide;
        input signed [8:0] value;
        begin wide={{2{value[8]}},value};end
    endfunction
    wire signed [10:0] gx_calc_s1=wide(x0_s1)+(wide(x1_s1)<<<1)+wide(x2_s1);
    wire signed [10:0] gy_calc_s1=wide(y0_s1)+(wide(y1_s1)<<<1)+wide(y2_s1);
    wire [10:0] abs_x=gx_s2[10]?-gx_s2:gx_s2;
    wire [10:0] abs_y=gy_s2[10]?-gy_s2:gy_s2;
    wire [10:0] magnitude=abs_x+abs_y;
    wire [10:0] supported_magnitude=(x_support_s2?abs_x:11'd0)+
                                    (y_support_s2?abs_y:11'd0);
    wire edge_hit=(supported_magnitude>threshold_s2)||
                  (diagonal_s2 && magnitude>threshold_s2);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            x0_s1<=0;x1_s1<=0;x2_s1<=0;y0_s1<=0;y1_s1<=0;y2_s1<=0;
            support_s1<=0;threshold_s1<=0;threshold_s2<=0;hp_s1<=0;hn_s1<=0;
            gx_s2<=0;gy_s2<=0;x_support_s2<=0;y_support_s2<=0;diagonal_s2<=0;
            valid_s1<=0;valid_s2<=0;rgb_out<=0;
        end else begin
            // Stage1: differences and independent diagonal contrast signs.
            x0_s1<=dx0;x1_s1<=dx1;x2_s1<=dx2;y0_s1<=dy0;y1_s1<=dy1;y2_s1<=dy2;
            support_s1<=support_now;
            threshold_s1<=(p11<96)?DARK_TH:((p11<160)?MID_TH:EDGE_TH);
            hp_s1<={h21>=support_now,h20>=support_now,h11>=support_now,
                    h10>=support_now,h01>=support_now,h00>=support_now};
            hn_s1<={h21<=-support_now,h20<=-support_now,h11<=-support_now,
                    h10<=-support_now,h01<=-support_now,h00<=-support_now};
            valid_s1<=window_valid;
            // Stage2: standard Sobel weights and qualified axes. Opposite
            // contrasts in adjacent rows rescue thin diagonal contours.
            gx_s2<=gx_calc_s1;gy_s2<=gy_calc_s1;
            x_support_s2<=coherent(x0_s1,x1_s1,x2_s1,support_s1,gx_calc_s1[10]);
            y_support_s2<=coherent(y0_s1,y1_s1,y2_s1,support_s1,gy_calc_s1[10]);
            diagonal_s2<=(hp_s1[0]&&hn_s1[2])||(hn_s1[0]&&hp_s1[2])||
                         (hp_s1[1]&&hn_s1[3])||(hn_s1[1]&&hp_s1[3])||
                         (hp_s1[2]&&hn_s1[4])||(hn_s1[2]&&hp_s1[4])||
                         (hp_s1[3]&&hn_s1[5])||(hn_s1[3]&&hp_s1[5]);
            threshold_s2<=threshold_s1;valid_s2<=valid_s1;
            // Stage3: black invalid windows; keep the public binary output.
            rgb_out<=(valid_s2 && edge_hit)?24'hffffff:24'h000000;
            if(I_frame) begin valid_s1<=0;valid_s2<=0;rgb_out<=0;end
        end
    end
endmodule
