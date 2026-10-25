module isp_top (
    input 			axi4s_video_aclk,
    input 			I_rst_n			,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    input 			O_tready		,
    output [127:0] 	O_tdata			,
    output 			O_tlast			,
    output 			O_tuser			,
    output 			O_tvalid		,
    output 			I_tready
);
 
wire         m_aixs_tvalid;  //synthesis keep 
  wire [127:0] m_aixs_tdata;  //synthesis keep 
  wire         m_aixs_tuser;  //synthesis keep 
  wire         m_aixs_tlast;  //synthesis keep 
  wire         m_aixs_tready;  //synthesis keep 
  
  wire [ 95:0] m_aixs_tdata_96;  //synthesis keep 


  wire         awb_O_tlast;  //synthesis keep 
  wire         awb_O_tuser;  //synthesis keep 
  wire [ 95:0] awb_O_tdata;  //synthesis keep 
  wire         awb_O_tvalid;  //synthesis keep 
  wire         awb_O_tready;  //synthesis keep 

  wire         awb_O_tuser_128;  //synthesis keep 
  wire         awb_O_tvalid_128;  //synthesis keep 
  wire [127:0] awb_O_tdata_128;  //synthesis keep 

  assign awb_O_tready = O_tready;
  assign O_tdata  = awb_O_tdata_128;
  assign O_tlast  = awb_O_tlast    ;
  assign O_tuser  = awb_O_tuser_128    ;
  assign O_tvalid = awb_O_tvalid_128   ;
  
  //assign m_aixs_tready = O_tready;
  //assign O_tdata  = m_aixs_tdata;
  //assign O_tlast  = m_aixs_tlast    ;
  //assign O_tuser  = m_aixs_tuser    ;
  //assign O_tvalid = m_aixs_tvalid   ;
 
 
demosaic #(
    .IMG_HEIGHT          (720 ), // 图像高度
    .IMG_WIDTH           (1280), // 图像宽度
    .data_complete_delay (50  ),
    .BAYER_MODE          ("BGGR")
)
u_demosaic
(
    .I_clk   			(axi4s_video_aclk)	,   // 时钟信号
    .I_rst_n 			(I_rst_n)	,   // 复位信号，低有效
    .axi4s_video_tdata 	(I_tdata)	,  // AXI4-Stream视频数据
    .axi4s_video_tdest 	(I_tdest)	,
    .axi4s_video_tlast 	(I_tlast)	,  // 行结束信号
    .axi4s_video_tvalid	(I_tvalid)	,  // 数据有效信号
    .axi4s_video_tuser 	(I_tuser)	,  // 帧开始信号
    .axi4s_video_tready	(I_tready)	,  // 从模块准备好接受数据
    .O_tlast  			(m_aixs_tlast)	,  // 输出行结束信号
    .O_tuser  			(m_aixs_tuser)	,  // 输出帧开始信号
    .O_tdata  			(m_aixs_tdata)	,  // 输出数据
    .O_tvalid 			(m_aixs_tvalid)	,  // 输出数据有效信号
    .O_tready   		(m_aixs_tready)	   // 输出数据准备好信号
);
 
data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(m_aixs_tdata_96)
);

awb #(
    .IMG_HEIGHT(720 ),
    .IMG_WIDTH (1280)
) u_awb (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (m_aixs_tlast),
    .I_tuser (m_aixs_tuser),
    .I_tdata (m_aixs_tdata_96),
    .I_tvalid(m_aixs_tvalid),
    .I_tready(m_aixs_tready),
    .O_tlast (awb_O_tlast ),
    .O_tuser (awb_O_tuser ),
    .O_tdata (awb_O_tdata ),
    .O_tvalid(awb_O_tvalid),
    .O_tready(awb_O_tready)
);

    data_96bit_to_128bit u_data_96bit_to_128bit(
        .I_clk              ( axi4s_video_aclk  ),
        .I_rst_n            ( I_rst_n           ),
	
        .I_96b_frame_start  ( awb_O_tuser	    ),
        .I_96b_valid        ( awb_O_tvalid 		),
        .I_96b_data         ( awb_O_tdata		),
	
        .O_128b_frame_start ( awb_O_tuser_128  	),
        .O_128b_valid       ( awb_O_tvalid_128 	),
        .O_128b_data        ( awb_O_tdata_128  	)
    );



//data96_128 u_data96_128 (
//    .I_tdata(awb_O_tdata),
//    .O_tdata(awb_O_tdata_128)
//);
	
endmodule

// RAW10 Bayer noise reduction, four pixels per word.
// Same-CFA taps: horizontal offsets -4,-2,0,+2,+4 and rows 0,-2,-4.
// Valid/user/last travel with each word; no downstream backpressure is assumed,
// matching the existing uial2axis -> ISP connection (I_tready unused).
module raw_bayer_nr #(
    parameter WORDS_PER_LINE=320,
    parameter IMAGE_HEIGHT=720
)(
    input wire clk,input wire rst_n,input wire [15:0] I_gain,
    input wire [39:0] I_data,input wire I_valid,I_user,I_last,
    output reg [39:0] O_data,output reg O_valid,O_user,O_last,
    output wire [1:0] O_nr_level
);
    reg [15:0] gain_meta,gain_sync,gain_prev;
    reg [1:0] level;
    assign O_nr_level=level; // Frame-held actual RAW filter mode, status only.
    reg [8:0] in_x;
    reg [9:0] in_y;
    reg [39:0] mid_word,left_word;
    reg mid_valid,mid_user,mid_last;
    reg [8:0] mid_x;
    reg [9:0] mid_y;
    reg [1:0] mid_level;
    function [1:0] choose_level;
        input [15:0] gain_value;input [1:0] previous;
        begin
            case(previous)
            0:choose_level=(gain_value>=160)?2:(gain_value>=96)?1:0;
            1:choose_level=(gain_value>=160)?2:(gain_value<=88)?0:1;
            default:choose_level=(gain_value<=88)?0:(gain_value<=152)?1:2;
            endcase
        end
    endfunction
    wire [1:0] next_level=(gain_sync==gain_prev)?choose_level(gain_sync,level):level;
    wire emit_word=mid_valid && (mid_last || I_valid);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            gain_meta<=0;gain_sync<=0;gain_prev<=0;level<=0;
            in_x<=0;in_y<=0;mid_word<=0;left_word<=0;
            mid_valid<=0;mid_user<=0;mid_last<=0;mid_x<=0;mid_y<=0;mid_level<=0;
        end else begin
            gain_meta<=I_gain;gain_sync<=gain_meta;gain_prev<=gain_sync;
            if(I_valid) begin
                left_word<=mid_word;mid_word<=I_data;mid_valid<=1;
                mid_user<=I_user;mid_last<=I_last;
                mid_x<=I_user?9'd0:in_x;mid_y<=I_user?10'd0:in_y;
                mid_level<=I_user?next_level:level;
                if(I_user) level<=next_level;
                if(I_last) begin
                    in_x<=0;
                    in_y<=(I_user?0:in_y)==IMAGE_HEIGHT-1?10'd0:(I_user?10'd1:in_y+1'b1);
                end else begin in_x<=(I_user?9'd0:in_x)+1'b1;in_y<=I_user?10'd0:in_y;end
            end else if(mid_valid && mid_last) mid_valid<=0;
        end
    end
    // Clamp missing horizontal neighbors to the same-CFA center sample.
    function [9:0] tap;
        input integer lane,offset;
        integer q;
        begin
            q=lane+offset;
            if(q<0) tap=(mid_x==0)?mid_word[lane*10 +:10]:left_word[(q+4)*10 +:10];
            else if(q>3) tap=mid_last?mid_word[lane*10 +:10]:I_data[(q-4)*10 +:10];
            else tap=mid_word[q*10 +:10];
        end
    endfunction
    reg [43:0] h_outer;
    reg [47:0] h_inner;
    reg [39:0] h1_raw,h2_raw,h2_data;
    reg h1_valid,h1_user,h1_last,h2_valid,h2_user,h2_last;
    reg [9:0] h1_y,h2_y;
    reg [1:0] h1_level,h2_level;
    reg [8:0] h1_addr,h2_addr;
    // Clock-only address registers permit embedded RAM inference; valid resets.
    always @(posedge clk) begin h1_addr<=mid_x;h2_addr<=h1_addr;end
    genvar k;
    generate for(k=0;k<4;k=k+1) begin:g_horizontal
        wire [10:0] outer_sum={1'b0,tap(k,-4)}+{1'b0,tap(k,4)};
        wire [11:0] inner_sum={2'b00,tap(k,-2)}+{2'b00,tap(k,0)}+{2'b00,tap(k,2)};
        wire [12:0] total={2'b00,h_outer[k*11 +:11]}+{h_inner[k*12 +:12],1'b0}+13'd4;
        always @(posedge clk) begin
            h_outer[k*11 +:11]<=outer_sum;
            h_inner[k*12 +:12]<=inner_sum;
            h2_data[k*10 +:10]<=total[12:3];
        end
    end endgenerate
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            h1_valid<=0;h1_user<=0;h1_last<=0;h2_valid<=0;h2_user<=0;h2_last<=0;
            h1_raw<=0;h2_raw<=0;h1_y<=0;h2_y<=0;h1_level<=0;h2_level<=0;
        end else begin
            h1_valid<=emit_word;h1_user<=mid_user;h1_last<=mid_last;
            h1_raw<=mid_word;h1_y<=mid_y;h1_level<=mid_level;
            h2_valid<=h1_valid;h2_user<=h1_user;h2_last<=h1_last;
            h2_raw<=h1_raw;h2_y<=h1_y;h2_level<=h1_level;
        end
    end
    wire [39:0] row1,row2,row3,row4;
    reg [3:0] dv,du,dl;
    reg [8:0] addr1,addr2,addr3;
    reg [39:0] cur1,cur2,cur3,cur4,raw1,raw2,raw3,raw4,p2a,p2b;
    reg [39:0] row_y;
    reg [7:0] row_level;
    always @(posedge clk) begin
        addr1<=h2_addr;addr2<=addr1;addr3<=addr2;
        cur1<=h2_data;cur2<=cur1;cur3<=cur2;cur4<=cur3;
        raw1<=h2_raw;raw2<=raw1;raw3<=raw2;raw4<=raw3;
        p2a<=row2;p2b<=p2a;
    end
    raw_nr_line_ram u_row1(clk,h2_valid,h2_addr,h2_data,row1);
    raw_nr_line_ram u_row2(clk,dv[0],addr1,row1,row2);
    raw_nr_line_ram u_row3(clk,dv[1],addr2,row2,row3);
    raw_nr_line_ram u_row4(clk,dv[2],addr3,row3,row4);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin dv<=0;du<=0;dl<=0;row_y<=0;row_level<=0;end
        else begin
            dv<={dv[2:0],h2_valid};du<={du[2:0],h2_user};dl<={dl[2:0],h2_last};
            row_y<={row_y[29:0],h2_y};row_level<={row_level[5:0],h2_level};
        end
    end
    reg [39:0] v_filtered,v_raw;
    reg v_valid,v_user,v_last;
    reg [1:0] v_level;
    generate for(k=0;k<4;k=k+1) begin:g_vertical
        wire [9:0] c=cur4[k*10 +:10];
        wire [9:0] p=(row_y[39:30]<2)?c:p2b[k*10 +:10];
        wire [9:0] q=(row_y[39:30]<4)?p:row4[k*10 +:10];
        wire [11:0] sum={2'b00,c}+{1'b0,p,1'b0}+{2'b00,q}+12'd2;
        always @(posedge clk) v_filtered[k*10 +:10]<=sum[11:2];
    end endgenerate
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin v_raw<=0;v_valid<=0;v_user<=0;v_last<=0;v_level<=0;end
        else begin
            v_raw<=raw4;v_valid<=dv[3];v_user<=du[3];v_last<=dl[3];v_level<=row_level[7:6];
        end
    end
    wire [3:0] edge_lane;
    wire [39:0] blended;
    generate for(k=0;k<4;k=k+1) begin:g_edge
        wire [9:0] r=v_raw[k*10 +:10],f=v_filtered[k*10 +:10];
        wire [9:0] delta=(r>=f)?r-f:f-r;
        wire [10:0] mix={1'b0,r}+{1'b0,f}+11'd1;
        // Large differences choose HALF filtering, never raw-only veto.
        // Noise spikes can exceed an edge threshold too.
        assign edge_lane[k]=(delta>10'd256); // Only very large contrast halves strong NR
        assign blended[k*10 +:10]=mix[10:1];
    end endgenerate
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin O_data<=0;O_valid<=0;O_user<=0;O_last<=0;end
        else begin
            O_valid<=v_valid;O_user<=v_user && v_valid;O_last<=v_last && v_valid;
            // Keep bright bypass exact; all enabled modes retain real averaging.
            // Whole-word half strength keeps the four CFA lanes consistent.
            if(v_level==0) O_data<=v_raw;
            else O_data<=(v_level==2 && !(|edge_lane))?v_filtered:blended;
        end
    end
endmodule

module raw_nr_line_ram(
    input wire clk,input wire wen,input wire [8:0] addr,
    input wire [39:0] din,output reg [39:0] dout
);
    (* ram_style="bram" *) reg [39:0] mem[0:319];
    always @(posedge clk) begin
        if(wen) mem[addr]<=din;
        dout<=mem[addr];
    end
endmodule
