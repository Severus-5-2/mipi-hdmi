/*****************************************************************
Company : MiLianKe Electronic Technology Co., Ltd.
WebSite:https://www.milianke.com
TechWeb:https://www.uisrc.com
tmall-shop:https://milianke.tmall.com
jd-shop:https://milianke.jd.com
taobao-shop: https://milianke.taobao.com
Description: 
The reference demo provided by Milianke is only used for learning. 
We cannot ensure that the demo itself is free of bugs, so users 
should be responsible for the technical problems and consequences
caused by the use of their own products.
@Author      :   XiaoQingquan 
@Time        :   2024/10 
version:     :   
@Description :   在原基础上删去了一些赘余的代码，并且优化了时序，加入BRAM IP核，
                 让从除法器出来的每一个数据都可以对上相应的像素数据进行增益，使
                 得它的适用行更加广泛了。
*****************************************************************/
module awb  #(   
    parameter IMG_HEIGHT = 1080,
    parameter IMG_WIDTH  = 1920
) 
(
    input                   I_clk  ,
    input                   I_rst_n,


    input                   I_tlast  ,
    input                   I_tuser  ,
    input [95:0]            I_tdata  ,
    input                   I_tvalid , 
    output                  I_tready ,

    output                  O_tlast  ,
    output                  O_tuser  ,
    output [95:0]           O_tdata  ,
    output                  O_tvalid ,
    input                   O_tready
);

/*****************************************************************
                          RGB565תRGB888                                        
*****************************************************************/

    // //RGB565??????888
    // wire [4:0]  rgb565_r   = I_st_data[11+:5];
    // wire [5:0]  rgb565_g   = I_st_data[5+:6];
    // wire [4:0]  rgb565_b   = I_st_data[0+:5];
    
    // wire [23:0] rgb888     = {{rgb565_r[4:0],rgb565_r[2:0]},{rgb565_g[5:0],rgb565_g[1:0]},{rgb565_b[4:0],rgb565_b[2:0]}};
    
    // wire [7:0]  rgb888_r   = rgb888[16+:8];
    // wire [7:0]  rgb888_g   = rgb888[8+:8] ;
    // wire [7:0]  rgb888_b   = rgb888[0+:8] ;
    
  /*****************************************************************
                          中间信号                                        
*****************************************************************/


  reg         I_tvalid_r0,I_tvalid_r1;
  reg  [95:0] I_tdata_d0,I_tdata_d1;


  //累加RGB通道的�??
  reg  [31:0]         sum_r[0:3];
  reg  [31:0]         sum_g[0:3];
  reg  [31:0]         sum_b[0:3];

  
  wire  [95:0]        data_d0 ;
  wire                valid_d0;
  wire                I_tuser_r0;    

  reg                 I_tuser_r1;

  reg   [95:0]        data_d1 ;
  reg                 valid_d1;
  reg   [9:0]         rgb_sum[0:3];//每个像素点RGB的和
  /*****************************************************************
                            平均值计�??                                      
*****************************************************************/

// 对输入数据和有效信号进行寄存，形成流水线
always @(posedge I_clk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        I_tdata_d0  <= 0;
        I_tdata_d1  <= 0;
        {I_tvalid_r0,I_tvalid_r1} <= 0;
    end else begin
        I_tdata_d0  <= I_tdata   ;
        I_tdata_d1  <= I_tdata_d0;
        I_tvalid_r0 <= I_tvalid;
        I_tvalid_r1 <= I_tvalid_r0;
    end
end

// 将输入数据拆分为4个像素的RGB分量
wire [7:0] rgb888_r_d[0:3];
wire [7:0] rgb888_g_d[0:3];
wire [7:0] rgb888_b_d[0:3];

assign  rgb888_r_d[0] = I_tdata[16+:8];
assign  rgb888_g_d[0] = I_tdata[8+:8];
assign  rgb888_b_d[0] = I_tdata[0+:8];


// 将输入数据拆分为4个像素的RGB分量
wire [7:0] rgb888_r_d0[0:3];
wire [7:0] rgb888_g_d0[0:3];
wire [7:0] rgb888_b_d0[0:3];

assign  rgb888_r_d0[0] = I_tdata_d0[16+:8];
assign  rgb888_g_d0[0] = I_tdata_d0[8+:8];
assign  rgb888_b_d0[0] = I_tdata_d0[0+:8];


    // Complete-frame gray-world statistics. All four lanes contribute equally;
    // clipped whites and near-black samples do not determine channel gains.
    reg user_d;
    wire frame_boundary = I_tuser && !user_d;
    always @(posedge I_clk or negedge I_rst_n)
        if (!I_rst_n) user_d <= 0; else user_d <= I_tuser;

    reg [9:0] group_r, group_g, group_b;
    reg group_any;
    integer lane;
    reg [9:0] lane_sum;
    always @* begin
        group_r=0; group_g=0; group_b=0; group_any=0; lane_sum=0;
        for (lane=0; lane<4; lane=lane+1) begin
            lane_sum={2'b0,I_tdata_d0[lane*24+16 +:8]} +
                     {2'b0,I_tdata_d0[lane*24+8 +:8]} + {2'b0,I_tdata_d0[lane*24 +:8]};
            if (lane_sum>10'd12 && lane_sum<10'd720) begin
                group_any=1;
                group_r=group_r+I_tdata_d0[lane*24+16 +:8];
                group_g=group_g+I_tdata_d0[lane*24+8 +:8];
                group_b=group_b+I_tdata_d0[lane*24 +:8];
            end
        end
    end
    reg [9:0] stats_r,stats_g,stats_b;
    // Broad neutral eligibility (linear RGB). Strongly colored surfaces
    // must not steer the entire frame's white balance. Four-to-one range
    // still admits a neutral reference under a pronounced warm/cool cast.
    wire [9:0] group_min_rg=(stats_r<stats_g)?stats_r:stats_g;
    wire [9:0] group_min=(group_min_rg<stats_b)?group_min_rg:stats_b;
    wire [9:0] group_max_rg=(stats_r>stats_g)?stats_r:stats_g;
    wire [9:0] group_max=(group_max_rg>stats_b)?group_max_rg:stats_b;
    wire [11:0] group_light={2'b00,stats_r}+{2'b00,stats_g}+{2'b00,stats_b};
    wire neutral_group=group_light>=12'd48 && {2'b00,group_max}<={group_min,2'b00};
    reg [8:0] neutral_count;
    reg [9:0] neutral_r,neutral_g,neutral_b;
    reg neutral_valid;
    reg stats_valid,stats_nonempty;
    reg [31:0] frame_r,frame_g,frame_b;
    reg gain_start;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            stats_r<=0; stats_g<=0; stats_b<=0; stats_valid<=0;
            sum_r[0]<=1; sum_g[0]<=1; sum_b[0]<=1;
            frame_r<=1; frame_g<=1; frame_b<=1;
            stats_nonempty<=0; gain_start<=0;neutral_count<=0;
            neutral_r<=0;neutral_g<=0;neutral_b<=0;neutral_valid<=0;
        end else begin
            stats_r<=group_r; stats_g<=group_g; stats_b<=group_b;
            stats_valid<=I_tvalid_r0 && group_any && !frame_boundary;
            // Separate sum -> neutral classification -> accumulation stages.
            neutral_r<=stats_r;neutral_g<=stats_g;neutral_b<=stats_b;
            neutral_valid<=stats_valid && neutral_group && !frame_boundary;
            gain_start<=0;
            if (frame_boundary) begin
                if (stats_nonempty && neutral_count>=256) begin
                    frame_r<=sum_r[0]; frame_g<=sum_g[0]; frame_b<=sum_b[0];
                    gain_start<=1;
                end
                sum_r[0]<=1; sum_g[0]<=1; sum_b[0]<=1;
                stats_nonempty<=0;neutral_count<=0;
            end else if (neutral_valid) begin
                if(neutral_count<256) neutral_count<=neutral_count+1'b1;
                sum_r[0]<=sum_r[0]+neutral_r;
                sum_g[0]<=sum_g[0]+neutral_g;
                sum_b[0]<=sum_b[0]+neutral_b;
                stats_nonempty<=1;
            end
        end
    end

  wire [47:0] dout_R[3:0];
  wire [47:0] dout_B[3:0];

  wire done[3:0];

  wire [47:0] in_r_x1;
  wire [47:0] in_b_x1;
  assign in_r_x1 = dout_R[0][47:0];
  assign in_b_x1 = dout_B[0][47:0];



/*****************************************************************
                   IP 调用                                    
*****************************************************************/
    //为了保证合理的延迟，这里使用了除法器

      divider div_gen_G_R_r1 (
          .clk(I_clk),  // input wire aclk
          .rst(!I_rst_n),
          .start(gain_start),
          .denominator(frame_r),  // input wire [31 : 0] s_axis_divisor_tdata
          .numerator({frame_g,16'd0}),  // input wire [47 : 0] s_axis_dividend_tdata
          .quotient(dout_R[0]),  // output wire [79 : 0] m_axis_dout_tdata
          .done(done[0])
      );

      divider divide_G_B_r1 (
          .clk(I_clk),  // input wire aclk
          .rst(!I_rst_n),
          .start(gain_start),
          .denominator(frame_b),  // input wire [31 : 0] s_axis_divisor_tdata
          .numerator({frame_g,16'd0}),  // input wire [47 : 0] s_axis_dividend_tdata
          .quotient(dout_B[0]),  // output wire [79 : 0] m_axis_dout_tdata
          .done()
      );

     
    signal_delay #(
        .IMG_HEIGHT   (IMG_HEIGHT),
        .IMG_WIDTH    (IMG_WIDTH ),
        .DATA_WIDTH   (96  ),
        .DELAY_CYCLE  (50)
    )signal_delay_d (
        /*input                       */ .I_clk  (I_clk  ),
        /*input                       */ .I_rst_n(I_rst_n),
        /*input                       */ .I_tuser(I_tuser),  //synthesis keep 
        /*input  wire                 */ .I_valid(I_tvalid_r1),  //synthesis keep 
        /*input  wire [DATA_WIDTH-1:0]*/ .I_data (I_tdata_d1 ),   //synthesis keep 
        /*output wire                 */ .O_valid(valid_d0   ),  //synthesis keep 
        /*output wire [DATA_WIDTH-1:0]*/ .O_data (data_d0    ),   //synthesis keep 
        /*output wire                 */ .O_tuser(I_tuser_r0 )
    );

    

/*****************************************************************
                            ��������                                      
*****************************************************************/

    reg [19:0] in_r_r1,in_b_r1;
    reg [19:0] pending_r,pending_b;
    function [19:0] stable_gain;
        input [19:0] previous;
        input [47:0] quotient;
        reg [19:0] limited;
        reg signed [21:0] delta,updated;
        begin
            limited=(quotient<48'h08000)?20'h08000:
                    (quotient>48'h40000)?20'h40000:quotient[19:0];
            delta=$signed({2'b00,limited})-$signed({2'b00,previous});
            updated=$signed({2'b00,previous})+(delta>>>2);
            stable_gain=updated[19:0];
        end
    endfunction
    // A divider result updates the pending gain; only a frame boundary may
    // change active gains. No spatial-prefix color drift within a frame.
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            in_r_r1<=20'h10000; in_b_r1<=20'h10000;
            pending_r<=20'h10000; pending_b<=20'h10000;
        end else begin
            if (done[0]) begin
                pending_r<=stable_gain(pending_r,in_r_x1);
                pending_b<=stable_gain(pending_b,in_b_x1);
            end
            if (frame_boundary) begin
                in_r_r1<=pending_r; in_b_r1<=pending_b;
            end
        end
    end

    ////信号同步
    always @(posedge I_clk or negedge I_rst_n) begin
      if (!I_rst_n) begin
          valid_d1 <= 0;
          data_d1  <= 0;
          I_tuser_r1 <= 0;
      end else begin
          valid_d1 <= valid_d0;
          data_d1  <= data_d0 ;
          I_tuser_r1 <= I_tuser_r0;
      end
    end
/*****************************************************************
                           ��������                                       
*****************************************************************/

    reg [27:0] R_new_r1;
    reg [27:0] G_new_r1;
    reg [27:0] B_new_r1;

    reg [27:0] R_new_r2;
    reg [27:0] G_new_r2;
    reg [27:0] B_new_r2;

    reg [27:0] R_new_r3;
    reg [27:0] G_new_r3;
    reg [27:0] B_new_r3;

    reg [27:0] R_new_r4;
    reg [27:0] G_new_r4;
    reg [27:0] B_new_r4;


    wire   [7:0] rgb888_r_d1[0:3];
    wire   [7:0] rgb888_g_d1[0:3];
    wire   [7:0] rgb888_b_d1[0:3];

    assign  rgb888_r_d1[0] = data_d1[16+:8];
    assign  rgb888_g_d1[0] = data_d1[8+:8];
    assign  rgb888_b_d1[0] = data_d1[0+:8];

    assign  rgb888_r_d1[1] = data_d1[40+:8];
    assign  rgb888_g_d1[1] = data_d1[32+:8];
    assign  rgb888_b_d1[1] = data_d1[24+:8];

    assign  rgb888_r_d1[2] = data_d1[64+:8];
    assign  rgb888_g_d1[2] = data_d1[56+:8];
    assign  rgb888_b_d1[2] = data_d1[48+:8];

    assign  rgb888_r_d1[3] = data_d1[88+:8];
    assign  rgb888_g_d1[3] = data_d1[80+:8];
    assign  rgb888_b_d1[3] = data_d1[72+:8];
    always @(posedge I_clk or negedge I_rst_n)begin 
        if(!I_rst_n)begin
            R_new_r1 <= 0;
            G_new_r1 <= 0;
            B_new_r1 <= 0;

            R_new_r2 <= 0;
            G_new_r2 <= 0;
            B_new_r2 <= 0;

            R_new_r3 <= 0;
            G_new_r3 <= 0;
            B_new_r3 <= 0;

            R_new_r4 <= 0;
            G_new_r4 <= 0;
            B_new_r4 <= 0;
        end
        else if(valid_d1) begin
            R_new_r1 <= in_r_r1 * rgb888_r_d1[0];
            G_new_r1 <=           rgb888_g_d1[0];
            B_new_r1 <= in_b_r1 * rgb888_b_d1[0];

            R_new_r2 <= in_r_r1 * rgb888_r_d1[1];
            G_new_r2 <=           rgb888_g_d1[1];
            B_new_r2 <= in_b_r1 * rgb888_b_d1[1];

            R_new_r3 <= in_r_r1 * rgb888_r_d1[2];
            G_new_r3 <=           rgb888_g_d1[2];
            B_new_r3 <= in_b_r1 * rgb888_b_d1[2];

            R_new_r4 <= in_r_r1 * rgb888_r_d1[3];
            G_new_r4 <=           rgb888_g_d1[3];
            B_new_r4 <= in_b_r1 * rgb888_b_d1[3];
        end
    end

/*****************************************************************
                            �������???                                      
*****************************************************************/

    wire [7:0] R_new_x1;
    wire [7:0] G_new_x1;
    wire [7:0] B_new_x1;

    wire [7:0] R_new_x2;
    wire [7:0] G_new_x2;
    wire [7:0] B_new_x2;

    wire [7:0] R_new_x3;
    wire [7:0] G_new_x3;
    wire [7:0] B_new_x3;

    wire [7:0] R_new_x4;
    wire [7:0] G_new_x4;
    wire [7:0] B_new_x4;

    assign R_new_x1 = (R_new_r1[27:24])?255:R_new_r1[23:16];
    assign G_new_x1 =  G_new_r1;
    assign B_new_x1 = (B_new_r1[27:24])?255:B_new_r1[23:16];

    assign R_new_x2 = (R_new_r2[27:24])?255:R_new_r2[23:16];
    assign G_new_x2 =  G_new_r2;
    assign B_new_x2 = (B_new_r2[27:24])?255:B_new_r2[23:16];

    assign R_new_x3 = (R_new_r3[27:24])?255:R_new_r3[23:16];
    assign G_new_x3 =  G_new_r3;
    assign B_new_x3 = (B_new_r3[27:24])?255:B_new_r3[23:16];

    assign R_new_x4 = (R_new_r4[27:24])?255:R_new_r4[23:16];
    assign G_new_x4 =  G_new_r4;
    assign B_new_x4 = (B_new_r4[27:24])?255:B_new_r4[23:16];
/*****************************************************************
                           ͬ���ź�                                       
*****************************************************************/

    reg valid_d2,I_tuser_r2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)begin
            valid_d2 <= 0;
            I_tuser_r2 <= 0;
        end
        else begin
            valid_d2 <= valid_d1;
            I_tuser_r2 <= I_tuser_r1;
        end
    end


    assign  O_tlast  = (valid_d1 == 0) && (valid_d2 == 1);   
    assign  O_tuser  = I_tuser ;   
    assign  O_tvalid = valid_d2;

    assign  O_tdata = O_tvalid?{R_new_x4,G_new_x4,B_new_x4,
                       R_new_x3,G_new_x3,B_new_x3,
                       R_new_x2,G_new_x2,B_new_x2,
                       R_new_x1,G_new_x1,B_new_x1}:0;
    // assign O_t_data = {R_new[7-:5],G_new[7-:6],B_new[7-:5]};
    assign  I_tready = O_tready;
 
endmodule



