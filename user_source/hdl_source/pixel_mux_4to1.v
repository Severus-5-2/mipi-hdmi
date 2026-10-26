//====================================================================
// 模块功能：四路24bit RGB像素数据流选择器
// mode[1:0]模式定义：
//   2'b00 → rgb0：原始摄像头图像
//   2'b01 → rgb1：灰度图
//   2'b10 → rgb2：二值化图像
//   2'b11 → rgb3：Sobel边缘检测图像
// 时钟域：S_hdmi_pixel_clk
//====================================================================
module pixel_mux_4to1 #(
parameter integer DISPLAY_NR_AB=0,
parameter integer DISPLAY_NR_FRAMES=240
)(
input               clk,        //HDMI像素时钟
input               rst_n,      //低电平复位
input               I_de, I_frame, // Canvas-aligned enables, no added output latency
output wire         O_display_nr_active,
input      [1:0]    mode,       //模式选择信号
input      [23:0]   rgb0,       //mode00 原图
input      [23:0]   rgb1,       //mode01 灰度图
input      [23:0]   rgb2,       //mode10 二值图
input      [23:0]   rgb3,       //mode11 Sobel边缘图
output reg [23:0]   rgb_out     //选中之后输出RGB像素
);

// Only the selected color display is filtered. rgb0 remains unchanged for
// detection, metering and every other consumer in the top level.
wire [23:0] display_rgb0;
generate if(DISPLAY_NR_AB) begin:g_display_nr
    reg [7:0] frames;
    reg phase;
    reg de_prev;
    (* syn_preserve=1 *) reg [71:0] history;
    wire first_pixel=I_frame || !de_prev;
    wire flip=I_frame && frames==DISPLAY_NR_FRAMES-1;
    // Look ahead on the frame's first pixel so the entire frame, including
    // that pixel and its OSD sample, uses exactly one comparison phase.
    wire active=phase ^ flip;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin frames<=0;phase<=0;de_prev<=0;history<=0;end
        else begin
            de_prev<=I_de;
            if(I_frame) begin
                if(flip) begin frames<=0;phase<=~phase;end
                else frames<=frames+1'b1;
            end
            if(I_de) begin
                if(first_pixel) history<={3{rgb0}};
                else history<={history[47:0],rgb0};
            end
        end
    end
    // Causal horizontal two-sample, stride-three filter. Outlying neighbors are replaced
    // by the CURRENT pixel jointly for all RGB channels. Strong color edges
    // therefore stay at their original location; no frame history or RAM.
    function [23:0] safe_neighbor;
        input [23:0] center,neighbor;
        reg [7:0] dr,dg,db;
        begin
            dr=center[23:16]>=neighbor[23:16]?center[23:16]-neighbor[23:16]:neighbor[23:16]-center[23:16];
            dg=center[15:8]>=neighbor[15:8]?center[15:8]-neighbor[15:8]:neighbor[15:8]-center[15:8];
            db=center[7:0]>=neighbor[7:0]?center[7:0]-neighbor[7:0]:neighbor[7:0]-center[7:0];
            safe_neighbor=(dr<=64 && dg<=64 && db<=64)?neighbor:center;
        end
    endfunction
    wire [23:0] q[0:1];
    assign q[0]=rgb0;
    genvar k;
    for(k=1;k<2;k=k+1) begin:g_neighbor
        assign q[k]=safe_neighbor(rgb0,history[48 +:24]);
    end
    wire [23:0] clean;
    genvar ch;
    for(ch=0;ch<3;ch=ch+1) begin:g_sum
        wire [8:0] total={1'b0,q[0][ch*8 +:8]}+{1'b0,q[1][ch*8 +:8]}+9'd1;
        assign clean[ch*8 +:8]=total[8:1];
    end
    assign display_rgb0=(active && I_de && !first_pixel)?clean:rgb0;
    assign O_display_nr_active=active && mode==0;
end else begin:g_no_display_nr
    assign display_rgb0=rgb0;
    assign O_display_nr_active=1'b0;
end endgenerate

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rgb_out <= 24'd0;
    end else begin
        case(mode)
            2'b00: rgb_out <= display_rgb0;
            2'b01: rgb_out <= rgb1;
            2'b10: rgb_out <= rgb2;
            2'b11: rgb_out <= rgb3;
            default:rgb_out <= rgb0; //异常默认输出原图
        endcase
    end
end

endmodule
