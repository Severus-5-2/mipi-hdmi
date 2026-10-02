//====================================================================
// 模块功能：24bit RGB888彩色图像 → 8bit单通道灰度
// 时钟域：S_hdmi_pixel_clk（HDMI像素时钟）
// 灰度公式：Gray = R*0.299 + G*0.587 + B*0.114
// 整数实现：Gray = (R*77 + G*150 + B*29) >> 8
//====================================================================
module rgb2gray(
input               clk,        //HDMI像素时钟 S_hdmi_pixel_clk
input               rst_n,      //低电平复位 S_hdmi_rst_n
input      [23:0]   rgb_in,     //输入24bit RGB {R,G,B}
output reg [7:0]    gray_out    //输出8bit灰度值
);

wire [7:0] r,g,b;
//拆分24bit RGB：高8bit=R，中间8bit=G，低8bit=B
assign r = rgb_in[23:16];
assign g = rgb_in[15:8];
assign b = rgb_in[7:0];

wire [15:0] gray_temp;
//整数乘法计算灰度，不使用浮点
assign gray_temp = r*8'd77 + g*8'd150 + b*8'd29;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        gray_out <= 8'd0;
    end else begin
        //右移8bit，取高8位作为最终灰度
        gray_out <= gray_temp[15:8];
    end
end

endmodule
