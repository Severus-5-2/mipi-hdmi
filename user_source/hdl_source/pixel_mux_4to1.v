//====================================================================
// 模块功能：四路24bit RGB像素数据流选择器
// mode[1:0]模式定义：
//   2'b00 → rgb0：原始摄像头图像
//   2'b01 → rgb1：灰度图
//   2'b10 → rgb2：二值化图像
//   2'b11 → rgb3：Sobel边缘检测图像
// 时钟域：S_hdmi_pixel_clk
//====================================================================
module pixel_mux_4to1(
input               clk,        //HDMI像素时钟
input               rst_n,      //低电平复位
input      [1:0]    mode,       //模式选择信号
input      [23:0]   rgb0,       //mode00 原图
input      [23:0]   rgb1,       //mode01 灰度图
input      [23:0]   rgb2,       //mode10 二值图
input      [23:0]   rgb3,       //mode11 Sobel边缘图
output reg [23:0]   rgb_out     //选中之后输出RGB像素
);

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rgb_out <= 24'd0;
    end else begin
        case(mode)
            2'b00: rgb_out <= rgb0;
            2'b01: rgb_out <= rgb1;
            2'b10: rgb_out <= rgb2;
            2'b11: rgb_out <= rgb3;
            default:rgb_out <= rgb0; //异常默认输出原图
        endcase
    end
end

endmodule
