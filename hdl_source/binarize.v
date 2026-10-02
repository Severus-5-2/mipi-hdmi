//====================================================================
// 模块功能：8bit灰度输入，输出24bit黑白RGB图像
// 大于阈值输出纯白色，小于等于阈值输出纯黑色
// 时钟域：S_hdmi_pixel_clk
// parameter THRESHOLD：二值化阈值，范围0~255，可修改
//====================================================================
module binarize #(
    parameter THRESHOLD = 160   //二值化判定阈值，可修改
)(
input               clk,        //HDMI像素时钟
input               rst_n,      //低电平复位
input      [7:0]    gray_in,    //输入8bit灰度
output reg [23:0]   rgb_out     //输出24bit黑白RGB
);

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rgb_out <= 24'd0;
    end else begin
        if(gray_in > THRESHOLD) begin
            rgb_out <= {8'hff,8'hff,8'hff}; // 亮度高于阈值：输出白色
        end else begin
            rgb_out <= {8'h00,8'h00,8'h00}; // 亮度低于阈值：输出黑色
        end
    end
end

endmodule
