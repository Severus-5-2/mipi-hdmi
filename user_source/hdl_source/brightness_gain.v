`timescale 1ns / 1ps

/*
 * brightness_gain.v —— FPGA 端数字亮度增益（不依赖 sensor I2C）
 *
 * 作用：对每个像素的 R/G/B 三通道分别乘以整数增益 GAIN，
 *       结果超过 255 则截断为 255（模拟真实过曝）。
 *
 * 背景：SC520CS 在 RAW 模式下，sensor 内部数字增益不生效，
 *       官方最大只有模拟 32x，弱光场景仍偏暗。
 *       因此在 FPGA 像素流上做数字增益，确定可控、立刻见效。
 *
 * 时序：1 级流水线（输入第 N 拍，输出第 N+1 拍）。
 *       本模块放在所有算法之前，故各模式延迟一致，mux 仍对齐。
 */
module brightness_gain #(
    parameter [7:0] GAIN = 8'd2   // 数字增益倍数，改成位宽明确的参数
) (
    input  wire        clk,       // 像素时钟 S_hdmi_pixel_clk
    input  wire        rst_n,     // 低电平复位
    input  wire [23:0] rgb_in,    // 输入像素 {R[23:16], G[15:8], B[7:0]}
    output reg  [23:0] rgb_out    // 增益后像素
);
    /* 三通道分别乘以增益，结果 16 位（最大 255*GAIN）。
     * 用乘法器（可综合到 DSP），GAIN 为常数时综合器会自动优化。 */
    wire [15:0] r_mul = rgb_in[23:16] * GAIN[15:0];
    wire [15:0] g_mul = rgb_in[15:8]  * GAIN[15:0];
    wire [15:0] b_mul = rgb_in[7:0]   * GAIN[15:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rgb_out <= 24'd0;
        end else begin
            /* 饱和截断：乘法结果高 8 位非 0 表示超过 255，输出 255；
             * 否则取低 8 位作为增益结果。 */
            rgb_out[23:16] <= (r_mul[15:8] != 8'd0) ? 8'hff : r_mul[7:0];
            rgb_out[15:8]  <= (g_mul[15:8] != 8'd0) ? 8'hff : g_mul[7:0];
            rgb_out[7:0]   <= (b_mul[15:8] != 8'd0) ? 8'hff : b_mul[7:0];
        end
    end
endmodule
