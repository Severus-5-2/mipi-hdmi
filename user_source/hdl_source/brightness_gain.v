module brightness_gain #(
    parameter [7:0] GAIN = 8'd2   // 数字增益倍数，改成位宽明确的参数
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [23:0] rgb_in,
    output reg  [23:0] rgb_out
);
    wire [15:0] r_mul = rgb_in[23:16] * GAIN;
    wire [15:0] g_mul = rgb_in[15:8]  * GAIN;
    wire [15:0] b_mul = rgb_in[7:0]   * GAIN;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rgb_out <= 24'd0;
        else begin
            rgb_out[23:16] <= (r_mul[15:8] != 8'd0) ? 8'hff : r_mul[7:0];
            rgb_out[15:8]  <= (g_mul[15:8] != 8'd0) ? 8'hff : g_mul[7:0];
            rgb_out[7:0]   <= (b_mul[15:8] != 8'd0) ? 8'hff : b_mul[7:0];
        end
    end
endmodule