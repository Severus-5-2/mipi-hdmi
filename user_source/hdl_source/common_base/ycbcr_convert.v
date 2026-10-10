//=============================================================================
//  ycbcr_convert.v  —  RGB888 → YCbCr（★ 两人共用，唯一一份）
//=============================================================================
//  项目  : MIPI-HDMI 实时视频图像处理系统（方案B §1.1）
//  归属  : user_source/hdl_source/common_base/
//  依据  : doc/接口冻结表.md v1.0 §2；《开发方案B》§1.1、§1.2
//
//  为什么必须公共：
//      方案B §1.1 明确「转换系数直接复用 denoise_fix/chroma_denoise.v 里已有的」。
//      两人若各写一份 RGB→YCbCr，系数/取整/级数必然有差，掩膜边界会差 1~2 像素。
//      ⇒ 全工程只保留这一份实现。
//
//-----------------------------------------------------------------------------
//  定标（与 chroma_denoise.v 位级一致，一个字都不能改）
//-----------------------------------------------------------------------------
//      Y  =  77R + 150G +  29B            → (x + 128) >>> 8
//      Cb = -43R -  85G + 128B  + 128     → ((x + 128) >>> 8) + 128
//      Cr = 128R - 107G -  21B  + 128     → ((x + 128) >>> 8) + 128
//  输出 Y/Cb/Cr 均为 8 bit，取值 0 ~ 255（**必须先限幅再输出**）。
//
//-----------------------------------------------------------------------------
//  流水线（3 拍，单级 ≤4 级逻辑；方案B §1.2 的 74.25MHz 13.33ns 预算）
//-----------------------------------------------------------------------------
//      拍 1 (P1)：9 个常量乘积寄存            （各乘积相互独立，≤1 级乘法）
//      拍 2 (P2)：3 项求和 + (+128) + (>>>8) + 偏置 （≤4 级）
//      拍 3 (P3)：限幅到 0~255                （≤3 级）
//
//  总延迟 = 3 拍。模块内部把 I_de / I_vsync / I_hsync / 传入坐标同步延时 3 拍，
//  保证 (O_y,O_cb,O_cr,O_de,O_pix_x,O_pix_y) 描述**同一个像素**。
//
//  ⚠ 输入侧对齐：本模块假定 I_rgb 与 I_de 同拍。若上游 RGB 相对 de 有滞后，
//     把滞后拍数填进 PIPE_DELAY（容差 0~15），用法同 ae_meter.PIPE_DELAY。
//
//-----------------------------------------------------------------------------
//  位宽与溢出核查
//-----------------------------------------------------------------------------
//   · 单个乘积最大 |128 × 255| = 32640 < 18bit 有符号上限 131071   ✔
//   · 3 项和最大 |±32640|          < 131071                        ✔
//   · (和+128)>>>8 ∈ [-128,128]，+128 ∈ [0,256]，用 10bit 有符号   ✔
//   · 限幅到 0~255 后取低 8 位
//=============================================================================

`timescale 1ns / 1ps
`include "common_defs.vh"

module ycbcr_convert #(
    parameter integer PIPE_DELAY = 0,      // I_rgb 相对 I_de 的滞后拍数（0~15）
    parameter [7:0]   Y_MIN      = 8'd0    // 亮度门限：Y < Y_MIN 时 O_gate = 0
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // ---- 输入（I_de 与 I_rgb 同拍，或在 PIPE_DELAY 内对齐）----
    input  wire                  I_de,
    input  wire                  I_vsync,
    input  wire                  I_hsync,
    input  wire [23:0]           I_rgb,        // {R[7:0], G[7:0], B[7:0]}

    // ---- 像素坐标（来自公共 pix_counter，可选；不需要时接 0）----
    input  wire [`PIX_X_W-1:0]   I_pix_x,
    input  wire [`PIX_Y_W-1:0]   I_pix_y,

    // ---- 输出（全部同拍）----
    output reg  [7:0]            O_y,
    output reg  [7:0]            O_cb,
    output reg  [7:0]            O_cr,
    output reg                   O_gate,       // Y >= Y_MIN（掩膜门控用）
    output reg                   O_de,
    output reg                   O_vsync,
    output reg                   O_hsync,
    output reg  [`PIX_X_W-1:0]   O_pix_x,
    output reg  [`PIX_Y_W-1:0]   O_pix_y
);

    //==================================================================
    // 0. 输入侧对齐（PIPE_DELAY 拍，0~15）
    //==================================================================
    reg [15:0] dly_de, dly_vsync, dly_hsync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dly_de    <= 16'd0;
            dly_vsync <= 16'd0;
            dly_hsync <= 16'd0;
        end else begin
            dly_de    <= {dly_de   [14:0], I_de};
            dly_vsync <= {dly_vsync[14:0], I_vsync};
            dly_hsync <= {dly_hsync[14:0], I_hsync};
        end
    end

    localparam integer PDSEL = (PIPE_DELAY <= 0) ? 0 : (PIPE_DELAY - 1);

    wire w_de    = (PIPE_DELAY <= 0) ? I_de    : dly_de   [PDSEL];
    wire w_vsync = (PIPE_DELAY <= 0) ? I_vsync : dly_vsync[PDSEL];
    wire w_hsync = (PIPE_DELAY <= 0) ? I_hsync : dly_hsync[PDSEL];

    wire [7:0] w_r = I_rgb[23:16];
    wire [7:0] w_g = I_rgb[15:8];
    wire [7:0] w_b = I_rgb[7:0];

    //==================================================================
    // 1. P1：常量乘积寄存
    //==================================================================
    // 无符号通道扩展为 18bit 有符号（符号位补 0），统一按有符号运算
    wire signed [17:0] r_ext = $signed({10'd0, w_r});
    wire signed [17:0] g_ext = $signed({10'd0, w_g});
    wire signed [17:0] b_ext = $signed({10'd0, w_b});

    reg signed [17:0] p_y_r,  p_y_g,  p_y_b;
    reg signed [17:0] p_cb_r, p_cb_g, p_cb_b;
    reg signed [17:0] p_cr_r, p_cr_g, p_cr_b;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_y_r  <= 18'sd0; p_y_g  <= 18'sd0; p_y_b  <= 18'sd0;
            p_cb_r <= 18'sd0; p_cb_g <= 18'sd0; p_cb_b <= 18'sd0;
            p_cr_r <= 18'sd0; p_cr_g <= 18'sd0; p_cr_b <= 18'sd0;
        end else begin
            p_y_r  <= `YCBCR_KR    * r_ext;
            p_y_g  <= `YCBCR_KG    * g_ext;
            p_y_b  <= `YCBCR_KB    * b_ext;
            p_cb_r <= `YCBCR_CB_KR * r_ext;
            p_cb_g <= `YCBCR_CB_KG * g_ext;
            p_cb_b <= `YCBCR_CB_KB * b_ext;
            p_cr_r <= `YCBCR_CR_KR * r_ext;
            p_cr_g <= `YCBCR_CR_KG * g_ext;
            p_cr_b <= `YCBCR_CR_KB * b_ext;
        end
    end

    //==================================================================
    // 2. P2：求和 + 四舍五入 + 算术右移 + 零偏置
    //==================================================================
    wire signed [17:0] y_sum  = p_y_r  + p_y_g  + p_y_b;
    wire signed [17:0] cb_sum = p_cb_r + p_cb_g + p_cb_b;
    wire signed [17:0] cr_sum = p_cr_r + p_cr_g + p_cr_b;

    reg signed [9:0] q_y, q_cb, q_cr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_y  <= 10'sd0;
            q_cb <= `YCBCR_BIAS;
            q_cr <= `YCBCR_BIAS;
        end else begin
            q_y  <= (y_sum  + `YCBCR_ROUND) >>> `YCBCR_SHIFT;
            q_cb <= ((cb_sum + `YCBCR_ROUND) >>> `YCBCR_SHIFT) + `YCBCR_BIAS;
            q_cr <= ((cr_sum + `YCBCR_ROUND) >>> `YCBCR_SHIFT) + `YCBCR_BIAS;
        end
    end

    //==================================================================
    // 3. P3：限幅到 0~255
    //==================================================================
    //  ⚠ 必须限幅（D1 位级仿真抓捕的坑）：
    //     q_cb/q_cr 理论值域可达 256（+128 偏置取整所致）。若下游拿未限幅值
    //     与最大只能填 255 的阈值比较，`256 <= 255` 不成立 ⇒ 最饱和的纯红/纯蓝
    //     被判为不命中。故**在此处统一限幅**，下游一律用 0~255 的值。
    wire [7:0] w_y_out  = (q_y  > 10'sd255) ? 8'd255 : (q_y  < 10'sd0) ? 8'd0 : q_y [7:0];
    wire [7:0] w_cb_out = (q_cb > 10'sd255) ? 8'd255 : (q_cb < 10'sd0) ? 8'd0 : q_cb[7:0];
    wire [7:0] w_cr_out = (q_cr > 10'sd255) ? 8'd255 : (q_cr < 10'sd0) ? 8'd0 : q_cr[7:0];

    //==================================================================
    // 4. 输出寄存（时序标志 / 坐标 统一 3 拍延时，与数据对齐）
    //==================================================================
    //  ⚠ 用 **2bit 移位 + 取 [1]** 正好 3 级；不要用 3bit 取 [2]（那会变 4 级，
    //     导致首像素丢失、整帧错位一拍 —— D1 位级仿真抓捕的第一个坑）。
    reg [1:0] d3_de, d3_vsync, d3_hsync;
    reg [`PIX_X_W-1:0] d3_x_0, d3_x_1;
    reg [`PIX_Y_W-1:0] d3_y_0, d3_y_1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d3_de    <= 2'd0;
            d3_vsync <= 2'd0;
            d3_hsync <= 2'd0;
            d3_x_0   <= {`PIX_X_W{1'b0}}; d3_x_1 <= {`PIX_X_W{1'b0}};
            d3_y_0   <= {`PIX_Y_W{1'b0}}; d3_y_1 <= {`PIX_Y_W{1'b0}};

            O_y <= 8'd0; O_cb <= 8'd128; O_cr <= 8'd128;
            O_gate <= 1'b0;
            O_de <= 1'b0; O_vsync <= 1'b0; O_hsync <= 1'b0;
            O_pix_x <= {`PIX_X_W{1'b0}};
            O_pix_y <= {`PIX_Y_W{1'b0}};
        end else begin
            d3_de    <= {d3_de   [0], w_de};
            d3_vsync <= {d3_vsync[0], w_vsync};
            d3_hsync <= {d3_hsync[0], w_hsync};

            // 坐标：外部传入的坐标在拍 0，需再延 2 拍与 P3 对齐
            d3_x_0 <= I_pix_x;  d3_x_1 <= d3_x_0;
            d3_y_0 <= I_pix_y;  d3_y_1 <= d3_y_0;

            O_y      <= w_y_out;
            O_cb     <= w_cb_out;
            O_cr     <= w_cr_out;
            O_gate   <= (w_y_out >= Y_MIN);

            O_de     <= d3_de   [1];
            O_vsync  <= d3_vsync[1];
            O_hsync  <= d3_hsync[1];
            O_pix_x  <= d3_x_1;
            O_pix_y  <= d3_y_1;
        end
    end

endmodule
