//=============================================================================
//  pix_counter.v  —  公共像素坐标计数器（★ 两人共用，唯一一份）
//=============================================================================
//  项目  : MIPI-HDMI 实时视频图像处理系统（方案B §1.3）
//  归属  : user_source/hdl_source/common_base/
//  依据  : doc/接口冻结表.md v1.0 §1.1
//
//  为什么必须公共（方案B §1.3 原话）：
//      「三人都用同一套坐标，**不要各写一份计数器**，否则联调必然对不齐。」
//      本模块由队长在 top 实例化**一次**，生成 S_pix_x / S_pix_y / S_frame_done，
//      所有下游模块统一使用这组信号，不得自建。
//
//-----------------------------------------------------------------------------
//  计数约定（与 hdmi_mixer.v 完全对齐）
//-----------------------------------------------------------------------------
//    · user 有效（帧起始）      → x = 0, y = 0
//    · de 有效且 last（行末）   → x = 0, y = y + 1
//    · de 有效且 !last          → x = x + 1
//    · x 回绕在 IMG_WIDTH-1，y 回绕在 IMG_HEIGHT-1（双保险）
//    · de 无效时**保持**（不计数），保证消隐期坐标不变
//
//  输出语义：O_pix_x / O_pix_y 与**输入侧** I_de 同拍（本模块不含额外流水，
//  下游如需要与自己的流水对齐，自行按拍数延时这两个信号）。
//=============================================================================

`timescale 1ns / 1ps
`include "common_defs.vh"

module pix_counter #(
    parameter integer IMG_WIDTH  = `IMG_WIDTH,
    parameter integer IMG_HEIGHT = `IMG_HEIGHT
)(
    input  wire                  clk,
    input  wire                  rst_n,        // 低电平复位
    input  wire                  I_de,
    input  wire                  I_user,       // 帧起始（同 hdmi_mixer.I_video_user）
    input  wire                  I_last,       // 行末像素（同 hdmi_mixer.I_video_last）
    input  wire                  I_vsync,

    output reg  [`PIX_X_W-1:0]   O_pix_x,      // 0 ~ IMG_WIDTH-1
    output reg  [`PIX_Y_W-1:0]   O_pix_y,      // 0 ~ IMG_HEIGHT-1
    output reg                   O_frame_done  // 与 I_user 同拍（帧界脉冲）
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            O_pix_x      <= {`PIX_X_W{1'b0}};
            O_pix_y      <= {`PIX_Y_W{1'b0}};
            O_frame_done <= 1'b0;
        end else begin
            O_frame_done <= I_user;

            if (I_user) begin
                // 帧起始：坐标归零
                O_pix_x <= {`PIX_X_W{1'b0}};
                O_pix_y <= {`PIX_Y_W{1'b0}};
            end else if (I_de) begin
                if (I_last) begin
                    O_pix_x <= {`PIX_X_W{1'b0}};
                    O_pix_y <= (O_pix_y == (IMG_HEIGHT - 1))
                             ? {`PIX_Y_W{1'b0}} : (O_pix_y + 1'b1);
                end else begin
                    O_pix_x <= (O_pix_x == (IMG_WIDTH - 1))
                             ? {`PIX_X_W{1'b0}} : (O_pix_x + 1'b1);
                end
            end
            // de 无效：保持（消隐期坐标不动）
        end
    end

endmodule
