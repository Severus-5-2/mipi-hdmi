//=============================================================================
//  target_bus.v  —  目标列表总线打包 / 解包（★ 两人共用，唯一一份）
//=============================================================================
//  项目  : MIPI-HDMI 实时视频图像处理系统（方案B §4：艺 → 晨）
//  归属  : user_source/hdl_source/common_base/
//  依据  : doc/接口冻结表.md v1.0 §2
//
//  为什么要这个文件：
//      384 bit 的总线如果两人各写一套位域拼接，一定会有「谁在高位谁在低位」
//      的分歧 —— 综合能过、上板时 cx/cy 全错位且极难排查。
//      ⇒ 打包与解包只走这一个模块。
//
//-----------------------------------------------------------------------------
//  单槽 96 bit 位域（`common_defs.vh` 已定义偏移）
//-----------------------------------------------------------------------------
//      [95:86] 保留 (10 bit，清零)
//      [85:66] area  (20 bit)
//      [65:56] ymax  (10 bit)
//      [55:46] ymin  (10 bit)
//      [45:35] xmax  (11 bit)
//      [34:24] xmin  (11 bit)
//      [23:14] cy    (10 bit)
//      [13:3]  cx    (11 bit)
//      [2:1]   cls   ( 2 bit)
//      [0]     valid ( 1 bit)
//
//  总线整体：4 槽，槽 0 在最低 96 bit（按 area 降序，槽 0 = 最大目标）
//
//  总线宽度参数化：SLOT_N 可调（默认 4），便于降级为单目标检测时复用。
//=============================================================================

`timescale 1ns / 1ps
`include "common_defs.vh"

//-----------------------------------------------------------------------------
//  打包器：把 4 个目标字段拼成 384 bit 总线
//-----------------------------------------------------------------------------
module target_pack #(
    parameter integer SLOT_N = `TARGET_MAX
)(
    // ---- 槽 0 ----
    input  wire                    i_v0,      input wire [1:0]  i_c0,
    input  wire [`PIX_X_W-1:0]     i_cx0,     input wire [`PIX_Y_W-1:0] i_cy0,
    input  wire [`PIX_X_W-1:0]     i_xmin0,   input wire [`PIX_X_W-1:0] i_xmax0,
    input  wire [`PIX_Y_W-1:0]     i_ymin0,   input wire [`PIX_Y_W-1:0] i_ymax0,
    input  wire [`TGT_AREA_W-1:0]  i_area0,
    // ---- 槽 1 ----
    input  wire                    i_v1,      input wire [1:0]  i_c1,
    input  wire [`PIX_X_W-1:0]     i_cx1,     input wire [`PIX_Y_W-1:0] i_cy1,
    input  wire [`PIX_X_W-1:0]     i_xmin1,   input wire [`PIX_X_W-1:0] i_xmax1,
    input  wire [`PIX_Y_W-1:0]     i_ymin1,   input wire [`PIX_Y_W-1:0] i_ymax1,
    input  wire [`TGT_AREA_W-1:0]  i_area1,
    // ---- 槽 2 ----
    input  wire                    i_v2,      input wire [1:0]  i_c2,
    input  wire [`PIX_X_W-1:0]     i_cx2,     input wire [`PIX_Y_W-1:0] i_cy2,
    input  wire [`PIX_X_W-1:0]     i_xmin2,   input wire [`PIX_X_W-1:0] i_xmax2,
    input  wire [`PIX_Y_W-1:0]     i_ymin2,   input wire [`PIX_Y_W-1:0] i_ymax2,
    input  wire [`TGT_AREA_W-1:0]  i_area2,
    // ---- 槽 3 ----
    input  wire                    i_v3,      input wire [1:0]  i_c3,
    input  wire [`PIX_X_W-1:0]     i_cx3,     input wire [`PIX_Y_W-1:0] i_cy3,
    input  wire [`PIX_X_W-1:0]     i_xmin3,   input wire [`PIX_X_W-1:0] i_xmax3,
    input  wire [`PIX_Y_W-1:0]     i_ymin3,   input wire [`PIX_Y_W-1:0] i_ymax3,
    input  wire [`TGT_AREA_W-1:0]  i_area3,

    output wire [`TGT_BUS_W-1:0]   O_bus
);

    function automatic [`TGT_SLOT_W-1:0] pack_slot;
        input v; input [1:0] c;
        input [`PIX_X_W-1:0] cx, xmin, xmax;
        input [`PIX_Y_W-1:0] cy, ymin, ymax;
        input [`TGT_AREA_W-1:0] area;
        begin
            pack_slot                     = {`TGT_SLOT_W{1'b0}};
            pack_slot[`TGT_O_VALID]       = v;
            pack_slot[`TGT_O_CLS   +: 2]  = (v) ? c     : 2'd0;   // 空槽字段清零
            pack_slot[`TGT_O_CX    +: 11] = (v) ? cx    : 11'd0;
            pack_slot[`TGT_O_CY    +: 10] = (v) ? cy    : 10'd0;
            pack_slot[`TGT_O_XMIN  +: 11] = (v) ? xmin  : 11'd0;
            pack_slot[`TGT_O_XMAX  +: 11] = (v) ? xmax  : 11'd0;
            pack_slot[`TGT_O_YMIN  +: 10] = (v) ? ymin  : 10'd0;
            pack_slot[`TGT_O_YMAX  +: 10] = (v) ? ymax  : 10'd0;
            pack_slot[`TGT_O_AREA  +: 20] = (v) ? area  : 20'd0;
        end
    endfunction

    assign O_bus[ 95:  0] = pack_slot(i_v0, i_c0, i_cx0, i_xmin0, i_xmax0, i_cy0, i_ymin0, i_ymax0, i_area0);
    assign O_bus[191: 96] = pack_slot(i_v1, i_c1, i_cx1, i_xmin1, i_xmax1, i_cy1, i_ymin1, i_ymax1, i_area1);
    assign O_bus[287:192] = pack_slot(i_v2, i_c2, i_cx2, i_xmin2, i_xmax2, i_cy2, i_ymin2, i_ymax2, i_area2);
    assign O_bus[383:288] = pack_slot(i_v3, i_c3, i_cx3, i_xmin3, i_xmax3, i_cy3, i_ymin3, i_ymax3, i_area3);

endmodule


//-----------------------------------------------------------------------------
//  解包器：从 384 bit 总线取出第 N 槽
//-----------------------------------------------------------------------------
module target_unpack #(
    parameter integer SLOT_IDX = 0
)(
    input  wire [`TGT_BUS_W-1:0]   I_bus,
    output wire                    O_valid,
    output wire [1:0]              O_cls,
    output wire [`PIX_X_W-1:0]     O_cx,
    output wire [`PIX_Y_W-1:0]     O_cy,
    output wire [`PIX_X_W-1:0]     O_xmin,
    output wire [`PIX_X_W-1:0]     O_xmax,
    output wire [`PIX_Y_W-1:0]     O_ymin,
    output wire [`PIX_Y_W-1:0]     O_ymax,
    output wire [`TGT_AREA_W-1:0]  O_area
);

    wire [`TGT_SLOT_W-1:0] slot = I_bus[SLOT_IDX*`TGT_SLOT_W +: `TGT_SLOT_W];

    assign O_valid = slot[`TGT_O_VALID];
    assign O_cls   = slot[`TGT_O_CLS   +: 2];
    assign O_cx    = (O_valid) ? slot[`TGT_O_CX   +: 11] : 11'd0;
    assign O_cy    = (O_valid) ? slot[`TGT_O_CY   +: 10] : 10'd0;
    assign O_xmin  = (O_valid) ? slot[`TGT_O_XMIN +: 11] : 11'd0;
    assign O_xmax  = (O_valid) ? slot[`TGT_O_XMAX +: 11] : 11'd0;
    assign O_ymin  = (O_valid) ? slot[`TGT_O_YMIN +: 10] : 10'd0;
    assign O_ymax  = (O_valid) ? slot[`TGT_O_YMAX +: 10] : 10'd0;
    assign O_area  = (O_valid) ? slot[`TGT_O_AREA +: 20] : 20'd0;

endmodule
