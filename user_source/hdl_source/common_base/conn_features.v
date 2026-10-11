//=============================================================================
//  conn_features.v  —  消隐期汇总 → 按 area 降序取前 4 → target_bus 打包
//=============================================================================
//  项目  : MIPI-HDMI 实时视频图像处理系统（方案B §2.3 / §3.1 / §4）
//  归属  : user_source/hdl_source/common_base/   （艺 → 晨）
//  依据  : doc/接口冻结表.md v1.4 §2
//
//  数据链路：
//      conn_label.O_cmp_*  ──(消隐期脉冲流，每连通域一拍)──►  conn_features
//                          ──►  O_target_bus(384bit) + O_target_valid_cnt(3bit)
//
//  ── 做什么 ────────────────────────────────────────────────────────────────
//   ① 把最多 32 个连通域按面积做「前 4 名擂台」（插入排序，一拍一个，纯寄存器）
//   ② 排序键：area 降序 → 同面积 xmin 升序 → 再同则紧凑 label 升序（确定性兜底）
//   ③ 帧界（I_frame_done）把擂台锁存进输出寄存器，并清空擂台
//   ④ 用 common_base/target_bus.v 的 target_pack 打包 384 bit（空槽字段清零）
//
//  ── 为什么用「擂台」而不是先存 32 个再排序 ────────────────────────────────
//      存 32 个槽需要 32×91 ≈ 2900 FF；擂台只要 4×91 ≈ 364 FF，
//      且天然一拍一个、无需额外的排序状态机。
//
//  ── 时序与「无目标」───────────────────────────────────────────────────────
//      conn_label 的汇总流在**场消隐期**发出；本模块在下一个 I_frame_done
//      （= 下一帧首）锁存，此后整帧稳定，供消费方随时采样（冻结表 §2.2）。
//      ⇒ 「本帧无目标」时擂台全空，锁存后 valid_cnt=0、所有字段清零，
//        **不会留上一帧残留**（冻结表 §2.4）。
//
//  ── 面积门限 ──────────────────────────────────────────────────────────────
//      低于最小面积门限的连通域**在 conn_label 里就不进汇总流**（其 MIN_AREA
//      参数负责），本模块不重复门限，以免两处参数不一致。
//
//  ── 已知边界 ──────────────────────────────────────────────────────────────
//      · 仅保证**同一帧内**的槽位次序；跨帧槽号不保证指向同一目标（冻结表 §2.3）。
//      · 目标数 >4 时只保留面积最大的 4 个，其余丢弃（方案B §3.1 深度 4）。
//=============================================================================

`timescale 1ns / 1ps
`include "common_defs.vh"

module conn_features #(
    parameter integer SLOT_N = `TARGET_MAX        // 4（降级为单目标检测时可改 1）
)(
    input  wire        clk,
    input  wire        rst_n,

    //---- 帧界：与 conn_label / pix_coord_gen 同源的帧起始脉冲 ----
    input  wire        I_frame_done,

    //---- 输入：conn_label 的帧末汇总脉冲流（每连通域一拍）----
    input  wire        I_cmp_valid,
    input  wire [5:0]  I_cmp_label,
    input  wire [1:0]  I_cmp_cls,
    input  wire [19:0] I_cmp_area,
    input  wire [10:0] I_cmp_xmin,
    input  wire [10:0] I_cmp_xmax,
    input  wire [9:0]  I_cmp_ymin,
    input  wire [9:0]  I_cmp_ymax,
    input  wire [10:0] I_cmp_cx,
    input  wire [9:0]  I_cmp_cy,

    //---- 输出：目标列表总线 ----
    output wire [`TGT_BUS_W-1:0] O_target_bus,      // 4 槽 × 96 bit
    output wire [2:0]            O_target_valid_cnt,// 本帧有效目标数 0~4
    output reg                   O_update           // 锁存脉冲（1 拍，可选用于触发下游）
);

    //=========================================================================
    //  0. 擂台槽内部打包（91 bit）
    //     [90:85] 紧凑 label（仅用于排序兜底，不进总线）
    //     [84:83] cls    [82:72] cx    [71:62] cy
    //     [61:51] xmin   [50:40] xmax  [39:30] ymin  [29:20] ymax
    //     [19:0]  area   （area == 0 即「空槽」）
    //=========================================================================
    localparam integer SW      = 91;
    localparam integer SF_AREA = 0;
    localparam integer SF_YMAX = 20;
    localparam integer SF_YMIN = 30;
    localparam integer SF_XMAX = 40;
    localparam integer SF_XMIN = 51;
    localparam integer SF_CY   = 62;
    localparam integer SF_CX   = 72;
    localparam integer SF_CLS  = 83;
    localparam integer SF_LBL  = 85;

    function [SW-1:0] mk_slot;
        input [5:0]  lb;
        input [1:0]  cl;
        input [19:0] ar;
        input [10:0] xa, xb, cxx;
        input [9:0]  ya, yb, cyy;
        begin
            mk_slot                = {SW{1'b0}};
            mk_slot[SF_AREA +: 20] = ar;
            mk_slot[SF_YMAX +: 10] = yb;
            mk_slot[SF_YMIN +: 10] = ya;
            mk_slot[SF_XMAX +: 11] = xb;
            mk_slot[SF_XMIN +: 11] = xa;
            mk_slot[SF_CY   +: 10] = cyy;
            mk_slot[SF_CX   +: 11] = cxx;
            mk_slot[SF_CLS  +: 2]  = cl;
            mk_slot[SF_LBL  +: 6]  = lb;
        end
    endfunction

    wire [SW-1:0] new_slot = mk_slot(I_cmp_label, I_cmp_cls, I_cmp_area,
                                     I_cmp_xmin, I_cmp_xmax, I_cmp_cx,
                                     I_cmp_ymin, I_cmp_ymax, I_cmp_cy);

    // new 是否应排在 cur **之前**（area 降序 → xmin 升序 → label 升序）
    function beat;
        input [SW-1:0] nw;
        input [SW-1:0] cu;
        reg   [19:0]   na, ca;
        reg   [10:0]   nx, cxr;
        reg   [5:0]    nl, clr;
        begin
            na  = nw[SF_AREA +: 20];
            ca  = cu[SF_AREA +: 20];
            nx  = nw[SF_XMIN +: 11];
            cxr = cu[SF_XMIN +: 11];
            nl  = nw[SF_LBL  +: 6];
            clr = cu[SF_LBL  +: 6];
            if      (na != ca) beat = (na > ca);        // 面积大者在前
            else if (nx != cxr) beat = (nx < cxr);      // 同面积：xmin 小者在前
            else               beat = (nl < clr);       // 再同：label 小者在前（确定序）
        end
        // 注：空槽 area==0，任何有效新元素（area>=1）都必然 >0 ⇒ 自然插到前面
    endfunction

    //=========================================================================
    //  1. 前 4 擂台（插入排序，一拍一个）
    //=========================================================================
    reg [SW-1:0] s0, s1, s2, s3;        // 工作擂台（帧内滚动更新）
    reg [SW-1:0] l0, l1, l2, l3;        // 锁存输出（整帧稳定）

    wire b0 = beat(new_slot, s0);
    wire b1 = beat(new_slot, s1);
    wire b2 = beat(new_slot, s2);
    wire b3 = beat(new_slot, s3);

    // 只插入到「第一个被击败」的槽位；被插入位置之后的槽整体后移一格，末位挤出
    wire i0 =  b0;
    wire i1 = ~b0 &  b1;
    wire i2 = ~b0 & ~b1 &  b2;
    wire i3 = ~b0 & ~b1 & ~b2 & b3;

    // 后移规则（★ 别写错：槽 j>t 应拿 s[j-1]，不是 s[0]）：
    //   t=0 → s0→s1, s1→s2, s2→s3
    //   t=1 →          s1→s2, s2→s3
    //   t=2 →                 s2→s3
    //   t=3 → 无后移
    wire        sh1 = (~b0 & ~b1);          // 插入点在槽 2/3 之前? 即 i0|i1
    wire        sh2 = (~b0 & ~b1 & ~b2);    // i0|i1|i2
    wire [SW-1:0] n0 = i0 ? new_slot : s0;
    wire [SW-1:0] n1 = i0 ? s0 : (i1 ? new_slot : s1);
    wire [SW-1:0] n2 = (i0 | i1) ? s1 : (i2 ? new_slot : s2);
    wire [SW-1:0] n3 = (~sh2)   ? s2 : (i3 ? new_slot : s3);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0 <= {SW{1'b0}}; s1 <= {SW{1'b0}};
            s2 <= {SW{1'b0}}; s3 <= {SW{1'b0}};
            l0 <= {SW{1'b0}}; l1 <= {SW{1'b0}};
            l2 <= {SW{1'b0}}; l3 <= {SW{1'b0}};
            O_update <= 1'b0;
        end else begin
            O_update <= 1'b0;
            if (I_frame_done) begin
                // 帧界：锁存本帧擂台 → 输出；随后清空擂台，准备下一帧
                l0 <= s0; l1 <= s1; l2 <= s2; l3 <= s3;
                s0 <= {SW{1'b0}}; s1 <= {SW{1'b0}};
                s2 <= {SW{1'b0}}; s3 <= {SW{1'b0}};
                O_update <= 1'b1;
            end else if (I_cmp_valid) begin
                s0 <= n0; s1 <= n1; s2 <= n2; s3 <= n3;
            end
        end
    end

    //=========================================================================
    //  2. 打包 384 bit（空槽 valid=0 且字段清零 —— 由 target_pack 保证）
    //=========================================================================
    wire v0 = (l0[SF_AREA +: 20] != 20'd0);
    wire v1 = (l1[SF_AREA +: 20] != 20'd0);
    wire v2 = (l2[SF_AREA +: 20] != 20'd0);
    wire v3 = (l3[SF_AREA +: 20] != 20'd0);

    assign O_target_valid_cnt = {2'b0, v0} + {2'b0, v1} + {2'b0, v2} + {2'b0, v3};

    target_pack #(.SLOT_N(SLOT_N)) u_pack (
        .i_v0   (v0),               .i_c0   (l0[SF_CLS  +: 2]),
        .i_cx0  (l0[SF_CX   +: 11]),.i_cy0  (l0[SF_CY   +: 10]),
        .i_xmin0(l0[SF_XMIN +: 11]),.i_xmax0(l0[SF_XMAX +: 11]),
        .i_ymin0(l0[SF_YMIN +: 10]),.i_ymax0(l0[SF_YMAX +: 10]),
        .i_area0(l0[SF_AREA +: 20]),

        .i_v1   (v1),               .i_c1   (l1[SF_CLS  +: 2]),
        .i_cx1  (l1[SF_CX   +: 11]),.i_cy1  (l1[SF_CY   +: 10]),
        .i_xmin1(l1[SF_XMIN +: 11]),.i_xmax1(l1[SF_XMAX +: 11]),
        .i_ymin1(l1[SF_YMIN +: 10]),.i_ymax1(l1[SF_YMAX +: 10]),
        .i_area1(l1[SF_AREA +: 20]),

        .i_v2   (v2),               .i_c2   (l2[SF_CLS  +: 2]),
        .i_cx2  (l2[SF_CX   +: 11]),.i_cy2  (l2[SF_CY   +: 10]),
        .i_xmin2(l2[SF_XMIN +: 11]),.i_xmax2(l2[SF_XMAX +: 11]),
        .i_ymin2(l2[SF_YMIN +: 10]),.i_ymax2(l2[SF_YMAX +: 10]),
        .i_area2(l2[SF_AREA +: 20]),

        .i_v3   (v3),               .i_c3   (l3[SF_CLS  +: 2]),
        .i_cx3  (l3[SF_CX   +: 11]),.i_cy3  (l3[SF_CY   +: 10]),
        .i_xmin3(l3[SF_XMIN +: 11]),.i_xmax3(l3[SF_XMAX +: 11]),
        .i_ymin3(l3[SF_YMIN +: 10]),.i_ymax3(l3[SF_YMAX +: 10]),
        .i_area3(l3[SF_AREA +: 20]),

        .O_bus  (O_target_bus)
    );

endmodule
