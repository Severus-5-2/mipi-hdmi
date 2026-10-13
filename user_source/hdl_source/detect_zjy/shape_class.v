//=============================================================================
//  shape_class.v —— 几何形状分类（开发方案B §2.3，D5「分类」）
//=============================================================================
//  项目：MIPI-HDMI 实时视频图像处理系统 · 方案B
//  负责人：张金艺   归属：user_source/hdl_source/detect_zjy/
//  依据：`doc/开发方案B-目标检测跟踪与运动检测.md` §2.3（分类特征）
//        `doc/接口冻结表.md` §2（目标列表总线）
//
//-----------------------------------------------------------------------------
//  一、功能
//-----------------------------------------------------------------------------
//  输入：target_bus（4 槽 × 96 bit，来自 conn_features）
//  输出：每槽的**几何形状**类别（2 bit）
//
//  形状编码（与颜色 `CLS_*` 独立，两者组合才是完整目标类型）：
//      2'd0 = 方（矩形/方块）
//      2'd1 = 圆（球）
//      2'd2 = 三角
//      2'd3 = 未知 / 槽无效
//
//-----------------------------------------------------------------------------
//  二、判据：填充度 f = area / (w × h)
//-----------------------------------------------------------------------------
//    w = xmax - xmin + 1 ，h = ymax - ymin + 1
//
//      方  ： f >= 88%            （实心矩形理论 ≈100%，留边缘/抗锯齿损耗）
//      圆  ： 72% <= f < 88%      （理论 π/4 ≈ 78.5%）
//      三角： 38% <= f < 72%      （理论 50%）
//      未知： f < 38% 或槽位无效
//
//  ★ 用「交叉相乘」代替除法，且用「移位加法」代替乘法器：
//      f >= T%  ⇔  area×100 >= bbox×T            （bbox = w × h）
//
//      area×100 = (area<<6) + (area<<5) + (area<<2)      64+32+4 = 100
//      bbox×88  = (bbox<<6) + (bbox<<4) + (bbox<<3)      64+16+8 =  88
//      bbox×72  = (bbox<<6) + (bbox<<3)                  64+ 8   =  72
//      bbox×38  = (bbox<<5) + (bbox<<2) + (bbox<<1)      32+ 4+2 =  38
//    ⇒ **零除法、零 DSP**（本工程 DSP 已用 37/40，非常紧张）
//
//-----------------------------------------------------------------------------
//  三、分时计算（省资源的关键）
//-----------------------------------------------------------------------------
//  分类结果**每帧只变一次**（目标特征整帧锁存），所以不需要 4 份并行硬件：
//  帧末触发后，用 **1 个乘法器** 分 4 个槽顺序算完，每槽 3 拍、共 12 拍。
//
//  每槽 3 拍：
//    P0：锁存该槽的 w / h / area / valid
//    P1：bbox <= w × h （组合乘法，只此一处；流水一拍）
//    P2：算 area×100 与 bbox×阈值 → 比较 → 写 O_shapes[该槽]
//
//  ⚠ 全帧消隐期约 49500 拍，12 拍可忽略。
//=============================================================================

`timescale 1ns / 1ps
`include "common_defs.vh"

module shape_class #(
    parameter integer N_SLOT = 4,          // 槽数（与 target_bus 一致）
    // ---- 填充度阈值（×100，单位 %）----
    parameter integer F_SQ   = 88,         // f >= 88  → 方
    parameter integer F_CIR  = 72,         // 72 <= f < 88 → 圆
    parameter integer F_TRI  = 38          // 38 <= f < 72 → 三角；再低 → 未知
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [`TGT_BUS_W-1:0] I_bus,    // 4 槽 × 96 bit（conn_features.O_target_bus）
    input  wire        I_frame_done,       // 帧末脉冲（触发重算）

    output reg  [2*N_SLOT-1:0] O_shapes    // 每槽 2 bit，槽 0 在低位
);

    //=========================================================================
    //  0. 常量与解包
    //=========================================================================
    localparam integer SW = `TGT_SLOT_W;   // 96

    // 形状编码
    localparam [1:0] SHP_SQ = 2'd0, SHP_CIR = 2'd1, SHP_TRI = 2'd2, SHP_UNK = 2'd3;

    wire [3:0] slot_v;
    assign slot_v[0] = I_bus[`TGT_O_VALID + 0*SW];
    assign slot_v[1] = I_bus[`TGT_O_VALID + 1*SW];
    assign slot_v[2] = I_bus[`TGT_O_VALID + 2*SW];
    assign slot_v[3] = I_bus[`TGT_O_VALID + 3*SW];

    // 每槽的 xmin/xmax/ymin/ymax/area（组合解包；分时只选中的那槽参与运算）
    wire [10:0] xmin_k [0:N_SLOT-1];
    wire [10:0] xmax_k [0:N_SLOT-1];
    wire [9:0]  ymin_k [0:N_SLOT-1];
    wire [9:0]  ymax_k [0:N_SLOT-1];
    wire [19:0] area_k [0:N_SLOT-1];

    genvar gi;
    generate
        for (gi = 0; gi < N_SLOT; gi = gi + 1) begin : g_unpack
            assign xmin_k[gi] = I_bus[`TGT_O_XMIN + gi*SW +: 11];
            assign xmax_k[gi] = I_bus[`TGT_O_XMAX + gi*SW +: 11];
            assign ymin_k[gi] = I_bus[`TGT_O_YMIN + gi*SW +: 10];
            assign ymax_k[gi] = I_bus[`TGT_O_YMAX + gi*SW +: 10];
            assign area_k[gi] = I_bus[`TGT_O_AREA + gi*SW +: 20];
        end
    endgenerate

    //=========================================================================
    //  1. 分时状态机
    //=========================================================================
    localparam [3:0] ST_IDLE = 4'd0;
    localparam [3:0] ST_P0   = 4'd1;       // 锁存 w/h/area
    localparam [3:0] ST_P1   = 4'd2;       // 算 bbox
    localparam [3:0] ST_P2   = 4'd3;       // 比较出结果
    localparam [3:0] ST_NEXT = 4'd4;       // 下一槽

    reg [3:0] st;
    reg [1:0] k;                           // 当前槽号

    // 当前槽选中的输入（分时 mux）
    reg [10:0] xmn, xmx;
    reg [9:0]  ymn, ymx;
    reg [19:0] ar;
    reg        vs;

    // P0 锁存的中间量
    reg [11:0] w_r;                        // w = xmax-xmin+1，最大 1280 → 12 bit
    reg [10:0] h_r;                        // h = ymax-ymin+1，最大  720 → 11 bit
    reg [19:0] area_r;
    reg        valid_r;

    // P1 结果
    reg [22:0] bbox_r;                     // w*h <= 1280*720 = 921600 → 20 bit，留 23
    reg [26:0] ar100_r;                    // area*100 <= 92160000 → 27 bit

    // P2 的常量乘法（移位加法，纯组合）
    wire [26:0] bbox_x88 = ({bbox_r, 6'd0}  + {bbox_r, 4'd0}  + {bbox_r, 3'd0});
    wire [26:0] bbox_x72 = ({bbox_r, 6'd0}  + {bbox_r, 3'd0});
    wire [26:0] bbox_x38 = ({bbox_r, 5'd0}  + {bbox_r, 2'd0}  + {bbox_r, 1'd0});

    // 填充度比较（交叉相乘，全部无除法）
    wire ge_tri = (ar100_r >= bbox_x38);
    wire ge_cir = (ar100_r >= bbox_x72);
    wire ge_sq  = (ar100_r >= bbox_x88);

    wire [1:0] shape_now = valid_r ? (ge_sq ? SHP_SQ :
                                      ge_cir ? SHP_CIR :
                                      ge_tri ? SHP_TRI : SHP_UNK)
                                   : SHP_UNK;

    // 当前槽的输入 mux（组合）
    always @(*) begin
        case (k)
            2'd0: begin xmn = xmin_k[0]; xmx = xmax_k[0]; ymn = ymin_k[0]; ymx = ymax_k[0]; ar = area_k[0]; vs = slot_v[0]; end
            2'd1: begin xmn = xmin_k[1]; xmx = xmax_k[1]; ymn = ymin_k[1]; ymx = ymax_k[1]; ar = area_k[1]; vs = slot_v[1]; end
            2'd2: begin xmn = xmin_k[2]; xmx = xmax_k[2]; ymn = ymin_k[2]; ymx = ymax_k[2]; ar = area_k[2]; vs = slot_v[2]; end
            default: begin xmn = xmin_k[3]; xmx = xmax_k[3]; ymn = ymin_k[3]; ymx = ymax_k[3]; ar = area_k[3]; vs = slot_v[3]; end
        endcase
    end

    //=========================================================================
    //  2. 状态推进
    //=========================================================================
    integer ii;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= ST_IDLE; k <= 2'd0;
            w_r <= 12'd0; h_r <= 11'd0; area_r <= 20'd0; valid_r <= 1'b0;
            bbox_r <= 23'd0; ar100_r <= 27'd0;
            O_shapes <= {2*N_SLOT{1'b1}};      // 上电全「未知」
        end else begin
            case (st)
                //---------------------------------------------------------
                ST_IDLE: begin
                    if (I_frame_done) begin
                        k  <= 2'd0;
                        st <= ST_P0;
                    end
                end
                //---------------------------------------------------------
                ST_P0: begin
                    // 锁存当前槽的 w / h / area / valid
                    w_r     <= {1'b0, xmx} - {1'b0, xmn} + 12'd1;
                    h_r     <= {1'b0, ymx} - {1'b0, ymn} + 11'd1;
                    area_r  <= ar;
                    valid_r <= vs;
                    st      <= ST_P1;
                end
                //---------------------------------------------------------
                ST_P1: begin
                    // bbox = w × h（唯一一处乘法，组合 + 流水一拍）
                    bbox_r  <= w_r * h_r;
                    // area×100 = (area<<6)+(area<<5)+(area<<2)
                    ar100_r <= ({7'd0, area_r, 6'd0} + {7'd0, area_r, 5'd0} + {7'd0, area_r, 2'd0});
                    st      <= ST_P2;
                end
                //---------------------------------------------------------
                ST_P2: begin
                    // 比较出结果，写入对应槽的 2 bit
                    case (k)
                        2'd0: O_shapes[1:0]   <= shape_now;
                        2'd1: O_shapes[3:2]   <= shape_now;
                        2'd2: O_shapes[5:4]   <= shape_now;
                        default: O_shapes[7:6] <= shape_now;
                    endcase
                    st <= ST_NEXT;
                end
                //---------------------------------------------------------
                ST_NEXT: begin
                    if (k == (N_SLOT - 1)) begin
                        st <= ST_IDLE;
                    end else begin
                        k  <= k + 2'd1;
                        st <= ST_P0;
                    end
                end
                //---------------------------------------------------------
                default: st <= ST_IDLE;
            endcase
        end
    end

endmodule
