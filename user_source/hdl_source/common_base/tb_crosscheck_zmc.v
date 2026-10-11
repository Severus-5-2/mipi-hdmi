//=============================================================================
//  tb_crosscheck_zmc.v  —  跨实现对拍：张金艺 RTL 链 vs 张铭晨期望值
//=============================================================================
//  目的：用张铭晨 tb_d2d4_chain.v 的**同一组 4 帧掩膜**，喂**我方** RTL 链
//        （detect_mask → conn_label → conn_features → O_target_bus），
//        导出每帧 4 槽 96bit，与张铭晨报告 §3.2 的期望值逐字段比对。
//
//  对拍原则：比"输出"不比"模块接口"（两边内部架构不同，接口形状不同，
//            但 S_target_bus 这一跨人接口必须逐字段一致）。
//
//  掩膜（与 zmc tb_d2d4_chain.v 完全相同）：
//    variant 0: 8x4@(1,1)cls0  6x6@(20,2)cls1  5x3@(3,10)cls2  4x4@(24,10)cls3
//    variant 1: 10x3@(0,0)cls2 10x3@(0,6)cls0  （同面积，测 xmin 兜底）
//    variant 2: 3x3@(5,5)cls1   （单目标）
//    variant 3: 无目标
//
//  用法：  iverilog -g2005 -I <common_base> -o x.vvp tb_crosscheck_zmc.v \
//                     conn_label.v conn_features.v target_bus.v
//          vvp x.vvp
//
//  ★ 实测结论（2026-10-11）：
//      本 TB 默认门限走 conn_label 默认 MIN_AREA=16（冻结值，见冻结表 v1.5）。
//      用门限 16 时，4 帧输出与张铭晨报告期望值**逐字段完全一致**：
//        帧0: 3 槽，面积 36/32/16（area=15 按门限丢）
//        帧1: 两个 area=30 → xmin 升序兜底生效 ✅
//        帧2: 无目标（area=9 按门限丢），valid_cnt=0 ✅
//        帧3: 无目标，valid_cnt=0、全槽清零 ✅
//      （首轮曾用门限 8 验证算法：帧0 四槽 36/32/16/15、帧2 area=9；门限最终冻结为 16。）
//      ⇒ **算法核心零偏差**。唯一需拉齐的「最小面积门限」已统一为 16，「cls 注入」双方已改用真实掩膜。
//      ⚠ tb 必须每行之间插 hblank（de=0）—— conn_label 靠 de 下降沿做行缓冲 bank 交换；
//        逐行连续 de=1（张铭晨 tb_d2d4_chain 的喂法）会让上一行游程不可见 → 每行独立成域。
//=============================================================================
`timescale 1ns / 1ps
`include "common_defs.vh"

module tb_crosscheck_zmc;

    localparam integer IW = 32;     // 仿真用帧宽（与 zmc TB 一致）
    localparam integer IH = 16;     // 仿真用帧高

    reg clk = 0, rst_n = 0;
    reg de, vsync, last, mask, fdone;
    reg [10:0] px;
    reg [9:0]  py;
    reg [1:0]  col;

    // conn_label ← conn_features 汇总流
    wire        cv;     wire [5:0]  clb;  wire [1:0] ccls;
    wire [19:0] carea;  wire [10:0] cxmin, cxmax, ccx;
    wire [9:0]  cymin, cymax, ccy;  wire [5:0] ccnt;

    // conn_features 输出
    wire [`TGT_BUS_W-1:0] tbus;
    wire [2:0]            vcnt;
    wire                  upd;

    conn_label #(
        .IMG_WIDTH(IW), .IMG_HEIGHT(IH)
    ) u_cl (
        .clk(clk), .rst_n(rst_n),
        .I_de(de), .I_mask(mask), .I_color(col),
        .I_pix_x(px), .I_pix_y(py),
        .I_frame_done(fdone),
        .O_label(), .O_label_valid(), .O_label_de(),
        .O_label_pix_x(), .O_label_pix_y(),
        .O_cmp_valid(cv), .O_cmp_label(clb), .O_cmp_cls(ccls),
        .O_cmp_area(carea), .O_cmp_xmin(cxmin), .O_cmp_xmax(cxmax),
        .O_cmp_cx(ccx), .O_cmp_ymin(cymin), .O_cmp_ymax(cymax),
        .O_cmp_cy(ccy), .O_cmp_cnt(ccnt)
    );

    conn_features #(.SLOT_N(4)) u_cf (
        .clk(clk), .rst_n(rst_n),
        .I_frame_done(fdone),
        .I_cmp_valid(cv), .I_cmp_label(clb), .I_cmp_cls(ccls),
        .I_cmp_area(carea), .I_cmp_xmin(cxmin), .I_cmp_xmax(cxmax),
        .I_cmp_ymin(cymin), .I_cmp_ymax(cymax),
        .I_cmp_cx(ccx), .I_cmp_cy(ccy),
        .O_target_bus(tbus), .O_target_valid_cnt(vcnt), .O_update(upd)
    );

    always #5 clk = ~clk;

    // ---- 掩膜图案（与 zmc 完全一致）----
    reg [IW-1:0] M [0:IH-1];
    integer TX[0:3], TY[0:3], TW[0:3], TH[0:3], TC[0:3];

    task build_frame;
        input integer variant;
        integer yy, xx, t, sxx, syy;
        begin
            for (yy=0; yy<IH; yy=yy+1) M[yy] = 0;
            case (variant)
            0: begin
                TX[0]=1;  TY[0]=1;  TW[0]=8; TH[0]=4;  TC[0]=0;
                TX[1]=20; TY[1]=2;  TW[1]=6; TH[1]=6;  TC[1]=1;
                TX[2]=3;  TY[2]=10; TW[2]=5; TH[2]=3;  TC[2]=2;
                TX[3]=24; TY[3]=10; TW[3]=4; TH[3]=4;  TC[3]=3;
            end
            1: begin
                TX[0]=0;  TY[0]=0;  TW[0]=10; TH[0]=3; TC[0]=2;
                TX[1]=0;  TY[1]=6;  TW[1]=10; TH[1]=3; TC[1]=0;
                TX[2]=0;  TY[2]=0;  TW[2]=0;  TH[2]=0; TC[2]=3;
                TX[3]=0;  TY[3]=0;  TW[3]=0;  TH[3]=0; TC[3]=3;
            end
            2: begin
                TX[0]=5;  TY[0]=5;  TW[0]=3; TH[0]=3; TC[0]=1;
                TX[1]=0;  TY[1]=0;  TW[1]=0; TH[1]=0; TC[1]=0;
                TX[2]=0;  TY[2]=0;  TW[2]=0; TH[2]=0; TC[2]=0;
                TX[3]=0;  TY[3]=0;  TW[3]=0; TH[3]=0; TC[3]=0;
            end
            default: begin
                TX[0]=0; TY[0]=0; TW[0]=0; TH[0]=0; TC[0]=0;
                TX[1]=0; TY[1]=0; TW[1]=0; TH[1]=0; TC[1]=0;
                TX[2]=0; TY[2]=0; TW[2]=0; TH[2]=0; TC[2]=0;
                TX[3]=0; TY[3]=0; TW[3]=0; TH[3]=0; TC[3]=0;
            end
            endcase
            for (t=0; t<4; t=t+1)
                if (TW[t]>0 && TH[t]>0)
                    for (syy=0; syy<TH[t]; syy=syy+1)
                        for (sxx=0; sxx<TW[t]; sxx=sxx+1)
                            if ((TY[t]+syy)<IH && (TX[t]+sxx)<IW)
                                M[TY[t]+syy][IW-1-(TX[t]+sxx)] = 1'b1;
        end
    endtask

    function integer whom;
        input integer x, y;
        integer t;
        begin
            whom = 0;
            for (t=0; t<4; t=t+1)
                if (TW[t]>0 && TH[t]>0)
                    if (x>=TX[t] && x<TX[t]+TW[t] && y>=TY[t] && y<TY[t]+TH[t])
                        whom = t+1;
        end
    endfunction

    integer f, i, px_i, py_i;
    reg [IW-1:0] rowbits;
    reg [95:0]   slot;

    task run_frame;
        input integer variant;
        input integer fno;
        begin
            build_frame(variant);
            $display("FRAME_BEGIN %0d", fno);
            // vsync 上升沿（坐标系归零）
            @(negedge clk); vsync=1; @(posedge clk);
            @(negedge clk); vsync=0;
            // 90 行有效（IH + 若干行让帧末 FSM 跑完）
            for (py_i=0; py_i<IH; py_i=py_i+1) begin
                rowbits = M[py_i];
                for (px_i=0; px_i<IW; px_i=px_i+1) begin
                    @(negedge clk);
                    de=1; px=px_i[10:0]; py=py_i[9:0];
                    mask = rowbits[IW-1-px_i];
                    last = (px_i==IW-1);
                    col  = whom(px_i, py_i)-1;  // 用真实 cls（0..3）
                    @(posedge clk);
                end
            // ★ 行间 hblank：de 拉低若干拍 —— conn_label 靠 de 下降沿做行缓冲 bank 交换。
            //   真实 HDMI 每行有 hblank，务必模拟；逐行连续 de=1 会让上一行游程不可见。
            repeat (4) begin @(negedge clk); de=0; mask=0; last=0; @(posedge clk); end
            end

            // 收尾：拉低 de（此行必为最后一行 y=IH-1，触发内部 frame_end）
            @(negedge clk); de=0; mask=0; last=0; col=2'b11;
            // conn_label 帧末 FSM：35(ADDM) + 33(SCAN) + 32组件×64(DIV) + 33(OUT) ≈ 2150 拍
            // 留足 3000 拍让 FSM 跑完、汇总流推完
            repeat (3000) @(posedge clk);
            // 帧末脉冲（用 frame_done，不是 vsync）→ conn_features 锁存擂台
            @(negedge clk); fdone=1; @(posedge clk);
            @(negedge clk); fdone=0;
            repeat (5) @(posedge clk);

            // 导出 4 槽
            $display("VCNT %0d", vcnt);
            for (i=0; i<4; i=i+1) begin
                slot = tbus[i*96 +: 96];
                $display("SLOT %0d v=%0d cls=%0d cx=%0d cy=%0d xmin=%0d xmax=%0d ymin=%0d ymax=%0d area=%0d",
                    i, slot[0], slot[2:1],
                    slot[13:3], slot[23:14], slot[34:24], slot[45:35],
                    slot[55:46], slot[65:56], slot[85:66]);
            end
            $display("FRAME_END %0d", fno);
        end
    endtask

    initial begin
        de=0; vsync=0; last=0; mask=0; fdone=0; px=0; py=0; col=2'b11;
        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (4) @(posedge clk);
        run_frame(0, 0);
        run_frame(1, 1);
        run_frame(2, 2);
        run_frame(3, 3);
        $display("ALL_DONE");
        $finish;
    end

endmodule
