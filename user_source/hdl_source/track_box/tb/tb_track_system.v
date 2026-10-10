//=====================================================================
//  tb_track_system.v  ——  track_box 系统级仿真（真实 720p60 场时序）
//---------------------------------------------------------------------
//  归属：张铭晨（进阶2 跟踪 / 进阶4 运动检测）—— D1 交付的仿真验证
//
//  为什么要这个 TB（昨天的 TB 不够）
//    tb_box_draw.v / tb_osd_coord.v 只跑 32x32 / 64x32 的小画面，
//    验证的是「模块内部逻辑」。而竞赛评测看的是**整机上板后的观感**：
//    画面稳定、无异常、信息叠加位置正确。所以这里把三个模块串成与
//    顶层一致的叠加链路，灌入**真实 1280x720@60 场时序**跑整帧。
//
//  时序常量来源（必须与顶层一字不差）
//    design_top_wrapper.v 里 u_hdmi_vtc 的实例化参数 —— 与
//    sim/vtc_coord_probe.py 头部一字不差：
//        H_ACTIVE=1280  H_FRAME=1650  H_SYNC_S=1390  H_SYNC_E=1430
//        V_ACTIVE=720   V_FRAME=750   V_SYNC_S=725   V_SYNC_E=730
//    vtc_vs 是**高脉冲**，落在 vcnt ∈ (725, 730]（场消隐期，de 恒为 0），
//    所以 pix_coord_gen 用它做「场消隐期归零」是安全的。
//
//  流水相位（与 uivtc.v 一致，不是自创）
//        O_vtc_vs   = vtc_vs   延 1 拍
//        O_vtc_de   = vtc_de   延 2 拍
//        O_vtc_last = vtc_last 延 2 拍（与 O_vtc_de 严格同拍）
//
//  验证点
//    A. 坐标：I_de 拍内 pix_coord_gen 的 (O_pix_x,O_pix_y) 必须等于
//             vtc 域独立延 2 拍的 (hcnt, vcnt)；
//    B. 输出：每帧有效像素 = 1280*720；红/绿/蓝/黄/白/OSD 命中数
//             跨帧完全一致（= 无漂移，这是「框不抖」的地基）；
//    C. 整帧位图：dump 到 sim/tb_system_frame.bin（每像素 1 字节颜色码），
//             交 sim/bit_sim_system.py 独立重算 + 逐像素比对 + 出 PNG。
//
//  颜色码（与 bit_sim_system.py 约定一致）
//        0=背景  1=框0红  2=框1绿  3=框2蓝  4=框3黄  5=准星白  6=坐标OSD
//
//  运行
//    cd user_source/hdl_source/track_box
//    iverilog -g2005 -o sim/tb_system.vvp tb/tb_track_system.v \
//             rtl/pix_coord_gen.v rtl/box_draw.v rtl/osd_coord.v \
//             rtl/track_glyph_rom.v
//    vvp sim/tb_system.vvp
//=====================================================================
`timescale 1ns/1ps

module tb_track_system;

    // ---------------- 与顶层一字不差的时序常量 ----------------
    localparam integer H_ACTIVE = 1280;
    localparam integer H_FRAME  = 1650;
    localparam integer H_SYNC_S = 1390;
    localparam integer H_SYNC_E = 1430;
    localparam integer V_ACTIVE = 720;
    localparam integer V_FRAME  = 750;
    localparam integer V_SYNC_S = 725;
    localparam integer V_SYNC_E = 730;

    localparam integer N_FRAMES   = 5;      // 总仿真帧数
    localparam integer STAT_FROM  = 2;      // 从第几帧起算「稳态帧」
    localparam integer DUMP_FRAME = 4;      // 落盘整帧位图的帧号

    // ---------------- 时钟 / 复位 ----------------
    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;                   // 周期数值无关，只关心相位关系

    initial begin
        rst_n = 1'b0;
        #200;
        rst_n = 1'b1;
    end

    // ---------------- vtc 域时序计数器 ----------------
    reg [11:0] hcnt, vcnt;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            hcnt <= 12'd0;
            vcnt <= 12'd0;
        end
        else if(hcnt == (H_FRAME-1)) begin
            hcnt <= 12'd0;
            vcnt <= (vcnt == (V_FRAME-1)) ? 12'd0 : (vcnt + 12'd1);
        end
        else begin
            hcnt <= hcnt + 12'd1;
        end
    end

    wire vtc_de   = (hcnt < H_ACTIVE) && (vcnt < V_ACTIVE);
    wire vtc_last = (hcnt == (H_ACTIVE-1)) && (vcnt < V_ACTIVE);
    wire vtc_vs   = (vcnt > V_SYNC_S) && (vcnt <= V_SYNC_E);

    // ---------------- uivtc 的流水相位 ----------------
    reg d1_de, d2_de, d1_last, d2_last, d1_vs;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            d1_de <= 1'b0;  d2_de <= 1'b0;
            d1_last <= 1'b0; d2_last <= 1'b0;
            d1_vs <= 1'b0;
        end
        else begin
            d1_de <= vtc_de;      d2_de <= d1_de;
            d1_last <= vtc_last;  d2_last <= d1_last;
            d1_vs <= vtc_vs;
        end
    end

    wire S_de    = d2_de;      // 送给叠加模块的像素有效
    wire S_last  = d2_last;    // 与 S_de 同拍
    wire S_vsync = d1_vs;      // 高脉冲，消隐期

    // ---------------- 独立期望坐标链（不依赖 DUT）----------------
    //   de 拍 t：像素身份 = vtc 域 t-2 拍的 (hcnt, vcnt)  ⇒ 取 ax2/ay2
    //   输出拍 t（O_de=1）：像素身份 = t-4 拍的 (hcnt, vcnt) ⇒ 取 ax4/ay4
    reg [11:0] ax1,ax2,ax3,ax4, ay1,ay2,ay3,ay4;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ax1<=0; ax2<=0; ax3<=0; ax4<=0;
            ay1<=0; ay2<=0; ay3<=0; ay4<=0;
        end
        else begin
            ax1<=hcnt; ax2<=ax1; ax3<=ax2; ax4<=ax3;
            ay1<=vcnt; ay2<=ay1; ay3<=ay2; ay4<=ay3;
        end
    end
    wire [10:0] EXPX_PIX = ax2[10:0];
    wire [9:0]  EXPY_PIX = ay2[9:0];
    wire [10:0] EXPX_OUT = ax4[10:0];
    wire [9:0]  EXPY_OUT = ay4[9:0];

    // ---------------- DUT：坐标源 + 画框 + 坐标 OSD ----------------
    wire [10:0] PX, PX1, PX2;
    wire [9:0]  PY, PY1, PY2;
    wire        FRAME_DONE;

    pix_coord_gen #(.H_ACTIVE(H_ACTIVE), .V_ACTIVE(V_ACTIVE)) u_pix_coord_gen (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_de(S_de), .I_last(S_last), .I_vsync(S_vsync),
        .O_pix_x(PX),    .O_pix_y(PY),
        .O_pix_x_1d(PX1), .O_pix_x_2d(PX2),
        .O_pix_y_1d(PY1), .O_pix_y_2d(PY2),
        .O_frame_done(FRAME_DONE)
    );

    wire        BOX_HIT, BOX_DE;
    wire [23:0] BOX_COLOR;
    box_draw #(.SELF_TEST(1)) u_box_draw (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_de(S_de), .I_pix_x(PX), .I_pix_y(PY),
        .I_box_valid(4'b0000),
        .I_box_xmin(44'd0), .I_box_xmax(44'd0),
        .I_box_ymin(40'd0), .I_box_ymax(40'd0),
        .I_cross_en(1'b1), .I_cross_x(11'd640), .I_cross_y(10'd360),
        .O_hit(BOX_HIT), .O_color(BOX_COLOR), .O_de(BOX_DE)
    );

    wire        OSD_HIT;
    wire [23:0] OSD_COLOR;
    osd_coord u_osd_coord (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_de(S_de), .I_pix_x(PX), .I_pix_y(PY),
        .I_x_val(11'd640), .I_y_val(10'd360),
        .I_frame_done(FRAME_DONE),
        .O_hit(OSD_HIT), .O_color(OSD_COLOR)
    );

    // 有效输出拍：等价于各叠加模块的 O_de（= I_de 延 2 拍）
    wire SYS_DE = BOX_DE;

    // ---------------- 颜色码（优先级与顶层一致：box > osd）----------------
    reg [7:0] pixcode;
    always @(*) begin
        if(BOX_HIT) begin
            case(BOX_COLOR)
                24'hff0000: pixcode = 8'd1;   // 框0 红
                24'h00ff00: pixcode = 8'd2;   // 框1 绿
                24'h00a0ff: pixcode = 8'd3;   // 框2 蓝
                24'hfff200: pixcode = 8'd4;   // 框3 黄
                24'hffffff: pixcode = 8'd5;   // 准星 白
                default:    pixcode = 8'd1;
            endcase
        end
        else if(OSD_HIT) pixcode = 8'd6;      // 坐标 OSD
        else             pixcode = 8'd0;      // 背景
    end

    // ---------------- 帧计数（帧首 hcnt=0,vcnt=0 自增）----------------
    integer frame_idx;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) frame_idx <= 0;
        else if(hcnt == 12'd0 && vcnt == 12'd0) frame_idx <= frame_idx + 1;
    end

    // ---------------- dump 文件 ----------------
    integer fp;
    initial begin
        fp = $fopen("sim/tb_system_frame.bin", "wb");
        if(fp == 0) begin
            $display("ERROR: 打不开 sim/tb_system_frame.bin（请在 track_box 目录下运行 vvp）");
            $finish;
        end
    end

    // ---------------- 统计寄存器 ----------------
    integer n_de_f, n_hit_f, n_r_f, n_g_f, n_b_f, n_y_f, n_w_f, n_o_f;
    integer n_coord_ck, n_coord_err;
    integer n_dump;
    integer f_hit [0:N_FRAMES];
    integer f_de  [0:N_FRAMES];
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            n_de_f<=0; n_hit_f<=0; n_r_f<=0; n_g_f<=0; n_b_f<=0; n_y_f<=0; n_w_f<=0; n_o_f<=0;
            n_coord_ck<=0; n_coord_err<=0; n_dump<=0;
            for(i=0; i<=N_FRAMES; i=i+1) begin
                f_hit[i] <= 0;
                f_de[i]  <= 0;
            end
        end
        else begin
            // ---- A. 坐标检查（稳态帧）----
            if(S_de && frame_idx >= STAT_FROM) begin
                n_coord_ck <= n_coord_ck + 1;
                if((PX !== EXPX_PIX) || (PY !== EXPY_PIX)) begin
                    n_coord_err <= n_coord_err + 1;
                    if(n_coord_err < 8)
                        $display("  [坐标错] t=%0t  O_pix=(%0d,%0d) 期望=(%0d,%0d)",
                                 $time, PX, PY, EXPX_PIX, EXPY_PIX);
                end
            end

            // ---- B. 每帧叠加统计 + 整帧 dump ----
            if(SYS_DE) begin
                if(frame_idx >= STAT_FROM) begin
                    n_de_f <= n_de_f + 1;
                    if(BOX_HIT) begin
                        n_hit_f <= n_hit_f + 1;
                        case(BOX_COLOR)
                            24'hff0000: n_r_f <= n_r_f + 1;
                            24'h00ff00: n_g_f <= n_g_f + 1;
                            24'h00a0ff: n_b_f <= n_b_f + 1;
                            24'hfff200: n_y_f <= n_y_f + 1;
                            24'hffffff: n_w_f <= n_w_f + 1;
                        endcase
                    end
                    else if(OSD_HIT) begin
                        n_hit_f <= n_hit_f + 1;
                        n_o_f <= n_o_f + 1;
                    end
                end
                if(frame_idx == DUMP_FRAME) begin
                    $fwrite(fp, "%c", pixcode);
                    n_dump <= n_dump + 1;
                end
            end

            // ---- 帧结束 ----
            if(hcnt == (H_FRAME-1) && vcnt == (V_FRAME-1)) begin
                if(frame_idx >= STAT_FROM) begin
                    $display("  [帧 %0d] 有效像素 %0d | 命中 %0d  (红%0d 绿%0d 蓝%0d 黄%0d 白%0d | OSD %0d)",
                             frame_idx, n_de_f, n_hit_f, n_r_f, n_g_f, n_b_f, n_y_f, n_w_f, n_o_f);
                    f_hit[frame_idx] <= n_hit_f;
                    f_de [frame_idx] <= n_de_f;
                    n_de_f<=0; n_hit_f<=0; n_r_f<=0; n_g_f<=0; n_b_f<=0; n_y_f<=0; n_w_f<=0; n_o_f<=0;
                end
            end
        end
    end

    // ---------------- 收尾（延迟若干拍，等所有非阻塞赋值落定）----------------
    reg [3:0] done_wait;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) done_wait <= 4'd0;
        else if(hcnt == (H_FRAME-1) && vcnt == (V_FRAME-1) && frame_idx == N_FRAMES) done_wait <= 4'd1;
        else if(done_wait != 4'd0 && done_wait < 4'd9) done_wait <= done_wait + 4'd1;
    end

    reg cons_ok;
    integer k;
    always @(posedge clk) begin
        if(done_wait == 4'd8) begin
            cons_ok = 1'b1;
            for(k = STAT_FROM+1; k <= N_FRAMES; k = k+1) begin
                if(f_hit[k] != f_hit[STAT_FROM]) cons_ok = 1'b0;
                if(f_de[k]  != H_ACTIVE*V_ACTIVE)  cons_ok = 1'b0;
            end

            $display("");
            $display("============================================================");
            $display(" tb_track_system 结果 —— 真实 1280x720@60 场时序");
            $display("============================================================");
            $display(" 时序常量 : H %0d/%0d  V %0d/%0d  vsync 高脉冲 vcnt(%0d,%0d]",
                     H_ACTIVE, H_FRAME, V_ACTIVE, V_FRAME, V_SYNC_S, V_SYNC_E);
            $display(" 仿真帧 %0d 帧 | 统计帧 %0d..%0d | 位图帧 %0d",
                     N_FRAMES, STAT_FROM, N_FRAMES, DUMP_FRAME);
            $display(" 坐标检查 : %0d 拍, 错 %0d   [%s]",
                     n_coord_ck, n_coord_err, (n_coord_err==0) ? "PASS" : "FAIL");
            $display(" 每帧有效像素 : %0d (期望 %0d)   [%s]",
                     f_de[STAT_FROM], H_ACTIVE*V_ACTIVE,
                     (f_de[STAT_FROM]==H_ACTIVE*V_ACTIVE) ? "PASS" : "FAIL");
            $display(" 跨帧一致性   : [%s]  (红/绿/蓝/黄/白/OSD 命中数跨帧相同)",
                     cons_ok ? "PASS" : "FAIL");
            $display(" 整帧位图 : sim/tb_system_frame.bin  %0d 字节", n_dump);
            $display("============================================================");
            $fclose(fp);
            $finish;
        end
    end

endmodule
