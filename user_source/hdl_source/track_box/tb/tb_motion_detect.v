//=====================================================================
//  tb_motion_detect.v  —— D7 运动检测模块级 TB（合成 1280x720 视频激励）
//---------------------------------------------------------------------
//  归属：张铭晨（进阶4 运动检测）
//
//  为什么这么造激励
//    运动检测的「正确性」= 对**上一帧与当前帧的差异**判定得对。所以 TB 不去
//    造真实相机画面，而是造**可控的两帧差分**：用「块常量图案」让每个 4x4 块的
//    灰度精确可控（R=G=B=v ⇒ rgb2gray 恰好输出 v），于是降采样域 ds(c,r) 就等于
//    该块的值 —— 与流水相位无关，Python 参考实现可以精确复算。
//
//  场景表（块坐标，1 块 = 64 像素；块网格 20 列 x 11 行 = ds 320x176）
//    帧号  场景                                    期望 OUT（帧末打印）
//     0   全黑                                    （缓冲初始化帧，不打印）
//     1   全黑                                    0 区  —— 静止零误报
//     2   A(3..7, 2..6)=200                       1 区  x192..511 y128..447
//     3   A 不变                                  0 区  —— 静止零误报
//     4   A 右移 4 块(7..11)                      1 区  x192..767 y128..447（隔 1 块合并）
//     5   只剩单块 C(15,8)=200                    1 区  x448..767 —— C 单块被滤掉
//     6   C 不变                                  0 区
//     7   A2(0..3,1..3)=200 + B(15..18,8..10)=120 2 区
//     8   不变                                    0 区
//     9   全黑                                    2 区（A2 区 + B 区）
//    10   列残留图案 v=[0,60,0,0]                 1 区  x0..1279 y0..703（降采样相位判据）
//    11   不变                                     0 区
//    12   全黑                                     1 区  x0..1279 y0..703
//
//  为什么帧 10 能判「降采样相位对不对」
//    图案按 x%4 取值 v=[0,120,0,0]（沿 y 不变）。取块左列 2 像素（列 4c,4c+1）
//    ⇒ ds = 4 行×(0+120) >>3 = 60 ≥ MOT_TH=32 ⇒ 全屏运动（1 个大框）；
//    若补偿错 1 拍、取成列 4c-1,4c（残留 0,0）⇒ ds = 0 ⇒ 一区也不出。
//    两种结果二值可分，能钉死相位。
//
//  输出打印格式（供 sim/bit_sim_motion_detect.py 抓取）
//    OUT end_f=<帧号> cnt=<n> act=<活动块数> | v xmin xmax ymin ymax | ... （4 槽）
//    SCENE f=<帧号> <标签>
//
//  运行
//    cd user_source/hdl_source/track_box
//    iverilog -g2005 -o sim/tb_motion_detect.vvp tb/tb_motion_detect.v \
//             rtl/motion_detect.v ../rgb2gray.v
//    vvp sim/tb_motion_detect.vvp | tee sim/motion_detect_rtl_out.txt
//=====================================================================
`timescale 1ns/1ps

module tb_motion_detect;

    localparam integer H_ACTIVE = 1280;
    localparam integer V_ACTIVE = 720;
    localparam integer N_FRAMES = 13;        // 帧 0..12

    // ---------------- 时钟 / 复位 ----------------
    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    initial begin
        rst_n = 1'b0;
        #200;
        rst_n = 1'b1;
    end

    // ---------------- 像素扫描（无消隐，逐行 1280x720）----------------
    reg [10:0] x;
    reg [9:0]  y;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= 11'd0;
            y <= 10'd0;
        end else if (x == (H_ACTIVE-1)) begin
            x <= 11'd0;
            y <= (y == (V_ACTIVE-1)) ? 10'd0 : (y + 10'd1);
        end else begin
            x <= x + 11'd1;
        end
    end

    wire I_de         = 1'b1;                               // 本 TB 无消隐
    wire I_frame_done = (x == 11'd0) && (y == 10'd0);       // 帧首有效像素

    // ---------------- 帧号 ----------------
    integer frame_idx;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) frame_idx <= 0;
        else if (x == 11'd0 && y == 10'd0) frame_idx <= frame_idx + 1;
    end

    // ---------------- 场景表（块坐标）----------------
    reg        S_r0on, S_r1on, S_paton;
    reg [4:0]  S_r0x1, S_r0x2, S_r1x1, S_r1x2;
    reg [3:0]  S_r0y1, S_r0y2, S_r1y1, S_r1y2;
    reg [7:0]  S_r0v,  S_r1v;
    reg [7:0]  S_pv0, S_pv1, S_pv2, S_pv3;

    always @(*) begin
        S_r0on = 1'b0; S_r1on = 1'b0; S_paton = 1'b0;
        S_r0x1 = 5'd0; S_r0x2 = 5'd0; S_r0y1 = 4'd0; S_r0y2 = 4'd0; S_r0v = 8'd0;
        S_r1x1 = 5'd0; S_r1x2 = 5'd0; S_r1y1 = 4'd0; S_r1y2 = 4'd0; S_r1v = 8'd0;
        S_pv0 = 8'd0; S_pv1 = 8'd0; S_pv2 = 8'd0; S_pv3 = 8'd0;
        case (frame_idx)
            2, 3: begin
                S_r0on = 1'b1; S_r0x1 = 5'd3;  S_r0x2 = 5'd7;
                S_r0y1 = 4'd2; S_r0y2 = 4'd6;  S_r0v = 8'd200;
            end
            4: begin
                S_r0on = 1'b1; S_r0x1 = 5'd7;  S_r0x2 = 5'd11;
                S_r0y1 = 4'd2; S_r0y2 = 4'd6;  S_r0v = 8'd200;
            end
            5, 6: begin
                S_r0on = 1'b1; S_r0x1 = 5'd15; S_r0x2 = 5'd15;
                S_r0y1 = 4'd8; S_r0y2 = 4'd8;  S_r0v = 8'd200;
            end
            7, 8: begin
                S_r0on = 1'b1; S_r0x1 = 5'd0;  S_r0x2 = 5'd3;
                S_r0y1 = 4'd1; S_r0y2 = 4'd3;  S_r0v = 8'd200;
                S_r1on = 1'b1; S_r1x1 = 5'd15; S_r1x2 = 5'd18;
                S_r1y1 = 4'd8; S_r1y2 = 4'd10; S_r1v = 8'd120;
            end
            10, 11: begin
                S_paton = 1'b1;
                S_pv0 = 8'd0; S_pv1 = 8'd120; S_pv2 = 8'd0; S_pv3 = 8'd0;
            end
            default: ; // 全黑
        endcase
    end

    // ---------------- 像素值生成 ----------------
    //  ⚠ 场景表用的是「64 像素块坐标」（块网格 20x11），所以取 x[10:6] / y[9:6]；
    //    不能写成 x[10:2]（那是 4 像素的 ds 单元坐标，会把矩形缩小 16 倍）
    reg [4:0] S_bx;
    reg [3:0] S_by;
    reg [7:0] S_pv;
    always @(*) begin
        S_bx = x[10:6];
        S_by = y[9:6];
        S_pv = 8'd0;
        if (S_paton) begin
            case (x[1:0])
                2'd0: S_pv = S_pv0;
                2'd1: S_pv = S_pv1;
                2'd2: S_pv = S_pv2;
                default: S_pv = S_pv3;
            endcase
        end else begin
            if (S_r0on && (S_bx >= S_r0x1) && (S_bx <= S_r0x2) &&
                          (S_by >= S_r0y1) && (S_by <= S_r0y2)) S_pv = S_r0v;
            if (S_r1on && (S_bx >= S_r1x1) && (S_bx <= S_r1x2) &&
                          (S_by >= S_r1y1) && (S_by <= S_r1y2)) S_pv = S_r1v;
        end
    end
    wire [23:0] I_rgb = {S_pv, S_pv, S_pv};

    // ---------------- DUT ----------------
    wire [3:0]  O_mot_valid;
    wire [43:0] O_mot_xmin, O_mot_xmax;
    wire [39:0] O_mot_ymin, O_mot_ymax;
    wire [2:0]  O_mot_cnt;
    wire        O_mot_pix, O_mot_pix_v;
    wire [7:0]  O_act_blk;

    motion_detect dut (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_de(I_de), .I_pix_x(x), .I_pix_y(y), .I_rgb(I_rgb),
        .I_frame_done(I_frame_done),
        .O_mot_valid(O_mot_valid),
        .O_mot_xmin(O_mot_xmin), .O_mot_xmax(O_mot_xmax),
        .O_mot_ymin(O_mot_ymin), .O_mot_ymax(O_mot_ymax),
        .O_mot_cnt(O_mot_cnt),
        .O_mot_pix(O_mot_pix), .O_mot_pix_v(O_mot_pix_v),
        .O_act_blk(O_act_blk)
    );

    // ---------------- 逐帧打印 ----------------
    integer k;
    always @(posedge clk) begin
        if (rst_n && x == (H_ACTIVE-1) && y == (V_ACTIVE-1) && frame_idx >= 1) begin
            $display("OUT end_f=%0d cnt=%0d act=%0d | %0d %0d %0d %0d %0d | %0d %0d %0d %0d %0d | %0d %0d %0d %0d %0d | %0d %0d %0d %0d %0d",
                frame_idx, O_mot_cnt, O_act_blk,
                O_mot_valid[0], O_mot_xmin[0*11 +: 11], O_mot_xmax[0*11 +: 11], O_mot_ymin[0*10 +: 10], O_mot_ymax[0*10 +: 10],
                O_mot_valid[1], O_mot_xmin[1*11 +: 11], O_mot_xmax[1*11 +: 11], O_mot_ymin[1*10 +: 10], O_mot_ymax[1*10 +: 10],
                O_mot_valid[2], O_mot_xmin[2*11 +: 11], O_mot_xmax[2*11 +: 11], O_mot_ymin[2*10 +: 10], O_mot_ymax[2*10 +: 10],
                O_mot_valid[3], O_mot_xmin[3*11 +: 11], O_mot_xmax[3*11 +: 11], O_mot_ymin[3*10 +: 10], O_mot_ymax[3*10 +: 10]);
        end
    end

    // ---------------- 收尾 ----------------
    reg [3:0] dwait;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) dwait <= 4'd0;
        else if (x == (H_ACTIVE-1) && y == (V_ACTIVE-1) && frame_idx == (N_FRAMES-1)) dwait <= 4'd1;
        else if (dwait != 4'd0 && dwait < 4'd9) dwait <= dwait + 4'd1;
    end

    always @(posedge clk) begin
        if (dwait == 4'd8) begin
            $display("ALL_DONE frames=%0d", N_FRAMES);
            $finish;
        end
    end

endmodule
