//=====================================================================
//  pix_coord_gen.v  ——  全画面公共像素坐标发生器（开发方案B 第 1.3 节）
//---------------------------------------------------------------------
//  归属：张铭晨（进阶2 跟踪 / 进阶4 运动检测）
//  定位：属于「三人共用的取数基础设施」。方案B 1.3 节明确要求
//        「三人都用同一套坐标，不要各写一份计数器，否则联调必然对不齐」，
//        本模块就是那一份唯一的坐标源：由郭在 top 里实例化一次，
//        再把 O_pix_x / O_pix_y 分发给艺（检测）和晨（跟踪 / 运动检测）。
//
//  接口语义
//    I_de    ：像素数据有效（与待处理像素同拍）
//    I_last  ：行末像素标志，**必须与 I_de 同拍**（已位级验证，见下）
//    I_vsync ：场同步（高有效，消隐期拉高）
//    O_pix_x[10:0]：0..1279，当前像素列号
//    O_pix_y[9:0] ：0..719， 当前像素行号
//    O_pix_x_1d/_2d、O_pix_y_1d/_2d：配套流水延时，供消费模块与像素数据对齐
//    O_frame_end  ：**帧末（场消隐期）脉冲** —— 与 I_vsync 上升沿**严格同拍**。
//                   此刻上一帧所有像素都已流过，各模块可安全启动"帧末结算"
//                   （清累加器 / 查等价表 / 算质心）。★ 我方 D2–D4 链用这个。
//    O_frame_done ：**帧首有效像素脉冲** —— 与帧首 de=1 那一拍**严格同拍**
//                   （此刻 O_pix_x/O_pix_y = 0），与冻结表 v1.4 口径一致。
//                   ⚠ 2026-10-11 更正：以前把它做成 vsync 上升沿，是**错的**。
//                    本工程真实 S_frame_done 来自 detect_color_mask.O_frame_done
//                    （= 内部 d3_user[1]，即对齐 user），落在帧首有效像素，
//                    与 vsync 上升沿差**整个场消隐期**（720p60 实测 39601 拍）。
//                    「归零用 vsync、锁存用 frame_done」——分工不同，不可混用。
//                   ⚠ 2026-10-14 修正：两信号此前是**寄存输出**（比源事件晚 1 拍，
//                    `O_frame_done` 落在 O_pix_x=1 那拍），已改为**组合输出**保严格同拍。
//                    实测脚本 tb/tb_pix_coord_phase.v，结论见文件尾。
//
//  ★为什么不用 I_video_user 清零（这是本模块与 hdmi_mixer 现有写法的关键差别）
//    hdmi_mixer.v 第 327~345 行的计数器写成「user 优先清零 + last 时行末翻行」。
//    sim/vtc_coord_probe.py 的位级审计证明这套写法有两个副作用：
//      · 首行（r=0）整行 S_x 比真实列号小 1 —— user 在首像素把本该的 +1 吃掉了；
//      · 帧首像素 (0,0) 采样到上一帧行末残留的 S_y=720。
//    两处都只影响 y=0 一行（被 160x160 Logo 覆盖，所以上板看不出来），
//    但画框一旦覆盖 y=0 就会错位。
//    本模块改用**场消隐期归零**（vsync 上升沿，此时 de 恒为 0）：
//      · 消隐期就把 x/y 清 0，下一帧首像素进来时坐标已经是 (0,0)，
//        不存在"首像素才清零"的滞后 ⇒ 无任何已知偏移。
//
//  位级验证：由 sim/vtc_coord_probe.py 的 Q1/Q1b 结论支撑
//    （I_last 与 I_de 严格同拍：实测 last=1 且 de=1 共 2160 拍 = 3 帧 x 720 行；
//      last=1 但 de=0 共 0 拍）
//=====================================================================
module pix_coord_gen #(
    parameter integer H_ACTIVE = 1280,      // 行有效像素数
    parameter integer V_ACTIVE = 720        // 帧有效行数
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_de,
    input  wire        I_last,
    input  wire        I_vsync,

    output reg  [10:0] O_pix_x,
    output reg  [9:0]  O_pix_y,

    // ---- 配套流水延时（与像素数据对齐用）----
    output reg  [10:0] O_pix_x_1d,
    output reg  [10:0] O_pix_x_2d,
    output reg  [9:0]  O_pix_y_1d,
    output reg  [9:0]  O_pix_y_2d,

    // ★ 帧末（场消隐期）脉冲 —— 与 I_vsync 上升沿**严格同拍**（组合输出）
    output wire        O_frame_end,
    // ★ 帧首有效像素脉冲 —— 与帧首 I_de=1 **严格同拍**（组合输出）
    output wire        O_frame_done
);

    //==================================================================
    // 1. vsync 上升沿检测（= 上一帧结束，本帧消隐期）
    //==================================================================
    reg  S_vsync_1d;
    wire W_vsync_rise = I_vsync & (~S_vsync_1d);

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_vsync_1d <= 1'b0;
        else
            S_vsync_1d <= I_vsync;
    end

    //==================================================================
    // 2. 像素坐标计数器
    //    · 消隐期（vsync 上升沿）整体归零 —— 避开首行 / 帧首偏移
    //    · 行内 de 时 +1；行末拍把 x 归零、y 翻行
    //==================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_pix_x <= 11'd0;
            O_pix_y <= 10'd0;
        end
        else if(W_vsync_rise) begin
            // 场消隐期归零。此时 de 必为 0（vsync 有效区与 de 有效区不重叠），
            // 所以与下面的 de 分支不会冲突。
            O_pix_x <= 11'd0;
            O_pix_y <= 10'd0;
        end
        else if(I_de) begin
            if(I_last) begin
                // 行末像素：本拍 x 已是 1279；拍末归零，下一拍即下一行首像素
                O_pix_x <= 11'd0;
                // y 翻行；末尾的 V_ACTIVE 回绕只是「万一没有 vsync」的安全兜底
                O_pix_y <= (O_pix_y == (V_ACTIVE - 1)) ? 10'd0 : (O_pix_y + 10'd1);
            end
            else begin
                O_pix_x <= (O_pix_x == (H_ACTIVE - 1)) ? 11'd0 : (O_pix_x + 11'd1);
            end
        end
    end

    //==================================================================
    // 3. 配套流水延时
    //    消费模块的像素数据通常比坐标晚 2 拍（例：hdmi_mixer 的 S_video_data_2d
    //    与 S_x_2d / S_y_2d 同款），直接用 _2d 即可对齐，不必各自再写延时链。
    //==================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_pix_x_1d <= 11'd0;
            O_pix_x_2d <= 11'd0;
            O_pix_y_1d <= 10'd0;
            O_pix_y_2d <= 10'd0;
        end
        else begin
            O_pix_x_1d <= O_pix_x;
            O_pix_x_2d <= O_pix_x_1d;
            O_pix_y_1d <= O_pix_y;
            O_pix_y_2d <= O_pix_y_1d;
        end
    end

    //==================================================================
    // 4. 两个帧标志脉冲（各 1 拍宽）★ 2026-10-14 相位对齐修正
    //    · O_frame_end ：与 I_vsync 上升沿**严格同拍**。
    //                    此刻上一帧所有像素都已流过，可安全启动"帧末结算"
    //                    （清累加器 / 查等价表 / 算质心）。
    //    · O_frame_done：与**帧首有效像素**（本帧第一个 I_de=1，此刻 O_pix_x=0）
    //                    **严格同拍**，与冻结表 v1.4「落于帧首有效像素」一致。
    //
    //    ⚠ 2026-10-14 修正（实测 `tb/tb_pix_coord_phase.v`）：
    //      此前两信号都写成**寄存输出**（`O_frame_end <= W_vsync_rise;` /
    //      `O_frame_done <= I_de && !S_in_frame;`），比源事件**晚 1 拍** ——
    //      实测 `O_frame_end` 比 I_vsync 上升沿晚 1 拍；`O_frame_done` 拉高那一拍
    //      `O_pix_x` 已经是 **1**（不是帧首的 0）⇒ 与冻结表口径不符。
    //      改为**组合输出**：源信号（I_vsync/I_de 经打拍、S_in_frame 为寄存器）
    //      均为寄存器输出，组合 AND 无毛刺，且与源事件严格同拍。
    //    实测（修正后）：fe-vs=0、fd-df=0、fd 拍 O_pix_x=0 O_pix_y=0。
    //
    //    ⚠ 分工提醒：坐标**归零**用 vsync 上升沿（§2 计数器），**结算/锁存**
    //      用本脉冲；「归零用 vsync、锁存用 frame_done」不可混用。
    //==================================================================
    assign O_frame_end = W_vsync_rise;

    // 帧首有效像素：vsync 之后第一次 I_de=1（保持到下次 vsync 上升沿才清）
    reg S_in_frame;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)          S_in_frame <= 1'b0;
        else if(W_vsync_rise) S_in_frame <= 1'b0;   // 场消隐期：清"本帧已开始"标志
        else if(I_de)         S_in_frame <= 1'b1;   // 帧内见过 de 后置位
    end

    // 与帧首 de=1 严格同拍（此刻计数器已归零 ⇒ O_pix_x/O_pix_y = 0）
    assign O_frame_done = I_de & (~S_in_frame);

endmodule
