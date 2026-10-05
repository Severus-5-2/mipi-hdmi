`timescale 1ns / 1ps

//=============================================================================
//  AE 控制（2026-10-05 修改版）
//  路径：user_source/hdl_source/sc520_ae_fix/ae_set.v
//  说明：模块名与端口和原 uiisp_beta/ae_set.v 完全一致，可直接整体替换。
//        在 TD 工程中请先移除原 ae_set.v，再加入本文件（避免模块重复定义）。
//
//  【本版改了什么】（只改了 3 个数字，接口完全不变）
//    1) 默认增益控制字 r_ag_set：144 → 207
//         144 = 16x（旧默认，也就是你现在在板上看到的亮度）
//         207 = 32x 模拟 × 1.969x 数字细增益 ≈ 63x，比原来亮 4 倍
//    2) 增益上限 r_ag_set 边界：176 → 239
//         176 = 32x（旧表天花板，因为旧 SC520GainTbl 从不使用数字粗增益）
//         239 = 126x（配合新版 SC520GainTbl，达到数据手册规定的系统上限）
//    3) 上电强制写入一次的初值保持"故意不一致"（r_ag 复位 17 ≠ r_ag_set 207），
//       这样上电必定触发一次曝光/增益寄存器写入，行为与原版一致。
//
//  【按键怎么用】（重要——之前一直没说清楚）
//    顶层例化：.I_btn({I_button, 2'b11})
//      I_btn[3] = I_button[1]  → 增益 +
//      I_btn[2] = I_button[0]  → 增益 -
//      I_btn[1] = 1'b1         → 曝光 +（恒定无效，未接按键）
//      I_btn[0] = 1'b1         → 曝光 -（恒定无效，未接按键）
//    即：板上的两个按键本来就是调增益的，曝光已满帧 1991，无需再调。
//    按键为低有效（按下 = 0）。增益按键每 32 ms 变化 1 级，
//    从默认 207(63x) 加到上限 239(126x) 约需长按 1 秒，
//    减到 176(32x) 约需 1 秒，方便上板现场选亮度。
//
//  【曝光为什么不动】
//    SC520CS 720p60 模式 VTS=1000，曝光单位是 1/2 行，上限为 2×VTS-9=1991。
//    1991 已经是满帧曝光（约 16.6 ms），没有任何上调空间，
//    室内场景只能靠增益补亮，这也是本次把增益阶梯放开到 126x 的原因。
//=============================================================================

module ae_set
(
    input  wire        I_clk,           // 24 MHz 配置时钟
    input  wire        I_rst,           // 高电平复位
    input  wire [ 3:0] I_btn,           // 按键，低有效：[2]增益- [3]增益+，[1:0]曝光（未接）
    input  wire        I_ae_cfg_done,   // 上一轮 AE 寄存器写入完成
    input  wire        I_cam_cfg_done,  // sensor 初始化寄存器写入完成
    output wire        O_ae_req,        // 请求写一次曝光/增益
    output wire [15:0] O_ae,            // 曝光值（单位 1/2 行）
    output wire [15:0] O_ag             // 增益控制字（送 SC520GainTbl 查表）
);

//-----------------------------------------------------------------------------
//  定时脉冲生成
//    rc_ae_tick：每计数 100000 个时钟产生一个 r_ae_tick（约 1 ms @24 MHz）
//    rc_ag_tick：由 r_ae_tick 累加，溢出时产生 r_ag_tick（约 32 ms）
//-----------------------------------------------------------------------------
reg [19:0] rc_ae_tick = 0;
reg        r_ae_tick  = 0;
reg [ 4:0] rc_ag_tick = 0;
reg        r_ag_tick  = 0;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        rc_ae_tick <= 0;
        r_ae_tick  <= 0;
        rc_ag_tick <= 0;
        r_ag_tick  <= 0;
    end else begin
        //  1 ms 节拍，供曝光增减使用
        rc_ae_tick <= rc_ae_tick + 1;
        r_ae_tick  <= 0;
        if (rc_ae_tick >= 99999) begin
            rc_ae_tick <= 0;
            r_ae_tick  <= 1;
        end
        //  32 ms 节拍，供增益增减使用（比曝光慢，便于精细调节）
        rc_ag_tick <= rc_ag_tick + r_ae_tick;
        r_ag_tick  <= (&rc_ag_tick) && r_ae_tick;
    end
end

//-----------------------------------------------------------------------------
//  目标值寄存器 + 按键处理
//-----------------------------------------------------------------------------
reg [3:0] r_button_0 = 0, r_button_1 = 0;

// 曝光目标值：1991 = 满帧曝光（2×VTS-9，VTS=1000），保持最大不动。
// 增益目标值：207 ≈ 63x（默认）；可用按键在 16(1x) ~ 239(126x) 之间现场调节。
reg [15:0] r_ae_set = 1991, r_ag_set = 207;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        r_button_0 <= 0;
        r_button_1 <= 0;
        r_ae_set   <= 1991;
        r_ag_set   <= 207;      // 默认 ≈63x（原先为 144 = 16x）
    end else begin
        // 按键同步两级 + 取反（低有效 → 按下为 1）
        r_button_0 <= ~I_btn;
        r_button_1 <= r_button_0;

        // ---- 曝光增减（当前未接按键，保留逻辑以便扩展）----
        if (r_button_1[0]) begin
            r_ae_set <= (r_ae_set > 2) ? r_ae_set - r_ae_tick : r_ae_set;
        end else if (r_button_1[1]) begin
            r_ae_set <= (r_ae_set < 1991) ? r_ae_set + r_ae_tick : r_ae_set;
        end

        // ---- 增益增减（板上两个按键实际接的就是这两路）----
        if (r_button_1[2]) begin
            // 减增益，下限 16（=1x）
            r_ag_set <= (r_ag_set > 16) ? r_ag_set - r_ag_tick : r_ag_set;
        end else if (r_button_1[3]) begin
            // 增增益，上限 239（=126x，旧版只有 176=32x）
            r_ag_set <= (r_ag_set < 239) ? r_ag_set + r_ag_tick : r_ag_set;
        end
    end
end

//-----------------------------------------------------------------------------
//  变化检测 → 产生一次写入请求
//    注意：复位后 r_ag(17) 与 r_ag_set(207) 故意不相等，
//    保证 sensor 初始化完成后必定下发一次曝光/增益，行为与原始版本一致。
//-----------------------------------------------------------------------------
reg        r_ae_req = 0;  // synthesis keep
reg [15:0] r_ae;
reg [15:0] r_ag;

assign O_ae     = r_ae;
assign O_ag     = r_ag;
assign O_ae_req = r_ae_req;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        r_ae_req <= 0;
        r_ae     <= 1990;
        r_ag     <= 17;         // 与 r_ag_set 不同 → 上电必定触发一次写入
    end else if (r_ae_req) begin
        r_ae_req <= 0;
    end else if (((r_ae != r_ae_set) || (r_ag != r_ag_set))
                 && (I_cam_cfg_done == 1'b1) && (I_ae_cfg_done == 1'b1)) begin
        r_ae     <= r_ae_set;
        r_ag     <= r_ag_set;
        r_ae_req <= 1;
    end
end

endmodule
