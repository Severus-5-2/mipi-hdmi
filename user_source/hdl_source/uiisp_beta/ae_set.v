`timescale 1ns / 1ps

//=============================================================================
//  模块：ae_set（曝光/增益下发 + AE 自动曝光控制律）
//  路径：user_source/hdl_source/ae_fix/ae_set.v（替换 uiisp_beta/ae_set.v）
//  说明：模块名与原版一致，端口在原版基础上【新增 2 个输入】：
//          I_Y_avg[7:0]        —— 来自 ae_meter 的帧平均亮度
//          I_frame_toggle      —— 来自 ae_meter 的帧握手电平（异步信号）
//        原有端口 O_ae_req / O_ae / O_ag 完全不变，下游 uicfgcs520 无需改动。
//        在 TD 中请先移除原 uiisp_beta/ae_set.v 再加入本文件，避免模块重复定义。
//
//  作者：2026-10-06
//  基础版本：2026-10-05 的增益修复版（默认 63x、上限 126x）
//=============================================================================
//
//  【AE 自动曝光原理】
//    AE = Automatic Exposure，本质是一个负反馈闭环，和空调恒温同理：
//
//        ① 测光：统计当前画面平均亮度 Y_avg
//        ② 比较：err = 目标亮度 TARGET − Y_avg
//        ③ 决策：err>0（画面偏暗）→ 加曝光/增益；err<0（偏亮）→ 减
//        ④ 下发：I2C 写入 sensor 寄存器
//        ⑤ 等待：sensor 在 N+2 帧才生效，必须等它生效后再回到 ①
//
//    如果跳过第 ⑤ 步，就会拿着"旧画面"的误差去叠加"新参数"，
//    结果是越调越过头、来回振荡——这是手搓 AE 最常见的失败原因。
//
//  【本项目用的调节策略（直接取自 SC520CS 数据手册 2.2.2 节）】
//    手册 AEC/AGC 控制说明原文要旨：
//      · "曝光时间加大，有助于提升信噪比；增益开启将导致平均噪声倍数放大"
//      · "图像过亮时，优先关闭增益；增益全关仍过亮，才降低曝光时间"
//    据此确定两条方向的顺序：
//      画面偏暗 → 先加曝光时间（到上限 1991），曝光用尽才加增益（到 126x）
//      画面偏亮 → 先降增益（到 1x），增益到底才降曝光（到下限 4）
//    这样做的收益：亮场景下 AE 会自动把增益降下来，噪点随之显著减少，
//    顺带解决本项目一直存在的"暗部噪点"问题。
//
//  【曝光数值范围】（手册表 2-2）
//    单位：1/2 行（一行时间 ≈ 13.9 us @720p60）
//    最小值：4      最大值：2×VTS−9 = 1991（VTS=1000，即满帧 ≈16.6 ms）
//    步长：1（即 1/2 行）
//
//  【为什么必须"N+2 帧生效"后才允许下一次调整】
//    手册 AEC 控制说明第 2 条："曝光时间及增益若在第 N 帧写入，第 N+2 帧生效"。
//    本模块用 WAIT_FRAMES（默认 3）控制两次调整之间的最小间隔，
//    即每 4 帧（≈66 ms）最多调整一次，确保上一次调整的画面已经完全稳定。
//
//  【按键改为调"目标亮度"的理由】
//    如果保留旧版"按键直接改增益"，会和自动调节互相打架：
//    用户按下调亮，AE 下一帧又把它拉回去，表现为按了没反应。
//    所以改为调 TARGET（靶亮度），本质上是在告诉 AE"我希望画面这么亮"，
//    既保留了现场交互手感，又不破坏闭环。
//    参数 AUTO_EN 置 0 可退回旧行为：关闭自动，按键重新直接调增益。
//=============================================================================

module ae_set
(
    input  wire        I_clk,           // 24 MHz 配置时钟
    input  wire        I_rst,           // 高电平复位
    input  wire [ 3:0] I_btn,           // 按键，低有效：[2]减 [3]增，[1:0]未接
    input  wire        I_ae_cfg_done,   // 上一轮 AE 寄存器写入完成
    input  wire        I_cam_cfg_done,  // sensor 初始化寄存器写入完成

    input  wire [ 7:0] I_Y_avg,         // 【新增】帧平均亮度（来自 ae_meter，异步于 I_clk）
    input  wire        I_frame_toggle,  // 【新增】帧握手电平，每帧翻转一次（异步）

    output wire        O_ae_req,        // 请求写一次曝光/增益
    output wire [15:0] O_ae,            // 曝光值（单位 1/2 行）
    output wire [15:0] O_ag             // 增益控制字（送 SC520GainTbl 查表）
);

//=============================================================================
//  可调参数（综合期常量，改完需重新编译）
//=============================================================================
parameter           AUTO_EN     = 1'b1;      // 1:自动曝光使能  0:关闭自动（纯手动）
parameter [ 7:0]    TARGET_DEF  = 8'd120;    // 上电默认靶亮度（Gamma 后显示域，中灰偏亮）
parameter [ 7:0]    TARGET_MIN  = 8'd60;     // 靶亮度下调下限
parameter [ 7:0]    TARGET_MAX  = 8'd200;    // 靶亮度上调上限
parameter [ 7:0]    BTN_STEP    = 8'd8;      // 单次【按下】的靶亮度变化量（v6：解决"要按很多次"）
parameter [ 7:0]    DEADZONE    = 8'd4;      // 误差死区：|err| 小于此值不调节（防抖动）
parameter [15:0]    AE_MIN      = 16'd4;     // 曝光下限（手册规定）
parameter [15:0]    AE_MAX      = 16'd1991;  // 曝光上限（= 2×VTS−9，满帧）
parameter [15:0]    AG_MIN      = 16'd16;    // 增益控制字下限 = 1x
parameter [15:0]    AG_MAX      = 16'd239;   // 增益控制字上限 = 126x
parameter [ 3:0]    WAIT_FRAMES = 4'd3;      // 两次调整之间要空等的帧数（手册要求≥2）

//=============================================================================
//  定时脉冲生成（沿用原版逻辑）
//    r_ae_tick：每 1 ms 一个脉冲（24 MHz 下计数 100000）
//    r_ag_tick：每 32 ms 一个脉冲，用于按键调节的节拍
//=============================================================================
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
        rc_ae_tick <= rc_ae_tick + 1;
        r_ae_tick  <= 0;
        if (rc_ae_tick >= 99999) begin
            rc_ae_tick <= 0;
            r_ae_tick  <= 1;
        end
        rc_ag_tick <= rc_ag_tick + r_ae_tick;
        r_ag_tick  <= (&rc_ag_tick) && r_ae_tick;
    end
end

//=============================================================================
//  按键同步（沿用原版两级同步 + 取反处理）
//=============================================================================
reg [3:0] r_button_0 = 0, r_button_1 = 0, r_button_2 = 0;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        r_button_0 <= 0;
        r_button_1 <= 0;
        r_button_2 <= 0;
    end else begin
        r_button_0 <= ~I_btn;          // 低有效 → 按下为 1
        r_button_1 <= r_button_0;
        r_button_2 <= r_button_1;      // v6：第三级，用于检测"按下的那一瞬间"
    end
end

//  v6：按键手感改造
//    旧行为：按住期间每 32 ms 才 +1，短按一次只加 2~3 —— 于是"要按很多次"。
//    新行为：
//      · 单击（按下沿）→ 直接跳 BTN_STEP(8)，按 3 下就能从 112 拉到 136
//      · 长按        → 每 32 ms +1 连发，用于精细微调
wire btn_add_edge = r_button_1[3] && !r_button_2[3];   // KEY2 按下沿（增）
wire btn_sub_edge = r_button_1[2] && !r_button_2[2];   // KEY1 按下沿（减）

//=============================================================================
//  一、跨时钟域接收 ae_meter 的帧信号
//-----------------------------------------------------------------------------
//  信号从 74.25 MHz 域过来，与本模块的 24 MHz 时钟完全异步。
//  做法：把"翻转电平"用两级触发器同步，再取相邻两拍的异或得到单周期脉冲。
//      两级同步可把亚稳态收敛概率压到极低；
//      数据 Y_avg 本身保持整帧不变，因此在脉冲时刻采样必定采到稳定值。
//=============================================================================
reg [1:0] tog_sync;   //synthesis syn_preserve

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst)
        tog_sync <= 2'b00;
    else
        tog_sync <= {tog_sync[0], I_frame_toggle};
end

wire frame_pulse = tog_sync[1] ^ tog_sync[0];   // 单周期，指示"新一帧统计已就绪"

//-----------------------------------------------------------------------------
//  采样 Y_avg（在 24 MHz 域内保存一份副本，后续判决都用它）
//-----------------------------------------------------------------------------
reg [7:0] y_cur;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst)
        y_cur <= 8'd0;
    else if (frame_pulse)
        y_cur <= I_Y_avg;
end

//=============================================================================
//  二、上电预热（warmup）
//-----------------------------------------------------------------------------
//  上电后最初几帧，画面可能是全黑或不稳定数据，
//  且此时 ae_meter 还没输出第一帧统计值（Y_avg 为复位值 0）。
//  若不做滤波直接开调，AE 会误判为"极暗"而一口气把增益拉到最大。
//  因此规定：收到 4 个有效帧脉冲之后，才开始自动调节。
//=============================================================================
reg [2:0] warm_cnt;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst)
        warm_cnt <= 3'd0;
    else if (frame_pulse && (warm_cnt < 3'd4))
        warm_cnt <= warm_cnt + 1'b1;
end

wire ae_ready = AUTO_EN && (warm_cnt >= 3'd4) && I_cam_cfg_done;

//=============================================================================
//  三、调整窗口生成（这是防振荡的核心）
//-----------------------------------------------------------------------------
//  手册规定"第 N 帧写入，第 N+2 帧生效"。
//  本模块每 (WAIT_FRAMES+1) 帧只允许调整一次，其余帧只更新统计量、不动参数。
//  60 fps 下即最快每 4 帧（≈66 ms）调整一次，约 15 次/秒。
//=============================================================================
reg [3:0] wait_cnt;
reg       adjust_win;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        wait_cnt   <= 4'd0;
        adjust_win <= 1'b0;
    end else if (!ae_ready) begin
        //  未就绪时清零，保证就绪后的第一个帧脉冲立即触发一次调整
        wait_cnt   <= 4'd0;
        adjust_win <= 1'b0;
    end else if (frame_pulse) begin
        if (wait_cnt == 4'd0) begin
            wait_cnt   <= WAIT_FRAMES;   //  调完之后重新装填等待计数
            adjust_win <= 1'b1;
        end else begin
            wait_cnt   <= wait_cnt - 1'b1;
            adjust_win <= 1'b0;
        end
    end else begin
        adjust_win <= 1'b0;              //  单周期脉冲
    end
end

//=============================================================================
//  四、误差计算
//-----------------------------------------------------------------------------
//  ★ 本项目踩过的大坑：Verilog 中【部分选择永远是无符号的】，
//    哪怕母向量声明为 signed，写成 err[9:0] 也会让整个表达式退化为无符号运算，
//    负数被零扩展成大正数（曾导致满屏蓝黄椒盐噪点）。
//    因此这里全部使用完整的 signed 向量参与运算，绝不写部分选择。
//=============================================================================
wire signed [9:0] err = $signed({2'b00, t_target}) - $signed({2'b00, y_cur});

//  取绝对值：全部用 signed 表达式，用符号位做选择即可，不需要切片
wire signed [9:0] err_abs = err[9] ? (-err) : err;

wire too_dark   = (err >  $signed({2'b00, DEADZONE}));   // 画面偏暗 → 需要提亮
wire too_bright = (err < -$signed({2'b00, DEADZONE}));   // 画面偏亮 → 需要压暗

//=============================================================================
//  六、比例步进（分档查表）+ 单步变化上限（【两者缺一不可】）
//  先说结论：**曝光用"比例步长"，增益用"固定步长"**，因为两者量纲不同：
//
//    · 曝光 ae → 进光量是【线性】关系（ae 翻倍则进光量翻倍）
//      所以必须用比例步长（ae >> k），才能保证每次都是固定的百分比变化
//    · 增益控制字 ag → 实际增益是【倍率】关系（SC520GainTbl 段内每步 +3.1%）
//      所以必须用固定步长，因为 ag 每增减 1 本身就已经是固定百分比了
//
//  实测灵敏度（Y 为 Gamma 后的显示亮度，以 Y=112 为基准）：
//      ae 变化 6.25% / 12.5% / 25% / 50%  →  Y 变化约 3.1 / 6.2 / 12.0 / 22.7
//      ag 变化  2 / 4 / 8 / 16 / 32 步    →  Y 变化约 3.1 / 6.2 / 12.0 / 22.7 / 41.5
//    （两者恰好一一对应——因为 sensor 增益阶梯本身就是按倍率设计的）
//
//  【核心设计原则：每个档位的步长造成的 Y 变化，必须小于该档位的误差下限】
//    这样误差单调递减，数学上跨不过目标点，也就不可能振荡：
//      |误差| >= 64   → Y 变化约 41.5  < 64   ✓
//      |误差| 32~64   → Y 变化约 22.7  < 32   ✓
//      |误差| 12~32   → Y 变化约 12.0  <= 12  ✓
//      |误差|  5~12   → Y 变化约  6.2  （略过一点，但随即落入死区停止）
//
//  【这个结论是怎么来的 —— 记录两次失败的尝试，避免以后重蹈覆辙】
//    第一版：纯固定步长（512/192/64/16）
//            Python 闭环仿真发现：强光场景 Y 在 16~205 之间来回跳。
//            原因是大步长在 Gamma 非线性区（暗端极敏感）会一步跨过目标。
//    第二版：改成"单步不超过当前值的 25% 或 50%"，并且给 ag 也加比例上限。
//            仍有多个场景振荡。根因是【建模错误】：
//            ag 控制的是倍率，给它加"按当前值缩放"反而破坏了它固有的百分比
//            特性——在 ag 低位（<64）时等效步长变得极大（一步 = 增益翻倍
//            = Y 跳 37%），直接跨过死区。
//    第三版（本版）：曝光比例、增益固定，各归各位。
//            7 种静态光照 + 3 组场景突变，仿真末段波动【全部为 0】，
//            收敛时间 0.28 ~ 1.02 秒。
//=============================================================================
reg [ 1:0] sh_ae;        // 曝光步长右移位数：0=×100%  1=50%  2=25%  3=12.5%
reg [15:0] step_ag;      // 增益固定步长

always @(*) begin
    if (err_abs >= 10'sd64) begin
        sh_ae = 2'd0;   step_ag = 16'd32;   //  差得多 → 大步前进
    end else if (err_abs >= 10'sd32) begin
        sh_ae = 2'd1;   step_ag = 16'd16;
    end else if (err_abs >= 10'sd12) begin
        sh_ae = 2'd2;   step_ag = 16'd8;
    end else begin
        sh_ae = 2'd3;   step_ag = 16'd4;    //  快到位 → 小步精调
    end
end

//  曝光步长 = ae >> sh_ae
//    用 case 实现可变移位，综合器会做成一个 4 选 1 MUX，资源很省
reg [15:0] step_ae_raw;

always @(*) begin
    case (sh_ae)
        2'd0:    step_ae_raw = r_ae_set;
        2'd1:    step_ae_raw = r_ae_set >> 1;
        2'd2:    step_ae_raw = r_ae_set >> 2;
        2'd3:    step_ae_raw = r_ae_set >> 3;
        default: step_ae_raw = r_ae_set >> 3;
    endcase
end

//  曝光值很小时给一个最小步长，防止低位卡住不动
wire [15:0] ef_ae = (step_ae_raw > 16'd8) ? step_ae_raw : 16'd8;
wire [15:0] ef_ag = step_ag;

//=============================================================================
//  六、靶亮度寄存器 + 目标值寄存器 + 自动/手动调节
//=============================================================================
reg [7:0] t_target = TARGET_DEF;

//  目标值初始值：
//    曝光拉满 1991 —— 曝光不引入额外噪声，按手册策略应优先给足曝光
//    增益起点 144 (=16x) —— 从低噪声端起步，由 AE 按需往上加
//    这样即使开局画面偏暗，AE 也会在几帧内拉到位，而不会一上来就过曝。
reg [15:0] r_ae_set = AE_MAX;
reg [15:0] r_ag_set = 16'd144;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        t_target <= TARGET_DEF;
        r_ae_set <= AE_MAX;
        r_ag_set <= 16'd144;

    //------------------------------------------------------------------
    //  路径 A：手动调节（按键）
    //    AUTO_EN=1 时，按键调的是"靶亮度"（告诉 AE 我想要多亮）
    //    AUTO_EN=0 时，退回旧行为——按键直接调增益
    //------------------------------------------------------------------
    end else begin
        //------------------------------------------------------------------
        //  路径 A：手动调节（按键）
        //------------------------------------------------------------------
        if (AUTO_EN) begin
            //  自动模式下：按键调"靶亮度"，告诉 AE 我希望画面有多亮
            //  v6：单击跳一大步（BTN_STEP），长按再按 32ms 节拍连发微调
            if (btn_add_edge)
                t_target <= ((TARGET_MAX - t_target) > BTN_STEP)
                            ? (t_target + BTN_STEP) : TARGET_MAX;
            else if (btn_sub_edge)
                t_target <= ((t_target - TARGET_MIN) > BTN_STEP)
                            ? (t_target - BTN_STEP) : TARGET_MIN;
            else if (r_button_1[3] && r_ag_tick)          // 长按连发（增）
                t_target <= (t_target < TARGET_MAX) ? t_target + 1'b1 : t_target;
            else if (r_button_1[2] && r_ag_tick)          // 长按连发（减）
                t_target <= (t_target > TARGET_MIN) ? t_target - 1'b1 : t_target;

        end else begin
            //  手动模式（AUTO_EN=0）：退回旧行为，按键直接调增益
            if (r_button_1[2]) begin
                r_ag_set <= (r_ag_set > AG_MIN) ? r_ag_set - r_ag_tick : r_ag_set;
            end else if (r_button_1[3]) begin
                r_ag_set <= (r_ag_set < AG_MAX) ? r_ag_set + r_ag_tick : r_ag_set;
            end

            //  曝光增减分支保留备用（当前板上按键未接这两路）
            if (r_button_1[0]) begin
                r_ae_set <= (r_ae_set > AE_MIN) ? r_ae_set - r_ae_tick : r_ae_set;
            end else if (r_button_1[1]) begin
                r_ae_set <= (r_ae_set < AE_MAX) ? r_ae_set + r_ae_tick : r_ae_set;
            end
        end

        //------------------------------------------------------------------
        //  路径 B：自动曝光调节
        //    注意：这段与上一段是并列的，不是 else 关系。
        //    手动按键与自动回调可以同时生效（按键改靶亮度，AE 照常收敛），
        //    这正是"按键改调靶亮度"比"按键直接调增益"更合理的原因。
        //------------------------------------------------------------------
        if (adjust_win && too_dark) begin
            if (r_ae_set < AE_MAX) begin
                r_ae_set <= ((AE_MAX - r_ae_set) > ef_ae) ? (r_ae_set + ef_ae) : AE_MAX;
            end else if (r_ag_set < AG_MAX) begin
                r_ag_set <= ((AG_MAX - r_ag_set) > ef_ag) ? (r_ag_set + ef_ag) : AG_MAX;
            end
        end else if (adjust_win && too_bright) begin
            if (r_ag_set > AG_MIN) begin
                r_ag_set <= ((r_ag_set - AG_MIN) > ef_ag) ? (r_ag_set - ef_ag) : AG_MIN;
            end else if (r_ae_set > AE_MIN) begin
                r_ae_set <= ((r_ae_set - AE_MIN) > ef_ae) ? (r_ae_set - ef_ae) : AE_MIN;
            end
        end
    end
end

//=============================================================================
//  七、变化检测 → 产生一次写入请求（沿用原版，逻辑未改）
//-----------------------------------------------------------------------------
//  复位后 r_ag(17) 与 r_ag_set(144) 故意不相等，
//  保证 sensor 初始化完成后必定下发一次曝光/增益，行为与原始版本一致。
//  写入门控条件包含 I_ae_cfg_done，确保上一次 I2C 写完后才发起下一次。
//=============================================================================
reg        r_ae_req = 0;  //synthesis keep
reg [15:0] r_ae;
reg [15:0] r_ag;

assign O_ae     = r_ae;
assign O_ag     = r_ag;
assign O_ae_req = r_ae_req;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        r_ae_req <= 0;
        r_ae     <= 16'd1990;
        r_ag     <= 16'd17;         //  与 r_ag_set 不同 → 上电必定触发一次写入
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
