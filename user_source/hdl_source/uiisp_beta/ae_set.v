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
//    本模块用 WAIT_FRAMES（默认 5）控制两次调整之间的最小间隔，
//    即等待新参数经过相机与显示缓存生效后再调整，确保上一次调整的画面已经完全稳定。
//
//  2026-10-28: boot with full AE. K1/K2 directly trim gain and hold that gain
//  after release; exposure keeps following the automatic brightness feedback.
//  A sustained scene-brightness change returns gain to full AE. BOTH held~1s
//  explicitly restores full AE. The brightness target remains128, gain16..175.
//=============================================================================

module ae_set
(
    input  wire        I_clk,           // 24 MHz 配置时钟
    input  wire        I_rst,           // 高电平复位
    input  wire [ 3:0] I_btn,           // 按键，低有效：[2]减增益 [3]加增益，[1:0]未接
    input  wire        I_ae_cfg_done,   // 上一轮 AE 寄存器写入完成
    input  wire        I_cam_cfg_done,  // sensor 初始化寄存器写入完成

    input  wire [ 7:0] I_Y_avg,         // 【新增】帧平均亮度（来自 ae_meter，异步于 I_clk）
    input  wire [1:0]  I_highlight,    // Frame-held highlight guard from ae_meter
    input  wire        I_frame_toggle,  // 【新增】帧握手电平，每帧翻转一次（异步）

    output wire        O_ae_req,        // 请求写一次曝光/增益
    output wire [15:0] O_ae,            // 曝光值（单位 1/2 行）
    output wire [15:0] O_ag,            // 增益控制字（送 SC520GainTbl 查表）
    output reg [27:0]  O_quality_status // {manual gain, target BCD[10:0], exposure BCD4}
);

//=============================================================================
//  可调参数（综合期常量，改完需重新编译）
//=============================================================================
parameter           AUTO_EN     = 1'b1;      // 1:自动曝光使能  0:关闭反馈调节
parameter [ 7:0]    TARGET_DEF  = 8'd128;    // 上电默认靶亮度（Gamma 后显示域，中灰偏亮）
                                             // v6.3 A项：120→145。120 对应线性域约 18% 灰，
                                             // 显示域偏暗（用户"和手机差距大"）；145 约 30% 灰，
                                             // 接近相机 18% 中灰经 sRGB 显示后的观感亮度。
// K1/K2 trim the desired gain (click BTN_STEP, hold1). Target stays fixed.
parameter [ 7:0]    TARGET_MIN  = 8'd60;
parameter [ 7:0]    TARGET_MAX  = 8'd200;
parameter [ 7:0]    BTN_STEP    = 8'd16;
parameter [ 7:0]    DEADZONE    = 8'd6;      // 误差死区：|err| 小于此值不调节（防抖动）
parameter [15:0]    AE_MIN      = 16'd4;     // 曝光下限（手册规定）
parameter [15:0]    AE_MAX      = 16'd1991;  // 曝光上限（= 2×VTS−9，满帧）
parameter [15:0]    AG_MIN      = 16'd16;    // 增益控制字下限 = 1x
// 2026-10-24: quality ceiling 55x (199); old 207 is 63x.
parameter [15:0]    AG_QUALITY_MAX = 16'd175; // Fixed quality ceiling; keys do not raise it
parameter [15:0]    AG_MAX      = 16'd199;   // 增益控制字上限
                                             // ★v6.6：239(126x) → 207(63x)。
                                             //   原因：现场照片定量分析证明屏幕噪点是
                                             //   **共模亮度噪声且 ∝ 增益**（同一相机：明亮
                                             //   白墙 σ≈2，昏暗房间 σ≈13，6.5 倍）。暗场景
                                             //   下 AE 会把增益顶到 126x，噪声直接不可用。
                                             //   63x 是"亮度/噪声"的折中点：噪声减半。
                                             //   ⚠️ 想更亮就改回 239；想更干净可降到 199(53x)。
                                             //   注意：只有在 AE 被顶到上限时这一项才起作用；
                                             //   若画面平均亮度已到靶值，说明增益未顶满，改它无效。
parameter [ 3:0]    WAIT_FRAMES = 4'd5;      // 两次调整之间要空等的帧数（手册要求≥2）

//=============================================================================
//  定时脉冲生成（沿用原版逻辑）
//    r_ae_tick：每约4.167ms一个脉冲（24MHz下计数100000）
//    r_ag_tick：每约133ms一个脉冲，用于长按增益微调
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

// Debounce at the existing tick:100000/24MHz ~=4.167ms per sample.
// Two matching samples, then edge detect in the24MHz domain.
reg [1:0] key_s0=0,key_s1=0,key_stable=0,key_prev=0;
reg [7:0] chord_ticks=0;
reg chord_lock=0;
wire [15:0] gain_ceiling=AG_QUALITY_MAX;
always @(posedge I_clk or posedge I_rst) begin
    if(I_rst) begin
        key_s0<=0;key_s1<=0;key_stable<=0;key_prev<=0;
        chord_ticks<=0;chord_lock<=0;
    end else begin
        key_prev<=key_stable;
        if(r_ae_tick) begin
            key_s0<=r_button_1[3:2];key_s1<=key_s0;
            if(key_s0==key_s1) key_stable<=key_s1;
            if(key_stable==2'b11) begin
                chord_lock<=1;
                if(chord_ticks<240) chord_ticks<=chord_ticks+1'b1;
            end else begin
                chord_ticks<=0;
                if(key_stable==0) chord_lock<=0;
            end
        end
    end
end
wire btn_add_edge=key_stable[1] && !key_prev[1];
wire btn_sub_edge=key_stable[0] && !key_prev[0];
wire reset_quality=AUTO_EN && key_stable==2'b11 && r_ae_tick && chord_ticks==239;
wire key_single=key_stable==2'b01 || key_stable==2'b10;
// Start repeat only after~300ms held, so a short click is exactly16 codes
// regardless of the free-running repeat tick phase.
reg [6:0] hold_ticks=0;
always @(posedge I_clk or posedge I_rst) begin
    if(I_rst) hold_ticks<=0;
    else if(!key_single || chord_lock || btn_add_edge || btn_sub_edge) hold_ticks<=0;
    else if(r_ae_tick && hold_ticks<72) hold_ticks<=hold_ticks+1'b1;
end
wire hold_repeat=hold_ticks==72 && r_ag_tick;
wire key_adjust=!chord_lock && key_single && (btn_add_edge || btn_sub_edge || hold_repeat);
wire [15:0] gain_key_step=(btn_add_edge || btn_sub_edge)?{8'd0,BTN_STEP}:16'd1;

//=============================================================================
//  一、跨时钟域接收 ae_meter 的帧信号
//-----------------------------------------------------------------------------
//  信号从 74.25 MHz 域过来，与本模块的 24 MHz 时钟完全异步。
//  做法：把"翻转电平"用两级触发器同步，再取相邻两拍的异或得到单周期脉冲。
//      两级同步可把亚稳态收敛概率压到极低；
//      数据 Y_avg 本身保持整帧不变，因此在脉冲时刻采样必定采到稳定值。
//=============================================================================
reg [2:0] tog_sync;   //synthesis syn_preserve

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst)
        tog_sync <= 3'b000;
    else
        tog_sync <= {tog_sync[1:0], I_frame_toggle};
end

wire frame_pulse = tog_sync[2] ^ tog_sync[1];   // 单周期，指示"新一帧统计已就绪"

//-----------------------------------------------------------------------------
//  采样 Y_avg（在 24 MHz 域内保存一份副本，后续判决都用它）
//-----------------------------------------------------------------------------
reg [7:0] y_cur;
reg [1:0] highlight_cur;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin y_cur<=0;highlight_cur<=0;end
    else if (frame_pulse) begin y_cur<=I_Y_avg;highlight_cur<=I_highlight;end
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
//  Post-write wait5frames;next decision uses fresh display statistics.
//=============================================================================
reg [15:0] r_ae_set=AE_MAX;
reg [15:0] r_ag_set=16'd96;
reg        r_ae_req = 0;  //synthesis keep
reg [15:0] r_ae;
reg [15:0] r_ag;
// Never stack decisions while commands are pending; wait fresh frames after I2C.
wire command_busy=r_ae_req || !I_ae_cfg_done || r_ae!=r_ae_set || r_ag!=r_ag_set;
reg [3:0] wait_cnt;
reg       adjust_win;

always @(posedge I_clk or posedge I_rst) begin
    if(I_rst) begin wait_cnt<=WAIT_FRAMES;adjust_win<=0;end
    else begin
        adjust_win<=0;
        if(!ae_ready || command_busy) wait_cnt<=WAIT_FRAMES;
        else if(frame_pulse) begin
            if(wait_cnt!=0) wait_cnt<=wait_cnt-1'b1;
            else begin adjust_win<=1;wait_cnt<=WAIT_FRAMES;end
        end
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

wire too_dark   = !highlight_cur[0] && (err >  $signed({2'b00, DEADZONE}));   // 画面偏暗 → 需要提亮
wire too_bright = highlight_cur[1] || (err < -$signed({2'b00, DEADZONE}));   // 画面偏亮 → 需要压暗

// Bounded steps + direction confirmation. No gain-dependent image curve.
// The208..235 highlight band can HOLD instead of repeatedly brightening/dimming.
reg [2:0] sh_ae;
reg [15:0] step_ag;
reg [1:0] vote_direction,last_direction;
wire [1:0] desired_direction=too_bright?2'd2:too_dark?2'd1:2'd0;
wire reverse_step=last_direction!=0 && desired_direction!=0 && last_direction!=desired_direction;
always @* begin
    if(highlight_cur[1] && err>=-$signed({2'b00,DEADZONE})) begin
        sh_ae=3'd4;step_ag=16'd2;
    end else if(err_abs>=64) begin sh_ae=3'd1;step_ag=16'd8;end
    else if(err_abs>=32) begin sh_ae=3'd2;step_ag=16'd4;end
    else if(err_abs>=12) begin sh_ae=3'd3;step_ag=16'd2;end
    else begin sh_ae=3'd4;step_ag=16'd1;end
end
reg [15:0] step_ae_raw;
always @* begin
    if(reverse_step) step_ae_raw=r_ae_set>>5;
    else case(sh_ae)
        1:step_ae_raw=r_ae_set>>1;
        2:step_ae_raw=r_ae_set>>2;
        3:step_ae_raw=r_ae_set>>3;
        default:step_ae_raw=r_ae_set>>4;
    endcase
end
wire [15:0] ef_ae=(step_ae_raw!=0)?step_ae_raw:16'd1;
wire [15:0] ef_ag=reverse_step?16'd1:step_ag;

//=============================================================================
//  六、手动增益干预 + 持续自动曝光 + 场景变化交回全自动
//=============================================================================
wire [7:0] t_target=TARGET_DEF;
reg manual_gain=0;
reg manual_ref_valid=0;
reg manual_anchor_valid=0;
reg [7:0] manual_y_ref=0;
reg manual_highlight_ref=0;
reg [7:0] manual_expected_drift=0;
reg [1:0] manual_scene_votes=0;
reg [1:0] manual_rebase_votes=0;
wire [7:0] manual_y_delta=(I_Y_avg>=manual_y_ref)?
    I_Y_avg-manual_y_ref:manual_y_ref-I_Y_avg;
wire [8:0] manual_scene_limit={1'b0,manual_expected_drift}+9'd16;
wire manual_scene_different=({1'b0,manual_y_delta}>=manual_scene_limit) ||
    (manual_expected_drift==0 && I_highlight[1]!=manual_highlight_ref);
// Keep an established scene anchor across our exposure writes. A deliberately
// conservative2x linear-response allowance plus16 distinguishes a large scene
// jump from the known exposure step, without a multiplier or sensor model.
// Include the minimum one-code step at short exposures without allowing a
// full-brightness allowance that could hide a real scene change.
wire [2:0] manual_step_shift=reverse_step?3'd5:sh_ae;
wire [2:0] manual_min_step_shift=(r_ae<16'd8)?3'd2:
    (r_ae<16'd16)?3'd3:(r_ae<16'd32)?3'd4:3'd5;
wire [2:0] manual_drift_shift=(manual_step_shift<manual_min_step_shift)?
    manual_step_shift:manual_min_step_shift;
wire [8:0] manual_own_drift=
    (({1'b0,(manual_y_ref >> manual_drift_shift)} << 1)+9'd1);
wire [8:0] manual_drift_sum={1'b0,manual_expected_drift}+manual_own_drift;
wire manual_exposure_snapshot=manual_gain && manual_anchor_valid &&
    r_ae_set!=r_ae && !r_ae_req && I_ae_cfg_done && I_cam_cfg_done;
// The first three fresh post-write frames allow sensor/display latency to pass.
// WAIT_FRAMES=5: wait_cnt2 at the fourth frame allows reference capture BEFORE
// the next AE decision. This also makes progress while exposure still adapts.
localparam [3:0] MANUAL_REF_WAIT=(WAIT_FRAMES>=3)?WAIT_FRAMES-3:0;
wire manual_observe_ready=ae_ready && !command_busy && key_stable==0 && !chord_lock;
wire manual_scene_return=manual_gain && manual_ref_valid && manual_observe_ready &&
    frame_pulse && manual_scene_different && manual_scene_votes==2;
// Give an observed external change up to three frames to be confirmed before
// our own next exposure write can invalidate the old reference. Gain stays
// held during this brief vote; exposure otherwise follows the normal AE loop.
wire manual_scene_pending=manual_gain && manual_ref_valid &&
    (manual_scene_votes!=0 || manual_rebase_votes!=0);
always @(posedge I_clk or posedge I_rst) begin
    if(I_rst) begin
        manual_gain<=0;manual_ref_valid<=0;manual_anchor_valid<=0;manual_y_ref<=0;
        manual_highlight_ref<=0;manual_expected_drift<=0;manual_scene_votes<=0;manual_rebase_votes<=0;
    end else if(reset_quality || manual_scene_return) begin
        manual_gain<=0;manual_ref_valid<=0;manual_anchor_valid<=0;
        manual_expected_drift<=0;manual_scene_votes<=0;manual_rebase_votes<=0;
    end else if(key_adjust) begin
        // The user's new gain itself can change brightness substantially:
        // begin its first scene reference only after that new command settles.
        manual_gain<=1;manual_ref_valid<=0;manual_anchor_valid<=0;
        manual_expected_drift<=0;manual_scene_votes<=0;manual_rebase_votes<=0;
    end else if(!manual_gain || !manual_observe_ready) begin
        // Never mistake our pending I2C/exposure/gain change or held key for
        // a new scene. Shared sensor feedback and sender continue normally.
        manual_ref_valid<=0;manual_scene_votes<=0;manual_rebase_votes<=0;
        if(!manual_gain) begin manual_anchor_valid<=0;manual_expected_drift<=0;end
        else if(manual_exposure_snapshot)
            manual_expected_drift<=manual_drift_sum[8]?8'd255:manual_drift_sum[7:0];
    end else if(frame_pulse) begin
        if(!manual_ref_valid) begin
            if(wait_cnt<=MANUAL_REF_WAIT) begin
                manual_ref_valid<=1;
                if(manual_anchor_valid && manual_scene_different) begin
                    // Do not absorb an external jump during our write/settle
                    // interval into a new baseline. Confirm it against the
                    // preserved old anchor for three fresh frames instead.
                    manual_scene_votes<=1;
                end else begin
                    manual_y_ref<=I_Y_avg;manual_highlight_ref<=I_highlight[1];
                    manual_anchor_valid<=1;manual_expected_drift<=0;manual_scene_votes<=0;
                end
            end
        end else if(manual_scene_different) begin
            manual_scene_votes<=manual_scene_votes+1'b1;manual_rebase_votes<=0;
        end else begin
            manual_scene_votes<=0;
            if(manual_expected_drift!=0) begin
                // An aborted scene candidate must not leave a widened
                // threshold forever. Three normal frames restore the anchor
                // and its ordinary16-level sensitivity before the next write.
                if(manual_rebase_votes==2) begin
                    manual_y_ref<=I_Y_avg;manual_highlight_ref<=I_highlight[1];
                    manual_expected_drift<=0;manual_rebase_votes<=0;
                end else manual_rebase_votes<=manual_rebase_votes+1'b1;
            end else manual_rebase_votes<=0;
        end
    end
end
always @(posedge I_clk or posedge I_rst) begin
    if(I_rst) begin
        r_ae_set<=AE_MAX;r_ag_set<=16'd96;vote_direction<=0;last_direction<=0;
    end else if(reset_quality || manual_scene_return) begin
        // A new policy epoch must confirm feedback direction afresh.
        vote_direction<=0;last_direction<=0;
    end else if(key_adjust) begin
        // Keys own the desired gain even while the sender is busy. The
        // original sender holds its current snapshot and later coalesces it.
        if(key_stable[0]) r_ag_set<=((r_ag_set-AG_MIN)>gain_key_step)?r_ag_set-gain_key_step:AG_MIN;
        else r_ag_set<=((gain_ceiling-r_ag_set)>gain_key_step)?r_ag_set+gain_key_step:gain_ceiling;
        vote_direction<=0;last_direction<=0;
    end else if(r_ag_set>gain_ceiling) begin
        r_ag_set<=gain_ceiling;vote_direction<=0;last_direction<=2;
    end else if(adjust_win && !command_busy && !manual_scene_pending) begin
        vote_direction<=desired_direction;
        // First changed direction waits for confirmation. First reversal is fine.
        if(desired_direction!=0 && vote_direction==desired_direction) begin
            if(desired_direction==2) begin
                if(!manual_gain && r_ag_set>AG_MIN) begin
                    r_ag_set<=((r_ag_set-AG_MIN)>ef_ag)?r_ag_set-ef_ag:AG_MIN;last_direction<=2;
                end else if(r_ae_set>AE_MIN) begin
                    r_ae_set<=((r_ae_set-AE_MIN)>ef_ae)?r_ae_set-ef_ae:AE_MIN;last_direction<=2;
                end
            end else begin
                if(r_ae_set<AE_MAX) begin
                    r_ae_set<=((AE_MAX-r_ae_set)>ef_ae)?r_ae_set+ef_ae:AE_MAX;last_direction<=1;
                end else if(!manual_gain && r_ag_set<gain_ceiling) begin
                    r_ag_set<=((gain_ceiling-r_ag_set)>ef_ag)?r_ag_set+ef_ag:gain_ceiling;last_direction<=1;
                end
            end
        end
    end
end

//=============================================================================
//  七、变化检测 → 产生一次写入请求（沿用原版，逻辑未改）
//-----------------------------------------------------------------------------
//  复位后 r_ag(17) 与 r_ag_set(96) 故意不相等，
//  保证 sensor 初始化完成后必定下发一次曝光/增益，行为与原始版本一致。
//  写入门控条件包含 I_ae_cfg_done，确保上一次 I2C 写完后才发起下一次。
//=============================================================================

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

function [15:0] bcd11;
    input [10:0] bin;
    integer i,j;
    reg [15:0] v;
    begin
        v=0;
        for(i=10;i>=0;i=i-1) begin
            for(j=0;j<4;j=j+1) if(v[j*4 +:4]>=5) v[j*4 +:4]=v[j*4 +:4]+4'd3;
            v={v[14:0],bin[i]};
        end
        bcd11=v;
    end
endfunction
wire [15:0] target_bcd=bcd11({3'b000,t_target});
wire [15:0] exposure_bcd=bcd11(r_ae[10:0]);
always @(posedge I_clk or posedge I_rst)
    if(I_rst) O_quality_status<=0;
    else O_quality_status<={manual_gain,target_bcd[10:0],exposure_bcd};

endmodule
