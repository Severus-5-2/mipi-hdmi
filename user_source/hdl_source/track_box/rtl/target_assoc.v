//=====================================================================
//  target_assoc.v  ——  跨帧目标关联 + 框稳定化（方案B §3.1 进阶2 / D5）
//---------------------------------------------------------------------
//  归属：张铭晨（进阶2 目标跟踪与信息标注）
//  依据：《开发方案B》§3.1 +《接口冻结表》v1.8 §2（S_target_bus 96bit × 4 槽）
//
//  做什么
//    消费张金艺检测链产出的「目标清单」（每帧最多 4 槽、按 area 降序、帧界锁存），
//    做跨帧关联，把「逐帧跳动的检测结果」变成「稳定的跟踪框」，再交给 box_draw。
//
//  五件套（方案B §3.1「框要稳」）
//    1) 最近邻关联：代价 = 曼哈顿距离 + (面积差 >> AREA_SHIFT) + 类别不一致惩罚
//    2) 速度外推   ：关联用「上次原始位置 + 速度」预测，消除 EMA 滞后带来的掉关联
//    3) EMA 平滑   ：画框位置 pos <= pos + ((new - pos) >>> EMA_SHIFT)（α=1/4，零乘法）
//    4) 死区       ：|Δ| < DEADZONE 时该字段不更新（消除静止时的像素级抖动）
//    5) 丢失滞回   ：连续 MAX_MISS 帧无匹配才删框（防闪烁 / 断框）
//
//  ⚠ 为什么必须做速度外推（实测踩到的坑）
//    只做 EMA 时，恒定速度 v 的稳态滞后 = v·(1-α)/α = 3v（α=1/4）。
//    v=30px/帧 ⇒ 滞后 90px，已经贴着 MATCH_TH=96；一旦越界，关联失败会被
//    误判成「新目标」→ **凭空多出一个重复框**（TB 场景 E 实测，f=72 起出现
//    第二个红框）。改成「用原始位置+速度做预测后关联」后，恒速目标的关联代价
//    ≈ 0；画面上的框仍走 EMA，所以既跟得稳又不抖。
//
//  ⚠ 新目标门控（防重复框的第二道闸）
//    只有在「附近（MATCH_TH 内）没有任何已有轨迹」时才允许新建轨迹。
//    否则「两个输入抢同一条轨迹」的落败者会去占空槽、凭空多一个框。
//
//  ⚠ 有符号运算（本项目历史事故高发区）
//    差值一律 $signed(...) 显式标注，再用 >>>（算术右移）。
//    Verilog 里「无符号 + 有符号」混合会整体按无符号算（负差右移变巨大正数）。
//    TB 场景 C 专门造奇数值反向移动盖这条（顺带验 floor 语义）。
//
//  ⚠ 面积差为什么要移位
//    area 量级 0~921600，直接与位置（0~2000）相加会被面积主导。
//    AREA_SHIFT=8 后「同目标帧间面积波动」≈0、「不同目标」≈几十，量级才可比。
//
//  ⚠⚠ 状态为什么用「扁平向量」而不是 unpacked 数组（踩过的坑，勿改回去）
//    iverilog 有个陷阱：**连续赋值里调用「读 unpacked 数组的函数」时，数组被写入
//    （尤其 NBA）不会触发该表达式重新求值**，函数返回值会一直卡在 x。
//      reg [10:0] R[0:3]; function f; input j; f = IN - R[j]; endfunction
//      wire A = f(2'd0);              // ← R[0] 变了 A 也不更新，恒为 x
//    而以 unpacked 数组为输入写 always @(*) 又会被 iverilog -Wall 告警
//    （"@* is sensitive to all N words in array"）。
//    ⇒ 因此本模块把 4 条轨迹的每个字段都压成**扁平向量**，用变址部分选择
//      `R_tcx[j*11 +: 11]` 寻址：既不触发数组告警，事件敏感也完全正确。
//
//  时序
//    一帧只跑一次：I_frame_done（= 工程 S_frame_done，落在帧首有效像素）触发，
//    **先等 ARM_DELAY(默认 8) 拍**再开始读总线（吸收「本模块拿到的帧脉冲」
//    与「生产方锁存总线那一拍」之间的相位差，见参数注释），
//    之后 7 拍完成 4 个目标的关联 + 更新 + 输出，帧内其余时间输出保持稳定。
//    「帧首那几个像素用旧框」无影响：y=0 行被常驻 Logo 覆盖。
//
//  位级验证
//    sim/bit_sim_target_assoc.py  独立实现本模块算法（含补码语义），逐帧对拍
//    tb/tb_target_assoc.v         iverilog 逐帧序列自检（打印 IN/OUT 供对拍）
//=====================================================================
module target_assoc #(
    //------------------------------------------------------------------
    //  可调参数（方案B §3.1 给定默认值）
    //------------------------------------------------------------------
    parameter integer EMA_SHIFT  = 2,     // 位置 EMA α = 1/4（= 1/2^2）
    parameter integer VEL_SHIFT  = 2,     // 速度 EMA α = 1/4
    parameter integer DEADZONE   = 2,     // 死区：|Δ| < 2 px 不更新
    parameter integer MAX_MISS   = 5,     // 连续 5 帧无匹配 → 删框
    parameter integer MATCH_TH   = 96,    // 关联代价门限（曼哈顿域）
    parameter integer AREA_SHIFT = 8,     // 面积差缩放（>> 8）
    // ⚠ CLS_PEN 故意设得 > MATCH_TH：效果 = 「类别不同就基本不关联」。
    //   演示场景各目标一色，颜色是强特征，这样最稳（不会两个不同色目标互相串框）。
    //   但注意：新目标门控用的是**含惩罚后的代价**，所以异色新目标（代价含 128）
    //   仍能越过门控正常建新轨迹。
    parameter integer CLS_PEN    = 128,

    //------------------------------------------------------------------
    //  ★ 帧脉冲 → 首拍读总线的间隔（2026-10-14 新增，把「相位」做成无关项）
    //    生产方（`common_base/conn_features`）是在**它自己的帧脉冲**那一拍
    //    用 NBA 锁存 `S_target_bus`；而本模块拿到的 `I_frame_done` 可能比那
    //    一拍**早若干拍** —— 典型情形：直接接 `pix_coord_gen.O_frame_done`
    //    （= 帧首有效像素），而总线锁存在 `S_det_fdone`
    //    （= 帧首 + 颜色检测内部流水 ≈ +3 拍）。
    //
    //    本模块是「**一拍读一个槽**」，若首拍早于总线更新，槽 0/1/2 会读到
    //    **上一帧**的值（实测见 `tb/tb_frame_latch_early.v`：不等待时只能容忍
    //    帧脉冲比总线锁存早 **2** 拍，早 3 拍就会读错）。
    //
    //    ⇒ 收到帧脉冲后先等 `ARM_DELAY` 拍再开始读总线，把这段相位差吸收掉。
    //      实测：`ARM_DELAY=8` 可容忍「脉冲比总线锁存早 **≤10** 拍」
    //      （真实链路约 3 拍，余量 ~3×）。帧内总线的其余时间完全稳定，
    //      所以等久一点没有任何副作用（一帧 39600 拍，本 FSM 总共只占 ~16 拍）。
    //
    //    ⚠ 但这只是**安全网**，不是主方案：更稳的做法是把 `I_frame_done`
    //      接到与总线锁存**同源**的那一路（`S_det_fdone`），此时相位差为 0。
    parameter integer ARM_DELAY  = 8
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    // ---- 帧界脉冲（= 工程 S_frame_done，落在帧首有效像素）----
    //      此时目标清单已稳定（生产方在 vsync 上升沿锁存，之后整帧保持不变）
    input  wire        I_frame_done,

    // ---- 目标清单总线（冻结表 v1.8 §2）----
    input  wire [2:0]  I_target_cnt,          // 本帧有效目标数（仅供参考，以槽 valid 为准）
    input  wire [96*4-1:0] I_target_bus,      // 4 槽 × 96 bit

    // ---- 输出：稳定后的框（直接接 box_draw 的 I_box_*）----
    output reg  [3:0]      O_box_valid,
    output reg  [4*11-1:0] O_box_xmin,
    output reg  [4*11-1:0] O_box_xmax,
    output reg  [4*10-1:0] O_box_ymin,
    output reg  [4*10-1:0] O_box_ymax,
    output reg  [4*2-1:0]  O_box_cls,
    output reg  [4*11-1:0] O_box_cx,          // 平滑后的质心（供 OSD 显示）
    output reg  [4*10-1:0] O_box_cy,

    // ---- 统计 ----
    output reg  [2:0]      O_track_cnt        // 当前在跟的目标数
);

    //==================================================================
    // 1. 位域常量（冻结表 v1.8 §2.1，低位在前）
    //      [0]valid [2:1]cls [13:3]cx [23:14]cy [34:24]xmin [45:35]xmax
    //      [55:46]ymin [65:56]ymax [85:66]area [95:86]保留
    //==================================================================
    localparam integer B_VALID = 0;
    localparam integer B_CLS   = 1;
    localparam integer B_CX    = 3;
    localparam integer B_CY    = 14;
    localparam integer B_XMIN  = 24;
    localparam integer B_XMAX  = 35;
    localparam integer B_YMIN  = 46;
    localparam integer B_YMAX  = 56;
    localparam integer B_AREA  = 66;

    // 每条轨迹字段位宽（用于变址部分选择 j*W +: W）
    localparam integer W_CXW = 11;
    localparam integer W_CYW = 10;
    localparam integer W_ARW = 20;
    localparam integer W_VEW = 13;      // 速度（13bit 有符号，±2047）
    localparam integer W_MSW = 3;

    localparam [2:0]   MISS_LIM = MAX_MISS;   // 3bit 化，便于比较

    //==================================================================
    // 2. 轨迹状态（4 条，扁平向量；见文件头「为什么用扁平向量」）
    //    px/py = 上次「原始」观测位置（关联用，不参与平滑）
    //    vx/vy = 速度估计（有符号，px/帧）
    //==================================================================
    reg [4*1 -1:0]  R_tv;       // 有效
    reg [4*2 -1:0]  R_tcls;     // 类别
    reg [4*11-1:0]  R_tcx;      // 平滑质心 x（画框用）
    reg [4*10-1:0]  R_tcy;      // 平滑质心 y（画框用）
    reg [4*11-1:0]  R_txmin;
    reg [4*11-1:0]  R_txmax;
    reg [4*10-1:0]  R_tymin;
    reg [4*10-1:0]  R_tymax;
    reg [4*20-1:0]  R_tarea;
    reg [4*11-1:0]  R_tpx;      // 上次原始质心 x
    reg [4*10-1:0]  R_tpy;      // 上次原始质心 y
    reg signed [4*13-1:0] R_tvx;// 速度 x（有符号）
    reg signed [4*13-1:0] R_tvy;// 速度 y（有符号）
    reg [4*3 -1:0]  R_tmiss;    // 连续未匹配帧数

    // 本帧已被某个输入目标占用的轨迹（防止两个目标抢同一条）
    reg [3:0]       S_taken;

    //==================================================================
    // 3. FSM
    //==================================================================
    localparam [2:0] ST_IDLE  = 3'd0,
                     ST_ARM   = 3'd4,   // ★ 收到帧脉冲后先等 ARM_DELAY 拍（吸收相位差）
                     ST_COST  = 3'd1,   // 阶段1：算 4 条轨迹代价（组合算、本拍末寄存）
                     ST_SEL   = 3'd5,   // 阶段2：对寄存的代价做最小选择（组合、本拍末寄存）
                     ST_UPD   = 3'd6,   // 阶段3：按选中的轨迹做速度/EMA 更新
                     ST_FIN   = 3'd2,   // 未匹配轨迹：miss++ / 超时删除
                     ST_OUT   = 3'd3;   // 工作区 → 输出寄存器

    reg [2:0] S_st;
    reg [2:0] S_i;                     // 当前处理的输入槽号
    reg [3:0] S_arm;                    // ★ 帧脉冲后的等待计数

    //==================================================================
    // 4. 当前输入槽解码（S_i 选槽；96 bit 的 4 选 1）
    //==================================================================
    wire [95:0] W_s0 = I_target_bus[0*96 +: 96];
    wire [95:0] W_s1 = I_target_bus[1*96 +: 96];
    wire [95:0] W_s2 = I_target_bus[2*96 +: 96];
    wire [95:0] W_s3 = I_target_bus[3*96 +: 96];

    wire [95:0] W_slot = (S_i == 3'd0) ? W_s0 :
                         (S_i == 3'd1) ? W_s1 :
                         (S_i == 3'd2) ? W_s2 : W_s3;

    wire        W_in_valid = W_slot[B_VALID];
    wire [1:0]  W_in_cls   = W_slot[B_CLS  +: 2];
    wire [10:0] W_in_cx    = W_slot[B_CX   +: 11];
    wire [9:0]  W_in_cy    = W_slot[B_CY   +: 10];
    wire [10:0] W_in_xmin  = W_slot[B_XMIN +: 11];
    wire [10:0] W_in_xmax  = W_slot[B_XMAX +: 11];
    wire [9:0]  W_in_ymin  = W_slot[B_YMIN +: 10];
    wire [9:0]  W_in_ymax  = W_slot[B_YMAX +: 10];
    wire [19:0] W_in_area  = W_slot[B_AREA +: 20];

    //==================================================================
    // 5. ★ 最近邻搜索 —— 三级流水（2026-10-15 加，解 Setup −6.9ns）
    //------------------------------------------------------------------
    //  背景：合板后 TD 报最差路径 10/10 全在 u_target_assoc 内，
    //        起点 R_tvy_reg[0]、终点 R_tvx/R_txmax，Logic Level 26
    //        （ADDER=18），Data Path 20.03ns > 13.333ns 预算。
    //        那一整条「预测位置 → 代价 → 最小选择 → 速度更新 → R_tvx」
    //        原本压在同一拍里。
    //
    //  改法（方案 A：只插寄存器，算法与结果逐字节不变）：
    //    第 1 拍 P_COST：对 4 条轨迹并行算「预测位置 + 代价」→ 打拍存 R_cost[4]
    //    第 2 拍 P_SEL ：对 4 个已寄存的代价做最小选择 → 打拍存 R_bj/R_best/…
    //    第 3 拍 P_UPD ：用选中的 R_bj 做速度 EMA + 位置 EMA → 写回轨迹
    //  ⇒ 原 18 级加法链切成 3 段，每段 ≤6 级；一拍多出的 2 个等待拍
    //    对帧率零影响（一帧 39600 拍，本 FSM 原本只用 ~16 拍）。
    //
    //  ★ 为什么"打拍"不改结果：原来的组合块本就是在「同一拍」内按 mi=0..3
    //    顺序算；拆成三段后，每一段用的输入（W_in_*、R_t* 、S_taken）
    //    都与原来同一个语义拍一致 —— 只是把「组合延迟」换成了「寄存器延迟」。
    //    代价：每条轨迹多 1 个代价寄存器 + 3 个零碎标志（约 20 bit × 4）。
    //
    //  ⚠ 注意：S_taken 在 ST_MATCH 里是「逐槽累加」的（每处理完一个输入槽就
    //    写一次），所以代价/选择/更新三级流水必须**按同一个输入槽**串起来，
    //    不能跨槽流水。因此下面的 FSM 走「一个输入槽 = 3 拍（COST/SEL/UPD）」。
    //==================================================================
    integer    ai;
    reg [12:0] C_best;
    reg [1:0]  C_bj;
    reg        C_hit;
    reg [12:0] C_any;
    reg        C_hit_any;
    reg [1:0]  C_alloc_j;
    reg        C_alloc_ok;

    // ---- 第 1 拍：4 条轨迹的代价（组合算、寄存器存）----
    //   ★ 用「扁平向量 + 变址部分选择」而不是 unpacked 数组：
    //     文件头已说明 —— iverilog -Wall 对 always @(*) 里的 unpacked 数组
    //     会报 "@* is sensitive to all N words in array"，与本工程「0 warning」
    //     的验收口径冲突。这里 4 项全部压成向量。
    reg [4*13-1:0] P_cost;             // 每条轨迹的代价（13bit × 4）
    reg [4*1 -1:0] P_valid;            // 该轨迹本拍是否有效（= R_tv）
    reg [4*1 -1:0] P_free;             // 该轨迹是否未被占用（= !S_taken）

    reg [10:0] C_prx;                  // 该轨迹的预测 x（组合中间量）
    reg [9:0]  C_pry;                  // 该轨迹的预测 y（组合中间量）
    reg signed [13:0] C_spx, C_spy;    // 预测中间量（有符号）
    reg [12:0] C_cost;
    reg [19:0] C_d;
    integer    mi;

    always @(*) begin
        // 组合：4 条轨迹的预测位置 + 代价（与旧版逐 mi 完全同式）
        C_prx = 11'd0;
        C_pry = 10'd0;
        C_spx = 14'sd0;
        C_spy = 14'sd0;
        C_cost = 13'd0;
        C_d    = 20'd0;
        P_cost  = {4{13'h1FFF}};
        for (mi = 0; mi < 4; mi = mi + 1) begin
            P_valid[mi] = R_tv[mi];
            P_free [mi] = !S_taken[mi];
            if (R_tv[mi]) begin
                // ---- 预测位置 = 上次原始位置 + 速度（1 帧外推），并夹到画面内 ----
                C_spx = $signed({3'b000, R_tpx[mi*W_CXW +: W_CXW]}) +
                        $signed(R_tvx[mi*W_VEW +: W_VEW]);
                C_prx = C_spx[13] ? 11'd0 :
                        (|C_spx[12:11] ? 11'd2047 : C_spx[10:0]);

                C_spy = $signed({3'b000, R_tpy[mi*W_CYW +: W_CYW]}) +
                        $signed(R_tvy[mi*W_VEW +: W_VEW]);
                C_pry = C_spy[13] ? 10'd0 :
                        (|C_spy[12:10] ? 10'd1023 : C_spy[9:0]);

                // ---- |Δcx| ----
                if (W_in_cx >= C_prx) C_cost = W_in_cx - C_prx;
                else                  C_cost = C_prx - W_in_cx;
                // ---- + |Δcy| ----
                if (W_in_cy >= C_pry) C_d = W_in_cy - C_pry;
                else                  C_d = C_pry - W_in_cy;
                C_cost = C_cost + C_d[9:0];
                // ---- + (|Δarea| >> AREA_SHIFT) ----
                if (W_in_area >= R_tarea[mi*W_ARW +: W_ARW])
                    C_d = W_in_area - R_tarea[mi*W_ARW +: W_ARW];
                else
                    C_d = R_tarea[mi*W_ARW +: W_ARW] - W_in_area;
                C_cost = C_cost + C_d[19:AREA_SHIFT];
                // ---- + 类别惩罚 ----
                if (W_in_cls != R_tcls[mi*2 +: 2])
                    C_cost = C_cost + CLS_PEN[12:0];

                P_cost[mi*13 +: 13] = C_cost;   // ★ 本拍末寄存，供下一拍做最小选择
            end
        end
    end

    // ---- 第 2 拍：对已寄存的 4 个代价做最小选择（组合）----
    //  ⚠⚠ 2026-10-15 修正：本段原先读的是 **组合的 `P_cost`/`P_valid`/`P_free`**，
    //     而不是上一拍寄存的 `Q_cost`/`Q_pv`/`Q_pf` —— 结果是 ST_COST 末尾写进 Q_* 的
    //     那一级流水**完全没被消费**（`Q_cost` 只写不读，是"死寄存器"，综合时被当冗余优化掉），
    //     COST 与 SEL 仍串在同一拍里。
    //     因为 ST_COST→ST_SEL 之间 `S_i` 与 `R_t*` 都不变，`P_cost` 在第 2 拍重算出来与
    //     `Q_cost` **值相同** ⇒ 功能没错（所以对拍全绿），但**组合链没被切开**。
    //     实测后果：布线后最差路径仍是「总线 → 槽选 mux → 代价计算 → 最小选择」一条龙
    //     （−1.707ns / Logic Level 20 / ADDER=12 / 终点 Q_cbest）。
    //     现改为读 `Q_*`（真·寄存版）⇒ 这一级流水才真正生效。
    always @(*) begin
        C_best     = 13'h1FFF;
        C_bj       = 2'd0;
        C_hit      = 1'b0;
        C_any      = 13'h1FFF;
        C_hit_any  = 1'b0;
        for (mi = 0; mi < 4; mi = mi + 1) begin
            if (Q_pv[mi]) begin
                // ---- 任意有效轨迹的最小代价（新目标门控）----
                if (!C_hit_any || Q_cost[mi*13 +: 13] < C_any) begin
                    C_any     = Q_cost[mi*13 +: 13];
                    C_hit_any = 1'b1;
                end
                // ---- 未被占用轨迹的最小代价（关联；平局取小号）----
                if (Q_pf[mi]) begin
                    if (!C_hit || Q_cost[mi*13 +: 13] < C_best) begin
                        C_best = Q_cost[mi*13 +: 13];
                        C_bj   = mi[1:0];
                        C_hit  = 1'b1;
                    end
                end
            end
        end
    end

    // ---- 空槽查找（新目标用；优先级 0 > 1 > 2 > 3）----
    always @(*) begin
        C_alloc_j  = 2'd0;
        C_alloc_ok = 1'b0;
        for (ai = 0; ai < 4; ai = ai + 1) begin
            if (!C_alloc_ok && !R_tv[ai] && !S_taken[ai]) begin
                C_alloc_j  = ai[1:0];
                C_alloc_ok = 1'b1;
            end
        end
    end

    // ★ 选择结果（第 2 拍组合，第 3 拍由 Q_* 寄存版使用）
    //   W_alloc_gate 在第 2 拍寄存进 Q_callgate，供第 3 拍使用
    wire W_alloc_gate = !C_hit_any || (C_any > MATCH_TH);

    //==================================================================
    // 5b. ★ 流水中间寄存器（三级流水的「拍间记忆」）
    //     —— 第 1 拍（COST）末尾写入、第 2 拍（SEL）读取；
    //        第 2 拍末尾把选择结果寄存、第 3 拍（UPD）读取。
    //==================================================================
    reg [4*13-1:0] Q_cost;             // 第 1 拍 → 第 2 拍：4 条轨迹的代价（13bit×4）
    reg [4*1 -1:0] Q_pv;               //                     该轨迹有效
    reg [4*1 -1:0] Q_pf;               //                     该轨迹未被占用
    reg [95:0]     Q_slot;             // 第 1 拍锁存的输入槽（给第 2/3 拍用）

    reg        Q_cbest_hit;            // 第 2 拍 → 第 3 拍：是否命中
    reg [1:0]  Q_cbj;                  //                     命中轨迹号
    reg [12:0] Q_cbest;                //                     最小代价
    reg        Q_callok;               //                     是否有空槽
    reg        Q_callgate;             //                     新目标门控（第 2 拍寄存）

    wire [95:0] W_qslot = Q_slot;      // 第 2/3 拍使用的输入槽（已锁存）
    wire        Q_in_valid = W_qslot[B_VALID];
    wire [1:0]  Q_in_cls   = W_qslot[B_CLS  +: 2];
    wire [10:0] Q_in_cx    = W_qslot[B_CX   +: 11];
    wire [9:0]  Q_in_cy    = W_qslot[B_CY   +: 10];
    wire [10:0] Q_in_xmin  = W_qslot[B_XMIN +: 11];
    wire [10:0] Q_in_xmax  = W_qslot[B_XMAX +: 11];
    wire [9:0]  Q_in_ymin  = W_qslot[B_YMIN +: 10];
    wire [9:0]  Q_in_ymax  = W_qslot[B_YMAX +: 10];
    wire [19:0] Q_in_area  = W_qslot[B_AREA +: 20];

    wire W_qmatch = Q_in_valid && Q_cbest_hit && (Q_cbest <= MATCH_TH);

    // ★ 新目标门控（第 2 拍寄存版，供第 3 拍使用；语义与 W_alloc_gate 完全一致）
    wire W_alloc_gate_q = Q_callgate;

    //==================================================================
    // 6a. 速度更新（针对第 3 拍选中的轨迹 Q_cbj）
    //     d_raw = cx_in - px（原始位移）；vx <= vx + ((d_raw - vx) >>> VEL_SHIFT)
    //     ★ 全部用第 2 拍寄存下来的输入值 / 轨迹号，故这一段只依赖寄存器输出
    //==================================================================
    wire signed [12:0] W_drx = $signed({2'b00, Q_in_cx}) - $signed({2'b00, R_tpx[Q_cbj*W_CXW +: W_CXW]});
    wire signed [12:0] W_dry = $signed({2'b00, Q_in_cy}) - $signed({2'b00, R_tpy[Q_cbj*W_CYW +: W_CYW]});
    wire signed [12:0] W_vdx = W_drx - $signed(R_tvx[Q_cbj*W_VEW +: W_VEW]);
    wire signed [12:0] W_vdy = W_dry - $signed(R_tvy[Q_cbj*W_VEW +: W_VEW]);

    wire signed [12:0] C_new_vx = $signed(R_tvx[Q_cbj*W_VEW +: W_VEW]) + (W_vdx >>> VEL_SHIFT);
    wire signed [12:0] C_new_vy = $signed(R_tvy[Q_cbj*W_VEW +: W_VEW]) + (W_vdy >>> VEL_SHIFT);

    //==================================================================
    // 6b. 6 个位置字段的「死区 + EMA」新值（针对第 3 拍选中的轨迹 Q_cbj）
    //     d = new - old（补码）；|d| >= DEADZONE 才更新：
    //        new = old + (d >>> EMA_SHIFT)          ← 算术右移，保留符号
    //==================================================================
    wire signed [12:0] W_dcx  = $signed({2'b00, Q_in_cx  }) - $signed({2'b00, R_tcx  [Q_cbj*W_CXW +: W_CXW]});
    wire signed [12:0] W_dcy  = $signed({2'b00, Q_in_cy  }) - $signed({2'b00, R_tcy  [Q_cbj*W_CYW +: W_CYW]});
    wire signed [12:0] W_dxmn = $signed({2'b00, Q_in_xmin}) - $signed({2'b00, R_txmin[Q_cbj*W_CXW +: W_CXW]});
    wire signed [12:0] W_dxmx = $signed({2'b00, Q_in_xmax}) - $signed({2'b00, R_txmax[Q_cbj*W_CXW +: W_CXW]});
    wire signed [12:0] W_dymn = $signed({2'b00, Q_in_ymin}) - $signed({2'b00, R_tymin[Q_cbj*W_CYW +: W_CYW]});
    wire signed [12:0] W_dymx = $signed({2'b00, Q_in_ymax}) - $signed({2'b00, R_tymax[Q_cbj*W_CYW +: W_CYW]});
    wire signed [20:0] W_dar  = $signed({1'b0,  Q_in_area}) - $signed({1'b0,  R_tarea[Q_cbj*W_ARW +: W_ARW]});

    // 绝对值（补码取反 +1；符号位决定）
    wire [12:0] W_adcx  = W_dcx [12] ? (~W_dcx  + 13'd1) : W_dcx [12:0];
    wire [12:0] W_adcy  = W_dcy [12] ? (~W_dcy  + 13'd1) : W_dcy [12:0];
    wire [12:0] W_adxmn = W_dxmn[12] ? (~W_dxmn + 13'd1) : W_dxmn[12:0];
    wire [12:0] W_adxmx = W_dxmx[12] ? (~W_dxmx + 13'd1) : W_dxmx[12:0];
    wire [12:0] W_adymn = W_dymn[12] ? (~W_dymn + 13'd1) : W_dymn[12:0];
    wire [12:0] W_adymx = W_dymx[12] ? (~W_dymx + 13'd1) : W_dymx[12:0];

    wire W_up_cx  = (W_adcx  >= DEADZONE[12:0]);
    wire W_up_cy  = (W_adcy  >= DEADZONE[12:0]);
    wire W_up_xmn = (W_adxmn >= DEADZONE[12:0]);
    wire W_up_xmx = (W_adxmx >= DEADZONE[12:0]);
    wire W_up_ymn = (W_adymn >= DEADZONE[12:0]);
    wire W_up_ymx = (W_adymx >= DEADZONE[12:0]);
    // 面积不做死区（面积差极少小于 2，死区对它无意义），直接 EMA
    wire W_up_ar  = 1'b1;

    wire signed [13:0] W_ncx  = $signed({3'b000, R_tcx  [Q_cbj*W_CXW +: W_CXW]}) + (W_dcx  >>> EMA_SHIFT);
    wire signed [13:0] W_ncy  = $signed({3'b000, R_tcy  [Q_cbj*W_CYW +: W_CYW]}) + (W_dcy  >>> EMA_SHIFT);
    wire signed [13:0] W_nxmn = $signed({3'b000, R_txmin[Q_cbj*W_CXW +: W_CXW]}) + (W_dxmn >>> EMA_SHIFT);
    wire signed [13:0] W_nxmx = $signed({3'b000, R_txmax[Q_cbj*W_CXW +: W_CXW]}) + (W_dxmx >>> EMA_SHIFT);
    wire signed [13:0] W_nymn = $signed({3'b000, R_tymin[Q_cbj*W_CYW +: W_CYW]}) + (W_dymn >>> EMA_SHIFT);
    wire signed [13:0] W_nymx = $signed({3'b000, R_tymax[Q_cbj*W_CYW +: W_CYW]}) + (W_dymx >>> EMA_SHIFT);
    wire signed [21:0] W_nar  = $signed({2'b00,  R_tarea[Q_cbj*W_ARW +: W_ARW]}) + (W_dar  >>> EMA_SHIFT);

    // 新目标时直接取输入值（无历史可平滑）
    wire [10:0] C_new_cx  = W_qmatch ? (W_up_cx  ? W_ncx [10:0] : R_tcx  [Q_cbj*W_CXW +: W_CXW]) : Q_in_cx;
    wire [9:0]  C_new_cy  = W_qmatch ? (W_up_cy  ? W_ncy [9:0]  : R_tcy  [Q_cbj*W_CYW +: W_CYW]) : Q_in_cy;
    wire [10:0] C_new_xmn = W_qmatch ? (W_up_xmn ? W_nxmn[10:0] : R_txmin[Q_cbj*W_CXW +: W_CXW]) : Q_in_xmin;
    wire [10:0] C_new_xmx = W_qmatch ? (W_up_xmx ? W_nxmx[10:0] : R_txmax[Q_cbj*W_CXW +: W_CXW]) : Q_in_xmax;
    wire [9:0]  C_new_ymn = W_qmatch ? (W_up_ymn ? W_nymn[9:0]  : R_tymin[Q_cbj*W_CYW +: W_CYW]) : Q_in_ymin;
    wire [9:0]  C_new_ymx = W_qmatch ? (W_up_ymx ? W_nymx[9:0]  : R_tymax[Q_cbj*W_CYW +: W_CYW]) : Q_in_ymax;
    wire [19:0] C_new_ar  = W_qmatch ? (W_up_ar  ? W_nar [19:0] : R_tarea[Q_cbj*W_ARW +: W_ARW]) : Q_in_area;

    //==================================================================
    // 7. 主状态机（每帧一次）
    //==================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_st     <= ST_IDLE;
            S_i      <= 3'd0;
            S_taken  <= 4'd0;
            R_tv     <= {4*1 {1'b0}};
            R_tcls   <= {4*2 {1'b0}};
            R_tcx    <= {4*11{1'b0}};
            R_tcy    <= {4*10{1'b0}};
            R_txmin  <= {4*11{1'b0}};
            R_txmax  <= {4*11{1'b0}};
            R_tymin  <= {4*10{1'b0}};
            R_tymax  <= {4*10{1'b0}};
            R_tarea  <= {4*20{1'b0}};
            R_tpx    <= {4*11{1'b0}};
            R_tpy    <= {4*10{1'b0}};
            R_tvx    <= {4*13{1'b0}};
            R_tvy    <= {4*13{1'b0}};
            R_tmiss  <= {4*3 {1'b0}};
            Q_slot   <= 96'd0;
            Q_cbest_hit <= 1'b0;
            Q_cbj       <= 2'd0;
            Q_cbest     <= 13'h1FFF;
            Q_callok    <= 1'b0;
            Q_callgate  <= 1'b0;
            O_box_valid <= 4'd0;
            O_box_xmin  <= {(4*11){1'b0}};
            O_box_xmax  <= {(4*11){1'b0}};
            O_box_ymin  <= {(4*10){1'b0}};
            O_box_ymax  <= {(4*10){1'b0}};
            O_box_cls   <= {(4*2){1'b0}};
            O_box_cx    <= {(4*11){1'b0}};
            O_box_cy    <= {(4*10){1'b0}};
            O_track_cnt <= 3'd0;
        end
        else begin
            case (S_st)

            //----------------------------------------------------------
            ST_IDLE: begin
                if (I_frame_done) begin
                    S_arm   <= ARM_DELAY[3:0];   // ★ 先等，再去读总线
                    S_st    <= ST_ARM;
                end
            end

            //----------------------------------------------------------
            // ★ 等 ARM_DELAY 拍：把「帧脉冲 vs 总线锁存」的相位差吸收掉
            //   （否则首拍早于总线更新 → 前几个槽读到上一帧；详见参数注释）
            ST_ARM: begin
                if (S_arm == 4'd0) begin
                    S_taken <= 4'd0;
                    S_i     <= 3'd0;
                    S_st    <= ST_COST;        // ★ 进入三级流水的第 1 拍
                end else begin
                    S_arm   <= S_arm - 4'd1;
                end
            end

            //----------------------------------------------------------
            // ★ 流水 阶段 1（COST）：锁存输入槽 + 算 4 条轨迹的代价
            //   组合块 P_cost[] 本拍算好，本拍末（时钟沿）寄存进 Q_cost[]
            //----------------------------------------------------------
            ST_COST: begin
                Q_slot <= W_slot;              // 锁存本槽，供 SEL/UPD 两拍使用
                Q_cost <= P_cost;              // 4 条轨迹代价打拍（整向量）
                Q_pv   <= P_valid;             // 有效性/占用性同步打拍
                Q_pf   <= P_free;
                S_st   <= ST_SEL;
            end

            //----------------------------------------------------------
            // ★ 流水 阶段 2（SEL）：对已寄存的代价做最小选择 → 寄存结果
            //----------------------------------------------------------
            ST_SEL: begin
                Q_cbest_hit <= C_hit;
                Q_cbj       <= C_bj;
                Q_cbest     <= C_best;
                Q_callok    <= C_alloc_ok;
                Q_callgate  <= W_alloc_gate;   // ★ 第 2 拍寄存门控，第 3 拍用
                S_st        <= ST_UPD;
            end

            //----------------------------------------------------------
            // ★ 流水 阶段 3（UPD）：按选中轨迹做速度/EMA 更新（或占空槽）
            //   全部输入来自寄存器（Q_*），故这一段与阶段 1/2 无组合耦合
            //----------------------------------------------------------
            ST_UPD: begin
                if (Q_in_valid) begin
                    if (W_qmatch) begin
                        // ---- 关联上已有轨迹：速度更新 + EMA + 死区 ----
                        S_taken [Q_cbj]                       <= 1'b1;
                        R_tmiss [Q_cbj*W_MSW +: W_MSW]        <= 3'd0;
                        R_tcls  [Q_cbj*2 +: 2]                <= Q_in_cls;  // 类别立即跟随
                        R_tpx   [Q_cbj*W_CXW +: W_CXW]        <= Q_in_cx;   // 原始位置
                        R_tpy   [Q_cbj*W_CYW +: W_CYW]        <= Q_in_cy;
                        R_tvx   [Q_cbj*W_VEW +: W_VEW]        <= C_new_vx;  // 速度 EMA
                        R_tvy   [Q_cbj*W_VEW +: W_VEW]        <= C_new_vy;
                        R_tcx   [Q_cbj*W_CXW +: W_CXW]        <= C_new_cx;  // 平滑框
                        R_tcy   [Q_cbj*W_CYW +: W_CYW]        <= C_new_cy;
                        R_txmin [Q_cbj*W_CXW +: W_CXW]        <= C_new_xmn;
                        R_txmax [Q_cbj*W_CXW +: W_CXW]        <= C_new_xmx;
                        R_tymin [Q_cbj*W_CYW +: W_CYW]        <= C_new_ymn;
                        R_tymax [Q_cbj*W_CYW +: W_CYW]        <= C_new_ymx;
                        R_tarea [Q_cbj*W_ARW +: W_ARW]        <= C_new_ar;
                    end
                    else if (Q_callok && W_alloc_gate_q) begin
                        // ---- 新目标：直接落一条空闲轨迹（不平滑，速度清零）----
                        S_taken [C_alloc_j]                   <= 1'b1;
                        R_tv    [C_alloc_j]                   <= 1'b1;
                        R_tmiss [C_alloc_j*W_MSW +: W_MSW]    <= 3'd0;
                        R_tcls  [C_alloc_j*2 +: 2]            <= Q_in_cls;
                        R_tpx   [C_alloc_j*W_CXW +: W_CXW]    <= Q_in_cx;
                        R_tpy   [C_alloc_j*W_CYW +: W_CYW]    <= Q_in_cy;
                        R_tvx   [C_alloc_j*W_VEW +: W_VEW]    <= 13'sd0;
                        R_tvy   [C_alloc_j*W_VEW +: W_VEW]    <= 13'sd0;
                        R_tcx   [C_alloc_j*W_CXW +: W_CXW]    <= Q_in_cx;
                        R_tcy   [C_alloc_j*W_CYW +: W_CYW]    <= Q_in_cy;
                        R_txmin [C_alloc_j*W_CXW +: W_CXW]    <= Q_in_xmin;
                        R_txmax [C_alloc_j*W_CXW +: W_CXW]    <= Q_in_xmax;
                        R_tymin [C_alloc_j*W_CYW +: W_CYW]    <= Q_in_ymin;
                        R_tymax [C_alloc_j*W_CYW +: W_CYW]    <= Q_in_ymax;
                        R_tarea [C_alloc_j*W_ARW +: W_ARW]    <= Q_in_area;
                    end
                    // 既匹配不上、又不满足新建门控 / 无空槽 → 丢弃（框已满或防重复）
                end
                if (S_i == 3'd3)
                    S_st <= ST_FIN;
                else begin
                    S_i  <= S_i + 3'd1;
                    S_st <= ST_COST;           // 下一个输入槽，回到流水首拍
                end
            end

            //----------------------------------------------------------
            // 未被任何输入目标认领的轨迹：miss++；连续 MAX_MISS 帧 → 删框
            ST_FIN: begin
                if (R_tv[0] && !S_taken[0]) begin
                    if (R_tmiss[0*W_MSW +: W_MSW] + 3'd1 >= MISS_LIM) R_tv[0] <= 1'b0;
                    else R_tmiss[0*W_MSW +: W_MSW] <= R_tmiss[0*W_MSW +: W_MSW] + 3'd1;
                end
                if (R_tv[1] && !S_taken[1]) begin
                    if (R_tmiss[1*W_MSW +: W_MSW] + 3'd1 >= MISS_LIM) R_tv[1] <= 1'b0;
                    else R_tmiss[1*W_MSW +: W_MSW] <= R_tmiss[1*W_MSW +: W_MSW] + 3'd1;
                end
                if (R_tv[2] && !S_taken[2]) begin
                    if (R_tmiss[2*W_MSW +: W_MSW] + 3'd1 >= MISS_LIM) R_tv[2] <= 1'b0;
                    else R_tmiss[2*W_MSW +: W_MSW] <= R_tmiss[2*W_MSW +: W_MSW] + 3'd1;
                end
                if (R_tv[3] && !S_taken[3]) begin
                    if (R_tmiss[3*W_MSW +: W_MSW] + 3'd1 >= MISS_LIM) R_tv[3] <= 1'b0;
                    else R_tmiss[3*W_MSW +: W_MSW] <= R_tmiss[3*W_MSW +: W_MSW] + 3'd1;
                end
                S_st <= ST_OUT;
            end

            //----------------------------------------------------------
            // 工作区 → 输出寄存器（原子更新，帧内不再抖动）
            ST_OUT: begin
                O_box_valid[0]         <= R_tv[0];
                O_box_valid[1]         <= R_tv[1];
                O_box_valid[2]         <= R_tv[2];
                O_box_valid[3]         <= R_tv[3];
                O_box_cls  [0*2  +: 2] <= R_tcls  [0*2  +: 2];
                O_box_cls  [1*2  +: 2] <= R_tcls  [1*2  +: 2];
                O_box_cls  [2*2  +: 2] <= R_tcls  [2*2  +: 2];
                O_box_cls  [3*2  +: 2] <= R_tcls  [3*2  +: 2];
                O_box_xmin [0*11 +: 11]<= R_txmin [0*11 +: 11];
                O_box_xmin [1*11 +: 11]<= R_txmin [1*11 +: 11];
                O_box_xmin [2*11 +: 11]<= R_txmin [2*11 +: 11];
                O_box_xmin [3*11 +: 11]<= R_txmin [3*11 +: 11];
                O_box_xmax [0*11 +: 11]<= R_txmax [0*11 +: 11];
                O_box_xmax [1*11 +: 11]<= R_txmax [1*11 +: 11];
                O_box_xmax [2*11 +: 11]<= R_txmax [2*11 +: 11];
                O_box_xmax [3*11 +: 11]<= R_txmax [3*11 +: 11];
                O_box_ymin [0*10 +: 10]<= R_tymin [0*10 +: 10];
                O_box_ymin [1*10 +: 10]<= R_tymin [1*10 +: 10];
                O_box_ymin [2*10 +: 10]<= R_tymin [2*10 +: 10];
                O_box_ymin [3*10 +: 10]<= R_tymin [3*10 +: 10];
                O_box_ymax [0*10 +: 10]<= R_tymax [0*10 +: 10];
                O_box_ymax [1*10 +: 10]<= R_tymax [1*10 +: 10];
                O_box_ymax [2*10 +: 10]<= R_tymax [2*10 +: 10];
                O_box_ymax [3*10 +: 10]<= R_tymax [3*10 +: 10];
                O_box_cx   [0*11 +: 11]<= R_tcx   [0*11 +: 11];
                O_box_cx   [1*11 +: 11]<= R_tcx   [1*11 +: 11];
                O_box_cx   [2*11 +: 11]<= R_tcx   [2*11 +: 11];
                O_box_cx   [3*11 +: 11]<= R_tcx   [3*11 +: 11];
                O_box_cy   [0*10 +: 10]<= R_tcy   [0*10 +: 10];
                O_box_cy   [1*10 +: 10]<= R_tcy   [1*10 +: 10];
                O_box_cy   [2*10 +: 10]<= R_tcy   [2*10 +: 10];
                O_box_cy   [3*10 +: 10]<= R_tcy   [3*10 +: 10];
                O_track_cnt <= {2'b00, R_tv[0]} + {2'b00, R_tv[1]} +
                               {2'b00, R_tv[2]} + {2'b00, R_tv[3]};
                S_st        <= ST_IDLE;
            end

            default: S_st <= ST_IDLE;
            endcase
        end
    end

endmodule
