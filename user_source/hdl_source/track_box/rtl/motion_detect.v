//=====================================================================
//  motion_detect.v  ——  帧差法运动检测（方案B §3.2 进阶4 / D7）
//---------------------------------------------------------------------
//  归属：张铭晨（进阶4 运动检测）
//  依据：《开发方案B》§3.2
//
//  做什么
//    对显示流做「帧差法」运动检测：相邻两帧同位置亮度差超阈值即为运动像素，
//    再经「块密度门控 + 游程合并」，输出最多 4 个运动区域框 + 区域数量。
//
//  五级流水（全部在像素时钟域）
//    ① 灰度      ：复用工程已编译的 rgb2gray（BT.601，77/150/29 >>8），不另起一套
//    ② 1/4 降采样：每个 4×4 块取**左上 2×2 求平均**（>>2）→ 320×180 的 ds 域
//    ③ 帧差      ：与上一帧 BRAM 同址比较，|ΔY| >= MOT_TH → 运动位
//    ④ 块密度门控：ds 域 16×16 块内统计运动像素数，>= BLK_MIN 才算「该块在动」
//                  ★ 下限是主力去噪：孤立噪点/色度漂移凑不满一个块
//                  ★ 2026-10-18 新增「整行过满丢弃」：某块行活跃块数 >= ROW_MAX_BLK
//                    （默认 20 = 整行全满）⇒ 判为全局亮度漂移（工频频闪/AE 抖动），整行不建框
//    ⑤ 游程合并  ：逐块行找连续段（**先做 1 块空洞桥接**：某块不活跃但左右都活跃则视为连续），
//                  再与上一块行的段（x 区间重叠或仅隔 1 块）合并 → 区域框；
//                  最后按最小跨块数过滤掉单块伪影
//
//  ⚠ 为什么去掉了「块级 3×3 形态学」（设计取舍，已评估并实测）
//    计划里写的是「先腐蚀后膨胀」。落地时发现：块边长 16（ds 域）= 64 个全分辨率
//    像素，块级 3×3 膨胀会把每个框**整体外扩 64 像素**，框明显大于目标本身，
//    精度反而变差；而腐蚀（0 填充）还会把画面最外一圈块整条抹掉。
//    去噪的真正主力其实是 ④ 块密度门控（孤立点根本无法让某块计数到 BLK_MIN）。
//    因此改为：④ 门限 12.5%（32/256）承担去噪，⑤ 用「1 块空洞桥接 + 相邻块
//    行合并」保证连通性，最后再用「最小跨块数 MIN_SPAN_BLK」滤掉单块伪影。
//    ⇒ 等价效果、框更贴合目标、面积更省。若要严格按原计划的形态学，在 ⑤ 前
//      对块图加一级 3×3 腐蚀+膨胀即可（约 40 行），但框会外扩。
//
//  ⚠ BRAM 读写时序（避免同址读写竞争，必须是三段）
//    ds 域每 4 个像素钟才产生 1 个样点，用三段流水把「发读 / 等数据 / 比较回写」分开：
//      S0 拍：锁存该样点坐标(±ds 值与地址)、发读
//      S1 拍：同步读的数据本拍末尾才更新（本拍看到的还是上一地址的数据）⇒ 不能用
//      S2 拍：S_rdata 才是本样点的上一帧值 ⇒ 比较放在这一拍
//      回写：S3 拍写入本帧值（读在前、写在后，无「同址读写」歧义）
//    ⚠ 若把比较提前到 S1 拍，会拿「上一个 ds 样点地址」的旧值去比 ⇒ 等于拿左邻块比，
//      静止画面也会因为水平边缘而「处处在动」。这是实测踩过的坑。
//    即使 I_de 停掉，也只是不再更新，不会自比自报运动。
//
//  ⚠ 已知固有特性（帧差法的物理限制，非 bug）
//    物体匀速移动时，差分会同时出现「新位置」和「旧位置」两块区域；位移小于
//    物体尺寸时两者重叠被合并成 1 个框，位移大于物体尺寸时会得到 2 个框。
//    这是帧差法本身的特性，若要单目标单框需引入背景建模/运动历史。
//
//  时序/规模
//    一帧一次输出：在 I_frame_done（帧首有效像素）把当前框表锁存到 O_*，
//    帧内保持不变 ⇒ 输出表示的是「上一帧」的运动结果（1 帧延迟）。
//    上一帧 BRAM = 320×180×8bit = 57600 B（≈450 Kbit）。
//
//  位级验证
//    sim/bit_sim_motion_detect.py  独立实现本模块算法，逐帧逐字段对拍
//    tb/tb_motion_detect.v         iverilog 激励（合成视频帧）+ 打印 IN/OUT
//=====================================================================
module motion_detect #(
    parameter integer IMG_W    = 1280,
    parameter integer IMG_H    = 720,
    parameter integer DS_W     = 320,       // 降采样域宽（= IMG_W/4）
    parameter integer DS_H     = 180,       // 降采样域高（= IMG_H/4）
    parameter integer BLK      = 16,        // 块边长（ds 域）
    parameter integer BGW      = DS_W/BLK,  // 20 块列
    parameter integer BGH      = 11,        // 11 块行（覆盖 ds 行 0..175）
    parameter integer MOT_TH   = 52,        // 帧差阈值（8bit 亮度域）
                                            // ★ 2026-10-19 15:05：36 → **52**（配合下方「零存储」降采样）。
                                            //   量纲：本版 ds = 2 像素平均（σ_ds = 0.707σ），v5.1 的 8 像素是 0.354σ
                                            //   ⇒ 要维持同样的假阳性率，阈值必须 ×2。
                                            //   临界像素噪声：2像素+52 ⇒ σ≈29.4（8像素+36 是 40.7）
                                            //   ⚠ 绝不能照搬现场的 36 —— 那个 36 跑在「半尺度」旧 ds 上（等效 72），
                                            //     照搬到这里等于等效 26 ⇒ 紫框必然回来。
                                            // ★ 2026-10-18 22:57 由 32 改 36：
                                            //   第 2 次下板（MOT_TH=32/等效 64）紫框仍在（能见 2 个大紫框），
                                            //   再抬一档 ⇒ 等效门槛 72。三色目标像素差都 >100 ⇒ 仍有余量。
                                            // ★ 2026-10-18 22:15 由 20 改 32：本版降采样仍带
                                            //   「最右一列代替所有列」的 bug ⇒ ds 只有真值一半 ⇒
                                            //   等效门槛 = 2×MOT_TH = 64。三色目标像素差 >80 ⇒ 仍检出；
                                            //   噪声侧余量翻倍。这是**纯常量改动**（不改网表结构），
                                            //   而 M57 的结构改动会让 TD 合成出坏硬件（上板满屏雪花）。
                                            //   ★ 2026-10-18 晚：16 → 20（Z54 要求单独验证，已实测通过）
                                            //   机理：MOT_TH 只抬"单像素亮度差"门槛，**不动"块内像素数"**，
                                            //        所以慢速目标的帧差带（5px 竖条、幅度=目标亮度差）不会被砍。
                                            //   实测（sim/bit_sim_d8_system.py，BLK_MIN 保持 32）：
                                            //        MOT_TH=20 ⇒ 7 条语义断言全 PASS（含"运动框完整包住真实变化区"）
                                            //        MOT_TH=24 ⇒ 同样全 PASS
                                            //   ⚠ 前提：目标与背景的亮度差要 > MOT_TH。d8_system 的目标对比度是 200
                                            //      （块值 200 vs 背景 0）⇒ 远高于门槛，所以 PASS 只证明"高对比度目标不受影响"。
                                            //      低对比度场景（灰物体在灰墙上，差 20~40）余量会变小 ⇒ 20 比 24 稳。
                                            //   ⇒ 若现场 MOT_TH=20 压不住噪声，可再上 24（d8 也 PASS）；仍不行则改降采样平均（见文件尾）。
    parameter integer BLK_MIN  = 32,        // 块内运动像素数门限（共 256 = 12.5%）
                                            //   ⚠ 2026-10-18 评估后【保持 32 不变】：
                                            //     实测（sim/bit_sim_d8_system.py）目标 dx≈5px/帧、高 100px 时，
                                            //     帧差只剩两条 ~5px 宽的竖带 ⇒ ds 域每块仅 16~32 个运动像素。
                                            //     抬到 64（25%）⇒ 该断言直接 FAIL（运动框包不住真实变化区）。
                                            //     ⇒ 紫条在"块内比例"这个维度上与慢速真目标重叠，**不能靠 BLK_MIN 区分**。
                                            //   ★ 对比（2026-10-18 晚实测）：MOT_TH 单独抬到 20/24 都是全 PASS
                                            //     ⇒ 两者机理不同：BLK_MIN 砍"块内像素数"，MOT_TH 只砍"单像素差值"。
    parameter integer ROW_MAX_BLK = 20,     // ★ 2026-10-18 新增：**块行活跃块数上限**
                                            //   某块行里有 >= ROW_MAX_BLK 个块被判"在动" ⇒ **整行丢弃**。
                                            //   依据：现场紫条是【全宽横贯】⇒ 该块行 20 块全活跃；
                                            //   而真实目标（人/车/手）横向不会占满整行 ⇒ 天然可区分。
                                            //   这正是"工频光源频闪 / AE 增益抖动"造成的整行亮度同向漂移的签名。
                                            //   默认 20 = 只杀"20 块全活跃"的完全全宽行（最保守）；
                                            //   设 21 = 关闭本功能；设 18 更激进（>=90% 就丢）。
    parameter integer MIN_SPAN_BLK = 2,     // 区域框最小跨块数（滤单块伪影）
    parameter integer MAXBOX   = 4          // 输出区域框上限（对齐 box_draw 4 槽）
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    // ---- 显示像素流（接 pix_coord_gen 的 O_de/O_pix_x/O_pix_y）----
    input  wire        I_de,
    input  wire [10:0] I_pix_x,
    input  wire [9:0]  I_pix_y,
    input  wire [23:0] I_rgb,

    // ---- 帧界（= 工程 S_frame_done，帧首有效像素）----
    input  wire        I_frame_done,

    // ---- 输出：运动区域框（全分辨率像素坐标，块坐标 ×64）----
    output reg  [MAXBOX*1 -1:0] O_mot_valid,
    output reg  [MAXBOX*11-1:0] O_mot_xmin,
    output reg  [MAXBOX*11-1:0] O_mot_xmax,
    output reg  [MAXBOX*10-1:0] O_mot_ymin,
    output reg  [MAXBOX*10-1:0] O_mot_ymax,
    output reg  [2:0]           O_mot_cnt,      // 运动区域数（0~4）

    // ---- 诊断/上板观测 ----
    output wire        O_mot_pix,               // 当前 ds 样点是否为运动像素
    output wire        O_mot_pix_v,             // 上述有效
    output reg  [7:0]  O_act_blk                // 上一帧活动块数（0~220）
);

    reg [7:0] S_act;                            // 活动块累加器（帧末锁存到 O_act_blk）

    //==================================================================
    // 1. 灰度（复用工程 rgb2gray）
    //==================================================================
    wire [7:0] W_gray;
    rgb2gray U_gray (
        .clk      (I_clk),
        .rst_n    (I_rst_n),
        .rgb_in   (I_rgb),
        .gray_out (W_gray)
    );

    //==================================================================
    // 2. 1/4 降采样（每个 4×4 块取左上 2×2 平均）
    //    水平：x[1:0]=1 起累加，x[1:0]=2 得到 2 像素和（1 拍后 S_hpair 有效）
    //    垂直：y[1:0]=0 锁存上半行对，y[1:0]=1 与下半行对相加 → >>2
    //    有效点落在 x[1:0]=3 & y[1:0]=1 那一拍
    //
    //    ⚠ 本拍同时把该 ds 样点的**真实坐标** (dsx,dsy) 一起锁存。
    //      不能在下一拍再去取 I_pix_x[10:2] —— 那时 x 已经 +1，算出来是 c+1，
    //      会让整幅 ds 图右移一列（帧差拿左邻块比、块列归属也错位）。
    //==================================================================
    reg [8:0]  S_hacc;
    reg [8:0]  S_hpair;
    reg [7:0]  S_ds;
    reg        S_ds_v;
    reg [8:0]  S_dsx;                           // ds 列号 0..319
    reg [7:0]  S_dsy;                           // ds 行号 0..179

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_hacc  <= 9'd0;
            S_hpair <= 9'd0;
            S_ds    <= 8'd0;
            S_ds_v  <= 1'b0;
            S_dsx   <= 9'd0;
            S_dsy   <= 8'd0;
        end else begin
            S_ds_v <= 1'b0;
            if (I_de) begin
                // ⚠ rgb2gray 的 gray_out 比坐标晚 1 拍（其内部有一级寄存器），
                //   所以整条降采样相对「坐标 x[1:0]」后移 1 拍取窗口，取到的正好是
                //   块左上 2×2：(4c,4r)(4c+1,4r)(4c,4r+1)(4c+1,4r+1)。
                //   不补偿的话窗口会偏成 (4c-1,4c)，整幅运动图左移 1 像素。
                if (I_pix_x[1:0] == 2'd1)
                    S_hacc <= {1'b0, W_gray};
                else if (I_pix_x[1:0] == 2'd2)
                    S_hpair <= S_hacc + {1'b0, W_gray};

                // ★ 2026-10-19 15:05「零存储」降采样（收编张铭晨 M71 v5.2 的最小改动版）
                //   旧写法（有 bug）：
                //     y[1:0]==0 → S_vpair <= S_hpair;     // 单寄存器，整行每个像素钟都被覆盖
                //     y[1:0]==1 → S_ds <= (S_vpair + S_hpair) >> 2;
                //   后果：S_vpair 留下的是「最右一列」的值 ⇒ ① 非最右列 ds 只有真值一半；
                //         ② 最右列的贡献与列无关 ⇒ 该块行所有块一起翻 ⇒ 一条 1 块行高的紫横条。
                //   现在：**去掉垂直平均，只做水平 2 像素平均** ⇒ 零存储、无跨列状态。
                //   取 y[1:0]==3（不是 ==1）：仍是"每 4 行取 1 行"，但垂直相位与 v5.1 一致
                //     ⇒ 下游 TB / 黄金向量一个字节都不用改（12 帧输出逐字段相同）。
                if (I_pix_x[1:0] == 2'd3) begin
                    if (I_pix_y[1:0] == 2'd3) begin
                        S_ds   <= S_hpair >> 1;     // 2 像素均值（与旧版 >>2 同尺度）
                        S_dsx  <= I_pix_x[10:2];    // = c
                        S_dsy  <= I_pix_y[9:2];     // = r
                        S_ds_v <= 1'b1;
                    end
                end
            end
        end
    end

    //==================================================================
    // 3. 帧差（上一帧 BRAM：发读 → 等数据 → 比 → 回写）
    //
    //    ⚠ 三段流水，缺一段就会「拿错地址比」：
    //      S0 拍：ds 有效 → 锁存坐标/数据 + 发读（S_raddr = 本样点地址）
    //      S1 拍：同步读的数据在**本拍末尾**才更新，S1 拍看到的还是上一个地址的数据
    //      S2 拍：S_rdata 才是本样点的上一帧值 ⇒ 比较必须放在这一拍
    //      回写：S2 拍置 S_we，S3 拍真正写入（读在前、写在后，无同址竞争）
    //==================================================================
    reg [7:0]  S_fbuf [0:DS_W*DS_H-1];
    reg [15:0] S_raddr;
    reg [7:0]  S_rdata;

    reg        S_st1;
    reg        S_st2;
    reg [7:0]  S_ds_l;
    reg [15:0] S_addr_l;
    reg [8:0]  S_dx_l;
    reg [7:0]  S_dy_l;

    reg        S_we;
    reg [15:0] S_waddr;
    reg [7:0]  S_wdata;

    // ⚠ 刻意用「移位加法」而不是 `* 16'd320`：
    //   工程 DSP 已 38/40（95%），常量乘法虽多数综合器会优化成移位，
    //   但**不能保证**不被推断成 DSP。320 = 256 + 64 ⇒
    //     S_dsy*320 = (S_dsy<<8) + (S_dsy<<6)，纯拼接 + 加法，零乘法器。
    //   数值完全等价（实测原式与移位式逐帧输出一致）。
    wire [15:0] W_addr = {S_dsy, 8'd0} + {2'b00, S_dsy, 6'd0} + {7'd0, S_dsx};

    always @(posedge I_clk) S_rdata <= S_fbuf[S_raddr];           // 同步读
    always @(posedge I_clk) if (S_we) S_fbuf[S_waddr] <= S_wdata; // 同步写

    wire [7:0]  W_diff = (S_ds_l >= S_rdata) ? (S_ds_l - S_rdata) : (S_rdata - S_ds_l);
    wire        W_mot  = (W_diff >= MOT_TH[7:0]);

    reg         S_mot_v;
    reg         S_mot;
    reg [8:0]   S_dx_p2;
    reg [7:0]   S_dy_p2;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_st1    <= 1'b0;
            S_st2    <= 1'b0;
            S_mot_v  <= 1'b0;
            S_mot    <= 1'b0;
            S_we     <= 1'b0;
            S_raddr  <= 16'd0;
            S_ds_l   <= 8'd0;
            S_addr_l <= 16'd0;
            S_dx_l   <= 9'd0;
            S_dy_l   <= 8'd0;
            S_dx_p2  <= 9'd0;
            S_dy_p2  <= 8'd0;
            S_waddr  <= 16'd0;
            S_wdata  <= 8'd0;
        end else begin
            // --- S0：ds 有效 → 锁存 + 发读 ---
            if (S_ds_v) begin
                S_raddr  <= W_addr;
                S_ds_l   <= S_ds;
                S_addr_l <= W_addr;
                S_dx_l   <= S_dsx;
                S_dy_l   <= S_dsy;
            end
            S_st1 <= S_ds_v;
            // --- S1：登记回写信息 + 坐标延后 ---
            if (S_st1) begin
                S_waddr <= S_addr_l;
                S_wdata <= S_ds_l;
                S_dx_p2 <= S_dx_l;
                S_dy_p2 <= S_dy_l;
            end
            S_st2 <= S_st1;
            // --- S2：S_rdata 已是本样点的上一帧值 → 比较 + 拉回写使能 ---
            S_mot_v <= 1'b0;
            S_we    <= 1'b0;
            if (S_st2) begin
                S_mot   <= W_mot;
                S_mot_v <= 1'b1;
                S_we    <= 1'b1;
            end
        end
    end

    assign O_mot_pix   = S_mot;
    assign O_mot_pix_v = S_mot_v;

    //==================================================================
    // 4. 块密度门控（ds 域 16×16 → 块图 20×11）
    //==================================================================
    reg [8:0]     S_bcnt [0:BGW-1];
    reg [BGW-1:0] S_pend;
    reg           S_rend;
    reg           S_brow_go;

    integer k;

    wire W_blkrow_end = S_mot_v && (S_dy_p2[3:0] == 4'hF) &&
                        (S_dx_p2 == 9'd319) && (S_dy_p2 < 8'd176);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_rend    <= 1'b0;
            S_brow_go <= 1'b0;
            S_pend    <= {BGW{1'b0}};
            for (k = 0; k < BGW; k = k + 1) S_bcnt[k] <= 9'd0;
        end else begin
            S_rend    <= 1'b0;
            S_brow_go <= 1'b0;

            if (S_mot_v && (S_dy_p2 < 8'd176) && S_mot)
                S_bcnt[S_dx_p2[8:4]] <= S_bcnt[S_dx_p2[8:4]] + 9'd1;

            if (W_blkrow_end) S_rend <= 1'b1;

            if (S_rend) begin
                for (k = 0; k < BGW; k = k + 1) begin
                    S_pend[k] <= (S_bcnt[k] >= BLK_MIN);
                    S_bcnt[k] <= 9'd0;
                end
                S_brow_go <= 1'b1;
            end
        end
    end

    //==================================================================
    // 5. 游程合并（块行 → 区域框）
    //==================================================================
    localparam [1:0] G_IDLE  = 2'd0,
                     G_SCAN  = 2'd1,
                     G_MERGE = 2'd2,
                     G_MDEC  = 2'd3;

    reg [1:0]     S_gst;
    reg [BGW-1:0] S_scan;
    reg [4:0]     S_col;
    reg           S_inrun;
    reg [4:0]     S_run_s, S_run_e;
    reg [1:0]     S_mj;
    reg           S_mdone;
    reg           S_lastcol;
    reg [3:0]     S_scan_row;
    reg [3:0]     S_row_in;

    reg           S_boxv [0:MAXBOX-1];
    reg [4:0]     S_bx1  [0:MAXBOX-1];
    reg [4:0]     S_bx2  [0:MAXBOX-1];
    reg [3:0]     S_by1  [0:MAXBOX-1];
    reg [3:0]     S_by2  [0:MAXBOX-1];

    wire [4:0] W_pc = S_pend[0] + S_pend[1] + S_pend[2] + S_pend[3] + S_pend[4] +
                      S_pend[5] + S_pend[6] + S_pend[7] + S_pend[8] + S_pend[9] +
                      S_pend[10]+ S_pend[11]+ S_pend[12]+ S_pend[13]+ S_pend[14]+
                      S_pend[15]+ S_pend[16]+ S_pend[17]+ S_pend[18]+ S_pend[19];

    // 单块空洞桥接：某块不活跃、但左右紧邻两块都活跃 ⇒ 视为连续
    //   目的：物体位移小于自身尺寸时，帧差会在**同一块行内**留下「旧位置 | 空洞 | 新位置」，
    //   不桥接就会输出 2 个框；桥接后合成 1 个框（更符合"运动区域连续"的观感）。
    wire [BGW-1:0] W_scan_prv = {1'b0, S_scan[BGW-1:1]};
    wire [BGW-1:0] W_scan_nxt = {S_scan[BGW-2:0], 1'b0};
    wire [BGW-1:0] W_scan_b   = S_scan | (W_scan_prv & W_scan_nxt);
    wire [31:0]    W_scan_b32 = {{(32-BGW){1'b0}}, W_scan_b};   // 变址越界防护

    // 最小跨块数过滤（滤掉单块伪影）
    wire W_big [0:MAXBOX-1];
    genvar gj;
    generate
    for (gj = 0; gj < MAXBOX; gj = gj + 1) begin : G_BIG
        assign W_big[gj] = ((S_bx2[gj] - S_bx1[gj] + 5'd1) >= MIN_SPAN_BLK[4:0]) ||
                           ((S_by2[gj] - S_by1[gj] + 4'd1) >= MIN_SPAN_BLK[3:0]);
    end
    endgenerate

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_gst     <= G_IDLE;
            S_scan    <= {BGW{1'b0}};
            S_col     <= 5'd0;
            S_inrun   <= 1'b0;
            S_run_s   <= 5'd0;
            S_run_e   <= 5'd0;
            S_mj      <= 2'd0;
            S_mdone   <= 1'b0;
            S_lastcol <= 1'b0;
            S_scan_row<= 4'd0;
            S_row_in  <= 4'd0;
            O_mot_valid <= {MAXBOX{1'b0}};
            O_mot_xmin  <= {(MAXBOX*11){1'b0}};
            O_mot_xmax  <= {(MAXBOX*11){1'b0}};
            O_mot_ymin  <= {(MAXBOX*10){1'b0}};
            O_mot_ymax  <= {(MAXBOX*10){1'b0}};
            O_mot_cnt   <= 3'd0;
            O_act_blk   <= 8'd0;
            S_act       <= 8'd0;
            for (k = 0; k < MAXBOX; k = k + 1) begin
                S_boxv[k] <= 1'b0;
                S_bx1[k]  <= 5'd0;
                S_bx2[k]  <= 5'd0;
                S_by1[k]  <= 4'd0;
                S_by2[k]  <= 4'd0;
            end
        end else begin
            if (I_frame_done) begin
                //------------------------------------------------------
                // 帧界：锁存上一帧结果；清框表，开新一帧
                //------------------------------------------------------
                O_mot_valid[0] <= S_boxv[0] && W_big[0];
                O_mot_valid[1] <= S_boxv[1] && W_big[1];
                O_mot_valid[2] <= S_boxv[2] && W_big[2];
                O_mot_valid[3] <= S_boxv[3] && W_big[3];
                // 坐标一律用 valid 门控：无效槽输出全 0（接口自洽，避免残留旧框）
                O_mot_xmin [0*11 +: 11] <= (S_boxv[0] && W_big[0]) ? {S_bx1[0], 6'b000000} : 11'd0;
                O_mot_xmin [1*11 +: 11] <= (S_boxv[1] && W_big[1]) ? {S_bx1[1], 6'b000000} : 11'd0;
                O_mot_xmin [2*11 +: 11] <= (S_boxv[2] && W_big[2]) ? {S_bx1[2], 6'b000000} : 11'd0;
                O_mot_xmin [3*11 +: 11] <= (S_boxv[3] && W_big[3]) ? {S_bx1[3], 6'b000000} : 11'd0;
                O_mot_xmax [0*11 +: 11] <= (S_boxv[0] && W_big[0]) ? ({S_bx2[0] + 5'd1, 6'b000000} - 11'd1) : 11'd0;
                O_mot_xmax [1*11 +: 11] <= (S_boxv[1] && W_big[1]) ? ({S_bx2[1] + 5'd1, 6'b000000} - 11'd1) : 11'd0;
                O_mot_xmax [2*11 +: 11] <= (S_boxv[2] && W_big[2]) ? ({S_bx2[2] + 5'd1, 6'b000000} - 11'd1) : 11'd0;
                O_mot_xmax [3*11 +: 11] <= (S_boxv[3] && W_big[3]) ? ({S_bx2[3] + 5'd1, 6'b000000} - 11'd1) : 11'd0;
                O_mot_ymin [0*10 +: 10] <= (S_boxv[0] && W_big[0]) ? {S_by1[0], 6'b000000} : 10'd0;
                O_mot_ymin [1*10 +: 10] <= (S_boxv[1] && W_big[1]) ? {S_by1[1], 6'b000000} : 10'd0;
                O_mot_ymin [2*10 +: 10] <= (S_boxv[2] && W_big[2]) ? {S_by1[2], 6'b000000} : 10'd0;
                O_mot_ymin [3*10 +: 10] <= (S_boxv[3] && W_big[3]) ? {S_by1[3], 6'b000000} : 10'd0;
                O_mot_ymax [0*10 +: 10] <= (S_boxv[0] && W_big[0]) ? ({S_by2[0], 6'b000000} + 10'd63) : 10'd0;
                O_mot_ymax [1*10 +: 10] <= (S_boxv[1] && W_big[1]) ? ({S_by2[1], 6'b000000} + 10'd63) : 10'd0;
                O_mot_ymax [2*10 +: 10] <= (S_boxv[2] && W_big[2]) ? ({S_by2[2], 6'b000000} + 10'd63) : 10'd0;
                O_mot_ymax [3*10 +: 10] <= (S_boxv[3] && W_big[3]) ? ({S_by2[3], 6'b000000} + 10'd63) : 10'd0;
                O_mot_cnt   <= {2'b00, (S_boxv[0] && W_big[0])} +
                               {2'b00, (S_boxv[1] && W_big[1])} +
                               {2'b00, (S_boxv[2] && W_big[2])} +
                               {2'b00, (S_boxv[3] && W_big[3])};
                O_act_blk   <= S_act;
                S_act       <= 8'd0;
                // 无效槽的坐标一并清零（否则会残留上一帧旧框坐标，接口不自洽）
                for (k = 0; k < MAXBOX; k = k + 1) begin
                    S_boxv[k] <= 1'b0;
                    S_bx1[k]  <= 5'd0;
                    S_bx2[k]  <= 5'd0;
                    S_by1[k]  <= 4'd0;
                    S_by2[k]  <= 4'd0;
                end
                S_row_in  <= 4'd0;
                S_gst     <= G_IDLE;
            end
            else begin
                if (S_brow_go) S_act <= S_act + {3'b000, W_pc};

                case (S_gst)
                //------------------------------------------------------
                G_IDLE: begin
                    if (S_brow_go) begin
                        // ★ 2026-10-18 新增：整行"过满"⇒丢弃
                        //   若本块行有 >= ROW_MAX_BLK 个块被判"在动"（默认 20 = 整行全满），
                        //   判为全局亮度漂移（工频频闪 / AE 增益抖动），**整行不建框**。
                        //   真目标横向不会占满整行 ⇒ 不受影响。
                        //   ⚠ O_act_blk 仍照原样统计（诊断用），所以可能看到「act=220 但无框」。
                        S_scan     <= (W_pc >= ROW_MAX_BLK[4:0]) ? {BGW{1'b0}} : S_pend;
                        S_scan_row <= S_row_in;
                        S_row_in   <= S_row_in + 4'd1;
                        S_col      <= 5'd0;
                        S_inrun    <= 1'b0;
                        S_mj       <= 2'd0;
                        S_mdone    <= 1'b0;
                        S_lastcol  <= 1'b0;
                        S_gst      <= G_SCAN;
                    end
                end
                //------------------------------------------------------
                G_SCAN: begin
                    if (W_scan_b32[S_col]) begin
                        if (!S_inrun) begin
                            S_inrun <= 1'b1;
                            S_run_s <= S_col;
                        end
                        S_run_e <= S_col;
                        if (S_col == (BGW - 1)) begin
                            S_mj      <= 2'd0;
                            S_mdone   <= 1'b0;
                            S_lastcol <= 1'b1;
                            S_gst     <= G_MERGE;
                        end
                        else S_col <= S_col + 5'd1;
                    end
                    else begin
                        if (S_inrun) begin
                            S_inrun <= 1'b0;
                            S_mj    <= 2'd0;
                            S_mdone <= 1'b0;
                            // ⚠ 若本段恰好在**末列**遇到 0 而结束，也必须置 lastcol，
                            //   否则 G_MDEC 会把 S_col 加到 20（越界）→ 状态机在本块行
                            //   内反复环绕、同一段被重复建框。这是实测踩过的坑。
                            if (S_col == (BGW - 1)) S_lastcol <= 1'b1;
                            S_gst   <= G_MERGE;
                        end
                        else if (S_col == (BGW - 1))
                            S_gst <= G_IDLE;              // 本行扫完
                        else
                            S_col <= S_col + 5'd1;
                    end
                end
                //------------------------------------------------------
                // 与上一块行(ymax==scan_row-1)、x 区间重叠或仅隔 1 块的框合并
                G_MERGE: begin
                    if (!S_mdone && S_boxv[S_mj] &&
                        (S_by2[S_mj] == (S_scan_row - 4'd1)) &&
                        (S_bx1[S_mj] <= (S_run_e + 5'd1)) &&
                        ((S_bx2[S_mj] + 5'd1) >= S_run_s)) begin
                        S_bx1[S_mj] <= (S_bx1[S_mj] < S_run_s) ? S_bx1[S_mj] : S_run_s;
                        S_bx2[S_mj] <= (S_bx2[S_mj] > S_run_e) ? S_bx2[S_mj] : S_run_e;
                        S_by2[S_mj] <= S_scan_row;
                        S_mdone     <= 1'b1;
                    end
                    if (S_mj == (MAXBOX-1)) S_gst <= G_MDEC;
                    else                    S_mj  <= S_mj + 2'd1;
                end
                //------------------------------------------------------
                G_MDEC: begin
                    if (!S_mdone) begin
                        // 没有可合并的 → 新建（优先空槽 0>1>2>3）
                        if      (!S_boxv[0]) begin S_boxv[0]<=1'b1; S_bx1[0]<=S_run_s; S_bx2[0]<=S_run_e; S_by1[0]<=S_scan_row; S_by2[0]<=S_scan_row; end
                        else if (!S_boxv[1]) begin S_boxv[1]<=1'b1; S_bx1[1]<=S_run_s; S_bx2[1]<=S_run_e; S_by1[1]<=S_scan_row; S_by2[1]<=S_scan_row; end
                        else if (!S_boxv[2]) begin S_boxv[2]<=1'b1; S_bx1[2]<=S_run_s; S_bx2[2]<=S_run_e; S_by1[2]<=S_scan_row; S_by2[2]<=S_scan_row; end
                        else if (!S_boxv[3]) begin S_boxv[3]<=1'b1; S_bx1[3]<=S_run_s; S_bx2[3]<=S_run_e; S_by1[3]<=S_scan_row; S_by2[3]<=S_scan_row; end
                        // 4 槽占满 → 丢弃该段（演示场景足够）
                    end
                    S_inrun <= 1'b0;
                    if (S_lastcol) begin
                        S_lastcol <= 1'b0;
                        S_gst     <= G_IDLE;
                    end
                    else begin
                        S_col <= S_col + 5'd1;
                        S_gst <= G_SCAN;
                    end
                end
                default: S_gst <= G_IDLE;
                endcase
            end
        end
    end

endmodule
