//=============================================================================
//  detect_color_mask.v  —  目标检测第一步：色度双阈值分割 + 二值掩膜
//=============================================================================
//  项目  : MIPI-HDMI 实时视频图像处理系统（方案B §2 扩展2，负责人：张金艺）
//  归属  : user_source/hdl_source/detect_zjy/
//  里程碑: D1（取数 + YCbCr + 阈值掩膜）
//
//  功能一句话：把 S_video_sat(RGB888) 转成 YCbCr，按「Cb/Cr 双阈值」分割出目标
//              颜色的二值掩膜，并输出与掩膜严格对齐的像素坐标 pix_x / pix_y。
//
//-----------------------------------------------------------------------------
//  为什么必须走 YCbCr 而不是直接在 RGB 上卡阈值（方案B §1.1）
//-----------------------------------------------------------------------------
//  链路里有 AWB，RGB 三通道比例会随光照漂移 —— 换个灯，RGB 阈值就失效。
//  色度分量 Cb/Cr 对亮度变化鲁棒得多，是颜色分割的正确空间。
//
//  与 chroma_denoise.v 使用**完全相同**的系数与取整方式（位级一致）：
//      Y  =  77R + 150G +  29B            → (x + 128) >>> 8
//      Cb = -43R -  85G + 128B  + 128     → ((x + 128) >>> 8) + 128
//      Cr = 128R - 107G -  21B  + 128     → ((x + 128) >>> 8) + 128
//  （+128 是四舍五入，>>> 是算术右移；两者都不能省，否则会有 1 LSB 系统性偏移）
//
//-----------------------------------------------------------------------------
//  流水线划分（74.25MHz，13.33ns，单级 ≤4 级逻辑；方案B §1.2）
//-----------------------------------------------------------------------------
//   拍 0  : 输入（I_rgb 与 I_de 同拍）
//   拍 1  : P1 —— 9 个常量乘积寄存（每个乘积独立，≤4 级）
//   拍 2  : P2 —— 3 项求和 + 四舍五入 + 算术右移 + 偏置 128（≈4 级）
//   拍 3  : P3 —— 双阈值比较（掩膜）+ 限幅（≈3 级）
//
//   总延迟 = 3 拍。模块内部自动把 I_de / I_vsync / I_hsync / I_user / I_last
//   以及**外部输入的公共坐标 I_pix_x / I_pix_y** 同步延时 3 拍，与 O_* 输出严格对齐，
//   因此下游拿到的 (O_mask, O_de, O_pix_x, O_pix_y) 是同一像素。
//
//  ★ 2026-10-10 改造：坐标改为「接入公共坐标」，不再自建计数器
//     · 依方案B §1.3 + 对账结论：全队统一用 `track_box/rtl/pix_coord_gen.v`
//       生成的 `S_pix_x` / `S_pix_y`。本模块删除内部 x_cnt / y_cnt。
//     · 输入请接 `S_pix_x_2d` / `S_pix_y_2d`（已延时 2 拍），并把 PIX_DLY 设 2；
//       模块内部再经 2 级寄存器 ⇒ 合计与 3 拍流水严格对齐。
//     · ⚠ 若接未延时的 `S_pix_x`，请把 PIX_DLY 设 0（模块会自动补足 3 级）。
//
//  ⚠ 输入侧对齐：本模块假定 I_rgb 与 I_de 同拍。但工程里 S_video_sat 比
//     S_hdmi_de 滞后若干拍（gamma 1 + chroma_denoise + sat_enhance 2）。
//     若直接接 S_hdmi_de，请把差值填进 PIPE_DELAY（用法同 ae_meter.PIPE_DELAY）。
//
//-----------------------------------------------------------------------------
//  位宽与溢出核查
//-----------------------------------------------------------------------------
//   · 单个乘积：最大 |128 × 255| = 32640  < 18bit 有符号上限 131071   ✔
//   · 3 项和  ：最大 |±32640|            < 131071                    ✔
//   · (和 + 128) >>> 8 ∈ [-128, 128]，再 +128 ∈ [0, 256]，用 10bit 有符号 ✔
//   · 限幅到 0~255 后取低 8 位
//=============================================================================

`timescale 1ns / 1ps

module detect_color_mask #(
    //------------------------------------------------------------------
    // 视频时序
    //------------------------------------------------------------------
    parameter integer IMG_WIDTH  = 1280,   // 一行有效像素数，必须与视频时序一致
    parameter integer IMG_HEIGHT = 720,    // 一帧有效行数
    parameter integer PIPE_DELAY = 0,      // I_rgb 相对 I_de 的滞后拍数（0~15）
    //------------------------------------------------------------------
    // 公共坐标的「已延时拍数」——由调用方按所接信号决定：
    //    接 S_pix_x_2d → PIX_DLY = 2（推荐，与 S_video_sat 同舞台）
    //    接 S_pix_x    → PIX_DLY = 0
    //    接 S_pix_x_1d → PIX_DLY = 1
    //  模块内部固定再补 (2 - PIX_DLY) 级，使坐标与 3 拍流水严格对齐。
    //------------------------------------------------------------------
    parameter integer PIX_DLY   = 2,

    //------------------------------------------------------------------
    // 亮度门限（抑制高增益下的暗部彩色噪点；0 = 关闭门控）
    //------------------------------------------------------------------
    parameter [7:0] Y_MIN = 8'd0,

    //------------------------------------------------------------------
    // ★ 2026-10-11 曝光自适应色相判据（v2）
    //------------------------------------------------------------------
    //  现场问题：低曝光时「检测不全」「掩膜/小绿块时有时无」「框只在移动时显现」
    //  根因（已用 sim_lowlight_mask.py 定量验证）：
    //    目标变暗 ⇒ 三通道同比例缩小 ⇒ 色差 (R−Y)、(B−Y) 同比例缩小
    //    ⇒ 饱和色朝中性灰 (Cb,Cr)=(128,128) 收敛 ⇒ 逃出**固定绝对窗口**。
    //    实测崩溃点（目标亮度百分比）：
    //        红球 35% 崩（Cr 从 255 降到 153，跌破 RED_CR_LO=155）
    //        蓝块 35% 崩（Cb 从 236 降到 153，跌破 BLUE_CB_LO=155）
    //        绿块 60% 就崩（Cb/Cr 从 82/69 升到 100/93，涨出窗口上界）
    //      ⇒ 「绿的最先没框」「时有时无」由此而来。
    //
    //  处置：把「固定绝对窗口」换成**以 (128,128) 为原点的方向 + 幅度判据**：
    //      dcb = Cb − 128 ，dcr = Cr − 128
    //      mag = |dcb| + |dcr|                         （饱和度幅度）
    //      红：dcr > 0 且 dcb ≤ 0 且 |dcr|*10 ≥ |dcb|*RATIO10  且 mag ≥ MAG_TH
    //      蓝：dcb > 0 且 dcr ≤ 0 且 |dcb|*10 ≥ |dcr|*RATIO10  且 mag ≥ MAG_TH
    //      绿：dcb < 0 且 dcr < 0                              且 mag ≥ MAG_TH
    //    方向判据对「整体亮度缩放」天然免疫（缩放不改变 dcb/dcr 的符号与比值），
    //    只留 mag 一个门限来挡暗部噪点。
    //
    //  定量收益（sim_lowlight_mask.py 同一套随机模型）：
    //    目标亮度     当前(绝对窗口)          本方案(MAG_TH=16)
    //      60%      红1.00 蓝1.00 绿0.00     红1.00 蓝1.00 绿1.00
    //      35%      红0.14 蓝0.11 绿0.00     红1.00 蓝1.00 绿1.00
    //      28%      全 0                     红1.00 蓝0.99 绿1.00
    //    干扰色误判率   5.7%   →   1.1%（降 5 倍）
    //    交叉错色率      —     →   0.00%
    //------------------------------------------------------------------
    //  幅度门限：|dcb|+|dcr| 低于此值视为「近中性灰」，不参与检测。
    //    · 越大 → 抗噪越强，但欠曝目标越晚被检出
    //    · 16 为实测最优点（见上表）；逐级可调 12~22
    //------------------------------------------------------------------
    parameter [7:0] MAG_TH = 8'd16,

    //------------------------------------------------------------------
    //  色相比值判据 RATIO10：用整数比较 |a|*10 >= |b|*RATIO10 实现 |a| >= |b|*RATIO/10
    //    · RATIO10=13 ⇔ 比值 1.3（实测最优：抗干扰色最好且不误杀目标）
    //    · 调大 → 红/蓝判定更严（更像纯色才认），抗干扰更强但欠曝目标更难认
    //    · 调小 → 更宽松，易把橙/肤色误判成红
    //------------------------------------------------------------------
    parameter [4:0] RATIO10 = 5'd13,

    //------------------------------------------------------------------
    // 【保留】固定绝对窗口 —— 兼容模式
    //   置 1 时走旧判据（D1 基线行为不变），供位级对拍 / 回归使用。
    //   置 0（默认）走上面的曝光自适应判据。
    //------------------------------------------------------------------
    parameter integer USE_ABS_WINDOW = 0,

    //------------------------------------------------------------------
    // 红色目标：Cb 偏低、Cr 偏高（仅 USE_ABS_WINDOW=1 时生效）
    //------------------------------------------------------------------
    parameter [7:0] RED_CB_LO = 8'd50,   RED_CB_HI = 8'd125,
    parameter [7:0] RED_CR_LO = 8'd155,  RED_CR_HI = 8'd255,

    //------------------------------------------------------------------
    // 蓝色目标：Cb 偏高、Cr 偏低（仅 USE_ABS_WINDOW=1 时生效）
    //------------------------------------------------------------------
    parameter [7:0] BLUE_CB_LO = 8'd155, BLUE_CB_HI = 8'd255,
    parameter [7:0] BLUE_CR_LO = 8'd40,  BLUE_CR_HI = 8'd125,

    //------------------------------------------------------------------
    // 绿色目标：Cb、Cr 双低（仅 USE_ABS_WINDOW=1 时生效）
    //------------------------------------------------------------------
    parameter [7:0] GREEN_CB_LO = 8'd20, GREEN_CB_HI = 8'd95,
    parameter [7:0] GREEN_CR_LO = 8'd0,  GREEN_CR_HI = 8'd85
)(
    input  wire        clk,          // 像素时钟 S_hdmi_pixel_clk (74.25MHz)
    input  wire        rst_n,        // 低电平复位

    // ---- 视频时序（与工程 uivtc 输出同款）----
    input  wire        I_de,         // 数据有效
    input  wire        I_vsync,      // 场同步
    input  wire        I_hsync,      // 行同步
    input  wire        I_user,       // 帧起始标志（与 hdmi_mixer 的 I_video_user 同义）
    input  wire        I_last,       // 行末像素标志（与 hdmi_mixer 的 I_video_last 同义）

    // ---- 像素数据 ----
    input  wire [23:0] I_rgb,        // 来自 S_video_sat（{R[7:0], G[7:0], B[7:0]}）

    // ---- 公共像素坐标（★ 接 pix_coord_gen 的输出，不再自建计数器）----
    input  wire [10:0] I_pix_x,      // 0 ~ 1279，接 S_pix_x_2d（配 PIX_DLY=2）
    input  wire [9:0]  I_pix_y,      // 0 ~  719，接 S_pix_y_2d

    // ---- 目标颜色选择（拨码 / 按键，静态信号）----
    //      2'd0 = 红   2'd1 = 蓝   2'd2 = 绿   2'd3 = 三色取或
    input  wire [1:0]  I_color_sel,

    // ---- 输出：二值掩膜（与 O_de / O_pix_x / O_pix_y 严格同拍）----
    output reg         O_mask,       // 选中颜色的掩膜（1 = 命中）
    output reg         O_mask_r,     // 三色独立掩膜（调试 / 灵活组合用）
    output reg         O_mask_g,
    output reg         O_mask_b,

    // ---- 输出：YCbCr 分量（供 D4 特征计算取「色相均值」用）----
    output reg  [7:0]  O_y,
    output reg  [7:0]  O_cb,
    output reg  [7:0]  O_cr,

    // ---- 输出：与掩膜对齐的时序与坐标 ----
    output reg         O_de,
    output reg         O_vsync,
    output reg         O_hsync,
    output reg         O_user,
    output reg         O_last,
    output reg  [10:0] O_pix_x,      // 0 ~ IMG_WIDTH-1，de 有效时计数，帧起始清零
    output reg  [9:0]  O_pix_y,      // 0 ~ IMG_HEIGHT-1，每行末 +1，帧起始清零
    output reg         O_frame_done  // 帧末脉冲（延时对齐后给出，下游做帧末统计）
);

    //==================================================================
    // 0. 输入侧对齐（PIPE_DELAY 拍，0~15）
    //==================================================================
    // 固定 16 拍容量的延时线；PIPE_DELAY=0 时直接取原始信号（用常量三元选择，
    // 避免出现 dly[-1] 这类越界索引）。
    reg [15:0] dly_de;
    reg [15:0] dly_vsync;
    reg [15:0] dly_hsync;
    reg [15:0] dly_user;
    reg [15:0] dly_last;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dly_de    <= 16'd0;
            dly_vsync <= 16'd0;
            dly_hsync <= 16'd0;
            dly_user  <= 16'd0;
            dly_last  <= 16'd0;
        end else begin
            dly_de    <= {dly_de   [14:0], I_de};
            dly_vsync <= {dly_vsync[14:0], I_vsync};
            dly_hsync <= {dly_hsync[14:0], I_hsync};
            dly_user  <= {dly_user [14:0], I_user};
            dly_last  <= {dly_last [14:0], I_last};
        end
    end

    localparam integer PDSEL = (PIPE_DELAY <= 0) ? 0 : (PIPE_DELAY - 1);

    wire w_de    = (PIPE_DELAY <= 0) ? I_de    : dly_de   [PDSEL];
    wire w_vsync = (PIPE_DELAY <= 0) ? I_vsync : dly_vsync[PDSEL];
    wire w_hsync = (PIPE_DELAY <= 0) ? I_hsync : dly_hsync[PDSEL];
    wire w_user  = (PIPE_DELAY <= 0) ? I_user  : dly_user [PDSEL];
    wire w_last  = (PIPE_DELAY <= 0) ? I_last  : dly_last [PDSEL];

    wire [23:0] w_rgb = I_rgb;   // I_rgb 已按 PIPE_DELAY 对齐，直接使用

    //==================================================================
    // 1. 公共坐标对齐（★ 2026-10-10 改造：不再自建计数器）
    //------------------------------------------------------------------
    //  依方案B §1.3 + 对账结论：全队统一使用 pix_coord_gen.v 生成的
    //  S_pix_x / S_pix_y。本模块只做「延时对齐」，不再自建 x/y 计数器。
    //
    //  对齐推导（以"拍 0" = 输入像素 I_rgb 所在拍为基准）：
    //    · 掩膜路径：拍 0 输入 → P1 → P2 → P3 → O_mask，合计 **3 拍**。
    //    · 调用方送入的 I_pix_x **已经延时了 PIX_DLY 拍**
    //      （接 S_pix_x_2d ⇒ PIX_DLY=2）。故本模块只需再延 (3 - PIX_DLY) 拍。
    //
    //  拍数推导（以 PIX_DLY=0 为例，位级实测校准）：
    //    · 掩膜路径：I_rgb(C0) → P1(C1) → P2(C2) → O_mask(C3)，共 3 拍。
    //      ⇒ O_pix_x 必须也在 C3 给出「C0 那个像素」的坐标。
    //    · pxc 链：pxc[0] 在 C1 = C0 的 I_pix_x；pxc[1] 在 C2 = C0 的 I_pix_x；
    //              pxc[2] 在 C3 = C0 的 I_pix_x。
    //    · O_pix_x 是「输出级寄存」，在 C3 时刻取的是 **C2 时该组合值**。
    //      故要拿到「C0 的坐标」，输出级输入应取 **C2 时刻等于 C0 坐标的那一级**
    //      ⇒ 即 pxc[1]（PIX_DLY=0）。
    //
    //  ⇒ 抽头 = 1 - PIX_DLY：
    //        PIX_DLY = 2 → 输出级接 I_pix_x 本身（外部已延 2，内部再 1 → 共 3）
    //        PIX_DLY = 1 → 输出级接 pxc[0]
    //        PIX_DLY = 0 → 输出级接 pxc[1]
    //    （★ 此映射由 iverilog 逐拍实测校准，见 cnt 测试：B_px 与 A_px 同为 3 拍对齐）
    //
    //  ⚠ 历史教训：坐标延时**必须与掩膜路径严格相等**，差一拍就整帧错位
    //    （2026-10-09 位级仿真抓到的 BUG #1 就是差一拍）。
    //  ⚠ PIX_DLY 只允许 0 / 1 / 2（超出范围在本工程无意义）。
    //==================================================================
    reg [10:0] pxc [0:2];     // 坐标 x 延时链：pxc[0]=I_pix_x(延1拍), pxc[1]=延2拍
    reg [9:0]  pyc [0:2];     // 坐标 y 延时链

    integer pi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (pi = 0; pi <= 2; pi = pi + 1) begin
                pxc[pi] <= 11'd0;
                pyc[pi] <= 10'd0;
            end
        end else begin
            pxc[0] <= I_pix_x;
            pyc[0] <= I_pix_y;
            for (pi = 1; pi <= 2; pi = pi + 1) begin
                pxc[pi] <= pxc[pi-1];
                pyc[pi] <= pyc[pi-1];
            end
        end
    end

    //  按 PIX_DLY 选出「第 5 节输出级寄存的输入」（抽头 = 1 - PIX_DLY）
    wire [10:0] w_px_aligned = (PIX_DLY >= 2) ? I_pix_x :
                               (PIX_DLY == 1) ? pxc[0] :
                               /* PIX_DLY == 0 */ pxc[1];
    wire [9:0]  w_py_aligned = (PIX_DLY >= 2) ? I_pix_y :
                               (PIX_DLY == 1) ? pyc[0] :
                               /* PIX_DLY == 0 */ pyc[1];

    //==================================================================
    // 2. P1：常量乘积寄存（9 个并行，各自 ≤4 级逻辑）
    //==================================================================
    wire [7:0] r_in = w_rgb[23:16];
    wire [7:0] g_in = w_rgb[15:8];
    wire [7:0] b_in = w_rgb[7:0];

    // 无符号通道扩展为 18bit 有符号（符号位为 0），统一做有符号运算
    wire signed [17:0] r_ext = $signed({10'd0, r_in});
    wire signed [17:0] g_ext = $signed({10'd0, g_in});
    wire signed [17:0] b_ext = $signed({10'd0, b_in});

    reg signed [17:0] p_y_r,  p_y_g,  p_y_b;
    reg signed [17:0] p_cb_r, p_cb_g, p_cb_b;
    reg signed [17:0] p_cr_r, p_cr_g, p_cr_b;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_y_r <= 18'sd0; p_y_g <= 18'sd0; p_y_b <= 18'sd0;
            p_cb_r <= 18'sd0; p_cb_g <= 18'sd0; p_cb_b <= 18'sd0;
            p_cr_r <= 18'sd0; p_cr_g <= 18'sd0; p_cr_b <= 18'sd0;
        end else begin
            // Y  = 77R + 150G + 29B
            p_y_r  <=  18'sd77  * r_ext;
            p_y_g  <=  18'sd150 * g_ext;
            p_y_b  <=  18'sd29  * b_ext;
            // Cb = -43R - 85G + 128B
            p_cb_r <= -18'sd43  * r_ext;
            p_cb_g <= -18'sd85  * g_ext;
            p_cb_b <=  18'sd128 * b_ext;
            // Cr = 128R - 107G - 21B
            p_cr_r <=  18'sd128 * r_ext;
            p_cr_g <= -18'sd107 * g_ext;
            p_cr_b <= -18'sd21  * b_ext;
        end
    end

    //==================================================================
    // 3. P2：求和 + 四舍五入(+128) + 算术右移(>>>8) + 偏置(+128)
    //==================================================================
    wire signed [17:0] y_sum  = p_y_r  + p_y_g  + p_y_b;
    wire signed [17:0] cb_sum = p_cb_r + p_cb_g + p_cb_b;
    wire signed [17:0] cr_sum = p_cr_r + p_cr_g + p_cr_b;

    reg signed [9:0] q_y, q_cb, q_cr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_y  <= 10'sd0;
            q_cb <= 10'sd128;
            q_cr <= 10'sd128;
        end else begin
            q_y  <= (y_sum  + 18'sd128) >>> 8;
            q_cb <= ((cb_sum + 18'sd128) >>> 8) + 10'sd128;
            q_cr <= ((cr_sum + 18'sd128) >>> 8) + 10'sd128;
        end
    end

    //==================================================================
    // 4. P3：先限幅，再做色度双阈值比较 → 掩膜
    //==================================================================
    //  ⚠ 顺序不能反（2026-10-09 位级仿真抓到的第二个坑）：
    //     q_cb / q_cr 的理论值域是 [1,256]，而阈值参数最大只能填 255。
    //     若拿**未限幅**的 q_* 去比较，最饱和的纯红(Cr=256)、纯蓝(Cb=256)
    //     会因 `256 <= 255` 不成立而被判为「不命中」——正好把最容易演示的
    //     高饱和目标漏掉。因此必须**先限幅到 0~255 再比较**。
    //
    //  q_y / q_cb / q_cr 经 +128 偏置后恒为非负 → 统一按无符号比较，
    //  既避开 signed/unsigned 混比的符号扩展陷阱，也让阈值参数保持 8bit 原样。
    wire [7:0] w_y_out  = (q_y  > 10'sd255) ? 8'd255 : (q_y  < 10'sd0) ? 8'd0 : q_y [7:0];
    wire [7:0] w_cb_out = (q_cb > 10'sd255) ? 8'd255 : (q_cb < 10'sd0) ? 8'd0 : q_cb[7:0];
    wire [7:0] w_cr_out = (q_cr > 10'sd255) ? 8'd255 : (q_cr < 10'sd0) ? 8'd0 : q_cr[7:0];

    wire w_gate = (w_y_out >= Y_MIN);

    //------------------------------------------------------------------
    //  ★ v2 曝光自适应色相判据（默认） / 兼容旧固定窗口（USE_ABS_WINDOW=1）
    //------------------------------------------------------------------
    //  以 (128,128) 为原点的带符号色差。w_cb_out/w_cr_out ∈ [0,255]，
    //  故 |dcb|、|dcr| ∈ [0,128]，各 8 bit 足够。
    wire signed [8:0] w_dcb = $signed({1'b0, w_cb_out}) - 9'sd128;
    wire signed [8:0] w_dcr = $signed({1'b0, w_cr_out}) - 9'sd128;
    wire [7:0] w_adcb = w_dcb[8] ? (~w_dcb[7:0] + 8'd1) : w_dcb[7:0];   // |dcb|
    wire [7:0] w_adcr = w_dcr[8] ? (~w_dcr[7:0] + 8'd1) : w_dcr[7:0];   // |dcr|

    //  饱和度幅度 |dcb|+|dcr| ∈ [0,256] → 9 bit
    wire [8:0] w_mag = {1'b0, w_adcb} + {1'b0, w_adcr};
    wire       w_mag_ok = (w_mag >= {1'b0, MAG_TH});

    //  比值判据：|a|*10 >= |b|*RATIO10  （|a| ≤ 128 ⇒ |a|*10 ≤ 1280，11 bit 够）
    wire [10:0] w_rcb10 = {3'b0, w_adcb} * 11'd10;
    wire [10:0] w_rcr10 = {3'b0, w_adcr} * 11'd10;
    wire [10:0] w_lim_cb = {6'b0, RATIO10} * {3'b0, w_adcb};
    wire [10:0] w_lim_cr = {6'b0, RATIO10} * {3'b0, w_adcr};

    //  自适应判据：方向 + 幅度 + 比值
    wire w_ad_r = w_mag_ok && (w_dcr > 9'sd0) && (w_dcb <= 9'sd0) && (w_rcr10 >= w_lim_cb);
    wire w_ad_b = w_mag_ok && (w_dcb > 9'sd0) && (w_dcr <= 9'sd0) && (w_rcb10 >= w_lim_cr);
    wire w_ad_g = w_mag_ok && (w_dcb < 9'sd0) && (w_dcr <  9'sd0);

    //  固定绝对窗口（兼容路径）
    wire w_ab_r = w_gate && (w_cb_out >= RED_CB_LO)   && (w_cb_out <= RED_CB_HI)   &&
                            (w_cr_out >= RED_CR_LO)   && (w_cr_out <= RED_CR_HI);
    wire w_ab_b = w_gate && (w_cb_out >= BLUE_CB_LO)  && (w_cb_out <= BLUE_CB_HI)  &&
                            (w_cr_out >= BLUE_CR_LO)  && (w_cr_out <= BLUE_CR_HI);
    wire w_ab_g = w_gate && (w_cb_out >= GREEN_CB_LO) && (w_cb_out <= GREEN_CB_HI) &&
                            (w_cr_out >= GREEN_CR_LO) && (w_cr_out <= GREEN_CR_HI);

    //  运行时可切（USE_ABS_WINDOW 是 parameter，综合时会常量折叠，不产生额外逻辑）
    wire w_mask_r = USE_ABS_WINDOW ? w_ab_r : w_ad_r;
    wire w_mask_b = USE_ABS_WINDOW ? w_ab_b : w_ad_b;
    wire w_mask_g = USE_ABS_WINDOW ? w_ab_g : w_ad_g;

    // 目标颜色选择（静态信号，两拍同步防亚稳态）
    reg [1:0] sel_s1, sel_s2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sel_s1 <= 2'd0;
            sel_s2 <= 2'd0;
        end else begin
            sel_s1 <= I_color_sel;
            sel_s2 <= sel_s1;
        end
    end

    reg w_mask_sel;
    always @(*) begin
        case (sel_s2)
            2'd0:    w_mask_sel = w_mask_r;
            2'd1:    w_mask_sel = w_mask_b;
            2'd2:    w_mask_sel = w_mask_g;
            default: w_mask_sel = w_mask_r | w_mask_b | w_mask_g;
        endcase
    end

    //（限幅与掩膜比较见上，必须先限幅再比较）

    //==================================================================
    // 5. 输出寄存（与 3 拍时序延时线对齐）
    //==================================================================
    //  ⚠ 这里曾经踩过一次「差一拍」的坑（2026-10-09 位级仿真抓到）：
    //     用 3bit 移位寄存器 ([2:0]) 再让 O_de <= d3_de[2]，等于串了 4 级寄存器，
    //     比坐标/掩膜的 3 级多出一拍 ⇒ 每个像素都错位一个，首像素被丢。
    //     ⇒ 时序标志统一用 **2bit 移位 + 取 [1]**，正好 3 级：
    //        w_de(C0) → d3_de[0](C1) → d3_de[1](C2) → O_de(C3)
    //     坐标由第 1 节的延时链按 PIX_DLY 抽头给出（已对齐到同一拍），不再另加。
    reg [1:0] d3_de, d3_vsync, d3_hsync, d3_user, d3_last;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d3_de    <= 2'd0;
            d3_vsync <= 2'd0;
            d3_hsync <= 2'd0;
            d3_user  <= 2'd0;
            d3_last  <= 2'd0;

            O_mask   <= 1'b0;
            O_mask_r <= 1'b0;
            O_mask_g <= 1'b0;
            O_mask_b <= 1'b0;
            O_y      <= 8'd0;
            O_cb     <= 8'd128;
            O_cr     <= 8'd128;

            O_de    <= 1'b0;
            O_vsync <= 1'b0;
            O_hsync <= 1'b0;
            O_user  <= 1'b0;
            O_last  <= 1'b0;
            O_pix_x <= 11'd0;
            O_pix_y <= 10'd0;
            O_frame_done <= 1'b0;
        end else begin
            // --- 时序延时线（2 级寄存器 + 输出寄存器 = 3 拍）---
            d3_de    <= {d3_de   [0], w_de};
            d3_vsync <= {d3_vsync[0], w_vsync};
            d3_hsync <= {d3_hsync[0], w_hsync};
            d3_user  <= {d3_user [0], w_user};
            d3_last  <= {d3_last [0], w_last};

            // --- 数据输出 ---
            O_mask   <= w_mask_sel;
            O_mask_r <= w_mask_r;
            O_mask_g <= w_mask_g;
            O_mask_b <= w_mask_b;
            O_y      <= w_y_out;
            O_cb     <= w_cb_out;
            O_cr     <= w_cr_out;

            // --- 时序与坐标输出 ---
            O_de    <= d3_de[1];
            O_vsync <= d3_vsync[1];
            O_hsync <= d3_hsync[1];
            O_user  <= d3_user[1];
            O_last  <= d3_last[1];
            O_pix_x <= w_px_aligned;    // 第 1 节延时链已按 PIX_DLY 对齐到同一拍
            O_pix_y <= w_py_aligned;

            // 帧末脉冲：对齐后的 I_user（此刻上一帧像素已全部流出流水线）
            O_frame_done <= d3_user[1];
        end
    end

endmodule
