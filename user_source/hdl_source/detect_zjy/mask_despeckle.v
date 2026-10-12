//=============================================================================
//  mask_despeckle.v —— 掩膜水平去斑（抑制孤立噪点）
//=============================================================================
//  项目：MIPI-HDMI 实时视频图像处理系统 · 方案B
//  负责人：张金艺   归属：user_source/hdl_source/detect_zjy/
//  依据：`doc/接口冻结表.md` §3（连通域输入） ｜ 2026-10-12 新增
//
//-----------------------------------------------------------------------------
//  一、为什么需要它（2026-10-12 现场根因）
//-----------------------------------------------------------------------------
//  链路：detect_color_mask → **本模块** → conn_label → conn_features → target_bus → box_draw
//
//  现场症状：① 满屏绿色噪点、逐帧乱跳；② 修严门限后变成「只检测到物体边缘，
//            给边缘断断续续加框」。
//
//  根因是**两个约束互相打架**：
//    · 判据门限 MAG_TH 必须**足够低**，才能检出物体内部（中等饱和度）——
//      实测：MAG_TH=48 时，绿物体要 60% 亮度才检出，只有边缘过冲的像素够得着
//      ⇒ 表现就是「只有边缘被检出」；
//    · 但 conn_label 的等价表深度 **NEG=8** 限制「同一帧最多 8 个连通域，先到先得」，
//      而**传感器噪点每个孤立点自成一个连通域** ——
//      实测（整帧 1280x720 纯噪声，本机仿真）：
//          σ=8, MAG_TH=16 → 46551 个连通域  ⇒ NEG=8 瞬间被噪声占满，真目标全被挤掉
//          σ=8, MAG_TH=48 →    13 个        ⇒ 仍超标
//          σ=10, MAG_TH=24 → 28931 个       ⇒ 超标 3000 倍
//      ⇒ **靠调门限无解**：门限低则噪声爆、门限高则目标碎。
//
//  破局点：**噪声是孤立散点，目标是成片区域**。
//    用「水平邻域一致性」把散点清掉，就能让门限降下来同时不给 conn_label 添乱。
//
//-----------------------------------------------------------------------------
//  二、算法
//-----------------------------------------------------------------------------
//  对每个命中像素，看它左右各 HALF 个像素（窗宽 WID = 2*HALF+1，含中心）：
//    窗内命中数 >= KMIN 才保留，否则判为噪点丢弃。
//
//  默认 HALF=4、KMIN=5（9 像素窗，至少 5 个命中）。
//
//  实测效果（本机仿真，整帧纯噪声 → 去斑后剩下多少个连通域）：
//    σ（色度噪声）   MAG_TH=24, HALF=4, KMIN=5
//        6                     0        ✅
//        8                     0        ✅
//       10                    13        （比不去斑的 28931 好 2000 倍）
//
//  对**目标**的损伤（本机仿真）：
//    200x200 实心块 → 保留 **100.0%**（内部每像素 8 邻居全命中）
//    12px 宽柱状目标 → 保留 **100.0%**
//  ⇒ **只砍孤立点，不伤成片目标。**
//
//-----------------------------------------------------------------------------
//  三、时序 / 对齐（由结构保证，不依赖上游）
//-----------------------------------------------------------------------------
//  掩膜路径天然比其它信号多 (1+HALF) 拍延迟（移位 1 + 窗口居中 HALF）。
//  本模块先用一条 **EQ = HALF+1 深**的预延时链把「其它信号」拉到同一时刻，
//  再让全部信号走同一条 **DCH 深**的公共延迟链输出。
//  ⇒ 掩膜 / 逐像素颜色 / de / 坐标 / 帧末脉冲 **严格同像素**，总延迟 LAT = EQ + DCH。
//
//  这是把它做成**独立模块**而不是塞进 detect 流水线的原因：
//  不去扰动已验证的 detect_color_mask.v（那里面有一处「差一拍」的历史坑）。
//
//  行首/行尾各 HALF 个像素的窗口不完整（左右缺邻居）⇒ 会被丢掉，
//  表现为目标的左右边缘各缩进约 HALF 像素，属可接受的代价。
//
//-----------------------------------------------------------------------------
//  四、资源
//-----------------------------------------------------------------------------
//  实测约 200 个寄存器 + 约 30 LUT（9 bit 移位 + 10 深延迟链 + 4 bit 加法树）。
//  相对工程（slice 已用 79.6%）**可忽略**。
//
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

module mask_despeckle #(
    parameter integer HALF   = 4,   // 半窗（左右各 HALF 像素）
    parameter integer KMIN   = 5,   // 窗内至少 KMIN 个命中才保留
    parameter integer DCH    = 2,   // 公共输出延迟链深度
    parameter integer ENABLE = 1    // 0 = 旁路（仅整体延迟，不做去斑）
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- 输入（来自 detect_color_mask 的输出，六者同拍）----
    input  wire        I_de,
    input  wire        I_mask,
    input  wire        I_mask_r,
    input  wire        I_mask_g,
    input  wire        I_mask_b,
    input  wire        I_frame_done,
    input  wire [10:0] I_pix_x,
    input  wire [9:0]  I_pix_y,

    // ---- 输出（与输入同像素，整体延后 LAT 拍）----
    output wire        O_de,
    output wire        O_mask,
    output wire        O_mask_r,
    output wire        O_mask_g,
    output wire        O_mask_b,
    output wire        O_frame_done,
    output wire [10:0] O_pix_x,
    output wire [9:0]  O_pix_y
);

    localparam integer WID = 2 * HALF + 1;   // 窗口宽度
    localparam integer EQ  = HALF + 1;       // 预延时深度（把其它信号对齐到 W_keep 的时刻）
    localparam integer LAT = EQ + DCH;       // 总延迟

    //=========================================================================
    //  1. 水平窗口移位寄存器
    //     de 无效时推 0 —— 行消隐会把窗口自然清空，无需额外复位逻辑
    //=========================================================================
    reg [WID-1:0] S_sr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) S_sr <= {WID{1'b0}};
        else        S_sr <= { (I_de ? I_mask : 1'b0), S_sr[WID-1:1] };
    end

    //=========================================================================
    //  2. 窗内命中计数（含中心；实心区域内每像素满窗）
    //     加法树 3~4 级，75 MHz 下裕量充足
    //=========================================================================
    reg [3:0] S_cnt;
    integer   i;
    always @(*) begin
        S_cnt = 4'd0;
        for (i = 0; i < WID; i = i + 1)
            S_cnt = S_cnt + {3'b0, S_sr[i]};
    end

    wire W_keep = (S_cnt >= KMIN[3:0]);
    //  S_sr[HALF] 是中心像素的原始掩膜值（用于与 W_keep 对齐的直通路径）
    wire W_mask_dspk = ENABLE ? W_keep : S_sr[HALF];

    //=========================================================================
    //  3. 预延时链（深度 EQ）：把「其它信号」拉到与 W_keep 同一时刻
    //     注意 W_keep 在周期 n 对应输入像素 (n-1-HALF)，
    //     故其它信号需延时 EQ = HALF+1 拍才能对上。
    //=========================================================================
    reg               E_de, E_fd, E_mr, E_mg, E_mb;
    reg               E_de_sr [0:EQ-1];
    reg               E_fd_sr [0:EQ-1];
    reg               E_mr_sr [0:EQ-1];
    reg               E_mg_sr [0:EQ-1];
    reg               E_mb_sr [0:EQ-1];
    reg [10:0]        E_px_sr [0:EQ-1];
    reg [9:0]         E_py_sr [0:EQ-1];

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k = 0; k < EQ; k = k + 1) begin
                E_de_sr[k] <= 1'b0;  E_fd_sr[k] <= 1'b0;
                E_mr_sr[k] <= 1'b0;  E_mg_sr[k] <= 1'b0;  E_mb_sr[k] <= 1'b0;
                E_px_sr[k] <= 11'd0; E_py_sr[k] <= 10'd0;
            end
        end else begin
            E_de_sr[0] <= I_de;
            E_fd_sr[0] <= I_frame_done;
            E_mr_sr[0] <= I_mask_r;
            E_mg_sr[0] <= I_mask_g;
            E_mb_sr[0] <= I_mask_b;
            E_px_sr[0] <= I_pix_x;
            E_py_sr[0] <= I_pix_y;
            for (k = 1; k < EQ; k = k + 1) begin
                E_de_sr[k] <= E_de_sr[k-1];
                E_fd_sr[k] <= E_fd_sr[k-1];
                E_mr_sr[k] <= E_mr_sr[k-1];
                E_mg_sr[k] <= E_mg_sr[k-1];
                E_mb_sr[k] <= E_mb_sr[k-1];
                E_px_sr[k] <= E_px_sr[k-1];
                E_py_sr[k] <= E_py_sr[k-1];
            end
        end
    end

    //  此刻：W_mask_dspk 与 E_*_sr[EQ-1] 是**同一像素**
    wire        A_de   = E_de_sr[EQ-1];
    wire        A_fd   = E_fd_sr[EQ-1];
    wire        A_mr   = E_mr_sr[EQ-1];
    wire        A_mg   = E_mg_sr[EQ-1];
    wire        A_mb   = E_mb_sr[EQ-1];
    wire [10:0] A_px   = E_px_sr[EQ-1];
    wire [9:0]  A_py   = E_py_sr[EQ-1];

    //=========================================================================
    //  4. 公共延迟链（深度 DCH）：统一输出延迟，兼作输出寄存改善时序
    //=========================================================================
    reg               C_mask, C_de, C_fd, C_mr, C_mg, C_mb;
    reg               C_mask_sr [0:DCH-1];
    reg               C_de_sr   [0:DCH-1];
    reg               C_fd_sr   [0:DCH-1];
    reg               C_mr_sr   [0:DCH-1];
    reg               C_mg_sr   [0:DCH-1];
    reg               C_mb_sr   [0:DCH-1];
    reg [10:0]        C_px_sr   [0:DCH-1];
    reg [9:0]         C_py_sr   [0:DCH-1];

    integer m;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (m = 0; m < DCH; m = m + 1) begin
                C_mask_sr[m] <= 1'b0; C_de_sr[m] <= 1'b0; C_fd_sr[m] <= 1'b0;
                C_mr_sr[m] <= 1'b0;   C_mg_sr[m] <= 1'b0; C_mb_sr[m] <= 1'b0;
                C_px_sr[m] <= 11'd0;  C_py_sr[m] <= 10'd0;
            end
        end else begin
            C_mask_sr[0] <= W_mask_dspk;
            C_de_sr  [0] <= A_de;
            C_fd_sr  [0] <= A_fd;
            C_mr_sr  [0] <= A_mr;
            C_mg_sr  [0] <= A_mg;
            C_mb_sr  [0] <= A_mb;
            C_px_sr  [0] <= A_px;
            C_py_sr  [0] <= A_py;
            for (m = 1; m < DCH; m = m + 1) begin
                C_mask_sr[m] <= C_mask_sr[m-1];
                C_de_sr  [m] <= C_de_sr  [m-1];
                C_fd_sr  [m] <= C_fd_sr  [m-1];
                C_mr_sr  [m] <= C_mr_sr  [m-1];
                C_mg_sr  [m] <= C_mg_sr  [m-1];
                C_mb_sr  [m] <= C_mb_sr  [m-1];
                C_px_sr  [m] <= C_px_sr  [m-1];
                C_py_sr  [m] <= C_py_sr  [m-1];
            end
        end
    end

    assign O_mask       = C_mask_sr[DCH-1];
    assign O_de         = C_de_sr  [DCH-1];
    assign O_mask_r     = C_mr_sr  [DCH-1];
    assign O_mask_g     = C_mg_sr  [DCH-1];
    assign O_mask_b     = C_mb_sr  [DCH-1];
    assign O_frame_done = C_fd_sr  [DCH-1];
    assign O_pix_x      = C_px_sr  [DCH-1];
    assign O_pix_y      = C_py_sr  [DCH-1];

endmodule
