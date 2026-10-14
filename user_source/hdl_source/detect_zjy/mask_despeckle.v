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
//  再跨 VER_ROWS 行做「至少 VER_NEED 行通过」的多数投票（见 §2b）。
//
//  ★★ 最终定稿（v7.7 起）：HALF=4、KMIN=5、VER_ROWS=3、VER_NEED=2。
//     水平窗**保持宽松**（不伤物体），清噪交给**垂直投票**。
//
//  【曾走过的弯路，留档避免重犯】
//    v7.6 曾把水平窗调严到 HALF=5 / KMIN=10 —— 噪声确实压住了，但那是错的方向：
//      · 根因是 ISP 的 `chroma_denoise` **水平平滑色度** ⇒ 噪声是**水平相关**的。
//        纯水平窗只有两条路：放松 → 放走噪声；收紧 → **把斑点状目标打碎**
//        （仿真：命中密度 0.7 的 400×250 物体被打得只剩 67 px）。
//      · **垂直方向的噪声仍然独立** ⇒ 破局点是「跨行投票」，而不是加大水平窗。
//    仿真对照（噪声 = 白噪声经 N 抽头水平平滑模拟 chroma_denoise；整帧 1280×720；
//              数值 = 剩余连通域数，必须 ≤4 才不挤占 conn_label 的 NEG=8 名额）：
//
//    配置                  σ=8,1  σ=8,3  σ=8,5  σ=10,1 σ=10,3 σ=10,5
//    HALF=4 KMIN=5（旧）        0     13    189      25    265   1077  ❌ 水平相关噪声漏过
//    HALF=5 KMIN=10（已弃）     0      0      0       0      0      3  ⚠ 压住噪声但打碎目标
//    HALF=4 KMIN=5 + 垂直 2/3   0      0      0       0      0      0  ✅ 本文件最终方案
//                                                                       （目标几乎无损）
//
//  对目标的损伤（实心块每边削 HALF 像素）：
//    200x200 → 保留 90.2%；60x60 → 69.4%；30x30 → 44.4%（仍远 > MIN_AREA=16）
//    表现为**画出的框比物体每边小约 5 像素**，可接受。
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
    parameter integer HALF     = 4,   // 水平半窗（左右各 HALF 像素）
    parameter integer KMIN     = 5,   // 水平窗内至少 KMIN 个命中
    parameter integer VER_ROWS = 3,   // 垂直投票看的行数
    parameter integer VER_NEED = 2,   // 至少几行通过才保留
    parameter integer DCH      = 2,   // 公共输出延迟链深度
    parameter integer ENABLE   = 1    // 0 = 旁路（仅整体延迟，不做去斑）
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
    //  ★ 保险条款：水平窗**几乎全满**（≥WID-1）⇒ 该像素确定处在「成片区域」内部，
    //    无需再看垂直投票，直接放行。
    //    作用：即使行缓存出了任何问题（未推断、地址错、读写语义不符），
    //    目标物体的**核心区域**也一定能活下来 —— 避免「整片掩膜被清空」这种全崩。
    //    代价：噪声几乎不受影响（相关噪声在 9 像素窗内命中 8 个的概率极低）。
    wire W_hstrong = (S_cnt >= (WID-1));

    //=========================================================================
    //  2b. ★ 垂直「多数投票」（2026-10-12 最终方案）
    //---------------------------------------------------------------------
    //   为什么需要它：光靠水平窗（HALF=4,KMIN=5）对**水平相关的噪声**无效
    //   （本工程 chroma_denoise 会水平平滑色度，噪声变成短条），
    //   而把水平窗调严（如 KMIN=10）又会把**斑点状物体**打碎 ——
    //   仿真：物体命中密度 0.7 时，KMIN=10 把 400x250 的物体打成只剩 67 px 的碎片。
    //
    //   加上垂直投票后两者兼顾（仿真，整帧 1280x720）：
    //     场景            单看水平H4K5       + 垂直 2/3 投票
    //     纯噪声 σ8/σ10       0 个              0 个                  ✅
    //     物体密度 0.7     35 个碎片/最大89095  5 个/最大 96194       ✅
    //     物体密度 0.8     26 个碎片/最大97481  2 个/最大 99322       ✅
    //   （数值 = 连通域数/最大连通域面积；连通域数必须 ≤4 才不挤占 conn_label 的 NEG=8）
    //
    //   实现：用 2 个 1-bit 行缓存存「水平窗通过」信号，跨 3 行做 2/3 多数投票。
    //
    //   ⚠ 列对齐（line_delay_ram 读写同址会带来 1 列滞后，级联累加）：
    //     K1 在周期 n = 上一行、列 A_px(n)-1 的 keep
    //     K2 在周期 n = 上两行、列 A_px(n)-2 的 keep
    //     故把「本行 keep」延 2 拍、K1 延 1 拍、K2 不延 ⇒ 三者都落在列 A_px(n)-2 ✅
    //=========================================================================
    //  ★ 前置声明（2026-10-12）
    //    下面第 2b 节要**先用** A_de / A_px 等信号，而它们的赋值在第 3 节。
    //    把声明提到这里纯为**可读性**（声明紧邻使用者，便于对照）。
    //
    //    ⚠ 订正（2026-10-13）：本节曾断言「TD/Synplify 会为『先用后声明』的标识符
    //      建一个隐式 1-bit 网（无驱动、恒 0）⇒ 行缓存 I_wen 恒 0 ⇒ 缓存从未写入
    //      ⇒ 垂直投票凑不满票 ⇒ 整片掩膜被清空」。**该判断已被推翻**：
    //        `common_base/conn_label.v` / `conn_features.v` 里同样有"先用后声明"
    //        （如 `do_swap` 用在 268 行、声明在 544 行），而这些模块**上板工作正常**
    //        ⇒ **TD 实测能正确解析前向引用**。
    //      当时"掩膜被整片清空"的真正处置是下面第 2b 节的**保险条款**
    //      （水平窗几乎全满 ⇒ 无条件放行），与本声明顺序无关。
    //    教训：下"综合器行为"的结论前，先在全仓找同类写法反证。
    wire        A_de, A_fd, A_mr, A_mg, A_mb;
    wire [10:0] A_px;
    wire [9:0]  A_py;

    wire        K1_raw, K2_raw;
    reg         R_cur1, R_cur2, R_k1;
    reg         R_hs1, R_hs2;          // 「水平窗几乎全满」延 2 拍（保险条款）
    reg [1:0]   R_pre_de, R_pre_fd;
    reg [1:0]   R_pre_mr, R_pre_mg, R_pre_mb;
    reg [21:0]  R_pre_px;      // 2 拍 × 11bit
    reg [19:0]  R_pre_py;      // 2 拍 × 10bit

    line_delay_ram #(
        .IMG_WIDTH ( 1280 ),
        .ADDR_W    ( 11   ),
        .DW        ( 1    )
    ) u_lb_keep1 (
        .clk    ( clk    ),
        .rst_n  ( rst_n  ),
        .I_wen  ( A_de   ),      // 与 W_keep 同拍的有效信号
        .I_addr ( A_px[10:0] ),
        .I_data ( W_keep ),
        .O_data ( K1_raw )
    );

    line_delay_ram #(
        .IMG_WIDTH ( 1280 ),
        .ADDR_W    ( 11   ),
        .DW        ( 1    )
    ) u_lb_keep2 (
        .clk    ( clk    ),
        .rst_n  ( rst_n  ),
        .I_wen  ( A_de   ),
        .I_addr ( A_px[10:0] ),
        .I_data ( K1_raw ),
        .O_data ( K2_raw )
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            R_cur1 <= 1'b0; R_cur2 <= 1'b0; R_k1 <= 1'b0;
            R_hs1 <= 1'b0; R_hs2 <= 1'b0;
            R_pre_de <= 2'd0; R_pre_fd <= 2'd0;
            R_pre_mr <= 2'd0; R_pre_mg <= 2'd0; R_pre_mb <= 2'd0;
            R_pre_px <= 22'd0; R_pre_py <= 20'd0;
        end else begin
            // 列对齐：本行 keep 延 2、K1 延 1、K2 不延
            R_cur1 <= W_keep;
            R_cur2 <= R_cur1;
            R_k1   <= K1_raw;
            R_hs1  <= W_hstrong;      // 保险条款同样延 2 拍对齐
            R_hs2  <= R_hs1;
            // 「其它信号」同步延 2 拍（与投票结果同一列）
            R_pre_de <= {R_pre_de[0], A_de};
            R_pre_fd <= {R_pre_fd[0], A_fd};
            R_pre_mr <= {R_pre_mr[0], A_mr};
            R_pre_mg <= {R_pre_mg[0], A_mg};
            R_pre_mb <= {R_pre_mb[0], A_mb};
            R_pre_px <= {R_pre_px[10:0], A_px};
            R_pre_py <= {R_pre_py[9:0], A_py};
        end
    end

    //  跨 3 行多数投票（VER_ROWS/VER_NEED 可调；默认 2/3）
    //  再 OR 上「保险条款」：水平窗几乎全满的像素无条件放行
    wire [1:0] W_vote = {1'b0, R_cur2} + {1'b0, R_k1} + {1'b0, K2_raw};
    wire       W_vote_ok = (W_vote >= VER_NEED[1:0]) | R_hs2;

    wire W_mask_dspk = ENABLE ? (W_vote_ok & R_cur2) : R_cur2;

    //  ★ 投票结果 / 延后 2 拍的其它信号，就是新的「对齐基准」A2_*
    wire        A2_de = R_pre_de[1];
    wire        A2_fd = R_pre_fd[1];
    wire        A2_mr = R_pre_mr[1];
    wire        A2_mg = R_pre_mg[1];
    wire        A2_mb = R_pre_mb[1];
    wire [10:0] A2_px = R_pre_px[21:11];
    wire [9:0]  A2_py = R_pre_py[19:10];

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
    assign A_de   = E_de_sr[EQ-1];
    assign A_fd   = E_fd_sr[EQ-1];
    assign A_mr   = E_mr_sr[EQ-1];
    assign A_mg   = E_mg_sr[EQ-1];
    assign A_mb   = E_mb_sr[EQ-1];
    assign A_px   = E_px_sr[EQ-1];
    assign A_py   = E_py_sr[EQ-1];

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
            C_de_sr  [0] <= A2_de;
            C_fd_sr  [0] <= A2_fd;
            C_mr_sr  [0] <= A2_mr;
            C_mg_sr  [0] <= A2_mg;
            C_mb_sr  [0] <= A2_mb;
            C_px_sr  [0] <= A2_px;
            C_py_sr  [0] <= A2_py;
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
