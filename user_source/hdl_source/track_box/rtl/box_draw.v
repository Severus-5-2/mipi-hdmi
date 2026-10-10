//=====================================================================
//  box_draw.v  ——  目标框 / 十字准星 叠加绘制（开发方案B 第 3.1 节 进阶2）
//---------------------------------------------------------------------
//  归属：张铭晨（进阶2 目标跟踪与信息标注 —— D1~D3 里程碑）
//  D1 目标（方案B 3.3 节第一行）：
//      「画框模块 + OSD 坐标显示（先画固定框，验证坐标系统）」
//      本模块即那一块画框，SELF_TEST=1 时画 4 个固定框做坐标标定。
//
//  功能
//    · 最多 4 个目标框，每框 2px 边框（BORDER_W 可调），框内不填充
//    · 十字准星（固定参考点，用于肉眼标定坐标）
//    · 与像素数据严格对齐的两级流水（见下方「时序对齐」）
//
//  时序对齐（★照抄 hdmi_mixer.v 的成熟写法，别自己发明）
//    第 1 拍：用 I_pix_x_1d / I_pix_y_1d 做「区域比较」，结果寄存成
//             S_hit_r / S_color_r；
//    第 2 拍：寄存后的 S_hit_r 与 I_pix_x_2d 严格同像素（因为 _2d 比 _1d 晚 1 拍），
//             所以输出级只剩一个 mux，关键路径 = 比较器(1~2 级) + 或与树(2 级)，安全。
//    ⇒ O_hit / O_color 比 I_de 晚 2 拍，与 hdmi_mixer 的 S_osd_hit 同一舞台。
//
//  位级验证
//    sim/bit_sim_box_draw.py   独立镜像本模块逻辑（边框判定 / 优先级 / 流水）
//    tb/tb_box_draw.v          iverilog 自检 testbench，输出逐像素比对结果
//=====================================================================
module box_draw #(
    //------------------------------------------------------------------
    //  SELF_TEST
    //    1 = 忽略 I_box_* 总线，画 4 个写死的固定框（D1 验证坐标系统用）
    //    0 = 用 I_box_* 总线（D4 起接入张金艺的检测结果）
    //  固定框坐标（1280x720 坐标系）：
    //    B0 (  64, 64)-( 256, 256)   左上，整框检验
    //    B1 (1088, 64)-(1279, 256)   ★右边框贴屏边 = 检验边缘裁剪
    //    B2 (  64,464)-( 256, 656)   左下，与 B0 关于屏心对称
    //      （原写 655 → 高度 192，比 B0 矮 1 px，与「对称」自检项矛盾；
    //        2026-10-10 修正为 656，使 B2 = B0 关于行区间中心镜像
    //        [64,256] ↔ [720-256, 720-64] = [464,656]）
    //    B3 ( 510,290)-( 770, 430)   ★框心正好 (640,360) = 屏幕中心
    //  十字准星默认 (640,360)，正好落在 B3 框心
    //  ⇒ 上板时「准星在 B3 正中心」+「OSD 读数 X:0640 Y:0360」互相印证，
    //    一眼就能判断坐标系统是否对齐（这就是 D1 要验的东西）。
    //------------------------------------------------------------------
    parameter integer SELF_TEST  = 1,
    parameter integer BORDER_W   = 2,
    parameter integer CROSS_HALF = 16,
    parameter [23:0]  COLOR_BOX0  = 24'hff0000,   // 红
    parameter [23:0]  COLOR_BOX1  = 24'h00ff00,   // 绿
    parameter [23:0]  COLOR_BOX2  = 24'h00a0ff,   // 蓝
    parameter [23:0]  COLOR_BOX3  = 24'hfff200,   // 黄（与现有 OSD 文字同色）
    parameter [23:0]  COLOR_CROSS = 24'hffffff    // 白
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    // ---- 像素流（与 I_de 同拍）----
    input  wire        I_de,
    input  wire [10:0] I_pix_x,
    input  wire [9:0]  I_pix_y,

    // ---- 目标框总线（SELF_TEST=1 时忽略；坐标系 0..1279 / 0..719）----
    input  wire [3:0]       I_box_valid,
    input  wire [4*11-1:0]  I_box_xmin,     // [10:0] 第 0 框, [21:11] 第 1 框 ...
    input  wire [4*11-1:0]  I_box_xmax,
    input  wire [4*10-1:0]  I_box_ymin,
    input  wire [4*10-1:0]  I_box_ymax,

    // ---- 十字准星 ----
    input  wire        I_cross_en,
    input  wire [10:0] I_cross_x,
    input  wire [9:0]  I_cross_y,

    // ---- 叠加输出（比 I_de 晚 2 拍；O_hit 已用延 2 拍的 de 门控）----
    output wire        O_hit,
    output wire [23:0] O_color,
    output wire        O_de
);

    //==================================================================
    // 1. 坐标与 de 的两级流水（与 hdmi_mixer 的 S_x_1d/S_x_2d 同款）
    //==================================================================
    reg [10:0] S_x_1d, S_x_2d;
    reg [9:0]  S_y_1d, S_y_2d;
    reg        S_de_1d, S_de_2d;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x_1d <= 11'd0;  S_x_2d <= 11'd0;
            S_y_1d <= 10'd0;  S_y_2d <= 10'd0;
            S_de_1d <= 1'b0;  S_de_2d <= 1'b0;
        end
        else begin
            S_x_1d <= I_pix_x;   S_x_2d <= S_x_1d;
            S_y_1d <= I_pix_y;   S_y_2d <= S_y_1d;
            S_de_1d <= I_de;     S_de_2d <= S_de_1d;
        end
    end

    //==================================================================
    // 2. 有效框坐标
    //    SELF_TEST=1 时整体替换为固定框；因为 SELF_TEST 是 parameter，
    //    综合时 mux 会被常量折叠掉，不产生额外逻辑。
    //==================================================================
    wire [10:0] B0_XMIN = SELF_TEST ? 11'd64   : I_box_xmin[0*11 +: 11];
    wire [10:0] B0_XMAX = SELF_TEST ? 11'd256  : I_box_xmax[0*11 +: 11];
    wire [9:0]  B0_YMIN = SELF_TEST ? 10'd64   : I_box_ymin[0*10 +: 10];
    wire [9:0]  B0_YMAX = SELF_TEST ? 10'd256  : I_box_ymax[0*10 +: 10];
    wire        B0_EN   = SELF_TEST ? 1'b1     : I_box_valid[0];

    wire [10:0] B1_XMIN = SELF_TEST ? 11'd1088 : I_box_xmin[1*11 +: 11];
    wire [10:0] B1_XMAX = SELF_TEST ? 11'd1279 : I_box_xmax[1*11 +: 11];
    wire [9:0]  B1_YMIN = SELF_TEST ? 10'd64   : I_box_ymin[1*10 +: 10];
    wire [9:0]  B1_YMAX = SELF_TEST ? 10'd256  : I_box_ymax[1*10 +: 10];
    wire        B1_EN   = SELF_TEST ? 1'b1     : I_box_valid[1];

    wire [10:0] B2_XMIN = SELF_TEST ? 11'd64   : I_box_xmin[2*11 +: 11];
    wire [10:0] B2_XMAX = SELF_TEST ? 11'd256  : I_box_xmax[2*11 +: 11];
    wire [9:0]  B2_YMIN = SELF_TEST ? 10'd464  : I_box_ymin[2*10 +: 10];
    wire [9:0]  B2_YMAX = SELF_TEST ? 10'd656  : I_box_ymax[2*10 +: 10];
    wire        B2_EN   = SELF_TEST ? 1'b1     : I_box_valid[2];

    wire [10:0] B3_XMIN = SELF_TEST ? 11'd510  : I_box_xmin[3*11 +: 11];
    wire [10:0] B3_XMAX = SELF_TEST ? 11'd770  : I_box_xmax[3*11 +: 11];
    wire [9:0]  B3_YMIN = SELF_TEST ? 10'd290  : I_box_ymin[3*10 +: 10];
    wire [9:0]  B3_YMAX = SELF_TEST ? 10'd430  : I_box_ymax[3*10 +: 10];
    wire        B3_EN   = SELF_TEST ? 1'b1     : I_box_valid[3];

    //==================================================================
    // 3. 边框判定（第 1 拍，用 _1d 坐标）
    //
    //    ★★ 这里踩过一个坑，写下来免得后人重踩（2026-10-09）★★
    //    错误写法（第一版，三份实现同时犯、还互相对上了）：
    //        (in_y && (x <= x0+1 || x >= x1-1))
    //      || (in_x && (y <= y0+1 || y >= y1-1))
    //    看起来"左右边 or 上下边"，实际漏了范围上界/下界：
    //      · 竖边子句里 `x <= x0+1` 没有 `x >= x0` 的下界 ⇒ 框左侧整片也被点亮；
    //      · 横边子句里 `y <= y0+1` 没有 `y >= y0` 的下界 ⇒ 框上方整片也被点亮。
    //    1280x720 上表现为"框上方和左边拖出一大片色块"，非常显眼。
    //    数值比对没抓到它（因为结构检查只扫了框内像素），
    //    是 sim/bit_sim_box_draw.py 的 ASCII 还原一眼看出来的
    //    ⇒ 结论：画框这种"空间图案"一定要出 ASCII / 位图肉眼过一遍，别只看计数。
    //
    //    正确写法：每条边都要有完整的区间上下界
    //      竖边   ：y ∈ [y0, y1] 且 ( x ∈ [x0, x0+BW-1] 或 x ∈ [x1-BW+1, x1] )
    //      横边   ：x ∈ [x0, x1] 且 ( y ∈ [y0, y0+BW-1] 或 y ∈ [y1-BW+1, y1] )
    //    退化框（宽或高 <= BORDER_W+1）不画，同时保证 x1-BW 之类的减法不下溢。
    //==================================================================
    wire [10:0] BW_X = BORDER_W - 1;     // x 侧内缩量
    wire [9:0]  BW_Y = BORDER_W - 1;     // y 侧内缩量

    wire B0_HIT = B0_EN
               && (B0_XMAX > (B0_XMIN + 11'd3)) && (B0_YMAX > (B0_YMIN + 10'd3))
               && ( ( (S_y_1d >= B0_YMIN) && (S_y_1d <= B0_YMAX)
                      && ( ( (S_x_1d >= B0_XMIN) && (S_x_1d <= (B0_XMIN + BW_X)) )
                        || ( (S_x_1d >= (B0_XMAX - BW_X)) && (S_x_1d <= B0_XMAX) ) ) )
                 || ( (S_x_1d >= B0_XMIN) && (S_x_1d <= B0_XMAX)
                      && ( ( (S_y_1d >= B0_YMIN) && (S_y_1d <= (B0_YMIN + BW_Y)) )
                        || ( (S_y_1d >= (B0_YMAX - BW_Y)) && (S_y_1d <= B0_YMAX) ) ) ) );

    wire B1_HIT = B1_EN
               && (B1_XMAX > (B1_XMIN + 11'd3)) && (B1_YMAX > (B1_YMIN + 10'd3))
               && ( ( (S_y_1d >= B1_YMIN) && (S_y_1d <= B1_YMAX)
                      && ( ( (S_x_1d >= B1_XMIN) && (S_x_1d <= (B1_XMIN + BW_X)) )
                        || ( (S_x_1d >= (B1_XMAX - BW_X)) && (S_x_1d <= B1_XMAX) ) ) )
                 || ( (S_x_1d >= B1_XMIN) && (S_x_1d <= B1_XMAX)
                      && ( ( (S_y_1d >= B1_YMIN) && (S_y_1d <= (B1_YMIN + BW_Y)) )
                        || ( (S_y_1d >= (B1_YMAX - BW_Y)) && (S_y_1d <= B1_YMAX) ) ) ) );

    wire B2_HIT = B2_EN
               && (B2_XMAX > (B2_XMIN + 11'd3)) && (B2_YMAX > (B2_YMIN + 10'd3))
               && ( ( (S_y_1d >= B2_YMIN) && (S_y_1d <= B2_YMAX)
                      && ( ( (S_x_1d >= B2_XMIN) && (S_x_1d <= (B2_XMIN + BW_X)) )
                        || ( (S_x_1d >= (B2_XMAX - BW_X)) && (S_x_1d <= B2_XMAX) ) ) )
                 || ( (S_x_1d >= B2_XMIN) && (S_x_1d <= B2_XMAX)
                      && ( ( (S_y_1d >= B2_YMIN) && (S_y_1d <= (B2_YMIN + BW_Y)) )
                        || ( (S_y_1d >= (B2_YMAX - BW_Y)) && (S_y_1d <= B2_YMAX) ) ) ) );

    wire B3_HIT = B3_EN
               && (B3_XMAX > (B3_XMIN + 11'd3)) && (B3_YMAX > (B3_YMIN + 10'd3))
               && ( ( (S_y_1d >= B3_YMIN) && (S_y_1d <= B3_YMAX)
                      && ( ( (S_x_1d >= B3_XMIN) && (S_x_1d <= (B3_XMIN + BW_X)) )
                        || ( (S_x_1d >= (B3_XMAX - BW_X)) && (S_x_1d <= B3_XMAX) ) ) )
                 || ( (S_x_1d >= B3_XMIN) && (S_x_1d <= B3_XMAX)
                      && ( ( (S_y_1d >= B3_YMIN) && (S_y_1d <= (B3_YMIN + BW_Y)) )
                        || ( (S_y_1d >= (B3_YMAX - BW_Y)) && (S_y_1d <= B3_YMAX) ) ) ) );

    //==================================================================
    // 4. 十字准星判定（第 1 拍）
    //    竖臂 2px：x ∈ [cx, cx+1]，y ∈ [cy-CROSS_HALF, cy+CROSS_HALF]
    //    横臂 2px：y ∈ [cy, cy+1]，x ∈ [cx-CROSS_HALF, cx+CROSS_HALF]
    //    ★减法定界用「> 才相减」的三目，避免 c < CROSS_HALF 时下溢成大数
    //      （下溢会让准星在靠边时整根消失，是很容易漏掉的一类边界 bug）。
    //==================================================================
    wire [10:0] W_CROSS_X   = SELF_TEST ? 11'd640 : I_cross_x;
    wire [9:0]  W_CROSS_Y   = SELF_TEST ? 10'd360 : I_cross_y;
    wire        W_CROSS_EN  = SELF_TEST ? 1'b1    : I_cross_en;

    wire [10:0] W_CX_LO = (W_CROSS_X > CROSS_HALF) ? (W_CROSS_X - CROSS_HALF) : 11'd0;
    wire [10:0] W_CX_HI = W_CROSS_X + CROSS_HALF;
    wire [9:0]  W_CY_LO = (W_CROSS_Y > CROSS_HALF) ? (W_CROSS_Y - CROSS_HALF) : 10'd0;
    wire [9:0]  W_CY_HI = W_CROSS_Y + CROSS_HALF;

    wire W_CROSS_V = (S_x_1d >= W_CROSS_X) && (S_x_1d <= (W_CROSS_X + 11'd1))
                  && (S_y_1d >= W_CY_LO)   && (S_y_1d <= W_CY_HI);      // 竖臂
    wire W_CROSS_H = (S_y_1d >= W_CROSS_Y) && (S_y_1d <= (W_CROSS_Y + 10'd1))
                  && (S_x_1d >= W_CX_LO)   && (S_x_1d <= W_CX_HI);      // 横臂
    wire W_CROSS_HIT = W_CROSS_EN && (W_CROSS_V || W_CROSS_H);

    //==================================================================
    // 5. 优先级 + 寄存（第 1 拍拍末）
    //    优先级：十字准星 > B0 > B1 > B2 > B3
    //    （准星只有 2px 宽，压在最上层才不会被框边盖住）
    //==================================================================
    reg        S_hit_r;
    reg [23:0] S_color_r;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_hit_r   <= 1'b0;
            S_color_r <= 24'd0;
        end
        else begin
            if(W_CROSS_HIT) begin
                S_hit_r   <= 1'b1;
                S_color_r <= COLOR_CROSS;
            end
            else if(B0_HIT) begin
                S_hit_r   <= 1'b1;
                S_color_r <= COLOR_BOX0;
            end
            else if(B1_HIT) begin
                S_hit_r   <= 1'b1;
                S_color_r <= COLOR_BOX1;
            end
            else if(B2_HIT) begin
                S_hit_r   <= 1'b1;
                S_color_r <= COLOR_BOX2;
            end
            else if(B3_HIT) begin
                S_hit_r   <= 1'b1;
                S_color_r <= COLOR_BOX3;
            end
            else begin
                S_hit_r   <= 1'b0;
                S_color_r <= 24'd0;
            end
        end
    end

    //==================================================================
    // 6. 输出级（第 2 拍）
    //    S_hit_r 由 _1d 坐标算出，与本拍的 S_x_2d 严格同像素
    //    （_2d 比 _1d 晚 1 拍，正好补上这一级寄存），
    //    所以这里只需一个 de 门控，没有任何组合比较 → 关键路径极短。
    //==================================================================
    assign O_hit   = S_hit_r & S_de_2d;
    assign O_color = S_color_r;
    assign O_de    = S_de_2d;

endmodule
