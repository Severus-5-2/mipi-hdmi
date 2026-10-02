//====================================================================
// 模块名称：sobel_3x3.v
// 模块功能：Sobel 3×3 算子 边缘检测（边缘白色、背景黑色）
// 工作原理：
//   1) 两块行缓存（ERAM）缓存前两行灰度，构造 3×3 滑动窗口；
//   2) 实时流处理中"下一行"尚未到来，窗口中心对齐"上一行"（输出延迟1行）；
//   3) 分别计算水平梯度 Gx、垂直梯度 Gy，幅度取 L1 近似 |Gx|+|Gy|；
//   4) 幅度超过 EDGE_TH 判为边缘（白色），否则黑色。
// 输入：8bit 灰度 gray_in + 数据有效 I_de（行计数/行缓存写入均由 de 使能）
// 输出：24bit RGB（边缘 24'hffffff，背景 24'h000000）
// 时钟域：S_hdmi_pixel_clk（HDMI 像素时钟）
// 备注：
//   - 行缓存用 (* ram_style="block" *) 强制综合进 ERAM 块存储器；
//   - 行首/行末因窗口延迟会有几像素黑边（固有现象，消隐期吸收）；
//   - 第1行缓存为0，顶部可能出现一条边（可接受）。
//====================================================================
module sobel_3x3 #(
    parameter IMG_WIDTH  = 1280,   // 图像宽度（像素）
    parameter IMG_HEIGHT = 720,    // 图像高度（行，仅用于参数对齐）
    parameter EDGE_TH    = 130     // 边缘阈值：越大只保留越强的边缘
)(
    input               clk,        // 像素时钟
    input               rst_n,      // 低电平复位
    input               I_de,       // 数据有效（必须接 S_hdmi_de）
    input      [7:0]    gray_in,    // 输入灰度像素（当前行）
    output reg [23:0]   rgb_out     // 输出 RGB（边缘白 / 背景黑）
);

    //==================================================================
    // 一、行缓存：两块 IMG_WIDTH×8bit，强制使用 ERAM 块存储器
    //   line_buf0：上一行（窗口中心行）
    //   line_buf1：上上行
    //==================================================================
    (* ram_style = "block" *) reg [7:0] line_buf0 [0:IMG_WIDTH-1];
    (* ram_style = "block" *) reg [7:0] line_buf1 [0:IMG_WIDTH-1];

    // 行内像素计数（0~IMG_WIDTH-1）
    reg [10:0] h_cnt;

    // 行计数：只在 de 有效时累加，行末清零（消隐期不计数，关键！）
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            h_cnt <= 11'd0;
        else if(I_de) begin
            if(h_cnt == IMG_WIDTH - 1)
                h_cnt <= 11'd0;
            else
                h_cnt <= h_cnt + 11'd1;
        end
    end

    // 行缓存写入：只在 de 有效时写入
    //   新像素 gray_in 写入 line_buf0；line_buf0 旧内容下移到 line_buf1
    always @(posedge clk) begin
        if(I_de) begin
            line_buf1[h_cnt] <= line_buf0[h_cnt];
            line_buf0[h_cnt] <= gray_in;
        end
    end

    //==================================================================
    // 二、3×3 滑动窗口（移位寄存器）
    //   实时对齐（中心 = 上一行）：
    //     p0（上上行） = line_buf1
    //     p1（上一行，中心） = line_buf0
    //     p2（当前行） = gray_in
    //   r?_c2 读当前位置，c1/c0 为前1/2拍 → 列序 h-2, h-1, h
    //==================================================================
    reg [7:0] r0_c0, r0_c1, r0_c2;   // 上上行
    reg [7:0] r1_c0, r1_c1, r1_c2;   // 上一行（中心）
    reg [7:0] r2_c0, r2_c1, r2_c2;   // 当前行

    // 窗口像素（按标准 3×3 排列）
    wire [7:0] p00, p01, p02;
    wire [7:0] p10, p11, p12;
    wire [7:0] p20, p21, p22;
    assign p00 = r0_c0; assign p01 = r0_c1; assign p02 = r0_c2;
    assign p10 = r1_c0; assign p11 = r1_c1; assign p12 = r1_c2;
    assign p20 = r2_c0; assign p21 = r2_c1; assign p22 = r2_c2;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            {r0_c0,r0_c1,r0_c2} <= 24'd0;
            {r1_c0,r1_c1,r1_c2} <= 24'd0;
            {r2_c0,r2_c1,r2_c2} <= 24'd0;
        end else begin
            // 同一时钟沿读行缓存（写入前旧值），列方向移位
            r0_c0 <= r0_c1; r0_c1 <= r0_c2; r0_c2 <= line_buf1[h_cnt];  // 上上行
            r1_c0 <= r1_c1; r1_c1 <= r1_c2; r1_c2 <= line_buf0[h_cnt];  // 上一行（中心）
            r2_c0 <= r2_c1; r2_c1 <= r2_c2; r2_c2 <= gray_in;           // 当前行
        end
    end

    //==================================================================
    // 三、Sobel 梯度计算
    //   Gx = (p02+2·p12+p22) - (p00+2·p10+p20)  水平梯度 → 检测垂直边缘
    //   Gy = (p20+2·p21+p22) - (p00+2·p01+p02)  垂直梯度 → 检测水平边缘
    //   {1'b0,data} 先零扩展再 $signed，保证 200 仍按 +200 处理
    //   范围：Gx/Gy 各 ±1020，用 11 位有符号（-1024~1023）
    //==================================================================
    // 各列/各行差分（显式有符号）
    wire signed [10:0] gx_d0 = $signed({1'b0,p02}) - $signed({1'b0,p00});
    wire signed [10:0] gx_d1 = $signed({1'b0,p12}) - $signed({1'b0,p10});
    wire signed [10:0] gx_d2 = $signed({1'b0,p22}) - $signed({1'b0,p20});

    wire signed [10:0] gy_d0 = $signed({1'b0,p20}) - $signed({1'b0,p00});
    wire signed [10:0] gy_d1 = $signed({1'b0,p21}) - $signed({1'b0,p01});
    wire signed [10:0] gy_d2 = $signed({1'b0,p22}) - $signed({1'b0,p02});

    // 中间行/列权重为 2（左移1位）
    wire signed [10:0] gx;
    wire signed [10:0] gy;
    assign gx = gx_d0 + (gx_d1 << 1) + gx_d2;
    assign gy = gy_d0 + (gy_d1 << 1) + gy_d2;

    // 梯度幅度 L1 近似：|Gx| + |Gy|（范围 0~2040，11 位无符号）
    // 符号位 [10]=1 表示负数，取绝对值
    wire [10:0] mag;
    assign mag = (gx[10] ? -gx : gx) + (gy[10] ? -gy : gy);

    //==================================================================
    // 四、边缘判定（打一拍输出，流水线收敛时序）
    //   mag > EDGE_TH → 白色边缘；否则黑色背景
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            rgb_out <= 24'h000000;
        else if(mag > EDGE_TH)
            rgb_out <= 24'hffffff;
        else
            rgb_out <= 24'h000000;
    end

endmodule
