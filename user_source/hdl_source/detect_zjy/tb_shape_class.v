//=============================================================================
//  tb_shape_class.v —— shape_class 位级自检
//=============================================================================
//  验证 4 类形状判据 + 无效槽 + 边界值：
//    槽0：方   w=100 h=100 area=10000  (f=100.0%)  → SHP_SQ  (0)
//    槽1：圆   w=100 h=100 area=7850   (f= 78.5%)  → SHP_CIR (1)
//    槽2：三角 w=100 h=100 area=5000   (f= 50.0%)  → SHP_TRI (2)
//    槽3：稀疏 w=100 h=100 area=1000   (f= 10.0%)  → SHP_UNK (3)
//  再验一轮：
//    槽0：valid=0（无效）                          → SHP_UNK (3)
//    槽1：方   w=200 h= 80 area=16000  (f=100.0%)  → SHP_SQ
//    槽2：圆   w=120 h=120 area=11304  (f=78.5%)   → SHP_CIR
//    槽3：三角 w= 64 h= 64 area= 2048  (f=50.0%)   → SHP_TRI
//  再验阈值边界（f 恰好在 88 / 72 / 38 附近）：
//    槽0：f= 88.0% (area=8800)  → SHP_SQ   （>= 88 应判方）
//    槽1：f= 87.9% (area=8790)  → SHP_CIR
//    槽2：f= 72.0% (area=7200)  → SHP_CIR  （>= 72 应判圆）
//    槽3：f= 38.0% (area=3800)  → SHP_TRI  （>= 38 应判三角）
//=============================================================================
`timescale 1ns / 1ps
`include "../common_base/common_defs.vh"   // ★ 相对本文件目录

module tb_shape_class;

    localparam integer SW = `TGT_SLOT_W;      // 96

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [`TGT_BUS_W-1:0] I_bus;
    reg  I_frame_done;
    wire [7:0] O_shapes;

    shape_class dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .I_bus        (I_bus),
        .I_frame_done (I_frame_done),
        .O_shapes     (O_shapes)
    );

    //---- 期望值与错误计数 ----
    reg [7:0] exp_shapes;
    integer err_cnt;

    //---- 构造一个槽 ----
    task set_slot;
        input integer        k;
        input                v;
        input [10:0]         xmn, xmx;
        input [9:0]          ymn, ymx;
        input [19:0]         ar;
        integer base;
        begin
            base = k * SW;
            I_bus[`TGT_O_VALID + base]        = v;
            I_bus[`TGT_O_XMIN  + base +: 11]  = xmn;
            I_bus[`TGT_O_XMAX  + base +: 11]  = xmx;
            I_bus[`TGT_O_YMIN  + base +: 10]  = ymn;
            I_bus[`TGT_O_YMAX  + base +: 10]  = ymx;
            I_bus[`TGT_O_AREA  + base +: 20]  = ar;
        end
    endtask

    //---- 跑一轮并以 exp_shapes 校验 ----
    task run_round;
        input [8*48-1:0] name;                // 场景名（ASCII，避免 UTF-8 被截断）
        begin
            @(negedge clk);
            I_frame_done = 1'b1;
            @(negedge clk);
            I_frame_done = 1'b0;
            repeat (20) @(negedge clk);       // 12 拍工作 + 余量
            if (O_shapes !== exp_shapes) begin
                err_cnt = err_cnt + 1;
                $display("  [FAIL] %0s : O_shapes=%b 期望=%b  (槽0=%b 槽1=%b 槽2=%b 槽3=%b)",
                         name, O_shapes, exp_shapes,
                         O_shapes[1:0], O_shapes[3:2], O_shapes[5:4], O_shapes[7:6]);
            end else begin
                $display("  [OK]   %0s : 槽0=%b 槽1=%b 槽2=%b 槽3=%b",
                         name, O_shapes[1:0], O_shapes[3:2], O_shapes[5:4], O_shapes[7:6]);
            end
        end
    endtask

    integer i;
    initial begin
        $dumpfile("tb_shape_class.vcd");
        $dumpvars(0, tb_shape_class);

        err_cnt      = 0;
        I_bus        = {`TGT_BUS_W{1'b0}};
        I_frame_done = 1'b0;

        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        $display("");
        $display("== shape_class 位级自检 ==");
        $display("（形状编码：0=方 1=圆 2=三角 3=未知）");
        $display("");

        //---------- 第 1 轮：四类基本形状（f = 100 / 78.5 / 50 / 10 %）----------
        set_slot(0, 1'b1, 11'd100, 11'd199, 10'd100, 10'd199, 20'd10000);  // f=100.0%
        set_slot(1, 1'b1, 11'd400, 11'd499, 10'd100, 10'd199, 20'd7850);   // f= 78.5%
        set_slot(2, 1'b1, 11'd700, 11'd799, 10'd100, 10'd199, 20'd5000);   // f= 50.0%
        set_slot(3, 1'b1, 11'd1000,11'd1099,10'd100, 10'd199, 20'd1000);   // f= 10.0%
        exp_shapes = {2'd3, 2'd2, 2'd1, 2'd0};      // 槽3=未知 槽2=三角 槽1=圆 槽0=方
        run_round("R1-basic-4-shapes");

        //---------- 第 2 轮：无效槽 + 换尺寸 ----------
        set_slot(0, 1'b0, 11'd0,   11'd0,   10'd0,   10'd0,   20'd0);      // 无效
        set_slot(1, 1'b1, 11'd10,  11'd209, 10'd10,  10'd89,  20'd16000);  // 200x80 f=100%
        set_slot(2, 1'b1, 11'd500, 11'd619, 10'd300, 10'd419, 20'd11304);  // 120x120 f=78.5%
        set_slot(3, 1'b1, 11'd900, 11'd963, 10'd500, 10'd563, 20'd2048);   // 64x64 f=50%
        exp_shapes = {2'd2, 2'd1, 2'd0, 2'd3};      // 槽3=三角 槽2=圆 槽1=方 槽0=未知
        run_round("R2-invalid-slot-resize");

        //---------- 第 3 轮：阈值边界（88 / 72 / 38 的临界）----------
        set_slot(0, 1'b1, 11'd100, 11'd199, 10'd100, 10'd199, 20'd8800);   // f=88.0% → 方
        set_slot(1, 1'b1, 11'd400, 11'd499, 10'd100, 10'd199, 20'd8790);   // f=87.9% → 圆
        set_slot(2, 1'b1, 11'd700, 11'd799, 10'd100, 10'd199, 20'd7200);   // f=72.0% → 圆
        set_slot(3, 1'b1, 11'd1000,11'd1099,10'd100, 10'd199, 20'd3800);   // f=38.0% → 三角
        exp_shapes = {2'd2, 2'd1, 2'd1, 2'd0};      // 槽3=三角 槽2=圆 槽1=圆 槽0=方
        run_round("R3-threshold-edge-88-72-38");

        $display("");
        if (err_cnt == 0)
            $display("[PASS] shape_class：3 轮 12 个槽位全部一致");
        else
            $display("[FAIL] shape_class：%0d 处不一致", err_cnt);
        $display("");
        $finish;
    end

endmodule
