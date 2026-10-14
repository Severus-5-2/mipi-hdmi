//=============================================================================
//  tb_shape_class_frame_align.v —— 自检 TB：测 shape_class 读 S_target_bus 的「采样窗口」
//=============================================================================
//  问题来源：张铭晨实测「帧脉冲比总线锁存早 1~N 拍」会让"一拍读一个槽"的消费者串帧。
//            我方 shape_class / osd_category 也是这类消费者 —— 要量出**我们的危险窗口多宽**。
//
//  方法：帧脉冲固定在 rel=50 拍（rel = cyc - base）；
//        总线在 rel = 50 + EARLY 拍从「帧 A」切到「帧 B」。
//          EARLY > 0 → 脉冲**早于**锁存 EARLY 拍
//          EARLY ≤ 0 → 脉冲**不早于**锁存
//        判别：全【方】=读到上一帧 A（一致，仅滞后）
//              全【三角】=读到当帧 B（正确）
//              其它      =★ 串帧
//=============================================================================
`timescale 1ns/1ps
`include "../common_base/common_defs.vh"

module tb_shape_class_frame_align;

    localparam integer SW = `TGT_SLOT_W;   // 96
    localparam integer NS = 4;
    localparam integer PULSE_AT = 50;      // 脉冲固定时刻（rel）

    reg  clk = 0, rst_n = 0;
    reg  [NS*SW-1:0] I_bus;
    reg              I_frame_done;
    wire [2*NS-1:0]  O_shapes;

    integer cyc  = 0;      // 自由计数（只由时钟块递增）
    integer base = 0;      // 本 case 的起点（由 task 在复位后设置）
    integer sw_at = 0;     // 总线切换的 rel 时刻 = PULSE_AT + EARLY
    integer rel;

    always #5 clk = ~clk;
    always @(posedge clk) cyc <= cyc + 1;
    always @(*) rel = cyc - base;

    function [SW-1:0] slot_of;
        input [19:0] area;
        reg   [SW-1:0] s;
        begin
            s = {SW{1'b0}};
            s[`TGT_O_VALID]        = 1'b1;
            s[`TGT_O_CLS   +: 2]   = 2'd0;      // 红
            s[`TGT_O_XMIN  +: 11]  = 11'd200;
            s[`TGT_O_XMAX  +: 11]  = 11'd299;   // w = 100
            s[`TGT_O_YMIN  +: 10]  = 10'd150;
            s[`TGT_O_YMAX  +: 10]  = 10'd249;   // h = 100
            s[`TGT_O_AREA  +: 20]  = area;
            slot_of = s;
        end
    endfunction

    function [NS*SW-1:0] bus_of;
        input [19:0] area;
        integer j; reg [NS*SW-1:0] b;
        begin
            b = {(NS*SW){1'b0}};
            for (j = 0; j < NS; j = j + 1) b[j*SW +: SW] = slot_of(area);
            bus_of = b;
        end
    endfunction

    // A = area 10000 → f=100% → 方(0)   B = area 5000 → f=50% → 三角(2)
    always @(*) I_bus        = (rel < sw_at) ? bus_of(20'd10000) : bus_of(20'd5000);
    always @(*) I_frame_done = (rel == PULSE_AT);

    shape_class #(.N_SLOT(NS)) u_dut (
        .clk(clk), .rst_n(rst_n),
        .I_bus(I_bus), .I_frame_done(I_frame_done),
        .O_shapes(O_shapes)
    );

    integer n_cur, n_prev, n_mix;      // 各类结果计数
    integer first_mix, last_mix;       // 串帧窗口

    task run_case;
        input integer early;
        integer c;
        begin
            sw_at = PULSE_AT + early;
            rst_n = 0;
            repeat (6) @(posedge clk);
            @(negedge clk); base = cyc;
            rst_n = 1;
            repeat (PULSE_AT + 120) @(posedge clk);
            @(negedge clk);

            $write("  脉冲早 %6d 拍   O_shapes=", early);
            for (c = 0; c < NS; c = c + 1) $write("%0d", O_shapes[2*c +: 2]);
            if      (O_shapes == {NS{2'd0}}) begin
                $display("   读到上一帧【方】= 一致（仅滞后）"); n_prev = n_prev + 1;
            end else if (O_shapes == {NS{2'd2}}) begin
                $display("   读到当帧【三角】= 正确");           n_cur  = n_cur  + 1;
            end else begin
                $display("   ★ 串帧（各槽读到不同帧）= 危险");   n_mix  = n_mix  + 1;
                if (first_mix < 0) first_mix = early;
                last_mix = early;
            end
        end
    endtask

    initial begin
        n_cur = 0; n_prev = 0; n_mix = 0; first_mix = -1; last_mix = -1;
        $display("");
        $display("=== shape_class 读 S_target_bus：帧脉冲【早于总线锁存】多少拍会出问题 ===");
        $display("    A=方(area=10000,f=1.00)   B=三角(area=5000,f=0.50)");
        $display("");
        run_case(-4); run_case(-1); run_case(0); run_case(1);
        run_case(2); run_case(3); run_case(5); run_case(9);
        run_case(13); run_case(14); run_case(20); run_case(39601);
        $display("");
        $display("  小结：读到当帧 %0d 例 / 读到上一帧(一致) %0d 例 / 串帧 %0d 例",
                 n_cur, n_prev, n_mix);
        if (n_mix > 0)
            $display("  ★ 串帧窗口：脉冲早于总线锁存 %0d ~ %0d 拍", first_mix, last_mix);
        $display("");
        $display("  结论（顶层接线铁律）：");
        $display("    · 早 <=1 拍  → 读到当帧（最好）");
        $display("    · 早 2~13 拍 → ★ 串帧（各槽读到不同帧，静默出错）");
        $display("    · 早 >=14 拍 → 读到上一帧（一致，仅整体滞后 1 帧，可接受）");
        $display("    ⇒ 消费者触发帧脉冲的时刻，要么 >= 总线锁存拍，要么 <= 锁存拍-14；");
        $display("      落在中间 12 拍窗口里就会静默串帧。");
        $display("");
        $display("  本项目现状：");
        $display("    · 旧版 pix_coord_gen（仓库现版）: S_frame_done 在 vsync 场消隐，");
        $display("      比总线锁存早约 39601 拍 ⇒ 落在 >=14 区 ⇒ 安全（仅滞后 1 帧）");
        $display("    · 新版 pix_coord_gen（张铭晨 10-14）: O_frame_done 在帧首，");
        $display("      比 S_det_fdone 早约 3 拍 ⇒ ★ 落在串帧窗口 ⇒ 必须先改顶层 3 处 I_frame_done");
        if (n_cur >= 4 && n_prev >= 3 && n_mix >= 4 &&
            first_mix == 2 && last_mix == 13)
            $display("\n[PASS] 采样窗口实测完成：早<=1 当帧 / 2~13 串帧 / >=14 一致 —— 与预期边界一致");
        else
            $display("\n[FAIL] 采样窗口与预期不符（期望：串帧窗口恰为 2~13）");
        $finish;
    end

endmodule
