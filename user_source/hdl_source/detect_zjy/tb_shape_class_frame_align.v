//=============================================================================
//  tb_shape_class_frame_align.v —— 自检 TB：测 shape_class 读 S_target_bus 的「采样窗口」
//=============================================================================
//  问题来源（2026-10-14，与张铭晨共同确认）：
//    读 S_target_bus 的消费者，若「采样首拍」落在**比总线锁存早**的一小段窗口内，
//    会读到**跨帧的混合结果**（各槽来自不同帧）—— 静默出错，不报错、不崩溃。
//
//  方法：帧脉冲固定在 rel = PULSE_AT 拍（rel = cyc - base）；
//        总线在 rel = PULSE_AT + EARLY 拍从「帧 A」切到「帧 B」。
//          EARLY > 0  → 脉冲**早于**总线锁存 EARLY 拍
//          EARLY ≤ 0  → 脉冲**不早于**锁存
//        判别：全【方】(area=10000)   = 读到 A（完整一帧，一致，仅滞后）
//              全【三角】(area=5000) = 读到 B（当帧，正确）
//              其它                  = ★ 串帧（各槽读到不同帧 = 危险）
//
//  ---------------------------------------------------------------------------
//  ★ 本 TB 的 PASS 判据**不依赖 `shape_class` 的具体分时节奏**（2026-10-14 修订）
//  ---------------------------------------------------------------------------
//  起因：张铭晨用本 TB + 四档"每槽取数间隔"的桩复跑，得到窗口上沿依次为
//        1 拍/槽 → 2~3 ｜ 2 拍/槽 → 2~5 ｜ 3 拍/槽 → 2~9 ｜ 4 拍/槽 → 2~13
//        只有 4 拍/槽 那档与旧版 TB 里写死的"2~13"吻合。
//        ⇒ **"2~13" 是"每槽 4 拍分时"这个具体实现下的数，不是常数**：
//          末槽取数拍 = 1 + (N_SLOT-1) × 每槽拍数；窗口上沿 = 末槽取数拍。
//        旧版 TB 把 `last_mix == 13` 写进 PASS 判据 ⇒ 哪天把 `shape_class`
//        改成一拍一槽（省面积）或改 `N_SLOT`，回归会**莫名其妙变红**。
//
//  现在只断言**与实现无关的结构性质**：
//    ① 「不早于锁存」（EARLY ≤ 1）→ 必须读到**当帧**（这是同源脉冲配置的底线）
//    ② 比锁存早**一个完整场消隐期**（EARLY ≥ FIELD_OFF）→ 必须读到**完整上一帧**（一致）
//    ③ 危险带**存在**、**连续**、**下沿 = 2**（早 0/1 拍安全、早 2 拍起出事）
//    ④ 危险带**外**没有任何串帧
//  ⇒ 窗口上沿只**报告**、不断言。这样调 `shape_class` 的节拍不会误报，
//    而"两端安全区消失 / 串帧跑到带外"这类**真回归**照样会被抓住。
//=============================================================================
`timescale 1ns/1ps
`include "../common_base/common_defs.vh"

module tb_shape_class_frame_align;

    localparam integer SW = `TGT_SLOT_W;   // 96
    localparam integer NS = 4;

    localparam integer PULSE_AT = 50;      // 脉冲固定时刻（rel）

    //-------------------------------------------------------------------------
    //  ★ 判据参数（三者都与 shape_class 的实现无关）
    //-------------------------------------------------------------------------
    localparam integer SCAN_MAX  = 48;     // 逐拍扫描 0..SCAN_MAX（覆盖到很慢的分时节奏）
    localparam integer FIELD_OFF = 39601;  // 一个完整场消隐期（旧版 S_frame_done 的相对偏移）
    localparam integer PIX_EARLY = 3;      // 新版 pix_coord_gen 的已知偏移（**外部模块属性**，仅用于风险提示）
    localparam integer N_CASES   = SCAN_MAX + 4;

    reg  clk = 0, rst_n = 0;
    reg  [NS*SW-1:0] I_bus;
    reg              I_frame_done;
    wire [2*NS-1:0]  O_shapes;

    integer cyc  = 0;
    integer base = 0;
    integer sw_at = 0;
    integer rel;

    always #5 clk = ~clk;
    always @(posedge clk) cyc <= cyc + 1;
    always @(*) rel = cyc - base;

    // 结果分类
    localparam integer RES_CUR  = 0;       // 读到当帧 B（正确）
    localparam integer RES_PREV = 1;       // 读到完整上一帧 A（一致，仅滞后）
    localparam integer RES_MIX  = 2;       // ★ 串帧

    integer res [0:N_CASES-1];
    integer offs[0:N_CASES-1];

    //  打印一个结果（用 task + 直接字面量，不用 %s ——
    //  ★ Verilog 的多分支字符串字面量会按**最长者**补齐，短串左补 0 后再 %s 打印会乱码）
    task show_res;
        input integer r;
        begin
            if      (r == RES_CUR)  $write("当帧");
            else if (r == RES_PREV) $write("完整上一帧(一致)");
            else                    $write("★ 串帧");
        end
    endtask

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

    // A = area 10000 → 填充度 1.00 → 方(0)      B = area 5000 → 填充度 0.50 → 三角(2)
    always @(*) I_bus        = (rel < sw_at) ? bus_of(20'd10000) : bus_of(20'd5000);
    always @(*) I_frame_done = (rel == PULSE_AT);

    shape_class #(.N_SLOT(NS)) u_dut (
        .clk(clk), .rst_n(rst_n),
        .I_bus(I_bus), .I_frame_done(I_frame_done),
        .O_shapes(O_shapes)
    );

    //-------------------------------------------------------------------------
    //  单次 case
    //-------------------------------------------------------------------------
    task run_one;
        input integer early;
        input integer idx;
        begin
            sw_at = PULSE_AT + early;
            rst_n = 0;
            repeat (6) @(posedge clk);
            @(negedge clk); base = cyc;
            rst_n = 1;
            repeat (PULSE_AT + 120) @(posedge clk);
            @(negedge clk);

            offs[idx] = early;
            if      (O_shapes == {NS{2'd0}}) res[idx] = RES_PREV;
            else if (O_shapes == {NS{2'd2}}) res[idx] = RES_CUR;
            else                             res[idx] = RES_MIX;
        end
    endtask

    //-------------------------------------------------------------------------
    //  逐拍扫描 + 结构判据
    //-------------------------------------------------------------------------
    integer i, e;
    integer n_cur, n_prev, n_mix;
    integer band_lo, band_hi;      // 串帧带（按 EARLY）
    integer mix_outside;           // 带外串帧个数
    integer idx_field;
    integer seen_first;            // 带是否连续
    integer ok_ok;

    initial begin
        // ① 两个"不早于锁存"的样本（EARLY = -4 / -1）
        run_one(-4, 0);
        run_one(-1, 1);
        // ② 主扫描：EARLY = 0 .. SCAN_MAX 逐拍（索引 2 .. SCAN_MAX+2）
        for (e = 0; e <= SCAN_MAX; e = e + 1) run_one(e, e + 2);
        // ③ 比锁存早一个完整场消隐期（索引 SCAN_MAX+3 —— 必须避开主扫描的末格）
        idx_field = SCAN_MAX + 3;
        run_one(FIELD_OFF, idx_field);

        // ---- 统计 ----
        n_cur = 0; n_prev = 0; n_mix = 0;
        band_lo = -1; band_hi = -1; seen_first = 0; mix_outside = 0;
        for (i = 0; i < N_CASES; i = i + 1) begin
            if      (res[i] == RES_CUR)  n_cur  = n_cur  + 1;
            else if (res[i] == RES_PREV) n_prev = n_prev + 1;
            else begin
                n_mix = n_mix + 1;
                if (band_lo < 0) begin band_lo = offs[i]; band_hi = offs[i]; end
                else begin
                    if (offs[i] == band_hi + 1) band_hi = offs[i];   // 连续
                    else                        mix_outside = mix_outside + 1;
                end
            end
        end

        // ---- 报告 ----
        $display("");
        $display("=== shape_class 读 S_target_bus 的「采样窗口」自检 ===");
        $display("    逐拍扫描 早于锁存 0 ~ %0d 拍，外加一个完整场消隐期（%0d 拍）", SCAN_MAX, FIELD_OFF);
        $display("    帧 A = 方(area=10000) / 帧 B = 三角(area=5000)");
        $display("");
        for (i = 0; i < N_CASES; i = i + 1) begin
            if (i == idx_field || offs[i] <= 16) begin
                $write("    早 %6d 拍 -> ", offs[i]);
                show_res(res[i]);
                $display("");
            end
        end
        $display("");
        $display("  ── 汇总（%0d 个偏移）──", N_CASES);
        $display("    当帧 %0d 例 / 完整上一帧(一致) %0d 例 / 串帧 %0d 例", n_cur, n_prev, n_mix);
        if (band_lo >= 0)
            $display("    ★ 串帧带（当前实现）= 早 %0d ~ %0d 拍   连续=%0d  带外串帧=%0d",
                     band_lo, band_hi, (mix_outside == 0), mix_outside);
        $display("");
        $display("  ── 接线铁律（不依赖具体实现）──");
        $display("    · 采样首拍 >= 总线锁存拍           -> 读到当帧（最佳，推荐）");
        $display("    · 或 比锁存拍早 >= 一个完整场消隐期 -> 读到完整上一帧（一致，仅滞后 1 帧）");
        $display("    · ★ 中间地带一律不可用：会静默串帧");
        $display("      ⚠ 该地带宽度 = 末槽取数拍，随模块自身的取槽节奏变化，");
        $display("         **不要按某个固定数字去凑**（本 TB 也只报告、不断言它）。");

        // ---- 风险提示（外部模块属性，不作为 PASS 条件）----
        if (band_lo >= 0 && PIX_EARLY >= band_lo && PIX_EARLY <= band_hi) begin
            $display("");
            $display("  ⚠ 风险提示：新版 pix_coord_gen 的 O_frame_done 比总线锁存早约 %0d 拍，", PIX_EARLY);
            $display("             落在本带上 ⇒ 换新版必须与顶层 I_frame_done 改接 S_det_fdone **同时做**。");
        end else if (band_lo >= 0) begin
            $display("");
            $display("  （提示：新版 pix_coord_gen 早约 %0d 拍，当前不在带上 —— 若换用请重新评估）", PIX_EARLY);
        end

        // ---- PASS 判据：只断言与实现无关的结构性质 ----
        ok_ok = 1;
        if (res[0] != RES_CUR || res[1] != RES_CUR) begin
            ok_ok = 0; $display("  !! 不早于锁存（早 -4 / -1 拍）未读到当帧");
        end
        for (e = 0; e <= 1; e = e + 1)
            if (res[e + 2] != RES_CUR) begin
                ok_ok = 0; $display("  !! 早 %0d 拍未读到当帧（同源脉冲配置的底线被破坏）", e);
            end
        if (res[idx_field] != RES_PREV) begin
            ok_ok = 0; $display("  !! 早一个完整场消隐期（%0d 拍）未读到完整上一帧", FIELD_OFF);
        end
        if (band_lo != 2) begin
            ok_ok = 0; $display("  !! 串帧带下沿不是 2（实际 %0d）", band_lo);
        end
        if (band_hi < 2) begin
            ok_ok = 0; $display("  !! 未检出串帧带（危险模型变了，需复核铁律）");
        end
        if (mix_outside != 0) begin
            ok_ok = 0; $display("  !! 串帧带不连续 / 带外也存在串帧（%0d 例）", mix_outside);
        end
        if (band_lo >= 0 && n_mix != (band_hi - band_lo + 1)) begin
            ok_ok = 0;
            $display("  !! 串帧个数 %0d 与带 [%0d,%0d] 的宽度不符（有零散串帧）",
                     n_mix, band_lo, band_hi);
        end

        $display("");
        if (ok_ok)
            $display("[PASS] 采样窗口结构自检通过：不早于锁存=当帧 / 场消隐偏移=一致上一帧 / 串帧带 2~%0d 连续", band_hi);
        else
            $display("[FAIL] 采样窗口结构自检未通过（见上方 !! 项）");
        $finish;
    end

endmodule
