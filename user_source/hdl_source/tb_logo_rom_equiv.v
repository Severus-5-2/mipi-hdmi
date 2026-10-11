//=============================================================================
//  tb_logo_rom_equiv.v — anlogic_logo_rom 位级等价对拍
//=============================================================================
//  目的：验证「case 版（旧）」与「ERAM 数组版（新）」在**全部 20427 个地址**
//        上输出逐位一致，包括：
//          · 6387 个有效地址
//          · 14040 个未初始化地址（旧 case 走 default=0，新数组应为 0）
//  方法：两份 RTL 同时例化，同一拍给同一地址，比较 O_pixel。
//        两种写法都是「同步读、1 拍延迟」，故可直接逐拍比对。
//
//  编译（iverilog）：
//      iverilog -g2005 -o tb_logo.vvp tb_logo_rom_equiv.v \
//               anlogic_logo_rom.v logo_rom_ref_case.v
//      vvp tb_logo.vvp
//  （logo_rom_ref_case.v 是把旧 case 版改个模块名的参考模型）
//=============================================================================

`timescale 1ns / 1ps

module tb_logo_rom_equiv;

    localparam integer MAXA = 20426;

    reg         clk;
    reg  [14:0] addr;
    wire [24:0] q_new;
    wire [24:0] q_ref;

    integer errors;
    integer i;

    // 被测：ERAM 数组版
    anlogic_logo_rom u_new (
        .I_clk   (clk),
        .I_addr  (addr),
        .O_pixel (q_new)
    );

    // 参考：原 case 版（模块名 logo_rom_ref_case）
    logo_rom_ref_case u_ref (
        .I_clk   (clk),
        .I_addr  (addr),
        .O_pixel (q_ref)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    initial begin
        errors = 0;
        addr   = 15'd0;

        // 复位后先跑 2 拍让同步读稳定
        repeat (2) @(negedge clk);

        for (i = 0; i <= MAXA; i = i + 1) begin
            addr = i[14:0];
            @(negedge clk);          // 驱动
            @(posedge clk);          // 采样，O_pixel 更新
            #1;                      // 让非阻塞赋值生效
            if (q_new !== q_ref) begin
                errors = errors + 1;
                if (errors <= 20)
                    $display("MISMATCH addr=%0d : new=%h ref=%h", i, q_new, q_ref);
            end
        end

        $display("========================================");
        if (errors == 0)
            $display("  [PASS] 全部 %0d 个地址逐位一致", MAXA + 1);
        else
            $display("  [FAIL] %0d 个地址不一致", errors);
        $display("========================================");
        $display("TB done");
        $finish;
    end

endmodule
