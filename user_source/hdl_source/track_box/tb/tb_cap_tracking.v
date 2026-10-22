`timescale 1ns/1ps
module tb_cap_tracking;
    reg clk=0,rst=0,fd=0;
    reg [383:0] bus=0;
    wire [3:0] valid;
    wire [43:0] xmin,xmax;
    wire [39:0] ymin,ymax;
    wire [2:0] count;
    always #5 clk=~clk;
    target_assoc #(.CAP_FILTER(1)) dut(.I_clk(clk),.I_rst_n(rst),.I_frame_done(fd),
        .I_target_cnt(3'd4),.I_target_bus(bus),.O_box_valid(valid),.O_track_cnt(count),
        .O_box_xmin(xmin),.O_box_xmax(xmax),.O_box_ymin(ymin),.O_box_ymax(ymax));
    function [95:0] box;
        input integer cl,xa,xb,ya,yb,area;
        reg [95:0] s;
        begin s=0; s[0]=1; s[2:1]=cl; s[13:3]=(xa+xb)/2; s[23:14]=(ya+yb)/2;
            s[34:24]=xa; s[45:35]=xb; s[55:46]=ya; s[65:56]=yb; s[85:66]=area; box=s; end
    endfunction
    task tick_frame;
        begin @(negedge clk); fd=1; @(negedge clk); fd=0; repeat(80) @(negedge clk); end
    endtask
    task reset;
        begin rst=0; bus=0; repeat(4) @(negedge clk); rst=1; end
    endtask
    initial begin
        reset;
        bus[95:0]=box(0,200,219,200,219,400); tick_frame;
        if(count!==0) $fatal(1,"single-frame flash shown");
        tick_frame;
        if(count!==1 || xmin[10:0]!==196 || xmax[10:0]!==223) $fatal(1,"confirmation/padding failed");
        // Rapid area growth at the same center must keep one track, with fast expansion.
        bus[95:0]=box(0,100,319,100,319,40000); tick_frame;
        if(count!==1 || xmin[10:0]!==96 || xmax[10:0]!==323) $fatal(1,"scale change duplicated or clipped box");
        // Internal fragments and one/two-row strips may not create tracks or shrink the parent.
        bus[191:96]=box(0,120,180,150,154,200);
        bus[287:192]=box(0,130,190,170,171,100);
        bus[383:288]=box(0,200,260,210,212,120);
        repeat(3) tick_frame;
        if(count!==1) $fatal(1,"nested/flat fragments created duplicate tracks");
        // A smaller measurement of the same track is not its own nested fragment.
        bus=0; bus[95:0]=box(0,200,219,200,219,400);
        repeat(6) tick_frame;
        if(count!==1 || xmax[10:0]>=300) $fatal(1,"shrinking target vetoed by its own old bbox");
        bus[95:0]=box(0,100,319,100,319,40000); tick_frame;
        // Distinct objects remain independent, including another color inside a red bbox.
        bus[191:96]=box(1,160,180,160,180,400);
        bus[287:192]=box(0,500,520,200,220,400); bus[383:288]=0;
        repeat(2) tick_frame;
        if(count!==3) $fatal(1,"independent colored objects lost");
        reset;
        bus[95:0]=box(1,0,7,0,7,64); repeat(2) tick_frame;
        if(count!==1 || xmin[10:0]!==0 || ymin[9:0]!==0) $fatal(1,"edge padding underflow");
        reset;
        bus[95:0]=box(1,1272,1279,712,719,64); repeat(2) tick_frame;
        if(count!==1 || xmax[10:0]!==1279 || ymax[9:0]!==719) $fatal(1,"edge padding overflow");
        reset;
        bus[95:0]=box(1,300,319,300,319,400); tick_frame;
        bus=0; tick_frame;
        bus[95:0]=box(1,300,319,300,319,400); tick_frame;
        if(count!==0) $fatal(1,"nonconsecutive noise confirmed");
        repeat(2) tick_frame;
        bus=0; repeat(5) tick_frame;
        if(count!==0) $fatal(1,"lost target never cleared");
        $display("CAP_TRACKING PASS"); $finish;
    end
endmodule
