`timescale 1ns/1ps
// Startup/reset regression: arbitrary RAM contents must not create motion.
// Uses the real pixel/grayscale/BRAM pipeline; never forces internal control.
module tb_motion_startup;
    reg clk=0, rst_n=0, de=0, fdone=0;
    reg [10:0] x=0;
    reg [9:0] y=0;
    reg [23:0] rgb=0;
    wire [3:0] valid;
    wire [43:0] xmin,xmax;
    wire [39:0] ymin,ymax;
    wire [2:0] cnt;
    wire [7:0] act;
    wire pix,pix_v;
    reg suppress=1;
    integer i;
    always #5 clk=~clk;
    motion_detect dut(.I_clk(clk),.I_rst_n(rst_n),.I_de(de),
        .I_pix_x(x),.I_pix_y(y),.I_rgb(rgb),.I_frame_done(fdone),
        .O_mot_valid(valid),.O_mot_xmin(xmin),.O_mot_xmax(xmax),
        .O_mot_ymin(ymin),.O_mot_ymax(ymax),.O_mot_cnt(cnt),
        .O_mot_pix(pix),.O_mot_pix_v(pix_v),.O_act_blk(act));
    always @(negedge clk)
        if(rst_n && suppress && pix_v && pix !== 1'b0)
            $fatal(1,"startup/reference frame produced motion or X");
    task boundary;
        begin
            @(negedge clk); de=0; fdone=1;
            @(negedge clk); fdone=0;
        end
    endtask
    task frame;
        input integer first_y;
        input integer object_on;
        integer px,py;
        reg [7:0] level;
        begin
            for(py=first_y;py<720;py=py+1)
                for(px=0;px<1280;px=px+1) begin
                    @(negedge clk);
                    de=1; x=px; y=py;
                    level=(object_on && px>=192 && px<=511 && py>=128 && py<=447)?200:0;
                    rgb={level,level,level};
                end
            @(negedge clk); de=0;
            repeat(100) @(negedge clk);
        end
    endtask
    task empty_result;
        begin
            if(valid !== 0 || cnt !== 0 || act !== 0 ||
               xmin !== 0 || xmax !== 0 || ymin !== 0 || ymax !== 0)
                $fatal(1,"expected empty result: valid=%h cnt=%h act=%h",valid,cnt,act);
        end
    endtask
    initial begin
        // Deliberately hostile power-on data, unlike the all-black first frame.
        for(i=0;i<57600;i=i+1) dut.S_fbuf[i]=255;
        repeat(5) @(negedge clk); rst_n=1;
        boundary; frame(0,0); boundary; empty_result;
        suppress=0;
        frame(0,1); boundary;
        if(valid !== 4'b0001 || cnt !== 1 || xmin !== 192 || xmax !== 511 ||
           ymin !== 128 || ymax !== 447)
            $fatal(1,"real motion lost or bbox wrong: %h %d %d %d %d",valid,xmin,xmax,ymin,ymax);
        frame(0,1); boundary; empty_result;
        // Reset mid-frame: stale RAM contains a target while incoming pixels are black.
        suppress=1; rst_n=0;
        repeat(5) @(negedge clk); rst_n=1;
        frame(500,0); boundary; empty_result;
        frame(0,0); boundary; empty_result;
        suppress=0;
        frame(0,1); boundary;
        if(valid !== 1 || cnt !== 1 || xmin !== 192 || xmax !== 511 ||
           ymin !== 128 || ymax !== 447)
            $fatal(1,"motion did not recover after mid-frame reset");
        $display("MOTION_STARTUP PASS: hostile RAM, warmup, static frame, mid-frame reset, recovery");
        $finish;
    end
endmodule
