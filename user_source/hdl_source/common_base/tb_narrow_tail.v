`timescale 1ns/1ps
module tb_narrow_tail;
    reg clk=0,rst=0,de=0,mask=0,fd=0;
    reg [10:0] x=0;
    reg [9:0] y=0;
    wire cv;
    wire [19:0] area;
    wire [9:0] ymax;
    integer px,py,w,seen=0;
    always #5 clk=~clk;
    conn_label #(.IMG_WIDTH(64),.IMG_HEIGHT(16),.CONTINUE_SHORT(1)) dut(
        .clk(clk),.rst_n(rst),.I_de(de),.I_mask(mask),.I_color(2'd0),
        .I_pix_x(x),.I_pix_y(y),.I_frame_done(fd),.O_cmp_valid(cv),.O_cmp_area(area),.O_cmp_ymax(ymax));
    always @(negedge clk) if(cv) begin
        seen=seen+1;
        if(seen==1 && (area!==34 || ymax!==6)) $fatal(1,"connected tail lost: area=%d ymax=%d",area,ymax);
        if(seen==2 && (area!==16 || ymax!==15)) $fatal(1,"last-row/right-edge event lost: area=%d ymax=%d",area,ymax);
    end
    initial begin
        repeat(4) @(negedge clk); rst=1; fd=1;
        @(negedge clk); fd=0;
        for(py=0;py<16;py=py+1) begin
            case(py) 2:w=8; 3:w=12; 4:w=8; 5:w=4; 6:w=2; default:w=0; endcase
            for(px=0;px<64;px=px+1) begin
                @(negedge clk); de=1; x=px; y=py;
                mask=((px>=24 && px<24+w) || (py==10 && px>=40 && px<44) || (py>=14 && px>=56));
            end
            @(negedge clk); de=0; mask=0;
            repeat(16) @(negedge clk);
        end
        repeat(3000) @(negedge clk);
        if(seen!==2) $fatal(1,"isolated short noise allocated label or missing summary: %d",seen);
        $display("NARROW_TAIL PASS"); $finish;
    end
endmodule
