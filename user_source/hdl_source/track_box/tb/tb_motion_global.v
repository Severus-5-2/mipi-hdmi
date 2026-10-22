`timescale 1ns/1ps
module tb_motion_global;
reg clk=0,rst=0,fd=0;wire[3:0] valid;wire[2:0] count;wire[43:0] xmin,xmax;wire[39:0] ymin,ymax;
always #5 clk=~clk;
motion_detect dut(.I_clk(clk),.I_rst_n(rst),.I_de(1'b0),.I_pix_x(11'd0),.I_pix_y(10'd0),.I_rgb(24'd0),.I_frame_done(fd),
 .O_mot_valid(valid),.O_mot_cnt(count),.O_mot_xmin(xmin),.O_mot_xmax(xmax),.O_mot_ymin(ymin),.O_mot_ymax(ymax));
task publish;
 input integer xa,xb,ya,yb,want;
 begin
  @(negedge clk);dut.S_boxv[0]=1;dut.S_bx1[0]=xa;dut.S_bx2[0]=xb;dut.S_by1[0]=ya;dut.S_by2[0]=yb;fd=1;
  @(negedge clk);fd=0;
  if(count!==want || valid!==(want?4'b1:4'b0)) $fatal(1,"global-change gate failed");
  if(!want && {xmin,xmax,ymin,ymax}!==0) $fatal(1,"rejected bbox not cleared");
 end
endtask
initial begin
 repeat(5) @(negedge clk);rst=1;
 publish(0,19,0,10,0);publish(1,18,1,9,0);publish(3,7,2,6,1);publish(1,18,1,8,1);
 $display("MOTION_GLOBAL PASS: global unions rejected, local motion retained");$finish;
end
endmodule
