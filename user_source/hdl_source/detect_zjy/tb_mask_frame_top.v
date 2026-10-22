`timescale 1ns/1ps
module tb_mask_frame_top;
reg clk=0,rst=0,de=0,m=0,fd=0;reg [10:0] x=0;reg [9:0] y=0;
wire od,om;wire [9:0] oy;integer i,f,px,py,good=0;
always #5 clk=~clk;
mask_despeckle dut(.clk(clk),.rst_n(rst),.I_de(de),.I_mask(m),.I_mask_r(1'b0),.I_mask_g(m),.I_mask_b(1'b0),
 .I_frame_done(fd),.I_pix_x(x),.I_pix_y(y),.O_de(od),.O_mask(om),.O_pix_y(oy));
always @(negedge clk) if(rst && od) begin
 if(oy<2 && om!==0) $fatal(1,"previous frame history leaked into top rows");
 if(oy>=3 && om===1) good=good+1;
end
initial begin
 for(i=0;i<1280;i=i+1) begin dut.u_lb_keep1.mem[i]=1;dut.u_lb_keep2.mem[i]=1;end
 repeat(5) @(negedge clk);rst=1;
 for(f=0;f<2;f=f+1) begin
  for(py=0;py<6;py=py+1) begin
   for(px=0;px<64;px=px+1) begin @(negedge clk);de=1;x=px;y=py;fd=(px==0&&py==0);
    m=(px>=20&&px<46&&(py==0||py>=3));end
   @(negedge clk);de=0;m=0;fd=0;repeat(16) @(negedge clk);
  end
  repeat(50) @(negedge clk);
 end
 if(good<50) $fatal(1,"real target lost after top-row guard");
 $display("MASK_FRAME_TOP PASS: stale RAM rejected, real target retained");$finish;
end
endmodule
