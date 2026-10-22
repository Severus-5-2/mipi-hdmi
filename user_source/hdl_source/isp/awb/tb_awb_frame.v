`timescale 1ns/1ps
// Control/arithmetic regression; the divider and delay below are latency models.
module tb_awb_frame;
reg clk=0,rst=0,valid=0,user=0;
reg [95:0] data=0;
wire ov; wire [95:0] od;
integer f=0,j,k,rr,gg,bb,checked=0;
reg [19:0] hold_r,hold_b;
always #5 clk=~clk;
awb #(.IMG_WIDTH(384),.IMG_HEIGHT(1)) dut(.I_clk(clk),.I_rst_n(rst),
 .I_tvalid(valid),.I_tuser(user),.I_tlast(1'b0),.I_tdata(data),.O_tready(1'b1),.O_tvalid(ov),.O_tdata(od));
always @(negedge clk) if(rst && ov && f>=22) begin
 rr=0;gg=0;bb=0;
 for(k=0;k<4;k=k+1) begin rr=rr+od[k*24+16 +:8];gg=gg+od[k*24+8 +:8];bb=bb+od[k*24 +:8];end
 if(rr<388 || rr>408 || gg!==400 || bb<388 || bb>408) $fatal(1,"frame mean not neutral: %d %d %d",rr,gg,bb);
 checked=checked+1;
end
initial begin
 repeat(5) @(negedge clk);rst=1;
 for(f=0;f<24;f=f+1) begin
  user=1; @(negedge clk);user=0;
  hold_r=dut.in_r_r1;hold_b=dut.in_b_r1;
  for(j=0;j<96;j=j+1) begin
   valid=1;data={24'h286432,24'h286432,24'h506464,24'h286432};
   @(negedge clk);
   if(dut.in_r_r1!==hold_r || dut.in_b_r1!==hold_b) $fatal(1,"AWB gain changed within frame");
  end
  valid=0;repeat(80) @(negedge clk);
 end
 if(checked<100) $fatal(1,"insufficient output coverage");
 if(dut.in_r_r1<130400 || dut.in_r_r1>131700 || dut.in_b_r1<104200 || dut.in_b_r1>105500)
  $fatal(1,"all-lane statistics failed: gains=%h/%h",dut.in_r_r1,dut.in_b_r1);
 if(dut.stable_gain(20'h10000,48'd0)!==20'h0e000 || dut.stable_gain(20'h10000,48'hffffffffffff)!==20'h1c000)
  $fatal(1,"gain clamp/signed smoothing failed");
 $display("AWB_FRAME PASS: all lanes, neutral mean, frame-constant gains, startup, clamps");$finish;
end
endmodule

module divider(input clk,input rst,input start,input [31:0] denominator,input [47:0] numerator,
 output done,output [47:0] quotient);
 reg [49:0] v=0;reg [47:0] q[0:49];integer i;
 always @(posedge clk) begin
  if(rst) begin v<=0;for(i=0;i<50;i=i+1) q[i]<=0;end
  else begin v<={v[48:0],start};q[0]<=denominator?numerator/denominator:48'hffffffffffff;
   for(i=1;i<50;i=i+1) q[i]<=q[i-1];end
 end
 assign done=v[49];assign quotient=q[49];
endmodule
module signal_delay #(parameter IMG_HEIGHT=1,IMG_WIDTH=384,DATA_WIDTH=96,DELAY_CYCLE=50)(
 input I_clk,input I_rst_n,input I_tuser,input I_valid,input [DATA_WIDTH-1:0] I_data,
 output O_valid,output [DATA_WIDTH-1:0] O_data,output O_tuser);
 reg [DELAY_CYCLE-1:0] v=0,u=0;reg [DATA_WIDTH-1:0] d[0:DELAY_CYCLE-1];integer i;
 always @(posedge I_clk) begin
  if(!I_rst_n) begin v<=0;u<=0;for(i=0;i<DELAY_CYCLE;i=i+1)d[i]<=0;end
  else begin v<={v[DELAY_CYCLE-2:0],I_valid};u<={u[DELAY_CYCLE-2:0],I_tuser};d[0]<=I_data;
   for(i=1;i<DELAY_CYCLE;i=i+1)d[i]<=d[i-1];end
 end
 assign O_valid=v[DELAY_CYCLE-1];assign O_tuser=u[DELAY_CYCLE-1];assign O_data=d[DELAY_CYCLE-1];
endmodule
