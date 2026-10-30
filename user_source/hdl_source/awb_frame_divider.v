// Sparse AWB-only unsigned divider. Starts are at least 256 cycles apart in
// awb.v (neutral_count qualification); no change to the general vendor IP.
// Preserve its 50-stage valid timing, including in-flight results across rst:
// the vendor pipeline has no local reset, only its output stage is reset.
module awb_frame_divider(
 input wire clk,rst,start,
 input wire [31:0] denominator,
 input wire [47:0] numerator,
 output reg done,
 output reg [47:0] quotient,
 output reg [31:0] remainder
);
 reg busy=0;
 reg [5:0] bit_index;
 reg [47:0] q_shift;
 reg [31:0] divisor,rem_shift;
 reg divisor_zero;
 reg [48:0] valid_pipe=0;
 wire [32:0] trial_rem={rem_shift,q_shift[47]};
 wire [32:0] delta=trial_rem-{1'b0,divisor};
 wire take=divisor_zero || !delta[32];
 always @(posedge clk) begin
  valid_pipe<={valid_pipe[47:0],start};
  if(start) begin
   busy<=1;bit_index<=0;q_shift<=numerator;
   divisor<=denominator;divisor_zero<=denominator==0;rem_shift<=0;
  end else if(busy) begin
   q_shift<={q_shift[46:0],take};
   rem_shift<=take?delta[31:0]:trial_rem[31:0];
   if(bit_index==47) busy<=0;
   else bit_index<=bit_index+1'b1;
  end
 end
 always @(posedge clk or posedge rst) begin
  if(rst)begin done<=0;quotient<=0;remainder<=0;end
  else begin
   done<=valid_pipe[48];
   if(valid_pipe[48])begin quotient<=q_shift;remainder<=rem_shift;end
  end
 end
endmodule
