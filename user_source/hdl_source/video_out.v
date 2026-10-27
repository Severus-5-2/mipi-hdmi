


module video_out #(parameter TEMPORAL_NR=0,parameter FRAME_WORDS=172800)(
    input wire I_controller_rst_n,
    input wire I_writer_en,input wire [24:0] I_writer_addr,
    input wire [3:0] I_page_complete,input wire [31:0] I_page_generation,
    output wire O_tnr_used,output wire O_tnr_fault,
    input wire        I_rst_n,

    input wire        I_ddr_clk,
    output wire       O_video_out_rd_busy,  
    input wire        I_video_in_wr_busy,   
    input wire[1:0]   I_video_out_rp,
    output wire       O_ddr_user_rd_en,
    output wire[24:0] O_ddr_user_addr,
    input wire        I_ddr_user_ready,
    input wire        I_ddr_user_rd_valid,
    input wire[127:0] I_ddr_user_rd_data,

    input wire        I_dsi_clk,
    input wire        I_video_vsync,
    input wire        I_video_rd_en,  
    output reg[23:0]  O_vdieo_data
);

    reg         S_fifo_rst;     
    reg[7:0]    S_fifo_rst_cnt;
    wire[8:0]   S_fifo_wr_num;       
    wire        S_fifo_rd_en;        
    wire[127:0] S_fifo_rd_data;      
    reg         S_video_vsync_1d;
    reg         S_video_vsync_2d;
    reg         S_video_vsync_3d;
    wire        S_video_frame_start;   
    wire        S_ddr_rd_trig;       
    reg         S_ddr_rd_valid;      
    reg[9:0]    S_ddr_rd_cnt;        
    reg[3:0]    S_fifo_rd_cnt;       
    reg         S_video_rd_en_1d;
    reg[3:0]    S_fifo_rd_cnt_1d;     
    reg[127:0]  S_fifo_rd_data_1d;    
	wire        S_fifo_emtpy;     



    localparam IMAGE_BASE_ADDR_0 = 25'd0;
    localparam IMAGE_BASE_ADDR_1 = 25'd7000000;
    localparam IMAGE_BASE_ADDR_2 = 25'd14000000;
    localparam IMAGE_BASE_ADDR_3 = 25'd21000000;


    always @(posedge I_ddr_clk) begin
        S_video_vsync_1d <= I_video_vsync;
        S_video_vsync_2d <=S_video_vsync_1d;
        S_video_vsync_3d <= S_video_vsync_2d;
    end

    assign S_video_frame_start = ~S_video_vsync_3d & S_video_vsync_2d;


    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_fifo_rst <= 1'b0;
        else
            if(S_video_frame_start)
                S_fifo_rst <= 1'b1;
            else if(S_fifo_rst_cnt == 'd10)
                S_fifo_rst <= 1'b0;
            else
                S_fifo_rst <= S_fifo_rst;
    end


    always @(posedge I_ddr_clk) begin
        if(S_fifo_rst)
            S_fifo_rst_cnt <= S_fifo_rst_cnt + 'd1;
        else
            S_fifo_rst_cnt <= 'd0;
    end

    wire fifo_push;wire [127:0] fifo_data;
    generate if(TEMPORAL_NR)begin:g_tnr
        reg running_d;
        reg pixel_underflow;
        wire reader_fault;
        always @(posedge I_dsi_clk or negedge I_rst_n)
            if(!I_rst_n)pixel_underflow<=0;
            else if(S_fifo_rst)pixel_underflow<=0;
            else if(S_fifo_rd_en && S_fifo_emtpy)pixel_underflow<=1;
        assign O_tnr_fault=reader_fault || pixel_underflow;
        always @(posedge I_ddr_clk or negedge I_controller_rst_n)
            if(!I_controller_rst_n)running_d<=0;else running_d<=I_rst_n && !S_fifo_rst;
        wire snapshot=I_rst_n && !S_fifo_rst && !running_d;
        tnr_video_reader #(.FRAME_WORDS(FRAME_WORDS)) reader(
            .clk(I_ddr_clk),.rst_n(I_controller_rst_n),.I_epoch(S_video_frame_start || !I_rst_n),
            .I_snapshot(snapshot),.I_running(I_rst_n && !S_fifo_rst),.I_writer_busy(I_video_in_wr_busy),
            .I_writer_en(I_writer_en),.I_writer_addr(I_writer_addr),
            .I_page(I_video_out_rp),.I_complete(I_page_complete),.I_generation(I_page_generation),
            .I_fifo_used(S_fifo_wr_num),.O_busy(O_video_out_rd_busy),.O_read(O_ddr_user_rd_en),.O_addr(O_ddr_user_addr),
            .I_ready(I_ddr_user_ready),.I_rd_valid(I_ddr_user_rd_valid),.I_rd_data(I_ddr_user_rd_data),
            .O_valid(fifo_push),.O_data(fifo_data),.O_used(O_tnr_used),.O_fault(reader_fault));
    end else begin:g_legacy
    reg [24:0] legacy_addr;
    /*
        这里需要注意fifo的剩余空间一定要大于将要写入的长度，因为ddr读出的时候会有至少40个时钟周期的延时，
        例如每次从ddr读出240个长度的数据，那么fifo的剩余空间最少要设为280，否则会引起fifo写满丢失数据。
    */
    assign S_ddr_rd_trig = (('d511 - S_fifo_wr_num) >= 'd300) && (!I_video_in_wr_busy) && (!S_ddr_rd_valid) && (!S_fifo_rst) && I_rst_n ? 1'b1 : 1'b0;

    /*
        fifo的深度是512，实际的深度是510，因此需要注意每次读出的长度避免为256，否则连续两次写入会引起fifo写满，丢失数据
    */
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_ddr_rd_valid <= 1'b0;
        else
			if(O_ddr_user_rd_en && S_ddr_rd_cnt == 'd239)
                S_ddr_rd_valid <= 1'b0;
            else if(S_ddr_rd_trig)
                S_ddr_rd_valid <= 1'b1;
            else
                S_ddr_rd_valid <= S_ddr_rd_valid;
    end

    assign O_ddr_user_rd_en = S_ddr_rd_valid & I_ddr_user_ready;

    always @(posedge I_ddr_clk) begin
        if(S_ddr_rd_valid)
            begin
                if(O_ddr_user_rd_en)
                    S_ddr_rd_cnt <= S_ddr_rd_cnt + 'd1;
                else
                    S_ddr_rd_cnt <= S_ddr_rd_cnt;
            end
        else    
            S_ddr_rd_cnt <= 'd0;
    end


    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            legacy_addr <= 'd0;
        else
            if(S_video_frame_start)
                begin
                    case(I_video_out_rp)
                        'd0: legacy_addr <= IMAGE_BASE_ADDR_0;
                        'd1: legacy_addr <= IMAGE_BASE_ADDR_1;
                        'd2: legacy_addr <= IMAGE_BASE_ADDR_2;
                        'd3: legacy_addr <= IMAGE_BASE_ADDR_3;
                    endcase
                end
            else if(O_ddr_user_rd_en)
                legacy_addr <= legacy_addr + 'd8;
            else    
                legacy_addr <= legacy_addr;
    end

    
    assign O_video_out_rd_busy = S_ddr_rd_trig | S_ddr_rd_valid;


    assign O_ddr_user_addr=legacy_addr;
    assign fifo_push=I_ddr_user_rd_valid;assign fifo_data=I_ddr_user_rd_data;
    assign O_tnr_used=0;assign O_tnr_fault=0;
    end endgenerate

    w128_d512_fifo U_w128_d512_fifo(
        .rst        ( S_fifo_rst | (TEMPORAL_NR && !I_rst_n) ),   

        .clkw       ( I_ddr_clk           ),     
        .we         ( fifo_push ),
        .di         ( fifo_data ),
        .wrusedw    ( S_fifo_wr_num       ),
        .afull      (),
        .full_flag  (),
 
        .clkr       ( I_dsi_clk           ),
        .re         ( S_fifo_rd_en        ), 
        .dout       ( S_fifo_rd_data      ),  
        .rdusedw    (), 
        .valid      (),  
        .empty_flag ( S_fifo_emtpy        ),
        .aempty     ()
    );


   always @(posedge I_dsi_clk) begin
       if(I_video_rd_en)
           S_fifo_rd_cnt <= S_fifo_rd_cnt + 'd1;
       else
           S_fifo_rd_cnt <= 'd0;
   end

   assign S_fifo_rd_en = I_video_rd_en && (S_fifo_rd_cnt == 'd0)  ? 1'b1 :
                         I_video_rd_en && (S_fifo_rd_cnt == 'd5)  ? 1'b1 :
                         I_video_rd_en && (S_fifo_rd_cnt == 'd10) ? 1'b1 : 1'b0;

   always @(posedge I_dsi_clk) begin
       S_video_rd_en_1d  <= I_video_rd_en;
       S_fifo_rd_cnt_1d  <= S_fifo_rd_cnt;
       S_fifo_rd_data_1d <= S_fifo_rd_data;
   end

   always @(posedge I_dsi_clk) begin
       if(S_video_rd_en_1d)
           begin
               case(S_fifo_rd_cnt_1d)
                   'd0: O_vdieo_data  <= S_fifo_rd_data[127:104];
                   'd1: O_vdieo_data  <= S_fifo_rd_data[103:80];
                   'd2: O_vdieo_data  <= S_fifo_rd_data[79:56];
                   'd3: O_vdieo_data  <= S_fifo_rd_data[55:32];
                   'd4: O_vdieo_data  <= S_fifo_rd_data[31:8];
                   'd5: O_vdieo_data  <= {S_fifo_rd_data_1d[7:0],S_fifo_rd_data[127:112]};
                   'd6: O_vdieo_data  <= S_fifo_rd_data[111:88];
                   'd7: O_vdieo_data  <= S_fifo_rd_data[87:64];
                   'd8: O_vdieo_data  <= S_fifo_rd_data[63:40];
                   'd9: O_vdieo_data  <= S_fifo_rd_data[39:16];
                   'd10: O_vdieo_data <= {S_fifo_rd_data_1d[15:0],S_fifo_rd_data[127:120]};
                   'd11: O_vdieo_data <= S_fifo_rd_data[119:96];
                   'd12: O_vdieo_data <= S_fifo_rd_data[95:72];
                   'd13: O_vdieo_data <= S_fifo_rd_data[71:48];
                   'd14: O_vdieo_data <= S_fifo_rd_data[47:24];
                   'd15: O_vdieo_data <= S_fifo_rd_data[23:0];
               endcase
           end
       else
           O_vdieo_data <= 'd0;
   end

    
endmodule

module tnr_pair_transport #(parameter MAX_READS=64)(
 input wire clk,rst_n,
 input wire I_epoch_start,I_ref_ok,
 input wire I_req_valid,input wire[24:0] I_current_addr,I_reference_addr,
 output wire O_req_ready,
 output wire O_cmd_valid,output wire[24:0] O_cmd_addr,input wire I_cmd_ready,
 input wire I_rd_valid,input wire[127:0] I_rd_data,
 output wire O_pair_valid,output wire[127:0] O_current,O_reference,
 output wire O_reference_usable,output reg O_protocol_fault,output wire O_pending_reference
);
localparam CW=$clog2(MAX_READS+1);
reg[CW-1:0] outstanding;
reg pending_reference,receive_reference,discard_epoch,reference_good;
assign O_pending_reference=pending_reference;
reg[24:0] reference_addr;
reg[127:0] current_word;
wire can_start=!pending_reference && !discard_epoch && !I_epoch_start && outstanding<MAX_READS-1;
assign O_req_ready=can_start && I_cmd_ready;
assign O_cmd_valid=pending_reference || (can_start && I_req_valid);
assign O_cmd_addr=pending_reference?reference_addr:I_current_addr;
wire fire=O_cmd_valid && I_cmd_ready;
assign O_pair_valid=I_rd_valid && receive_reference && !discard_epoch && !I_epoch_start;
assign O_current=current_word;assign O_reference=I_rd_data;
assign O_reference_usable=reference_good && I_ref_ok;
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  outstanding<=0;pending_reference<=0;receive_reference<=0;
  discard_epoch<=0;reference_good<=0;reference_addr<=0;current_word<=0;
  O_protocol_fault<=0;
 end else begin
  case({fire,I_rd_valid})
   2'b10:outstanding<=outstanding+1'b1;
   2'b01:outstanding<=outstanding-1'b1;
   default:outstanding<=outstanding;
  endcase
  if(I_rd_valid && outstanding==0)O_protocol_fault<=1;
  if(fire && outstanding>=MAX_READS && !I_rd_valid)O_protocol_fault<=1;
  if(fire)begin
   if(pending_reference)pending_reference<=0;
   else begin pending_reference<=1;reference_addr<=I_reference_addr;end
  end
  if(I_epoch_start)begin discard_epoch<=1;reference_good<=I_ref_ok;end
  else begin
   if(discard_epoch && outstanding==0 && !pending_reference)begin
    discard_epoch<=0;
    if(receive_reference)O_protocol_fault<=1;
   end
   if(!I_ref_ok)reference_good<=0;
  end
  if(I_rd_valid)begin
   receive_reference<=!receive_reference;
   if(!receive_reference)current_word<=I_rd_data;

  end
 end
end
endmodule

// Packed RGB888 temporal filter. A small word window shares one conservative
// motion decision; the two adjoining bytes cover split RGB pixels. Any strong
// component change makes the whole word current-only, preserving color edges.
// This is two-frame averaging, not recursive history. Threshold is epoch-held.
module tnr_rgb_word_filter(
 input wire clk,rst_n,I_epoch,
 input wire I_valid,I_good,
 input wire [127:0] I_current,I_reference,
 input wire [7:0] I_threshold,
 output reg O_valid,output reg [127:0] O_data,output reg O_blended
);
reg [127:0] cur,refword;
reg [15:0] tail_cur,tail_ref;
reg good,tail_good,pending;
reg [1:0] phase,next_phase;
reg [7:0] threshold;
wire process=pending && (phase==2 || I_valid);
wire [15:0] left_cur=(phase==0)?cur[127:112]:tail_cur;
wire [15:0] left_ref=(phase==0)?refword[127:112]:tail_ref;
wire [15:0] right_cur=(phase==2)?cur[15:0]:I_current[127:112];
wire [15:0] right_ref=(phase==2)?refword[15:0]:I_reference[127:112];
wire [159:0] wc={left_cur,cur,right_cur},wr={left_ref,refword,right_ref};
wire [19:0] close_byte;
genvar k;
generate for(k=0;k<20;k=k+1)begin:g_diff
 wire [7:0] a=wc[k*8 +:8],b=wr[k*8 +:8];
 wire [7:0] delta=(a>=b)?a-b:b-a;
 assign close_byte[k]=delta<=threshold;
end endgenerate
wire use_reference=good && (phase==0 || tail_good) && (phase==2 || I_good) && (&close_byte);
wire [127:0] average;
generate for(k=0;k<16;k=k+1)begin:g_mean
 wire [8:0] sum={1'b0,cur[k*8 +:8]}+{1'b0,refword[k*8 +:8]}+9'd1;
 assign average[k*8 +:8]=sum[8:1];
end endgenerate
always @(posedge clk or negedge rst_n) begin
 if(!rst_n)begin
  cur<=0;refword<=0;tail_cur<=0;tail_ref<=0;good<=0;tail_good<=0;
  pending<=0;phase<=0;next_phase<=0;threshold<=0;O_valid<=0;O_data<=0;O_blended<=0;
 end else if(I_epoch)begin pending<=0;next_phase<=0;O_valid<=0;O_blended<=0;tail_good<=0;end
 else begin
  O_valid<=process;O_blended<=process && use_reference;
  if(process)begin
   O_data<=use_reference?average:cur;
   tail_cur<=cur[15:0];tail_ref<=refword[15:0];tail_good<=good;pending<=0;
  end
  if(I_valid)begin
   cur<=I_current;refword<=I_reference;good<=I_good;pending<=1;phase<=next_phase;threshold<=I_threshold;
   next_phase<=(next_phase==2)?2'd0:next_phase+1'b1;
  end
 end
end
endmodule

module tnr_video_reader #(parameter FRAME_WORDS=172800,parameter BURST_WORDS=96)(
 input wire clk,rst_n,I_epoch,I_snapshot,I_running,
 input wire I_writer_busy,I_writer_en,input wire [24:0] I_writer_addr,
 input wire [1:0] I_page,input wire [3:0] I_complete,input wire [31:0] I_generation,
 input wire [8:0] I_fifo_used,
 output wire O_busy,output wire O_read,output wire [24:0] O_addr,
 input wire I_ready,I_rd_valid,input wire [127:0] I_rd_data,
 output wire O_valid,output wire [127:0] O_data,
 output reg O_used,output reg O_fault
);
function [24:0] base;
 input [1:0] p;
 begin case(p)0:base=0;1:base=7000000;2:base=14000000;3:base=21000000;endcase end
endfunction
function [1:0] page;
 input [24:0] a;
 begin page=(a>=21000000)?3:(a>=14000000)?2:(a>=7000000)?1:0;end
endfunction
reg [1:0] cp,rp;
reg reference_good;
reg snapshot_done;
reg [17:0] frame_word;
wire [24:0] current_addr=base(cp)+{4'b0,frame_word,3'b0};
wire [24:0] reference_addr=base(rp)+{4'b0,frame_word,3'b0};

reg burst;
reg [6:0] burst_words;
reg [9:0] reserved_words;
reg [1:0] reservation_phase;
wire [1:0] previous_page=I_page-2'd1;
wire [7:0] generation_gap=I_generation[I_page*8 +:8]-I_generation[previous_page*8 +:8];
wire snapshot_good=I_complete[I_page] && I_complete[previous_page] && generation_gap==1;
// Complete flags clear on the first queued overwrite. reference_good is
// sticky-invalid until snapshot, so a newly completed generation cannot revive it.
wire alive=I_complete[cp] && I_complete[rp] && !(I_writer_en && (page(I_writer_addr)==cp || page(I_writer_addr)==rp));
wire reference_ok=reference_good && alive;
wire [10:0] available=11'd510-{2'b00,I_fifo_used};
wire enough_burst=available>={1'b0,reserved_words}+BURST_WORDS+32;
wire start=I_running && snapshot_done && !I_epoch && !I_snapshot && !burst && !I_writer_busy && !I_writer_en &&
 frame_word<FRAME_WORDS && enough_burst;
wire [10:0] needed={1'b0,reserved_words}+11'd3+11'd32;
wire space_ok=reservation_phase!=0 || available>=needed;
wire req_valid=burst && burst_words<BURST_WORDS && I_running && space_ok;
wire restart=I_epoch || I_snapshot;
wire transport_ref_ok=I_snapshot?snapshot_good:reference_ok;
wire req_ready,cmd_valid,cmd_ready,pending_ref,pair_valid,pair_good,transport_fault;
wire [24:0] command_addr;
wire [127:0] current_word,reference_word;
wire take=req_valid && req_ready;
wire reserve_block=take && reservation_phase==0;
assign cmd_ready=I_ready && !I_writer_busy && !I_writer_en;
assign O_read=cmd_valid && cmd_ready;
assign O_addr=command_addr;
assign O_busy=start || burst || pending_ref;
tnr_pair_transport #(.MAX_READS(64)) transport(
 .clk(clk),.rst_n(rst_n),.I_epoch_start(restart),.I_ref_ok(transport_ref_ok),
 .I_req_valid(req_valid),.I_current_addr(current_addr),
 .I_reference_addr(reference_ok?reference_addr:current_addr),.O_req_ready(req_ready),
 .O_cmd_valid(cmd_valid),.O_cmd_addr(command_addr),.I_cmd_ready(cmd_ready),
 .I_rd_valid(I_rd_valid),.I_rd_data(I_rd_data),.O_pair_valid(pair_valid),.O_current(current_word),.O_reference(reference_word),
 .O_reference_usable(pair_good),.O_protocol_fault(transport_fault),.O_pending_reference(pending_ref));
wire filtered_valid,blended;
tnr_rgb_word_filter filter(
 .clk(clk),.rst_n(rst_n),.I_epoch(restart),.I_valid(pair_valid),.I_good(pair_good && reference_ok),
 .I_current(current_word),.I_reference(reference_word),.I_threshold(8'd24),
 .O_valid(filtered_valid),.O_data(O_data),.O_blended(blended));
assign O_valid=filtered_valid && !restart && I_running;
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  cp<=0;rp<=0;reference_good<=0;snapshot_done<=0;
  frame_word<=0;burst<=0;burst_words<=0;reserved_words<=0;reservation_phase<=0;O_used<=0;O_fault<=0;
 end else begin
  if(transport_fault)O_fault<=1;
  if(restart)begin
   cp<=I_page;rp<=previous_page;
   reference_good<=I_snapshot && snapshot_good;snapshot_done<=I_snapshot;
   frame_word<=0;burst<=0;burst_words<=0;reserved_words<=0;reservation_phase<=0;O_used<=0;
  end else begin
   if(!alive)begin reference_good<=0;O_used<=0;end
   else if(O_valid && blended)O_used<=1;
   if(start)begin burst<=1;burst_words<=0;end
   if(take)begin
    frame_word<=frame_word+1'b1;burst_words<=burst_words+1'b1;
    reservation_phase<=(reservation_phase==2)?2'd0:reservation_phase+1'b1;
   end
   // The last pair's reference must be submitted before yielding to writes.
   if(burst && burst_words==BURST_WORDS && O_read)burst<=0;
   case({reserve_block,O_valid})
    2'b10:reserved_words<=reserved_words+10'd3;
    2'b01:reserved_words<=reserved_words-1'b1;
    2'b11:reserved_words<=reserved_words+10'd2;
   endcase
   if(filtered_valid && !I_running)O_fault<=1;
   if(O_valid && (reserved_words==0 || available==0))O_fault<=1;
  end
 end
end
endmodule
