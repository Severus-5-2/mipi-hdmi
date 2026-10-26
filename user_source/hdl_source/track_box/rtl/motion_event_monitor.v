// Shared frame difference for zone alarms and per-color movement status.
// I_gray is the existing rgb2gray output (two clocks after raw RGB).
// All state is in the HDMI pixel domain. No additional frame RAM or DSP.
module motion_event_monitor #(
    parameter integer MOT_TH = 32,
    parameter integer ROI_X0 = 320,
    parameter integer ROI_X1 = 960,  // exclusive
    parameter integer ROI_Y0 = 192,
    parameter integer ROI_Y1 = 576,  // exclusive
    parameter integer ROI_MIN_HITS = 8,
    parameter integer GLOBAL_LIMIT = 19200,
    parameter integer AUTO_CLEAR_FRAMES = 30 // consecutive quiet video frames
)(
    input wire I_clk, I_rst_n,
    input wire I_de,
    input wire [10:0] I_pix_x,
    input wire [9:0] I_pix_y,
    input wire [7:0] I_gray,
    input wire I_frame_done,
    input wire I_color_mode, // 1=color status, 0=regional alarm
    input wire I_mode_change, // synchronous pulse on the mode commit boundary
    input wire [3:0] I_box_valid,
    input wire [7:0] I_box_cls,
    input wire [43:0] I_box_xmin, I_box_xmax,
    input wire [39:0] I_box_ymin, I_box_ymax,
    output reg O_scene_busy, // Broad scene change: blue-track acquisition only
    output reg O_alarm,
    output reg O_ready,
    output reg [7:0] O_events_bcd,
    output reg [11:0] O_roi_bcd,
    // 0=NONE, 1=STILL, 2=MOVE, 3=WAIT (warming/scene disturbance/too small)
    output reg [1:0] O_red_state, O_blue_state
);
    localparam integer DS_W=320, DS_H=180;
    localparam [8:0] RX0=ROI_X0/4, RX1=(ROI_X1-1)/4;
    localparam [7:0] RY0=ROI_Y0/4, RY1=(ROI_Y1-1)/4;

    // S_gray_out follows raw HDMI RGB by TWO clocks: gamma_lut and
    // rgb2gray each register once. At x=4c+2 gray is pixel 4c; at
    // x=4c+3 gray is pixel 4c+1. Average directly on that second clock
    // so the last block also completes before active DE ends.
    reg [7:0] S_hfirst;
    reg [7:0] S_ds;
    reg S_ds_v;
    reg [8:0] S_dsx;
    reg [7:0] S_dsy;
    wire [8:0] horizontal_pair={1'b0,S_hfirst}+{1'b0,I_gray};
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_hfirst<=0; S_ds<=0; S_ds_v<=0; S_dsx<=0; S_dsy<=0;
        end else begin
            S_ds_v<=0;
            if(I_de && I_pix_y[1:0]==2'd3) begin
                if(I_pix_x[1:0]==2'd2) S_hfirst<=I_gray;
                if(I_pix_x[1:0]==2'd3) begin
                    S_ds<=horizontal_pair[8:1];
                    S_dsx<=I_pix_x[10:2]; S_dsy<=I_pix_y[9:2];
                    S_ds_v<=1;
                end
            end
        end
    end

    //==================================================================
    // 3. 帧差（上一帧 BRAM：发读 → 等数据 → 比 → 回写）
    //
    //    ⚠ 三段流水，缺一段就会「拿错地址比」：
    //      S0 拍：ds 有效 → 锁存坐标/数据 + 发读（S_raddr = 本样点地址）
    //      S1 拍：同步读的数据在**本拍末尾**才更新，S1 拍看到的还是上一个地址的数据
    //      S2 拍：S_rdata 才是本样点的上一帧值 ⇒ 比较必须放在这一拍
    //      回写：S2 拍置 S_we，S3 拍真正写入（读在前、写在后，无同址竞争）
    //==================================================================
    reg [7:0]  S_fbuf [0:DS_W*DS_H-1];
    reg [15:0] S_raddr;
    reg [7:0]  S_rdata;
    // One 320-bit ROW of previous motion flags suppresses isolated changed
    // samples. This is independent of the existing previous-frame gray RAM.
    reg line_motion [0:DS_W-1];
    reg previous_row_motion;
    reg previous_horizontal_motion;

    // RAM 不复位，保留 BRAM 推断。复位后先等待帧首，再完整写入一帧；
    // 第二个帧首起才允许报告帧差，避免上电未知数据或帧中复位的残帧误报。
    reg S_frame_seen;
    reg S_prev_frame_valid;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_frame_seen       <= 1'b0;
            S_prev_frame_valid <= 1'b0;
        end else if (I_frame_done) begin
            S_frame_seen       <= 1'b1;
            S_prev_frame_valid <= S_frame_seen;
        end
    end

    reg        S_st1;
    reg        S_st2;
    reg [7:0]  S_ds_l;
    reg [15:0] S_addr_l;
    reg [8:0]  S_dx_l;
    reg [7:0]  S_dy_l;

    reg        S_we;
    reg [15:0] S_waddr;
    reg [7:0]  S_wdata;

    // ⚠ 刻意用「移位加法」而不是 `* 16'd320`：
    //   工程 DSP 已 38/40（95%），常量乘法虽多数综合器会优化成移位，
    //   但**不能保证**不被推断成 DSP。320 = 256 + 64 ⇒
    //     S_dsy*320 = (S_dsy<<8) + (S_dsy<<6)，纯拼接 + 加法，零乘法器。
    //   数值完全等价（实测原式与移位式逐帧输出一致）。
    wire [15:0] W_addr = {S_dsy, 8'd0} + {2'b00, S_dsy, 6'd0} + {7'd0, S_dsx};

    always @(posedge I_clk) S_rdata <= S_fbuf[S_raddr];           // 同步读
    always @(posedge I_clk) if (S_we) S_fbuf[S_waddr] <= S_wdata; // 同步写

    wire [7:0]  W_diff = (S_ds_l >= S_rdata) ? (S_ds_l - S_rdata) : (S_rdata - S_ds_l);
    wire        W_mot  = (W_diff >= MOT_TH[7:0]);

    reg         S_mot_v;
    reg         S_mot;
    reg [8:0]   S_dx_p2;
    reg [7:0]   S_dy_p2;

    always @(posedge I_clk) begin
        if(S_st1) previous_row_motion<=line_motion[S_dx_l];
        if(S_st2) line_motion[S_dx_p2]<=W_mot && S_prev_frame_valid;
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_st1    <= 1'b0;
            S_st2    <= 1'b0;
            S_mot_v  <= 1'b0;
            S_mot    <= 1'b0;
            S_we     <= 1'b0;
            S_raddr  <= 16'd0;
            S_ds_l   <= 8'd0;
            S_addr_l <= 16'd0;
            S_dx_l   <= 9'd0;
            S_dy_l   <= 8'd0;
            S_dx_p2  <= 9'd0;
            S_dy_p2  <= 8'd0;
            S_waddr  <= 16'd0;
            S_wdata  <= 8'd0;
            previous_horizontal_motion<=0;
        end else begin
            // --- S0：ds 有效 → 锁存 + 发读 ---
            if (S_ds_v) begin
                S_raddr  <= W_addr;
                S_ds_l   <= S_ds;
                S_addr_l <= W_addr;
                S_dx_l   <= S_dsx;
                S_dy_l   <= S_dsy;
            end
            S_st1 <= S_ds_v;
            // --- S1：登记回写信息 + 坐标延后 ---
            if (S_st1) begin
                S_waddr <= S_addr_l;
                S_wdata <= S_ds_l;
                S_dx_p2 <= S_dx_l;
                S_dy_p2 <= S_dy_l;
            end
            S_st2 <= S_st1;
            // --- S2：S_rdata 已是本样点的上一帧值 → 比较 + 拉回写使能 ---
            S_mot_v <= 1'b0;
            S_we    <= 1'b0;
            if (S_st2) begin
                S_mot   <= W_mot && S_prev_frame_valid &&
                    ((S_dx_p2!=0 && previous_horizontal_motion) ||
                     (S_dy_p2!=0 && previous_row_motion));
                previous_horizontal_motion<=W_mot && S_prev_frame_valid;
                S_mot_v <= 1'b1;
                S_we    <= 1'b1;
            end
        end
    end



    // The four boxes are checked sequentially during the four clocks between
    // downsampled pixels. A sample can contribute only once to each color.
    wire [36:0] box0 = {I_box_valid[0],I_box_cls[1:0],I_box_xmin[10:2],I_box_xmax[10:2],I_box_ymin[9:2],I_box_ymax[9:2]};
    wire [36:0] box1 = {I_box_valid[1],I_box_cls[3:2],I_box_xmin[21:13],I_box_xmax[21:13],I_box_ymin[19:12],I_box_ymax[19:12]};
    wire [36:0] box2 = {I_box_valid[2],I_box_cls[5:4],I_box_xmin[32:24],I_box_xmax[32:24],I_box_ymin[29:22],I_box_ymax[29:22]};
    wire [36:0] box3 = {I_box_valid[3],I_box_cls[7:6],I_box_xmin[43:35],I_box_xmax[43:35],I_box_ymin[39:32],I_box_ymax[39:32]};
    reg [1:0] slot;
    reg [36:0] selected_box;
    always @* begin
        case(slot)
            0:selected_box=box0; 1:selected_box=box1;
            2:selected_box=box2; default:selected_box=box3;
        endcase
    end
    reg busy, sample_move, sample_roi, red_hit, blue_hit;
    reg [8:0] sample_x;
    reg [7:0] sample_y;
    wire box_hit = selected_box[36] && sample_x>=selected_box[33:25] &&
        sample_x<=selected_box[24:16] && sample_y>=selected_box[15:8] && sample_y<=selected_box[7:0];
    wire red_now = box_hit && selected_box[35:34]==2'd0;
    wire blue_now = box_hit && selected_box[35:34]==2'd1;
    reg stat_valid, stat_move, stat_roi, stat_red, stat_blue;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            slot<=0; busy<=0; sample_move<=0; sample_roi<=0;
            sample_x<=0; sample_y<=0; red_hit<=0; blue_hit<=0;
            stat_valid<=0; stat_move<=0; stat_roi<=0; stat_red<=0; stat_blue<=0;
        end else begin
            stat_valid<=0;
            if(busy) begin
                red_hit<=red_hit|red_now;
                blue_hit<=blue_hit|blue_now;
                if(slot==3) begin
                    stat_valid<=1; stat_move<=sample_move; stat_roi<=sample_roi;
                    stat_red<=red_hit|red_now; stat_blue<=blue_hit|blue_now;
                    busy<=0;
                end else slot<=slot+2'd1;
            end
            // On a four-clock boundary this also completes the previous
            // sample above; nonblocking assignments retain its old fields.
            if(S_mot_v) begin
                sample_x<=S_dx_p2; sample_y<=S_dy_p2; sample_move<=S_mot;
                sample_roi<=S_dx_p2>=RX0 && S_dx_p2<=RX1 && S_dy_p2>=RY0 && S_dy_p2<=RY1;
                slot<=0; busy<=1; red_hit<=0; blue_hit<=0;
            end
            if(I_frame_done) begin busy<=0; stat_valid<=0; end
        end
    end

    reg [14:0] global_hits;
    reg [7:0] roi_hits;
    // Maximum is 320*180=57600; preserve the full area ratio for big boxes.
    reg [15:0] red_samples, blue_samples, red_motion, blue_motion;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            global_hits<=0; roi_hits<=0;
            red_samples<=0; blue_samples<=0; red_motion<=0; blue_motion<=0;
        end else if(I_frame_done) begin
            global_hits<=0; roi_hits<=0;
            red_samples<=0; blue_samples<=0; red_motion<=0; blue_motion<=0;
        end else if(stat_valid) begin
            if(I_color_mode && stat_red && !(&red_samples)) red_samples<=red_samples+16'd1;
            if(I_color_mode && stat_blue && !(&blue_samples)) blue_samples<=blue_samples+16'd1;
            if(stat_move) begin
                if(global_hits<GLOBAL_LIMIT) global_hits<=global_hits+15'd1;
                if(!I_color_mode && stat_roi && !(&roi_hits)) roi_hits<=roi_hits+8'd1;
                if(I_color_mode && stat_red && !(&red_motion)) red_motion<=red_motion+16'd1;
                if(I_color_mode && stat_blue && !(&blue_motion)) blue_motion<=blue_motion+16'd1;
            end
        end
    end
    wire global_change = global_hits>=GLOBAL_LIMIT;

    // Reuse the existing noise-cleaned frame differences. Broad disturbance
    // requires >=64 changed samples in at least THREE image quadrants;
    // a small moving cap in one/two quadrants cannot trigger this criterion.
    reg [6:0] scene_q0,scene_q1,scene_q2,scene_q3;
    reg [1:0] acquisition_hold;
    reg [1:0] scene_stat_quad;
    reg previous_row_left_motion,scene_sample_motion;
    reg sample_scene_move,stat_scene_move;
    // A broad-scene sample needs a coherent 2x2 block, not only the looser
    // pair support used by the sensitive regional/target motion features.
    // This prevents independent high-gain grain across all quadrants from
    // permanently suppressing blue acquisition. No extra row/frame RAM.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            previous_row_left_motion<=0;scene_sample_motion<=0;
            sample_scene_move<=0;stat_scene_move<=0;
        end else begin
            if(S_st2) begin
                previous_row_left_motion<=previous_row_motion;
                scene_sample_motion<=W_mot && S_prev_frame_valid &&
                    S_dx_p2!=0 && S_dy_p2!=0 && previous_horizontal_motion &&
                    previous_row_motion && previous_row_left_motion;
            end
            if(S_mot_v) sample_scene_move<=scene_sample_motion;
            if(busy && slot==3) stat_scene_move<=sample_scene_move;
        end
    end
    // Coordinates travel with stat_valid; sample_x/y may already contain
    // the next sample when that valid reaches the counters.
    always @(posedge I_clk)
        if(busy && slot==3) scene_stat_quad<={sample_y>=DS_H/2,sample_x>=DS_W/2};
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin scene_q0<=0;scene_q1<=0;scene_q2<=0;scene_q3<=0;end
        else if(I_frame_done) begin scene_q0<=0;scene_q1<=0;scene_q2<=0;scene_q3<=0;end
        else if(stat_valid && stat_scene_move) begin
            case(scene_stat_quad)
                0:if(!scene_q0[6])scene_q0<=scene_q0+1'b1;
                1:if(!scene_q1[6])scene_q1<=scene_q1+1'b1;
                2:if(!scene_q2[6])scene_q2<=scene_q2+1'b1;
                3:if(!scene_q3[6])scene_q3<=scene_q3+1'b1;
            endcase
        end
    end
    wire [3:0] scene_regions={scene_q3[6],scene_q2[6],scene_q1[6],scene_q0[6]};
    wire scene_spread=(&scene_regions[2:0]) || (&scene_regions[3:1]) ||
                      (scene_regions[3] && scene_regions[0] && (scene_regions[1] || scene_regions[2]));
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin O_scene_busy<=1;acquisition_hold<=2;end
        else if(I_mode_change) begin O_scene_busy<=1;acquisition_hold<=2;end
        else if(I_frame_done) begin
            if(!S_prev_frame_valid || scene_spread || global_change) begin
                O_scene_busy<=1;acquisition_hold<=2;
            end else if(acquisition_hold!=0) begin
                O_scene_busy<=1;acquisition_hold<=acquisition_hold-1'b1;
            end else O_scene_busy<=0;
        end
    end
    wire red_present = (box0[36] && box0[35:34]==0) || (box1[36] && box1[35:34]==0) ||
        (box2[36] && box2[35:34]==0) || (box3[36] && box3[35:34]==0);
    wire blue_present = (box0[36] && box0[35:34]==1) || (box1[36] && box1[35:34]==1) ||
        (box2[36] && box2[35:34]==1) || (box3[36] && box3[35:34]==1);
    wire red_active = red_motion>=16'd2 && red_motion>=(red_samples>>5);
    wire blue_active = blue_motion>=16'd2 && blue_motion>=(blue_samples>>5);

    // Convert the latest regional activity to three decimal digits over eight
    // clocks. No division/multiplication and no long combinational BCD chain.
    reg [7:0] activity_binary;
    reg [11:0] activity_digits;
    reg [3:0] activity_steps;
    wire [3:0] bcd0=activity_digits[3:0]>=5?activity_digits[3:0]+4'd3:activity_digits[3:0];
    wire [3:0] bcd1=activity_digits[7:4]>=5?activity_digits[7:4]+4'd3:activity_digits[7:4];
    wire [3:0] bcd2=activity_digits[11:8]>=5?activity_digits[11:8]+4'd3:activity_digits[11:8];
    wire [11:0] activity_next={bcd2[2:0],bcd1,bcd0,activity_binary[7]};
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            activity_binary<=0; activity_digits<=0; activity_steps<=0; O_roi_bcd<=0;
        end else if(I_frame_done) begin
            activity_binary<=roi_hits; activity_digits<=0; activity_steps<=8;
        end else if(activity_steps!=0) begin
            activity_binary<={activity_binary[6:0],1'b0};
            activity_digits<=activity_next; activity_steps<=activity_steps-1'b1;
            if(activity_steps==1) O_roi_bcd<=activity_next;
        end
    end
    reg [7:0] red_history, blue_history;
    reg [4:0] zone_history;
    wire [7:0] next_red = {red_history[6:0],red_active};
    wire [7:0] next_blue = {blue_history[6:0],blue_active};
    wire [4:0] next_zone = {zone_history[3:0],(roi_hits>=ROI_MIN_HITS)};
    function two_hits;
        input [7:0] bits;
        // Clearing the least significant set bit leaves a nonzero value
        // exactly when there were at least two hits. No serial popcount chain.
        begin two_hits=|(bits & (bits-8'd1)); end
    endfunction
    wire zone_confirm = two_hits({3'd0,next_zone});
    reg [5:0] warm_frames;
    reg [2:0] scene_hold;
    // Saturating counter: no-motion time is restarted by each active frame.
    localparam integer QUIET_BITS=$clog2(AUTO_CLEAR_FRAMES+9);
    reg [QUIET_BITS-1:0] quiet_frames;
    localparam integer QUIET_LIMIT=(AUTO_CLEAR_FRAMES<8)?8:AUTO_CLEAR_FRAMES;
    wire roi_quiet=(roi_hits<ROI_MIN_HITS);
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_alarm<=0; O_ready<=0; O_events_bcd<=0;
            O_red_state<=0; O_blue_state<=0;
            red_history<=0; blue_history<=0; zone_history<=0;
            warm_frames<=31; scene_hold<=0; quiet_frames<=0;
        end else if(I_mode_change) begin
            // Retain only the cumulative episode count and shared gray RAM.
            // Both modes start with clean decisions, never a previous alarm.
            O_alarm<=0; O_ready<=0; O_red_state<=0; O_blue_state<=0;
            red_history<=0; blue_history<=0; zone_history<=0;
            warm_frames<=7; scene_hold<=0; quiet_frames<=0;
        end else if(I_frame_done) begin
            if(I_color_mode || !S_prev_frame_valid || warm_frames!=0 ||
               global_change || scene_hold!=0 || !roi_quiet) quiet_frames<=0;
            else if(quiet_frames<QUIET_LIMIT) quiet_frames<=quiet_frames+1'b1;

            if(!S_prev_frame_valid || warm_frames!=0 || global_change || scene_hold!=0) begin
                O_ready<=0; zone_history<=0; red_history<=0; blue_history<=0;
                O_red_state<=(I_color_mode && red_present)?2'd3:2'd0;
                O_blue_state<=(I_color_mode && blue_present)?2'd3:2'd0;
                if(S_prev_frame_valid && warm_frames!=0) warm_frames<=warm_frames-6'd1;
                if(global_change) scene_hold<=7;
                else if(scene_hold!=0) scene_hold<=scene_hold-3'd1;
            end else if(!I_color_mode) begin
                // Regional decisions cannot consume target boxes. Quietness
                // is measured from pixels inside the fixed zone, any color.
                O_red_state<=0; O_blue_state<=0;
                red_history<=0; blue_history<=0;
                if(O_alarm && roi_quiet && quiet_frames>=AUTO_CLEAR_FRAMES-1) begin
                    O_alarm<=0; O_ready<=1; zone_history<=0;
                end else if(!O_ready && quiet_frames<8) begin
                    O_ready<=0; zone_history<=0;
                end else begin
                    O_ready<=1; zone_history<=next_zone;
                    if(!O_alarm && zone_confirm) begin
                        O_alarm<=1;
                        if(O_events_bcd!=8'h99) begin
                            if(O_events_bcd[3:0]==9)
                                O_events_bcd<={O_events_bcd[7:4]+4'd1,4'd0};
                            else O_events_bcd<=O_events_bcd+8'd1;
                        end
                    end
                end
            end else begin
                // Color mode does not arm, count or latch regional events.
                O_alarm<=0; O_ready<=0; zone_history<=0;
                if(!red_present) begin O_red_state<=0; red_history<=0; end
                else if(red_samples<4) begin O_red_state<=3; red_history<=0; end
                else begin
                    red_history<=next_red;
                    O_red_state<=(two_hits(next_red) || (O_red_state==2 && |next_red))?2'd2:2'd1;
                end
                if(!blue_present) begin O_blue_state<=0; blue_history<=0; end
                else if(blue_samples<4) begin O_blue_state<=3; blue_history<=0; end
                else begin
                    blue_history<=next_blue;
                    O_blue_state<=(two_hits(next_blue) || (O_blue_state==2 && |next_blue))?2'd2:2'd1;
                end
            end
        end
    end
endmodule
