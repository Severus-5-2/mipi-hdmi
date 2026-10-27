

`include "common_base/common_defs.vh"

module design_top_wrapper (
    input wire        I_sys_clk,
    input wire        I_rst_n,
      
    output wire       O_cam_scl,
    inout  wire       IO_cam_sda,
    output wire       O_cam_24m,
    output wire       O_cam_rst,
      
    inout wire [1:0]  I_button,    // 板载轻触按键，低有效：[0]=KEY1(D5)=减，[1]=KEY2(A9)=增
    input             I_sw1,   // 拨码开关SW1（A4），拨上=0，拨下=1（实测确认）
    input             I_sw2,   // 拨码开关SW2（B4），拨上=0，拨下=1（实测确认）
    input             I_sw4,   // SW4(D11): high=three-color + red/blue motion; low=zone alarm


    output wire       O_screen_pwm,
    output wire       O_tmds_ch0_p,
    output wire       O_tmds_ch1_p,
    output wire       O_tmds_ch2_p,
    output wire       O_tmds_clk_p,

    inout wire        IO_rx_clk_pad_n, 
    inout wire        IO_rx_clk_pad_p, 
    inout wire[3:0]   IO_rx_data_pad_n,
    inout wire[3:0]   IO_rx_data_pad_p,

    output wire[12:0] ddr_addr,
    output wire[ 1:0] ddr_ba,
    output wire[ 0:0] ddr_cke,
    output wire[ 0:0] ddr_odt,
    output wire[ 0:0] ddr_cs_n,
    output wire       ddr_ras_n,
    output wire       ddr_cas_n,
    output wire       ddr_we_n,
    output wire[ 0:0] ddr_ck_p,
    output wire[ 0:0] ddr_ck_n,
    inout wire[1:0]   ddr_dm,
    inout wire[15:0]  ddr_dq,
    inout wire[1:0]   ddr_dqs_p,
    inout wire[1:0]   ddr_dqs_n   
);


    wire        	S_100m_clk;
    wire        	S_24m_clk;
    wire        	S_aux_50m_clk;
	wire        	S_10m_clk;
		
	wire        	S_pll_lock;
    wire        	S_rst_n;
	reg  [17:0] 	S_cam_power_cnt;
	reg         	S_cam_pwdnb;
	reg         	S_cam_i2c_ready;
	(* keep = "true" *) wire [15:0] S_cam_sensor_id;
	(* keep = "true" *) wire        S_cam_sensor_id_ok;
	(* keep = "true" *) wire        S_cam_i2c_error;
	localparam integer CAM_PWDNB_LOW_CYCLES = 23810;  // >= 1 ms at 23.809524 MHz
	localparam integer CAM_I2C_START_CYCLES = 142857; // release + >= 5 ms
	
	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire [1:0] S_raw_nr_level;
    wire [27:0] S_ae_quality_status;
    wire [1:0] S_ae_highlight;
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire 			S_ae_req; 
	
	wire        	S_csi_rx_clk;  
    wire        	S_hs_rx_valid;  
    wire[15:0]  	S_hs_rx_data;   
	wire[1:0]   	S_lane_error;
	
	wire        	S_csi_frame_start;       
	wire        	S_csi_frame_end;         
	wire        	S_csi_valid;             
	wire[31:0]  	S_csi_data;  
		
	wire        	S_raw10_frame_start; 
	wire        	S_raw10_frame_end;   
	wire        	S_raw10_valid;       
	wire [39:0] 	S_raw10_data;
		
	wire 			S_axis_tlast;  
	wire 			S_axis_tuser;  
	wire [39:0]		S_axis_tdata;  
	wire 			S_axis_tvalid; 
	
	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;
	
	wire			S_ISP_O_tready;
	wire [127:0]	S_ISP_O_tdata ;
	wire 			S_ISP_O_tlast ;
	wire 			S_ISP_O_tuser ;
	wire 			S_ISP_O_tvalid; 
	

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;


    wire        S_vi_128b_frame_start;
    wire        S_vi_128b_valid;      
    wire[127:0] S_vi_128b_data;    
    
    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy; 
    wire[1:0]   S_video_out_rp;     

    wire        S_ddr_user_wr_en;       
    wire        S_ddr_user_rd_en;       
    wire[24:0]  S_ddr_user_addr;        
    wire[127:0] S_ddr_user_wr_data;     
    wire        S_ddr_user_ready;       
    wire        S_ddr_user_rd_valid;    
    wire[127:0] S_ddr_user_rd_data;     

    wire        S_vi_ddr_wr_en;   
    wire[24:0]  S_vi_ddr_wr_addr; 
    wire[127:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;   
    wire[24:0]  S_vo_ddr_rd_addr; 
    wire        S_vo_ddr_rd_ready;
    wire        S_vo_dbg_fifo_rst;
    wire        S_vo_dbg_ddr_rd_valid;
    wire        S_vo_dbg_fifo_rd_en;
    wire        S_vo_dbg_fifo_empty;

    wire        S_init_calib_complete; 
    wire[24:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[127:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[15:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[127:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       
    wire        S_mc_dbg_fifo_afull;
    wire        S_mc_dbg_app_rdy;
    wire        S_mc_dbg_wdf_rdy;
    wire        S_mc_dbg_cmd_pop;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;

    wire[23:0]  S_video_rd_data;
    reg         S_dbg_raw_seen;
    reg         S_dbg_isp_seen;
    reg         S_dbg_ddr_wr_seen;
    reg         S_dbg_ddr_rd_seen;
    reg         S_dbg_cam_cfg_seen;
    reg         S_dbg_hs_seen;
    reg         S_dbg_csi_seen;
    reg         S_dbg_raw10_seen;
    reg         S_dbg_isp_fs_seen;
    reg         S_dbg_isp_valid_seen;
    reg         S_dbg_ddr_req_seen;
    reg         S_dbg_ddr_ready_seen;
    reg [1:0]   S_dbg_raw_seen_sync;
    reg [1:0]   S_dbg_isp_seen_sync;
    reg [1:0]   S_dbg_ddr_wr_seen_sync;
    reg [1:0]   S_dbg_ddr_rd_seen_sync;
    reg [1:0]   S_dbg_cam_cfg_seen_sync;
    reg [1:0]   S_dbg_hs_seen_sync;
    reg [1:0]   S_dbg_csi_seen_sync;
    reg [1:0]   S_dbg_raw10_seen_sync;
    reg [1:0]   S_dbg_isp_fs_seen_sync;
    reg [1:0]   S_dbg_isp_valid_seen_sync;
    reg [1:0]   S_dbg_ddr_req_seen_sync;
    reg [1:0]   S_dbg_ddr_ready_seen_sync;
    reg         S_dbg_video_nonzero_seen;
    reg         S_dbg_rd_req_seen;
    reg         S_dbg_rd_valid_seen;
    reg         S_dbg_rd_nonzero_seen;
    reg [1:0]   S_dbg_rd_req_seen_sync;
    reg [1:0]   S_dbg_rd_valid_seen_sync;
    reg [1:0]   S_dbg_rd_nonzero_seen_sync;
    reg         S_dbg_mc_init_seen;
    reg         S_dbg_mc_app_rdy_seen;
    reg         S_dbg_mc_wdf_rdy_seen;
    reg         S_dbg_mc_cmd_pop_seen;
    reg [1:0]   S_dbg_mc_init_seen_sync;
    reg [1:0]   S_dbg_mc_app_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_wdf_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_cmd_pop_seen_sync;
    wire[3:0]   S_hdmi_debug_status;
    wire        S_lane_error_any;

    wire [23:0] S_proc_rgb_out;    //多路选择器输出→送hdmi_mixer
    wire [23:0] S_video_bright;    // 亮度增强后的像素（所有算法共用）
    wire [1:0] S_mode_sel;         // 拨码开关模式选择（2位！）


    //===== 新增图像处理信号（S_hdmi_pixel_clk域）=====
    wire [7:0]  S_gray_out;        //RGB转灰度输出
    wire [23:0] S_rgb_gray;        //灰度扩展为24bit
    wire [23:0] S_rgb_bin;         //二值化输出
    wire [23:0] S_rgb_sobel;       //Sobel边缘输出
    wire [23:0] S_video_sat;       // 饱和度增强后的像素（仅彩色原图模式使用）
    wire [23:0] S_video_dn;        // 色度降噪后的像素（仅彩色原图模式使用）

    //===== 张铭晨 track_box：像素坐标 + 画框 + 坐标 OSD（D1）=====
    wire [10:0] S_pix_x, S_pix_x_1d, S_pix_x_2d;
    wire [9:0]  S_pix_y, S_pix_y_1d, S_pix_y_2d;
    localparam FIELD_METER_DIAG=1'b0; // Temporary white-wall measurement overlay
    wire [359:0] S_field_bcd;
    wire [1:0] S_field_ready;
    wire        S_frame_done;      // 每帧结束脉冲（消隐期计算用）
    wire        S_box_hit;         // 画框 / 十字准星命中
    wire [23:0] S_box_color;
    wire        S_coord_hit;       // 坐标 OSD 命中
    wire [23:0] S_coord_color;

    //===== 叠加仲裁（张铭晨 + 张金艺 共用一条进入 hdmi_mixer 的通道）=====
    //  优先级（高→低）：坐标数字 OSD > 画框/准星 > 检测掩膜
    //  ★合并张金艺代码时：只把 S_mask_hit / S_mask_color 的驱动源换成 detect 模块输出，
    //    不要另开 hdmi_mixer 端口 —— 两人改同一个文件必然冲突。
    wire        S_mask_hit;        // 预留：目标检测掩膜/伪彩（张金艺 D1）
    wire [23:0] S_mask_color;      // 预留：掩膜颜色
    wire        S_ovl_hit;         // 仲裁结果 → hdmi_mixer.I_ext_hit
    wire [23:0] S_ovl_color;       // 仲裁结果 → hdmi_mixer.I_ext_color

    //===== 张金艺 D2/D4：连通域标注 + 特征汇总 → 目标总线 =====
    wire        S_det_de, S_det_fdone;      // detect_color_mask 对齐后的 de / 帧末
    wire [10:0] S_det_px;                   // 对齐后的像素坐标
    wire [9:0]  S_det_py;
    wire [1:0]  S_det_color;                // 单像素颜色类别（红>蓝>绿）

    // ★ 2026-10-12：去斑后（mask_despeckle 输出）—— 这一路才进 conn_label
    wire        S_dspk_mask;
    wire        S_dspk_mr, S_dspk_mg, S_dspk_mb;
    wire        S_dspk_de, S_dspk_fdone;
    wire [10:0] S_dspk_px;
    wire [9:0]  S_dspk_py;
    wire [1:0]  S_dspk_color;

    wire [5:0]  S_lbl;                      // conn_label 每像素 label 流
    wire        S_lbl_v, S_lbl_de;
    wire [10:0] S_lbl_x;
    wire [9:0]  S_lbl_y;

    wire        S_cmp_valid;                // conn_label 帧末汇总流
    wire [5:0]  S_cmp_label;
    wire [1:0]  S_cmp_cls;
    wire [19:0] S_cmp_area;
    wire [10:0] S_cmp_xmin, S_cmp_xmax, S_cmp_cx;
    wire [9:0]  S_cmp_ymin, S_cmp_ymax, S_cmp_cy;
    wire [5:0]  S_cmp_cnt;

    wire [`TGT_BUS_W-1:0] S_target_bus;     // conn_features 输出的 4×96 目标总线
    wire [2:0]            S_target_cnt;
    wire                  S_target_update;

    // ★张金艺 D5/D6（方案B §2.3/§2.4）：几何分类 + 类别数量 OSD
    wire [7:0]  S_shapes;                  // 每槽 2bit 形状（0=方 1=圆 2=三角 3=未知）
    wire        S_cat_hit;                 // 类别数量 OSD 命中
    wire [23:0] S_cat_color;

    wire [3:0]  S_box_valid;                // 目标总线展开 → box_draw
    wire [7:0]  S_box_cls;                  // ★每槽 2bit 类别 → box_draw 按颜色画框
    wire [43:0] S_box_xmin, S_box_xmax;
    wire [39:0] S_box_ymin, S_box_ymax;

    //==================================================================
    //  ★ 2026-10-14 合板：张铭晨 D5/D7/D8 接入（两边功能并到一起，交接版）
    //    D5 target_assoc  ：目标关联（EMA 平滑 + 漏检保持）→ 把"逐帧跳动的清单"变"稳定的框"
    //    D7 motion_event_monitor：共享帧差缓存，区域告警与红蓝动静状态
    //    D8 motion_event_osd：警戒区边界、事件次数与颜色状态文字
    //    ★ 注意：S_box_* 的**驱动源已从"总线直接展开"改为 target_assoc 的跟踪输出**
    //      —— 这才是 D5 的意义所在（否则跟踪模块等于没接）。
    //==================================================================
    wire        S_frame_end;                // pix_coord_gen 新版：帧末（场消隐）脉冲
    wire [43:0] S_trk_cx;                   // target_assoc：平滑后的质心（ILA 调试用）
    wire [39:0] S_trk_cy;
    wire [2:0]  S_track_cnt;                // 当前在跟的目标数

    localparam integer ALARM_X0=320, ALARM_X1=960;
    localparam integer ALARM_Y0=192, ALARM_Y1=576;
    wire S_event_alarm, S_event_ready, S_event_hit;
    wire [7:0] S_event_count;
    wire [1:0] S_red_motion, S_blue_motion;
    wire [23:0] S_event_color;
    wire [11:0] S_roi_activity_bcd;
    wire S_detection_full;
    reg [1:0] R_alarm_switch_sync;
    reg R_alarm_switch_stable;
    reg R_color_mode; // Debounced SW4 level, committed only at a frame boundary.
    // Seven empty lists exceed the tracker's fixed MAX_MISS=5. Reentering
    // color mode always expires every old track before admitting detections.
    reg [2:0] R_tracking_flush;
    wire S_color_tracking_active=R_color_mode && (R_tracking_flush==0);
    wire [3:0] S_color_box_valid=S_box_valid & {4{S_color_tracking_active}};
    wire [`TGT_BUS_W-1:0] S_tracking_bus;
    wire [`TGT_BUS_W-1:0] S_count_bus;
    wire [11:0] S_color_counts;
    genvar count_slot;
    generate for(count_slot=0;count_slot<4;count_slot=count_slot+1) begin: stable_color_counts
        assign S_count_bus[count_slot*`TGT_SLOT_W +: `TGT_SLOT_W]=
            {{(`TGT_SLOT_W-3){1'b0}},S_box_cls[count_slot*2 +: 2],S_color_box_valid[count_slot]};
    end endgenerate

    genvar color_slot;
    generate for(color_slot=0;color_slot<`TARGET_MAX;color_slot=color_slot+1) begin: g_color_mode_valid
        // Gate only valid, never the 95 payload bits in each target slot.
        assign S_tracking_bus[color_slot*`TGT_SLOT_W +: `TGT_SLOT_W]=
            {S_target_bus[color_slot*`TGT_SLOT_W+1 +: `TGT_SLOT_W-1],
             S_target_bus[color_slot*`TGT_SLOT_W] && S_color_tracking_active};
    end endgenerate
    reg [19:0] R_alarm_switch_debounce;
    wire S_function_change=S_det_fdone && (R_color_mode!=R_alarm_switch_stable);

    //===== 自动曝光 AE 测光信号 =====
    wire [7:0]  S_ae_y_avg;        // 帧平均亮度（74.25MHz 域，更新后保持整帧）
    wire        S_ae_frame_tog;    // 帧握手电平，每帧翻转（跨时钟域用）


    // ===== 拨码开关模式选择 =====
    // 拨码开关是静态电平：拨上=0，拨下=1（注意与上拉方向相反，实测确认），无需消抖
    // 只需双拍同步防亚稳态
    reg [1:0] sw_sync1;
    reg [1:0] sw_sync2;
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) 
      begin
        if(!S_hdmi_rst_n) 
          begin
            sw_sync1 <= 2'b00;
            sw_sync2 <= 2'b00;
          end
        else 
          begin
            sw_sync1 <= {I_sw2, I_sw1};  // {SW2, SW1} → 00/01/10/11
            sw_sync2 <= sw_sync1;
          end
      end
    assign S_mode_sel = sw_sync2;   // 输出给pixel_mux_4to1
 


    assign O_screen_pwm = 1'b1;
    assign S_video_out_vsync = ~S_hdmi_vsync;
	
	
    assign S_rst_n 		= S_pll_lock;
    assign S_hdmi_rst_n = S_pll_lock;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw_seen <= 1'b0;
        else if(S_raw10_frame_start | S_axis_tvalid)
            S_dbg_raw_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_seen <= 1'b0;
        else if(S_ISP_O_tuser | S_ISP_O_tvalid)
            S_dbg_isp_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_wr_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_wr_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_rd_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_ddr_rd_seen <= 1'b1;
    end

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_cam_cfg_seen <= 1'b0;
        else if(S_cam_cfg_done)
            S_dbg_cam_cfg_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_hs_seen <= 1'b0;
        else if(S_hs_rx_valid)
            S_dbg_hs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_csi_seen <= 1'b0;
        else if(S_csi_frame_start | S_csi_valid)
            S_dbg_csi_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw10_seen <= 1'b0;
        else if(S_raw10_frame_start | S_raw10_valid)
            S_dbg_raw10_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_fs_seen <= 1'b0;
        else if(S_ISP_O_tuser)
            S_dbg_isp_fs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_valid_seen <= 1'b0;
        else if(S_ISP_O_tvalid)
            S_dbg_isp_valid_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_req_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_ready_seen <= 1'b0;
        else if(S_ddr_user_ready)
            S_dbg_ddr_ready_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_vo_ddr_rd_en)
            S_dbg_rd_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_rd_valid_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
            S_dbg_rd_nonzero_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_init_seen <= 1'b0;
        else if(S_init_calib_complete)
            S_dbg_mc_init_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_app_rdy_seen <= 1'b0;
        else if(S_mc_dbg_app_rdy)
            S_dbg_mc_app_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_wdf_rdy_seen <= 1'b0;
        else if(S_mc_dbg_wdf_rdy)
            S_dbg_mc_wdf_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_cmd_pop_seen <= 1'b0;
        else if(S_mc_dbg_cmd_pop)
            S_dbg_mc_cmd_pop_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_dbg_cam_cfg_seen_sync <= 2'b00;
            S_dbg_hs_seen_sync      <= 2'b00;
            S_dbg_csi_seen_sync     <= 2'b00;
            S_dbg_raw10_seen_sync   <= 2'b00;
            S_dbg_raw_seen_sync    <= 2'b00;
            S_dbg_isp_seen_sync    <= 2'b00;
            S_dbg_ddr_wr_seen_sync <= 2'b00;
            S_dbg_ddr_rd_seen_sync <= 2'b00;
            S_dbg_isp_fs_seen_sync  <= 2'b00;
            S_dbg_isp_valid_seen_sync <= 2'b00;
            S_dbg_ddr_req_seen_sync <= 2'b00;
            S_dbg_ddr_ready_seen_sync <= 2'b00;
            S_dbg_video_nonzero_seen <= 1'b0;
            S_dbg_rd_req_seen_sync <= 2'b00;
            S_dbg_rd_valid_seen_sync <= 2'b00;
            S_dbg_rd_nonzero_seen_sync <= 2'b00;
            S_dbg_mc_init_seen_sync <= 2'b00;
            S_dbg_mc_app_rdy_seen_sync <= 2'b00;
            S_dbg_mc_wdf_rdy_seen_sync <= 2'b00;
            S_dbg_mc_cmd_pop_seen_sync <= 2'b00;
        end
        else begin
            S_dbg_cam_cfg_seen_sync <= {S_dbg_cam_cfg_seen_sync[0], S_dbg_cam_cfg_seen};
            S_dbg_hs_seen_sync      <= {S_dbg_hs_seen_sync[0],      S_dbg_hs_seen};
            S_dbg_csi_seen_sync     <= {S_dbg_csi_seen_sync[0],     S_dbg_csi_seen};
            S_dbg_raw10_seen_sync   <= {S_dbg_raw10_seen_sync[0],   S_dbg_raw10_seen};
            S_dbg_raw_seen_sync    <= {S_dbg_raw_seen_sync[0],    S_dbg_raw_seen};
            S_dbg_isp_seen_sync    <= {S_dbg_isp_seen_sync[0],    S_dbg_isp_seen};
            S_dbg_ddr_wr_seen_sync <= {S_dbg_ddr_wr_seen_sync[0], S_dbg_ddr_wr_seen};
            S_dbg_ddr_rd_seen_sync <= {S_dbg_ddr_rd_seen_sync[0], S_dbg_ddr_rd_seen};
            S_dbg_isp_fs_seen_sync  <= {S_dbg_isp_fs_seen_sync[0],  S_dbg_isp_fs_seen};
            S_dbg_isp_valid_seen_sync <= {S_dbg_isp_valid_seen_sync[0], S_dbg_isp_valid_seen};
            S_dbg_ddr_req_seen_sync <= {S_dbg_ddr_req_seen_sync[0], S_dbg_ddr_req_seen};
            S_dbg_ddr_ready_seen_sync <= {S_dbg_ddr_ready_seen_sync[0], S_dbg_ddr_ready_seen};
            S_dbg_rd_req_seen_sync <= {S_dbg_rd_req_seen_sync[0], S_dbg_rd_req_seen};
            S_dbg_rd_valid_seen_sync <= {S_dbg_rd_valid_seen_sync[0], S_dbg_rd_valid_seen};
            S_dbg_rd_nonzero_seen_sync <= {S_dbg_rd_nonzero_seen_sync[0], S_dbg_rd_nonzero_seen};
            S_dbg_mc_init_seen_sync <= {S_dbg_mc_init_seen_sync[0], S_dbg_mc_init_seen};
            S_dbg_mc_app_rdy_seen_sync <= {S_dbg_mc_app_rdy_seen_sync[0], S_dbg_mc_app_rdy_seen};
            S_dbg_mc_wdf_rdy_seen_sync <= {S_dbg_mc_wdf_rdy_seen_sync[0], S_dbg_mc_wdf_rdy_seen};
            S_dbg_mc_cmd_pop_seen_sync <= {S_dbg_mc_cmd_pop_seen_sync[0], S_dbg_mc_cmd_pop_seen};
            if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
                S_dbg_video_nonzero_seen <= 1'b1;
        end
    end

    assign S_lane_error_any = |S_lane_error;

    assign S_vo_dbg_fifo_rst = 1'b0;
    assign S_vo_dbg_ddr_rd_valid = 1'b0;
    assign S_vo_dbg_fifo_rd_en = 1'b0;
    assign S_vo_dbg_fifo_empty = 1'b0;
    assign S_mc_dbg_fifo_afull = 1'b0;
    assign S_mc_dbg_app_rdy = 1'b0;
    assign S_mc_dbg_wdf_rdy = 1'b0;
    assign S_mc_dbg_cmd_pop = 1'b0;

    assign S_hdmi_debug_status = {
        (|S_ddr_user_rd_data),
        S_ddr_user_rd_valid,
        S_vo_ddr_rd_en,
        S_vi_ddr_wr_en
    };

    //0. 黑电平扣除 + Gamma 校正（替代原 brightness_gain）
    //   目的：解决线性 RAW 直接上屏造成的"雾感/发暗/颜色闷"
    //   处理顺序：先扣黑电平（线性域偏移）-> 再做 Gamma 查表（非线性映射）
    //   流水线延迟与原 brightness_gain 一致（1 拍），下游算法链的对齐关系不变
    //   输出信号名沿用 S_video_bright，灰度/二值化/Sobel/mux 均无需改动
    gamma_lut #(
        .EN_GAMMA (1     ),   // 1:启用Gamma校正；改 0 可上板做 A/B 对比
        .EN_BLC   (0     ),   // ★ v5 起改为 0：黑电平已前移至 raw10_unpacket，
                              //   此处若再扣会造成重复扣除（暗部被切零）
        .BL_LEVEL (8'd16 )    // EN_BLC=0 时本参数不起作用；需要回退时把上面改回 1 即可
    ) u_gamma_lut(
        .clk    (S_hdmi_pixel_clk),
        .rst_n  (S_hdmi_rst_n),
        .rgb_in (S_video_rd_data),
        .rgb_out(S_video_bright)
    );

    //0.6 色度降噪（Gamma 之后、饱和度之前）
    //    目的：消除高增益（63x）带来的**彩色噪点**
    //    原理：转 YCbCr 后只对色度 Cb/Cr 做 3×3 可分离低通，亮度 Y 完全不动
    //          → 噪点被平滑，但边缘/纹理不会糊（人眼对色度分辨率本就低）
    //    资源：2 个行延时 BRAM；流水线设计（单级≤4 级逻辑），时序宽松
    //    延时：垂直上移 1 行 + 水平右移 1 像素，肉眼不可察；仅作用于彩色通路
    // gamma_lut registers RGB once; match its valid enable at the row RAM input.
    reg R_gamma_de,R_gamma_vsync,R_gamma_frame;
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin R_gamma_de<=0;R_gamma_vsync<=0;R_gamma_frame<=0;end
        else begin R_gamma_de<=S_hdmi_de;R_gamma_vsync<=S_hdmi_vsync;R_gamma_frame<=S_frame_done;end
    end
    chroma_denoise #(
        .IMG_WIDTH (1280  ),   // 必须与视频时序的行像素数一致，否则行缓存会错位
        .ADDR_W    (12    ),
        .NR_MODE   (2     ),
        .LUMA_NR   (1     ),   // 2:完整3×3；1:仅水平（不占BRAM）；0:旁路
        .NR_EN     (1     )    // 1:启用；0:直通（A/B 对比用）
    ) u_chroma_denoise(
        .clk    (S_hdmi_pixel_clk),
        .rst_n  (S_hdmi_rst_n),
        .I_de   (R_gamma_de),       // Matches registered gamma RGB
        .I_vsync(R_gamma_vsync),.I_frame(R_gamma_frame),
        .rgb_in (S_video_bright),
        .rgb_out(S_video_dn)
    );

    //0.5 饱和度增强（Gamma 之后、MUX 之前，仅作用于彩色原图通路）
    //    目的：解决"颜色发闷/色彩浓度不足"（无 CCM、无饱和度处理导致）
    //    原理：保持亮度 Y 不变，只把色度分量 (C-Y) 放大 SAT_GAIN 倍
    //    注意：灰度/二值/Sobel 分支数据 R=G=B，经本模块输出恒等于输入，
    //          故本模块只接 MUX 的 rgb0，其余分支仍从 S_video_bright 取，不受影响
    //    延迟：2 拍（v2 两级流水版，修复 v1 在此处造成的 -1.7ns 时序违例）
    //          仅彩色原图模式受影响，表现为画面右移 1 像素，模式互斥显示不可见
    sat_enhance #(
        .EN       (1     ),   // 1:启用饱和度增强；改 0 可上板做 A/B 对比
        // ★ 2026-10-15 下板反馈调参：现场「红/绿物体看着饱和度很低」+ 检测因色度不足而碎裂
        //   ⇒ 色度增益 1.5x → 1.75x（检测链取的就是本模块输出 S_video_sat，等于同时给检测加约 17% 色度裕量）。
        //   ⚠ 为什么不动 `common_defs` 的 `MAG_TH`：改那个常量会让 TD 的 place 阶段报
        //     `PHY-8077: AL_MAP_SEQ with unsupported ppm`（`-packarea` 把单扇出 DFF 打包进
        //     分布式 RAM 后，11 个实例落位找不到 PIB 资源）—— 已实测复现 3 次、二分定位；
        //     **改本参数（SAT_GAIN）则 place/route 正常**。等效灵敏度用这里加，不动门限常量。
        // ★ 2026-10-18 22:20：112（1.75x）→ 128（2.0x）。这是**纯常量改动**（网表拓扑不变），
        //   目的是给检测链补色度裕量 —— S_video_sat 就是检测输入，增益 +14% ⇒ 掩膜幅度 +14%
        //   ⇒ 直接改善「绿最差（|d_g| 天生小）、红只包 2/3」这两条。
        //   附带：128/64=2.0 ⇒ d×128>>6 = d×2，是纯移位，逻辑比 112 还少一点。
        //   注意：sat_enhance 保持 Y 不变 ⇒ 不影响 motion_detect（它吃 Y）。
        // ★ 2026-10-18 22:57：128（2.0x）→ 112（1.75x）【回退】。
        //   实测（第 2 次下板）：2.0x 下①画面明显过曝发黄（R/G/B 全被推到限幅附近）、
        //   ②红色/橙色物体**完全不出框** —— 因为红色本已饱和，再 ×2 后 R 削顶到 255，
        //   Cb/Cr 的比值判据失效、判不出颜色 ⇒ 掩膜没了。蓝色因色度幅度小反而正常。
        //   ⇒ 结论：1.75x 是这台相机/这个场景的合理上限，不要再往上抬。
        .SAT_GAIN (9'd112),   // 色度增益，6位小数定点：64=1.0x 96=1.5x 112=1.75x 128=2.0x
        .DEADZONE (5'd6  )    // 色度死区：|C-Y|≤该值视为灰色不增强（抑制彩色噪点），0=关闭
    ) u_sat_enhance(
        .clk    (S_hdmi_pixel_clk),
        .rst_n  (S_hdmi_rst_n),
        .rgb_in (S_video_dn),      // 色度降噪之后（原为 S_video_bright）
        .rgb_out(S_video_sat)
    );

    //0.7 自动曝光测光器（AE 闭环的第①步）
    //    统计 Gamma 之后、MUX 之前的整帧平均亮度，送给 24MHz 域的 ae_set 做控制律。
    //    注意这里统计的是【最终显示画面的亮度】，与肉眼所见一致，
    //    因此靶亮度 TARGET 可以用直觉友好的 112（中灰偏亮）。
    //    本模块只是"旁路取样"，不改动任何像素数据，对显示通路零影响。
    ae_meter #(
        .IMG_WIDTH  (1280  ),      // 必须与视频时序一致
        .IMG_HEIGHT (720   ),
        .PIPE_DELAY (1     )       // S_video_bright 比 S_hdmi_de 晚 1 拍（gamma_lut 输出寄存）
    ) u_ae_meter(
        .clk            (S_hdmi_pixel_clk),
        .rst_n          (S_hdmi_rst_n),
        .I_de           (S_hdmi_de),
        .I_rgb          (S_video_bright),
        .O_highlight(S_ae_highlight),
        .O_Y_avg        (S_ae_y_avg),
        .O_frame_toggle (S_ae_frame_tog)
    );

    //1. RGB转灰度



    //1. RGB转灰度
    rgb2gray u_rgb2gray(
        .clk      (S_hdmi_pixel_clk),
        .rst_n    (S_hdmi_rst_n),
        .rgb_in   (S_video_bright),
        .gray_out (S_gray_out)
    );

    assign S_rgb_gray = {S_gray_out, S_gray_out, S_gray_out};

    //2. 二值化
    binarize #(
        .THRESHOLD(120)
    ) u_binarize(
        .clk      (S_hdmi_pixel_clk),
        .rst_n    (S_hdmi_rst_n),
        .gray_in  (S_gray_out),
        .rgb_out  (S_rgb_bin)
    );

    //3. Sobel边缘检测
    sobel_3x3 #(
        .IMG_WIDTH (1280),
        .IMG_HEIGHT(720),
        .EDGE_TH   (130)
    ) u_sobel_3x3(
        .clk      (S_hdmi_pixel_clk),
        .rst_n    (S_hdmi_rst_n),
        .I_de     (S_hdmi_de),
        .gray_in  (S_gray_out),
        .rgb_out  (S_rgb_sobel)
    );

    //4. 四路像素选择器
    wire S_display_nr_active;
    wire S_scene_busy;
    pixel_mux_4to1 #(.DISPLAY_NR_AB(0),.DISPLAY_NR_FRAMES(240)) u_pixel_mux_4to1(
        .clk     (S_hdmi_pixel_clk),
        .rst_n   (S_hdmi_rst_n),
        .I_de(S_hdmi_de),.I_frame(S_frame_done),.O_display_nr_active(S_display_nr_active),
        .mode    (S_mode_sel),
        .rgb0    (S_video_sat),       // 彩色原图：经饱和度增强（其余分支不受影响）
        .rgb1    (S_rgb_gray),
        .rgb2    (S_rgb_bin),
        .rgb3    (S_rgb_sobel),
         .rgb_out (S_proc_rgb_out)
    );
	
	// SC520CS上电后先保持PWDNB/XSHUTDN为低，再释放并等待内部上电完成，
	// 避免PLL刚锁定就开始I2C初始化而造成寄存器偶发写入失败。
	always @(posedge S_24m_clk or negedge S_pll_lock) begin
		if (!S_pll_lock) begin
			S_cam_power_cnt <= 18'd0;
			S_cam_pwdnb <= 1'b0;
			S_cam_i2c_ready <= 1'b0;
		end else begin
			if (S_cam_power_cnt < CAM_I2C_START_CYCLES)
				S_cam_power_cnt <= S_cam_power_cnt + 1'b1;
			if (S_cam_power_cnt >= CAM_PWDNB_LOW_CYCLES)
				S_cam_pwdnb <= 1'b1;
			if (S_cam_power_cnt >= CAM_I2C_START_CYCLES - 1)
				S_cam_i2c_ready <= 1'b1;
		end
	end

	assign O_cam_rst 	= S_cam_pwdnb;
    assign O_cam_24m 	= S_24m_clk;
	
	
	
	
    PLL u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset    	 ( ~I_rst_n          ),
        
        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk4_out    ( S_hdmi_pixel_clk  ),
        .clk5_out    ( S_hdmi_serial_clk ),

        .lock        ( S_pll_lock        )

    );


	

  
  ae_set #(
      .AUTO_EN    (1      ),   // Always active, including during button adjustment
      .TARGET_DEF (8'd128 ),   // Quality target; explicit override matches ae_set default
      .WAIT_FRAMES(4'd5   )    // 两次调整间隔帧数（手册要求曝光 N+2 帧生效）
  ) u_ae_set (
      .I_clk(S_24m_clk),
      .I_rst(~S_rst_n),
      .I_btn({I_button,2'b11}),
      .I_cam_cfg_done(S_cam_cfg_done),
      .I_ae_cfg_done(S_ae_cfg_done),
      .I_highlight(S_ae_highlight),
      .I_Y_avg       (S_ae_y_avg),      // 来自 ae_meter（异步于本时钟域，内部已做同步）
      .I_frame_toggle(S_ae_frame_tog),
      .O_ae_req(S_ae_req),
      .O_ae(S_ae),
      .O_ag(S_ag),.O_quality_status(S_ae_quality_status)
  );
  
    // SC520CS原厂真实HD模式：1280x720@60、RAW10、2-Lane、828 Mbps/Lane。
    // 当前寄存器表已关闭Slave/FSIN等待，Sensor以free-run方式连续输出。
    uicfgcs520 #(
      .CLK_DIV(23809524 / 100000 - 1)
  ) u_uicfgcs520 (
      .I_clk(S_24m_clk),  //系统时钟输入
      .I_rst_n(S_cam_i2c_ready), // SC520CS power-up delay completed
      .I_ae_req(S_ae_req),
      .I_ae(S_ae),
      .I_ag(S_ag),
      .O_cam_scl(O_cam_scl),  //I2C总线，SCL时钟
      .IO_cam_sda(IO_cam_sda),  //I2C总线，SDA数据
      .O_cfg_done(S_cam_cfg_done),  //摄像头寄存器初始化完成
      .O_ae_cfg_done(S_ae_cfg_done),  //AE配置完成
      .O_sensor_id(S_cam_sensor_id),
      .O_sensor_id_ok(S_cam_sensor_id_ok),
      .O_iic_error(S_cam_i2c_error)
  );
	
    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        .HS_EQUALIZER          ( "3dB"   ),// 828 Mbps/Lane接收均衡
        .HS_VGA_GAIN           ( "8dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),
     
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
      
        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),
      
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
      
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
      
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );
	
	
 //  cwc cwc_inst
 //(
 //    .probe0(S_hs_rx_valid),
 //    .probe1(S_hs_rx_data),
 //    .probe2(S_lane_error),
 //    .clk(S_csi_rx_clk)
 //);

 //csi 解码为RAW数据
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        )
);


//解码为RAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data),

//黑电平扣除前移至此（v5 修复）：标准的 ISP 顺序应为 BLC → Demosaic → AWB。
//原先黑电平是在链路末端的 gamma_lut 里扣的，导致 AWB 统计到带 pedestal 的数据，
//白平衡被拉歪 —— 这就是画面"偏紫 + 雾感"的根因。
//此处在 RAW10 域扣 64，相当于 8bit 域的 16；同时必须把 gamma_lut 的 EN_BLC 关掉，
//否则会被扣两次，暗部直接切零。
.I_camera_black_level(10'd64)
  );


//将数据转为stream流
uial2axis #(
.IMG_WIDTH(1280),
.IMG_HEIGHT(720),
.INPUT_DATA_WIDTH(40)
) 
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw10_data       ),
.I_data_valid(S_raw10_valid      ),
.I_data_start(S_raw10_frame_start),
.I_data_end  (S_raw10_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast )
);
	

// 原厂HD模式直接输出1280x720 RAW10，不再进行FPGA固定窗口裁剪。
raw_bayer_nr u_raw_bayer_nr(
    .clk(S_csi_rx_clk),.rst_n(S_rst_n),.I_gain(S_ag),
    .I_data(S_axis_tdata),.I_valid(S_axis_tvalid),.I_user(S_axis_tuser),.I_last(S_axis_tlast),
    .O_data(S_raw_tdata),.O_valid(S_raw_tvalid),.O_user(S_raw_tuser),.O_last(S_raw_tlast),
    .O_nr_level(S_raw_nr_level)
);


//cwc1 cwc1_Inst
//  (
//      .probe0(S_axis_tuser),
//      .probe1(S_axis_tvalid),
//      .probe2(S_axis_tlast),
//      .probe3(S_axis_tdata),
//      .probe4(S_ISP_O_tuser),
//      .probe5(S_ISP_O_tvalid),
//      .probe6(S_ISP_O_tlast),
//      .probe7(S_ISP_O_tdata),
//      .probe8 (S_raw_tdata ),
//      .probe9 (S_raw_tlast ),
//      .probe10(S_raw_tvalid),
//      .probe11(S_raw_tuser ),
//      .clk(S_csi_rx_clk)
//  );


//ISP算法顶层模块
isp_top u_isp_top (
.axi4s_video_aclk(S_csi_rx_clk),
.I_rst_n         (S_rst_n),
.I_tlast         (S_raw_tlast	),
.I_tuser         (S_raw_tuser	),
.I_tdata         (S_raw_tdata	),
.I_tvalid        (S_raw_tvalid	),
.I_tdest         (),
.O_tready        (S_ISP_O_tready),
.O_tdata         (S_ISP_O_tdata ),
.O_tlast         (S_ISP_O_tlast ),
.O_tuser         (S_ISP_O_tuser ),
.O_tvalid        (S_ISP_O_tvalid),
.I_tready        ()
  );


    video_in u_video_in(
        .I_rst_n              (  S_rst_n             ),

        .I_camera_clk         ( S_csi_rx_clk          ),
        .I_camera_frame_start ( S_ISP_O_tuser  ),
        .I_camera_valid       ( S_ISP_O_tvalid ),
        .I_camera_data        ( S_ISP_O_tdata  ),
        .I_mipi_rx_error      ( 1'b0			      ),

        .I_ddr_clk            ( S_ddr_clk             ),
		.I_display_pause      ( 1'b0                  ),
        .I_video_out_rd_busy  ( S_video_out_rd_busy   ),
        .O_video_in_wr_busy   ( S_video_in_wr_busy    ),
        .O_video_out_rp       ( S_video_out_rp        ),

        .O_ddr_user_wr_en     ( S_vi_ddr_wr_en        ),
        .O_ddr_user_addr      ( S_vi_ddr_wr_addr      ),
        .O_ddr_user_wr_data   ( S_vi_ddr_wr_data      ),
        .I_ddr_user_ready     ( S_ddr_user_ready      )
    );



    assign S_ddr_user_wr_en    = S_vi_ddr_wr_en;

    assign S_ddr_user_rd_en    = S_vo_ddr_rd_en;

    assign S_ddr_user_addr     = S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                 S_vo_ddr_rd_en ? S_vo_ddr_rd_addr : 'd0;

    assign S_ddr_user_wr_data  = S_vi_ddr_wr_data;


    assign S_video_out_rst_n = S_hdmi_rst_n;

    wire [3:0] S_tnr_page_complete;
    wire [31:0] S_tnr_page_generation;
    wire S_tnr_used,S_tnr_fault;
    wire [7:0] S_tnr_statistics;
    reg [7:0] S_tnr_statistics_meta,S_tnr_statistics_sync;
    reg [1:0] S_tnr_cfg_sync,S_tnr_used_sync,S_tnr_fault_sync;
    reg [3:0] S_tnr_pages_meta,S_tnr_pages_sync;
    always @(posedge S_ddr_clk or negedge S_rst_n)
        if(!S_rst_n)S_tnr_cfg_sync<=0;else S_tnr_cfg_sync<={S_tnr_cfg_sync[0],S_ae_req};
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n)
        if(!S_hdmi_rst_n)begin S_tnr_used_sync<=0;S_tnr_fault_sync<=0;S_tnr_pages_meta<=0;S_tnr_pages_sync<=0;S_tnr_statistics_meta<=0;S_tnr_statistics_sync<=0;end
        else begin
            S_tnr_used_sync<={S_tnr_used_sync[0],S_tnr_used};S_tnr_fault_sync<={S_tnr_fault_sync[0],S_tnr_fault};
            S_tnr_pages_meta<=S_tnr_page_complete;S_tnr_pages_sync<=S_tnr_pages_meta;
            S_tnr_statistics_meta<=S_tnr_statistics;S_tnr_statistics_sync<=S_tnr_statistics_meta;
        end
    tnr_page_monitor u_tnr_page_monitor(
        .clk(S_ddr_clk),.rst_n(S_rst_n),.I_settings_busy(S_tnr_cfg_sync[1]),
        .I_enqueue_write(S_vi_ddr_wr_en),.I_enqueue_addr(S_vi_ddr_wr_addr),
        .I_mc_write(S_mc_app_en && S_mc_app_wdf_wren),.I_mc_addr(S_mc_app_addr),
        .O_complete(S_tnr_page_complete),.O_generation(S_tnr_page_generation));
    video_out #(.TEMPORAL_NR(1)) u_video_out(
        .I_controller_rst_n(S_rst_n),.I_writer_en(S_vi_ddr_wr_en),.I_writer_addr(S_vi_ddr_wr_addr),
        .I_page_complete(S_tnr_page_complete),.I_page_generation(S_tnr_page_generation),
        .O_tnr_used(S_tnr_used),.O_tnr_fault(S_tnr_fault),.O_tnr_statistics(S_tnr_statistics),
        .I_rst_n             ( S_video_out_rst_n    ),
        .I_ddr_clk           ( S_ddr_clk            ),
 
        .O_video_out_rd_busy ( S_video_out_rd_busy  ),
        .I_video_in_wr_busy  ( S_video_in_wr_busy   ),
        .I_video_out_rp      ( S_video_out_rp       ),
 
        .O_ddr_user_rd_en    ( S_vo_ddr_rd_en       ),
        .O_ddr_user_addr     ( S_vo_ddr_rd_addr     ),
        .I_ddr_user_ready    ( S_ddr_user_ready     ),
        .I_ddr_user_rd_valid ( S_ddr_user_rd_valid  ),
        .I_ddr_user_rd_data  ( S_ddr_user_rd_data   ),

        .I_dsi_clk           ( S_hdmi_pixel_clk     ),
        .I_video_vsync       ( S_video_out_vsync    ),
        .I_video_rd_en       ( S_hdmi_window_rd_en  ),
        .O_vdieo_data        ( S_video_rd_data      )
    );



    mc_to_user_interface u_mc_to_user_interface(
        .I_clk                   ( S_ddr_clk               ),
        .I_rst_n                 ( S_rst_n                 ),

        .I_ddr_user_wr_en        ( S_ddr_user_wr_en        ),
        .I_ddr_user_rd_en        ( S_ddr_user_rd_en        ),
        .I_ddr_user_addr         ( S_ddr_user_addr         ),
        .I_ddr_user_wr_data      ( S_ddr_user_wr_data      ),
        .O_ddr_user_ready        ( S_ddr_user_ready        ),
        .O_ddr_user_rd_valid     ( S_ddr_user_rd_valid     ),
        .O_ddr_user_rd_data      ( S_ddr_user_rd_data      ),
            
        .O_mc_app_en             ( S_mc_app_en             ),
        .O_mc_app_addr           ( S_mc_app_addr           ),
        .O_mc_app_cmd            ( S_mc_app_cmd            ),
        .I_mc_app_rdy            ( S_mc_app_rdy            ),
        .O_mc_app_wdf_wren       ( S_mc_app_wdf_wren       ),
        .O_mc_app_wdf_data       ( S_mc_app_wdf_data       ),
        .O_mc_app_wdf_end        ( S_mc_app_wdf_end        ),
        .O_mc_app_wdf_mask       ( S_mc_app_wdf_mask       ),
        .I_mc_app_wdf_rdy        ( S_mc_app_wdf_rdy        ),
        .I_mc_app_rd_data        ( S_mc_app_rd_data        ),
        .I_mc_app_rd_data_end    ( S_mc_app_rd_data_end    ),
        .I_mc_app_rd_data_valid  ( S_mc_app_rd_data_valid  )
    );



    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper(
        .I_sys_clk              ( I_sys_clk              ),
        .I_sys_rst_n            ( S_rst_n                ),

        .O_ddr_clk              ( S_ddr_clk              ),
        .O_init_calib_complete  ( S_init_calib_complete  ),
        .I_mc_app_addr          ( S_mc_app_addr          ),
        .I_mc_app_cmd           ( S_mc_app_cmd           ),
        .I_mc_app_en            ( S_mc_app_en            ),
        .I_mc_app_wdf_data      ( S_mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( S_mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( S_mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( S_mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( S_mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( S_mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( S_mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( S_mc_app_wdf_rdy       ),

        .ddr_addr               ( ddr_addr               ),
        .ddr_ba                 ( ddr_ba                 ),
        .ddr_cke                ( ddr_cke                ),
        .ddr_odt                ( ddr_odt                ),
        .ddr_cs_n               ( ddr_cs_n               ),
        .ddr_ras_n              ( ddr_ras_n              ),
        .ddr_cas_n              ( ddr_cas_n              ),
        .ddr_we_n               ( ddr_we_n               ),
        .ddr_ck_p               ( ddr_ck_p               ),
        .ddr_ck_n               ( ddr_ck_n               ),
        .ddr_dm                 ( ddr_dm                 ),
        .ddr_dq                 ( ddr_dq                 ),
        .ddr_dqs_p              ( ddr_dqs_p              ),
        .ddr_dqs_n              ( ddr_dqs_n              )
    );



    uivtc #(
        .H_ActiveSize ( 1280 ),
        .H_FrameSize  ( 1650 ),
        .H_SyncStart  ( 1390 ),
        .H_SyncEnd    ( 1430 ),
        .V_ActiveSize ( 720  ),
        .V_FrameSize  ( 750  ),
        .V_SyncStart  ( 725  ),
        .V_SyncEnd    ( 730  )
    )u_hdmi_vtc(
        .I_vtc_rstn    ( S_hdmi_rst_n    ),
        .I_vtc_clk     ( S_hdmi_pixel_clk ),
        .O_vtc_vs      ( S_hdmi_vsync     ),
        .O_vtc_hs      ( S_hdmi_hsync     ),
        .O_vtc_de_valid( S_hdmi_de        ),
        .O_vtc_user    ( S_hdmi_user      ),
        .O_vtc_last    ( S_hdmi_last      )
    );

    //==================================================================
    // ★张铭晨 track_box —— D1 交付：画框 + 坐标 OSD（SELF_TEST 固定框）
    //   目的：上板验证「坐标系统 + 画框对齐」是否正确。
    //   SELF_TEST=1 ⇒ 画 4 个写死坐标的框 + 白色十字准星（准星在屏心 640,360），
    //   与左下角 OSD 读数 X:0640 / Y:0360 互相印证。
    //   ★D4（张金艺）：SELF_TEST 已改 0，I_box_* 接真实检测目标（由下方
    //     conn_features 的 384bit 目标总线展开）；十字准星保留作对位基准。
    //   注：本段仅在 Rrrrrr_ 分支集成，未改动任何原有逻辑。
    //==================================================================
    pix_coord_gen #(
        .H_ACTIVE ( 1280 ),
        .V_ACTIVE ( 720  )
    )u_pix_coord_gen(
        .I_clk        ( S_hdmi_pixel_clk ),
        .I_rst_n      ( S_hdmi_rst_n     ),
        .I_de         ( S_hdmi_de        ),
        .I_last       ( S_hdmi_last      ),
        .I_vsync      ( S_hdmi_vsync     ),
        .O_pix_x      ( S_pix_x          ),
        .O_pix_y      ( S_pix_y          ),
        .O_pix_x_1d   ( S_pix_x_1d       ),
        .O_pix_x_2d   ( S_pix_x_2d       ),
        .O_pix_y_1d   ( S_pix_y_1d       ),
        .O_pix_y_2d   ( S_pix_y_2d       ),
        // ★ 2026-10-14：升级到张铭晨新版（带 O_frame_end）。
        //   新版语义：O_frame_end = 帧末（场消隐，vsync 上升沿）；O_frame_done = 帧首有效像素。
        //   ⚠ 由 O_frame_done 语义变化带来的"消费者采样时刻"问题已按冻结表 §1.1 铁律处置：
        //     三个读总线的消费者（osd_coord / shape_class / osd_category）的 I_frame_done
        //     已统一改接 S_det_fdone（与 S_target_bus 锁存同源 ⇒ 早 0 拍，绝对安全）。
        .O_frame_end  ( S_frame_end      ),
        .O_frame_done ( S_frame_done     )
    );

    box_draw #(
        .SELF_TEST ( 0 ),       // ★D4：1→0，改画真实检测目标（原 1 = 固定标定框）
        // ★2026-10-11：按「颜色类别」上色，而不是按「槽位序号」上色。
        //   原实现槽0红/槽1绿/槽2蓝/槽3黄 —— 但 conn_features 是「面积降序」排槽，
        //   槽号不代表颜色，现场表现为「蓝色物体也画红框」。
        //   cls 编码（common_defs.vh / conn_label）：0=红 1=蓝 2=绿 3=未分类
        .COLOR_CLS0 ( 24'hff0000 ),   // 红
        .COLOR_CLS1 ( 24'h00a0ff ),   // 蓝
        .COLOR_CLS2 ( 24'h00ff00 ),   // 绿
        .COLOR_CLS3 ( 24'hfff200 )    // 未分类 → 黄（提示：检测到了但没分出颜色）
    )u_box_draw(
        .I_clk       ( S_hdmi_pixel_clk ),
        .I_rst_n     ( S_hdmi_rst_n     ),
        .I_de        ( S_hdmi_de        ),
        .I_pix_x     ( S_pix_x          ),
        .I_pix_y     ( S_pix_y          ),
        .I_box_valid ( S_color_box_valid ),   // ← conn_features 目标总线（4 槽展开）
        .I_box_cls   ( S_box_cls        ),   // ← 每槽 2bit 颜色类别
        .I_box_xmin  ( S_box_xmin       ),
        .I_box_xmax  ( S_box_xmax       ),
        .I_box_ymin  ( S_box_ymin       ),
        .I_box_ymax  ( S_box_ymax       ),
        .I_cross_en  ( 1'b1             ),   // 十字准星保留（屏心 640,360，便于对位）
        .I_cross_x   ( 11'd640          ),
        .I_cross_y   ( 10'd360          ),
        .O_hit       ( S_box_hit        ),
        .O_color     ( S_box_color      ),
        .O_de        (                  )
    );

    osd_coord u_osd_coord(
        .I_clk        ( S_hdmi_pixel_clk ),
        .I_rst_n      ( S_hdmi_rst_n     ),
        .I_de         ( S_hdmi_de        ),
        .I_pix_x      ( S_pix_x          ),
        .I_pix_y      ( S_pix_y          ),
        .I_x_val      ( 11'd640          ),   // D1 固定显示标定点
        .I_y_val      ( 10'd360          ),
        // ★ 2026-10-14：改接 S_det_fdone（与 S_target_bus 锁存同源）——
        //   冻结表 §1.1 铁律：采样首拍须 ≥ 总线锁存拍 或 早 ≥ 一个完整场消隐期。
        //   新版 pix_coord_gen 的 O_frame_done 在帧首（比锁存早 ≈3 拍），
        //   若仍接 S_frame_done 会落进"中间地带"造成静默串帧。
        .I_frame_done ( S_det_fdone      ),
        .O_hit        ( S_coord_hit      ),
        .O_color      ( S_coord_color    )
    );

    //==================================================================
    //  ★张金艺 D5/D6（方案B §2.3 分类 / §2.4 OSD 文字）
    //    D5：shape_class 从目标总线算「几何形状」（方/圆/三角，填充度判据）
    //    D6：osd_category 在屏幕左上角显示「颜色计数 + 形状计数」两行：
    //          R:1 B:2 G:1     （红/蓝/绿 各几个）
    //          S:1 C:2 T:1     （方/圆/三角 各几个）
    //    ⚠ OSD 只读目标总线（整帧锁存），**不参与数据通路**；
    //      与 box_draw / osd_coord 一样工作在**视频像素域**（S_hdmi_de + S_pix_x/y）。
    //==================================================================
    // 2026-10-24: omit S/C/T classification to relieve FPGA congestion.
    // Keep color counts, target tracking, coordinates and both motion modes.
    assign S_shapes = 8'hff; // Every slot is unknown; no classifier hardware.

    osd_category #(
        // ★ 2026-10-15 下板反馈：原来放 (16,16) 会与**左上角的常驻 Logo** 叠在一起
        //   ⇒ 挪到**屏幕右下角**。计算（字符格 16×16、COLS=11）：
        //     x 起 = 1280 − 11×16 − 16 = 1088；两行 y = 656 / 672（底边留 16 px）
        .X0 ( 1088 ),              // 区域左上角（16 的整数倍 ⇒ 零除法）
        .Y0 ( 656 ),
        .SHOW_SHAPES ( 0 ),         // Preserve R/B/G counts; omit S/C/T row.
        .COLS ( 11 )               // "R:1 B:2 G:1" = 11 个字符格
    )u_osd_category(
        .clk          ( S_hdmi_pixel_clk ),
        .rst_n        ( S_hdmi_rst_n     ),
        .I_de         ( S_hdmi_de        ),
        .I_pix_x      ( S_pix_x          ),
        .I_pix_y      ( S_pix_y          ),
        .I_bus        ( S_count_bus      ),
        .I_shapes     ( S_shapes         ),
        .I_valid_cnt  ( S_track_cnt      ),
        // ★ 2026-10-14：改接 S_det_fdone（保持三处同源，省得再分辨"哪个要改"）。
        //   注：本模块在单时钟沿内组合读完 4 槽（单拍取数）⇒ 本不受"中间地带"影响，
        //   只是让它与总线锁存对齐、结果更直观（滞后 0 帧而非 1 帧）。
        .I_frame_done ( S_det_fdone      ),
        .O_counts     ( S_color_counts   ),
        .O_hit        ( S_cat_hit        ),
        .O_color      ( S_cat_color      )
    );

    //==================================================================
    //  ★张铭晨 D5/D7/D8（2026-10-14 合板接入：两边功能并到一起）
    //    D5 target_assoc  → 跟踪稳定化：从 S_target_bus 产生"稳定的框"
    //        输出**直接驱动 S_box_***（原"总线逐槽展开"那一段已改为接它）
    //    D7/D8 → 区域运动告警、红蓝动静统计及只读显示
    //    ⚠ 三者的 I_frame_done 全部接 S_det_fdone（与总线锁存同源 ⇒ 早 0 拍，
    //      绝不会落进冻结表 §1.1 说的"中间地带"）
    //==================================================================
    target_assoc #(.SERIAL_COST(1), .SCENE_GUARD(1), .ACQUIRE_MIN(16), .CAP_FILTER(1), .MAX_MISS(5), .EMA_SHIFT(1), .MATCH_TH(144), .CLS_PEN(256)) u_target_assoc (
        .I_clk         ( S_hdmi_pixel_clk ),
        .I_rst_n       ( S_hdmi_rst_n     ),
        .I_frame_done  ( S_det_fdone      ),
        .I_scene_busy(S_scene_busy),
        .I_target_cnt  ( S_color_tracking_active ? S_target_cnt : 3'd0 ),
        .I_target_bus  ( S_tracking_bus   ),
        .O_box_valid   ( S_box_valid      ),
        .O_box_xmin    ( S_box_xmin       ),
        .O_box_xmax    ( S_box_xmax       ),
        .O_box_ymin    ( S_box_ymin       ),
        .O_box_ymax    ( S_box_ymax       ),
        .O_box_cls     ( S_box_cls        ),
        .O_box_cx      ( S_trk_cx         ),
        .O_box_cy      ( S_trk_cy         ),
        .O_track_cnt   ( S_track_cnt      )
    );

    // Shared frame difference: fixed-zone alarm plus per-color status.
    // Shared grayscale is delayed two clocks (gamma + gray). The new
    // frontend compensates both. SW4 is synchronized/debounced, then
    // committed at the frame boundary; inactive decisions are cleared.
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            R_alarm_switch_sync<=0; R_alarm_switch_stable<=0;
            R_alarm_switch_debounce<=0; R_color_mode<=0; R_tracking_flush<=3'd7;
        end else begin
            R_alarm_switch_sync<={R_alarm_switch_sync[0],I_sw4};
            if(S_det_fdone) begin
                R_color_mode<=R_alarm_switch_stable;
                if(!R_alarm_switch_stable || R_color_mode!=R_alarm_switch_stable)
                    R_tracking_flush<=3'd7;
                else if(R_tracking_flush!=0) R_tracking_flush<=R_tracking_flush-3'd1;
            end
            if(R_alarm_switch_sync[1]==R_alarm_switch_stable) R_alarm_switch_debounce<=0;
            else if(R_alarm_switch_debounce==20'd749999) begin
                R_alarm_switch_stable<=R_alarm_switch_sync[1];
                R_alarm_switch_debounce<=0;
            end else R_alarm_switch_debounce<=R_alarm_switch_debounce+1'b1;
        end
    end
    motion_event_monitor #(
        .ROI_X0(ALARM_X0), .ROI_X1(ALARM_X1),
        .ROI_Y0(ALARM_Y0), .ROI_Y1(ALARM_Y1)
    )u_motion_event_monitor(
        .I_clk(S_hdmi_pixel_clk), .I_rst_n(S_hdmi_rst_n),
        .I_de(S_hdmi_de), .I_pix_x(S_pix_x), .I_pix_y(S_pix_y),
        .I_gray(S_gray_out), .I_frame_done(S_det_fdone),
        .I_color_mode(R_color_mode), .I_mode_change(S_function_change),
        .I_box_valid(S_color_box_valid), .I_box_cls(S_box_cls),
        .I_box_xmin(S_box_xmin), .I_box_xmax(S_box_xmax),
        .I_box_ymin(S_box_ymin), .I_box_ymax(S_box_ymax),
        .O_scene_busy(S_scene_busy),
        .O_alarm(S_event_alarm), .O_ready(S_event_ready),
        .O_events_bcd(S_event_count),
        .O_roi_bcd(S_roi_activity_bcd),
        .O_red_state(S_red_motion), .O_blue_state(S_blue_motion)
    );
    generate if(FIELD_METER_DIAG) begin:g_field_diag
        color_field_meter u_color_field_meter(
            .clk(S_hdmi_pixel_clk),.rst_n(S_hdmi_rst_n),
            .I_de(S_hdmi_de),.I_frame(S_frame_done),.I_x(S_pix_x),.I_y(S_pix_y),
            .I_linear(S_video_rd_data),.I_display(S_video_sat),
            .O_bcd(S_field_bcd),.O_ready(S_field_ready)
        );
    end else begin:g_no_field_diag
        assign S_field_bcd=360'd0;assign S_field_ready=2'd0;
    end endgenerate
    motion_event_osd #(.DISPLAY_NR_AB(1),
        .ROI_X0(ALARM_X0), .ROI_X1(ALARM_X1),
        .ROI_Y0(ALARM_Y0), .ROI_Y1(ALARM_Y1)
    )u_motion_event_osd(
        .I_clk(S_hdmi_pixel_clk), .I_rst_n(S_hdmi_rst_n),
        .I_de(S_hdmi_de), .I_pix_x(S_pix_x), .I_pix_y(S_pix_y),
        .I_color_mode(R_color_mode), .I_counts(S_color_counts),.I_nr_level(S_raw_nr_level),
        .I_tnr_statistics(S_tnr_statistics_sync),.I_display_nr_active(S_tnr_used_sync[1]),.I_tnr_pages(S_tnr_pages_sync),.I_tnr_fault(S_tnr_fault_sync[1]),.I_scene_busy(S_scene_busy),.I_ae_quality_status(S_ae_quality_status),
        .I_cal_enable(FIELD_METER_DIAG),.I_cal_bcd(S_field_bcd),.I_cal_ready(S_field_ready),
        .I_alarm(S_event_alarm), .I_ready(S_event_ready),
        .I_events_bcd(S_event_count),
        .I_roi_bcd(S_roi_activity_bcd), .I_det_full(S_detection_full),
        .I_red_state(S_red_motion), .I_blue_state(S_blue_motion),
        .O_hit(S_event_hit), .O_color(S_event_color)
    );

    // ===== 叠加仲裁：坐标数字 > 画框 > 检测掩膜 =====
    // ★2026-10-10 合并：S_mask_hit / S_mask_color 已接到 detect_color_mask 输出（见下方例化）。
    // ★连续赋值必须写在模块体内、且在任何模块例化的端口连接列表之外。

    //==================================================================
    // ★张金艺 detect_zjy —— D1 交付：YCbCr 双阈值颜色掩膜
    //   取数点：S_video_sat（与 S_hdmi_de 同源，PIPE_DELAY=0）
    //   坐标  ：接 S_pix_x_2d / S_pix_y_2d（由 u_pix_coord_gen 生成）→ PIX_DLY=2
    //   输出  ：O_mask → S_mask_hit（选色伪彩）
    //   ★D4：I_color_sel 改 2'd3（三色取或），配合 conn_label 做「多色同帧」分类；
    //         同时把对齐后的 O_de / O_pix_x / O_pix_y / O_frame_done 引出给 conn_label。
    //==================================================================
    wire        S_det_mask_r, S_det_mask_g, S_det_mask_b;

    detect_color_mask #(
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720  ),
        .PIPE_DELAY ( 0    ),   // S_video_sat 与 S_hdmi_de 同拍
        .PIX_DLY    ( 2    ),   // 接 S_pix_x_2d / S_pix_y_2d
        // ★ 2026-10-11 v2：曝光自适应色相判据（修「低曝光检测不全 / 掩膜时有时无」）
        //   根因：目标变暗 ⇒ 色差按比例缩小 ⇒ 逃出固定绝对窗口（绿块 60% 亮度就崩）。
        //   处置：改为以 (128,128) 为原点的「方向 + 幅度」判据，对整体亮度缩放免疫。
        //   实测：35% 亮度下红/蓝/绿全部 1.00 检出（原来 0.14/0.11/0.00），
        //         干扰色误判率 5.7% → 1.1%。详见 detect_color_mask.v 参数区注释。
        .MAG_TH     ( `MAG_TH  ),
        .RATIO10    ( `RATIO10 ),
        .G_AXIS_TH  ( `G_AXIS_TH ),
        .ADAPT_GREEN_AXIS ( 1 ), // night bands must relax green axis floors as well as total chroma
        // ★ 2026-10-12 血泪教训：这三行原来写的是字面量 `.MAG_TH(8'd16)`
        //   —— 只改了 detect_color_mask.v 的 parameter 默认值，被这里
        //      显式覆盖回 16，**新门限根本没生效**（掩膜仍满屏噪声 →
        //      conn_label 每行游程爆表 → 超出 NEG=8 → 真目标被挤掉 → 无框）。
        //   现改为引 common_defs.vh 的宏，与模块默认值**同源**，不会再分叉。
        //   改门限只需改 common_defs.vh 一处（§5b）。
        // ★ 2026-10-15 夜间（低照度）自适应门限 —— 让暗场目标不再"够不着门限"：
        //   按 ae_meter 的**帧平均亮度**分 4 档下调 MAG_TH（24/20/16/12，带 ±6 滞回）。
        //   亮度 ≥112（AE 跟得上）时输出仍 = `MAG_TH`，行为与改造前一致。
        //   ⚠ 设 NIGHT_EN(0) 可一键退回"固定门限"行为（无需改 detect_color_mask.v）。
        .NIGHT_EN   ( 1     ),          // 1 = 启用夜间自适应；0 = 固定门限（同改造前）
        .MAG_B0     ( `MAG_TH ),        // 档 0 门限必须与 §5b 的 `MAG_TH` 同源
        .USE_ABS_WINDOW ( 0   )
    )u_detect_color_mask(
        .clk        ( S_hdmi_pixel_clk ),
        .rst_n      ( S_hdmi_rst_n     ),
        .I_de       ( S_hdmi_de        ),
        .I_vsync    ( S_hdmi_vsync     ),
        .I_hsync    ( S_hdmi_hsync     ),
        .I_user     ( S_hdmi_user      ),
        .I_last     ( S_hdmi_last      ),
        .I_rgb      ( S_video_sat      ),
        .I_pix_x    ( S_pix_x_2d       ),
        .I_pix_y    ( S_pix_y_2d       ),
        .I_color_sel( `COLOR_SEL_OR    ),   // ★D4：2'd3 = 三色取或（多色同帧）
        // ★ 2026-10-15 夜间自适应输入：直接取 ae_meter 的两路"旁路取样"信号，
        //   不动 AE 闭环、不动任何像素数据 ⇒ 对显示与曝光零影响。
        .I_y_avg        ( S_ae_y_avg   ),   // 帧平均亮度（更新后保持整帧）
        .I_frame_toggle ( S_ae_frame_tog ), // 每帧翻转一次（作为采样脉冲）
        .O_mask     ( S_mask_hit       ),
        .O_mask_r   ( S_det_mask_r     ),
        .O_mask_g   ( S_det_mask_g     ),
        .O_mask_b   ( S_det_mask_b     ),
        .O_y        (                  ),
        .O_cb       (                  ),
        .O_cr       (                  ),
        .O_de       ( S_det_de         ),   // ★D4：对齐后的 de / 坐标 / 帧末引出给 conn_label
        .O_vsync    (                  ),
        .O_hsync    (                  ),
        .O_user     (                  ),
        .O_last     (                  ),
        .O_pix_x    ( S_det_px         ),
        .O_pix_y    ( S_det_py         ),
        .O_frame_done( S_det_fdone     )
    );

    //==================================================================
    // ★ 2026-10-12：掩膜水平去斑（mask_despeckle）
    //   为什么需要它（现场三轮排查得出的根因）：
    //     conn_label 的等价表深度 **NEG=8** 规定「同一帧最多 8 个连通域，先到先得」，
    //     而**传感器噪点每个孤立点自成一个连通域**（σ=8 时实测 4.6 万个）
    //     ⇒ 靠调高判据门限压噪声会把目标一起压掉（MAG_TH=48 时只有物体边缘能检出）；
    //       靠调低门限则噪声占满 NEG 名额，真目标被挤掉。
    //     去斑利用「噪声是孤立散点、目标是成片区域」把散点清掉：
    //       实测 σ=8 时噪声连通域 11253 → 0，而实心目标 100% 保留。
    //   ⚠ 注意：显示叠加（S_ovl_hit）也取**去斑后**的掩膜 S_dspk_mask ——
    //     它就是真正喂给 conn_label 的那一路。看到什么，连通域就吃到什么，
    //     是判断「框为什么出不来」最直接的证据（代价：绿点整体右移约 7 像素）。
    //==================================================================
    assign S_det_color = S_det_mask_r ? `CLS_RED   :
                         S_det_mask_b ? `CLS_BLUE  :
                         S_det_mask_g ? `CLS_GREEN : `CLS_UNKNOWN;

    mask_despeckle #(
        .HALF       ( 4 ),      // 水平半窗（左右各 4 ⇒ 9 像素窗）
        .KMIN       ( 5 ),      // 水平窗内至少 5 个命中
        .VER_ROWS   ( 3 ),      // 垂直投票看 3 行
        .VER_NEED   ( 2 ),      // 至少 2 行通过才保留
        .DCH        ( 2 ),      // 公共输出延迟链深度
        .ENABLE     ( 1 ),      // 1 = 启用去斑；0 = 旁路（仅延迟）
        .COLOR_CLEAN ( 1 )      // classify filled holes; remove UNKNOWN islands before CCL
    )u_mask_despeckle(
        .clk            ( S_hdmi_pixel_clk ),
        .rst_n          ( S_hdmi_rst_n     ),
        .I_de           ( S_det_de         ),
        .I_mask         ( S_mask_hit       ),
        .I_mask_r       ( S_det_mask_r     ),
        .I_mask_g       ( S_det_mask_g     ),
        .I_mask_b       ( S_det_mask_b     ),
        .I_frame_done   ( S_det_fdone      ),
        .I_pix_x        ( S_det_px         ),
        .I_pix_y        ( S_det_py         ),
        .O_de           ( S_dspk_de        ),
        .O_mask         ( S_dspk_mask      ),
        .O_mask_r       ( S_dspk_mr        ),
        .O_mask_g       ( S_dspk_mg        ),
        .O_mask_b       ( S_dspk_mb        ),
        .O_frame_done   ( S_dspk_fdone     ),
        .O_pix_x        ( S_dspk_px        ),
        .O_pix_y        ( S_dspk_py        )
    );

    //  去斑后的逐像素颜色（与去斑后的掩膜同像素）
    assign S_dspk_color = S_dspk_mr ? `CLS_RED   :
                          S_dspk_mb ? `CLS_BLUE  :
                          S_dspk_mg ? `CLS_GREEN : `CLS_UNKNOWN;

    conn_label #(
        .IMG_WIDTH  ( `IMG_WIDTH  ),
        .IMG_HEIGHT ( `IMG_HEIGHT ),
        .MIN_AREA   ( `MIN_AREA    ),    // 小面积噪声门限
        .CONTINUE_SHORT ( 1 ),           // preserve narrow connected edges; seed threshold stays 8
        .COLOR_SEPARATE ( 1 ),           // touching red/blue/green objects remain distinct
        .RESERVE_COLORS ( 1 ),           // reserve labels for colors arriving later in raster order
        .RECLAIM_MIN_AREA ( 16 ), .LMIN(5), .EMIT_COMPLETED(1),
        .OUT_SPACING ( 64 ), .CCL_CAPACITY ( 10 ), .LINK_GAP(3),
        .RECLAIM_SMALL ( 1 )             // free expired tiny/flat leaf regions during line blanking
    )u_conn_label(
        .clk            ( S_hdmi_pixel_clk ),
        .rst_n          ( S_hdmi_rst_n     ),
        .I_de           ( S_dspk_de        ),   // ★ 用去斑后的一路
        .I_mask         ( S_dspk_mask      ),
        .I_color        ( S_dspk_color     ),
        .I_pix_x        ( S_dspk_px        ),
        .I_pix_y        ( S_dspk_py        ),
        .I_frame_done   ( S_dspk_fdone     ),
        .O_label        ( S_lbl            ),
        .O_label_valid  ( S_lbl_v          ),
        .O_label_de     ( S_lbl_de         ),
        .O_label_pix_x  ( S_lbl_x          ),
        .O_label_pix_y  ( S_lbl_y          ),
        .O_cmp_valid    ( S_cmp_valid      ),
        .O_cmp_label    ( S_cmp_label      ),
        .O_cmp_cls      ( S_cmp_cls        ),
        .O_cmp_area     ( S_cmp_area       ),
        .O_cmp_xmin     ( S_cmp_xmin       ),
        .O_cmp_xmax     ( S_cmp_xmax       ),
        .O_cmp_cx       ( S_cmp_cx         ),
        .O_cmp_ymin     ( S_cmp_ymin       ),
        .O_cmp_ymax     ( S_cmp_ymax       ),
        .O_cmp_cy       ( S_cmp_cy         ),
        .O_cmp_cnt      ( S_cmp_cnt        ),
        .O_capacity_full ( S_detection_full )
    );

    conn_features #(
        .SLOT_N ( `TARGET_MAX ),        // 4
        .GROUP_CAPS ( 1 ), .GROUP_MIN_AREA ( 16 ), .GROUP_GAP ( 16 ), .CAP_GEOMETRY(1),
        .FAIR_COLORS ( 1 )              // retain each color's best component before duplicates
    )u_conn_features(
        .clk               ( S_hdmi_pixel_clk ),
        .rst_n             ( S_hdmi_rst_n     ),
        .I_frame_done      ( S_det_fdone      ),
        .I_cmp_valid       ( S_cmp_valid      ),
        .I_cmp_label       ( S_cmp_label      ),
        .I_cmp_cls         ( S_cmp_cls        ),
        .I_cmp_area        ( S_cmp_area       ),
        .I_cmp_xmin        ( S_cmp_xmin       ),
        .I_cmp_xmax        ( S_cmp_xmax       ),
        .I_cmp_ymin        ( S_cmp_ymin       ),
        .I_cmp_ymax        ( S_cmp_ymax       ),
        .I_cmp_cx          ( S_cmp_cx         ),
        .I_cmp_cy          ( S_cmp_cy         ),
        .O_target_bus      ( S_target_bus     ),
        .O_target_valid_cnt( S_target_cnt     ),
        .O_update          ( S_target_update  )
    );

    //---- target_bus(4×96) → box_draw 的 4 框输入 ----
    //   ★ 2026-10-14 合板变更：这里**不再**直接逐槽展开总线，而是改由
    //     **`u_target_assoc`（张铭晨 D5 目标关联）**输出驱动 S_box_* ——
    //     它做了 EMA 平滑 + 漏检保持 + 删框，把"逐帧跳动的清单"变成"稳定的框"。
    //     原逐槽展开（generate + 位域切片）已删除，避免两处同时驱动同一组 wire。
    //     若需临时回退到"直通"（不跟踪），把上面的 target_assoc 例化换成
    //     直通赋值即可（`.O_box_*` → 总线位域），或在 target_assoc 内设旁路。
    //   单槽位域（common_defs.vh §3）：valid[0] cls[2:1] cx[13:3] cy[23:14]
    //                                  xmin[34:24] xmax[45:35] ymin[55:46] ymax[65:56]

    //------------------------------------------------------------------
    //  ★ 2026-10-12：掩膜叠加显示开关
    //    现场反馈：检测掩膜以绿色小点铺满画面（人/墙/帘子上全是）且逐帧乱跳。
    //    ★ v7.9（2026-10-13）：改回 **0（关闭）** —— 用户从一开始就要求
    //      "不要把检测到的物体用绿色的点标记出来"。绿点本就只是调试手段。
    //    1 = 显示（**调试用**）。它显示的是**去斑后**的掩膜 S_dspk_mask
    //               （= 真正喂给 conn_label 的那一路）⇒ **屏幕上每个绿点就是一条
    //                 会占用 conn_label 名额的连通域**，是排查"框为什么出不来"最直接的证据。
    //    0 = 不显示（★ 当前默认）
    //    ⚠ 关掉的只是**显示**：检测 → 去斑 → 连通域 → target_bus → 画框 整条链路不受影响。
    //      需要现场看掩膜形态时，把这里改回 1、重新综合即可。
    //------------------------------------------------------------------
    localparam MASK_OVERLAY_EN = 0;

    assign S_mask_color = 24'h00_FF_00;   // 掩膜颜色（绿）
    //    优先级（高→低）：坐标 OSD > 跟踪框 > 类别 OSD > 区域与状态提示 > 掩膜
    //    hdmi_mixer 不需要改 —— 所有人共用 S_ovl_hit / S_ovl_color 这一条通道。
    // Select one feature overlay; shared video/AE processing remains continuous.
    // Inactive histories are cleared; the tracker consumes empty target lists.
    assign S_ovl_hit = S_event_hit | (R_color_mode &&
        (S_box_hit | S_coord_hit | S_cat_hit | (MASK_OVERLAY_EN && S_dspk_mask)));
    assign S_ovl_color = (R_color_mode && S_coord_hit) ? S_coord_color :
                         (R_color_mode && S_box_hit) ? S_box_color :
                         (R_color_mode && S_cat_hit) ? S_cat_color :
                         S_event_hit ? S_event_color : S_mask_color;

    hdmi_mixer #(
        .H_OFFSET   ( 0    ),
        .V_OFFSET   ( 0    ),
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720  ),
        .DEBUG_MODE ( 0    ),
        // ★2026-10-08 中文 OSD：常驻显示「帧数 / AE 增益 / 当前模式」。
        //   比赛硬性要求：切换模式必须在屏幕上显示当前模式名。
        //   若只想留安路 Logo、不叠加任何信息，改 1'b0 即可。
        .OSD_INFO_EN( 1'b1 )
    )u_hdmi_mixer(
        .I_clk           ( S_hdmi_pixel_clk   ),
        .I_rst_n         ( S_hdmi_rst_n       ),
        .I_video_vsync   ( S_hdmi_vsync       ),
        .I_video_hsync   ( S_hdmi_hsync       ),
        .I_video_de      ( S_hdmi_de          ),
        .I_video_user    ( S_hdmi_user        ),
        .I_video_last    ( S_hdmi_last        ),
        .I_debug_status  ( S_hdmi_debug_status),
        // ★当前显示模式（拨码 SW1/SW2）。S_mode_sel 已在像素时钟域两级同步，
        //   与 hdmi_mixer 同域，直接连线即可，无需再加同步器。
        .I_mode          ( S_mode_sel         ),
        .I_ae_y          ( S_ae_y_avg         ),   // 预留：帧平均亮度（当前不显示）
        .I_ae_ag         ( S_ag               ),   // AE 增益控制字（OSD 第 2 行）
        // ★叠加仲裁输出（画框 / 准星 / 坐标 OSD / 检测掩膜），优先级低于 Logo 与文字 OSD
        .I_ext_hit       ( S_ovl_hit          ),
        .I_ext_color     ( S_ovl_color        ),
        .O_video_rd_en   ( S_hdmi_window_rd_en),
        .I_video_rd_data ( S_proc_rgb_out  ),
        .O_hdmi_vsync    ( S_hdmi_out_vsync   ),
        .O_hdmi_hsync    ( S_hdmi_out_hsync   ),
        .O_hdmi_de       ( S_hdmi_out_de      ),
        .O_hdmi_data     ( S_hdmi_out_data    )
    );

    hdmi_tx u_hdmi_tx(
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        .I_rst              ( ~S_hdmi_rst_n     ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_hdmi_out_vsync  ),
        .I_video_in_de      ( S_hdmi_out_de     ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( S_hdmi_out_data   ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p      ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p      ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p      ),
        .O_tmds_clk_p       ( O_tmds_clk_p      )
    );


    
endmodule
