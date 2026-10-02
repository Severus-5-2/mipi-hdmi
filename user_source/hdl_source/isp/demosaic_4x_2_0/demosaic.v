/*****************************************************************
Company : MiLianKe Electronic Technology Co., Ltd.
WebSite:https://www.milianke.com
TechWeb:https://www.uisrc.com
tmall-shop:https://milianke.tmall.com
jd-shop:https://milianke.jd.com
taobao-shop: https://milianke.taobao.com
Description: 
The reference demo provided by Milianke is only used for learning. 
We cannot ensure that the demo itself is free of bugs, so users 
should be responsible for the technical problems and consequences
caused by the use of their own products.
@Author      :   XiaoQingquan 
@Time        :   2025/01 
version:     :   1.1
@Description :   ��ģ����Ҫ������3x3������γɣ��Լ���ͼ��߽����صĴ���
                 ��ԭ�������Ż���ʱ�򣬰�ԭ����һЩδ��ȫ������źŽ��ж��롣
*****************************************************************/

module demosaic #(
    parameter IMG_HEIGHT          = 1080,  // ͼ��߶�
    parameter IMG_WIDTH           = 1920,   // ͼ�����
    parameter data_complete_delay = 50  ,
    parameter BAYER_MODE          = "BGGR"
)
(
    input           I_clk   ,   
    input           I_rst_n ,   

    input [39:0]            axi4s_video_tdata ,  
    input [9:0]             axi4s_video_tdest ,
    input                   axi4s_video_tlast ,  
    input                   axi4s_video_tvalid,  
    input                   axi4s_video_tuser ,  
    output                  axi4s_video_tready,  

    output                  O_tlast  ,  // ����н����ź�
    output                  O_tuser  ,  // ���֡��ʼ�ź�
    output [127:0]          O_tdata  ,  // �������
    output                  O_tvalid ,  // ���������Ч�ź�
    input                   O_tready    // �������׼�����ź�
);


wire demosaic_tlast ; 
wire demosaic_tuser ; 
wire demosaic_tvalid; 
wire demosaic_tready; 

wire        bayer_ypos      ;
wire [95:0] matrix_last_line;
wire [95:0] matrix_cur_line ;
wire [95:0] matrix_next_line;

raw_matrix_3x3_buffer #(
    .IMG_HEIGHT         (IMG_HEIGHT         ), 
    .IMG_WIDTH          (IMG_WIDTH          ), 
    .data_complete_delay(data_complete_delay)
)raw_matrix_3x3_buffer_d
(
    /*input           */.I_clk    (I_clk   ),   // ʱ���ź�
    /*input           */.I_rst_n  (I_rst_n ),   // ��λ�źţ�����Ч

   /*input [39:0]    */.axi4s_video_tdata (axi4s_video_tdata ),  // AXI4-Stream��Ƶ����
   /*input [9:0]     */.axi4s_video_tdest (axi4s_video_tdest ),
   /*input           */.axi4s_video_tlast (axi4s_video_tlast ),  // �н����ź�
   /*input           */.axi4s_video_tvalid(axi4s_video_tvalid),  // ������Ч�ź�
   /*input           */.axi4s_video_tuser (axi4s_video_tuser ),  // ֡��ʼ�ź�
   /*output          */.axi4s_video_tready(axi4s_video_tready),  // ��ģ��׼���ý�������

   /*output          */.O_tlast         (demosaic_tlast ),   // �н����ź����
   /*output          */.O_tuser         (demosaic_tuser ),   // ֡��ʼ�ź����
   /*output          */.O_tvalid        (demosaic_tvalid),   // ������Ч�ź����
   /*input           */.O_tready        (demosaic_tready),   // ��һ��ģ��׼���ý�������

   /*output          */.bayer_ypos      (bayer_ypos      ),   // Bayer�˲�����λ��
   /*output  [95:0]  */.matrix_last_line(matrix_last_line),   // ��һ��3x3��������
   /*output  [95:0]  */.matrix_cur_line (matrix_cur_line ),   // ��ǰ��3x3��������
   /*output  [95:0]  */.matrix_next_line(matrix_next_line)    // ��һ��3x3��������
);

bilinear_interpolation #(
    .BAYER_MODE (BAYER_MODE)
)bilinear_interpolation_d(
    /*input            */.I_clk            (I_clk   ),
    /*input            */.I_rst_n          (I_rst_n ),

    //input
    /*input            */.I_tlast      (demosaic_tlast )    ,
    /*input            */.I_tuser      (demosaic_tuser )    ,
    /*input            */.I_tvalid     (demosaic_tvalid)    ,
    /*output           */.I_tready     (demosaic_tready)    ,
  
    /*input            */.bayer_ypos       (bayer_ypos       ),
    /*input   [95:0]   */.matrix_last_line (matrix_last_line ),
    /*input   [95:0]   */.matrix_cur_line  (matrix_cur_line  ),
    /*input   [95:0]   */.matrix_next_line (matrix_next_line ),
  
    //output
    /*output           */.O_tlast (O_tlast ),
    /*output           */.O_tuser (O_tuser ),
    /*output [127 : 0] */.O_tdata (O_tdata ),
    /*output           */.O_tvalid(O_tvalid),
    /*input            */.O_tready(O_tready)

);



endmodule