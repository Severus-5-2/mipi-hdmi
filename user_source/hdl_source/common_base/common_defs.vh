//=============================================================================
//  common_defs.vh  —  两人共用的全局常量、位宽与接口宏定义
//=============================================================================
//  项目  : MIPI-HDMI 实时视频图像处理系统（方案B）
//  归属  : user_source/hdl_source/common_base/   ★ 两人共同维护，唯一一份
//  依据  : doc/接口冻结表.md v1.0
//
//  ⚠ 本文件是本项目的「接口宪法」。修改任何一处都必须：
//     ① 双方同意  ② 更新 doc/接口冻结表.md 版本号  ③ 重跑位级对拍
//
//  用法（在 .v 文件顶部）：
//      `include "common_defs.vh"
//  并在 TD 工程里把本文件所在目录加入 include 搜索路径。
//=============================================================================

`ifndef COMMON_DEFS_VH
`define COMMON_DEFS_VH

//-----------------------------------------------------------------------------
//  1. 视频时序常量（必须与 uivtc / vtc 配置严格一致）
//-----------------------------------------------------------------------------
`define IMG_WIDTH        1280        // 一行有效像素数
`define IMG_HEIGHT       720         // 一帧有效行数
`define PIX_X_W          11          // 列坐标位宽：0 ~ 1279 需 11 bit
`define PIX_Y_W          10          // 行坐标位宽：0 ~  719 需 10 bit

`define PIX_X_MAX        11'd1279
`define PIX_Y_MAX        10'd719

//-----------------------------------------------------------------------------
//  2. YCbCr 转换系数（位级复用 chroma_denoise.v，方案B §1.1 强制）
//-----------------------------------------------------------------------------
//     Y  =  77R + 150G +  29B            → (x + 128) >>> 8
//     Cb = -43R -  85G + 128B  + 128     → ((x + 128) >>> 8) + 128
//     Cr = 128R - 107G -  21B  + 128     → ((x + 128) >>> 8) + 128
//
//  ⚠ 这两个数不能改：+128 是四舍五入，>>>8 是算术右移。少任何一个都会有
//     1 LSB 系统性偏移，导致掩膜边缘与 D1 验证结果不一致。
//-----------------------------------------------------------------------------
`define YCBCR_KR         18'sd77
`define YCBCR_KG         18'sd150
`define YCBCR_KB         18'sd29

`define YCBCR_CB_KR     -18'sd43
`define YCBCR_CB_KG     -18'sd85
`define YCBCR_CB_KB      18'sd128

`define YCBCR_CR_KR      18'sd128
`define YCBCR_CR_KG     -18'sd107
`define YCBCR_CR_KB     -18'sd21

`define YCBCR_ROUND      18'sd128     // 四舍五入偏置
`define YCBCR_SHIFT      8            // 算术右移位数
`define YCBCR_BIAS       10'sd128     // Cb/Cr 零偏置

//-----------------------------------------------------------------------------
//  3. 目标列表总线（§4：艺 → 晨）
//-----------------------------------------------------------------------------
`define TARGET_MAX       4           // 最多 4 个目标
`define TARGET_CNT_W     3           // 有效目标计数位宽

`define TGT_VALID_W      1
`define TGT_CLS_W        2
`define TGT_CX_W         11
`define TGT_CY_W         10
`define TGT_XMIN_W       11
`define TGT_XMAX_W       11
`define TGT_YMIN_W       10
`define TGT_YMAX_W       10
`define TGT_AREA_W       20          // 1280×720 = 921600 → 20 bit

//  单槽实际字段合计 = 1+2+11+10+11+11+10+10+20 = 86 bit
//  为便于跨模块传递与后续扩展（fill_ratio / aspect），对齐到 96 bit
`define TGT_SLOT_W       96
`define TGT_BUS_W        (`TGT_SLOT_W * `TARGET_MAX)   // 384 bit

// ---- 单槽字段位偏移（打包用 {高,...,低} 风格）----
//   [95:86] 保留  (10 bit)
//   [85:66] area  (20 bit)
//   [65:56] ymax  (10 bit)
//   [55:46] ymin  (10 bit)
//   [45:35] xmax  (11 bit)
//   [34:24] xmin  (11 bit)
//   [23:14] cy    (10 bit)
//   [13:3]  cx    (11 bit)
//   [2:1]   cls   ( 2 bit)
//   [0]     valid ( 1 bit)
`define TGT_O_VALID      0
`define TGT_O_CLS        1
`define TGT_O_CX         3
`define TGT_O_CY         14
`define TGT_O_XMIN       24
`define TGT_O_XMAX       35
`define TGT_O_YMIN       46
`define TGT_O_YMAX       56
`define TGT_O_AREA       66

//-----------------------------------------------------------------------------
//  4. 分类编号（cls 字段）
//-----------------------------------------------------------------------------
`define CLS_RED          2'd0
`define CLS_BLUE         2'd1
`define CLS_GREEN        2'd2
`define CLS_UNKNOWN      2'd3

//-----------------------------------------------------------------------------
//  5. 连通域 label 常量（§4：晨 → 郭）
//-----------------------------------------------------------------------------
`define LABEL_MAX        32          // 等价表深度 32（方案B §2.2）
`define LABEL_W          6           // 0 ~ 32 需 6 bit
`define LABEL_NONE       6'd0        // ★ 0 = 无归属，有效 label 从 1 开始
`define LABEL_FIRST      6'd1

//-----------------------------------------------------------------------------
//  6. 颜色选择（拨码 / 按键）
//-----------------------------------------------------------------------------
`define COLOR_SEL_RED    2'd0
`define COLOR_SEL_BLUE   2'd1
`define COLOR_SEL_GREEN  2'd2
`define COLOR_SEL_OR     2'd3        // 三色取或

//-----------------------------------------------------------------------------
//  7. 画框常量（§4：晨 → 郭）
//-----------------------------------------------------------------------------
`define BOX_LINE_W       2           // 框线宽 2 px
`define BOX_COLOR_RED    24'hFF0000
`define BOX_COLOR_BLUE   24'h0000FF
`define BOX_COLOR_GREEN  24'h00FF00

//-----------------------------------------------------------------------------
//  8. 运动检测降采样（方案B §3.2 推荐）
//-----------------------------------------------------------------------------
`define MOTION_DS        4           // 降采样倍数 → 320×180
`define MOTION_W         320
`define MOTION_H         180

`endif // COMMON_DEFS_VH
