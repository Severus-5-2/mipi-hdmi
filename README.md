# MIPI-HDMI 实时视频图像处理系统

基于安路 PH1P35 FPGA 的实时视频图像处理系统：MIPI 摄像头采集 → ISP 图像处理 → DDR 帧缓存 → HDMI 显示，支持 4 种图像处理模式通过拨码开关实时切换。

## 硬件平台

- FPGA：安路 PH1P35（PH1P35MDG324）
- 开发板：康芯 HX1P35A
- 摄像头：SC520CS（RAW10，2-Lane MIPI，828Mbps/Lane）
- 显示：HDMI 1280×720@60

## 功能特性

拨码开关 SW1/SW2 实时切换 4 种模式（拨上=0，拨下=1）：

| 模式 | SW2 | SW1 | 功能 |
|:---:|:---:|:---:|:---|
| 00 | 上 | 上 | 原图（RGB 彩色） |
| 01 | 上 | 下 | 灰度化 |
| 10 | 下 | 上 | 二值化 |
| 11 | 下 | 下 | Sobel 边缘检测 |

> 开关物理位置：SW1 为左数第 4 位，SW2 为左数第 3 位（板上顺序 4,3,2,1,5,6）。

## 系统架构

MIPI 摄像头 → CSI/RAW10 解包 → ISP (去马赛克 / 白平衡) → DDR 帧缓存 → 视频输出 → 图像处理 → HDMI 混合 → HDMI 显示


## 工程结构

├── constraints_source/     # 引脚与时序约束
│   ├── pin.adc
│   └── timing.sdc
├── hdl_source/             # Verilog 源代码
│   ├── design_top_wrapper.v   # 顶层模块
│   ├── rgb2gray.v             # 灰度化
│   ├── binarize.v             # 二值化
│   ├── sobel_3x3.v            # Sobel 边缘检测
│   ├── pixel_mux_4to1.v       # 四路像素选择器
│   └── ...
├── ip_source/              # IP 核（PLL / FIFO / ERAM /divider）
└── doc/                    # 设计文档与报告


## 关键算法

- **灰度化**：Gray = (R×77 + G×150 + B×29) >> 8
- **二值化**：固定阈值判定（THRESHOLD 参数可配）
- **Sobel**：3×3 窗口梯度，幅度 |Gx|+|Gy| 超过 EDGE_TH 判为边缘

## 开发工具

- 安路 TangDynasty (TD)


