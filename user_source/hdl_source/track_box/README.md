# track_box —— 张铭晨 D1 交付包（进阶2 目标跟踪与信息标注）

> 归属：**张铭晨**（开发方案B 第 3 章，负责 进阶2 跟踪 + 进阶4 运动检测）
> 对应里程碑：**D1 —— 画框模块 + OSD 坐标显示（先画固定框，验证坐标系统）**
> 全部为**新增文件**，未改动工程里任何现有代码（没碰 `design_top_wrapper.v`、`hdmi_mixer.v`、`.al`、`.sdc`）。

---

## 一、这份包里有什么

```
user_source/hdl_source/track_box/
├── README.md                      ← 本文件
├── rtl/
│   ├── pix_coord_gen.v            ★ 全画面唯一像素坐标源（方案B 1.3 节要求）
│   ├── box_draw.v                 ★ 画框 + 十字准星（4 目标，2px 边，含固定框自测）
│   ├── osd_coord.v                ★ 坐标数字 OSD（"X:0640 / Y:0360"）
│   └── track_glyph_rom.v          16x16 字模（0-9 / ':' / X / Y），脚本生成
├── tb/
│   ├── tb_box_draw.v              自检 testbench（已用 iverilog 12.0 实跑通过）
│   ├── tb_osd_coord.v             自检 testbench（已实跑，支持 +xv/+yv 换值）
│   └── tb_track_system.v          ★ 系统级 testbench（真实 720p60 场时序、整帧）
└── sim/
    ├── vtc_coord_probe.py         坐标系统位级审计（D1 的核心：验证坐标）
    ├── glyph_rom_tool.py          字模生成 + 从 RTL 原文渲染校验
    ├── bit_sim_box_draw.py        box_draw 第三方独立实现 + ASCII 预览
    ├── bit_sim_osd_coord.py       坐标 OSD 端到端核对（从 RTL 解析字模拼字）
    └── bit_sim_system.py          ★ 系统级第三方核对 + 整帧 PNG 出图
```

---

## 二、D1 做了什么

| 方案B 要求（3.3 节 D1） | 本包对应 |
|:---|:---|
| 画框模块 | `box_draw.v` —— 4 目标、2px 边框、十字准星、两级流水与像素数据对齐 |
| OSD 坐标显示 | `osd_coord.v` —— 屏幕左下角两行 `X:xxxx` / `Y:yyyy` |
| **验证坐标系统** | `vtc_coord_probe.py` 位级审计 + `pix_coord_gen.v` 新写一版无偏移计数器 |
| 先画固定框 | `box_draw.v` 的 `SELF_TEST=1`：4 个写死坐标的框，**框 B3 的框心正好是屏幕中心 (640,360)**，与 OSD 读数 `X:0640 Y:0360` 互相印证 |

---

## 三、验证结果（都是真跑出来的，不是"应该没问题"）

### 1. 坐标系统位级审计 —— `sim/vtc_coord_probe.py`

逐周期逐位重演 `uivtc.v` 的 hcnt/vcnt/全部流水寄存器，再喂给 `hdmi_mixer`
第 327~345 行的坐标计数器。3 帧 / 3712500 拍，结果：

| 项 | 结果 |
|:---|:---|
| `O_vtc_last` 与 `O_vtc_de` 是否同拍 | **PASS** —— last 恰为每行末像素一拍（2160 = 3 帧 x 720 行），不同拍 0 次 |
| `O_vtc_user` 与 `O_vtc_de` 是否同拍 | **PASS** —— 每帧一拍（第 1 帧无 user，属正常，共 F-1 个） |
| 第 r 行第 k 像素处 (S_x,S_y) ≡ (k,r) | **PASS**（r≥1 零偏移，1840640 个像素 0 错） |
| OSD 四处区域常量可达性 | **PASS**（Logo 25600 / 帧数 2304 / 增益 1536 / 模式 1280，全部命中） |

### 2. `tb_box_draw.v`（iverilog 12.0 实跑）

```
 last 与 de: 同拍 96 次 / 不同拍 0 次  [PASS]
 坐标检查  : 6144 拍, 错 0  [PASS]
 叠加检查  : 6144 拍, 错 0  [PASS]
 frame_done 脉冲数 : 2（期望 2）
 全部 PASS
```

### 3. `sim/bit_sim_box_draw.py check`（TB dump vs 第三方独立模型）

```
 逐像素比对 : 2048 个像素, 不一致 0   [PASS]
 框0 (4,4)-(20,14):   全画面命中 96  / 解析式 96  / 越界 0  [PASS]
 框1 (40,4)-(63,14):  全画面命中 124 / 解析式 124 / 越界 0  [PASS]
 框2 (4,20)-(20,31):  全画面命中 100 / 解析式 100 / 越界 0  [PASS]
 框3 (28,12)-(44,24): 全画面命中 104 / 解析式 104 / 越界 0  [PASS]
 三份实现（RTL / TB 模型 / 本脚本）结果完全一致。
```

### 4. `sim/bit_sim_osd_coord.py`（端到端：数值 → 屏幕字形）

| 数值 | 逐像素一致 |
|:---|:---|
| X=640 / Y=360 | **PASS**（315 像素全中，屏幕确实渲染出 `X:0640` / `Y:0360`）|
| X=1279 / Y=719 | **PASS**（最大值路径）|
| X=0 / Y=0 | **PASS**（前导零路径）|

### 5. 系统级仿真 —— `tb/tb_track_system.v` + `sim/bit_sim_system.py`（2026-10-10 补）

前四项验证的是「模块内部逻辑」，用的是 32x32 / 64x32 的小画面。上板观感由
**整机链路 + 真实场时序**决定，所以补了这一级仿真：

- **真实 1280x720@60 场时序**：`H 1280/1650`（sync 1390~1430）、
  `V 720/750`（vsync 高脉冲 vcnt 726~730）—— 与 `design_top_wrapper.v` 里
  `u_hdmi_vtc` 的参数一字不差；`de / last / vsync` 相位按 `uivtc.v` 的流水
  （延 1 / 延 2 拍）复现。
- **串联链路**：`pix_coord_gen → box_draw + osd_coord`，叠加优先级与顶层一致
  （box > osd），输出整帧位图 `sim/tb_system_frame.bin`（1280x720 颜色码）。
- **第三方独立重算**：`sim/bit_sim_system.py` 用「几何 + 从 RTL 原文解析的
  字模」从零重算整帧，与 RTL dump **逐像素**比对，并出 PNG 供肉眼复核。

实测（5 帧，统计第 2~5 帧）：

```
 坐标检查 : 3686405 拍, 错 0   [PASS]
 每帧有效像素 : 921600 (期望 921600)   [PASS]
 跨帧一致性   : [PASS]  (红/绿/蓝/黄/白/OSD 命中数跨帧完全相同)
 逐像素比对 : 921600 个像素, 不一致 0   [PASS]
 分色：背景914985 / 红1528 / 绿1524 / 蓝1528 / 黄1592 / 白128 / OSD315  全 [PASS]
```

出图（`sim/*.png`，已肉眼复核）：`system_full.png` 整帧 + 四张局部放大。
画面 = 左上红框、右上绿框（右边贴屏 x=1279）、中央黄框、白色十字准星落框心、
左下 `X:0640` / `Y:0360`，与设计一致。

---

## 四、⚠️ 这次挖出来的 3 个真 bug（都在本包内已修）

### bug 1｜边框判定漏范围约束（最隐蔽）

**错**：`(in_y && (x <= x0+1 || x >= x1-1)) || (in_x && (y <= y0+1 || y >= y1-1))`
—— 竖边子句少了 `x >= x0`、横边子句少了 `y >= y0`，于是**框上方与左侧整片**被点亮，
1280x720 上会拖出一大块色斑。

**更值得记的是它怎么被抓到的**：RTL、TB 期望模型、Python 模型三份实现来自
同一个错误心智模型，**互相比对全部 PASS**；是我的结构检查只扫框内像素，
把"框外误判"整个盖住了。最后是 ASCII 位图还原一眼看出来的。

> 结论：画框这种**空间图案**必须出位图/ASCII 肉眼过一遍，不能只看计数。

### bug 2｜位选越界导致 6 个字符格全变 '0'

`osd_coord.v` 里我照抄了 `hdmi_mixer.v` 的 `S_x_1d[11:4]`，
但**那边 `S_x` 是 12 位**，我这边的 `S_x_1d` 是 11 位 ⇒ 选不存在的 bit11
⇒ Verilog 返回 `1'bx` ⇒ `W_col` 全变未知 ⇒ 字符码 case 全部落到 default
⇒ 屏幕 6 个格子全画成同一个字符。

> 结论：**抄写法之前先核对被抄对象的位宽**。已改成 `[10:4]`。

### bug 3｜TB 边沿检测寄存器更新顺序写反，仿真跑死

先 `S_vsync_r = W_vsync;` 再判 `W_vsync && (S_vsync_r == 0)` ⇒ 恒假
⇒ 边沿永不到达 ⇒ 仿真 20s 超时才发现。已改成**先判边沿、再更新寄存器**。

### 另外两条"不是 bug 但要知道"的边界

- `hdmi_mixer` 现有坐标计数器：**首行 (r=0) 整行 S_x 比真实列号小 1**，
  且帧首像素 (0,0) 会采到上一帧残留的 `S_y=720`（同拍 user 才清 0）。
  只影响 y=0 一行、且被 160x160 Logo 覆盖 ⇒ 上板看不出来；
  但**画框一旦覆盖 y=0 就必须按 k+1 修正**。
  本包新写的 `pix_coord_gen.v` 改用「场消隐期归零」绕开了这个问题，无已知偏移。
- 复位后第 1 帧第 0 行会被灌进 **4 个伪 de 拍**（`rst_sync` 生效前 hcnt 冻结在 0
  而 `vtc_de` 已是 1）。只影响复位后那一帧那一行，行末 `last` 即把坐标归零。
  做波形对比时要跳过启动期，否则会误判成"坐标错乱"。

---

## 四点五、2026-10-10 复核修正（1 处）

| 位置 | 原值 | 改为 | 原因 |
|:---|:---|:---|:---|
| `box_draw.v` `B2_YMAX`（SELF_TEST 固定框） | `10'd655` | **`10'd656`** | B2=(64,464)-(256,655) 高 192，比 B0=(64,64)-(256,256) 高 193 **矮 1 px**，与第六节自检第 4 条「B0 与 B2 关于屏心对称」**自相矛盾**。改为 656 后 B2=[464,656]，与 B0 的 [64,256] 关于行区间中心严格镜像（720−256=464、720−64=656），两边同高 193。 |

同步更新了 `sim/bit_sim_box_draw.py` 的 `ST_BOXES` 与 `sim/bit_sim_system.py` 的 `BOXES`。
`tb_box_draw.v` 走的是 `SELF_TEST=0` 总线通路，不受影响（改动后已重跑确认 PASS）。

> 这不是逻辑 bug，而是**自检项与常量不符**——上板按第六节第 4 条检查时会看到
> “不对称”，容易误判成坐标系统出错，所以必须修掉。

---

## 五、给郭的 top 集成指引

> 方案B 1.4 节：**top 由郭一人维护，队友只交模块**。所以下面只是"接线建议"，
> 具体改由郭来做。本包**没有**动 `design_top_wrapper.v` / `.al` / `.sdc`。

### 5.1 在 `design_top_wrapper.v` 里实例化（像素时钟域 `S_hdmi_pixel_clk`）

```verilog
// ---- 张铭晨：全工程唯一像素坐标源（方案B 1.3）----
wire [10:0] S_pix_x, S_pix_x_1d, S_pix_x_2d;
wire [9:0]  S_pix_y, S_pix_y_1d, S_pix_y_2d;
wire        S_frame_done;

pix_coord_gen #(.H_ACTIVE(1280), .V_ACTIVE(720)) u_pix_coord_gen (
    .I_clk(S_hdmi_pixel_clk), .I_rst_n(S_hdmi_rst_n),
    .I_de(S_hdmi_de), .I_last(S_hdmi_last), .I_vsync(S_hdmi_vsync),
    .O_pix_x(S_pix_x), .O_pix_y(S_pix_y),
    .O_pix_x_1d(S_pix_x_1d), .O_pix_x_2d(S_pix_x_2d),
    .O_pix_y_1d(S_pix_y_1d), .O_pix_y_2d(S_pix_y_2d),
    .O_frame_done(S_frame_done)
);

// ---- D1：先画固定框验证坐标（SELF_TEST=1，忽略总线）----
wire        S_box_hit;
wire [23:0] S_box_color;

box_draw #(.SELF_TEST(1)) u_box_draw (
    .I_clk(S_hdmi_pixel_clk), .I_rst_n(S_hdmi_rst_n),
    .I_de(S_hdmi_de), .I_pix_x(S_pix_x), .I_pix_y(S_pix_y),
    .I_box_valid(4'b0000),          // SELF_TEST=1 时这些口被常量折叠，接 0 即可
    .I_box_xmin(44'd0), .I_box_xmax(44'd0),
    .I_box_ymin(40'd0), .I_box_ymax(40'd0),
    .I_cross_en(1'b1), .I_cross_x(11'd640), .I_cross_y(10'd360),
    .O_hit(S_box_hit), .O_color(S_box_color), .O_de()
);

// ---- 坐标 OSD（D1 固定显示标定点 640/360；D4 起改接目标框心）----
wire        S_coord_hit;
wire [23:0] S_coord_color;

osd_coord u_osd_coord (
    .I_clk(S_hdmi_pixel_clk), .I_rst_n(S_hdmi_rst_n),
    .I_de(S_hdmi_de), .I_pix_x(S_pix_x), .I_pix_y(S_pix_y),
    .I_x_val(11'd640), .I_y_val(10'd360),
    .I_frame_done(S_frame_done),
    .O_hit(S_coord_hit), .O_color(S_coord_color)
);
```

### 5.2 `hdmi_mixer.v` 需要加两个输入端口（这是唯一需要改 hdmi_mixer 的地方）

```verilog
// 端口表里加：
input  wire        I_ext_hit,      // 外部叠加命中（box_draw / osd_coord）
input  wire [23:0] I_ext_color,
```

然后在**第 17 节**那个 `always @(*)` 的 `S_osd_hit` 里补一个**最低优先级**分支
（放在现有文字分支之后，保证 Logo 与五处中文 OSD 仍在最上层）：

```verilog
else if(I_ext_hit) begin
    S_osd_hit   = 1'b1;
    S_osd_color = I_ext_color;
end
```

顶层实例化时：

```verilog
.I_ext_hit   ( S_box_hit   || S_coord_hit   ),
.I_ext_color ( S_box_hit ? S_box_color : S_coord_color ),
```

**对齐说明（已核算，可放心）**：`box_draw` / `osd_coord` 内部都按
「坐标延 2 拍 + de 延 2 拍」处理，与 `hdmi_mixer` 的 `S_x_2d` / `S_video_de_2d`
是同一个舞台，两边都是 `I_video_de` 起算的 2 拍 ⇒ 像素级对齐。
若上板后发现框整体右/下偏 1~2 px，先查这里是不是多/少接了一级延时。

### 5.3 加入 TD 编译清单

新建文件后必须同时加进 `td_project/camera_to_dsi_display.al`（否则 TD 不编译）：

```
user_source/hdl_source/track_box/rtl/pix_coord_gen.v
user_source/hdl_source/track_box/rtl/box_draw.v
user_source/hdl_source/track_box/rtl/osd_coord.v
user_source/hdl_source/track_box/rtl/track_glyph_rom.v
```

---

## 六、上板自检（D1 验收动作）

烧录后照着看这四条，全过就说明坐标系统与画框对齐都对：

1. 十字准星是否正好落在 **B3 框的正中心**？
2. 左下角 OSD 读数是否 **X:0640 / Y:0360**？
3. **B1 的右边框**是否贴着屏幕最右列（x=1279）而没有缺一段？
4. **B0 与 B2** 是否关于屏幕水平中线（y=360）对称？

任一条错 → 先查坐标源，再查延时拍数。ASCII 预览见：

```bash
cd user_source/hdl_source/track_box
python sim/bit_sim_box_draw.py preview
```

---

## 七、复现全部验证（一条条可跑）

```bash
cd E:/workbuddy_1/user_source/hdl_source/track_box

# 1) 坐标系统位级审计
python sim/vtc_coord_probe.py 3

# 2) 字模生成 + 从 RTL 原文渲染校验（肉眼确认 0-9 / : / X / Y 字形）
python sim/glyph_rom_tool.py gen
python sim/glyph_rom_tool.py check

# 3) 画框
iverilog -g2005 -o sim/tb_box_draw.vvp tb/tb_box_draw.v \
         rtl/box_draw.v rtl/pix_coord_gen.v
vvp sim/tb_box_draw.vvp
python sim/bit_sim_box_draw.py check
python sim/bit_sim_box_draw.py preview

# 4) 坐标 OSD（三种数值各跑一遍）
iverilog -g2005 -o sim/tb_osd_coord.vvp tb/tb_osd_coord.v \
         rtl/osd_coord.v rtl/pix_coord_gen.v rtl/track_glyph_rom.v
vvp sim/tb_osd_coord.vvp +xv=640  +yv=360 && python sim/bit_sim_osd_coord.py --xv 640  --yv 360
vvp sim/tb_osd_coord.vvp +xv=1279 +yv=719 && python sim/bit_sim_osd_coord.py --xv 1279 --yv 719
vvp sim/tb_osd_coord.vvp +xv=0    +yv=0   && python sim/bit_sim_osd_coord.py --xv 0    --yv 0

# 5) 系统级仿真（真实 720p60 场时序，整帧 1280x720）
iverilog -g2005 -o sim/tb_system.vvp tb/tb_track_system.v \
         rtl/pix_coord_gen.v rtl/box_draw.v rtl/osd_coord.v rtl/track_glyph_rom.v
vvp sim/tb_system.vvp          # 约 45 s；产出 sim/tb_system_frame.bin
python sim/bit_sim_system.py   # 第三方逐像素核对 + 出 5 张 PNG
```

`tb_box_draw_out.txt` / `tb_osd_coord_out.txt` 是仿真 dump，可随时删。

---

## 八、D2~D3 计划（按方案B 3.3 节）

| 天 | 任务 | 依赖 |
|:---|:---|:---|
| D2 | 目标列表总线接入（替掉 `SELF_TEST`）：`{valid, cls, cx, cy, xmin, xmax, ymin, ymax, area}` 深度 4，vsync 锁存 | 张金艺给出检测结果 |
| D2 | OSD 增加目标编号（在框旁标 `1/2/3/4`） | 需要给字模加数字以外的标号字形 |
| D3 | 最近邻关联（代价 = 曼哈顿距离 + 面积差 + 类别惩罚）+ EMA 平滑 + 死区 + 丢失滞回 | 段 3.1 |
| D3 | 画框加粗/变色区分「已锁定 / 新目标 / 即将丢失」 | 本包 |

> 段 3.1 的"框要稳"三件套（EMA `box = box + ((new-box)>>2)`、死区 `|Δ|<2px` 不更新、
> 连续 5 帧无匹配才删框）留到 D3，属于关联逻辑不在本画框模块内。

---

## 九、Git 状态说明

- 本包全部是**新增文件**（`git status` 显示为 untracked），**未 commit、未 push**。
- 远端配置未改动：`origin` → `zmc-833/mipi-hdmi`（可写），
  `upstream` → `Severus-5-2/mipi-hdmi` 且 `pushurl = DISABLED`（已锁死）。
- 需要提交时请先确认 `git remote get-url --push origin` 是 `zmc-833` 的地址；
  push 需要 GitHub PAT（本机没有已存凭据）。
