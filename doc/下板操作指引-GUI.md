# 下板操作指引 —— 合并后工程（2026-10-10）

> 合并已完成并推送：`Rrrrrr_` 分支 commit **`df13a22`**
> 本机 TD：`C:\TD`（V6.2.178840），license 有效期至 **2029-08-17**，`HOST_ID=ANY`

---

## 一、已完成的验证（软件侧全绿）

| 验证项 | 结果 |
|:---|:---|
| 集成包 MD5 校验 | ✅ 4 个文件全部一致，无损坏 |
| 三文件 diff（你的 vs 他的） | ✅ **全部是新增，你的代码零改动** |
| iverilog 语法检查（全模块） | ✅ 通过 |
| `detect_color_mask` 像素级对拍 | ✅ **272/272**，灰阶零色偏，色卡断言通过 |
| `ycbcr_convert` 等价性 | ✅ 逐位完全一致 |
| `box_draw` TB | ✅ 坐标 6144 拍 0 错、叠加 0 错 |
| `osd_coord` TB | ✅ 区域/颜色错误 0 |
| `tb_track_system` 系统级 | ✅ 921600 像素 × 368 万拍坐标，0 错，跨帧一致 |
| `bit_sim_system.py` 位级 | ✅ 逐像素完全一致，5 张 PNG 已出 |
| **TD 源码解析** | ✅ **Successfully analyzed 99 source files，0 ERROR** |

---

## 二、TD 命令行能做 / 不能做（重要结论）

我实测了 `C:\TD\bin\td_commands_prompt.exe` 的批处理模式：

**能做**（已验证）：
```bash
cd D:/workbuddy/mipi-hdmi/td_project
cat tools/td_full_flow.tcl | C:/TD/bin/td_commands_prompt.exe
```
- `import_device ph1_35p.db -package PH1P35MDG324 -speed 3` ✅
- `open_project -single_run ./camera_to_dsi_display.al` ✅ → **99 文件解析，0 ERROR**

**不能做**（TD 命令行 batch 的限制）：
- **综合**：`launch_runs syn_1` → `PRJ-8806 ERROR: Run syn_1 has exited unexpectedly`
  - 根因：batch 模式下 `elaborate` 只加载了 **42 个文件**（全是 DDR IP），
    `design_top_wrapper.v` 未被解析到 → `HDL-8001: Incorrect top-level module name`
  - 这是 TD 从 `_Runs/syn_1/` 目录解析相对路径 `../../../user_source/` 的缺陷
  - **不是我们代码的问题**：源文件解析阶段 99 个全过

> 张铭晨的文档里也提到过同一现象（"命令行方式没能把器件导入这一步配对上...布局布线/时序/bitgen 没有在这里跑完"）。
> **结论：综合/布局布线/bitgen 必须走 TD GUI。**

---

## 三、GUI 操作步骤（你要做的）

### 步骤 1：打开工程

```
① 启动 TD 6.2.1（C:\TD\bin\design_integrator.exe 或开始菜单）
② File → Open Project
③ 选 D:/workbuddy/mipi-hdmi/td_project/camera_to_dsi_display.al
④ 确认器件栏显示：PH1P35MDG324
```

### 步骤 2：综合

```
左侧流程树 → Synthesis (syn_1) → 右键 → Run
```

**过关条件**：Messages 里搜 `track_box` 和 `detect_color_mask` → **应该一条都没有**。
若出现 `HDL-5xxx ERROR`，把原文发我。

### 步骤 3：布局布线

```
左侧流程树 → Physical Design (phy_1) → 右键 → Run
```

**过关条件**：看 **Timing Summary** 的 **WNS**。D1 新增的只是比较器 + 小 ROM，
预期 **WNS ≥ 0**。若为负，多半不是 D1 引入的（DDR/MIPI 才是大头）。

### 步骤 4：下载

```
Tools → Download → 选 camera_to_dsi_display.bit → 下载到 SRAM
```

**前置检查**（重要）：
- PH1P35 板 + HX1P35A 底板已连接
- SC520CS 摄像头 MIPI 排线插牢
- HDMI 屏或采集卡已接
- USB 下载器已插（驱动装好）
- 拨码开关：建议先拨到**彩色原图模式**

---

## 四、屏幕上应该看到什么

### 张铭晨侧（画框 + 坐标 OSD）

| 元素 | 位置 | 颜色 |
|:---|:---|:---|
| B0 红框 | 左上 (64,64)–(256,256) | 纯红 |
| B1 绿框 | 右上 (1088,64)–(1279,256) | 纯绿，**右边框贴屏** |
| B2 蓝框 | 左下 (64,464)–(256,656) | 蓝，与 B0 镜像 |
| B3 黄框 | 中央 (510,290)–(770,430) | 黄，框心 (640,360) |
| 白色准星 | 中心 (640,360) | 白 |
| 坐标 OSD | 左下 | 黄，第一行 `X:0640`，第二行 `Y:0360` |

**核心验收点**：**准星压在 B3 黄框正中心** ⟺ **OSD 读出 `X:0640`/`Y:0360`**

### 你的掩膜（张金艺侧）

| # | 现象 |
|:---|:---|
| 1 | 目标物区域变色（默认绿），背景不变 |
| 2 | 掩膜在底、画框在上，互不遮挡 |
| 3 | 掩膜边缘不拖尾、不整行串色 |

---

## 五、上板自检 6 条

**张铭晨侧（4 条）**：
- [ ] 1. 白色准星在 B3 黄框正中心？
- [ ] 2. 左下 OSD 读 `X:0640` / `Y:0360`？
- [ ] 3. B1 右边框贴到 x=1279，没缺一段？
- [ ] 4. B0 与 B2 关于 y=360 对称？

**你的掩膜（2 条）**：
- [ ] 5. 目标物区域变色，背景不变？
- [ ] 6. 两者共存互不遮挡？

---

## 六、可能踩的坑（照这个排查）

| 现象 | 先查什么 |
|:---|:---|
| 综合报 license 错 | `C:\TD\license\Anlogic.lic` 是否在（本机**已确认存在**，有效期至 2029） |
| 4 个框完全没出现 | `.al` 里 track_box 4 条是否 `UsedInSyn=true`（**已确认**）；top 里 `u_box_draw` 是否被综合 |
| 框整体右/下偏 1~2 px | 流水拍数（`box_draw`/`osd_coord` 是「延 2 拍」，与 mixer `S_x_2d` 对齐） |
| 掩膜整体偏移 | `detect_color_mask` 的 `PIX_DLY=2`（接 `S_pix_x_2d`）；已配好 |
| 掩膜完全不出现 | 顶层 `u_detect_color_mask` 例化是否被综合；`S_mask_hit` 是否接到模块输出 |
| 掩膜盖住了框 | 优先级：坐标 OSD > 画框 > 掩膜。若掩膜在上，查 `S_ovl_color` 的三元表达式 |

---

## 七、回传给我

1. 自检 6 条分别 YES / NO
2. 屏幕实拍或截图（整屏）
3. Messages 里搜 `track_box` / `detect_color_mask` 的结果
4. Timing Summary 的 **WNS** 数值
5. 若有不符：哪个框/哪块掩膜、偏多少、往哪个方向偏
