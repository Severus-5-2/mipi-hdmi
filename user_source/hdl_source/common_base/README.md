# common_base —— 两人共用的公共基础模块

> 项目：MIPI-HDMI 实时视频图像处理系统（方案B）
> 维护：**张金艺**（扩展2）+ **张铭晨**（进阶2 / 进阶4）
> 依据：`doc/协作规范-代码一致性.md`、`doc/接口冻结表.md`

---

## 这个目录是什么

方案B §1 要求「两人共用一套『取数 + 连通域 + OSD』基础设施」。

**"共用一套"的唯一可靠实现方式，是物理上只有一份文件。** 本目录就是那一份。

```
common_base/
├── README.md              本文件
├── common_defs.vh         全局常量、位宽、接口位域（★ 接口宪法）
├── pix_counter.v          公共像素坐标 x/y + frame_done（§1.3）
├── ycbcr_convert.v        RGB888 → YCbCr（§1.1，位级复用 chroma_denoise）
├── target_bus.v           目标列表总线打包/解包（§4：艺 → 晨）
├── conn_label.v           [待建] 游程法连通域（D2，张金艺主导）
├── conn_features.v        [待建] 帧末特征统计：面积/Σx/Σy/bbox
└── div_shift_sub.v        [待建] 移位-减法除法器（消隐期质心除法）
```

---

## 五条铁律

1. ❌ **禁止复制粘贴本目录里任何内容到自己目录。**
   要用就实例化，或者 `` `include "common_defs.vh" ``。
   复制 = 分叉，分叉 = 合板出错。

2. ✅ **私有目录只放"两人不共用"的逻辑。**
   - 张金艺私有（`detect_zjy/`）：分类规则表、OSD 文字排版、BCD→ASCII
   - 张铭晨私有（`track_zmc/`）：最近邻匹配、EMA 平滑、帧差、形态学
   - **两人都用的东西 → 必须进 `common_base/`**

3. ⚠ **改本目录必须提 PR + 另一人 review。**
   提交信息里要写清：**为什么改 / 影响谁 / 是否影响接口**。

4. 🔒 **改接口相关文件（`common_defs.vh`）需双方同意**，
   并同步更新 `doc/接口冻结表.md` 的版本号。

5. 📌 **每个公共模块必须自带 testbench**，testbench 也要进版本库。

---

## 使用方法

### 集成到 TD 工程

1. 把本目录下的 `.v` 加入 `td_project/camera_to_dsi_display.al` 的源文件清单；
2. 把本目录路径加入 **include 搜索路径**（因为 `.v` 里用了 `` `include "common_defs.vh" ``）；
3. `.vh` 文件**不要**单独加入综合清单，它只被 include。

### 在代码里引用

```verilog
`include "common_defs.vh"

// 用宏，不要写魔法数字
if (w_pix_x == `PIX_X_MAX) ...

target_pack u_pack (...);
```

### 编译顺序

TD / iverilog 按依赖自动解析。命令行仿真（iverilog）建议：

```bash
iverilog -I user_source/hdl_source/common_base \
         -o sim.out tb_xxx.v xxx.v \
         user_source/hdl_source/common_base/pix_counter.v \
         user_source/hdl_source/common_base/ycbcr_convert.v
```

---

## 修改日志

| 日期 | 修改人 | 内容 | 双方确认 |
|:---|:---|:---|:---|
| 2026-10-10 | 张金艺 | 首版建立：`common_defs.vh` / `pix_counter.v` / `ycbcr_convert.v` / `target_bus.v` | 待确认 |

---

## 待建模块排期

| 模块 | 里程碑 | 主导 | 说明 |
|:---|:---|:---|:---|
| `conn_label.v` | D2–D4 | 张金艺 | 行内 RLE + 行间并查表 + 深度 32 等价表。**张铭晨的运动检测强依赖此模块** |
| `conn_features.v` | D2–D4 | 张金艺 | 帧末统计 area / Σx / Σy / bbox |
| `div_shift_sub.v` | D4 | 张金艺 | vsync 消隐期移位-减法除法，算质心 |
