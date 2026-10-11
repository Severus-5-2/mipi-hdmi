# conn_features.v — 交付说明（D4：特征汇总 → target_bus）

> 项目：MIPI-HDMI 实时视频图像处理系统 · 方案B §2.3 / §3.1 / §4
> 负责人：**张金艺** ｜ 归属：`user_source/hdl_source/common_base/`（艺 → 晨）
> 依据：`doc/接口冻结表.md v1.1 §2`

---

## 一、一句话说明

把 `conn_label` 的**消隐期汇总脉冲流**（最多 32 个连通域，每拍一个）做**面积降序的前 4 名擂台**，
在帧界锁存后经 `target_bus.v` 打成 **384 bit 目标列表总线**，即冻结表 §2 的 `S_target_bus`，
供张铭晨的跟踪/关联消费。

---

## 二、交付物清单

| 文件 | 作用 |
|:---|:---|
| `conn_features.v` | 模块本体（综合用） |
| `tb_target_bus.v` | **全链路**位级 testbench（`conn_label` + `conn_features` + `target_pack`） |
| `verify_target_bus.py` | 全链路位级对拍脚本（复用 `verify_conn_label.py` 的图样与参考实现） |
| `run_verify_target_bus.sh` | 一键复现 |

---

## 三、端口

### 输入

| 端口 | 位宽 | 说明 |
|:---|:---:|:---|
| `clk` / `rst_n` | 1 | 像素时钟 / 低电平复位 |
| `I_frame_done` | 1 | 帧界脉冲（与 `conn_label` / `pix_coord_gen` **同源**）。用于锁存 + 清擂台 |
| `I_cmp_valid` | 1 | 汇总有效（接 `conn_label.O_cmp_valid`） |
| `I_cmp_label` | 6 | 紧凑编号（排序兜底用） |
| `I_cmp_cls` | 2 | 颜色分类（`CLS_*`） |
| `I_cmp_area` | 20 | 面积（排序主键） |
| `I_cmp_xmin/xmax` | 11 | 包围盒左右 |
| `I_cmp_ymin/ymax` | 10 | 包围盒上下 |
| `I_cmp_cx / I_cmp_cy` | 11 / 10 | 质心 |

> 以上 `I_cmp_*` 直接一一接到 `conn_label` 的同名 `O_cmp_*`（含 `O_cmp_cnt` 不需要接）。

### 输出

| 端口 | 位宽 | 说明 |
|:---|:---:|:---|
| `O_target_bus` | **384** | 4 槽 × 96 bit，槽 0 = 面积最大者；空槽字段全 0 |
| `O_target_valid_cnt` | 3 | 本帧有效目标数（0~4） |
| `O_update` | 1 | 锁存脉冲（1 拍），可选用于触发下游 |

---

## 四、参数

| 参数 | 默认 | 说明 |
|:---|:---:|:---|
| `SLOT_N` | `TARGET_MAX`(=4) | 槽数，转发给 `target_pack`；降级单目标检测时可改 1 |

---

## 五、排序规则（冻结表 §2.3，必须一致）

**槽位按 `area` 从大到小排列**，即：

```
键 1：area   降序（大者在前）
键 2：xmin   升序（同面积时，靠左者在前）   ← 冻结表 §2.3 明确要求，保证确定性
键 3：label  升序（再相同时的兜底，仍是确定序）
```

- **只保证同一帧内**的次序；**跨帧槽号不保证指向同一目标**（冻结表 §2.3）。
  跨帧的对应关系由消费方（张铭晨）用 `cls` + 位置自己关联。
- 目标数 >4 → 只保留面积最大的 4 个，其余丢弃（方案B §3.1 深度 4）。

---

## 六、实现要点

```
每拍（I_cmp_valid）：把新连通域插入 4 槽擂台
    只插入到「第一个被击败」的槽位 t；t 之后的槽整体后移一格，末位挤出
    后移规则 ★ 槽 j>t 拿 s[j-1]：
        t=0 → s0→s1, s1→s2, s2→s3
        t=1 →          s1→s2, s2→s3
        t=2 →                 s2→s3
        t=3 → 无后移
帧界（I_frame_done）：l* ← s*（锁存输出），s* ← 0（清擂台），O_update 拉高 1 拍
```

- **擂台 vs 先存后排**：存 32 个槽需 32 × 91 ≈ 2900 FF；擂台只要 4 × 91 ≈ **364 FF**，
  且天然一拍一个，无需额外排序状态机。
- 槽内 91 bit = `label(6) + cls(2) + cx(11) + cy(10) + xmin(11) + xmax(11) + ymin(10) + ymax(10) + area(20)`，
  其中 `label` **不进总线**，只用于排序兜底。
- 空槽判据：`area == 0`（`conn_label` 已保证进流的目标 `area ≥ MIN_AREA ≥ 1`）。

---

## 七、时序与「无目标」

- 汇总流在**场消隐期**发出；本模块在**下一个 `I_frame_done`（下一帧首）**锁存，
  此后整帧稳定，消费方随时可采样（冻结表 §2.2）。
- **本帧无目标** → 擂台全空 → 锁存后 `valid_cnt = 0` 且所有字段为 0，
  **不会留上一帧残留**（冻结表 §2.4）。

> ⚠ 注意区分两个「帧末」：
> - `conn_label` 的汇总 FSM 触发在**本帧最后一行结束**（消隐期，内部检测）；
> - 本模块的**锁存**在**下一帧首的 `I_frame_done`**。
>   两者的先后关系决定了「本帧结果在下一帧期间稳定」，与冻结表 §2.2 一致。

---

## 八、面积门限

**低于最小面积门限的连通域在 `conn_label` 里就不进汇总流**（其 `MIN_AREA` 参数负责）。
本模块**不重复门限**，以免两处参数不一致。

---

## 九、验证（全链路位级对拍，已通过）

```bash
cd user_source/hdl_source/common_base
bash run_verify_target_bus.sh
```

**当前结果（2026-10-11）**：11 帧图样，`target_bus` 384 bit **逐位**全过。

| 帧 | 覆盖点 | 结果 |
|:---:|:---|:---:|
| 0 | 无目标 → 全 0 | ✅ |
| 1 | 单目标 | ✅ |
| 2 | 双目标（面积不同：27 / 21） | ✅ |
| 3 | **双目标面积相同（36 / 36）→ 走 xmin 升序** | ✅ |
| 4~6 | L 形 / U 形 / 环形 单目标 + cls | ✅ |
| 7 | 9 个同面积目标 → **前 4 + label 兜底序** | ✅ |
| 8 | 对角相接两块（面积相同 20 / 20） | ✅ |
| 9 | 满帧单目标（全未知 → 未分类） | ✅ |
| 10 | 32 个同面积目标 → **前 4 + label 兜底序** | ✅ |

另校验：
- `B#0`（首帧锁存时的空擂台）**必须全 0** —— 已校验；
- 每帧 `valid_cnt` 与参考一致；
- 384 bit **独立于 RTL 的第二套位域实现**（Python 里按冻结表手工拼位）交叉验证了 `target_bus.v`。

> 🩸 调试期抓到的真 bug：**擂台后移表达式写错**（插入到槽 1 时，槽 2 误取 `s0` 导致复制一份）。
> 只在「两个目标面积相同」时暴露 —— 面积不同时插入点恒在槽 0、恰好绕开了该分支。
> 这类 bug 功能级仿真极易漏过，正是本项目坚持**位级对拍**的原因。

---

## 十、集成方法（给郭）

```verilog
wire [`TGT_BUS_W-1:0] S_target_bus;
wire [2:0]            S_target_valid_cnt;

conn_features #(
    .SLOT_N(`TARGET_MAX)
) u_conn_features (
    .clk(S_hdmi_pixel_clk), .rst_n(S_hdmi_rst_n),
    .I_frame_done(S_det_frame_done),
    .I_cmp_valid  (S_cmp_valid),
    .I_cmp_label  (S_cmp_label),
    .I_cmp_cls    (S_cmp_cls),
    .I_cmp_area   (S_cmp_area),
    .I_cmp_xmin   (S_cmp_xmin), .I_cmp_xmax(S_cmp_xmax),
    .I_cmp_ymin   (S_cmp_ymin), .I_cmp_ymax(S_cmp_ymax),
    .I_cmp_cx     (S_cmp_cx),   .I_cmp_cy  (S_cmp_cy),
    .O_target_bus       (S_target_bus),
    .O_target_valid_cnt (S_target_valid_cnt),
    .O_update           ()
);
```

**加进 TD 工程**：GUI `Source → Add Source...` 加 `conn_features.v`
（**手工改 `.al` 无效**）；本模块依赖 `target_bus.v`（工程内已有，勿重复添加）。
**testbench 不要加进综合清单**。

---

## 十一、已知限制

| 限制 | 影响 | 备注 |
|:---|:---|:---|
| 目标 >4 只留面积最大的 4 个 | 小目标被丢弃 | 方案B §3.1 深度 4 |
| 同面积 + 同 xmin 时按 label 升序 | 该次序无物理含义 | 仅为确定序；冻结表只要求保证「同帧确定」 |
| 跨帧槽号不稳定 | 直接按槽号做关联会「框跳目标」 | 冻结表 §2.3 已明确，须用 `cls`+位置关联 |
| 面积门限在 `conn_label` | 改门限要改 `conn_label` 的 `MIN_AREA` | 刻意不做两处门限 |

---

*文档生成：2026-10-11*
