# 本版本使用的 TD 自定义流程

`syn_1.tcl`、`phy_1.tcl` 是从 2026-10-22 22:56 成功生成 bit 的运行目录原样保存的构建输入。其中物理流程保留 `set_param place hfn_opt off`，用于本工程的高扇出优化兼容处理。两份文件的执行阶段由 TD 生成的 `settings.cfg` 决定。

工程文件对综合和物理运行都设置了 `useDefaultScript="false"`。编译目录被 Git 忽略，因此新检出工程后需先在 TD 打开工程，生成运行目录和配置文件，再将下面两份模板恢复到对应目录。TD 版本为 6.2.178840。

在工程根目录执行：

```powershell
Copy-Item -LiteralPath .\tools\td_run_templates\syn_1.tcl -Destination .\td_project\camera_to_dsi_display_Runs\syn_1\camera_to_dsi_display.tcl
Copy-Item -LiteralPath .\tools\td_run_templates\phy_1.tcl -Destination .\td_project\camera_to_dsi_display_Runs\phy_1\camera_to_dsi_display.tcl
```

随后使用原有 `tools/td_full_flow.tcl` 或 TD 中的对应运行。不要把模板当作独立控制台入口，入口仍是 `td_full_flow.tcl`。

本次仅保存实际构建使用的脚本，没有改写 RTL 或重新生成 bit。最终结果见 `doc/多颜色同时出框-去紫框-2026-10-22.md`；bit 及完整日志仍保存在本地交付目录。
