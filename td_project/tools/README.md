# TD 发布门禁

`td_log_gate.ps1` 检查 TD 物理实现日志中的阻断错误，并可将 final timing report 的端点数量与已知基线比较。TD 有时会在记录 `PHY-8077` 后继续到 bitgen，因此“流程结束/bit 文件存在”不能单独作为放行条件。

在 PowerShell 中运行：

```powershell
& .\td_project\tools\td_log_gate.ps1 `
  -RunLog .\td_project\camera_to_dsi_display_Runs\phy_1\run.log `
  -TimingReport .\td_project\camera_to_dsi_display_Runs\phy_1\final_timing.rpt `
  -BaselineEndpoints 18180
```

返回码 `0` 为未发现阻断项，`1` 为 BLOCK，`2` 为输入文件问题。被 `Similar messages will be suppressed` 隐藏的记录会明确提示数量未知。

对当前历史构建和 2026-10-20 候选构建的核对结果：历史版 PASS（18180 endpoints）；候选版 BLOCK（PHY-8077，端点 18152，低于基线 28，超过默认允许减少的 20）。

端点基线是本工程当前版本的参考值。更新设计后，应先复核合法的端点变化，再显式更新基线；门禁不能替代资源报告、最终时序和人工检查。
