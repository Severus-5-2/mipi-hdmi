# ==================================================================
# TD 命令行验证脚本 —— 合并后工程（张金艺 + 张铭晨）
#   目标：源码解析 + elaboration + 综合，验证合并后无错
#   用法：cd C:/TD/bin && td_commands_prompt.exe
#         source D:/workbuddy/mipi-hdmi/tools/td_check.tcl
# ==================================================================

puts "===== [1/4] 导入器件 PH1P35MDG324 ====="
import_device ph1_35p.db -package PH1P35MDG324 -speed -3

puts "===== [2/4] single_run 模式打开现有工程 ====="
open_project -single_run D:/workbuddy/mipi-hdmi/td_project/camera_to_dsi_display.al

puts "===== [3/4] 运行综合 ====="
launch_runs syn_1 -jobs 4
wait_run syn_1 -quiet

puts "===== [4/4] 运行布局布线 + 出报告 ====="
launch_runs phy_1 -jobs 4
wait_run phy_1 -quiet

report_timing_summary -file D:/workbuddy/mipi-hdmi/td_project/cmd_timing_summary.rpt
report_area -file D:/workbuddy/mipi-hdmi/td_project/cmd_area.rpt
report_qor -step route -file D:/workbuddy/mipi-hdmi/td_project/cmd_qor.rpt

puts "===== 完成，报告已写在 td_project/ 下 ====="
