puts "===== 1 import device ====="
import_device ph1_35p.db -package PH1P35MDG324 -speed 3

puts "===== 2 open project ====="
open_project -single_run ./camera_to_dsi_display.al

puts "===== 3 reset runs ====="
reset_runs syn_1

puts "===== 4 launch syn_1 ====="
launch_runs syn_1 -jobs 4
wait_run syn_1 -quiet
puts "===== syn_1 done ====="

puts "===== 5 launch phy_1 ====="
launch_runs phy_1 -jobs 4
wait_run phy_1 -quiet
puts "===== phy_1 done ====="

puts "===== 6 reports ====="
report_timing_summary -file ./cmd_timing_summary.rpt
report_area -file ./cmd_area.rpt
report_qor -step route -file ./cmd_qor.rpt
puts "===== reports done ====="

puts "===== ALL DONE ====="
exit
