set build_start [clock seconds]
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
puts "Checking native phy_1 reports (controller has no imported netlist)"
foreach result {camera_to_dsi_display_Runs/phy_1/camera_to_dsi_display_pr.timing camera_to_dsi_display_Runs/phy_1/camera_to_dsi_display_phy.area camera_to_dsi_display_Runs/phy_1/camera_to_dsi_display.bit} {if {![file exists $result] || [file size $result] == 0 || [file mtime $result] < $build_start} {puts stderr "BUILD FAILED: missing or stale result $result"; exit 1}}
puts "===== reports done ====="

puts "===== ALL DONE ====="
exit
