create_project -force vexriscv ./build/project -part xc7a35tcpg236-1
add_files -fileset constrs_1 ./constraints.xdc
add_files -scan_for_includes .
update_compile_order -fileset sources_1
set_property top top [current_fileset]
update_compile_order -fileset sources_1
reset_run synth_1
launch_runs synth_1 -jobs 24 
wait_on_run synth_1
set_property STEPS.WRITE_BITSTREAM.ARGS.BIN_FILE true [get_runs impl_1]
launch_runs -to_step write_bitstream impl_1 -jobs 24
wait_on_run impl_1