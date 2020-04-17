open_project ./build/project_5/project_5.xpr
add_files -norecurse [glob ./hdl/*.sv]
add_files -norecurse [glob ./hdl/asm/*.mem]
add_files -fileset constrs_1 -norecurse ./constraint/Nexys_A7.xdc
close_project
