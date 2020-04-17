create_project -force project_5 ./build/project_5 -part xc7a100tcsg324-1
add_files -norecurse [glob ./hdl/*.sv]
add_files -norecurse [glob ./hdl/asm/*.mem]
add_files -fileset constrs_1 -norecurse ./constraint/Nexys_A7.xdc
close_project
