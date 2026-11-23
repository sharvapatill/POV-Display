# Non-project batch build of basys3_top -> build/vivado/basys3_top.bit
#   vivado -mode batch -nojournal -source vivado_build.tcl
# Does not touch the GUI project (.xpr).
set src [file dirname [file normalize [info script]]]
set out $src/build/vivado
file mkdir $out

read_verilog [list \
    $src/SPI_driver.v $src/SD_cmd.v $src/SD_controller.v \
    $src/sd_block_adapter.v \
    $src/fat32_locate.v $src/fat32_reader.v $src/fat32_top.v \
    $src/basys3_top.v]
read_xdc $src/basys3_sd.xdc

synth_design -top basys3_top -part xc7a35tcpg236-1
report_utilization -file $out/util_synth.rpt
opt_design
place_design
route_design
report_utilization     -file $out/util_impl.rpt
report_timing_summary  -file $out/timing.rpt
report_drc             -file $out/drc.rpt
report_cdc -details    -file $out/cdc.rpt
write_bitstream -force $out/basys3_top.bit
