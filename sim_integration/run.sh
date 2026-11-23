#!/bin/sh
# System sim: basys3_top (SD_controller + sd_block_adapter + fat32_top)
# against a bit-level SPI SD card serving a generated FAT32 image.
# Needs: python3 (or python), iverilog/vvp.  Run from anywhere.
set -e
cd "$(dirname "$0")"
PY=$(command -v python3 || command -v python)
"$PY" make_fat32_image.py .
iverilog -g2005 -Wall -o tb_basys3_top.vvp -s tb_basys3_top tb_basys3_top.v \
    ../SPI_driver.v ../SD_cmd.v ../SD_controller.v ../sd_block_adapter.v \
    ../fat32_locate.v ../fat32_reader.v ../fat32_top.v ../basys3_top.v
vvp -n tb_basys3_top.vvp
