#!/bin/bash -f
# ****************************************************************************
# Vivado (TM) v2019.1 (64-bit)
#
# Filename    : elaborate.sh
# Simulator   : Xilinx Vivado Simulator
# Description : Script for elaborating the compiled design
#
# Generated by Vivado on Mon Apr 27 13:33:05 PDT 2020
# SW Build 2552052 on Fri May 24 14:47:09 MDT 2019
#
# Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
#
# usage: elaborate.sh
#
# ****************************************************************************
set -Eeuo pipefail
echo "xelab -wto 17d30a6cbc7b4debbd5068ed0aa3d74f --incr --debug typical --relax --mt 8 -L xil_defaultlib -L unisims_ver -L unimacro_ver -L secureip --snapshot cpu_tb_behav xil_defaultlib.cpu_tb xil_defaultlib.glbl -log elaborate.log"
xelab -wto 17d30a6cbc7b4debbd5068ed0aa3d74f --incr --debug typical --relax --mt 8 -L xil_defaultlib -L unisims_ver -L unimacro_ver -L secureip --snapshot cpu_tb_behav xil_defaultlib.cpu_tb xil_defaultlib.glbl -log elaborate.log

