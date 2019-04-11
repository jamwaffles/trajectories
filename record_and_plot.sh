#!/bin/bash

PLOT_SCRIPT="$1"

export RUST_LOG=trajectories=trace

PLOT_NAME=$(basename $PLOT_SCRIPT .plt)

RS_GREP="RS ${PLOT_NAME}"
CPP_GREP="CPP ${PLOT_NAME}"

BASE_CMD="cargo test -p trajectories --release compare_test_2 -- --nocapture"

RS_CSV="./target/${PLOT_NAME}_rs.csv"
CPP_CSV="./target/${PLOT_NAME}_cpp.csv"

echo "Recording Rust data to ${RS_CSV}"
$($BASE_CMD 2>&1 | rg "${RS_GREP}" > ${RS_CSV})

echo "Recording C++ data to ${CPP_CSV}"
$($BASE_CMD 2>&1 | rg "${CPP_GREP}" > ${CPP_CSV})

echo "Plot ${PLOT_SCRIPT}"
gnuplot $PLOT_SCRIPT

echo "Done"
