#!/bin/bash

PLOT_SCRIPT="$1"

PLOT_NAME=$(basename $PLOT_SCRIPT .plt)

RS_GREP="RS ${PLOT_NAME}"
CPP_GREP="CPP ${PLOT_NAME}"

BASE_CMD="cargo test -p trajectories --release compare_test_2"

RS_CSV="./target/instrument_${PLOT_NAME}.csv"
CPP_CSV="./target/${PLOT_NAME}_cpp.csv"

echo "Recording Rust data to ${RS_CSV}"
INSTRUMENT=$PLOT_NAME $BASE_CMD

echo "Recording C++ data to ${CPP_CSV}"
RUST_LOG=trajectories=trace $BASE_CMD -- --nocapture 2>&1 | rg "${CPP_GREP}" > ${CPP_CSV}

echo "Plot ${PLOT_SCRIPT}"
gnuplot $PLOT_SCRIPT

echo "Done"
