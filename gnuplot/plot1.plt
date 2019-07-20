#!/usr/bin/gnuplot -c

# Plot a single value (second column) against time (first column)

# Usage: `./plot1 <input file CSV> <output file SVG>

set key autotitle columnhead
set datafile separator ","

set term svg size 2500,1250
set timestamp

set y2tics
set xlabel "Time"

set output ARG2

# set multiplot layout 2,1

load '../gnuplot/dark2.pal'

plot ARG1 using 1:2 ls 1 with lines title 'Value 1'
