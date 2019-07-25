#!/usr/bin/gnuplot -c

# Plot two values (second and third columns) against time (first column)

# Usage: `./plot2 <input file CSV> <output file SVG>

set key autotitle columnhead
set datafile separator ","

set term svg size 2500,1250
set timestamp

set y2tics
set ylabel "Position"
set y2label "Velocity"
set xlabel "Time"

set output ARG2

# set multiplot layout 2,1

load '../gnuplot/dark2.pal'

plot ARG1 using 1:2 ls 1 with linespoints title 'Position', \
ARG1 using 1:3 ls 2 with linespoints axes x1y2 title 'Velocity'
