#!/usr/bin/gnuplot -c

# Usage: `./plot-jerk <input file CSV> <output file SVG>

set output ARG2

set offset graph 0.10, 0.10, 0.10, 0.10
set key autotitle columnhead
set datafile separator ","

set term svg size 1440,1440
set timestamp

# set xrange [0:15]

# set multiplot layout 3,1
set title "Jerk output"

load '../gnuplot/dark2.pal'

do for [i=1:6] {
    set style line i linewidth 2
}

plot ARG1 using 1:2 ls 1 with lines title 'Position X', \
ARG1 using 1:3 ls 2 with lines title 'Position Y', \
ARG1 using 1:4 ls 3 with lines title 'Position Z', \
ARG1 using 1:5 ls 4 dt 4 with lines title 'Velocity X', \
ARG1 using 1:6 ls 5 dt 4 with lines title 'Velocity Y', \
ARG1 using 1:7 ls 6 dt 4 with lines title 'Velocity Z', \
ARG1 using 1:8 ls 7 dt 6 with lines title 'Accel X', \
ARG1 using 1:9 ls 8 dt 6 with lines title 'Accel Y', \
ARG1 using 1:10 ls 9 dt 6 with lines title 'Accel Z'
# ARG1 using 1:5 ls 4 dt 4 with lines axes x1y2 title 'Velocity X', \
# ARG1 using 1:6 ls 5 dt 4 with lines axes x1y2 title 'Velocity Y', \
# ARG1 using 1:7 ls 6 dt 4 with lines axes x1y2 title 'Velocity Z'
