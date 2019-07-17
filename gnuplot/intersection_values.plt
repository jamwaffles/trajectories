set key autotitle columnhead
set datafile separator ","

# Set term to `canvas` if on a linux machine
set term svg size 2500,1250
set timestamp

set y2tics
set xlabel "Position"

set termoption enhanced

set output 'target/intersection_values.svg'

set multiplot layout 2,1

load 'gnuplot/paired.pal'

set title "C++"

plot 'target/intersection_values_cpp.csv' using 2:3 ls 1 with lines title 'Start slope'
# 'target/intersection_values_cpp.csv' using 2:3:3 ls 1 with labels hypertext,
# 'target/intersection_values_cpp.csv' using 2:3:(sprintf('Int. pos. %f: start slope  %f', $2, $3)) with labels center offset 0,0.5, \

set title 'Rust'

plot 'target/instrument_intersection_values.csv' using 2:3 ls 1 with lines title 'Start slope', \
# 'target/intersection_values_rs.csv' using 2:3:(sprintf('Int. pos. %f: start slope  %f', $2, $3)) with labels center offset 0,0.5, \
