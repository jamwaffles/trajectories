set key autotitle columnhead
set datafile separator ","

set term svg size 2500,1250
set timestamp
set xrange [0:1200]

set y2tics
set xlabel "Position"

set output 'target/forward_sw_point.svg'

set multiplot layout 2,1

load 'gnuplot/dark2.pal'

set pointsize 2

set title "C++"

plot 'target/forward_sw_point_cpp.csv' using 2:3 ls 1 with linespoints title 'CPP Velocity', \
'target/forward_sw_point_cpp.csv' using 2:4 ls 2 with linespoints title 'CPP Before acceleration', \
'target/forward_sw_point_cpp.csv' using 2:5 ls 3 with linespoints title 'CPP After acceleration'

plot 'target/forward_sw_point_rs.csv' using 2:3 ls 1 pt 4 with linespoints title 'RS Velocity', \
'target/forward_sw_point_rs.csv' using 2:4 ls 2 pt 4 with linespoints title 'RS Before acceleration', \
'target/forward_sw_point_rs.csv' using 2:5 ls 3 pt 4 with linespoints title 'RS After acceleration'
