set key autotitle columnhead
set datafile separator ","

set term svg size 2500,1250
set timestamp

set y2tics
set xlabel "Position"
# set xrange [0:1400]

set output 'target/switching_point.svg'

set multiplot layout 2,1

load 'gnuplot/dark2.pal'

set title "C++"

plot 'target/switching_point_cpp.csv' using 2:3 ls 1 with points title 'Switching point position', \
'target/switching_point_cpp.csv' using 2:3:(sprintf('%f', $2)) with labels center offset 0,0.5, \

set title 'Rust'

# set grid x2tics
# set x2tics (50, 140.23806829245765, 380.71934373703334, 879.8507114774787, 1084.8016572321708, 1163.3414735719157)

plot 'target/instrument_switching_point.csv' using 2:3 ls 1 with points title 'Switching point position', \
'target/instrument_switching_point.csv' using 2:3:(sprintf('%f', $2)) with labels center offset 0,0.5, \
