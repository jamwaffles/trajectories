set key autotitle columnhead
set datafile separator ","

set term svg size 2500,1250
set timestamp

set y2tics
set xlabel "Position"

set output 'target/next_sw_point.svg'

set multiplot layout 2,1

load 'gnuplot/dark2.pal'

set title "C++"

plot 'target/next_sw_point_cpp.csv' using 2:3 ls 1 with linespoints title 'Switching point position', \
'target/next_sw_point_cpp.csv' using 2:4 ls 2 with linespoints title 'Switching point velocity', \
'target/next_sw_point_cpp.csv' using 2:5 ls 3 with linespoints title 'Before accel', \
'target/next_sw_point_cpp.csv' using 2:6 ls 4 with linespoints title 'After accel', \
'target/next_sw_point_cpp.csv' using 2:3:(sprintf('%f: %f', $2, $3)) with labels notitle center offset 0,0.5, \
'target/next_sw_point_cpp.csv' using 2:4:(sprintf('%f: %f', $2, $4)) with labels notitle center offset 0,1, \
'target/next_sw_point_cpp.csv' using 2:5:(sprintf('%f: %f', $2, $5)) with labels notitle center offset 0,0.5, \
'target/next_sw_point_cpp.csv' using 2:6:(sprintf('%f: %f', $2, $6)) with labels notitle center offset 0,-0.5

set title 'Rust'

# set grid x2tics
# set x2tics (50, 140.23806829245765, 380.71934373703334, 879.8507114774787, 1084.8016572321708, 1163.3414735719157)

plot 'target/next_sw_point_rs.csv' using 2:3 ls 1 with linespoints title 'Switching point position', \
'target/next_sw_point_rs.csv' using 2:4 ls 2 with linespoints title 'Switching point velocity', \
'target/next_sw_point_rs.csv' using 2:5 ls 3 with linespoints title 'Before accel', \
'target/next_sw_point_rs.csv' using 2:6 ls 4 with linespoints title 'After accel', \
'target/next_sw_point_rs.csv' using 2:3:(sprintf('%f: %f', $2, $3)) with labels notitle center offset 0,0.5, \
'target/next_sw_point_rs.csv' using 2:4:(sprintf('%f: %f', $2, $4)) with labels notitle center offset 0,1, \
'target/next_sw_point_rs.csv' using 2:5:(sprintf('%f: %f', $2, $5)) with labels notitle center offset 0,0.5, \
'target/next_sw_point_rs.csv' using 2:6:(sprintf('%f: %f', $2, $6)) with labels notitle center offset 0,-0.5
