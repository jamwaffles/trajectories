set key autotitle columnhead
set datafile separator ","

set term svg size 2500,1250
set timestamp

set y2tics
set xlabel "Position"
# set xrange [0:1400]

set output 'target/circ_seg.svg'

set multiplot layout 2,1

load 'gnuplot/dark2.pal'

set title "C++"

plot 'target/circ_seg_cpp.csv' using 2:3 ls 1 with points title "Angle", \
'target/circ_seg_cpp.csv' using 2:4 ls 2 with points title "Radius", \
'target/circ_seg_cpp.csv' using 2:3:(sprintf('Len %f: ang %f', $2, $3)) with labels center offset 0,0.5, \
'target/circ_seg_cpp.csv' using 2:4:(sprintf('Len %f: rad %f', $2, $4)) with labels center offset 0,0.5, \

set title 'Rust'

# set grid x2tics
# set x2tics (50, 140.23806829245765, 380.71934373703334, 879.8507114774787, 1084.8016572321708, 1163.3414735719157)

plot 'target/circ_seg_rs.csv' using 2:3 ls 1 with points title "Angle", \
'target/circ_seg_rs.csv' using 2:4 ls 2 with points title "Radius", \
'target/circ_seg_rs.csv' using 2:3:(sprintf('Len %f: ang %f', $2, $3)) with labels center offset 0,0.5, \
'target/circ_seg_rs.csv' using 2:4:(sprintf('Len %f: rad %f', $2, $4)) with labels center offset 0,0.5, \
