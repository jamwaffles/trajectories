set key autotitle columnhead
set datafile separator ","

set term svg size 2500,1250
set timestamp

set y2tics
set xlabel "Position"
set xrange [1050:1250]

set output 'target/integ_fwd_end_step.svg'

set multiplot layout 2,1

load 'gnuplot/dark2.pal'

set title "C++"

plot 'target/integ_fwd_end_step_cpp.csv' using 2:3 ls 1 with linespoints title 'Last point point position', \
'target/integ_fwd_end_step_cpp.csv' using 2:3:(sprintf('%f: %f', $2, $3)) with labels center offset 0,0.5, \

set title 'Rust'

# set grid x2tics
# set x2tics (50, 140.23806829245765, 380.71934373703334, 879.8507114774787, 1084.8016572321708, 1163.3414735719157)

plot 'target/instrument_integ_fwd_end_step.csv' using 2:3 ls 1 with linespoints title 'Last point point position', \
'target/instrument_integ_fwd_end_step.csv' using 2:3:(sprintf('%f: %f', $2, $3)) with labels center offset 0,0.5, \
