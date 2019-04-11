set key autotitle columnhead
set datafile separator ","

set term svg size 2500,1250
set timestamp

set y2tics
set xlabel "Position"

set output 'target/acc_at.svg'

set multiplot layout 2,1

load 'gnuplot/dark2.pal'

set title "C++"

plot 'target/back_step_cpp.csv' using 2:3 ls 1 with dots title 'Velocity', \
'target/back_step_cpp.csv' using 2:4 ls 2 with dots title 'Max acceleration at'

set title 'Rust'

# set grid x2tics
# set x2tics (50, 140.23806829245765, 380.71934373703334, 879.8507114774787, 1084.8016572321708, 1163.3414735719157)

plot 'target/back_step_rs.csv' using 2:3 ls 1 with dots title 'Velocity', \
'target/back_step_rs.csv' using 2:4 ls 2 with dots title 'Max acceleration at'
