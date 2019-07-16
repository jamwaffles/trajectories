set key autotitle columnhead
set datafile separator ","

set term svg size 2500,1250
set timestamp

set y2tics
set xlabel "Time"

set output 'target/get_pos.svg'

set multiplot layout 2,1

load 'gnuplot/dark2.pal'

set title "C++"

plot 'target/get_pos_cpp.csv' using 2:3 ls 1 with linespoints axes x1y2 title 'Previous pos', \
'target/get_pos_cpp.csv' using 2:4 ls 2 with linespoints title 'Previous vel', \
'target/get_pos_cpp.csv' using 2:5 ls 3 with linespoints axes x1y2 title 'Current pos', \
'target/get_pos_cpp.csv' using 2:6 ls 4 with linespoints title 'Current vel'

set title 'Rust'

plot 'target/instrument_get_pos.csv' using 2:3 ls 1 with linespoints axes x1y2 title 'Previous pos', \
'target/instrument_get_pos.csv' using 2:4 ls 2 with linespoints title 'Previous vel', \
'target/instrument_get_pos.csv' using 2:5 ls 3 with linespoints axes x1y2 title 'Current pos', \
'target/instrument_get_pos.csv' using 2:6 ls 4 with linespoints title 'Current vel'
