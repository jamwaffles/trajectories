set key autotitle columnhead
set datafile separator ","

set term png size 1440,900

set output 'compare_cpp.png'

plot 'example_cpp.csv' using 1:2 with lines title 'Position X', 'example_cpp.csv' using 1:3 with lines title 'Position Y', 'example_cpp.csv' using 1:4 with lines title 'Position Z', 'example_cpp.csv' using 1:5 with lines title 'Accel X', 'example_cpp.csv' using 1:6 with lines title 'Accel X', 'example_cpp.csv' using 1:7 with lines title 'Accel X'

set output 'compare_bindings.png'

plot 'test_sys.csv' using 1:2 with lines title 'Position X', 'test_sys.csv' using 1:3 with lines title 'Position Y', 'test_sys.csv' using 1:4 with lines title 'Position Z', 'test_sys.csv' using 1:5 with lines title 'Accel X', 'test_sys.csv' using 1:6 with lines title 'Accel X', 'test_sys.csv' using 1:7 with lines title 'Accel X'