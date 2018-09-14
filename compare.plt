set key autotitle columnhead
set datafile separator ","

set term svg size 1440,900

set output 'target/compare_cpp.svg'
set title 'C++ Rust bindings'

plot 'target/plot_cpp_bindings.csv' using 1:2 with lines title 'Position X', 'target/plot_cpp_bindings.csv' using 1:3 with lines title 'Position Y', 'target/plot_cpp_bindings.csv' using 1:4 with lines title 'Position Z' , 'target/plot_cpp_bindings.csv' using 1:5 with lines title 'Accel X', 'target/plot_cpp_bindings.csv' using 1:6 with lines title 'Accel Y', 'target/plot_cpp_bindings.csv' using 1:7 with lines title 'Accel Z'

set output 'target/compare_bindings.svg'
set title 'C++ native'

plot 'target/plot_cpp_native.csv' using 1:2 with lines title 'Position X', 'target/plot_cpp_native.csv' using 1:3 with lines title 'Position Y', 'target/plot_cpp_native.csv' using 1:4 with lines title 'Position Z' , 'target/plot_cpp_native.csv' using 1:5 with lines title 'Accel X', 'target/plot_cpp_native.csv' using 1:6 with lines title 'Accel Y', 'target/plot_cpp_native.csv' using 1:7 with lines title 'Accel Z'