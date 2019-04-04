# Produce test data with `cargo test --release compare_test_2`

set key autotitle columnhead
set datafile separator ","

set term svg size 1440,1440
set timestamp

set y2tics
set ylabel "Position"
set y2label "Velocity"

set output 'target/compare_test_2_native_cpp.svg'

set multiplot layout 2,1 \
              title "Compare C++ and Rust output of test2() in Test.cpp"

set title 'C++ Rust bindings'

plot 'target/compare_cpp_output.csv' using 1:2 lw 2 ps 0.4 pt 2 with lines title 'Position X', \
'target/compare_cpp_output.csv' using 1:3 lw 2 ps 0.4 pt 2 with lines title 'Position Y', \
'target/compare_cpp_output.csv' using 1:4 lw 2 ps 0.4 pt 2 with lines title 'Position Z', \
'target/compare_cpp_output.csv' using 1:5 lw 2 dt 4 with lines axes x1y2 title 'Velocity X', \
'target/compare_cpp_output.csv' using 1:6 lw 2 dt 4 with lines axes x1y2 title 'Velocity Y', \
'target/compare_cpp_output.csv' using 1:7 lw 2 dt 4 with lines axes x1y2 title 'Velocity Z'

set title 'Rust native'

set grid x2tics
set x2tics (50, 140.23806829245765, 380.71934373703334, 879.8507114774787, 1084.8016572321708, 1163.3414735719157)

plot 'target/compare_rust_output.csv' using 1:2 lw 2 ps 0.4 pt 2 with lines title 'Position X', \
'target/compare_rust_output.csv' using 1:3 lw 2 ps 0.4 pt 2 with lines title 'Position Y', \
'target/compare_rust_output.csv' using 1:4 lw 2 ps 0.4 pt 2 with lines title 'Position Z', \
'target/compare_rust_output.csv' using 1:5 lw 2 dt 4 with lines axes x1y2  title 'Velocity X', \
'target/compare_rust_output.csv' using 1:6 lw 2 dt 4 with lines axes x1y2  title 'Velocity Y', \
'target/compare_rust_output.csv' using 1:7 lw 2 dt 4 with lines axes x1y2  title 'Velocity Z'
