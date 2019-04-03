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

plot 'target/compare_cpp_output.csv' using 1:2 with lines title 'Position X', \
'target/compare_cpp_output.csv' using 1:3 with lines title 'Position Y', \
'target/compare_cpp_output.csv' using 1:4 with lines title 'Position Z', \
'target/compare_cpp_output.csv' using 1:5 with lines axes x1y2 title 'Velocity X', \
'target/compare_cpp_output.csv' using 1:6 with lines axes x1y2 title 'Velocity Y', \
'target/compare_cpp_output.csv' using 1:7 with lines axes x1y2 title 'Velocity Z'

set title 'Rust native'

plot 'target/compare_rust_output.csv' using 1:2 with lines title 'Position X', \
'target/compare_rust_output.csv' using 1:3 with lines title 'Position Y', \
'target/compare_rust_output.csv' using 1:4 with lines title 'Position Z', \
'target/compare_rust_output.csv' using 1:5 with lines axes x1y2  title 'Velocity X', \
'target/compare_rust_output.csv' using 1:6 with lines axes x1y2  title 'Velocity Y', \
'target/compare_rust_output.csv' using 1:7 with lines axes x1y2  title 'Velocity Z'
