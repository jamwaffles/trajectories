# Produce test data with `cargo test --release compare_test_2`

set key autotitle columnhead
set datafile separator ","

set term svg size 2500,1300
set timestamp

set output 'target/overlay_test_2_native_cpp.svg'

set multiplot layout 2,1 \
              title "Overlay C++ and Rust output of test2() in Test.cpp"

set title 'Position overlay'

load 'gnuplot/paired.pal'

do for [i=1:6] {
    set style line i linewidth 2
}

plot 'target/compare_rust_output.csv' using 1:2 ls 1 with lines title 'Position Rust X', \
'target/compare_rust_output.csv' using 1:3 ls 3 with lines title 'Position Rust Y', \
'target/compare_rust_output.csv' using 1:4 ls 5 with lines title 'Position Rust Z', \
'target/compare_cpp_output.csv' using 1:2 ls 2 dt 4 with lines title 'Position C++ X', \
'target/compare_cpp_output.csv' using 1:3 ls 4 dt 4 with lines title 'Position C++ Y', \
'target/compare_cpp_output.csv' using 1:4 ls 6 dt 4 with lines title 'Position C++ Z'


set title 'Velocity overlay'

set grid x2tics
set x2tics (50, 140.23806829245765, 380.71934373703334, 879.8507114774787, 1084.8016572321708, 1163.3414735719157)

plot 'target/compare_rust_output.csv' using 1:5 ls 1 with lines axes x1y2  title 'Velocity Rust X', \
'target/compare_rust_output.csv' using 1:6 ls 3 with lines axes x1y2  title 'Velocity Rust Y', \
'target/compare_rust_output.csv' using 1:7 ls 5 with lines axes x1y2  title 'Velocity Rust Z' , \
'target/compare_cpp_output.csv' using 1:5 ls 2 dt 4 with lines axes x1y2 title 'Velocity C++ X', \
'target/compare_cpp_output.csv' using 1:6 ls 4 dt 4 with lines axes x1y2 title 'Velocity C++ Y', \
'target/compare_cpp_output.csv' using 1:7 ls 6 dt 4 with lines axes x1y2 title 'Velocity C++ Z'
