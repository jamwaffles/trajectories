#!/bin/sh

# Compare C++ code with Rust bindings

cargo test --release
cargo test --release -- --ignored

echo "C++ native..."
cd ./trajectories-sys
g++ -I/usr/local/Cellar/eigen/3.3.4/include/eigen3 -I/usr/include/eigen3 Example.cpp Trajectory.cpp Path.cpp -o example && ./example > ../target/plot_cpp_native.csv
cd ..
echo "Done"

echo "Plotting..."
gnuplot compare.plt
echo "Done"
