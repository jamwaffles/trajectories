# Trajectories Rust

### `trajectories-sys`

Original C++ code from <https://github.com/tobiaskunz/trajectories> with added Rust Bindgen config.

### Build bindings

```bash
# macOS
brew install eigen

# Linux
sudo apt-get install -y libeigen3-dev libclang-6.0-dev

cargo Build
```

### View graph

```bash
cargo build
dot -Tpng "./bindings.dot" -o "./bindings.png"
```

### Build example and tests

```bash
# macOS
brew install eigen

# Linux
sudo apt-get install -y libeigen3-dev libclang-6.0-dev

g++ -I/usr/local/Cellar/eigen/3.3.4/include/eigen3 -I/usr/include/eigen3 Example.cpp Trajectory.cpp Path.cpp -o example -O3
./example > out-example.txt

g++ -I/usr/local/Cellar/eigen/3.3.4/include/eigen3 -I/usr/include/eigen3 Test.cpp Trajectory.cpp Path.cpp -o test -O3
./test
```

## TODO

* [x] Prepare path; introduce circular blends between straight segments, shorten straight segments

## Debugging values against C++ reference implementation

1. Uncomment any `// COMP` and following `cout` lines in `trajectories-sys/Path.cpp` and `trajectories-sys/Trajectory.cpp`
1. Find a Gnuplot script with values to compare with `ls gnuplot/*.pl`
1. Run `./record_and_plot.sh ./gnuplot/<script>.plt`
1. Open `target/<script>.svg` or `target/<script>.png`
