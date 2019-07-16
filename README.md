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

## Debugging values against C++ reference implementation

1. Uncomment any `// COMP` and following `cout` lines in `trajectories-sys/Path.cpp` and `trajectories-sys/Trajectory.cpp`
1. Find a Gnuplot script with values to compare with `ls gnuplot/*.pl`
1. Run `./record_and_plot.sh ./gnuplot/<script>.plt [test_filter_string]`
1. Open `target/<script>.svg` or `target/<script>.png`

## Profiling

Inspired by <http://athemathmo.github.io/2016/09/14/tools-for-profiling-rust.html>

1. `apt install libunwind-dev graphviz`
1. Install gperftools like [here](https://github.com/AtheMathmo/cpuprofiler#installation). This installs `pprof` too.
1. Run `cargo test --release --all-features profile_native`
1. Run `pprof --web target/debug/profile_native-053b2b3c4f96d9b9 target/profiles/profile-eab52bd8a8b7be7c.profile`

OR

Install `cargo-profiler` and just use that
