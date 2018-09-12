# Trajectories Rust

### `trajectories-sys`

Original C++ code from <https://github.com/tobiaskunz/trajectories> with added broken Rust Bindgen config.

### Build bindings (broken)

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
