# Trajectories Rust

## Build bindings (broken)

```bash
brew install eigen
cargo Build
```

## View graph

```bash
cargo build
dot -Tpng "./bindings.dot" -o "./bindings.png"
```

## Build example

```bash
g++ -I/usr/local/Cellar/eigen/3.3.4/include/eigen3 Example.cpp Trajectory.cpp Path.cpp -o example -O3
./example > out-example.txt

g++ -I/usr/local/Cellar/eigen/3.3.4/include/eigen3 Test.cpp Trajectory.cpp Path.cpp -o test -O3
./test
```
