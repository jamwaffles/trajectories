# Trajectories Rust

## Build bindings

```bash
brew install eigen
cargo Build
```

## View graph

```bash
cargo build
dot -Tpng "./bindings.dot" -o "./bindings.png"
```
