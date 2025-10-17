# micro_traffic_sim_core — core for traffic simulation via cellular automata

Table of contents
- [Introduction](#introduction)
- [Project layout](#project-layout)
- [Quick start (Rust)](#quick-start-rust)
  - [Run example: NaSch one-lane](#run-example-nasch-one-lane)
  - [Build release binary for example](#build-release-binary-for-example)
  - [Benchmark with hyperfine](#benchmark-with-hyperfine)
- [Key modules / API pointers](#key-modules--api-pointers)

## Introduction

`micro_traffic_sim_core` is a Rust core library for microscopic traffic simulation (NaSch-like cellular automata and agent-based behaviours). The codebase contains utilities for grids, agents, intentions, conflicts, pathfinding, and session simulation.

## Project layout

Top-level modules are exported from [`src/lib.rs`](src/lib.rs).

Examples live under the `examples/` directory. Notable examples:
- [`examples/nasch-one-lane/main.rs`](examples/nasch-one-lane/main.rs) — minimal one-lane NaSch example
- [`examples/nasch-two-lanes/main.rs`](examples/nasch-two-lanes/main.rs)
- [`examples/nasch-lanes-merge/main.rs`](examples/nasch-lanes-merge/main.rs)
- [`examples/nasch-roads-merge/main.rs`](examples/nasch-roads-merge/main.rs)

Benchmarks:
- [`benches/shortest_path_benchmark.rs`](benches/shortest_path_benchmark.rs)

## Quick start (Rust)

Prerequisites
- Rust toolchain (`rustup` + `cargo`)
- Optionally: `hyperfine` for repeated benchmarking

### Run example: NaSch one-lane

Build and run an example (prints CSV-like step output).

* Run directly (debug):
    ```sh
    cargo run --example nasch-one-lane
    ```

* Capture output to a file (useful for plotting):
    ```sh
    cargo run --example nasch-one-lane > examples/nasch-one-lane/output.txt
    ```

### Build release binary for the example:
```sh
cargo build --release --example nasch-one-lane
./target/release/examples/nasch-one-lane
```

### Benchmark with hyperfine
```sh
hyperfine -i --shell=none --output=pipe --runs 30 --warmup 2 -n "Rust NaSch version" "./target/release/examples/nasch-one-lane"
```

## Key modules / API pointers

- Library entry: [`src/lib.rs`](src/lib.rs)
- Simulation session and runtime:
  - [`src/simulation/session.rs`](src/simulation/session.rs) — `simulation::session::Session`
  - [`src/simulation/grids_storage.rs`](src/simulation/grids_storage.rs) — `simulation::grids_storage::GridsStorage`
- Grid and geometry:
  - [`src/grid/cell.rs`](src/grid/cell.rs) — `grid::cell::Cell`
  - [`src/grid/road_network.rs`](src/grid/road_network.rs) — `grid::road_network::GridRoads`
  - [`src/geom/point.rs`](src/geom/point.rs) — `geom::new_point`
- Agents, intentions, behaviour:
  - [`src/agents/vehicle.rs`](src/agents/vehicle.rs) — `agents::Vehicle`
  - Intentions module: [`src/intentions/mod.rs`](src/intentions/mod.rs)
- Conflicts & solver:
  - [`src/conflicts/conflicts.rs`](src/conflicts/conflicts.rs)
  - Conflict zones: [`src/conflict_zones/`](src/conflict_zones/)
- Pathfinding:
  - [`src/shortest_path/router.rs`](src/shortest_path/router.rs) — `shortest_path::router::shortest_path`
