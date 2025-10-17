# micro_traffic_sim_core — core for traffic simulation via cellular automata

Under development. API may change.

I've not even published this crate on crates.io yet.

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
- Geometry and spatial utilities:
  - Point - [`src/geom/point.rs`](src/geom/point.rs)
  - Utilities - [`src/geom/spatial.rs`](src/geom/spatial.rs)
- Grid (road network graph):
  - Cell unit - [`src/grid/cell.rs`](src/grid/cell.rs)
  - Road network - [`src/grid/road_network.rs`](src/grid/road_network.rs)
- Pathfinding:
  - A* star for grid-based road network graph - [`src/shortest_path/router.rs`](src/shortest_path/router.rs)
- Traffic lights and signals:
  - Signals - [`src/traffic_lights/signals.rs`](src/traffic_lights/signals.rs)
  - Signal groups (in context of single junction) - [`src/traffic_lights/groups.rs`](src/traffic_lights/groups.rs)
  - Traffic light (as single junction) - [`src/traffic_lights/lights.rs`](src/traffic_lights/lights.rs)
- Agents and related functionality:
  - Agents' behaviour types - [`src/agents/behaviour.rs`](src/agents/behaviour.rs)
  - Vehicle agents - [`src/agents/vehicle.rs`](src/agents/vehicle.rs)
- Trips
  - Trips generation - [`src/trips/trip.rs`](src/trips/trip.rs)
- Intentions:
    - Main utilities - [`src/intentions/intention.rs`](src/intentions/intention.rs)
    - Intentions storage - [`src/intentions/intentions_datastorage.rs`](src/intentions/intentions_datastorage.rs)
    - Intentions paths processing - [`src/intentions/intention_path.rs`](src/intentions/intention_path.rs)
- Conflicts & solvers:
  - Conflicts - [`src/conflicts`](src/conflicts)
  - Conflict zones - [`src/conflict_zones/conflict_zones.rs`](src/conflict_zones/conflict_zones.rs)
- Movement
    - Basic movement - [`src/movement/movement.rs`](src/movement/movement.rs)
- Simulation session and runtime:
  - Grids storage (if future we can have multiple type of grids: for vehicles and for pedestrians) - [`src/simulation/grids_storage.rs`](src/simulation/grids_storage.rs)
  - Simulation session and steps - [`src/simulation/session.rs`](src/simulation/session.rs)