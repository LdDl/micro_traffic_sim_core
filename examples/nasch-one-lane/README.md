* Debug:
    ```bash
    RUST_LOG=debug cargo run --example nasch-one-lane
    ```

* Benchmark:
    ```bash
    # Build
    cargo build --release --example nasch-one-lane
    # Hyperfine
    hyperfine -i --shell=none --output=pipe --runs 30 --warmup 2 -n "Rust NaSch version" "./target/release/examples/nasch-one-lane"
    # Plain `time`
    time ./target/release/examples/nasch-one-lane
    ```

    ```bash
    # Hyperfine:
    Benchmark 1: Rust NaSch version
    Time (mean ± σ):      1.958 s ±  0.037 s    [User: 1.929 s, System: 0.020 s]
    Range (min … max):    1.902 s …  2.052 s    30 runs
    # time:
    ./target/release/examples/nasch-one-lane  1.85s user 0.02s system 99% cpu 1.881 total
    ```