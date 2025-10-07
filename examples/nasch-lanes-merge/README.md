* Debug:
    ```bash
    RUST_LOG=debug cargo run --example nasch-lanes-merge
    ```

* Benchmark:
    ```bash
    # Build
    cargo build --release --example nasch-lanes-merge
    # Hyperfine
    hyperfine -i --shell=none --output=pipe --runs 30 --warmup 2 -n "Rust NaSch version" "./target/release/examples/nasch-lanes-merge"
    # Plain `time`
    time ./target/release/examples/nasch-lanes-merge
    ```

    ```bash
    # Hyperfine:
    Benchmark 1: Rust NaSch version
    Time (mean ± σ):      1.958 s ±  0.037 s    [User: 1.929 s, System: 0.020 s]
    Range (min … max):    1.902 s …  2.052 s    30 runs
    # time:
    ./target/release/examples/nasch-lanes-merge  1.85s user 0.02s system 99% cpu 1.881 total
    ```

* Plot (using [gnuplot](http://www.gnuplot.info/))
    ```bash
    cargo run --example nasch-lanes-merge > examples/nasch-lanes-merge/output.txt
    gnuplot examples/nasch-lanes-merge/plot_anim.gnuplot
    ```