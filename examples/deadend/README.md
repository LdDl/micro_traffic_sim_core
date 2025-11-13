* Debug:
    ```bash
    RUST_LOG=debug cargo run --example deadend
    ```

* Plot (using [gnuplot](http://www.gnuplot.info/))
    ```bash
    cargo run --example deadend > examples/deadend/output.txt
    gnuplot examples/deadend/plot_anim.gnuplot
    ```