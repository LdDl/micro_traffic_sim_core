* Debug:
    ```bash
    RUST_LOG=debug cargo run --example isolated
    ```

* Plot (using [gnuplot](http://www.gnuplot.info/))
    ```bash
    cargo run --example isolated > examples/isolated/output.txt
    gnuplot examples/isolated/plot_anim.gnuplot
    ```