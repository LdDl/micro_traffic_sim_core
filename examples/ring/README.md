* Debug:
    ```bash
    RUST_LOG=debug cargo run --example ring
    ```

* Plot (using [gnuplot](http://www.gnuplot.info/))
    ```bash
    cargo run --example ring > examples/ring/output.txt
    gnuplot examples/ring/plot_anim.gnuplot
    ```