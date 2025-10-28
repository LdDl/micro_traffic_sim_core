* Debug:
    ```bash
    RUST_LOG=debug cargo run --example tutorial
    ```

* Plot (using [gnuplot](http://www.gnuplot.info/))
    ```bash
    cargo run --example tutorial > examples/tutorial/output.txt
    gnuplot examples/tutorial/plot_anim.gnuplot
    ```