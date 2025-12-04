#!/bin/bash
# Run all-tail example from any directory

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Get the project root (two levels up from examples/all-tail)
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Build and run the simulation
echo "Building and running all-tail..."
(cd "$PROJECT_ROOT" && cargo run --example all-tail)

# Generate visualizations (run from script dir so config.gnu is found)
echo "Generating animated GIF..."
(cd "$SCRIPT_DIR" && gnuplot plot.gnu)

echo "Generating static PNG..."
(cd "$SCRIPT_DIR" && gnuplot plot_static.gnu)

echo ""
echo "Done! Output files in: $SCRIPT_DIR"
echo "  - simulation.gif"
echo "  - simulation_frames.png"
