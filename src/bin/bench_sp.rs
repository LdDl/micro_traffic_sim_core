use micro_traffic_sim_core::grid::road_network::GridRoads;
use micro_traffic_sim_core::shortest_path::router::shortest_path;
use micro_traffic_sim_core::utils::generators::generate_one_lane_cells;

// cargo build --release --bin bench_sp
// hyperfine -i --shell=none --output=pipe --runs 30 --warmup 2 -n "Rust version" "./target/release/bench_sp"
pub fn main() {
    let cells_data = generate_one_lane_cells(5000.0, 4.5, 2);
    let mut grid = GridRoads::new();

    for cell in &cells_data {
        grid.add_cell(cell.clone());
    }

    for _ in 0..100 {
        let start_cell = &cells_data[0];
        let end_cell = &cells_data[cells_data.len() - 1];
        match shortest_path(start_cell, end_cell, &grid, true, None) {
            Ok(path) => {
                // Do something with the path, if needed
                let _ = path;
            }
            Err(e) => panic!("Error during shortest path calculation: {:?}", e),
        }
    }
}
