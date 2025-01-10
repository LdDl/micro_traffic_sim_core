use micro_traffic_sim_core::grid::road_network::GridRoads;
use micro_traffic_sim_core::shortest_path::router::shortest_path;
use micro_traffic_sim_core::utils::generators::generate_one_lane_cells;

use criterion::{criterion_group, criterion_main, Criterion};
use std::hint::black_box;

pub fn benchmark_shortest_path(c: &mut Criterion) {
    // Generate road network
    let cells_data = generate_one_lane_cells(5000.0, 4.5, 2);
    let mut grid = GridRoads::new();

    for cell in &cells_data {
        grid.add_cell(cell.clone());
    }

    // Benchmark loop
    c.bench_function("shortest_path_a_star", |b| {
        b.iter(|| {
            let start_cell = &cells_data[0];
            let end_cell = &cells_data[cells_data.len() - 1];
            match shortest_path(
                black_box(start_cell),
                black_box(end_cell),
                black_box(&grid),
                black_box(true),
                black_box(None),
            ) {
                Ok(path) => {
                    // Do something with the path, if needed
                    let _ = path;
                }
                Err(e) => panic!("Error during shortest path calculation: {:?}", e),
            }
        })
    });
}

criterion_group!(benches, benchmark_shortest_path);
criterion_main!(benches);
