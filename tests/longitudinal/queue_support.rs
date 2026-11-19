use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::grid::road_network::GridRoads;
use micro_traffic_sim_core::simulation::grids_storage::GridsStorage;

pub fn straight_road(end: i64) -> GridsStorage {
    let mut grid = GridRoads::new();
    for i in 1..=end {
        let forward = if i < end { i + 1 } else { -1 };
        grid.add_cell(
            Cell::new(i)
                .with_point(new_point(i as f64, 0.0, None))
                .with_speed_limit(3)
                .with_forward_node(forward)
                .with_left_node(-1)
                .with_right_node(-1)
                .build(),
        );
    }
    GridsStorage::new().with_vehicles_net(grid).build()
}
