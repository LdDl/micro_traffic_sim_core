use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::{cell::Cell, road_network::GridRoads, zones::ZoneType};

fn main() {
    // Create the grid for two lanes in single direction with allowed maneuvers
    let mut grid = GridRoads::new();
    let mut cell_id = 1;

    // First lane
    // Spawn cell - where vehicles appear
    let cell = Cell::new(cell_id)
        .with_point(new_point(1.0, 1.0, None))
        .with_zone_type(ZoneType::Birth)
        .with_speed_limit(3)
        .with_forward_node(2)
        .with_left_node(7)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    cell_id += 1;

    let cell = Cell::new(cell_id)
        .with_point(new_point(2.0, 1.0, None))
        .with_zone_type(ZoneType::Common)
        .with_speed_limit(3)
        .with_forward_node(3)
        .with_left_node(8)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    cell_id += 1;

    let cell = Cell::new(cell_id)
        .with_point(new_point(3.0, 1.0, None))
        .with_zone_type(ZoneType::Common)
        .with_speed_limit(3)
        .with_forward_node(4)
        .with_left_node(9)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    cell_id += 1;

    let cell = Cell::new(cell_id)
        .with_point(new_point(4.0, 1.0, None))
        .with_zone_type(ZoneType::Common)
        .with_speed_limit(3)
        .with_forward_node(5)
        .with_left_node(10)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    cell_id += 1;

    // Cell where vehicle disappears
    let cell = Cell::new(cell_id)
        .with_point(new_point(5.0, 1.0, None))
        .with_zone_type(ZoneType::Death)
        .with_speed_limit(3)
        .with_forward_node(-1)
        .with_left_node(-1)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
    cell_id += 1;

    // Second lane
    let cell = Cell::new(cell_id)
        .with_point(new_point(1.0, 2.0, None))
        .with_zone_type(ZoneType::Birth)
        .with_speed_limit(3)
        .with_forward_node(7)
        .with_left_node(-1)
        .with_right_node(2)
        .build();
    grid.add_cell(cell);
    cell_id += 1;

    let cell = Cell::new(cell_id)
        .with_point(new_point(2.0, 2.0, None))
        .with_zone_type(ZoneType::Common)
        .with_speed_limit(3)
        .with_forward_node(8)
        .with_left_node(-1)
        .with_right_node(3)
        .build();
    grid.add_cell(cell);
    cell_id += 1;

    let cell = Cell::new(cell_id)
        .with_point(new_point(3.0, 2.0, None))
        .with_zone_type(ZoneType::Common)
        .with_speed_limit(3)
        .with_forward_node(9)
        .with_left_node(-1)
        .with_right_node(4)
        .build();
    grid.add_cell(cell);
    cell_id += 1;

    let cell = Cell::new(cell_id)
        .with_point(new_point(4.0, 2.0, None))
        .with_zone_type(ZoneType::Common)
        .with_speed_limit(3)
        .with_forward_node(10)
        .with_left_node(-1)
        .with_right_node(5)
        .build();
    grid.add_cell(cell);
    cell_id += 1;

    let cell = Cell::new(cell_id)
        .with_point(new_point(5.0, 2.0, None))
        .with_zone_type(ZoneType::Death)
        .with_speed_limit(3)
        .with_forward_node(-1)
        .with_left_node(-1)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);
}