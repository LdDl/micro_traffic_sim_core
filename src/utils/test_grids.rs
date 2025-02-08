use crate::{
    geom::new_point,
    grid::{cell::Cell, road_network::GridRoads, zones::ZoneType},
};

/// Creates a simple grid for testing purposes
///
/// # Example
/// ```
/// // .         (44) ---(55) ---(66)
/// // .            \   /       /
/// // .             \ /       /
/// // .              |       |
/// // .            /   \    /
/// // .           /     \  /
/// // .         (4) ----(5) ----(6)
/// // .            \   /   \   /   \
/// // .             \ /     \ /     \ Forward
/// // .              |       |      (7)
/// // .            /   \    / \     / Forward
/// // .           /     \  /   \   /
/// // . (101) --(1) ----(2) ----(3)
/// // .                             \
/// // .                              (8)
/// ```
///
pub fn create_pretty_simple_grid() -> GridRoads {
    let mut grid = GridRoads::new();

    // Add cells to grid
    grid.add_cell(
        Cell::new(101)
            .with_point(new_point(0.0, 1.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(4)
            .with_left_node(-1)
            .with_forward_node(1)
            .with_right_node(-1)
            .with_meso_link(999)
            .build(),
    );

    grid.add_cell(
        Cell::new(1)
            .with_point(new_point(1.0, 1.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(4)
            .with_left_node(5)
            .with_forward_node(2)
            .with_right_node(-1)
            .with_meso_link(999)
            .build(),
    );

    grid.add_cell(
        Cell::new(2)
            .with_point(new_point(2.0, 1.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(4)
            .with_left_node(6)
            .with_forward_node(3)
            .with_right_node(-1)
            .with_meso_link(999)
            .build(),
    );

    grid.add_cell(
        Cell::new(3)
            .with_point(new_point(3.0, 1.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(4)
            .with_left_node(-1)
            .with_forward_node(7)
            .with_right_node(8)
            .with_meso_link(999)
            .build(),
    );

    grid.add_cell(
        Cell::new(4)
            .with_point(new_point(1.0, 2.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(4)
            .with_left_node(55)
            .with_forward_node(5)
            .with_right_node(2)
            .with_meso_link(999)
            .build(),
    );

    grid.add_cell(
        Cell::new(5)
            .with_point(new_point(2.0, 2.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(4)
            .with_left_node(66)
            .with_forward_node(6)
            .with_right_node(3)
            .with_meso_link(999)
            .build(),
    );

    grid.add_cell(
        Cell::new(6)
            .with_point(new_point(3.0, 2.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(4)
            .with_left_node(-1)
            .with_forward_node(7)
            .with_right_node(-1)
            .with_meso_link(999)
            .build(),
    );

    grid.add_cell(
        Cell::new(7)
            .with_point(new_point(4.0, 1.5, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(4)
            .with_left_node(-1)
            .with_forward_node(-1)
            .with_right_node(-1)
            .with_meso_link(999)
            .build(),
    );

    grid.add_cell(
        Cell::new(8)
            .with_point(new_point(5.0, 0.5, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(4)
            .with_left_node(-1)
            .with_forward_node(-1)
            .with_right_node(-1)
            .with_meso_link(999)
            .build(),
    );

    grid.add_cell(
        Cell::new(44)
            .with_point(new_point(1.0, 3.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(4)
            .with_left_node(-1)
            .with_forward_node(55)
            .with_right_node(5)
            .with_meso_link(999)
            .build(),
    );

    grid.add_cell(
        Cell::new(55)
            .with_point(new_point(2.0, 3.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(4)
            .with_left_node(-1)
            .with_forward_node(66)
            .with_right_node(-1)
            .with_meso_link(999)
            .build(),
    );

    grid.add_cell(
        Cell::new(66)
            .with_point(new_point(3.0, 3.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(4)
            .with_left_node(-1)
            .with_forward_node(-1)
            .with_right_node(-1)
            .with_meso_link(999)
            .build(),
    );

    grid
}