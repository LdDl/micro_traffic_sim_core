use crate::{
    geom::new_point,
    grid::{cell::Cell, road_network::GridRoads, zones::ZoneType},
};

/// Creates a simple grid for testing purposes with complex lane change patterns.
///
/// This grid provides a test case with multiple lanes, merging points,
/// and various connection types for complex behavior testing.
///
/// # Usage
/// 
/// ```rust
/// use micro_traffic_sim_core::utils::test_grids::create_pretty_simple_grid;
/// 
/// let grid = create_pretty_simple_grid();
/// println!("Test grid has {} cells", grid.get_cells_num());
/// ```
///
/// # Example
/// ```text
/// .         (44) ---(55) ---(66)
/// .            \   /       /
/// .             \ /       /
/// .              |       |
/// .            /   \    /
/// .           /     \  /
/// .         (4) ----(5) ----(6)
/// .            \   /   \   /   \
/// .             \ /     \ /     \ Forward
/// .              |       |      (7)
/// .            /   \    / \     / Forward
/// .           /     \  /   \   /
/// . (101) --(1) ----(2) ----(3)
/// .                             \
/// .                              (8)
/// ```
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

/// Creates a conflict zones grid for testing purposes
///
/// # Usage
/// 
/// ```rust
/// use micro_traffic_sim_core::utils::test_grids::create_conflict_zones_grid;
/// 
/// let grid = create_conflict_zones_grid();
/// println!("Conflict zones grid has {} cells", grid.get_cells_num());
/// ```
/// 
/// # Example
/// ```
/// // .  (7)
/// // .     \
/// // .      \               ---(6)
/// // .      (8)            /
/// // .        \           (5)
/// // .         \         /
/// // .         (9)      /
/// // .           \   (4)
/// // .           ---/
/// // .         /   \
/// // .       (3)   (10)
/// // .       /      \
/// // .      /        \
/// // .    (2)        (11)
/// // .     |          \
/// // .    /            \
/// // .  (1)            (12)
/// // .
/// ```
pub fn create_conflict_zones_grid() -> GridRoads {
    let mut grid = GridRoads::new();

    grid.add_cell(
        Cell::new(1)
            .with_point(new_point(22.0, 16.0, None))
            .with_zone_type(ZoneType::Birth)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(2)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(2)
            .with_point(new_point(36.0, 51.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(3)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(3)
            .with_point(new_point(52.0, 67.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(4)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(4)
            .with_point(new_point(77.0, 81.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(5)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(5)
            .with_point(new_point(98.0, 90.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(6)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(6)
            .with_point(new_point(125.0, 100.0, None))
            .with_zone_type(ZoneType::Death)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(-1)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(7)
            .with_point(new_point(17.0, 125.0, None))
            .with_zone_type(ZoneType::Birth)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(8)
            .with_right_node(-1)
            .with_meso_link(2)
            .build(),
    );

    grid.add_cell(
        Cell::new(8)
            .with_point(new_point(38.0, 108.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(9)
            .with_right_node(-1)
            .with_meso_link(2)
            .build(),
    );

    grid.add_cell(
        Cell::new(9)
            .with_point(new_point(58.0, 88.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(10)
            .with_right_node(-1)
            .with_meso_link(2)
            .build(),
    );

    grid.add_cell(
        Cell::new(10)
            .with_point(new_point(75.0, 63.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(11)
            .with_right_node(-1)
            .with_meso_link(2)
            .build(),
    );

    grid.add_cell(
        Cell::new(11)
            .with_point(new_point(89.0, 35.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(12)
            .with_right_node(-1)
            .with_meso_link(2)
            .build(),
    );

    grid.add_cell(
        Cell::new(12)
            .with_point(new_point(98.0, 8.0, None))
            .with_zone_type(ZoneType::Death)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(-1)
            .with_right_node(-1)
            .with_meso_link(2)
            .build(),
    );

    grid
}

/// Creates a conflict zones multiple grid for testing purposes
///
/// # Usage
/// 
/// ```rust
/// use micro_traffic_sim_core::utils::test_grids::create_conflict_zones_multiple_grid;
/// 
/// let grid = create_conflict_zones_multiple_grid();
/// println!("Conflict zones multiple grid has {} cells", grid.get_cells_num());
/// ```
/// 
/// # Example
/// ```
/// // .  (7)
/// // .     \
/// // .      \               ---(6)
/// // .      (8)            /
/// // . (13)   \           (5)
/// // .    |    \         /
/// // .    (14)  (9)      /
/// // .       \   \   (4)
/// // .        ---___/
/// // .         /   \
/// // .       (3)   (10)
/// // .       /      \
/// // .      /        \
/// // .    (2)        (11)
/// // .     |          \
/// // .    /            \
/// // .  (1)            (12)
/// // .
/// ```
///
pub fn create_conflict_zones_multiple_grid() -> GridRoads {
    let mut grid = GridRoads::new();

    grid.add_cell(
        Cell::new(1)
            .with_point(new_point(22.0, 16.0, None))
            .with_zone_type(ZoneType::Birth)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(2)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(2)
            .with_point(new_point(36.0, 51.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(3)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(3)
            .with_point(new_point(52.0, 67.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(4)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(4)
            .with_point(new_point(77.0, 81.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(5)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(5)
            .with_point(new_point(98.0, 90.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(6)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(6)
            .with_point(new_point(125.0, 100.0, None))
            .with_zone_type(ZoneType::Death)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(-1)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(7)
            .with_point(new_point(17.0, 125.0, None))
            .with_zone_type(ZoneType::Birth)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(8)
            .with_right_node(-1)
            .with_meso_link(2)
            .build(),
    );

    grid.add_cell(
        Cell::new(8)
            .with_point(new_point(38.0, 108.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(9)
            .with_right_node(-1)
            .with_meso_link(2)
            .build(),
    );

    // Different from conflictZonesGrid: cell 9 has right_node(10) instead of forward_node(10)
    grid.add_cell(
        Cell::new(9)
            .with_point(new_point(58.0, 88.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(-1)
            .with_right_node(10)
            .with_meso_link(2)
            .build(),
    );

    // Different from conflictZonesGrid: cell 10 has meso_link(3) instead of meso_link(2)
    grid.add_cell(
        Cell::new(10)
            .with_point(new_point(75.0, 63.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(11)
            .with_right_node(-1)
            .with_meso_link(3)
            .build(),
    );

    grid.add_cell(
        Cell::new(11)
            .with_point(new_point(89.0, 35.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(12)
            .with_right_node(-1)
            .with_meso_link(3)
            .build(),
    );

    grid.add_cell(
        Cell::new(12)
            .with_point(new_point(98.0, 8.0, None))
            .with_zone_type(ZoneType::Death)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(-1)
            .with_right_node(-1)
            .with_meso_link(3)
            .build(),
    );

    grid.add_cell(
        Cell::new(13)
            .with_point(new_point(48.0, 78.0, None))
            .with_zone_type(ZoneType::Birth)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(14)
            .with_right_node(-1)
            .with_meso_link(4)
            .build(),
    );

    grid.add_cell(
        Cell::new(14)
            .with_point(new_point(58.0, 85.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(10)
            .with_right_node(-1)
            .with_meso_link(4)
            .build(),
    );

    grid
}

/// Creates a simple cross shape grid for testing purposes
///
/// # Usage
/// 
/// ```rust
/// use micro_traffic_sim_core::utils::test_grids::create_simple_cross_shape_grid;
/// 
/// let grid = create_simple_cross_shape_grid();
/// println!("Simple cross shape grid has {} cells", grid.get_cells_num());
/// ```
/// 
/// # Example
/// ```
/// // . (1)             (9)
/// // .    \           /
/// // .     \         /
/// // .    (2)      (8)
/// // .       \     /
/// // .         (3)
/// // .       /     \
/// // .     (7)     (4)
/// // .     /         \
/// // .    /           \
/// // . (6)            (5)
/// ```
///
pub fn create_simple_cross_shape_grid() -> GridRoads {
    let mut grid = GridRoads::new();

    grid.add_cell(
        Cell::new(1)
            .with_point(new_point(1.0, 5.0, None))
            .with_zone_type(ZoneType::Birth)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(2)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(2)
            .with_point(new_point(2.0, 4.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(3)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(3)
            .with_point(new_point(3.0, 3.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(8)
            .with_forward_node(-1)
            .with_right_node(4)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(4)
            .with_point(new_point(4.0, 2.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(5)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(5)
            .with_point(new_point(5.0, 1.0, None))
            .with_zone_type(ZoneType::Death)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(-1)
            .with_right_node(-1)
            .with_meso_link(1)
            .build(),
    );

    grid.add_cell(
        Cell::new(6)
            .with_point(new_point(1.0, 1.0, None))
            .with_zone_type(ZoneType::Birth)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(7)
            .with_right_node(-1)
            .with_meso_link(2)
            .build(),
    );

    grid.add_cell(
        Cell::new(7)
            .with_point(new_point(2.0, 2.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(3)
            .with_right_node(-1)
            .with_meso_link(2)
            .build(),
    );

    grid.add_cell(
        Cell::new(8)
            .with_point(new_point(4.0, 4.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(9)
            .with_right_node(-1)
            .with_meso_link(3)
            .build(),
    );

    grid.add_cell(
        Cell::new(9)
            .with_point(new_point(5.0, 5.0, None))
            .with_zone_type(ZoneType::Death)
            .with_speed_limit(3)
            .with_left_node(-1)
            .with_forward_node(-1)
            .with_right_node(-1)
            .with_meso_link(3)
            .build(),
    );

    grid
}