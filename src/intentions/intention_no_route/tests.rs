use super::*;
use crate::grid::cell::Cell;
use crate::grid::road_network::GridRoads;

// A cell with no forward or lateral exits returns DeadEndReached carrying that cell's ID.
#[test]
fn test_process_no_route_found_deadend() {
    // Create a cell with no exits
    let cell = Cell::new(1)
        .with_forward_node(-1)
        .with_right_node(-1)
        .with_left_node(-1)
        .build();
    let mut net = GridRoads::new();
    net.add_cell(cell.clone());

    // Should return DeadEndReached error
    let result = process_no_route_found(&cell, &net);
    match result {
        Err(NoRouteError::DeadEndReached { cell_id }) => {
            assert_eq!(cell_id, cell.get_id());
        }
        other => panic!("Expected DeadEndReached error, got {:?}", other),
    }
}
