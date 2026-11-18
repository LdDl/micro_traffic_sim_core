use super::*;
use crate::geom::new_point;
use crate::grid::zones::ZoneType;
use crate::utils::generators::generate_one_lane_cells;

// Following A* parent links returns vertices and maneuvers in forward order, with the
// final node's accumulated cost.
#[test]
fn test_reconstruct_path() {
    // 1 --forward--> 33 --left--> 22
    // Parent links point backward. The returned route follows the arrows.
    let cell1 = Cell::new(1).build();
    let cell2 = Cell::new(33).build();
    let cell3 = Cell::new(22).build();

    let current_node1 = Rc::new(RefCell::new(AStarNode::new(
        &cell1,
        12.98,
        30.78,
        None,
        LaneChangeType::Undefined,
    )));

    let current_node2 = Rc::new(RefCell::new(AStarNode::new(
        &cell2,
        30.78,
        42.15,
        Some(current_node1.clone()),
        LaneChangeType::NoChange,
    )));

    let current_node3 = Rc::new(RefCell::new(AStarNode::new(
        &cell3,
        42.15,
        42.15,
        Some(current_node2.clone()),
        LaneChangeType::ChangeLeft,
    )));

    // Reconstruct the path
    let path = reconstruct_path(&current_node3);

    // Correct expected values
    let correct_vertices = vec![1, 33, 22];
    let correct_maneuvers = vec![LaneChangeType::NoChange, LaneChangeType::ChangeLeft];

    // Assertions
    assert_eq!(
        correct_vertices.len(),
        path.vertices().len(),
        "Incorrect path.vertices length"
    );
    assert_eq!(
        correct_maneuvers.len(),
        path.maneuvers().len(),
        "Incorrect path.maneuvers length"
    );

    for (i, vertex) in path.vertices().iter().enumerate() {
        assert_eq!(
            correct_vertices[i],
            vertex.get_id(),
            "Incorrect vertex at position {}",
            i
        );
    }

    for (i, maneuver) in path.maneuvers().iter().enumerate() {
        assert_eq!(
            correct_maneuvers[i], *maneuver,
            "Incorrect maneuver at position {}",
            i
        );
    }

    assert_eq!(
        current_node3.borrow().g_cost,
        path.cost(),
        "Incorrect path cost"
    );
}

// Routing across the generated two-lane road matches the reference travel-time cost,
// including one diagonal lane change.
#[test]
fn test_generated_two_lane_route_cost() {
    let cells_data = generate_one_lane_cells(5000.0, 4.5, 2);
    let mut grid = GridRoads::new();
    for cell in &cells_data {
        grid.add_cell(cell.clone());
    }
    let start_cell = grid.get_cell(&1).unwrap();
    let target_cell = grid.get_cell(&2224).unwrap();
    let path_result = shortest_path(&start_cell, &target_cell, &grid, true, None);
    assert!(
        path_result.is_ok(),
        "Pathfinding failed: {:?}",
        path_result.err()
    );
    let path = path_result.unwrap();
    // Travel-time cost is (1110 + sqrt(2)) / 3: unit forward edges,
    // one diagonal lane change, and speed limit 3 throughout the fixture.
    let correct_cost = 370.4714045207875;
    assert!(
        (path.cost() - correct_cost).abs() < 0.001,
        "Cost should be {}, but got {}",
        correct_cost,
        path.cost()
    );
}

// The two-road fixture selects the expected route, maneuver sequence, and cost from cell 7
// to cell 15.
#[test]
fn test_route_vertices_maneuvers_and_cost_across_two_roads() {
    // Expected route: 7 -> 8 -> 9 -> 10 -> 11 -> 12 -> 17 -> 14 -> 15
    // Only 17 -> 14 is a lane change. All other listed edges are forward.
    let mut grid = GridRoads::new();
    grid.add_cell(
        Cell::new(1)
            .with_point(new_point(0.0, 1.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(-1)
            .with_forward_node(2)
            .with_right_node(8)
            .with_meso_link(999)
            .build(),
    );
    grid.add_cell(
        Cell::new(2)
            .with_point(new_point(1.0, 1.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(-1)
            .with_forward_node(3)
            .with_right_node(9)
            .with_meso_link(999)
            .build(),
    );
    grid.add_cell(
        Cell::new(3)
            .with_point(new_point(2.0, 1.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(-1)
            .with_forward_node(4)
            .with_right_node(10)
            .with_meso_link(999)
            .build(),
    );
    grid.add_cell(
        Cell::new(4)
            .with_point(new_point(3.0, 1.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(-1)
            .with_forward_node(5)
            .with_right_node(11)
            .with_meso_link(999)
            .build(),
    );
    grid.add_cell(
        Cell::new(5)
            .with_point(new_point(4.0, 1.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(-1)
            .with_forward_node(6)
            .with_right_node(12)
            .with_meso_link(999)
            .build(),
    );
    grid.add_cell(
        Cell::new(6)
            .with_point(new_point(5.0, 1.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(-1)
            .with_forward_node(13)
            .with_right_node(17)
            .with_meso_link(999)
            .build(),
    );
    grid.add_cell(
        Cell::new(7)
            .with_point(new_point(0.0, 0.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(2)
            .with_forward_node(8)
            .with_right_node(-1)
            .with_meso_link(999)
            .build(),
    );
    grid.add_cell(
        Cell::new(8)
            .with_point(new_point(1.0, 0.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(3)
            .with_forward_node(9)
            .with_right_node(-1)
            .with_meso_link(999)
            .build(),
    );
    grid.add_cell(
        Cell::new(9)
            .with_point(new_point(2.0, 0.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(4)
            .with_forward_node(10)
            .with_right_node(-1)
            .with_meso_link(999)
            .build(),
    );
    grid.add_cell(
        Cell::new(10)
            .with_point(new_point(3.0, 0.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(5)
            .with_forward_node(11)
            .with_right_node(-1)
            .with_meso_link(999)
            .build(),
    );
    grid.add_cell(
        Cell::new(11)
            .with_point(new_point(4.0, 0.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(6)
            .with_forward_node(12)
            .with_right_node(-1)
            .with_meso_link(999)
            .build(),
    );
    grid.add_cell(
        Cell::new(12)
            .with_point(new_point(5.0, 0.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(13)
            .with_forward_node(17)
            .with_right_node(-1)
            .with_meso_link(999)
            .build(),
    );

    grid.add_cell(
        Cell::new(17)
            .with_point(new_point(12.0, -1.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(14)
            .with_forward_node(18)
            .with_right_node(-1)
            .with_meso_link(100500)
            .build(),
    );
    grid.add_cell(
        Cell::new(18)
            .with_point(new_point(12.0, -2.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(15)
            .with_forward_node(19)
            .with_right_node(-1)
            .with_meso_link(100500)
            .build(),
    );
    grid.add_cell(
        Cell::new(19)
            .with_point(new_point(12.0, -3.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(16)
            .with_forward_node(20)
            .with_right_node(-1)
            .with_meso_link(100500)
            .build(),
    );
    grid.add_cell(
        Cell::new(20)
            .with_point(new_point(12.0, -4.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(-1)
            .with_forward_node(-1)
            .with_right_node(-1)
            .with_meso_link(100500)
            .build(),
    );

    grid.add_cell(
        Cell::new(13)
            .with_point(new_point(13.0, -1.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(-1)
            .with_forward_node(14)
            .with_right_node(18)
            .with_meso_link(100500)
            .build(),
    );
    grid.add_cell(
        Cell::new(14)
            .with_point(new_point(13.0, -2.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(-1)
            .with_forward_node(15)
            .with_right_node(19)
            .with_meso_link(100500)
            .build(),
    );
    grid.add_cell(
        Cell::new(15)
            .with_point(new_point(13.0, -3.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(-1)
            .with_forward_node(16)
            .with_right_node(20)
            .with_meso_link(100500)
            .build(),
    );
    grid.add_cell(
        Cell::new(16)
            .with_point(new_point(13.0, -4.0, None))
            .with_zone_type(ZoneType::Common)
            .with_speed_limit(1)
            .with_left_node(-1)
            .with_forward_node(-1)
            .with_right_node(-1)
            .with_meso_link(100500)
            .build(),
    );

    // Run the shortest path algorithm for a specific source and target
    let start_cell = grid.get_cell(&7).unwrap();
    let target_cell = grid.get_cell(&15).unwrap();
    let path_result = shortest_path(&start_cell, &target_cell, &grid, true, None);
    assert!(
        path_result.is_ok(),
        "Pathfinding failed: {:?}",
        path_result.err()
    );
    let path = path_result.unwrap();

    let correct_cost = 14.485281374238571;
    let correct_vertices = vec![7, 8, 9, 10, 11, 12, 17, 14, 15];
    let correct_maneuvers = vec![
        LaneChangeType::NoChange,
        LaneChangeType::NoChange,
        LaneChangeType::NoChange,
        LaneChangeType::NoChange,
        LaneChangeType::NoChange,
        LaneChangeType::NoChange,
        LaneChangeType::ChangeLeft,
        LaneChangeType::NoChange,
    ];

    assert!(
        (path.cost() - correct_cost).abs() < 0.001,
        "Cost should be {}, but got {}",
        correct_cost,
        path.cost()
    );

    assert_eq!(
        correct_vertices.len(),
        path.vertices().len(),
        "Incorrect path.vertices length"
    );

    assert_eq!(
        correct_maneuvers.len(),
        path.maneuvers().len(),
        "Incorrect path.maneuvers length"
    );

    for (i, vertex) in path.vertices().iter().enumerate() {
        assert_eq!(
            correct_vertices[i],
            vertex.get_id(),
            "Incorrect vertex at position {}",
            i
        );
    }

    for (i, maneuver) in path.maneuvers().iter().enumerate() {
        assert_eq!(
            correct_maneuvers[i], *maneuver,
            "Incorrect maneuver at position {}",
            i
        );
    }
}

// A five-step forward traversal wraps around a four-cell directed ring and includes the
// starting cell.
#[test]
fn test_forward_path_wraps_around_ring() {
    use crate::geom::new_point;
    use crate::grid::cell::Cell;
    use crate::grid::road_network::GridRoads;
    // Directed ring: 1, 2, 3, 4, then back to 1.
    let mut grid = GridRoads::new();
    for i in 1..=4 {
        let next = if i == 4 { 1 } else { i + 1 };
        grid.add_cell(
            Cell::new(i)
                .with_point(new_point(i as f64, 0.0, None))
                .with_forward_node(next)
                .build(),
        );
    }
    let start_cell = grid.get_cell(&1).unwrap();
    // Traverse 5 steps forward (should wrap around)
    let observe = 5;
    let path_opt = path_no_goal(start_cell, &grid, false, observe);
    assert!(path_opt.is_ok(), "Path should exist");
    let path = path_opt.unwrap();
    let ids: Vec<i64> = path.vertices().iter().map(|c| c.get_id()).collect();
    // step: 0 1 2 3 4 5
    // cell: 1 2 3 4 1 2
    // Five forward steps wrap around the four-cell ring once.
    assert_eq!(
        ids,
        vec![1, 2, 3, 4, 1, 2],
        "Incorrect circular path traversal"
    );
}
