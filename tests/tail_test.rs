use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::{cell::Cell, road_network::GridRoads};
use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::behaviour::BehaviourType;
use micro_traffic_sim_core::simulation::session::Session;
use micro_traffic_sim_core::simulation::grids_storage::GridsStorage;
use micro_traffic_sim_core::grid::cell::CellID;

/// Expected state represents expected vehicle position at a given step
struct ExpectedState {
    head_cell: CellID,
    tail_cells: Vec<CellID>,
}

/// TestGnuplotTail tests a multi-cell vehicle navigating an L-shaped road with a right turn.
/// Road layout:
/// [1]-[2]-[3]-[4]-[5]-[6]-[7]-[8]-[9]-[10]
///                                        \
///                                        [11]-[12]-[13]-[14]-[15]-[16]-[17]-[18]-[19]-[20]
#[test]
fn test_gnuplot_tail() {
    let mut grid = GridRoads::new();

    // First row (y=1): cells 1-10
    for i in 1..=10 {
        let forward = if i < 10 { i + 1 } else { -1 };
        let right = if i == 10 { 11 } else { -1 };
        let cell = Cell::new(i)
            .with_point(new_point(i as f64, 1.0, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(right)
            .build();
        grid.add_cell(cell);
    }

    // Second row (y=0): cells 11-20
    for i in 11..=20 {
        let forward = if i < 20 { i + 1 } else { -1 };
        let cell = Cell::new(i)
            .with_point(new_point(i as f64, 0.0, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(-1)
            .build();
        grid.add_cell(cell);
    }

    // Create grids storage and session
    let grids_storage = GridsStorage::new().with_vehicles_net(grid).build();
    let mut session = Session::new(grids_storage, None);

    // Create vehicle at cell 3 with tail size 2
    let vehicles = vec![
        Vehicle::new(1)
            .with_cell(3)
            .with_tail_size(2, vec![1, 2])
            .with_destination(20)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
    ];
    session.add_vehicles(vehicles);

    // Expected states for each step
    let expected_states = vec![
        ExpectedState { head_cell: 4, tail_cells: vec![2, 3] },
        ExpectedState { head_cell: 5, tail_cells: vec![3, 4] },
        ExpectedState { head_cell: 6, tail_cells: vec![4, 5] },
        ExpectedState { head_cell: 7, tail_cells: vec![5, 6] },
        ExpectedState { head_cell: 8, tail_cells: vec![6, 7] },
        ExpectedState { head_cell: 9, tail_cells: vec![7, 8] },
        ExpectedState { head_cell: 10, tail_cells: vec![8, 9] },
        ExpectedState { head_cell: 11, tail_cells: vec![9, 10] },
        ExpectedState { head_cell: 12, tail_cells: vec![10, 11] },
        ExpectedState { head_cell: 13, tail_cells: vec![11, 12] },
        ExpectedState { head_cell: 14, tail_cells: vec![12, 13] },
        ExpectedState { head_cell: 15, tail_cells: vec![13, 14] },
        ExpectedState { head_cell: 16, tail_cells: vec![14, 15] },
        ExpectedState { head_cell: 17, tail_cells: vec![15, 16] },
        ExpectedState { head_cell: 18, tail_cells: vec![16, 17] },
        ExpectedState { head_cell: 19, tail_cells: vec![17, 18] },
        // Vehicle reaches destination at step 16 and is removed
    ];

    // Run simulation
    let time_sim = 30;
    let mut vehicle_reached_destination = false;

    for step in 0..time_sim {
        match session.step() {
            Ok(state) => {
                // Check if vehicle reached destination (no more vehicles in simulation)
                if state.vehicles.is_empty() {
                    vehicle_reached_destination = true;
                } else if step < expected_states.len() {
                    let vehicle_state = &state.vehicles[0];
                    let expected = &expected_states[step];

                    // Assert head position
                    assert_eq!(
                        vehicle_state.last_cell, expected.head_cell,
                        "Step {}: expected head at cell {}, got {}",
                        step, expected.head_cell, vehicle_state.last_cell
                    );

                    // Assert tail positions
                    assert_eq!(
                        vehicle_state.tail_cells.len(), expected.tail_cells.len(),
                        "Step {}: expected tail length {}, got {}",
                        step, expected.tail_cells.len(), vehicle_state.tail_cells.len()
                    );
                    for (i, expected_tail) in expected.tail_cells.iter().enumerate() {
                        assert_eq!(
                            vehicle_state.tail_cells[i], *expected_tail,
                            "Step {}: expected tail[{}] = {}, got {}",
                            step, i, expected_tail, vehicle_state.tail_cells[i]
                        );
                    }
                }
            }
            Err(e) => panic!("Error at step {}: {}", step, e),
        }
    }

    assert!(vehicle_reached_destination, "Vehicle did not reach destination within {} steps", time_sim);
}

/// TestPingpongTail tests a multi-cell vehicle navigating a zigzag road with right-then-left turns.
/// This test verifies that the vehicle correctly blocks when maneuvers are not allowed due to tail.
/// Road layout:
/// [1]-[2]-[3]-[4]-[5]-[6]-[7]-[8]-[9]-[10]    [12]-[13]-[14]-[15]-[16]
///                                        \   /
///                                        [11]
#[test]
fn test_pingpong_tail() {
    let mut grid = GridRoads::new();

    // First row (y=1): cells 1-10
    for i in 1..=10 {
        let forward = if i < 10 { i + 1 } else { -1 };
        let right = if i == 10 { 11 } else { -1 };
        let cell = Cell::new(i)
            .with_point(new_point(i as f64, 1.0, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(right)
            .build();
        grid.add_cell(cell);
    }

    // Cell 11 (y=0.5): transition cell with LEFT to 12
    let cell = Cell::new(11)
        .with_point(new_point(11.0, 0.5, None))
        .with_speed_limit(3)
        .with_forward_node(-1)
        .with_left_node(12)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);

    // Second row (y=1): cells 12-16
    for i in 12..=16 {
        let forward = if i < 16 { i + 1 } else { -1 };
        let cell = Cell::new(i)
            .with_point(new_point(i as f64, 1.0, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(-1)
            .build();
        grid.add_cell(cell);
    }

    // Create grids storage and session
    let grids_storage = GridsStorage::new().with_vehicles_net(grid).build();
    let mut session = Session::new(grids_storage, None);

    // Create vehicle at cell 3 with tail size 2
    let vehicles = vec![
        Vehicle::new(1)
            .with_cell(3)
            .with_tail_size(2, vec![1, 2])
            .with_destination(16)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
    ];
    session.add_vehicles(vehicles);

    // Expected states for each step
    // Vehicle moves forward until step 7, then blocks at cell 11
    let expected_states = vec![
        ExpectedState { head_cell: 4, tail_cells: vec![2, 3] },
        ExpectedState { head_cell: 5, tail_cells: vec![3, 4] },
        ExpectedState { head_cell: 6, tail_cells: vec![4, 5] },
        ExpectedState { head_cell: 7, tail_cells: vec![5, 6] },
        ExpectedState { head_cell: 8, tail_cells: vec![6, 7] },
        ExpectedState { head_cell: 9, tail_cells: vec![7, 8] },
        ExpectedState { head_cell: 10, tail_cells: vec![8, 9] },
        ExpectedState { head_cell: 11, tail_cells: vec![9, 10] }, // Step 7: arrives at cell 11
        ExpectedState { head_cell: 11, tail_cells: vec![9, 10] }, // Step 8: blocked
        ExpectedState { head_cell: 11, tail_cells: vec![9, 10] }, // Step 9: blocked
        ExpectedState { head_cell: 11, tail_cells: vec![9, 10] }, // Step 10: blocked
        ExpectedState { head_cell: 11, tail_cells: vec![9, 10] }, // Step 11: blocked
        ExpectedState { head_cell: 11, tail_cells: vec![9, 10] }, // Step 12: blocked
        ExpectedState { head_cell: 11, tail_cells: vec![9, 10] }, // Step 13: blocked
        ExpectedState { head_cell: 11, tail_cells: vec![9, 10] }, // Step 14: blocked
        ExpectedState { head_cell: 11, tail_cells: vec![9, 10] }, // Step 15: blocked
        ExpectedState { head_cell: 11, tail_cells: vec![9, 10] }, // Step 16: blocked
        ExpectedState { head_cell: 11, tail_cells: vec![9, 10] }, // Step 17: blocked
        ExpectedState { head_cell: 11, tail_cells: vec![9, 10] }, // Step 18: blocked
        ExpectedState { head_cell: 11, tail_cells: vec![9, 10] }, // Step 19: blocked
    ];

    // Run simulation - vehicle should block at cell 11 without panicking
    let time_sim = 20;

    for step in 0..time_sim {
        match session.step() {
            Ok(state) => {
                if !state.vehicles.is_empty() && step < expected_states.len() {
                    let vehicle_state = &state.vehicles[0];
                    let expected = &expected_states[step];

                    // Assert head position
                    assert_eq!(
                        vehicle_state.last_cell, expected.head_cell,
                        "Step {}: expected head at cell {}, got {}",
                        step, expected.head_cell, vehicle_state.last_cell
                    );

                    // Assert tail positions
                    assert_eq!(
                        vehicle_state.tail_cells.len(), expected.tail_cells.len(),
                        "Step {}: expected tail length {}, got {}",
                        step, expected.tail_cells.len(), vehicle_state.tail_cells.len()
                    );
                    for (i, expected_tail) in expected.tail_cells.iter().enumerate() {
                        assert_eq!(
                            vehicle_state.tail_cells[i], *expected_tail,
                            "Step {}: expected tail[{}] = {}, got {}",
                            step, i, expected_tail, vehicle_state.tail_cells[i]
                        );
                    }
                }
            }
            Err(e) => panic!("Error at step {}: {}", step, e),
        }
    }

    // Note: Vehicle will be blocked at cell 11 because it cannot perform
    // LEFT maneuver while tail is still completing RIGHT maneuver.
    // This is expected behavior for the ping-pong scenario.
}

/// TestExtendedZigzagTailSize1 tests a vehicle with tail size 1 navigating extended zigzag road.
/// Vehicle CAN pass through because tail completes maneuver before head needs next maneuver.
/// Road layout:
/// [1]-[2]-[3]-[4]-[5]-[6]-[7]-[8]-[9]-[10]          [13]-[14]-[15]-[16]
///                                        \          /
///                                        [11] - [12]
#[test]
fn test_extended_zigzag_tail_size_1() {
    let mut grid = GridRoads::new();

    // First row (y=1): cells 1-10
    for i in 1..=10 {
        let forward = if i < 10 { i + 1 } else { -1 };
        let right = if i == 10 { 11 } else { -1 };
        let cell = Cell::new(i)
            .with_point(new_point(i as f64, 1.0, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(right)
            .build();
        grid.add_cell(cell);
    }

    // Cell 11 (y=0.5): transition cell with FORWARD to 12
    let cell = Cell::new(11)
        .with_point(new_point(11.0, 0.5, None))
        .with_speed_limit(3)
        .with_forward_node(12)
        .with_left_node(-1)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);

    // Cell 12 (y=0.5): transition cell with LEFT to 13
    let cell = Cell::new(12)
        .with_point(new_point(12.0, 0.5, None))
        .with_speed_limit(3)
        .with_forward_node(-1)
        .with_left_node(13)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);

    // Second row (y=1): cells 13-16
    for i in 13..=16 {
        let forward = if i < 16 { i + 1 } else { -1 };
        let cell = Cell::new(i)
            .with_point(new_point(i as f64, 1.0, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(-1)
            .build();
        grid.add_cell(cell);
    }

    // Create grids storage and session
    let grids_storage = GridsStorage::new().with_vehicles_net(grid).build();
    let mut session = Session::new(grids_storage, None);

    // Create vehicle at cell 3 with tail size 1
    let vehicles = vec![
        Vehicle::new(1)
            .with_cell(3)
            .with_tail_size(1, vec![2])
            .with_destination(16)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
    ];
    session.add_vehicles(vehicles);

    // Expected states for each step
    // Vehicle with tail size 1 CAN pass through the zigzag
    let expected_states = vec![
        ExpectedState { head_cell: 4, tail_cells: vec![3] },
        ExpectedState { head_cell: 5, tail_cells: vec![4] },
        ExpectedState { head_cell: 6, tail_cells: vec![5] },
        ExpectedState { head_cell: 7, tail_cells: vec![6] },
        ExpectedState { head_cell: 8, tail_cells: vec![7] },
        ExpectedState { head_cell: 9, tail_cells: vec![8] },
        ExpectedState { head_cell: 10, tail_cells: vec![9] },
        ExpectedState { head_cell: 11, tail_cells: vec![10] }, // RIGHT maneuver to 11
        ExpectedState { head_cell: 12, tail_cells: vec![11] }, // FORWARD to 12, tail completed RIGHT
        ExpectedState { head_cell: 13, tail_cells: vec![12] }, // LEFT to 13 allowed
        ExpectedState { head_cell: 14, tail_cells: vec![13] },
        ExpectedState { head_cell: 15, tail_cells: vec![14] },
        // Vehicle reaches destination at cell 16 and is removed
    ];

    // Run simulation
    let time_sim = 20;
    let mut vehicle_reached_destination = false;

    for step in 0..time_sim {
        match session.step() {
            Ok(state) => {
                // Check if vehicle reached destination (no more vehicles in simulation)
                if state.vehicles.is_empty() {
                    vehicle_reached_destination = true;
                } else if step < expected_states.len() {
                    let vehicle_state = &state.vehicles[0];
                    let expected = &expected_states[step];

                    // Assert head position
                    assert_eq!(
                        vehicle_state.last_cell, expected.head_cell,
                        "Step {}: expected head at cell {}, got {}",
                        step, expected.head_cell, vehicle_state.last_cell
                    );

                    // Assert tail positions
                    assert_eq!(
                        vehicle_state.tail_cells.len(), expected.tail_cells.len(),
                        "Step {}: expected tail length {}, got {}",
                        step, expected.tail_cells.len(), vehicle_state.tail_cells.len()
                    );
                    for (i, expected_tail) in expected.tail_cells.iter().enumerate() {
                        assert_eq!(
                            vehicle_state.tail_cells[i], *expected_tail,
                            "Step {}: expected tail[{}] = {}, got {}",
                            step, i, expected_tail, vehicle_state.tail_cells[i]
                        );
                    }
                }
            }
            Err(e) => panic!("Error at step {}: {}", step, e),
        }
    }

    assert!(vehicle_reached_destination, "Vehicle did not reach destination within {} steps", time_sim);
}

/// TestExtendedZigzagTailSize2 tests a vehicle with tail size 2 navigating extended zigzag road.
/// Vehicle CANNOT pass through because tail is still completing maneuver when head needs next maneuver.
/// Road layout:
/// [1]-[2]-[3]-[4]-[5]-[6]-[7]-[8]-[9]-[10]          [13]-[14]-[15]-[16]
///                                        \          /
///                                        [11] - [12]
#[test]
fn test_extended_zigzag_tail_size_2() {
    let mut grid = GridRoads::new();

    // First row (y=1): cells 1-10
    for i in 1..=10 {
        let forward = if i < 10 { i + 1 } else { -1 };
        let right = if i == 10 { 11 } else { -1 };
        let cell = Cell::new(i)
            .with_point(new_point(i as f64, 1.0, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(right)
            .build();
        grid.add_cell(cell);
    }

    // Cell 11 (y=0.5): transition cell with FORWARD to 12
    let cell = Cell::new(11)
        .with_point(new_point(11.0, 0.5, None))
        .with_speed_limit(3)
        .with_forward_node(12)
        .with_left_node(-1)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);

    // Cell 12 (y=0.5): transition cell with LEFT to 13
    let cell = Cell::new(12)
        .with_point(new_point(12.0, 0.5, None))
        .with_speed_limit(3)
        .with_forward_node(-1)
        .with_left_node(13)
        .with_right_node(-1)
        .build();
    grid.add_cell(cell);

    // Second row (y=1): cells 13-16
    for i in 13..=16 {
        let forward = if i < 16 { i + 1 } else { -1 };
        let cell = Cell::new(i)
            .with_point(new_point(i as f64, 1.0, None))
            .with_speed_limit(3)
            .with_forward_node(forward)
            .with_left_node(-1)
            .with_right_node(-1)
            .build();
        grid.add_cell(cell);
    }

    // Create grids storage and session
    let grids_storage = GridsStorage::new().with_vehicles_net(grid).build();
    let mut session = Session::new(grids_storage, None);

    // Create vehicle at cell 3 with tail size 2
    let vehicles = vec![
        Vehicle::new(1)
            .with_cell(3)
            .with_tail_size(2, vec![1, 2])
            .with_destination(16)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .build(),
    ];
    session.add_vehicles(vehicles);

    // Expected states for each step
    // Vehicle with tail size 2 gets stuck at cell 12
    let expected_states = vec![
        ExpectedState { head_cell: 4, tail_cells: vec![2, 3] },
        ExpectedState { head_cell: 5, tail_cells: vec![3, 4] },
        ExpectedState { head_cell: 6, tail_cells: vec![4, 5] },
        ExpectedState { head_cell: 7, tail_cells: vec![5, 6] },
        ExpectedState { head_cell: 8, tail_cells: vec![6, 7] },
        ExpectedState { head_cell: 9, tail_cells: vec![7, 8] },
        ExpectedState { head_cell: 10, tail_cells: vec![8, 9] },
        ExpectedState { head_cell: 11, tail_cells: vec![9, 10] },  // RIGHT maneuver to 11
        ExpectedState { head_cell: 12, tail_cells: vec![10, 11] }, // FORWARD to 12, tail[0]=10 hasn't done maneuver
        ExpectedState { head_cell: 12, tail_cells: vec![10, 11] }, // Stuck - can't do LEFT to 13
        ExpectedState { head_cell: 12, tail_cells: vec![10, 11] }, // Stuck
        ExpectedState { head_cell: 12, tail_cells: vec![10, 11] }, // Stuck
        ExpectedState { head_cell: 12, tail_cells: vec![10, 11] }, // Stuck
        ExpectedState { head_cell: 12, tail_cells: vec![10, 11] }, // Stuck
        ExpectedState { head_cell: 12, tail_cells: vec![10, 11] }, // Stuck
        ExpectedState { head_cell: 12, tail_cells: vec![10, 11] }, // Stuck
        ExpectedState { head_cell: 12, tail_cells: vec![10, 11] }, // Stuck
        ExpectedState { head_cell: 12, tail_cells: vec![10, 11] }, // Stuck
        ExpectedState { head_cell: 12, tail_cells: vec![10, 11] }, // Stuck
        ExpectedState { head_cell: 12, tail_cells: vec![10, 11] }, // Stuck
    ];

    // Run simulation - vehicle should block at cell 12 without panicking
    let time_sim = 20;

    for step in 0..time_sim {
        match session.step() {
            Ok(state) => {
                if !state.vehicles.is_empty() && step < expected_states.len() {
                    let vehicle_state = &state.vehicles[0];
                    let expected = &expected_states[step];

                    // Assert head position
                    assert_eq!(
                        vehicle_state.last_cell, expected.head_cell,
                        "Step {}: expected head at cell {}, got {}",
                        step, expected.head_cell, vehicle_state.last_cell
                    );

                    // Assert tail positions
                    assert_eq!(
                        vehicle_state.tail_cells.len(), expected.tail_cells.len(),
                        "Step {}: expected tail length {}, got {}",
                        step, expected.tail_cells.len(), vehicle_state.tail_cells.len()
                    );
                    for (i, expected_tail) in expected.tail_cells.iter().enumerate() {
                        assert_eq!(
                            vehicle_state.tail_cells[i], *expected_tail,
                            "Step {}: expected tail[{}] = {}, got {}",
                            step, i, expected_tail, vehicle_state.tail_cells[i]
                        );
                    }
                }
            }
            Err(e) => panic!("Error at step {}: {}", step, e),
        }
    }

    // Note: Vehicle will be blocked at cell 12 because it cannot perform
    // LEFT maneuver to cell 13 while tail[0] is still at cell 10 (pre-maneuver position).
    // This is expected behavior for the extended zigzag scenario with tail size 2.
}
