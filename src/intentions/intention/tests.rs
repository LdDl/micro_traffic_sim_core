use super::*;
use crate::geom::new_point;
use crate::grid::cell::Cell;
use crate::utils::test_grids::create_pretty_simple_grid;
#[test]
fn lane_change_rear_margin_counts_distance_to_merge_cell() {
    let mut net = GridRoads::new();
    for id in 1..=12 {
        net.add_cell(
            Cell::new(id)
                .with_forward_node(if id < 12 { id + 1 } else { -1 })
                .with_left_node(-1)
                .with_right_node(-1)
                .with_speed_limit(5)
                .build(),
        );
    }
    // A merge into cell 10 has one empty cell before the rear body in cell 8.
    // The check uses D = 2, not the count of one empty cell.
    let occupancy = HashMap::from([(8, 1)]);
    let moving_rear = HashMap::from([(8, 1)]);
    assert!(lane_change_rear_safe(
        9,
        &net,
        &occupancy,
        &moving_rear,
        1,
        6
    ));
    assert!(!lane_change_rear_safe(
        9,
        &net,
        &occupancy,
        &moving_rear,
        2,
        7
    ));
    let stopped_rear = HashMap::from([(8, 0)]);
    assert!(lane_change_rear_safe(
        9,
        &net,
        &occupancy,
        &stopped_rear,
        2,
        7
    ));
    assert!(!lane_change_rear_safe(
        9,
        &net,
        &occupancy,
        &stopped_rear,
        3,
        8
    ));
}

#[test]
fn test_intention() {
    let net = create_pretty_simple_grid();

    // Case 1: basics
    let current_state: HashMap<CellID, VehicleID> = HashMap::from([(101, 1)]);
    let vehicle_1 = Vehicle::new(1)
        .with_cell(101)
        .with_speed(1)
        .with_speed_limit(1)
        .with_destination(7)
        .build();
    let intention = find_intention(
        &net,
        &current_state,
        &HashMap::new(),
        &vehicle_1,
        &LocalLogger::none(),
    )
    .unwrap();
    let correct_intention = VehicleIntention {
        intention_cell_id: 1,
        intention_speed: 1,
        intention_maneuver: LaneChangeType::NoChange,
        ..Default::default()
    };
    assert_eq!(intention, correct_intention);

    // Case 2: move with speed > 1
    let current_state: HashMap<CellID, VehicleID> = HashMap::from([(101, 1)]);
    let vehicle_1 = Vehicle::new(1)
        .with_cell(101)
        .with_speed(3)
        .with_speed_limit(3)
        .with_destination(7)
        .build();
    let intention = find_intention(
        &net,
        &current_state,
        &HashMap::new(),
        &vehicle_1,
        &LocalLogger::none(),
    )
    .unwrap();
    let correct_intention = VehicleIntention {
        intention_cell_id: 3,
        intention_speed: 3,
        intention_maneuver: LaneChangeType::NoChange,
        intermediate_cells: vec![1, 2],
        ..Default::default()
    };
    assert_eq!(intention, correct_intention);

    // Case 3: move with speed > 1, but intention speed will be less due maneuver
    // before last cell in path. Intention cell will be cell
    // before last cell in path due the same reason.
    let current_state: HashMap<CellID, VehicleID> = HashMap::from([(101, 1)]);
    let vehicle_1 = Vehicle::new(1)
        .with_cell(101)
        .with_speed(4)
        .with_destination(8)
        .build();
    let intention = find_intention(
        &net,
        &current_state,
        &HashMap::new(),
        &vehicle_1,
        &LocalLogger::none(),
    )
    .unwrap();
    let correct_intention = VehicleIntention {
        intention_cell_id: 3,
        intention_speed: 3,
        intention_maneuver: LaneChangeType::NoChange,
        intermediate_cells: vec![1, 2],
        ..Default::default()
    };
    assert_eq!(intention, correct_intention);

    // Case 4: vehicle could not move due other vehicle in front
    let current_state: HashMap<CellID, VehicleID> = HashMap::from([(101, 1), (1, 2)]);
    let vehicle_1 = Vehicle::new(1)
        .with_cell(101)
        .with_speed(3)
        .with_destination(7)
        .build();
    let intention = find_intention(
        &net,
        &current_state,
        &HashMap::new(),
        &vehicle_1,
        &LocalLogger::none(),
    )
    .unwrap();
    let correct_intention = VehicleIntention {
        intention_cell_id: 101,
        intention_speed: 0,
        intention_maneuver: LaneChangeType::Block,
        should_stop: true,
        ..Default::default()
    };
    assert_eq!(intention, correct_intention);

    // Case 5: vehicle could move but not that far and will decrease speed due the other vehicle in front
    let current_state: HashMap<CellID, VehicleID> = HashMap::from([(101, 1), (3, 2)]);
    let vehicle_1 = Vehicle::new(1)
        .with_cell(101)
        .with_speed(3)
        .with_destination(7)
        .build();
    let intention = find_intention(
        &net,
        &current_state,
        &HashMap::new(),
        &vehicle_1,
        &LocalLogger::none(),
    )
    .unwrap();
    let correct_intention = VehicleIntention {
        intention_cell_id: 2,
        intention_speed: 2,
        intention_maneuver: LaneChangeType::NoChange,
        intermediate_cells: vec![1],
        ..Default::default()
    };
    assert_eq!(intention, correct_intention);

    // Case 6: vehicle has speed more than is needed to reach destination (vehicle should slow down)
    let current_state: HashMap<CellID, VehicleID> = HashMap::from([(101, 1)]);
    let vehicle_1 = Vehicle::new(1)
        .with_cell(101)
        .with_speed(3)
        .with_destination(2)
        .build();
    let intention = find_intention(
        &net,
        &current_state,
        &HashMap::new(),
        &vehicle_1,
        &LocalLogger::none(),
    )
    .unwrap();
    let correct_intention = VehicleIntention {
        intention_cell_id: 2,
        intention_speed: 2,
        intention_maneuver: LaneChangeType::NoChange,
        intermediate_cells: vec![1],
        ..Default::default()
    };
    assert_eq!(intention, correct_intention);

    // Case 7: vehicle can reach destination
    let current_state: HashMap<CellID, VehicleID> = HashMap::from([(101, 1)]);
    let vehicle_1 = Vehicle::new(1)
        .with_cell(101)
        .with_speed(4)
        .with_destination(7)
        .build();
    let intention = find_intention(
        &net,
        &current_state,
        &HashMap::new(),
        &vehicle_1,
        &LocalLogger::none(),
    )
    .unwrap();
    let correct_intention = VehicleIntention {
        intention_cell_id: 7,
        intention_speed: 4,
        intention_maneuver: LaneChangeType::NoChange,
        intermediate_cells: vec![1, 2, 3],
        ..Default::default()
    };
    assert_eq!(intention, correct_intention);
}
#[test]
fn test_alternate_intention() {
    let speed_limit = 4;
    let mut net = GridRoads::new();
    net.add_cell(
        Cell::new(1)
            .with_speed_limit(speed_limit)
            .with_forward_node(2)
            .with_point(new_point(0.0, 0.0, None))
            .build(),
    );
    net.add_cell(
        Cell::new(2)
            .with_speed_limit(speed_limit)
            .with_right_node(8) /* Maneuver allowed */
            .with_forward_node(3)
            .with_point(new_point(1.0, 0.0, None))
            .build(),
    );
    net.add_cell(
        Cell::new(3)
            .with_speed_limit(speed_limit)
            .with_forward_node(4)
            .with_point(new_point(2.0, 0.0, None))
            .build(),
    );
    net.add_cell(
        Cell::new(4)
            .with_speed_limit(speed_limit)
            .with_forward_node(5)
            .with_point(new_point(3.0, 0.0, None))
            .build(),
    );
    net.add_cell(
        Cell::new(5)
            .with_speed_limit(speed_limit)
            .with_forward_node(6)
            .with_point(new_point(4.0, 0.0, None))
            .build(),
    );
    net.add_cell(
        Cell::new(6)
            .with_speed_limit(speed_limit)
            .with_point(new_point(5.0, 0.0, None))
            .build(),
    );

    // Other lane
    net.add_cell(
        Cell::new(7)
            .with_speed_limit(speed_limit)
            .with_forward_node(8)
            .with_point(new_point(0.0, 1.0, None))
            .build(),
    );
    net.add_cell(
        Cell::new(8)
            .with_speed_limit(speed_limit)
            .with_forward_node(9)
            .with_point(new_point(1.0, 1.0, None))
            .build(),
    );
    net.add_cell(
        Cell::new(9)
            .with_speed_limit(speed_limit)
            .with_forward_node(10)
            .with_point(new_point(2.0, 1.0, None))
            .build(),
    );
    net.add_cell(
        Cell::new(10)
            .with_speed_limit(speed_limit)
            .with_forward_node(11)
            .with_point(new_point(3.0, 1.0, None))
            .build(),
    );
    net.add_cell(
        Cell::new(11)
            .with_speed_limit(speed_limit)
            .with_forward_node(12)
            .with_point(new_point(4.0, 1.0, None))
            .build(),
    );
    net.add_cell(
        Cell::new(12)
            .with_speed_limit(speed_limit)
            .with_point(new_point(5.0, 1.0, None))
            .build(),
    );

    // Unreachable
    net.add_cell(
        Cell::new(500)
            .with_speed_limit(speed_limit)
            .with_point(new_point(10.0, 10.0, None))
            .build(),
    );

    // Case 1: Vehicle tries to find path via right maneuver (because of vehicle in front) and founds it
    let source_cell = net.get_cell(&2).unwrap();
    let blocked_cell = net.get_cell(&3).unwrap();
    let dest_cell = net.get_cell(&12).unwrap();
    let mut vehicle = Vehicle::new(42)
        .with_cell(source_cell.get_id())
        .with_speed(3)
        .with_destination(dest_cell.get_id())
        .build();
    let blocking_vehicle = Vehicle::new(78).with_cell(blocked_cell.get_id()).build();

    let mut current_state: HashMap<CellID, VehicleID> = HashMap::new();
    current_state.insert(source_cell.get_id(), vehicle.id);
    current_state.insert(blocked_cell.get_id(), blocking_vehicle.id);

    let mut intentions: Intentions = Intentions::new();
    let collected_intention =
        find_alternate_intention(&net, &current_state, &HashMap::new(), &vehicle, true, 10, 0);
    assert!(collected_intention.is_ok());
    let unwrapped_intention = collected_intention.unwrap();
    vehicle.set_intention(unwrapped_intention);
    intentions.add_intention(&mut vehicle, IntentionType::Target);

    // Speed should be 1 for cases when the vehicle can move, and 0 for cases when it could not move
    let mut correct_vehicle = Vehicle::new(42)
        .with_cell(source_cell.get_id())
        .with_destination(dest_cell.get_id())
        .build();
    let mut correct_intention = VehicleIntention::default();
    correct_intention.intention_cell_id = source_cell.get_right_id();
    correct_intention.intention_maneuver = LaneChangeType::ChangeRight;
    correct_intention.intention_speed = 1;
    correct_vehicle.set_intention(correct_intention);
    let mut correct_intentions = Intentions::new();
    correct_intentions.add_intention(&mut correct_vehicle, IntentionType::Target);
    assert_eq!(
        correct_intentions.len(),
        intentions.len(),
        "Incorrect number of intentions"
    );
    for (i, _corr_int) in correct_intentions.iter() {
        assert_eq!(
            intentions.get(i).is_some(),
            true,
            "No intention for cell #{} in correct intentions. Intention: {:?}",
            i,
            intentions.get(i)
        );
    }
    for (i, _int) in intentions.iter() {
        assert_eq!(
            correct_intentions.get(i).is_some(),
            true,
            "No intention for cell #{} in found intentions. Correct intention: {:?}",
            i,
            correct_intentions.get(i)
        );
    }
    for (i, int) in intentions.iter() {
        assert_eq!(
            correct_intentions.get(i).unwrap().len(),
            int.len(),
            "Incorrect number of intentions for cell #{}",
            i
        );
        for (j, intention) in int.iter().enumerate() {
            assert_eq!(
                correct_intentions.get(i).unwrap()[j].int_type,
                intention.int_type,
                "Incorrect intention type for cell #{} at pos #{}",
                i,
                j
            );
            assert_eq!(
                correct_intentions.get(i).unwrap()[j].vehicle_id,
                intention.vehicle_id,
                "Incorrect vehicle ID for cell #{} at pos #{}",
                i,
                j
            );
        }
    }

    // Case 2: Vehicle tries to find path via right maneuver (because of vehicle in front) but cannot find it
    // Could be demonstrated by trying to find path from vehicle's cell to any isolated cell
    let source_cell = net.get_cell(&2).unwrap();
    let blocked_cell = net.get_cell(&3).unwrap();
    let dest_cell = net.get_cell(&500).unwrap();
    let mut vehicle = Vehicle::new(42)
        .with_cell(source_cell.get_id())
        .with_speed(3)
        .with_destination(dest_cell.get_id())
        .build();
    let blocking_vehicle = Vehicle::new(78).with_cell(blocked_cell.get_id()).build();

    let mut current_state: HashMap<CellID, VehicleID> = HashMap::new();
    current_state.insert(source_cell.get_id(), vehicle.id);
    current_state.insert(blocked_cell.get_id(), blocking_vehicle.id);

    let mut intentions = Intentions::new();
    let collected_intention =
        find_alternate_intention(&net, &current_state, &HashMap::new(), &vehicle, true, 10, 0);
    assert!(collected_intention.is_ok());
    let unwrapped_intention = collected_intention.unwrap();
    assert_eq!(unwrapped_intention.intention_cell_id, source_cell.get_id());
    assert_eq!(unwrapped_intention.intention_speed, 0);
    assert_eq!(
        unwrapped_intention.intention_maneuver,
        LaneChangeType::Block
    );
    vehicle.set_intention(unwrapped_intention);
    intentions.add_intention(&mut vehicle, IntentionType::Target);

    // Cell 500 has no incoming edge, and the forward cell is occupied.
    // An unreachable goal cannot justify entering the adjacent lane.
    let mut correct_vehicle = Vehicle::new(42)
        .with_cell(source_cell.get_id())
        .with_destination(dest_cell.get_id())
        .build();
    let mut correct_intention = VehicleIntention::default();
    correct_intention.intention_cell_id = source_cell.get_id();
    correct_intention.intention_maneuver = LaneChangeType::Block;
    correct_intention.intention_speed = 0;
    let mut correct_intentions = Intentions::new();
    correct_vehicle.set_intention(correct_intention);
    correct_intentions.add_intention(&mut correct_vehicle, IntentionType::Target);
    assert_eq!(
        correct_intentions.len(),
        intentions.len(),
        "Incorrect number of intentions"
    );
    for (i, _corr_int) in correct_intentions.iter() {
        assert_eq!(
            intentions.get(i).is_some(),
            true,
            "No intention for cell #{} in correct intentions. Intention: {:?}",
            i,
            intentions.get(i)
        );
    }
    for (i, _int) in intentions.iter() {
        assert_eq!(
            correct_intentions.get(i).is_some(),
            true,
            "No intention for cell #{} in found intentions. Correct intention: {:?}",
            i,
            correct_intentions.get(i)
        );
    }
    for (i, int) in intentions.iter() {
        assert_eq!(
            correct_intentions.get(i).unwrap().len(),
            int.len(),
            "Incorrect number of intentions for cell #{}",
            i
        );
        for (j, intention) in int.iter().enumerate() {
            assert_eq!(
                correct_intentions.get(i).unwrap()[j].int_type,
                intention.int_type,
                "Incorrect intention type for cell #{} at pos #{}",
                i,
                j
            );
            assert_eq!(
                correct_intentions.get(i).unwrap()[j].vehicle_id,
                intention.vehicle_id,
                "Incorrect vehicle ID for cell #{} at pos #{}",
                i,
                j
            );
        }
    }

    // Case 3: Vehicle tries to find path via right maneuver (because of vehicle in front) but cannot find it
    // and there is also another vehicle in the right maneuver cell
    let source_cell = net.get_cell(&2).unwrap();
    let blocked_cell = net.get_cell(&3).unwrap();
    let dest_cell = net.get_cell(&500).unwrap();
    let mut vehicle = Vehicle::new(42)
        .with_cell(source_cell.get_id())
        .with_speed(3)
        .with_destination(dest_cell.get_id())
        .build();
    let blocking_vehicle = Vehicle::new(78).with_cell(blocked_cell.get_id()).build();
    let blocking_vehicle2 = Vehicle::new(4278)
        .with_cell(source_cell.get_right_id())
        .build();

    let mut current_state: HashMap<CellID, VehicleID> = HashMap::new();
    current_state.insert(source_cell.get_id(), vehicle.id);
    current_state.insert(blocked_cell.get_id(), blocking_vehicle.id);
    current_state.insert(source_cell.get_right_id(), blocking_vehicle2.id);

    let mut intentions = Intentions::new();
    let collected_intention =
        find_alternate_intention(&net, &current_state, &HashMap::new(), &vehicle, true, 10, 0);
    assert!(collected_intention.is_ok());
    let unwrapped_intention = collected_intention.unwrap();
    vehicle.set_intention(unwrapped_intention);
    intentions.add_intention(&mut vehicle, IntentionType::Target);

    // Speed should be 1 for cases when the vehicle can move, and 0 for cases when it could not move
    // Vehicle could not move either forward or right
    let mut correct_vehicle = Vehicle::new(42)
        .with_cell(source_cell.get_id())
        .with_destination(dest_cell.get_id())
        .build();
    let mut correct_intention = VehicleIntention::default();
    correct_intention.intention_cell_id = source_cell.get_id();
    correct_intention.intention_maneuver = LaneChangeType::Block;
    correct_intention.intention_speed = 0;
    let mut correct_intentions = Intentions::new();
    correct_vehicle.set_intention(correct_intention);
    correct_intentions.add_intention(&mut correct_vehicle, IntentionType::Target);
    assert_eq!(
        correct_intentions.len(),
        intentions.len(),
        "Incorrect number of intentions"
    );
    for (i, _corr_int) in correct_intentions.iter() {
        assert_eq!(
            intentions.get(i).is_some(),
            true,
            "No intention for cell #{} in correct intentions. Intention: {:?}",
            i,
            intentions.get(i)
        );
    }
    for (i, _int) in intentions.iter() {
        assert_eq!(
            correct_intentions.get(i).is_some(),
            true,
            "No intention for cell #{} in found intentions. Correct intention: {:?}",
            i,
            correct_intentions.get(i)
        );
    }
    for (i, int) in intentions.iter() {
        assert_eq!(
            correct_intentions.get(i).unwrap().len(),
            int.len(),
            "Incorrect number of intentions for cell #{}",
            i
        );
        for (j, intention) in int.iter().enumerate() {
            assert_eq!(
                correct_intentions.get(i).unwrap()[j].int_type,
                intention.int_type,
                "Incorrect intention type for cell #{} at pos #{}",
                i,
                j
            );
            assert_eq!(
                correct_intentions.get(i).unwrap()[j].vehicle_id,
                intention.vehicle_id,
                "Incorrect vehicle ID for cell #{} at pos #{}",
                i,
                j
            );
        }
    }
}

/// The forward-roll reachability guard must honour the SAME bound the per-tick reconnect
/// uses (`reconnect_max_depth`), not a separate hard-coded constant. A roll whose rejoin
/// point sits at BFS depth 2 is authorized only when the bound is >= 2; otherwise the
/// guard would green-light a roll that next tick's reconnect (same bound) cannot follow,
/// forcing a needless full A*. Fast paths (route continues straight / forward is the
/// destination) bypass the BFS and ignore the bound entirely.
#[test]
fn test_forward_guard_uses_reconnect_depth_bound() {
    let sl = 3;
    let mut net = GridRoads::new();
    // Cached route 1 -> 2 -> 3. The vehicle sits at route[0]=1.
    net.add_cell(
        Cell::new(1)
            .with_speed_limit(sl)
            .with_forward_node(2)
            .with_point(new_point(0.0, 0.0, None))
            .build(),
    );
    net.add_cell(
        Cell::new(2)
            .with_speed_limit(sl)
            .with_forward_node(3)
            .with_point(new_point(1.0, 0.0, None))
            .build(),
    );
    net.add_cell(
        Cell::new(3)
            .with_speed_limit(sl)
            .with_point(new_point(2.0, 0.0, None))
            .build(),
    );
    // Off-route detour 10 -> 11 -> 2: rejoins the cache (cell 2) at BFS depth 2.
    net.add_cell(
        Cell::new(10)
            .with_speed_limit(sl)
            .with_forward_node(11)
            .with_point(new_point(0.0, 1.0, None))
            .build(),
    );
    net.add_cell(
        Cell::new(11)
            .with_speed_limit(sl)
            .with_forward_node(2)
            .with_point(new_point(1.0, 1.0, None))
            .build(),
    );

    let mut vehicle = Vehicle::new(1).with_cell(1).with_speed(3).build();
    vehicle.cached_route = vec![1, 2, 3];
    vehicle.route_idx = 0;

    // BFS path: rejoin is at depth 2, so the bound decides.
    assert!(
        !forward_keeps_reachable(&vehicle, 10, &net, 1),
        "bound 1 < rejoin depth 2 -> not authorized"
    );
    assert!(
        forward_keeps_reachable(&vehicle, 10, &net, 2),
        "bound 2 == rejoin depth 2 -> authorized"
    );

    // Fast paths ignore the bound: route continues straight onto 2, and 3 is the destination.
    assert!(
        forward_keeps_reachable(&vehicle, 2, &net, 0),
        "route continues straight onto forward -> always ok"
    );
    assert!(
        forward_keeps_reachable(&vehicle, 3, &net, 0),
        "forward is the destination (last route cell) -> always ok"
    );
}

/// Regression (lost-for-tailed): an off-cache vehicle whose only route to the destination
/// needs a lane change that is momentarily on cooldown (tail mid-maneuver, modelled here by
/// `timer_non_maneuvers > 0`) must NOT latch `confusion`. The maneuvers-disabled A* fails,
/// but the destination is still reachable WITH a maneuver, so the vehicle rolls forward and
/// stays un-confused (recoverable) instead of being written off as lost. Confusion must
/// still latch when the destination is genuinely unreachable even with maneuvers.
#[test]
fn test_offcache_cooldown_reachable_via_maneuver_does_not_confuse() {
    let sl = 3;
    let mut net = GridRoads::new();
    // From cell 1 the ONLY route to 4 is the lane change 1 -(right)-> 3 -> 4.
    // Going straight (1 -> 2) is a dead-end, so a maneuvers-disabled A* finds no path.
    net.add_cell(
        Cell::new(1)
            .with_speed_limit(sl)
            .with_forward_node(2)
            .with_right_node(3)
            .with_point(new_point(0.0, 0.0, None))
            .build(),
    );
    // dead-end
    net.add_cell(
        Cell::new(2)
            .with_speed_limit(sl)
            .with_point(new_point(1.0, 0.0, None))
            .build(),
    );
    net.add_cell(
        Cell::new(3)
            .with_speed_limit(sl)
            .with_forward_node(4)
            .with_point(new_point(0.0, 1.0, None))
            .build(),
    );
    // destination
    net.add_cell(
        Cell::new(4)
            .with_speed_limit(sl)
            .with_point(new_point(1.0, 1.0, None))
            .build(),
    );
    // disconnected
    net.add_cell(
        Cell::new(500)
            .with_speed_limit(sl)
            .with_point(new_point(9.0, 9.0, None))
            .build(),
    );

    let state: HashMap<CellID, VehicleID> = HashMap::new();

    // Reachable-only-via-maneuver + maneuver on cooldown -> must NOT confuse.
    let mut v = Vehicle::new(1)
        .with_cell(1)
        .with_speed(3)
        .with_destination(4)
        .build();
    // tail still completing a previous maneuver -> maneuvers disabled
    v.timer_non_maneuvers = 5;
    let intention =
        find_intention(&net, &state, &HashMap::new(), &v, &LocalLogger::none()).unwrap();
    assert_ne!(
        intention.confusion,
        Some(true),
        "reachable-via-maneuver must not latch confusion (would be lost)"
    );

    // Control: genuinely unreachable destination, maneuvers allowed -> confusion DOES latch.
    let v = Vehicle::new(2)
        .with_cell(1)
        .with_speed(3)
        .with_destination(500)
        .build();
    let intention =
        find_intention(&net, &state, &HashMap::new(), &v, &LocalLogger::none()).unwrap();
    assert_eq!(
        intention.confusion,
        Some(true),
        "truly unreachable destination still latches confusion"
    );
}
