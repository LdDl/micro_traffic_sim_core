use super::*;
#[test]
fn test_observable_path() {
    let speed_limit = 4;
    let cell1 = Cell::new(1).with_speed_limit(speed_limit).build();
    let cell2 = Cell::new(2).with_speed_limit(speed_limit).build();
    let mut cell3 = Cell::new(3).with_speed_limit(speed_limit).build();
    let cell4 = Cell::new(4).with_speed_limit(speed_limit).build();
    let cell5 = Cell::new(5).with_speed_limit(speed_limit).build();

    let mut path = Path::new(
        vec![&cell1, &cell2, &cell3, &cell4, &cell5],
        vec![
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
            LaneChangeType::ChangeLeft,
        ],
        10.0,
    );

    let mut current_state = HashMap::new();
    let observable_path = process_path(&mut path, speed_limit, cell5.get_id(), &current_state);

    let correct_path = ObservablePath {
        wanted_maneuver: LaneChangeType::NoChange,
        last_cell_state: CellState::Free,
        trimmed_path: vec![&cell2, &cell3, &cell4],
        has_vehicle_on_path: false,
        speed_limit_reached: false,
        stopped_on_maneuver: false,
        stopped_speed_possible: false,
        blocking_cell_id: -1,
    };

    assert_eq!(
        observable_path.wanted_maneuver, correct_path.wanted_maneuver,
        "Incorrect maneuver for observable path"
    );

    assert_eq!(
        observable_path.last_cell_state, correct_path.last_cell_state,
        "Incorrect last cell state for observable path"
    );

    assert_eq!(
        observable_path.trimmed_path.len(),
        correct_path.trimmed_path.len(),
        "Incorrect number of cells vehicle can move forward"
    );

    for i in 0..observable_path.trimmed_path.len() {
        assert_eq!(
            observable_path.trimmed_path[i].get_id(),
            correct_path.trimmed_path[i].get_id(),
            "Incorrect cell at pos #{} in observable path",
            i
        );
    }

    // Add vehicle as obstacle
    current_state.insert(cell4.get_id(), 1);
    // Update path since it has been mutated in process_path function
    path = Path::new(
        vec![&cell1, &cell2, &cell3, &cell4, &cell5],
        vec![
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
            LaneChangeType::ChangeLeft,
        ],
        10.0,
    );
    let observable_path = process_path(&mut path, speed_limit, cell5.get_id(), &current_state);

    let correct_path = ObservablePath {
        wanted_maneuver: LaneChangeType::NoChange,
        last_cell_state: CellState::Free,
        trimmed_path: vec![&cell2, &cell3],
        has_vehicle_on_path: false,
        speed_limit_reached: false,
        stopped_on_maneuver: false,
        stopped_speed_possible: false,
        blocking_cell_id: -1,
    };

    assert_eq!(
        observable_path.wanted_maneuver, correct_path.wanted_maneuver,
        "Incorrect maneuver for observable path"
    );

    assert_eq!(
        observable_path.last_cell_state, correct_path.last_cell_state,
        "Incorrect last cell state for observable path"
    );

    assert_eq!(
        observable_path.trimmed_path.len(),
        correct_path.trimmed_path.len(),
        "Incorrect number of cells vehicle can move forward"
    );

    for i in 0..observable_path.trimmed_path.len() {
        assert_eq!(
            observable_path.trimmed_path[i].get_id(),
            correct_path.trimmed_path[i].get_id(),
            "Incorrect cell at pos #{} in observable path",
            i
        );
    }
    // Clean current states
    current_state.remove(&cell4.get_id());

    // Add maneuver in the middle of the path
    path = Path::new(
        vec![&cell1, &cell2, &cell3, &cell4, &cell5],
        vec![
            LaneChangeType::NoChange,
            LaneChangeType::ChangeLeft,
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
        ],
        10.0,
    );
    let observable_path = process_path(&mut path, speed_limit, cell5.get_id(), &current_state);

    let correct_path = ObservablePath {
        wanted_maneuver: LaneChangeType::NoChange,
        last_cell_state: CellState::Free,
        trimmed_path: vec![&cell2],
        has_vehicle_on_path: false,
        speed_limit_reached: false,
        stopped_on_maneuver: true,
        stopped_speed_possible: false,
        blocking_cell_id: -1,
    };

    assert_eq!(
        observable_path.wanted_maneuver, correct_path.wanted_maneuver,
        "Incorrect maneuver for observable path"
    );

    assert_eq!(
        observable_path.last_cell_state, correct_path.last_cell_state,
        "Incorrect last cell state for observable path"
    );

    assert_eq!(
        observable_path.trimmed_path.len(),
        correct_path.trimmed_path.len(),
        "Incorrect number of cells vehicle can move forward"
    );

    for i in 0..observable_path.trimmed_path.len() {
        assert_eq!(
            observable_path.trimmed_path[i].get_id(),
            correct_path.trimmed_path[i].get_id(),
            "Incorrect cell at pos #{} in observable path",
            i
        );
    }

    // Add manuever to the start of the path
    path = Path::new(
        vec![&cell1, &cell2, &cell3, &cell4, &cell5],
        vec![
            LaneChangeType::ChangeLeft,
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
        ],
        10.0,
    );
    let observable_path = process_path(&mut path, speed_limit, cell5.get_id(), &current_state);

    let correct_path = ObservablePath {
        wanted_maneuver: LaneChangeType::ChangeLeft,
        last_cell_state: CellState::Free,
        trimmed_path: vec![],
        has_vehicle_on_path: false,
        speed_limit_reached: false,
        stopped_on_maneuver: true,
        stopped_speed_possible: false,
        blocking_cell_id: -1,
    };

    assert_eq!(
        observable_path.wanted_maneuver, correct_path.wanted_maneuver,
        "Incorrect maneuver for observable path"
    );

    assert_eq!(
        observable_path.last_cell_state, correct_path.last_cell_state,
        "Incorrect last cell state for observable path"
    );

    assert_eq!(
        observable_path.trimmed_path.len(),
        correct_path.trimmed_path.len(),
        "Incorrect number of cells vehicle can move forward"
    );

    for i in 0..observable_path.trimmed_path.len() {
        assert_eq!(
            observable_path.trimmed_path[i].get_id(),
            correct_path.trimmed_path[i].get_id(),
            "Incorrect cell at pos #{} in observable path",
            i
        );
    }

    // Check speed restriction
    let vehicle_speed = 1;
    path = Path::new(
        vec![&cell1, &cell2, &cell3, &cell4, &cell5],
        vec![
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
        ],
        10.0,
    );
    let observable_path = process_path(&mut path, vehicle_speed, cell5.get_id(), &current_state);

    let correct_path = ObservablePath {
        wanted_maneuver: LaneChangeType::NoChange,
        last_cell_state: CellState::Free,
        trimmed_path: vec![&cell2],
        has_vehicle_on_path: false,
        speed_limit_reached: true,
        stopped_on_maneuver: false,
        stopped_speed_possible: false,
        blocking_cell_id: -1,
    };

    assert_eq!(
        observable_path.wanted_maneuver, correct_path.wanted_maneuver,
        "Incorrect maneuver for observable path"
    );

    assert_eq!(
        observable_path.last_cell_state, correct_path.last_cell_state,
        "Incorrect last cell state for observable path"
    );

    assert_eq!(
        observable_path.trimmed_path.len(),
        correct_path.trimmed_path.len(),
        "Incorrect number of cells vehicle can move forward"
    );

    for i in 0..observable_path.trimmed_path.len() {
        assert_eq!(
            observable_path.trimmed_path[i].get_id(),
            correct_path.trimmed_path[i].get_id(),
            "Incorrect cell at pos #{} in observable path",
            i
        );
    }

    // Check traffic light banned state
    cell3.set_state(CellState::Banned);
    path = Path::new(
        vec![&cell1, &cell2, &cell3, &cell4, &cell5],
        vec![
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
            LaneChangeType::NoChange,
        ],
        10.0,
    );
    let observable_path = process_path(&mut path, speed_limit, cell5.get_id(), &current_state);

    let correct_path = ObservablePath {
        wanted_maneuver: LaneChangeType::NoChange,
        last_cell_state: CellState::Banned,
        trimmed_path: vec![&cell2],
        has_vehicle_on_path: false,
        speed_limit_reached: false,
        stopped_on_maneuver: false,
        stopped_speed_possible: false,
        blocking_cell_id: -1,
    };

    assert_eq!(
        observable_path.wanted_maneuver, correct_path.wanted_maneuver,
        "Incorrect maneuver for observable path"
    );

    assert_eq!(
        observable_path.last_cell_state, correct_path.last_cell_state,
        "Incorrect last cell state for observable path"
    );

    assert_eq!(
        observable_path.trimmed_path.len(),
        correct_path.trimmed_path.len(),
        "Incorrect number of cells vehicle can move forward"
    );

    for i in 0..observable_path.trimmed_path.len() {
        assert_eq!(
            observable_path.trimmed_path[i].get_id(),
            correct_path.trimmed_path[i].get_id(),
            "Incorrect cell at pos #{} in observable path",
            i
        );
    }
}
