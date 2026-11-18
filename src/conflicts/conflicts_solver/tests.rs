use super::*;
use crate::agents::{Vehicle, VehicleIntention, VehiclesStorage};
use crate::behaviour::BehaviourType;
use crate::conflicts::conflict_types::{CellConflict, ConflictType};
use crate::grid::cell::CellID;
use crate::maneuver::LaneChangeType;

fn create_test_vehicle(
    id: u64,
    speed: i32,
    maneuver: LaneChangeType,
    intention_cell: CellID,
) -> Vehicle {
    let mut vehicle = Vehicle::new(id)
        .with_behaviour(BehaviourType::Undefined)
        .with_speed(speed)
        .build();
    vehicle.set_intention(VehicleIntention {
        intention_maneuver: maneuver,
        intention_cell_id: intention_cell,
        intention_speed: speed,
        ..Default::default()
    });
    vehicle
}

// Winning a tail conflict protects a stopped truck's body without moving its head to the
// contested tail cell.
#[test]
fn test_block_winner_stays_in_place() {
    // Truck: 8(tail) 9(tail) 10(head, stopped)
    // Another vehicle requests cell 9. Winning a Tail conflict protects
    // the truck's body. It must leave the head at 10 with zero speed.
    let mut truck = Vehicle::new(1)
        .with_behaviour(BehaviourType::Undefined)
        .with_cell(10)
        .with_tail_size(2, vec![8, 9])
        .with_speed(0)
        .build();
    truck.set_intention(VehicleIntention {
        intention_maneuver: LaneChangeType::Block,
        intention_cell_id: 10,
        intention_speed: 0,
        ..Default::default()
    });
    let other = create_test_vehicle(2, 1, LaneChangeType::NoChange, 9);
    let conflicts = vec![CellConflict {
        cell_id: 9,
        participants: vec![1, 2],
        priority_participant_index: 0,
        conflict_type: ConflictType::Tail,
    }];
    let mut vehicles = VehiclesStorage::new();
    vehicles.insert(1, truck);
    vehicles.insert(2, other);
    solve_conflicts(conflicts, &mut vehicles, &LocalLogger::none())
        .expect("solve_conflicts failed");
    let truck = vehicles.get(&1).unwrap();
    assert_eq!(
        truck.intention.intention_cell_id, 10,
        "Block winner must stay on its head cell"
    );
    assert_eq!(
        truck.intention.intention_speed, 0,
        "Block winner must keep zero speed"
    );
}

// An empty conflict list and vehicle store resolve successfully.
#[test]
fn test_solve_conflicts_empty_conflicts_list() {
    let conflicts: Vec<CellConflict> = vec![];
    let mut vehicles = VehiclesStorage::new();
    let result = solve_conflicts(conflicts, &mut vehicles, &LocalLogger::none());
    assert!(
        result.is_ok(),
        "solve_conflicts should handle empty conflicts list"
    );
}

// A one-participant conflict returns InsufficientParticipants with the contested cell ID.
#[test]
fn test_solve_conflicts_insufficient_participants() {
    let vehicle = create_test_vehicle(1, 2, LaneChangeType::NoChange, 5);

    let conflicts = vec![CellConflict {
        cell_id: 5,
        participants: vec![1],
        priority_participant_index: 0,
        conflict_type: ConflictType::MergeForward,
    }];

    let mut vehicles = VehiclesStorage::new();
    vehicles.insert(1, vehicle);
    let result = solve_conflicts(conflicts, &mut vehicles, &LocalLogger::none());
    assert!(
        result.is_err(),
        "solve_conflicts should return error for conflicts with insufficient participants"
    );

    match result {
        Err(ConflictSolverError::InsufficientParticipants(cell_id, _)) => {
            assert_eq!(cell_id, 5, "Error should contain correct cell ID");
        }
        _ => panic!("Expected InsufficientParticipants error"),
    }
}

// An out-of-range priority index returns the cell ID, invalid index, and participant
// count.
#[test]
fn test_solve_conflicts_invalid_priority_index() {
    let vehicle1 = create_test_vehicle(1, 2, LaneChangeType::NoChange, 5);
    let vehicle2 = create_test_vehicle(2, 2, LaneChangeType::NoChange, 5);

    let conflicts = vec![CellConflict {
        cell_id: 5,
        participants: vec![1, 2],
        // Invalid index (only 0 and 1 are valid)
        priority_participant_index: 5,
        conflict_type: ConflictType::MergeForward,
    }];

    let mut vehicles = VehiclesStorage::new();
    vehicles.insert(1, vehicle1);
    vehicles.insert(2, vehicle2);
    let result = solve_conflicts(conflicts, &mut vehicles, &LocalLogger::none());
    assert!(
        result.is_err(),
        "solve_conflicts should return error for invalid priority index"
    );

    match result {
        Err(ConflictSolverError::InvalidPriorityIndex(cell_id, index, len)) => {
            assert_eq!(cell_id, 5);
            assert_eq!(index, 5);
            assert_eq!(len, 2);
        }
        _ => panic!("Expected InvalidPriorityIndex error"),
    }
}

// The nominated lane changer receives a one-cell move to the contested target while the
// forward-moving opponent stops.
#[test]
fn test_solve_conflicts_common_conflict() {
    let priority_vehicle = create_test_vehicle(1, 3, LaneChangeType::ChangeLeft, 5);
    let blocked_vehicle = create_test_vehicle(2, 2, LaneChangeType::NoChange, 5);

    let conflicts = vec![CellConflict {
        cell_id: 5,
        participants: vec![1, 2],
        // First vehicle (index 0) has priority
        priority_participant_index: 0,
        conflict_type: ConflictType::ForwardLaneChange,
    }];

    let mut vehicles = VehiclesStorage::new();
    vehicles.insert(1, priority_vehicle);
    vehicles.insert(2, blocked_vehicle);
    let result = solve_conflicts(conflicts, &mut vehicles, &LocalLogger::none());
    assert!(
        result.is_ok(),
        "solve_conflicts should handle common conflicts"
    );

    // After solving, we can check the vehicle states
    assert_eq!(
        vehicles.get(&1).unwrap().intention.intention_cell_id,
        5,
        "Priority vehicle should have target cell"
    );
    assert_eq!(
        vehicles.get(&1).unwrap().intention.intention_speed,
        1,
        "Priority vehicle should have speed 1"
    );
    assert_eq!(
        vehicles.get(&2).unwrap().intention.intention_speed,
        0,
        "Blocked vehicle should have speed 0"
    );
}

// For the supplied crossing constraint, the nominated left changer proceeds one cell and
// the right changer stops.
#[test]
fn test_solve_conflicts_trajectories_conflict() {
    let left_vehicle = create_test_vehicle(1, 2, LaneChangeType::ChangeLeft, 5);
    let right_vehicle = create_test_vehicle(2, 2, LaneChangeType::ChangeRight, 6);

    let conflicts = vec![CellConflict {
        // Trajectories conflict has no specific cell
        cell_id: -1,
        participants: vec![1, 2],
        // Left vehicle has priority
        priority_participant_index: 0,
        conflict_type: ConflictType::CrossLaneChange,
    }];

    let mut vehicles = VehiclesStorage::new();
    vehicles.insert(1, left_vehicle);
    vehicles.insert(2, right_vehicle);
    let result = solve_conflicts(conflicts, &mut vehicles, &LocalLogger::none());
    assert!(
        result.is_ok(),
        "solve_conflicts should handle trajectories conflicts"
    );

    // Check that right vehicle is blocked (right maneuver loses to left maneuver)
    assert_eq!(
        vehicles.get(&2).unwrap().intention.intention_speed,
        0,
        "Right vehicle should be blocked"
    );
    assert_eq!(
        vehicles.get(&1).unwrap().intention.intention_speed,
        1,
        "Left vehicle should continue"
    );
}

// The solver honors a supplied right-changing priority winner. Aggression selection
// belongs to the earlier conflict calculation.
#[test]
fn test_solve_conflicts_cross_lane_aggressor_priority() {
    let left_vehicle = create_test_vehicle(1, 2, LaneChangeType::ChangeLeft, 5);
    let right_vehicle = create_test_vehicle(2, 2, LaneChangeType::ChangeRight, 6);
    let conflicts = vec![CellConflict {
        cell_id: -1,
        participants: vec![1, 2],
        // right (vehicle 2) is the priority winner (aggressor)
        priority_participant_index: 1,
        conflict_type: ConflictType::CrossLaneChange,
    }];
    let mut vehicles = VehiclesStorage::new();
    vehicles.insert(1, left_vehicle);
    vehicles.insert(2, right_vehicle);
    let result = solve_conflicts(conflicts, &mut vehicles, &LocalLogger::none());
    assert!(result.is_ok());
    assert_ne!(
        vehicles.get(&2).unwrap().intention.intention_speed,
        0,
        "right maneuver (priority winner) proceeds"
    );
    assert_eq!(
        vehicles.get(&1).unwrap().intention.intention_speed,
        0,
        "left maneuver yields to the priority winner"
    );
}

// The yielding vehicle's intention returns to its current cell with zero speed and no
// intermediate cells.
#[test]
fn test_solve_conflicts_conflict_zone() {
    let priority_vehicle = create_test_vehicle(1, 2, LaneChangeType::NoChange, 5);
    let blocked_vehicle = create_test_vehicle(2, 2, LaneChangeType::NoChange, 6);

    let conflicts = vec![CellConflict {
        cell_id: 10,
        participants: vec![1, 2],
        // First vehicle has priority
        priority_participant_index: 0,
        conflict_type: ConflictType::CrossConflictZone,
    }];

    let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
    vehicles.insert(1, priority_vehicle);
    vehicles.insert(2, blocked_vehicle);
    let result = solve_conflicts(conflicts, &mut vehicles, &LocalLogger::none());
    assert!(
        result.is_ok(),
        "solve_conflicts should handle conflict zone conflicts"
    );

    // A yielding vehicle remains in its current cell with zero displacement.
    let blocked = vehicles.get(&2).unwrap();
    assert_eq!(blocked.intention.intention_cell_id, blocked.cell_id);
    assert_eq!(blocked.intention.intention_speed, 0);
    assert!(blocked.intention.intermediate_cells.is_empty());
}

// Two independent contested targets retain their respective priority winners and stop the
// other participants.
#[test]
fn test_solve_conflicts_multiple_conflicts() {
    let vehicle1 = create_test_vehicle(1, 2, LaneChangeType::ChangeLeft, 5);
    let vehicle2 = create_test_vehicle(2, 2, LaneChangeType::NoChange, 5);
    let vehicle3 = create_test_vehicle(3, 1, LaneChangeType::ChangeRight, 7);
    let vehicle4 = create_test_vehicle(4, 1, LaneChangeType::NoChange, 7);

    let conflicts = vec![
        CellConflict {
            cell_id: 5,
            participants: vec![1, 2],
            // Vehicle 1 has priority
            priority_participant_index: 0,
            conflict_type: ConflictType::ForwardLaneChange,
        },
        CellConflict {
            cell_id: 7,
            participants: vec![3, 4],
            // Vehicle 4 has priority (index 1)
            priority_participant_index: 1,
            conflict_type: ConflictType::ForwardLaneChange,
        },
    ];

    let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
    vehicles.insert(1, vehicle1);
    vehicles.insert(2, vehicle2);
    vehicles.insert(3, vehicle3);
    vehicles.insert(4, vehicle4);
    let result = solve_conflicts(conflicts, &mut vehicles, &LocalLogger::none());
    assert!(
        result.is_ok(),
        "solve_conflicts should handle multiple conflicts"
    );

    // Check first conflict resolution
    assert_eq!(
        vehicles.get(&1).unwrap().intention.intention_cell_id,
        5,
        "Vehicle 1 should get target cell"
    );
    assert_eq!(
        vehicles.get(&1).unwrap().intention.intention_speed,
        1,
        "Vehicle 1 should have speed 1"
    );
    assert_eq!(
        vehicles.get(&2).unwrap().intention.intention_speed,
        0,
        "Vehicle 2 should be blocked"
    );

    // Check second conflict resolution
    assert_eq!(
        vehicles.get(&3).unwrap().intention.intention_speed,
        0,
        "Vehicle 3 should be blocked"
    );
    assert_eq!(
        vehicles.get(&4).unwrap().intention.intention_speed,
        1,
        "Vehicle 4 should keep its original speed (has priority, NoChange)"
    );
}

// A desperate winner reserves cell 20 first. The normal winner's two-cell path is
// shortened to cell 11 before that reservation.
#[test]
fn test_normal_multicell_winner_does_not_sweep_claimed_cell() {
    // desperate grant:              20
    // normal requested prefix: 11 -> 20
    // normal granted prefix:   11
    // The input lists the normal conflict first.
    let log = LocalLogger::none();
    let mut vehicles = VehiclesStorage::new();

    // Desperate winner of conflict on cell 20 -> reserves cell 20.
    let mut d = create_test_vehicle(1, 1, LaneChangeType::NoChange, 20);
    d.wait_steps = 9999;
    vehicles.insert(1, d);
    // loser of that conflict
    vehicles.insert(2, create_test_vehicle(2, 1, LaneChangeType::NoChange, 20));

    // Normal priority winner with a 2-cell move whose head is the reserved cell 20.
    let mut w = Vehicle::new(3)
        .with_behaviour(BehaviourType::Undefined)
        .with_speed(2)
        .build();
    w.set_intention(VehicleIntention {
        intention_maneuver: LaneChangeType::NoChange,
        intention_cell_id: 20,
        intermediate_cells: vec![11],
        intention_speed: 2,
        ..Default::default()
    });
    vehicles.insert(3, w);
    // loser of conflict on 11
    vehicles.insert(4, create_test_vehicle(4, 1, LaneChangeType::NoChange, 11));

    // The normal conflict is listed FIRST to prove desperate-first reserves 20 before it.
    let conflicts = vec![
        CellConflict {
            cell_id: 11,
            participants: vec![3, 4],
            priority_participant_index: 0,
            conflict_type: ConflictType::MergeForward,
        },
        CellConflict {
            cell_id: 20,
            participants: vec![1, 2],
            priority_participant_index: 0,
            conflict_type: ConflictType::MergeForward,
        },
    ];
    solve_conflicts(conflicts, &mut vehicles, &log).unwrap();

    let w = vehicles.get(&3).unwrap();
    assert_eq!(
        w.intention.intention_cell_id, 11,
        "winner clamped to stop before the reserved cell"
    );
    assert!(
        w.intention.intermediate_cells.is_empty(),
        "no multi-cell sweep through the reserved cell"
    );
    assert_eq!(w.intention.intention_speed, 1, "single-cell step");
    assert_ne!(
        vehicles.get(&1).unwrap().intention.intention_speed,
        0,
        "desperate vehicle keeps the reserved cell 20"
    );
}

#[path = "tests/desperate.rs"]
mod desperate;
