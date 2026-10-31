use super::*;
use crate::agents::{TailIntentionManeuver, Vehicle, VehicleIntention};
use crate::conflict_zones::{ConflictEdge, ConflictWinnerType, ConflictZone};
use crate::grid::cell::Cell;
use crate::grid::road_network::GridRoads;
use crate::intentions::{IntentionType, Intentions};
use crate::maneuver::LaneChangeType;
use crate::utils::rand::random_f64;
use crate::verbose::{LocalLogger, VerboseLevel};
use indexmap::IndexMap;
use std::collections::HashMap;

fn zone(id: i32, a: i64, b: i64, priority: ConflictWinnerType) -> ConflictZone {
    ConflictZone::new(
        id,
        ConflictEdge {
            source: a - 1,
            target: a,
        },
        ConflictEdge {
            source: b - 1,
            target: b,
        },
    )
    .with_winner_type(priority)
    .build()
}

fn intended(target: i64) -> Intentions {
    let mut car = Vehicle::new(1).with_cell(target - 1).build();
    car.set_intention(VehicleIntention {
        intention_cell_id: target,
        ..Default::default()
    });
    let mut intentions = Intentions::new();
    intentions.add_intention(&mut car, IntentionType::Target);
    intentions
}

#[test]
fn crossing_index_keeps_all_shared_targets_and_replaces_incidence() {
    let mut index = CrossingZoneIndex::default();
    index.replace(&zone(10, 4, 104, ConflictWinnerType::First));
    index.replace(&zone(3, 4, 204, ConflictWinnerType::Second));
    assert_eq!(index.candidates(&intended(4)), vec![3, 10]);
    assert_eq!(index.candidates(&intended(104)), vec![10]);
    index.replace(&zone(10, 304, 404, ConflictWinnerType::Equal));
    assert_eq!(index.candidates(&intended(4)), vec![3]);
    assert!(index.candidates(&intended(104)).is_empty());
    assert_eq!(index.candidates(&intended(404)), vec![10]);
    index.replace(&zone(3, 4, 4, ConflictWinnerType::First));
    assert!(index.candidates(&intended(4)).is_empty());
    assert!(index.candidates(&intended(204)).is_empty());
    index.replace(&zone(3, 104, 204, ConflictWinnerType::Undefined));
    assert_eq!(index.candidates(&intended(104)), vec![3]);
    index.replace(&zone(3, 104, 204, ConflictWinnerType::First));
    assert_eq!(index.candidates(&intended(104)), vec![3]);
}

fn collect_fixture(indexed: bool, case: usize, order: usize) -> String {
    let mut net = GridRoads::new();
    for base in [0, 100, 200, 300, 400] {
        for cell in base..base + 10 {
            net.add_cell(
                Cell::new(cell)
                    .with_forward_node(if cell % 100 == 9 { -1 } else { cell + 1 })
                    .with_left_node(-1)
                    .with_right_node(-1)
                    .with_speed_limit(5)
                    .build(),
            );
        }
    }
    let mut zones = HashMap::new();
    let mut cells = HashMap::new();
    let mut index = CrossingZoneIndex::default();
    let mut add = |zone: ConflictZone| {
        index.replace(&zone);
        for target in [zone.get_first_edge().target, zone.get_second_edge().target] {
            if target >= 0 {
                cells.insert(target, zone.get_id());
            }
        }
        zones.insert(zone.get_id(), zone);
    };
    let priority = match case % 4 {
        0 => ConflictWinnerType::First,
        1 => ConflictWinnerType::Second,
        2 => ConflictWinnerType::Equal,
        _ => ConflictWinnerType::Undefined,
    };
    for id in if order == 0 {
        vec![20, 2, 7]
    } else {
        vec![7, 2, 20]
    } {
        let (a, b) = match id {
            2 => (4, 104),
            7 => (4, 204),
            _ => (5, 304),
        };
        add(zone(id, a, b, priority.clone()));
    }
    match case / 4 {
        1 => add(zone(2, 304, 404, priority.clone())),
        2 => add(zone(2, 4, 4, priority.clone())),
        3 => {
            add(zone(2, 4, 4, priority.clone()));
            add(zone(2, 104, 304, priority.clone()));
        }
        4 => add(zone(2, 404, 104, priority.clone())),
        _ => {}
    }
    // Inactive zones deliberately interleave their IDs with active ones.
    for id in [-10, 1, 3, 6, 8, 30] {
        add(zone(
            id,
            1000 + i64::from(id) * 2,
            2000 + i64::from(id) * 2,
            ConflictWinnerType::Equal,
        ));
    }
    let vehicle_ids = if order == 0 {
        vec![1, 2, 3, 4]
    } else {
        vec![4, 2, 1, 3]
    };
    let mut vehicles = IndexMap::new();
    let mut intentions = Intentions::new();
    for id in vehicle_ids {
        let base = (id - 1) as i64 * 100;
        let mut vehicle = Vehicle::new(id).with_cell(base + 2).with_speed(3).build();
        vehicle.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_speed: 3,
            intention_cell_id: base + 5,
            intermediate_cells: vec![base + 3, base + 4],
            ..Default::default()
        });
        if case / 4 == 5 && id == 2 {
            vehicle.intention.tail_intention_cells = vec![104];
            vehicle.intention.tail_maneuver = TailIntentionManeuver {
                source_cell_maneuver: 103,
                target_cell_maneuver: 104,
                intention_maneuver: LaneChangeType::NoChange,
            };
            intentions.add_intention(&mut vehicle, IntentionType::Tail);
        }
        intentions.add_intention(&mut vehicle, IntentionType::Target);
        intentions.add_intention(&mut vehicle, IntentionType::Transit);
        vehicles.insert(id, vehicle);
    }
    if case / 4 == 6 {
        vehicles.shift_remove(&2);
    }
    if case / 4 == 7 {
        cells.insert(5, 999999);
    }
    let logger = LocalLogger::new(VerboseLevel::None);
    let result = if indexed {
        collect_conflicts_indexed(
            &intentions,
            &net,
            &zones,
            &cells,
            &logger,
            &mut vehicles,
            &index,
        )
    } else {
        collect_conflicts(&intentions, &net, &zones, &cells, &logger, &mut vehicles)
    };
    let mut output = format!("{result:?}");
    if let Ok(conflicts) = result {
        output.push_str(&format!(
            ";solver={:?}",
            solve_conflicts(conflicts, &mut vehicles, &logger)
        ));
    }
    for (id, vehicle) in vehicles {
        output.push_str(&format!(
            ";{id};{};{:?}",
            vehicle.is_conflict_participant, vehicle.intention
        ));
    }
    // Fresh threads have identical initial seeded streams; the sentinel checks draw count and order.
    for _ in 0..16 {
        output.push_str(&format!(";{:016x}", random_f64().to_bits()));
    }
    output
}

#[test]
fn indexed_collection_matches_full_scan_constraints_errors_solver_and_rng() {
    for case in 0..32 {
        for order in 0..2 {
            let old = std::thread::spawn(move || collect_fixture(false, case, order))
                .join()
                .unwrap();
            let new = std::thread::spawn(move || collect_fixture(true, case, order))
                .join()
                .unwrap();
            assert_eq!(old, new, "case {case}, order {order}");
        }
    }
}

fn malformed_fixture(indexed: bool, case: usize) -> String {
    let mut net = GridRoads::new();
    net.add_cell(
        Cell::new(4)
            .with_forward_node(5)
            .with_left_node(-1)
            .with_right_node(-1)
            .build(),
    );
    let target = if case == 1 { -1 } else { 4 };
    let mut vehicle = Vehicle::new(1).with_cell(3).build();
    vehicle.set_intention(VehicleIntention {
        intention_cell_id: target,
        ..Default::default()
    });
    let mut intentions = Intentions::new();
    intentions.add_intention(&mut vehicle, IntentionType::Target);
    let mut vehicles = IndexMap::new();
    if case >= 2 {
        vehicles.insert(1, vehicle);
    }
    let mut zones = HashMap::new();
    let mut cells = HashMap::new();
    let mut index = CrossingZoneIndex::default();
    if case != 2 {
        let first = if case == 3 { 14 } else { target };
        let z = zone(1, first, 104, ConflictWinnerType::Equal);
        index.replace(&z);
        zones.insert(1, z);
        cells.insert(target, 1);
    } else {
        cells.insert(target, 99);
    }
    let logger = LocalLogger::none();
    let result = if indexed {
        collect_conflicts_indexed(
            &intentions,
            &net,
            &zones,
            &cells,
            &logger,
            &mut vehicles,
            &index,
        )
    } else {
        collect_conflicts(&intentions, &net, &zones, &cells, &logger, &mut vehicles)
    };
    match case {
        0 | 1 => assert!(
            matches!(&result, Err(ConflictError::InvalidVehicle(message)) if message == "Vehicle 1 not found")
        ),
        2 => assert!(
            matches!(&result, Err(ConflictError::InvalidVehicle(message)) if message == "Conflict zone 99 not found")
        ),
        3 => assert!(matches!(&result, Ok(conflicts) if conflicts.is_empty())),
        _ => unreachable!(),
    }
    let mut output = format!("{result:?}");
    for _ in 0..16 {
        output.push_str(&format!(";{:016x}", random_f64().to_bits()));
    }
    output
}

#[test]
fn indexed_one_sided_invalid_and_stale_references_keep_full_scan_behavior() {
    for case in 0..4 {
        let old = std::thread::spawn(move || malformed_fixture(false, case))
            .join()
            .unwrap();
        let new = std::thread::spawn(move || malformed_fixture(true, case))
            .join()
            .unwrap();
        assert_eq!(old, new, "case {case}");
    }
}
