use micro_traffic_sim_core::agents::{Vehicle, VehicleIntention};
use micro_traffic_sim_core::conflict_zones::{ConflictEdge, ConflictWinnerType, ConflictZone};
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::simulation::session::Session;

#[test]
fn a_later_patience_conflict_cannot_give_a_stopped_vehicle_positive_speed() {
    let mut s = Session::default(None);
    s.add_cells(
        [
            (10, 30),
            (20, 30),
            (30, 31),
            (31, 32),
            (32, -1),
            (40, 50),
            (50, 51),
            (51, -1),
        ]
        .into_iter()
        .map(|(id, next)| {
            Cell::new(id)
                .with_forward_node(next)
                .with_speed_limit(1)
                .build()
        })
        .collect(),
    );
    s.add_conflict_zone(
        ConflictZone::new(
            1,
            ConflictEdge {
                source: 10,
                target: 30,
            },
            ConflictEdge {
                source: 40,
                target: 50,
            },
        )
        .with_winner_type(ConflictWinnerType::Second)
        .build(),
    );
    s.add_vehicles(
        [(1, 10, 32, 10000), (2, 20, 32, 20000), (3, 40, 51, 0)]
            .into_iter()
            .map(|(id, from, destination, wait)| {
                let mut v = Vehicle::new(id)
                    .with_cell(from)
                    .with_destination(destination)
                    .with_speed(1)
                    .with_speed_limit(1)
                    .with_slowdown(0.0)
                    .with_slow_to_start(0.0)
                    .build();
                v.wait_steps = wait;
                v.stop_ticks = 5;
                v
            })
            .collect(),
    );
    s.step().unwrap();
    let held = &s.get_vehicles()[&1];
    assert_eq!(held.cell_id, 10);
    assert_eq!(held.speed, 0, "a denied move must remain an actual stop");
    assert_eq!(
        held.stop_ticks, 6,
        "later priority cannot erase the completed standing tick"
    );
}

#[test]
fn winner_metadata_cannot_redirect_a_lane_change() {
    use indexmap::IndexMap;
    use micro_traffic_sim_core::conflicts::{solve_conflicts, CellConflict, ConflictType};
    use micro_traffic_sim_core::maneuver::LaneChangeType;
    use micro_traffic_sim_core::verbose::LocalLogger;
    let mut cars = IndexMap::new();
    for id in 1..=2 {
        let mut v = Vehicle::new(id)
            .with_cell(id as i64 * 10)
            .with_speed(1)
            .build();
        v.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeLeft,
            intention_cell_id: 30,
            intention_speed: 1,
            ..Default::default()
        });
        cars.insert(id, v);
    }
    solve_conflicts(
        vec![CellConflict {
            cell_id: 99,
            participants: vec![1, 2],
            priority_participant_index: 0,
            conflict_type: ConflictType::MergeLaneChange,
        }],
        &mut cars,
        &LocalLogger::none(),
    )
    .unwrap();
    assert_eq!(
        cars[&1].intention.intention_cell_id, 30,
        "the restriction operation accepts no replacement destination"
    );
}

fn entered(v: &Vehicle) -> Vec<i64> {
    if v.intention.intention_cell_id == v.cell_id {
        Vec::new()
    } else {
        v.intention
            .intermediate_cells
            .iter()
            .copied()
            .chain(std::iter::once(v.intention.intention_cell_id))
            .collect()
    }
}

#[test]
fn restriction_sequences_preserve_paths_speed_and_materialized_bodies() {
    use micro_traffic_sim_core::maneuver::LaneChangeType;
    let maneuvers = [
        LaneChangeType::NoChange,
        LaneChangeType::ChangeLeft,
        LaneChangeType::ChangeRight,
    ];
    let mut sequences = 0;
    for maneuver in maneuvers {
        for length in 0..=6usize {
            if maneuver != LaneChangeType::NoChange && length > 1 {
                continue;
            }
            for tail_size in 0..=3usize {
                let tail: Vec<i64> = (20..20 + tail_size as i64).collect();
                let make_vehicle = || {
                    let mut original = Vehicle::new(1)
                        .with_cell(10)
                        .with_tail_size(tail_size, tail.clone())
                        .with_speed(length as i32)
                        .build();
                    let path: Vec<i64> = (0..length as i64).collect();
                    original.set_intention(VehicleIntention {
                        intention_cell_id: path.last().copied().unwrap_or(10),
                        intention_speed: length as i32,
                        intention_maneuver: maneuver,
                        intermediate_cells: path
                            .iter()
                            .copied()
                            .take(length.saturating_sub(1))
                            .collect(),
                        ..Default::default()
                    });
                    original
                };
                for word in 0..216usize {
                    let mut v = make_vehicle();
                    let mut operations = word;
                    for _ in 0..3 {
                        let before = entered(&v);
                        let speed_before = v.intention.intention_speed;
                        match operations % 6 {
                            0 => v.block_with_speed(0),
                            1 => v.restrict_intention_to_first_step(),
                            2 => v.trim_forward_intention(0),
                            3 => v.trim_forward_intention(1),
                            4 => v.trim_forward_intention(2),
                            _ => v.trim_forward_intention(7),
                        }
                        operations /= 6;
                        let after = entered(&v);
                        assert!(before.starts_with(&after));
                        assert!(v.intention.intention_speed <= speed_before);
                        assert_eq!(v.intention.intention_speed as usize, after.len());
                        assert_eq!(v.cell_id, 10);
                        assert_eq!(v.tail_cells, tail);
                        if !after.is_empty() && after.len() < before.len() {
                            let body: Vec<i64> = tail
                                .iter()
                                .copied()
                                .chain(std::iter::once(10))
                                .chain(after.iter().copied())
                                .collect();
                            let expected_tail = &body[body.len() - tail_size - 1..body.len() - 1];
                            assert_eq!(v.intention.tail_intention_cells, expected_tail);
                        }
                    }
                    sequences += 1;
                }
            }
        }
    }
    assert_eq!(sequences, 9504);
}
