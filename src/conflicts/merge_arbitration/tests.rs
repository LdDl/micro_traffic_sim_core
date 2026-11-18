use super::*;
use crate::agents::VehicleIntention;
use crate::conflict_zones::ConflictEdge;
use crate::utils::rand::random_f64;

fn sample(mode: u8) -> (Option<VehicleID>, Vec<u64>) {
    let mut vehicles = IndexMap::new();
    for (id, source) in [(1, 100), (2, 200), (3, 200)] {
        let mut vehicle = Vehicle::new(id).with_cell(source).with_speed(1).build();
        vehicle.set_intention(VehicleIntention {
            intention_maneuver: crate::maneuver::LaneChangeType::NoChange,
            intention_cell_id: 900,
            intention_speed: 1,
            ..Default::default()
        });
        vehicles.insert(id, vehicle);
    }
    let claims: Vec<_> = vehicles
        .keys()
        .map(|&id| CellIntention::new(id, IntentionType::Target))
        .collect();
    let refs: Vec<_> = claims.iter().collect();
    let zone = ConflictZone::new(
        1,
        ConflictEdge {
            source: if mode == 1 { 999 } else { 100 },
            target: 900,
        },
        ConflictEdge {
            source: 200,
            target: 900,
        },
    )
    .with_winner_type(ConflictWinnerType::Equal)
    .build();
    let winner = match mode {
        0 => None,
        3 => Some(if random_bool(0.5) { 1 } else { 2 }),
        _ => match arbitrate(900, &refs, &[&zone], &vehicles).unwrap() {
            MergeDecision::Grant(winner) => Some(winner.get_vehicle_id()),
            MergeDecision::Unregulated => None,
            MergeDecision::Yield(_) => panic!("One two-approach zone cannot form a cycle"),
        },
    };
    (winner, (0..32).map(|_| random_f64().to_bits()).collect())
}

fn fresh_sample(mode: u8) -> (Option<VehicleID>, Vec<u64>) {
    std::thread::spawn(move || sample(mode)).join().unwrap()
}

#[test]
fn unrelated_merge_does_not_consume_randomness() {
    assert_eq!(fresh_sample(0), fresh_sample(1));
}

#[test]
fn equal_priority_uses_one_draw_for_every_participant_on_an_approach() {
    assert_eq!(fresh_sample(2), fresh_sample(3));
}
#[test]
fn pair_fast_path_matches_group_constraints_and_random_stream() {
    fn decision_id(d: MergeDecision<'_>) -> (u8, VehicleID) {
        match d {
            MergeDecision::Unregulated => (0, 0),
            MergeDecision::Grant(winner) => (1, winner.get_vehicle_id()),
            MergeDecision::Yield(candidate) => (2, candidate.get_vehicle_id()),
        }
    }
    fn run(pair: bool) -> Vec<((u8, VehicleID), u64)> {
        let paths = [
            vec![100, 900],
            vec![200, 900],
            vec![100, 800, 900],
            vec![100, 900, 200, 900],
            vec![200, 900, 100, 900],
        ];
        let priorities = [
            None,
            Some(ConflictWinnerType::First),
            Some(ConflictWinnerType::Second),
            Some(ConflictWinnerType::Equal),
            Some(ConflictWinnerType::Undefined),
        ];
        let a = CellIntention::new(1, IntentionType::Target);
        let b = CellIntention::new(2, IntentionType::Target);
        let mut records = Vec::new();
        for pa in &paths {
            for pb in &paths {
                let mut vehicles = IndexMap::new();
                for (id, path) in [(1, pa), (2, pb)] {
                    let mut v = Vehicle::new(id).with_cell(path[0]).with_speed(1).build();
                    v.set_intention(VehicleIntention {
                        intention_maneuver: crate::maneuver::LaneChangeType::NoChange,
                        intention_cell_id: 900,
                        intermediate_cells: path[1..path.len() - 1].to_vec(),
                        intention_speed: 1,
                        ..Default::default()
                    });
                    vehicles.insert(id, v);
                }
                for p in priorities {
                    for q in priorities {
                        let zones: Vec<_> = [(1, 100, 200, p), (2, 200, 100, q)]
                            .into_iter()
                            .filter_map(|(id, first, second, priority)| {
                                priority.map(|priority| {
                                    ConflictZone::new(
                                        id,
                                        ConflictEdge {
                                            source: first,
                                            target: 900,
                                        },
                                        ConflictEdge {
                                            source: second,
                                            target: 900,
                                        },
                                    )
                                    .with_winner_type(priority)
                                    .build()
                                })
                            })
                            .collect();
                        let refs: Vec<_> = zones.iter().collect();
                        let d = if pair {
                            arbitrate_pair(900, &a, &b, &refs, &vehicles)
                        } else {
                            arbitrate_group(900, &[&a, &b], &refs, &vehicles)
                        }
                        .unwrap();
                        records.push((decision_id(d), random_f64().to_bits()));
                    }
                }
            }
        }
        assert_eq!(records.len(), 625);
        records
    }
    let fast = std::thread::spawn(|| run(true)).join().unwrap();
    let general = std::thread::spawn(|| run(false)).join().unwrap();
    assert_eq!(fast, general);
}
