use indexmap::IndexMap;
use micro_traffic_sim_core::agents::{Vehicle, VehicleIntention};
use micro_traffic_sim_core::conflict_zones::{ConflictEdge, ConflictWinnerType, ConflictZone};
use micro_traffic_sim_core::conflicts::{ConflictType, new_conflict_multiple, solve_conflicts};
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::intentions::{CellIntention, IntentionType};
use micro_traffic_sim_core::maneuver::LaneChangeType;
use micro_traffic_sim_core::verbose::LocalLogger;
use std::collections::HashMap;

fn copy_fixture(vehicles: &IndexMap<u64, Vehicle>) -> IndexMap<u64, Vehicle> {
    vehicles
        .iter()
        .map(|(&id, v)| {
            let mut copy = Vehicle::new(id)
                .with_cell(v.cell_id)
                .with_speed(v.speed)
                .build();
            copy.set_intention(v.intention.clone());
            copy.wait_steps = v.wait_steps;
            copy.yield_cycle_delay = v.yield_cycle_delay;
            copy.yield_cycle_wait_steps = v.yield_cycle_wait_steps;
            (id, copy)
        })
        .collect()
}

fn car(id: u64, from: i64, path: &[i64]) -> Vehicle {
    let mut v = Vehicle::new(id)
        .with_cell(from)
        .with_speed(path.len() as i32)
        .build();
    v.set_intention(VehicleIntention {
        intention_maneuver: LaneChangeType::NoChange,
        intention_cell_id: *path.last().unwrap(),
        intention_speed: path.len() as i32,
        intermediate_cells: path[..path.len() - 1].to_vec(),
        ..Default::default()
    });
    v
}
fn zone(id: i32, a: i64, b: i64, winner: ConflictWinnerType) -> ConflictZone {
    ConflictZone::new(
        id,
        ConflictEdge {
            source: a,
            target: 900,
        },
        ConflictEdge {
            source: b,
            target: 900,
        },
    )
    .with_winner_type(winner)
    .build()
}
fn conflict(
    vehicles: &IndexMap<u64, Vehicle>,
    zones: &HashMap<i32, ConflictZone>,
    order: &[u64],
) -> micro_traffic_sim_core::conflicts::CellConflict {
    let claims: Vec<_> = order
        .iter()
        .map(|&id| CellIntention::new(id, IntentionType::Target))
        .collect();
    let legacy = zones
        .keys()
        .max()
        .map(|&id| (900, id))
        .into_iter()
        .collect();
    new_conflict_multiple(&Cell::new(900).build(), zones, &legacy, &claims, vehicles).unwrap()
}
#[test]
fn absent_priority_approach_cannot_supply_a_winner() {
    // 326(empty)   464(44)   627(54)
    //        \        |        /
    //                  900
    // The zone prefers 326 over 464, but no vehicle arrives from 326.
    // The winner must be an actual participant: vehicle 44 or 54.
    let vehicles = IndexMap::from([(44, car(44, 464, &[900])), (54, car(54, 627, &[900]))]);
    let zones = HashMap::from([(1, zone(1, 326, 464, ConflictWinnerType::First))]);
    let c = conflict(&vehicles, &zones, &[44, 54]);
    assert_eq!(c.participants[c.priority_participant_index], 44);
    assert_ne!(c.conflict_type, ConflictType::MergeForwardConflictZone);
}
#[test]
fn a_source_elsewhere_in_the_path_is_not_the_incoming_edge() {
    // 44: 464, 326, 800, 900
    // 54: 627, 326,      900
    // The zone prefers incoming edge (326, 900) over (800, 900).
    // Only vehicle 54 uses the preferred edge, although both visit 326.
    let vehicles = IndexMap::from([
        (44, car(44, 464, &[326, 800, 900])),
        (54, car(54, 627, &[326, 900])),
    ]);
    let zones = HashMap::from([(1, zone(1, 326, 800, ConflictWinnerType::First))]);
    for order in [[44, 54], [54, 44]] {
        let c = conflict(&vehicles, &zones, &order);
        assert_eq!(c.participants[c.priority_participant_index], 54);
        assert_eq!(c.conflict_type, ConflictType::MergeForwardConflictZone);
    }
}
#[test]
fn all_shared_target_zones_constrain_a_three_vehicle_merge() {
    // 100(A)  200(B)  300(C)
    //     \      |      /
    //           900
    // C has priority over B, and B over A. Both zones must participate,
    // so C wins regardless of the order in which claims are collected.
    let vehicles = IndexMap::from([
        (1, car(1, 100, &[900])),
        (2, car(2, 200, &[900])),
        (3, car(3, 300, &[900])),
    ]);
    let zones = HashMap::from([
        (1, zone(1, 300, 200, ConflictWinnerType::First)),
        (2, zone(2, 200, 100, ConflictWinnerType::First)),
    ]);
    for order in [
        [1, 2, 3],
        [1, 3, 2],
        [2, 1, 3],
        [2, 3, 1],
        [3, 1, 2],
        [3, 2, 1],
    ] {
        let c = conflict(&vehicles, &zones, &order);
        assert_eq!(c.participants[c.priority_participant_index], 3);
    }
}

#[test]
fn cyclic_priority_yields_until_its_own_cycle_delay() {
    let original = IndexMap::from([
        (1, car(1, 100, &[900])),
        (2, car(2, 200, &[900])),
        (3, car(3, 300, &[900])),
    ]);
    let zones = HashMap::from([
        (1, zone(1, 100, 200, ConflictWinnerType::First)),
        (2, zone(2, 200, 300, ConflictWinnerType::First)),
        (3, zone(3, 300, 100, ConflictWinnerType::First)),
    ]);
    for ready in [false, true] {
        let mut vehicles = copy_fixture(&original);
        if ready {
            for v in vehicles.values_mut() {
                v.yield_cycle_wait_steps = v.yield_cycle_delay;
            }
        }
        let c = conflict(&vehicles, &zones, &[3, 1, 2]);
        assert_eq!(c.conflict_type, ConflictType::MutualYield);
        solve_conflicts(vec![c], &mut vehicles, &LocalLogger::none()).unwrap();
        for v in vehicles.values() {
            assert_eq!(
                v.intention.intention_cell_id,
                if ready && v.id == 1 { 900 } else { v.cell_id }
            );
        }
    }
}

#[test]
fn every_four_approach_priority_graph_matches_an_independent_oracle() {
    let vehicles = IndexMap::from([
        (1, car(1, 100, &[900])),
        (2, car(2, 200, &[900])),
        (3, car(3, 300, &[900])),
        (4, car(4, 400, &[900])),
    ]);
    let mut orders = Vec::new();
    for a in 1..=4 {
        for b in 1..=4 {
            for c in 1..=4 {
                for d in 1..=4 {
                    let order = [a, b, c, d];
                    if (0..4).all(|i| (i + 1..4).all(|j| order[i] != order[j])) {
                        orders.push(order);
                    }
                }
            }
        }
    }
    let edges: Vec<_> = (1..=4)
        .flat_map(|a| (1..=4).filter(move |&b| a != b).map(move |b| (a, b)))
        .collect();
    let mut checked = 0;
    for mask in 0..(1 << 12) {
        let selected: Vec<_> = edges
            .iter()
            .enumerate()
            .filter(|(i, _)| mask & (1 << i) != 0)
            .collect();
        let expected = (1..=4).find(|id| !selected.iter().any(|(_, (_, loser))| loser == id));
        for reverse in [false, true] {
            let mut zones = HashMap::new();
            let mut ordered = selected.clone();
            if reverse {
                ordered.reverse();
            }
            for (i, &(winner, loser)) in ordered {
                zones.insert(
                    i as i32,
                    zone(
                        i as i32,
                        winner * 100,
                        loser * 100,
                        ConflictWinnerType::First,
                    ),
                );
            }
            for order in &orders {
                let c = conflict(&vehicles, &zones, order);
                if let Some(id) = expected {
                    assert_ne!(c.conflict_type, ConflictType::MutualYield);
                    assert_eq!(
                        c.participants[c.priority_participant_index], id as u64,
                        "mask={mask},order={order:?}"
                    );
                } else {
                    assert_eq!(c.conflict_type, ConflictType::MutualYield);
                }
                let mut copy = copy_fixture(&vehicles);
                solve_conflicts(vec![c], &mut copy, &LocalLogger::none()).unwrap();
                let granted: Vec<_> = copy
                    .values()
                    .filter(|v| v.intention.intention_cell_id == 900)
                    .map(|v| v.id)
                    .collect();
                assert_eq!(
                    granted,
                    expected.map(|id| id as u64).into_iter().collect::<Vec<_>>()
                );
                checked += 1;
            }
        }
    }
    assert_eq!(checked, 196608);
}

#[test]
fn actual_session_retains_all_shared_target_priorities_and_occupied_cells() {
    use micro_traffic_sim_core::simulation::session::Session;
    let mut session = Session::default(None);
    session.add_cells(
        [100, 200, 300]
            .into_iter()
            .map(|id| {
                Cell::new(id)
                    .with_forward_node(900)
                    .with_speed_limit(1)
                    .build()
            })
            .chain([
                Cell::new(900)
                    .with_forward_node(901)
                    .with_speed_limit(1)
                    .build(),
                Cell::new(901).with_speed_limit(1).build(),
            ])
            .collect(),
    );
    session.add_conflict_zone(zone(1, 300, 200, ConflictWinnerType::First));
    session.add_conflict_zone(zone(2, 200, 100, ConflictWinnerType::First));
    session.add_vehicles(
        (1..=3)
            .map(|id| {
                Vehicle::new(id)
                    .with_cell(id as i64 * 100)
                    .with_destination(901)
                    .with_speed(1)
                    .with_speed_limit(1)
                    .with_slowdown(0.0)
                    .with_slow_to_start(0.0)
                    .build()
            })
            .collect(),
    );
    for tick in 0..8 {
        let occupied: HashMap<_, _> = session
            .get_vehicles()
            .values()
            .map(|v| (v.cell_id, v.id))
            .collect();
        let state = session.step().unwrap();
        assert_eq!(state.vehicles_lost, 0);
        for v in session.get_vehicles().values() {
            assert!(occupied.get(&v.cell_id).is_none_or(|&owner| owner == v.id));
        }
        if tick == 0 {
            assert_eq!(session.get_vehicles()[&3].cell_id, 900);
            assert_eq!(session.get_vehicles()[&1].cell_id, 100);
            assert_eq!(session.get_vehicles()[&2].cell_id, 200);
        }
        if tick == 7 {
            assert_eq!(state.vehicles_completed, 3);
        }
    }
}

#[test]
fn merge_index_keeps_replacements_and_ignores_stale_legacy_lookup() {
    use micro_traffic_sim_core::conflicts::{
        CrossingZoneIndex, collect_conflicts, collect_conflicts_indexed,
    };
    use micro_traffic_sim_core::grid::road_network::GridRoads;
    use micro_traffic_sim_core::intentions::Intentions;
    let mut net = GridRoads::new();
    for id in [100, 200, 300, 900, 901] {
        net.add_cell(Cell::new(id).build());
    }
    let mut vehicles = IndexMap::from([
        (1, car(1, 100, &[900])),
        (2, car(2, 200, &[900])),
        (3, car(3, 300, &[900])),
    ]);
    let mut claims = Intentions::new();
    for v in vehicles.values_mut() {
        claims.add_intention(v, IntentionType::Target);
    }
    let mut zones = HashMap::new();
    let mut index = CrossingZoneIndex::default();
    for z in [
        zone(1, 300, 200, ConflictWinnerType::First),
        zone(2, 200, 100, ConflictWinnerType::First),
    ] {
        index.replace(&z);
        zones.insert(z.get_id(), z);
    }
    assert_eq!(index.all_candidates(&claims), vec![1, 2]);
    for legacy in [
        HashMap::new(),
        HashMap::from([(900, 999)]),
        HashMap::from([(900, 1)]),
    ] {
        let a = collect_conflicts(
            &claims,
            &net,
            &zones,
            &legacy,
            &LocalLogger::none(),
            &mut copy_fixture(&vehicles),
        )
        .unwrap();
        let b = collect_conflicts_indexed(
            &claims,
            &net,
            &zones,
            &legacy,
            &LocalLogger::none(),
            &mut copy_fixture(&vehicles),
            &index,
        )
        .unwrap();
        assert_eq!(format!("{a:?}"), format!("{b:?}"));
        assert_eq!(a[0].participants[a[0].priority_participant_index], 3);
    }
    let changed = ConflictZone::new(
        1,
        ConflictEdge {
            source: 100,
            target: 901,
        },
        ConflictEdge {
            source: 200,
            target: 901,
        },
    )
    .build();
    index.replace(&changed);
    zones.insert(1, changed);
    assert_eq!(index.all_candidates(&claims), vec![2]);
}

#[test]
fn physical_tail_outranks_merge_priority_even_for_a_desperate_driver() {
    // 100(car)   900(truck tail)   901(truck head)
    // The car requests 900. Priority and patience cannot make an occupied
    // body cell available in the current tick.
    let mut vehicles = IndexMap::from([(1, car(1, 100, &[900])), (2, car(2, 901, &[902]))]);
    vehicles[&2].tail_cells = vec![900];
    vehicles[&1].wait_steps = 10000;
    let zones = HashMap::from([(1, zone(1, 100, 200, ConflictWinnerType::First))]);
    let claims = [
        CellIntention::new(1, IntentionType::Target),
        CellIntention::new(2, IntentionType::Tail),
    ];
    let c = new_conflict_multiple(
        &Cell::new(900).build(),
        &zones,
        &HashMap::new(),
        &claims,
        &vehicles,
    )
    .unwrap();
    assert_eq!(c.conflict_type, ConflictType::Tail);
    assert_eq!(c.participants[c.priority_participant_index], 2);
    solve_conflicts(vec![c], &mut vehicles, &LocalLogger::none()).unwrap();
    assert_eq!(vehicles[&1].intention.intention_cell_id, 100);
    assert_eq!(vehicles[&1].intention.intention_speed, 0);
}

#[test]
fn mature_cycles_preserve_external_priorities_for_every_four_approach_graph() {
    use std::cmp::Reverse;
    let edges: Vec<_> = (1..=4)
        .flat_map(|a| (1..=4).filter(move |&b| a != b).map(move |b| (a, b)))
        .collect();
    let mut orders = Vec::new();
    for a in 1..=4 {
        for b in 1..=4 {
            for c in 1..=4 {
                for d in 1..=4 {
                    let order = [a, b, c, d];
                    if (0..4).all(|i| (i + 1..4).all(|j| order[i] != order[j])) {
                        orders.push(order);
                    }
                }
            }
        }
    }
    let mut checked = 0;
    for mask in 0..(1 << 12) {
        let selected: Vec<_> = edges
            .iter()
            .enumerate()
            .filter(|(i, _)| mask & (1 << i) != 0)
            .collect();
        if (1..=4).any(|id| !selected.iter().any(|(_, (_, loser))| *loser == id)) {
            continue;
        }
        // Independent Floyd-Warshall oracle, distinct from the Rust DFS implementation.
        let mut reach = [[false; 4]; 4];
        for i in 0..4 {
            reach[i][i] = true;
        }
        for &(_, &(a, b)) in &selected {
            reach[(a - 1) as usize][(b - 1) as usize] = true;
        }
        for k in 0..4 {
            for i in 0..4 {
                for j in 0..4 {
                    reach[i][j] |= reach[i][k] && reach[k][j];
                }
            }
        }
        for (ages, delays) in [([4, 4, 4, 4], [4, 4, 4, 4]), ([4, 4, 2, 100], [5, 3, 4, 5])] {
            let candidate = (0..4)
                .filter(|&i| (0..4).all(|j| !reach[j][i] || reach[i][j]))
                .max_by_key(|&i| (ages[i] >= delays[i], ages[i], Reverse(i)))
                .unwrap();
            let expected = (ages[candidate] >= delays[candidate]).then_some(candidate as u64 + 1);
            for reverse in [false, true] {
                let mut zones = HashMap::new();
                let mut definitions = selected.clone();
                if reverse {
                    definitions.reverse();
                }
                for (i, &(a, b)) in definitions {
                    zones.insert(
                        i as i32,
                        zone(i as i32, a * 100, b * 100, ConflictWinnerType::First),
                    );
                }
                for order in &orders {
                    let mut vehicles = IndexMap::new();
                    for id in 1..=4 {
                        let mut v = car(id, id as i64 * 100, &[900]);
                        v.yield_cycle_wait_steps = ages[(id - 1) as usize];
                        v.yield_cycle_delay = delays[(id - 1) as usize];
                        v.wait_steps = 10000;
                        vehicles.insert(id, v);
                    }
                    let c = conflict(&vehicles, &zones, order);
                    assert_eq!(c.conflict_type, ConflictType::MutualYield);
                    assert_eq!(
                        c.participants[c.priority_participant_index],
                        candidate as u64 + 1
                    );
                    solve_conflicts(vec![c], &mut vehicles, &LocalLogger::none()).unwrap();
                    let granted: Vec<_> = vehicles
                        .values()
                        .filter(|v| v.intention.intention_cell_id == 900)
                        .map(|v| v.id)
                        .collect();
                    assert_eq!(
                        granted,
                        expected.into_iter().collect::<Vec<_>>(),
                        "mask={mask}, order={order:?}"
                    );
                    checked += 1;
                }
            }
        }
    }
    assert_eq!(checked, 230496);
}

#[test]
fn cycle_release_preserves_physical_bodies_and_rebuilds_its_own_tail() {
    for occupied in [false, true] {
        let mut vehicles = IndexMap::from([
            (1, car(1, 100, &[900])),
            (2, car(2, 200, &[900])),
            (3, car(3, 300, &[900])),
        ]);
        vehicles[&1].tail_cells = vec![99];
        for v in vehicles.values_mut() {
            v.yield_cycle_wait_steps = 4;
        }
        if occupied {
            vehicles.insert(9, car(9, 901, &[902]));
            vehicles[&9].tail_cells = vec![900];
        }
        let zones = HashMap::from([
            (1, zone(1, 100, 200, ConflictWinnerType::First)),
            (2, zone(2, 200, 300, ConflictWinnerType::First)),
            (3, zone(3, 300, 100, ConflictWinnerType::First)),
        ]);
        let c = conflict(&vehicles, &zones, &[3, 2, 1]);
        solve_conflicts(vec![c], &mut vehicles, &LocalLogger::none()).unwrap();
        assert_eq!(
            vehicles[&1].intention.intention_cell_id,
            if occupied { 100 } else { 900 }
        );
        if !occupied {
            assert_eq!(vehicles[&1].intention.tail_intention_cells, vec![100]);
        }
        for id in [2, 3] {
            assert_eq!(vehicles[&id].intention.intention_cell_id, id as i64 * 100);
        }
    }
}

#[test]
fn persistent_losers_outrank_a_previously_served_vehicle() {
    let mut vehicles = IndexMap::from([
        (1, car(1, 100, &[900])),
        (2, car(2, 200, &[900])),
        (3, car(3, 300, &[900])),
    ]);
    for v in vehicles.values_mut() {
        v.yield_cycle_wait_steps = 4;
    }
    let zones = HashMap::from([
        (1, zone(1, 100, 200, ConflictWinnerType::First)),
        (2, zone(2, 200, 300, ConflictWinnerType::First)),
        (3, zone(3, 300, 100, ConflictWinnerType::First)),
    ]);
    for expected in [1, 2, 3] {
        let c = conflict(&vehicles, &zones, &[3, 2, 1]);
        assert_eq!(c.participants[c.priority_participant_index], expected);
        solve_conflicts(vec![c], &mut vehicles, &LocalLogger::none()).unwrap();
        for (&id, v) in vehicles.iter_mut() {
            v.update_yield_cycle_wait(id == expected, id != expected);
            v.set_intention(car(id, id as i64 * 100, &[900]).intention);
        }
    }
}
