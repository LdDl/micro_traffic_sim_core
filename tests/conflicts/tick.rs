use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::simulation::session::Session;

#[test]
fn a_cell_merge_cannot_hide_crossing_lane_changes() {
    // 10(A)   20(B)   30(C)
    //   | \   / |     /
    //   |  \ /  |    /
    //   |  / \  |   /
    //   | /   \ |  /
    //  11      21
    // A wants 21, B wants 11, C wants 21. Resolving the shared target must
    // still prevent A and B from executing both crossing diagonals.
    for order in [
        [1, 2, 3],
        [1, 3, 2],
        [2, 1, 3],
        [2, 3, 1],
        [3, 1, 2],
        [3, 2, 1],
    ] {
        let mut session = Session::default(None);
        session.add_cells(vec![
            Cell::new(10)
                .with_forward_node(11)
                .with_right_node(21)
                .with_speed_limit(1)
                .build(),
            Cell::new(20)
                .with_forward_node(21)
                .with_left_node(11)
                .with_speed_limit(1)
                .build(),
            Cell::new(30)
                .with_forward_node(21)
                .with_speed_limit(1)
                .build(),
            Cell::new(11)
                .with_forward_node(12)
                .with_speed_limit(1)
                .build(),
            Cell::new(21)
                .with_forward_node(22)
                .with_speed_limit(1)
                .build(),
            Cell::new(12).build(),
            Cell::new(22).build(),
        ]);
        session.add_vehicles(
            order
                .into_iter()
                .map(|id| {
                    Vehicle::new(id)
                        .with_cell(id as i64 * 10)
                        .with_destination(if id == 2 { 12 } else { 22 })
                        .with_cooperative_level(if id == 1 { 0.0 } else { 1.0 })
                        .with_speed(1)
                        .with_speed_limit(1)
                        .with_slowdown(0.0)
                        .with_slow_to_start(0.0)
                        .build()
                })
                .collect(),
        );
        session.step().unwrap();
        let a = &session.get_vehicles()[&1];
        let b = &session.get_vehicles()[&2];
        assert!(
            a.cell_id != 21 || b.cell_id != 11,
            "both incompatible diagonals executed: order={order:?}, a={a:?}, b={b:?}"
        );
    }
}

#[test]
fn an_unrelated_left_candidate_cannot_hide_a_crossing_opponent() {
    // 10(A)   20(B)   30(C)
    //   | \   / |      |
    //   |  \ /  |      |
    //   |  / \  |      |
    //   | /   \ |      |
    //  11      21      31
    // C also requests 11. Its left turn must not hide the crossing A/B pair.
    use indexmap::IndexMap;
    use micro_traffic_sim_core::agents::VehicleIntention;
    use micro_traffic_sim_core::conflicts::find_cross_trajectories_conflict_naive;
    use micro_traffic_sim_core::grid::road_network::GridRoads;
    use micro_traffic_sim_core::intentions::{CellIntention, IntentionType, Intentions};
    use micro_traffic_sim_core::maneuver::LaneChangeType;
    let mut net = GridRoads::new();
    for cell in [
        Cell::new(10)
            .with_forward_node(11)
            .with_right_node(21)
            .build(),
        Cell::new(20)
            .with_forward_node(21)
            .with_left_node(11)
            .build(),
        Cell::new(30)
            .with_forward_node(31)
            .with_left_node(11)
            .build(),
        Cell::new(11).build(),
        Cell::new(21).build(),
        Cell::new(31).build(),
    ] {
        net.add_cell(cell);
    }
    for order in [[1, 3, 2], [1, 2, 3]] {
        let mut cars = IndexMap::new();
        let mut claims = Intentions::new();
        for id in order {
            let mut v = Vehicle::new(id).with_cell(id as i64 * 10).build();
            v.set_intention(VehicleIntention {
                intention_maneuver: if id == 1 {
                    LaneChangeType::ChangeRight
                } else {
                    LaneChangeType::ChangeLeft
                },
                intention_cell_id: if id == 1 { 21 } else { 11 },
                intention_speed: 1,
                ..Default::default()
            });
            claims.add_intention(&mut v, IntentionType::Target);
            cars.insert(id, v);
        }
        let found = find_cross_trajectories_conflict_naive(
            &CellIntention::new(1, IntentionType::Target),
            net.get_cell(&21).unwrap(),
            &claims,
            &net,
            &cars,
        )
        .unwrap()
        .expect("the actual crossing pair must be retained");
        assert_eq!(found.side_vehicle_id, 2);
    }
}

#[test]
fn composed_session_preserves_cells_bodies_and_incompatible_edges() {
    use micro_traffic_sim_core::simulation::clearance::LongitudinalClearance;
    use std::collections::{BTreeMap, BTreeSet};
    let orders = [
        [1, 2, 3],
        [1, 3, 2],
        [2, 1, 3],
        [2, 3, 1],
        [3, 1, 2],
        [3, 2, 1],
    ];
    let mut cases = 0;
    let mut ticks = 0;
    for order in orders {
        for profile in 0..8 {
            for old_wait in [0, 10000] {
                for policy in [
                    LongitudinalClearance::Conservative,
                    LongitudinalClearance::MovingLeader,
                ] {
                    for tail in 0..=2 {
                        for limit in 1..=3 {
                            for noise in 0..4 {
                                for gap in 0..=1 {
                                    let mut session = Session::default(None);
                                    session.set_longitudinal_clearance(policy);
                                    let mut cells = vec![
                                        Cell::new(10)
                                            .with_forward_node(11)
                                            .with_right_node(21)
                                            .with_speed_limit(limit)
                                            .build(),
                                        Cell::new(20)
                                            .with_forward_node(21)
                                            .with_left_node(11)
                                            .with_speed_limit(limit)
                                            .build(),
                                        Cell::new(30)
                                            .with_forward_node(21)
                                            .with_speed_limit(limit)
                                            .build(),
                                    ];
                                    for id in [
                                        8, 9, 18, 19, 28, 29, 11, 12, 13, 14, 15, 21, 22, 23, 24,
                                        25,
                                    ] {
                                        cells.push(
                                            Cell::new(id)
                                                .with_forward_node(if id == 15 || id == 25 {
                                                    -1
                                                } else {
                                                    id + 1
                                                })
                                                .with_speed_limit(limit)
                                                .build(),
                                        );
                                    }
                                    let legal: BTreeMap<_, _> = cells
                                        .iter()
                                        .map(|c| {
                                            (
                                                c.get_id(),
                                                [
                                                    c.get_forward_id(),
                                                    c.get_left_id(),
                                                    c.get_right_id(),
                                                ],
                                            )
                                        })
                                        .collect();
                                    session.add_cells(cells);
                                    session.add_vehicles(
                                        order
                                            .into_iter()
                                            .map(|id| {
                                                let from = id as i64 * 10;
                                                let mut v = Vehicle::new(id)
                                                    .with_cell(from)
                                                    .with_tail_size(
                                                        tail,
                                                        ((from - tail as i64)..from).collect(),
                                                    )
                                                    .with_destination(if id == 2 { 15 } else { 25 })
                                                    .with_cooperative_level(
                                                        if profile & (1 << (id - 1)) == 0 {
                                                            0.0
                                                        } else {
                                                            1.0
                                                        },
                                                    )
                                                    .with_standstill_gap(gap)
                                                    .with_speed(limit)
                                                    .with_speed_limit(limit)
                                                    .with_slowdown((noise & 1) as f64)
                                                    .with_slow_to_start(((noise >> 1) & 1) as f64)
                                                    .build();
                                                v.wait_steps = old_wait;
                                                v
                                            })
                                            .collect(),
                                    );
                                    for tick in 0..4 {
                                        let before: BTreeMap<_, _> = session
                                            .get_vehicles()
                                            .values()
                                            .map(|v| (v.id, (v.cell_id, v.tail_cells.clone())))
                                            .collect();
                                        let initial: BTreeMap<_, _> = before
                                            .iter()
                                            .flat_map(|(&id, (head, tail))| {
                                                tail.iter()
                                                    .copied()
                                                    .chain(std::iter::once(*head))
                                                    .filter(|&c| c >= 0)
                                                    .map(move |c| (c, id))
                                            })
                                            .collect();
                                        session.step().unwrap();
                                        let mut occupied = BTreeMap::new();
                                        let mut swept = BTreeMap::new();
                                        let mut edges = BTreeSet::new();
                                        for v in session.get_vehicles().values() {
                                            let (old, _) = &before[&v.id];
                                            let path = if *old == v.cell_id {
                                                Vec::new()
                                            } else {
                                                v.intention
                                                    .intermediate_cells
                                                    .iter()
                                                    .copied()
                                                    .chain(std::iter::once(v.cell_id))
                                                    .collect()
                                            };
                                            let mut previous = *old;
                                            for &cell in &path {
                                                assert!(
                                                    legal[&previous].contains(&cell),
                                                    "invalid executed edge {previous} {cell}"
                                                );
                                                assert!(
                                                    initial
                                                        .get(&cell)
                                                        .is_none_or(|&owner| owner == v.id),
                                                    "entered an initial foreign body"
                                                );
                                                assert!(
                                                    swept
                                                        .insert(cell, v.id)
                                                        .is_none_or(|owner| owner == v.id),
                                                    "overlapping swept paths"
                                                );
                                                edges.insert((previous, cell));
                                                previous = cell;
                                            }
                                            for cell in v
                                                .tail_cells
                                                .iter()
                                                .copied()
                                                .chain(std::iter::once(v.cell_id))
                                                .filter(|&c| c >= 0)
                                            {
                                                assert!(
                                                    occupied.insert(cell, v.id).is_none(),
                                                    "overlapping final bodies: case={cases}, tick={tick}"
                                                );
                                                assert!(
                                                    initial.get(&cell) == Some(&v.id)
                                                        || path.contains(&cell),
                                                    "body escaped its initial cells and granted path"
                                                );
                                            }
                                        }
                                        assert!(
                                            !(edges.contains(&(10, 21))
                                                && edges.contains(&(20, 11))),
                                            "incompatible diagonals: case={cases}, tick={tick}"
                                        );
                                        ticks += 1;
                                    }
                                    cases += 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    assert_eq!(cases, 13824);
    assert_eq!(ticks, 55296);
}

#[test]
fn converging_changes_with_the_same_direction_have_a_defined_priority() {
    for right in [false, true] {
        for order in [[1, 2], [2, 1]] {
            let mut session = Session::default(None);
            let mut cells = vec![
                Cell::new(90)
                    .with_forward_node(91)
                    .with_speed_limit(1)
                    .build(),
                Cell::new(91).build(),
            ];
            for id in [10, 20] {
                cells.push(
                    Cell::new(id)
                        .with_forward_node(id + 1)
                        .with_left_node(if right { -1 } else { 90 })
                        .with_right_node(if right { 90 } else { -1 })
                        .with_speed_limit(1)
                        .build(),
                );
                cells.push(Cell::new(id + 1).build());
            }
            session.add_cells(cells);
            session.add_vehicles(
                order
                    .into_iter()
                    .map(|id| {
                        Vehicle::new(id)
                            .with_cell(id as i64 * 10)
                            .with_destination(91)
                            .with_cooperative_level(1.0)
                            .with_speed(1)
                            .with_speed_limit(1)
                            .with_slowdown(0.0)
                            .with_slow_to_start(0.0)
                            .build()
                    })
                    .collect(),
            );
            session.step().unwrap();
            assert_eq!(session.get_vehicles()[&1].cell_id, 90);
            assert_eq!(session.get_vehicles()[&2].cell_id, 20);
        }
    }
}

#[test]
fn overlapping_constraints_only_shorten_initial_paths() {
    // 10   20       30   40
    //   \ /           \ /
    //   100           200
    //    |             |
    //   101           201
    //    |             |
    //   102           202
    // Priority constraints also link the approaches across the two merges.
    // Every resolution must remain a prefix of the original requested path.
    use indexmap::IndexMap;
    use micro_traffic_sim_core::agents::VehicleIntention;
    use micro_traffic_sim_core::conflict_zones::{ConflictEdge, ConflictWinnerType, ConflictZone};
    use micro_traffic_sim_core::conflicts::{collect_conflicts, solve_conflicts};
    use micro_traffic_sim_core::grid::road_network::GridRoads;
    use micro_traffic_sim_core::intentions::{IntentionType, Intentions};
    use micro_traffic_sim_core::maneuver::LaneChangeType;
    use micro_traffic_sim_core::verbose::LocalLogger;
    use std::collections::{BTreeMap, HashMap};
    let speed_levels: usize = 3;
    let vehicle_count = 4;
    let speed_combinations = speed_levels * speed_levels * speed_levels * speed_levels;
    let wait_combinations = 1 << vehicle_count;
    let mut cases = 0;
    for shift in 0..vehicle_count {
        for reversed in [false, true] {
            for speeds in 0..speed_combinations {
                for waits in 0..wait_combinations {
                    for cyclic in [false, true] {
                        let mut net = GridRoads::new();
                        for (id, next) in [
                            (10, 100),
                            (20, 100),
                            (30, 200),
                            (40, 200),
                            (100, 101),
                            (101, 102),
                            (102, -1),
                            (200, 201),
                            (201, 202),
                            (202, -1),
                        ] {
                            net.add_cell(
                                Cell::new(id)
                                    .with_forward_node(next)
                                    .with_speed_limit(speed_levels as i32)
                                    .build(),
                            );
                        }
                        let mut cars = IndexMap::new();
                        let mut claims = Intentions::new();
                        let mut initial = BTreeMap::new();
                        let mut order: Vec<u64> = (0..vehicle_count)
                            .map(|i| ((i + shift) % vehicle_count + 1) as u64)
                            .collect();
                        if reversed {
                            order.reverse();
                        }
                        for id in order {
                            let length =
                                1 + speeds / speed_levels.pow((id - 1) as u32) % speed_levels;
                            let base = if id <= 2 { 100 } else { 200 };
                            let path: Vec<_> = (base..base + length as i64).collect();
                            let mut v = Vehicle::new(id)
                                .with_cell(id as i64 * 10)
                                .with_speed(length as i32)
                                .with_speed_limit(speed_levels as i32)
                                .with_cooperative_level(1.0)
                                .build();
                            v.wait_steps = if waits & (1 << (id - 1)) == 0 {
                                0
                            } else {
                                v.patience() + 1
                            };
                            v.yield_cycle_wait_steps = 4;
                            v.set_intention(VehicleIntention {
                                intention_maneuver: LaneChangeType::NoChange,
                                intention_speed: length as i32,
                                intention_cell_id: *path.last().unwrap(),
                                intermediate_cells: path[..path.len() - 1].to_vec(),
                                ..Default::default()
                            });
                            initial.insert(id, path);
                            claims.add_intention(&mut v, IntentionType::Target);
                            cars.insert(id, v);
                        }
                        let mut zones = HashMap::new();
                        for (id, a, b, first) in [
                            (1, 10, 30, true),
                            (2, 20, 40, false),
                            (3, 10, 20, true),
                            (4, 10, 20, !cyclic),
                        ] {
                            let z = ConflictZone::new(
                                id,
                                ConflictEdge {
                                    source: a,
                                    target: if a <= 20 { 100 } else { 200 },
                                },
                                ConflictEdge {
                                    source: b,
                                    target: if b <= 20 { 100 } else { 200 },
                                },
                            )
                            .with_winner_type(if first {
                                ConflictWinnerType::First
                            } else {
                                ConflictWinnerType::Second
                            })
                            .build();
                            zones.insert(id, z);
                        }
                        let conflicts = collect_conflicts(
                            &claims,
                            &net,
                            &zones,
                            &HashMap::new(),
                            &LocalLogger::none(),
                            &mut cars,
                        )
                        .unwrap();
                        solve_conflicts(conflicts, &mut cars, &LocalLogger::none()).unwrap();
                        let mut entered = BTreeMap::new();
                        let mut paths = BTreeMap::new();
                        for v in cars.values() {
                            let path: Vec<_> = if v.intention.intention_cell_id == v.cell_id {
                                vec![]
                            } else {
                                v.intention
                                    .intermediate_cells
                                    .iter()
                                    .copied()
                                    .chain(std::iter::once(v.intention.intention_cell_id))
                                    .collect()
                            };
                            assert!(
                                initial[&v.id].starts_with(&path),
                                "solver expanded or redirected a path: {v:?}"
                            );
                            for &c in &path {
                                assert!(
                                    entered.insert(c, v.id).is_none(),
                                    "two grants share cell {c}: case={cases}"
                                );
                            }
                            paths.insert(v.id, path);
                        }
                        assert!(paths[&1].is_empty() || paths[&3].is_empty(), "lost zone 1");
                        assert!(paths[&2].is_empty() || paths[&4].is_empty(), "lost zone 2");
                        cases += 1;
                    }
                }
            }
        }
    }
    assert_eq!(cases, 20736);
}

#[test]
fn ordinary_patience_cannot_cross_a_body_in_a_declared_zone() {
    // 10(tail)   20(car)
    //    |          |
    // 100(head)    21
    //    |          |
    //   101        22
    // The zone links edges (10, 100) and (20, 21). The truck body protects
    // its edge even when the car has exhausted its ordinary patience.
    use micro_traffic_sim_core::conflict_zones::{ConflictEdge, ConflictWinnerType, ConflictZone};
    for stopped in [false, true] {
        for wait in [0, 10000] {
            for priority in [ConflictWinnerType::First, ConflictWinnerType::Second] {
                let mut s = Session::default(None);
                s.add_cells(vec![
                    Cell::new(10)
                        .with_forward_node(11)
                        .with_right_node(100)
                        .with_speed_limit(1)
                        .build(),
                    Cell::new(11).build(),
                    Cell::new(100)
                        .with_forward_node(101)
                        .with_speed_limit(1)
                        .build(),
                    Cell::new(101)
                        .with_forward_node(102)
                        .with_speed_limit(1)
                        .build(),
                    Cell::new(102).build(),
                    Cell::new(20)
                        .with_forward_node(21)
                        .with_speed_limit(1)
                        .build(),
                    Cell::new(21)
                        .with_forward_node(22)
                        .with_speed_limit(1)
                        .build(),
                    Cell::new(22).build(),
                ]);
                s.add_conflict_zone(
                    ConflictZone::new(
                        1,
                        ConflictEdge {
                            source: 10,
                            target: 100,
                        },
                        ConflictEdge {
                            source: 20,
                            target: 21,
                        },
                    )
                    .with_winner_type(priority)
                    .build(),
                );
                let truck = Vehicle::new(1)
                    .with_cell(100)
                    .with_tail_size(1, vec![10])
                    .with_destination(102)
                    .with_speed(1)
                    .with_speed_limit(1)
                    .with_slowdown(0.0)
                    .with_slow_to_start(0.0)
                    .build();
                let mut car = Vehicle::new(2)
                    .with_cell(20)
                    .with_destination(22)
                    .with_speed(1)
                    .with_speed_limit(1)
                    .with_slowdown(0.0)
                    .with_slow_to_start(0.0)
                    .build();
                car.wait_steps = wait;
                s.add_vehicles(vec![truck, car]);
                if stopped {
                    s.add_vehicles(vec![
                        Vehicle::new(3)
                            .with_cell(101)
                            .with_destination(102)
                            .with_speed(0)
                            .with_speed_limit(1)
                            .with_slowdown(1.0)
                            .with_slow_to_start(1.0)
                            .build(),
                    ]);
                }
                s.step().unwrap();
                assert_eq!(
                    s.get_vehicles()[&2].cell_id,
                    20,
                    "patience cannot override an occupied body edge"
                );
            }
        }
    }
}

#[test]
fn stopped_bodies_protect_both_diagonal_directions_without_a_zone() {
    // 10     20
    //   \   /
    //    \ /
    //    / \
    //   /   \
    // 11     21
    // One diagonal is occupied by a truck body. The other is requested by
    // a car. Neither direction may cross the body, even without a zone.
    for stopped in [false, true] {
        for length in [1, 3] {
            for speed in [1, 3] {
                for truck_right in [false, true] {
                    for wait in [0, 10000] {
                        let mut s = Session::default(None);
                        s.add_cells(vec![
                            Cell::new(8).with_forward_node(9).build(),
                            Cell::new(9).with_forward_node(10).build(),
                            Cell::new(18).with_forward_node(19).build(),
                            Cell::new(19).with_forward_node(20).build(),
                            Cell::new(10)
                                .with_forward_node(11)
                                .with_right_node(21)
                                .with_speed_limit(3)
                                .build(),
                            Cell::new(20)
                                .with_forward_node(21)
                                .with_left_node(11)
                                .with_speed_limit(3)
                                .build(),
                            Cell::new(11)
                                .with_forward_node(12)
                                .with_speed_limit(3)
                                .build(),
                            Cell::new(12)
                                .with_forward_node(13)
                                .with_speed_limit(3)
                                .build(),
                            Cell::new(13).build(),
                            Cell::new(21)
                                .with_forward_node(22)
                                .with_speed_limit(3)
                                .build(),
                            Cell::new(22)
                                .with_forward_node(23)
                                .with_speed_limit(3)
                                .build(),
                            Cell::new(23).build(),
                        ]);
                        let head = if truck_right { 21 } else { 11 };
                        let tail = if truck_right { 10 } else { 20 };
                        let other = if truck_right { 20 } else { 10 };
                        let dest = if truck_right { 13 } else { 23 };
                        let truck = Vehicle::new(1)
                            .with_cell(head)
                            .with_tail_size(length as usize, ((tail - length + 1)..=tail).collect())
                            .with_destination(head + 2)
                            .with_speed(speed)
                            .with_speed_limit(speed)
                            .with_slowdown(0.0)
                            .with_slow_to_start(0.0)
                            .build();
                        let barrier = Vehicle::new(3)
                            .with_cell(head + 1)
                            .with_destination(head + 2)
                            .with_speed(0)
                            .with_speed_limit(1)
                            .with_slowdown(1.0)
                            .with_slow_to_start(1.0)
                            .build();
                        let mut car = Vehicle::new(2)
                            .with_cell(other)
                            .with_destination(dest)
                            .with_speed(1)
                            .with_speed_limit(1)
                            .with_slowdown(0.0)
                            .with_slow_to_start(0.0)
                            .build();
                        car.wait_steps = wait;
                        s.add_vehicles(vec![truck, car]);
                        if stopped {
                            s.add_vehicles(vec![barrier]);
                        }
                        s.step().unwrap();
                        assert_eq!(
                            s.get_vehicles()[&2].cell_id,
                            other,
                            "crossed a static body: truck_right={truck_right}, wait={wait}"
                        );
                    }
                }
            }
        }
    }
}
