use micro_traffic_sim_core::agents::{TAIL_PLACEHOLDER, Vehicle};
use micro_traffic_sim_core::behaviour::BehaviourType;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::grid::road_network::GridRoads;
use micro_traffic_sim_core::maneuver::LaneChangeType;
use micro_traffic_sim_core::simulation::session::Session;

fn two_lanes(turn: i64, right: bool, diagonal: bool) -> Vec<Cell> {
    (0..24)
        .chain(100..124)
        .map(|id| {
            let side = if id == turn {
                100 + turn + i64::from(diagonal)
            } else {
                -1
            };
            Cell::new(id)
                .with_forward_node(if id == 23 || id == 123 { -1 } else { id + 1 })
                .with_left_node(if right { -1 } else { side })
                .with_right_node(if right { side } else { -1 })
                .with_speed_limit(3)
                .build()
        })
        .collect()
}

fn follows_turn(tail: usize, materialized: bool, right: bool, diagonal: bool, turn: i64) {
    let head = if materialized { tail as i64 } else { 0 };
    let mut session = Session::default(None);
    session.add_cells(two_lanes(turn, right, diagonal));
    session.add_vehicles(vec![
        Vehicle::new(1)
            .with_cell(head)
            .with_tail_size(
                tail,
                if materialized {
                    (0..head).collect()
                } else {
                    vec![]
                },
            )
            .with_destination(123)
            .with_speed(1)
            .with_speed_limit(1)
            .with_behaviour(BehaviourType::Aggressive)
            .with_slowdown(0.0)
            .with_slow_to_start(0.0)
            .build(),
    ]);
    let mut trace = Vec::new();
    let mut crossed = false;
    for t in 0..50 {
        session.step().unwrap();
        let Some(v) = session.get_vehicles().get(&1) else {
            assert!(crossed, "disappeared before turn: {trace:?}");
            return;
        };
        crossed |= v.cell_id >= 100;
        let body: Vec<_> = v
            .tail_cells
            .iter()
            .copied()
            .filter(|&x| x != TAIL_PLACEHOLDER)
            .chain(std::iter::once(v.cell_id))
            .collect();
        let unique: std::collections::HashSet<_> = body.iter().copied().collect();
        assert_eq!(body.len(), unique.len(), "overlapping own body: {body:?}");
        trace.push(format!("{t}:head={} tail={:?} intent={:?} target={} speed={} timers={}/{}/{} tail_maneuver={:?}",
            v.cell_id,v.tail_cells,v.intention.intention_maneuver,v.intention.intention_cell_id,v.speed,
            v.timer_non_maneuvers,v.timer_non_acceleration,v.timer_non_slowdown,v.intention.tail_maneuver));
    }
    panic!(
        "truck did not finish: tail={tail} materialized={materialized} right={right} diagonal={diagonal} turn={turn}; {trace:?}"
    );
}

#[test]
fn truck_turns_at_entry() {
    for tail in 0..=3 {
        for right in [false, true] {
            for diagonal in [false, true] {
                follows_turn(tail, false, right, diagonal, 0);
            }
        }
    }
}

#[test]
fn truck_changes_lane_after_driving_straight() {
    for tail in 0..=3 {
        for materialized in [false, true] {
            for right in [false, true] {
                for diagonal in [false, true] {
                    follows_turn(tail, materialized, right, diagonal, 8);
                }
            }
        }
    }
}

#[test]
fn tail_on_cell_zero_still_completes_its_turn() {
    let mut net = GridRoads::new();
    for cell in two_lanes(0, true, false) {
        net.add_cell(cell);
    }
    let truck = Vehicle::new(1)
        .with_cell(100)
        .with_tail_size(1, vec![0])
        .build();
    let maneuver = truck.scan_tail_maneuver(&net).unwrap();
    assert_eq!(maneuver.intention_maneuver, LaneChangeType::ChangeRight);
    assert_eq!(maneuver.source_cell_maneuver, 0);
    assert_eq!(maneuver.target_cell_maneuver, 100);
}

#[test]
fn truck_on_fully_connected_parallel_lanes() {
    use micro_traffic_sim_core::geom::new_point;
    for tail in 0..=3 {
        for right in [false, true] {
            for diagonal in [false, true] {
                for start_speed in [0, 1, 3] {
                    for gap in [0, 1] {
                        let mut session = Session::default(None);
                        let cells = (0..40)
                            .chain(100..140)
                            .map(|id| {
                                let lane = id / 100;
                                let pos = id % 100;
                                let side = if pos < 38 {
                                    (1 - lane) * 100 + pos + i64::from(diagonal)
                                } else {
                                    -1
                                };
                                let change_right = (lane == 0) == right;
                                Cell::new(id)
                                    .with_point(new_point(
                                        pos as f64 * 4.5,
                                        lane as f64 * 3.5,
                                        None,
                                    ))
                                    .with_forward_node(if pos == 39 { -1 } else { id + 1 })
                                    .with_right_node(if change_right { side } else { -1 })
                                    .with_left_node(if change_right { -1 } else { side })
                                    .with_speed_limit(3)
                                    .build()
                            })
                            .collect();
                        session.add_cells(cells);
                        session.add_vehicles(vec![
                            Vehicle::new(1)
                                .with_cell(0)
                                .with_tail_size(tail, vec![])
                                .with_destination(139)
                                .with_speed(start_speed)
                                .with_speed_limit(3)
                                .with_standstill_gap(gap)
                                .with_slowdown(0.0)
                                .with_slow_to_start(0.0)
                                .build(),
                        ]);
                        let mut trace = Vec::new();
                        for _ in 0..100 {
                            session.step().unwrap();
                            if let Some(v) = session.get_vehicles().get(&1) {
                                trace.push((
                                    v.cell_id,
                                    v.tail_cells.clone(),
                                    v.intention.intention_maneuver,
                                    v.timer_non_maneuvers,
                                ));
                            } else {
                                break;
                            }
                        }
                        assert!(
                            session.get_vehicles().is_empty(),
                            "tail={tail}, right={right}, diagonal={diagonal}, start_speed={start_speed}, gap={gap}, trace={trace:?}"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn route_can_change_lane_into_cell_zero() {
    for tail in 0..=3 {
        let mut session = Session::default(None);
        let mut cells: Vec<_> = (0..4)
            .map(|id| {
                Cell::new(id)
                    .with_forward_node(if id == 3 { -1 } else { id + 1 })
                    .with_speed_limit(1)
                    .build()
            })
            .collect();
        cells.push(Cell::new(10).with_right_node(0).with_speed_limit(1).build());
        session.add_cells(cells);
        session.add_vehicles(vec![
            Vehicle::new(1)
                .with_cell(10)
                .with_destination(3)
                .with_tail_size(tail, vec![])
                .with_speed_limit(1)
                .with_slowdown(0.0)
                .with_slow_to_start(0.0)
                .build(),
        ]);
        for _ in 0..12 {
            session.step().unwrap();
        }
        assert!(
            session.get_vehicles().is_empty(),
            "tail={tail}: {:?}",
            session.get_vehicles()
        );
    }
}
