use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::conflict_zones::{ConflictEdge, ConflictWinnerType, ConflictZone};
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::simulation::session::Session;
use std::collections::HashMap;

fn session(order: &[u64], old_wait: i32) -> Session {
    // 100(A)  200(B)  300(C)
    //     \      |      /
    //           900
    //            |
    //           901
    // A has priority over B, B over C, C over A: a cycle.
    // Its waiting clock is separate from ordinary driver patience.
    let mut s = Session::default(None);
    s.add_cells(
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
    for (id, first, second) in [(1, 100, 200), (2, 200, 300), (3, 300, 100)] {
        s.add_conflict_zone(
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
            .with_winner_type(ConflictWinnerType::First)
            .build(),
        );
    }
    s.add_vehicles(
        order
            .iter()
            .map(|&id| {
                let mut v = Vehicle::new(id)
                    .with_cell(id as i64 * 100)
                    .with_destination(901)
                    .with_speed(1)
                    .with_speed_limit(1)
                    .with_slowdown(0.0)
                    .with_slow_to_start(0.0)
                    .build();
                v.wait_steps = old_wait;
                v
            })
            .collect(),
    );
    s
}

#[test]
fn equal_delays_release_one_vehicle_after_four_waiting_ticks() {
    for order in [
        [1, 2, 3],
        [1, 3, 2],
        [2, 1, 3],
        [2, 3, 1],
        [3, 1, 2],
        [3, 2, 1],
    ] {
        let mut s = session(&order, 0);
        for _ in 0..4 {
            s.step().unwrap();
            for (&id, v) in s.get_vehicles() {
                assert_eq!(v.cell_id, id as i64 * 100);
            }
        }
        s.step().unwrap();
        assert_eq!(s.get_vehicles()[&1].cell_id, 900);
        assert_eq!(s.get_vehicles()[&2].cell_id, 200);
        assert_eq!(s.get_vehicles()[&3].cell_id, 300);
        for _ in 0..10 {
            let before: HashMap<_, _> = s
                .get_vehicles()
                .values()
                .map(|v| (v.cell_id, v.id))
                .collect();
            let state = s.step().unwrap();
            assert_eq!(state.vehicles_lost, 0);
            for v in s.get_vehicles().values() {
                assert!(before.get(&v.cell_id).is_none_or(|&id| id == v.id));
            }
        }
        assert!(s.get_vehicles().is_empty());
    }
}

#[test]
fn ordinary_patience_does_not_bypass_a_cycle_delay() {
    let mut s = session(&[3, 2, 1], 10000);
    s.step().unwrap();
    for (&id, v) in s.get_vehicles() {
        assert_eq!(v.cell_id, id as i64 * 100);
    }
}

#[test]
fn cycle_clock_counts_completed_holds_once_and_keeps_loser_age() {
    use micro_traffic_sim_core::agents::DEFAULT_YIELD_CYCLE_DELAY;
    let mut v = Vehicle::new(1).build();
    assert_eq!(v.yield_cycle_delay, DEFAULT_YIELD_CYCLE_DELAY);
    assert_eq!(DEFAULT_YIELD_CYCLE_DELAY, 4);
    for expected in 1..=4 {
        v.hold_for_yield_cycle();
        v.hold_for_yield_cycle();
        assert_eq!(v.yield_cycle_wait_steps, expected - 1);
        v.update_yield_cycle_wait(false, true);
        assert_eq!(v.yield_cycle_wait_steps, expected);
    }
    assert!(v.yield_cycle_ready());
    // Ordinary queue waiting cannot increase the cycle clock.
    v.update_yield_cycle_wait(false, true);
    assert_eq!(v.yield_cycle_wait_steps, 4);
    // A red light or dwell pauses accumulated cycle waiting.
    v.hold_for_yield_cycle();
    v.update_yield_cycle_wait(false, false);
    assert_eq!(v.yield_cycle_wait_steps, 4);
    v.hold_for_yield_cycle();
    v.update_yield_cycle_wait(false, true);
    assert_eq!(v.yield_cycle_wait_steps, 5);
    v.update_yield_cycle_wait(true, false);
    assert_eq!(v.yield_cycle_wait_steps, 0);
    v.yield_cycle_wait_steps = i32::MAX;
    v.hold_for_yield_cycle();
    v.update_yield_cycle_wait(false, true);
    assert_eq!(v.yield_cycle_wait_steps, i32::MAX);
    assert_eq!(
        Vehicle::new(2)
            .with_yield_cycle_delay(-1)
            .build()
            .yield_cycle_delay,
        0
    );
}

#[test]
fn agent_delays_three_and_five_are_respected_with_equal_waiting_age() {
    let mut s = session(&[3, 1, 2], 0);
    s.add_vehicles(
        (1..=3)
            .map(|id| {
                Vehicle::new(id)
                    .with_cell(id as i64 * 100)
                    .with_destination(901)
                    .with_speed(1)
                    .with_speed_limit(1)
                    .with_slowdown(0.0)
                    .with_slow_to_start(0.0)
                    .with_yield_cycle_delay(if id == 1 { 5 } else { 3 })
                    .build()
            })
            .collect(),
    );
    for tick in 1..=3 {
        s.step().unwrap();
        for (&id, v) in s.get_vehicles() {
            assert_eq!(v.cell_id, id as i64 * 100);
            assert_eq!(v.yield_cycle_wait_steps, tick);
        }
    }
    s.step().unwrap();
    assert_eq!(s.get_vehicles()[&2].cell_id, 900);
    assert_eq!(s.get_vehicles()[&1].yield_cycle_wait_steps, 4);
    assert_eq!(s.get_vehicles()[&3].yield_cycle_wait_steps, 4);
    assert_eq!(s.get_vehicles()[&2].yield_cycle_wait_steps, 0);
}

#[test]
fn external_edge_priority_can_still_stop_a_ready_cycle_candidate() {
    for old_wait in [0, 10000] {
        let mut s = session(&[1, 2, 3], old_wait);
        for _ in 0..4 {
            s.step().unwrap();
        }
        s.add_cells(vec![
            Cell::new(500)
                .with_forward_node(600)
                .with_speed_limit(1)
                .build(),
            Cell::new(600)
                .with_forward_node(601)
                .with_speed_limit(1)
                .build(),
            Cell::new(601).build(),
        ]);
        s.add_conflict_zone(
            ConflictZone::new(
                4,
                ConflictEdge {
                    source: 500,
                    target: 600,
                },
                ConflictEdge {
                    source: 100,
                    target: 900,
                },
            )
            .with_winner_type(ConflictWinnerType::First)
            .build(),
        );
        s.add_vehicles(vec![
            Vehicle::new(9)
                .with_cell(500)
                .with_destination(601)
                .with_speed(1)
                .with_speed_limit(1)
                .with_slowdown(0.0)
                .with_slow_to_start(0.0)
                .build(),
        ]);
        s.step().unwrap();
        assert_eq!(s.get_vehicles()[&9].cell_id, 600);
        for id in 1..=3 {
            assert_eq!(s.get_vehicles()[&id].cell_id, id as i64 * 100);
        }
        assert_eq!(s.get_vehicles()[&1].yield_cycle_wait_steps, 4);
    }
}
