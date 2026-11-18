use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::behaviour::BehaviourType;
use micro_traffic_sim_core::grid::cell::{Cell, CellState};
use micro_traffic_sim_core::maneuver::LaneChangeType;
use micro_traffic_sim_core::simulation::session::Session;
use std::collections::HashMap;

fn lanes() -> Vec<Cell> {
    // 2(car) 3(head) 4  5 ... 10
    //        /
    // 53(tail) 54   55 56 ... 61
    // The truck has just changed lanes from 53 to 3. Its cached route
    // requests another change from 3 to 55 before the first tail has cleared.
    (2..=10)
        .chain(53..=61)
        .map(|id| {
            let upper = id < 53;
            let last = if upper { 10 } else { 61 };
            Cell::new(id)
                .with_speed_limit(4)
                .with_forward_node(if id == last { -1 } else { id + 1 })
                .with_right_node(if upper && id < last { id + 52 } else { -1 })
                .with_left_node(if !upper && id < last { id - 50 } else { -1 })
                .build()
        })
        .collect()
}

fn turning_truck(tail: usize, cached: bool) -> Vehicle {
    let mut truck = Vehicle::new(1)
        .with_cell(3)
        .with_destination(61)
        .with_tail_size(
            tail,
            if tail > 0 {
                std::iter::repeat_n(-1, tail - 1)
                    .chain(std::iter::once(53))
                    .collect()
            } else {
                vec![]
            },
        )
        .with_speed(1)
        .with_speed_limit(4)
        .with_slowdown(0.0)
        .with_slow_to_start(0.0)
        .build();
    truck.timer_non_acceleration = tail as i64;
    truck.timer_non_maneuvers = tail as i64;
    truck.timer_non_slowdown = tail as i64;
    if cached {
        truck.cached_route = std::iter::once(3).chain(55..=61).collect();
    }
    truck
}

fn step_without_body_reuse(session: &mut Session) {
    let before: HashMap<_, _> = session
        .get_vehicles()
        .values()
        .flat_map(|v| {
            std::iter::once(v.cell_id)
                .chain(v.tail_cells.iter().copied().filter(|&id| id >= 0))
                .map(move |cell| (cell, v.id))
        })
        .collect();
    session.step().unwrap();
    let mut occupied = HashMap::new();
    for v in session.get_vehicles().values() {
        for cell in
            std::iter::once(v.cell_id).chain(v.tail_cells.iter().copied().filter(|&id| id >= 0))
        {
            assert!(
                occupied.insert(cell, v.id).is_none(),
                "overlapping body at {cell}"
            );
            assert!(
                before.get(&cell).is_none_or(|&owner| owner == v.id),
                "same-tick body reuse at {cell}"
            );
        }
        for cell in &v.intention.intermediate_cells {
            assert!(before.get(cell).is_none_or(|&owner| owner == v.id));
        }
    }
}

#[test]
fn cached_and_rebuilt_routes_allow_the_tail_to_finish() {
    for cached in [true, false] {
        for tail in 1..=3 {
            let mut session = Session::default(None);
            session.add_cells(lanes());
            session.add_vehicles(vec![
                turning_truck(tail, cached),
                Vehicle::new(2)
                    .with_cell(2)
                    .with_destination(10)
                    .with_speed_limit(1)
                    .with_slowdown(0.0)
                    .with_slow_to_start(0.0)
                    .build(),
            ]);
            for step in 1..=tail {
                step_without_body_reuse(&mut session);
                let truck = &session.get_vehicles()[&1];
                assert_eq!(
                    truck.cell_id,
                    3 + step as i64,
                    "tail={tail}, cached={cached}"
                );
                assert_eq!(truck.intention.intention_maneuver, LaneChangeType::NoChange);
                assert_eq!(truck.timer_non_maneuvers, (tail - step) as i64);
                assert!(!truck.confusion);
                if step == 1 {
                    assert_eq!(session.get_vehicles()[&2].cell_id, 2);
                }
            }
            let mut completed = 0;
            for _ in 0..30 {
                let prior = session.get_vehicles().len();
                step_without_body_reuse(&mut session);
                completed += prior - session.get_vehicles().len();
            }
            assert_eq!(completed, 2);
            assert!(session.get_vehicles().is_empty());
        }
    }
}

#[test]
fn tail_recovery_does_not_enter_an_unreachable_forward_pocket() {
    for tail in [1, 2] {
        let mut cells = lanes();
        for cell in &mut cells {
            if cell.get_id() == 4 + (tail as i64 - 1) {
                cell.set_forward_id(-1);
                cell.set_right_id(-1);
            }
        }
        let mut session = Session::default(None);
        session.add_cells(cells);
        session.add_vehicles(vec![turning_truck(tail, true)]);
        for _ in 0..5 {
            step_without_body_reuse(&mut session);
            let truck = &session.get_vehicles()[&1];
            assert_eq!(truck.cell_id, 3);
            assert!(!truck.confusion);
            assert_eq!(truck.timer_non_maneuvers, tail as i64);
        }
    }
}

#[test]
fn tail_recovery_obeys_occupancy_signals_limits_and_dawdling() {
    for restriction in ["body", "red", "speed_zero", "dawdle"] {
        let mut cells = lanes();
        if restriction == "red" {
            cells
                .iter_mut()
                .find(|c| c.get_id() == 4)
                .unwrap()
                .set_state(CellState::Banned);
        }
        if restriction == "speed_zero" {
            *cells.iter_mut().find(|c| c.get_id() == 4).unwrap() = Cell::new(4)
                .with_forward_node(5)
                .with_right_node(56)
                .with_speed_limit(0)
                .build();
        }
        let mut session = Session::default(None);
        session.add_cells(cells);
        let mut truck = turning_truck(1, true);
        if restriction == "dawdle" {
            truck.timer_non_slowdown = 0;
            truck.slow_down_factor_p = 1.0;
            truck.slow_to_start_factor_p0 = 1.0;
        }
        session.add_vehicles(vec![truck]);
        if restriction == "body" {
            session.add_vehicles(vec![
                Vehicle::new(2)
                    .with_cell(4)
                    .with_behaviour(BehaviourType::Block)
                    .build(),
            ]);
        }
        step_without_body_reuse(&mut session);
        let truck = &session.get_vehicles()[&1];
        assert_eq!(truck.cell_id, 3, "{restriction}");
        assert_eq!(truck.tail_cells, vec![53]);
        assert_eq!(truck.timer_non_maneuvers, 1);
    }
}

#[test]
fn one_cell_vehicle_can_follow_the_immediate_lane_change() {
    let mut session = Session::default(None);
    session.add_cells(lanes());
    session.add_vehicles(vec![turning_truck(0, true)]);
    step_without_body_reuse(&mut session);
    assert_eq!(session.get_vehicles()[&1].cell_id, 55);
    assert_eq!(
        session.get_vehicles()[&1].intention.intention_maneuver,
        LaneChangeType::ChangeRight
    );
}
