use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::simulation::clearance::LongitudinalClearance;
use micro_traffic_sim_core::simulation::session::Session;

fn road(policy: LongitudinalClearance) -> Session {
    let mut session = Session::default(None);
    session.set_longitudinal_clearance(policy);
    session.add_cells((1..=30).map(|id| Cell::new(id)
        .with_forward_node(if id < 30 { id + 1 } else { -1 })
        .with_left_node(-1).with_right_node(-1).with_speed_limit(1).build()).collect());
    session
}

fn car(id: u64, cell: i64, speed: i32, reserve: i32, rear_margin: i32) -> Vehicle {
    Vehicle::new(id).with_cell(cell).with_speed(speed).with_speed_limit(1)
        .with_destination(-1).with_trip_destination(30)
        .with_standstill_gap(reserve).with_min_safe_distance(rear_margin)
        .with_slowdown(0.0).with_slow_to_start(0.0).build()
}

#[test]
fn rear_lane_change_margin_does_not_set_forward_reserve() {
    for rear_margin in [0, 1, 3, 64] {
        for (policy, reserve, expected) in [
            (LongitudinalClearance::Conservative, 0, 11),
            (LongitudinalClearance::Conservative, 1, 10),
            (LongitudinalClearance::MovingLeader, 0, 11),
            (LongitudinalClearance::MovingLeader, 1, 11),
        ] {
            let mut session = road(policy);
            session.add_vehicles(vec![car(1, 10, 1, reserve, rear_margin), car(2, 12, 1, 0, 0)]);
            session.step().unwrap();
            let vehicles = session.get_vehicles();
            assert_eq!(vehicles[&1].cell_id, expected);
            assert_eq!(vehicles[&2].cell_id, 13);
            assert!(vehicles[&1].cell_id < 12);
            assert!(vehicles[&1].intention.intermediate_cells.iter().all(|&cell| cell < 12));
        }
    }
}

#[test]
fn both_policies_reject_same_tick_reuse_of_a_leader_body_cell() {
    for policy in [LongitudinalClearance::Conservative, LongitudinalClearance::MovingLeader] {
        for reserve in [0, 1] {
            for leader_speed in [0, 1] {
                let mut session = road(policy);
                session.add_vehicles(vec![car(1, 10, 1, reserve, 0), car(2, 11, leader_speed, 0, 0)]);
                session.step().unwrap();
                let vehicles = session.get_vehicles();
                assert_eq!(vehicles[&1].cell_id, 10);
                assert_eq!(vehicles[&1].speed, 0);
                assert_eq!(vehicles[&2].cell_id, 12);
            }
        }
    }
}

#[test]
fn a_same_tick_start_does_not_supply_moving_leader_credit() {
    let mut session = road(LongitudinalClearance::MovingLeader);
    session.add_vehicles(vec![car(1, 10, 1, 1, 0), car(2, 12, 0, 0, 0)]);
    session.step().unwrap();
    let vehicles = session.get_vehicles();
    assert_eq!(vehicles[&1].cell_id, 10);
    assert_eq!(vehicles[&2].cell_id, 13);
}
