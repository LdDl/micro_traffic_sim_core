use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::simulation::clearance::LongitudinalClearance;
use micro_traffic_sim_core::simulation::grids_storage::GridsStorage;
use micro_traffic_sim_core::simulation::session::Session;

#[test]
fn both_session_constructors_preserve_conservative_defaults() {
    assert_eq!(LongitudinalClearance::default(), LongitudinalClearance::Conservative);
    assert_eq!(Session::default(None).get_longitudinal_clearance(), LongitudinalClearance::Conservative);
    let session = Session::new(GridsStorage::new().build(), None);
    assert_eq!(session.get_longitudinal_clearance(), LongitudinalClearance::Conservative);
}

#[test]
fn explicit_policy_selects_the_declared_moving_leader_contract() {
    for (policy, expected) in [(LongitudinalClearance::Conservative, 0), (LongitudinalClearance::MovingLeader, 1)] {
        let mut session = Session::default(None);
        session.set_longitudinal_clearance(policy);
        assert_eq!(session.get_longitudinal_clearance(), policy);
        session.add_cells((0..=20).map(|x| Cell::new(x)
            .with_forward_node(if x < 20 { x + 1 } else { -1 })
            .with_left_node(-1).with_right_node(-1).with_speed_limit(1).build()).collect());
        session.add_vehicles(vec![
            Vehicle::new(1).with_cell(5).with_speed(0).with_speed_limit(1)
                .with_trip_destination(20).with_standstill_gap(1)
                .with_slowdown(0.0).with_slow_to_start(0.0).build(),
            Vehicle::new(2).with_cell(7).with_speed(1).with_speed_limit(1)
                .with_trip_destination(20).with_standstill_gap(0)
                .with_slowdown(0.0).with_slow_to_start(0.0).build(),
        ]);
        session.step().unwrap();
        assert_eq!(session.get_vehicles()[&1].speed, expected);
        assert_eq!(session.get_vehicles()[&1].cell_id, 5 + i64::from(expected));
        assert_eq!(session.get_vehicles()[&2].speed, 1);
    }
}
