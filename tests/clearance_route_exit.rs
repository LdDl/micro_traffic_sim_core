use micro_traffic_sim_core::agents::{SynchronizationParameters, Vehicle};
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::simulation::{clearance::LongitudinalClearance, session::Session};

#[test]
fn reserved_spacing_does_not_trap_a_planned_exit_behind_the_abandoned_queue() {
    for policy in [
        LongitudinalClearance::Conservative,
        LongitudinalClearance::MovingLeader,
    ] {
        for sync in [false, true] {
            let mut session = Session::default(None);
            session.set_longitudinal_clearance(policy);
            session.add_cells(
                (0..20)
                    .chain(100..120)
                    .map(|id| {
                        Cell::new(id)
                            .with_forward_node(if id == 19 || id == 119 { -1 } else { id + 1 })
                            .with_left_node(if id == 4 { 104 } else { -1 })
                            .with_right_node(-1)
                            .with_speed_limit(5)
                            .build()
                    })
                    .collect(),
            );
            let synchronization = if sync {
                SynchronizationParameters {
                    noise: 0.0,
                    over_acceleration: 0.0,
                    over_acceleration_low: 0.0,
                    free_speed_over_acceleration: Some(0.0),
                    ..SynchronizationParameters::COARSE_GRID
                }
            } else {
                SynchronizationParameters::DISABLED
            };
            let mut car = Vehicle::new(1)
                .with_cell(3)
                .with_speed(1)
                .with_speed_limit(5)
                .with_destination(119)
                .with_trip_destination(119)
                .with_standstill_gap(1)
                .with_slowdown(0.0)
                .with_slow_to_start(0.0)
                .with_synchronization(synchronization)
                .build();
            car.cached_route = vec![
                3, 4, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118,
                119,
            ];
            session.add_vehicles(vec![
                car,
                Vehicle::new(2)
                    .with_cell(5)
                    .with_speed(0)
                    .with_speed_limit(0)
                    .with_destination(-1)
                    .with_trip_destination(-1)
                    .build(),
            ]);
            session.step().unwrap();
            assert_eq!(
                session.get_vehicles()[&1].cell_id,
                4,
                "must reach the planned exit: {policy:?}, sync {sync}"
            );
            session.step().unwrap();
            assert!(
                session.get_vehicles()[&1].cell_id >= 104,
                "must take the planned exit"
            );
            assert_eq!(session.get_vehicles()[&2].cell_id, 5);
        }
    }
}
