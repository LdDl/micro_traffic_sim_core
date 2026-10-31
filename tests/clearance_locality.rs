use micro_traffic_sim_core::agents::{SynchronizationParameters, Vehicle};
use micro_traffic_sim_core::geom::{new_point, SRID};
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::grid::zones::ZoneType;
use micro_traffic_sim_core::simulation::session::Session;

fn road(end: i64) -> Session {
    let mut session = Session::default(Some(SRID::WGS84));
    session.add_cells((0..=end).map(|x| Cell::new(x)
        .with_forward_node(if x < end { x + 1 } else { -1 })
        .with_left_node(-1).with_right_node(-1).with_speed_limit(5)
        .with_zone_type(ZoneType::Common).with_meso_link(0)
        .with_point(new_point(x as f64 * 1e-4, 0.0, Some(SRID::WGS84)))
        .build()).collect());
    session
}

fn car(id: u64, x: i64, speed: i32, gap: i32, sync: bool, end: i64) -> Vehicle {
    let mut car = Vehicle::new(id).with_cell(x).with_speed(speed).with_speed_limit(5)
        .with_destination(-1).with_trip_destination(end).with_standstill_gap(gap)
        .with_slowdown(0.0).with_slow_to_start(0.0).build();
    if sync {
        car.synchronization = SynchronizationParameters {
            noise: 0.0, over_acceleration: 0.0, over_acceleration_low: 0.0,
            free_speed_over_acceleration: Some(0.0),
            ..SynchronizationParameters::COARSE_GRID
        };
    }
    car
}

#[test]
fn forward_clearance_matches_scalar_rule_and_preserves_spacing() {
    let mut checked = 0;
    for sync in [false, true] {
        for speed in 0..=5 {
            for leader_speed in 0..=5 {
                for distance in 0..=12 {
                    for reserve in 0..=3 {
                        let mut session = road(60);
                        session.add_vehicles(vec![car(1, 10, speed, reserve, sync, 60),
                            car(2, 11 + i64::from(distance), leader_speed, 0, sync, 60)]);
                        session.step().unwrap();
                        let adapted = if sync && speed > 0 && f64::from(distance) <= (2.55 * f64::from(speed)).floor() {
                            speed + (leader_speed - speed).signum()
                        } else {
                            speed + 1
                        };
                        let expected = adapted.clamp(0, 5).min((distance - reserve).max(0));
                        let cars = session.get_vehicles();
                        let follower = &cars[&1];
                        let leader = &cars[&2];
                        assert_eq!(follower.speed, expected,
                            "sync {sync}, v {speed}, leader {leader_speed}, distance {distance}, reserve {reserve}");
                        assert_eq!(follower.cell_id, 10 + i64::from(expected));
                        let final_gap = leader.cell_id - follower.cell_id - 1;
                        assert!(final_gap >= i64::from(distance.min(reserve)));
                        checked += 1;
                    }
                }
            }
        }
    }
    assert_eq!(checked, 3744);
}

#[test]
fn distant_stop_cannot_change_rear_motion_through_a_gap_cascade() {
    for count in [4, 16, 64, 256] {
        for gap in [0, 1] {
            let mut rear = Vec::new();
            for terminal_stopped in [false, true] {
                let end = 2 * count + 40;
                let mut session = road(end);
                let vehicles = (0..count).map(|i| {
                    let stopped = terminal_stopped && i + 1 == count;
                    let mut vehicle = car(i as u64 + 1, 10 + 2 * i, if stopped { 0 } else { 1 }, gap, true, end);
                    vehicle.slow_to_start_factor_p0 = 1.0;
                    vehicle.stop_ticks = if stopped { 8 } else { 0 };
                    vehicle
                }).collect();
                session.add_vehicles(vehicles);
                session.step().unwrap();
                let cars = session.get_vehicles();
                rear.push((cars[&1].cell_id, cars[&1].speed));
                for id in 1..count as u64 {
                    assert!(cars[&(id + 1)].cell_id - cars[&id].cell_id - 1 >= i64::from(gap));
                }
            }
            assert_eq!(rear[0], rear[1], "count {count}, gap {gap}");
            assert_eq!(rear[0], if gap == 0 { (11, 1) } else { (10, 0) });
        }
    }
}
