use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::grid::road_network::GridRoads;
use micro_traffic_sim_core::intentions::find_intention;
use micro_traffic_sim_core::verbose::LocalLogger;
use std::collections::HashMap;

fn car(speed: i32) -> Vehicle {
    let mut vehicle = Vehicle::new(1)
        .with_cell(1)
        .with_destination(-1)
        .with_speed(speed)
        .with_speed_limit(5)
        .with_standstill_gap(0)
        .with_slow_to_start(1.0)
        .build();
    vehicle.synchronization.time_gap = 2.55;
    vehicle.synchronization.pinch_speed = 4;
    vehicle.synchronization.start_delay = 2;
    vehicle.synchronization.noise = 1.0;
    vehicle.synchronization.free_speed_noise = Some(0.0);
    vehicle
}

fn speed(vehicle: &Vehicle, leader: i32, gap: i64, limit: i32) -> i32 {
    let mut road = GridRoads::new();
    for x in 1..=30 {
        road.add_cell(
            Cell::new(x)
                .with_point(new_point(x as f64, 0.0, None))
                .with_forward_node(if x < 30 { x + 1 } else { -1 })
                .with_left_node(-1)
                .with_right_node(-1)
                .with_speed_limit(limit)
                .build(),
        );
    }
    let occupancy = HashMap::from([(1, 1), (gap + 2, 2)]);
    let speeds = HashMap::from([(1, vehicle.speed), (gap + 2, leader)]);
    find_intention(&road, &occupancy, &speeds, vehicle, &LocalLogger::none())
        .unwrap()
        .intention_speed
}

#[test]
fn full_speed_noise_can_be_suppressed_or_restored() {
    let mut vehicle = car(5);
    assert_eq!(speed(&vehicle, 5, 8, 5), 5);
    vehicle.synchronization.free_speed_noise = None;
    assert_eq!(speed(&vehicle, 5, 8, 5), 4);
    vehicle.synchronization.free_speed_noise = Some(1.0);
    assert_eq!(speed(&vehicle, 5, 8, 5), 4);
}

#[test]
fn sublimit_noise_remains_even_when_preliminary_speed_reaches_limit() {
    assert_eq!(speed(&car(4), 5, 8, 5), 4);
    assert_eq!(speed(&car(4), 4, 8, 5), 3);
}

#[test]
fn slower_leader_still_causes_adaptation() {
    assert_eq!(speed(&car(5), 3, 8, 5), 4);
}

#[test]
fn suppression_does_not_remove_physical_gap_limit() {
    assert_eq!(speed(&car(5), 5, 3, 5), 3);
}

#[test]
fn established_restart_probability_keeps_priority() {
    let mut vehicle = car(0);
    vehicle.synchronization.noise = 0.0;
    vehicle.stop_ticks = 1;
    assert_eq!(speed(&vehicle, 5, 8, 5), 1);
    vehicle.stop_ticks = 2;
    assert_eq!(speed(&vehicle, 5, 8, 5), 0);
}

#[test]
fn effective_road_and_vehicle_limits_both_apply() {
    assert_eq!(speed(&car(3), 3, 8, 3), 3);
    let mut vehicle = car(3);
    vehicle.speed_limit = 3;
    assert_eq!(speed(&vehicle, 3, 8, 5), 3);
    // Clipping an overspeed vehicle is not the same as cruising at the current limit.
    assert_eq!(speed(&car(5), 3, 8, 3), 2);
}

#[test]
fn acceleration_and_slowdown_timers_keep_their_gates() {
    let mut vehicle = car(4);
    vehicle.synchronization.noise = 0.0;
    vehicle.synchronization.over_acceleration = 1.0;
    vehicle.timer_non_acceleration = 1;
    assert_eq!(speed(&vehicle, 5, 8, 5), 4);
    vehicle = car(4);
    vehicle.timer_non_slowdown = 1;
    assert_eq!(speed(&vehicle, 4, 8, 5), 4);
}
