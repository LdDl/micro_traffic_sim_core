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
    vehicle.synchronization.noise = 0.0;
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
fn at_limit_override_attenuates_adaptation_and_none_preserves_it() {
    let mut vehicle = car(5);
    assert_eq!(speed(&vehicle, 4, 8, 5), 4);
    vehicle.synchronization.free_speed_over_acceleration = Some(1.0);
    assert_eq!(speed(&vehicle, 4, 8, 5), 5);
    vehicle.synchronization.free_speed_over_acceleration = None;
    assert_eq!(speed(&vehicle, 4, 8, 5), 4);
}

#[test]
fn sublimit_adaptation_is_unchanged() {
    let mut vehicle = car(4);
    vehicle.synchronization.free_speed_over_acceleration = Some(1.0);
    assert_eq!(speed(&vehicle, 3, 8, 5), 3);
    assert_eq!(speed(&vehicle, 4, 8, 5), 4);
}

#[test]
fn override_uses_positive_effective_current_limit() {
    let mut vehicle = car(3);
    vehicle.synchronization.free_speed_over_acceleration = Some(1.0);
    assert_eq!(speed(&vehicle, 2, 5, 3), 3);
    vehicle.speed_limit = 3;
    assert_eq!(speed(&vehicle, 2, 5, 5), 3);
    vehicle.speed = 5;
    assert_eq!(speed(&vehicle, 2, 5, 3), 2);
}

#[test]
fn gap_clipping_and_negative_priority_remain() {
    let mut vehicle = car(5);
    vehicle.synchronization.free_speed_over_acceleration = Some(1.0);
    assert_eq!(speed(&vehicle, 3, 3, 5), 3);
    vehicle.synchronization.free_speed_noise = Some(1.0);
    assert_eq!(speed(&vehicle, 4, 8, 5), 3);
}

#[test]
fn optional_probability_is_clamped() {
    use micro_traffic_sim_core::agents::SynchronizationParameters;
    let mut sync = SynchronizationParameters::COARSE_GRID;
    assert_eq!(sync.free_speed_over_acceleration, None);
    sync.free_speed_over_acceleration = Some(f64::NAN);
    assert_eq!(sync.clamped().free_speed_over_acceleration, Some(0.0));
    sync.free_speed_over_acceleration = Some(2.0);
    assert_eq!(sync.clamped().free_speed_over_acceleration, Some(1.0));
}
