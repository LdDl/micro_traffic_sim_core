use super::support::{car, speed};

#[test]
fn full_speed_noise_can_be_suppressed_or_restored() {
    let mut vehicle = car(5, 1.0);
    assert_eq!(speed(&vehicle, 5, 8, 5), 5);
    vehicle.synchronization.free_speed_noise = None;
    assert_eq!(speed(&vehicle, 5, 8, 5), 4);
    vehicle.synchronization.free_speed_noise = Some(1.0);
    assert_eq!(speed(&vehicle, 5, 8, 5), 4);
}

#[test]
fn sublimit_noise_remains_even_when_preliminary_speed_reaches_limit() {
    assert_eq!(speed(&car(4, 1.0), 5, 8, 5), 4);
    assert_eq!(speed(&car(4, 1.0), 4, 8, 5), 3);
}

#[test]
fn slower_leader_still_causes_adaptation() {
    assert_eq!(speed(&car(5, 1.0), 3, 8, 5), 4);
}

#[test]
fn suppression_does_not_remove_physical_gap_limit() {
    assert_eq!(speed(&car(5, 1.0), 5, 3, 5), 3);
}

#[test]
fn established_restart_probability_keeps_priority() {
    let mut vehicle = car(0, 1.0);
    vehicle.synchronization.noise = 0.0;
    vehicle.stop_ticks = 1;
    assert_eq!(speed(&vehicle, 5, 8, 5), 1);
    vehicle.stop_ticks = 2;
    assert_eq!(speed(&vehicle, 5, 8, 5), 0);
}

#[test]
fn effective_road_and_vehicle_limits_both_apply() {
    assert_eq!(speed(&car(3, 1.0), 3, 8, 3), 3);
    let mut vehicle = car(3, 1.0);
    vehicle.speed_limit = 3;
    assert_eq!(speed(&vehicle, 3, 8, 5), 3);
    // Clipping an overspeed vehicle is not the same as cruising at the current limit.
    assert_eq!(speed(&car(5, 1.0), 3, 8, 3), 2);
}

#[test]
fn acceleration_and_slowdown_timers_keep_their_gates() {
    let mut vehicle = car(4, 1.0);
    vehicle.synchronization.noise = 0.0;
    vehicle.synchronization.over_acceleration = 1.0;
    vehicle.timer_non_acceleration = 1;
    assert_eq!(speed(&vehicle, 5, 8, 5), 4);
    vehicle = car(4, 1.0);
    vehicle.timer_non_slowdown = 1;
    assert_eq!(speed(&vehicle, 4, 8, 5), 4);
}
