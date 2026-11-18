use super::support::{car, speed};

#[test]
fn at_limit_override_attenuates_adaptation_and_none_preserves_it() {
    let mut vehicle = car(5, 0.0);
    assert_eq!(speed(&vehicle, 4, 8, 5), 4);
    vehicle.synchronization.free_speed_over_acceleration = Some(1.0);
    assert_eq!(speed(&vehicle, 4, 8, 5), 5);
    vehicle.synchronization.free_speed_over_acceleration = None;
    assert_eq!(speed(&vehicle, 4, 8, 5), 4);
}

#[test]
fn sublimit_adaptation_is_unchanged() {
    let mut vehicle = car(4, 0.0);
    vehicle.synchronization.free_speed_over_acceleration = Some(1.0);
    assert_eq!(speed(&vehicle, 3, 8, 5), 3);
    assert_eq!(speed(&vehicle, 4, 8, 5), 4);
}

#[test]
fn override_uses_positive_effective_current_limit() {
    let mut vehicle = car(3, 0.0);
    vehicle.synchronization.free_speed_over_acceleration = Some(1.0);
    assert_eq!(speed(&vehicle, 2, 5, 3), 3);
    vehicle.speed_limit = 3;
    assert_eq!(speed(&vehicle, 2, 5, 5), 3);
    vehicle.speed = 5;
    assert_eq!(speed(&vehicle, 2, 5, 3), 2);
}

#[test]
fn gap_clipping_and_negative_priority_remain() {
    let mut vehicle = car(5, 0.0);
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
