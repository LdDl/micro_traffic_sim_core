use micro_traffic_sim_core::agents::{SynchronizationParameters, Vehicle};
use micro_traffic_sim_core::behaviour::{BehaviourParameters, BehaviourType};
use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::simulation::session::Session;
use micro_traffic_sim_core::trips::trip::{Trip, TripType};

struct SpawnedConfiguration {
    synchronization: SynchronizationParameters,
    standstill_gap: i32,
}

fn spawned(
    parameters: Option<SynchronizationParameters>,
    share: Option<f64>,
) -> SpawnedConfiguration {
    let mut session = Session::default(None);
    session.add_cells(
        (1..=40)
            .map(|id| {
                Cell::new(id)
                    .with_point(new_point(id as f64, 0.0, None))
                    .with_forward_node(if id < 40 { id + 1 } else { -1 })
                    .with_left_node(-1)
                    .with_right_node(-1)
                    .with_speed_limit(5)
                    .build()
            })
            .collect(),
    );
    let mut builder = Trip::new(1, 40, TripType::Constant)
        .with_time(1)
        .with_allowed_behaviour_type(BehaviourType::Aggressive)
        .with_standstill_gap(0);
    if let Some(parameters) = parameters {
        builder = builder.with_synchronization(parameters);
    }
    if let Some(share) = share {
        builder = builder.with_standstill_gap_share(share);
    }
    session.add_trip(builder.build());
    session.generate_vehicles();
    assert_eq!(session.get_vehicles().len(), 1);
    let vehicle = session.get_vehicles().values().next().unwrap();
    SpawnedConfiguration {
        synchronization: vehicle.synchronization,
        standstill_gap: vehicle.standstill_gap,
    }
}

#[test]
fn existing_constructors_and_presets_keep_synchronization_disabled() {
    assert_eq!(
        Vehicle::new(1).build().synchronization,
        SynchronizationParameters::DISABLED
    );
    for kind in [
        BehaviourType::Aggressive,
        BehaviourType::Cooperative,
        BehaviourType::Undefined,
        BehaviourType::LimitSpeedByTrip,
        BehaviourType::Block,
    ] {
        assert_eq!(
            BehaviourParameters::from_behaviour_type(kind).synchronization(),
            SynchronizationParameters::DISABLED
        );
    }
    assert_eq!(
        spawned(None, None).synchronization,
        SynchronizationParameters::DISABLED
    );
    assert_eq!(spawned(None, None).standstill_gap, 0);
}

#[test]
fn per_trip_parameters_reach_the_actual_generated_vehicle() {
    let parameters = SynchronizationParameters {
        time_gap: 3.125,
        minimum_moving_range: 4,
        noise: 0.07,
        free_speed_noise: Some(0.013),
        free_speed_over_acceleration: Some(0.041),
        over_acceleration: 0.021,
        over_acceleration_low: 0.08,
        pinch_speed: 3,
        start_delay: 7,
    };
    assert_eq!(
        spawned(Some(parameters), Some(1.0)).synchronization,
        parameters
    );
    assert_eq!(spawned(Some(parameters), Some(1.0)).standstill_gap, 1);
    assert_eq!(spawned(Some(parameters), Some(0.0)).standstill_gap, 0);
    let behaviour = BehaviourParameters::from_behaviour_type(BehaviourType::Cooperative)
        .with_synchronization(parameters);
    assert_eq!(behaviour.synchronization(), parameters);
    assert_eq!(
        behaviour
            .with_synchronization(SynchronizationParameters::DISABLED)
            .synchronization(),
        SynchronizationParameters::DISABLED
    );
}

#[test]
fn invalid_configuration_is_bounded_before_generation() {
    let parameters = SynchronizationParameters {
        time_gap: f64::INFINITY,
        minimum_moving_range: -7,
        noise: f64::NAN,
        free_speed_noise: Some(-1.0),
        free_speed_over_acceleration: Some(-0.2),
        over_acceleration: 2.0,
        over_acceleration_low: -2.0,
        pinch_speed: -1,
        start_delay: -2,
    };
    let vehicle = spawned(Some(parameters), Some(f64::NAN));
    assert_eq!(vehicle.synchronization.time_gap, 0.0);
    assert_eq!(vehicle.synchronization.minimum_moving_range, 0);
    assert_eq!(vehicle.synchronization.noise, 0.0);
    assert_eq!(vehicle.synchronization.free_speed_noise, Some(0.0));
    assert_eq!(
        vehicle.synchronization.free_speed_over_acceleration,
        Some(0.0)
    );
    assert_eq!(vehicle.synchronization.over_acceleration, 1.0);
    assert_eq!(vehicle.synchronization.over_acceleration_low, 0.0);
    assert_eq!(vehicle.synchronization.pinch_speed, 0);
    assert_eq!(vehicle.synchronization.start_delay, 0);
    assert_eq!(vehicle.standstill_gap, 0);
}

#[test]
fn deterministic_synchronization_keeps_distinct_speeds_at_the_same_density() {
    for speed in [2, 3] {
        let mut session = Session::default(None);
        session.add_cells(
            (0..80)
                .map(|id| {
                    Cell::new(id)
                        .with_point(new_point(id as f64, 0.0, None))
                        .with_forward_node((id + 1) % 80)
                        .with_left_node(-1)
                        .with_right_node(-1)
                        .with_speed_limit(5)
                        .build()
                })
                .collect(),
        );
        session.add_vehicles(
            (0..20)
                .map(|id| {
                    Vehicle::new(id)
                        .with_cell((4 * id) as i64)
                        .with_destination(-1)
                        .with_speed(speed)
                        .with_speed_limit(5)
                        .with_standstill_gap(0)
                        .with_slow_to_start(0.0)
                        .with_synchronization(SynchronizationParameters {
                            time_gap: 2.55,
                            ..SynchronizationParameters::DISABLED
                        })
                        .build()
                })
                .collect(),
        );
        for _ in 0..100 {
            session.step().unwrap();
            assert_eq!(session.get_vehicles().len(), 20);
            assert!(
                session
                    .get_vehicles()
                    .values()
                    .all(|car| car.speed == speed)
            );
        }
    }
}

fn long_vehicle_probability(share: f64, explicit: Option<f64>) -> f64 {
    let mut session = Session::default(None);
    session.add_cells(
        (1..=3)
            .map(|id| {
                Cell::new(id)
                    .with_point(new_point(id as f64, 0.0, None))
                    .with_forward_node(if id < 3 { id + 1 } else { -1 })
                    .with_left_node(-1)
                    .with_right_node(-1)
                    .with_speed_limit(5)
                    .build()
            })
            .collect(),
    );
    let mut builder = Trip::new(1, 3, TripType::Constant)
        .with_time(1)
        .with_allowed_behaviour_type(BehaviourType::Aggressive)
        .with_vehicle_tail_size(2)
        .with_standstill_gap_share(share)
        .with_synchronization(SynchronizationParameters::COARSE_GRID);
    if let Some(value) = explicit {
        builder = builder.with_slow_to_start(value);
    }
    session.add_trip(builder.build());
    session.generate_vehicles();
    assert_eq!(session.get_vehicles().len(), 1);
    session
        .get_vehicles()
        .values()
        .next()
        .unwrap()
        .slow_to_start_factor_p0
}

#[test]
fn length_calibration_uses_the_shared_population_mean() {
    for (share, expected) in [
        (0.0, 97.0 / 120.0),
        (2.0 / 3.0, 65.0 / 88.0),
        (1.0, 57.0 / 80.0),
    ] {
        assert!((long_vehicle_probability(share, None) - expected).abs() < 1e-12);
    }
    assert_eq!(long_vehicle_probability(2.0 / 3.0, Some(0.6)), 0.6);
    for share in [0.0, 2.0 / 3.0, 1.0] {
        assert_eq!(Vehicle::slow_to_start_for_mean_gap(0.425, 1, share), 0.425);
    }
}
