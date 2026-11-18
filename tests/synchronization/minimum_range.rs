use micro_traffic_sim_core::agents::{SynchronizationParameters, Vehicle, VehicleIntention};
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::grid::road_network::GridRoads;
use micro_traffic_sim_core::intentions::find_intention;
use micro_traffic_sim_core::simulation::clearance::LongitudinalClearance;
use micro_traffic_sim_core::simulation::session::Session;
use micro_traffic_sim_core::verbose::LocalLogger;
use std::collections::HashMap;

fn car(speed: i32, minimum: i32, noise: i32) -> Vehicle {
    let negative = if noise < 0 { 1.0 } else { 0.0 };
    let positive = if noise > 0 { 1.0 } else { 0.0 };
    Vehicle::new(1)
        .with_cell(1)
        .with_destination(-1)
        .with_speed(speed)
        .with_speed_limit(8)
        .with_standstill_gap(0)
        .with_slowdown(negative)
        .with_slow_to_start(negative)
        .with_synchronization(SynchronizationParameters {
            minimum_moving_range: minimum,
            noise: negative,
            free_speed_noise: Some(negative),
            over_acceleration: positive,
            over_acceleration_low: positive,
            free_speed_over_acceleration: Some(positive),
            ..SynchronizationParameters::COARSE_GRID
        })
        .build()
}

fn road(limit: i32) -> GridRoads {
    let mut road = GridRoads::new();
    for id in 1..=40 {
        road.add_cell(
            Cell::new(id)
                .with_forward_node(if id < 40 { id + 1 } else { -1 })
                .with_left_node(-1)
                .with_right_node(-1)
                .with_speed_limit(limit)
                .build(),
        );
    }
    road
}

fn intention(road: &GridRoads, vehicle: &Vehicle, leader: i32, gap: i32) -> VehicleIntention {
    let leader_cell = i64::from(gap) + 2;
    let occupancy = HashMap::from([(1, 1), (leader_cell, 2)]);
    let speeds = HashMap::from([(1, vehicle.speed), (leader_cell, leader)]);
    find_intention(road, &occupancy, &speeds, vehicle, &LocalLogger::none()).unwrap()
}

#[test]
fn minimum_range_keeps_the_speed_one_family_inside_adaptation() {
    let road = road(8);
    for (gap, baseline, extended) in [(2, 1, 1), (3, 2, 1), (4, 2, 2)] {
        assert_eq!(
            intention(&road, &car(1, 0, 0), 1, gap).intention_speed,
            baseline
        );
        assert_eq!(
            intention(&road, &car(1, 3, 0), 1, gap).intention_speed,
            extended
        );
    }
}

#[test]
fn floor_preserves_the_old_fast_gate_instead_of_widening_its_coefficient() {
    let road = road(8);
    let baseline = car(5, 0, 0);
    let floored = car(5, 3, 0);
    let mut widened = car(5, 0, 0);
    widened.synchronization.time_gap = 3.0;
    assert_eq!(intention(&road, &baseline, 4, 13).intention_speed, 6);
    assert_eq!(intention(&road, &floored, 4, 13).intention_speed, 6);
    assert_eq!(intention(&road, &widened, 4, 13).intention_speed, 4);
    let road = self::road(5);
    assert_eq!(intention(&road, &baseline, 4, 13).intention_speed, 5);
    assert_eq!(intention(&road, &floored, 4, 13).intention_speed, 5);
    assert_eq!(intention(&road, &widened, 4, 13).intention_speed, 4);
}

#[test]
fn higher_speed_intentions_match_for_every_deterministic_noise_branch() {
    let road = road(8);
    for speed in 2..=8 {
        for leader in 0..=8 {
            for gap in 0..=22 {
                for noise in [-1, 0, 1] {
                    for timer in [0, 3] {
                        let mut baseline = car(speed, 0, noise);
                        let mut floored = car(speed, 3, noise);
                        baseline.timer_non_acceleration = timer;
                        floored.timer_non_acceleration = timer;
                        assert_eq!(
                            format!("{:?}", intention(&road, &baseline, leader, gap)),
                            format!("{:?}", intention(&road, &floored, leader, gap)),
                            "speed {speed}, leader {leader}, gap {gap}, noise {noise}, timer {timer}"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn stopped_restart_and_acceleration_timer_are_independent_of_the_floor() {
    let road = road(8);
    for stood in [0, 1, 2, 8] {
        for leader in [0, 1, 5] {
            for gap in 0..=5 {
                for noise in [-1, 0, 1] {
                    let mut baseline = car(0, 0, noise);
                    let mut floored = car(0, 30, noise);
                    baseline.stop_ticks = stood;
                    floored.stop_ticks = stood;
                    baseline.timer_non_acceleration = 10;
                    floored.timer_non_acceleration = 10;
                    assert_eq!(
                        format!("{:?}", intention(&road, &baseline, leader, gap)),
                        format!("{:?}", intention(&road, &floored, leader, gap))
                    );
                }
            }
        }
    }
}

#[test]
fn positive_floor_does_not_enable_disabled_synchronization() {
    let road = road(8);
    for speed in 0..=8 {
        for leader in [0, 1, 8] {
            for gap in 0..=10 {
                for noise in [-1, 0] {
                    let mut baseline = car(speed, 0, noise);
                    let mut floored = car(speed, 30, noise);
                    baseline.synchronization.time_gap = 0.0;
                    floored.synchronization.time_gap = 0.0;
                    assert_eq!(
                        format!("{:?}", intention(&road, &baseline, leader, gap)),
                        format!("{:?}", intention(&road, &floored, leader, gap))
                    );
                }
            }
        }
    }
}

#[test]
fn floor_eligibility_uses_positive_speed_after_clipping_to_the_limit() {
    assert_eq!(intention(&road(1), &car(5, 0, 0), 0, 3).intention_speed, 1);
    assert_eq!(intention(&road(1), &car(5, 3, 0), 0, 3).intention_speed, 0);
    assert_eq!(intention(&road(0), &car(5, 3, 0), 0, 3).intention_speed, 0);
}

#[test]
fn an_extended_range_never_permits_reusing_the_leaders_initial_cell() {
    for policy in [
        LongitudinalClearance::Conservative,
        LongitudinalClearance::MovingLeader,
    ] {
        for minimum in [0, 3, 30] {
            for reserve in [0, 1] {
                for gap in 0..=6 {
                    let mut session = Session::default(None);
                    session.set_longitudinal_clearance(policy);
                    session.add_cells(
                        (1..=40)
                            .map(|id| {
                                Cell::new(id)
                                    .with_forward_node(if id < 40 { id + 1 } else { -1 })
                                    .with_left_node(-1)
                                    .with_right_node(-1)
                                    .with_speed_limit(8)
                                    .build()
                            })
                            .collect(),
                    );
                    let mut follower = car(5, minimum, 0);
                    follower.standstill_gap = reserve;
                    let mut leader = car(5, minimum, 0);
                    leader.id = 2;
                    leader.cell_id = i64::from(gap) + 2;
                    let initial_leader_cell = leader.cell_id;
                    session.add_vehicles(vec![follower, leader]);
                    session.step().unwrap();
                    let vehicles = session.get_vehicles();
                    assert!(vehicles[&1].cell_id < initial_leader_cell);
                    assert!(
                        vehicles[&1]
                            .intention
                            .intermediate_cells
                            .iter()
                            .all(|&cell| cell < initial_leader_cell)
                    );
                    assert!(
                        vehicles[&2].cell_id - vehicles[&1].cell_id - 1
                            >= i64::from(reserve.min(gap))
                    );
                }
            }
        }
    }
}
