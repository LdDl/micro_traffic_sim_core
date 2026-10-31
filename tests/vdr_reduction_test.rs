use micro_traffic_sim_core::agents::{RelativeRandomization, Vehicle};
use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::grid::road_network::GridRoads;
use micro_traffic_sim_core::intentions::find_intention;
use micro_traffic_sim_core::verbose::LocalLogger;
use std::collections::HashMap;

// Independent reference for an unobstructed straight route, with a fixed gap to its leader.
// Probabilities 0 and 1 exercise both randomization outcomes without depending on RNG order.
fn reference_speed(vehicle: &Vehicle, limit: i32, gap: i32) -> i32 {
    let mut speed = vehicle.speed.min(limit);
    if vehicle.timer_non_acceleration <= 0 || vehicle.speed == 0 {
        speed = (speed + 1).min(limit);
    }
    let probability = if vehicle.speed == 0 {
        vehicle.slow_to_start_factor_p0
    } else {
        vehicle.slow_down_factor_p
    };
    if vehicle.timer_non_slowdown <= 0 && probability == 1.0 {
        speed = (speed - 1).max(0);
    }
    speed.min(gap)
}

#[test]
fn vdr_reduction_matches_independent_longitudinal_reference() {
    let mut net = GridRoads::new();
    for cell in 1..=40 {
        net.add_cell(Cell::new(cell)
            .with_point(new_point(cell as f64, 0.0, None))
            .with_forward_node(if cell < 40 { cell + 1 } else { -1 })
            .with_left_node(-1)
            .with_right_node(-1)
            .with_speed_limit(8)
            .build());
    }
    let logger = LocalLogger::none();
    for limit in [1, 3, 8] {
        for speed in [0, 1, 2, 5, 9] {
            for gap in [0, 1, 2, 6, 15] {
                for leader_speed in [0, 2, 8] {
                    for lookahead in [0, 1, 15] {
                        for stood in [0, 5, i32::MAX] {
                            for acceleration_timer in [0, 1] {
                                for slowdown_timer in [0, 1] {
                                    for (p, p0) in [(0.0, 0.0), (0.0, 1.0), (1.0, 0.0), (1.0, 1.0)] {
                                        let mut vehicle = Vehicle::new(1)
                                            .with_cell(2)
                                            .with_destination(-1)
                                            .with_speed(speed)
                                            .with_speed_limit(limit)
                                            .with_slowdown(p)
                                            .with_slow_to_start(p0)
                                            .with_relative_randomization(RelativeRandomization { lookahead, ..RelativeRandomization::VDR })
                                            .build();
                                        vehicle.stop_ticks = stood;
                                        vehicle.timer_non_acceleration = acceleration_timer;
                                        vehicle.timer_non_slowdown = slowdown_timer;
                                        let occupancy = HashMap::from([(2, 1), (3 + gap as i64, 2)]);
                                        let speeds = HashMap::from([(2, speed), (3 + gap as i64, leader_speed)]);
                                        let expected = reference_speed(&vehicle, limit, gap);
                                        let intention = find_intention(&net, &occupancy, &speeds, &vehicle, &logger).unwrap();
                                        assert_eq!(intention.intention_speed, expected,
                                            "speed {speed}, limit {limit}, gap {gap}, leader {leader_speed}, lookahead {lookahead}, stood {stood}, timers {acceleration_timer}/{slowdown_timer}, p {p}, p0 {p0}");
                                        assert_eq!(intention.intention_cell_id, 2 + expected as i64);
                                        vehicle.intention = intention;
                                        vehicle.apply_intention();
                                        assert_eq!(vehicle.stop_ticks, if expected == 0 { stood.saturating_add(1) } else { 0 });
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
