use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::grid::road_network::GridRoads;
use micro_traffic_sim_core::intentions::find_intention;
use micro_traffic_sim_core::verbose::LocalLogger;
use std::collections::HashMap;

pub fn car(speed: i32, noise: f64) -> Vehicle {
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
    vehicle.synchronization.noise = noise;
    vehicle.synchronization.free_speed_noise = Some(0.0);
    vehicle
}

pub fn speed(vehicle: &Vehicle, leader: i32, gap: i64, limit: i32) -> i32 {
    // 1(F)   [gap empty cells]   (2 + gap)(L)
    // Only the follower's intention is measured. The speed map supplies L's
    // current speed; the test controls the follower's noise separately.
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
