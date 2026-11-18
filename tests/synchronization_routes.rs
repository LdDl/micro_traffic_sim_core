use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::grid::road_network::GridRoads;
use micro_traffic_sim_core::intentions::find_intention;
use micro_traffic_sim_core::verbose::LocalLogger;
use std::collections::HashMap;

fn road() -> GridRoads {
    let mut net = GridRoads::new();
    for id in 1..=12 {
        net.add_cell(
            Cell::new(id)
                .with_point(new_point(id as f64, 0.0, None))
                .with_forward_node(if id < 12 { id + 1 } else { -1 })
                .with_left_node(if id == 3 { 10 } else { -1 })
                .with_right_node(-1)
                .with_speed_limit(5)
                .build(),
        );
    }
    net
}

#[test]
fn planned_exit_does_not_follow_the_queue_left_behind() {
    let net = road();
    let mut car = Vehicle::new(1)
        .with_cell(1)
        .with_destination(12)
        .with_speed(2)
        .with_speed_limit(5)
        .with_standstill_gap(0)
        .build();
    car.cached_route = vec![1, 2, 3, 10, 11, 12];
    car.synchronization.time_gap = 2.55;
    let occupancy = HashMap::from([(1, 1), (4, 2)]);
    let speeds = HashMap::from([(1, 2), (4, 0)]);
    let revised = find_intention(&net, &occupancy, &speeds, &car, &LocalLogger::none()).unwrap();
    assert!(revised.intention_speed > 1);
    assert_ne!(revised.intention_cell_id, 4);
}

#[test]
fn route_leader_before_the_exit_still_limits_motion() {
    let net = road();
    let mut car = Vehicle::new(1)
        .with_cell(1)
        .with_destination(12)
        .with_speed(2)
        .with_speed_limit(5)
        .with_standstill_gap(0)
        .build();
    car.cached_route = vec![1, 2, 3, 10, 11, 12];
    car.synchronization.time_gap = 2.55;
    let occupancy = HashMap::from([(1, 1), (3, 2)]);
    let speeds = HashMap::from([(1, 2), (3, 0)]);
    let actual = find_intention(&net, &occupancy, &speeds, &car, &LocalLogger::none()).unwrap();
    assert_eq!(actual.intention_speed, 1);
    assert_eq!(actual.intention_cell_id, 2);
}

#[test]
fn restart_delay_distinguishes_short_and_established_stops() {
    let net = road();
    let occupancy = HashMap::from([(1, 1)]);
    let speeds = HashMap::from([(1, 0)]);
    let mut car = Vehicle::new(1)
        .with_cell(1)
        .with_destination(-1)
        .with_speed(0)
        .with_speed_limit(5)
        .with_standstill_gap(0)
        .with_slow_to_start(1.0)
        .build();
    car.synchronization.time_gap = 2.55;
    car.synchronization.start_delay = 5;
    for stood in [0, 1, 4, 5, 8] {
        car.stop_ticks = stood;
        let actual = find_intention(&net, &occupancy, &speeds, &car, &LocalLogger::none()).unwrap();
        assert_eq!(actual.intention_speed, if stood < 5 { 1 } else { 0 });
    }
}
