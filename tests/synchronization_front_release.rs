use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::behaviour::BehaviourType;
use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::grid::road_network::GridRoads;
use micro_traffic_sim_core::simulation::grids_storage::GridsStorage;
use micro_traffic_sim_core::simulation::session::Session;

fn road(limit: i32) -> GridsStorage {
    let mut grid = GridRoads::new();
    for cell in 1..=60 {
        grid.add_cell(
            Cell::new(cell)
                .with_point(new_point(cell as f64, 0.0, None))
                .with_speed_limit(limit)
                .with_forward_node(if cell < 60 { cell + 1 } else { -1 })
                .with_left_node(-1)
                .with_right_node(-1)
                .build(),
        );
    }
    GridsStorage::new().with_vehicles_net(grid).build()
}

fn car(id: u64, cell: i64, speed: i32, limit: i32, gap: i32, p0: f64) -> Vehicle {
    let mut result = Vehicle::new(id)
        .with_cell(cell)
        .with_destination(60)
        .with_speed(speed)
        .with_speed_limit(limit)
        .with_behaviour(BehaviourType::Aggressive)
        .with_slowdown(0.0)
        .with_slow_to_start(p0)
        .with_standstill_gap(gap)
        .build();
    result.synchronization.time_gap = 2.55;
    result.synchronization.noise = 0.2 / 9.0;
    result.synchronization.free_speed_noise = Some(0.0);
    result.synchronization.over_acceleration = 0.052 / 9.0;
    result.synchronization.over_acceleration_low = 0.2 / 9.0;
    result.synchronization.pinch_speed = 4;
    result.synchronization.start_delay = 2;
    result.stop_ticks = if speed == 0 { 8 } else { 0 };
    result
}

#[test]
fn ready_compact_head_release_is_independent_of_leader_speed() {
    let mut checked = 0;
    for limit in 1..=8 {
        for leader_speed in 0..=limit {
            for empty_gap in 1..=8 {
                for head_gap in 0..=1 {
                    for follower_gap in 0..=1 {
                        for p0 in [0.0, 1.0] {
                            for stood in [2, 8] {
                                let follower_cell = 20 - i64::from(follower_gap + 1);
                                let mut session = Session::new(road(limit), None);
                                let mut head = car(1, 20, 0, limit, head_gap, p0);
                                head.stop_ticks = stood;
                                head.timer_non_acceleration = 10;
                                session.add_vehicles(vec![
                                    head,
                                    car(2, 21 + i64::from(empty_gap), leader_speed, limit, 0, 0.0),
                                    car(3, follower_cell, 0, limit, follower_gap, 0.0),
                                ]);
                                session.step().expect("single-lane step");
                                let cars = session.get_vehicles();
                                // The reserved cell is unavailable even when the leader is moving.
                                let expected = i32::from(p0 == 0.0 && empty_gap > head_gap);
                                assert_eq!(
                                    cars[&1].speed, expected,
                                    "limit {limit}, leader {leader_speed}, gap {empty_gap}, own {head_gap}, p0 {p0}"
                                );
                                assert_eq!(cars[&1].cell_id, 20 + i64::from(expected));
                                assert_eq!(cars[&3].cell_id, follower_cell);
                                checked += 1;
                            }
                        }
                    }
                }
            }
        }
    }
    assert_eq!(checked, 5632);
}

#[test]
fn current_leader_motion_alone_does_not_guarantee_a_release() {
    for empty_gap in [1, 2] {
        let mut session = Session::new(road(5), None);
        let head = car(1, 20, 0, 5, 1, 0.0);
        let mut leader = car(2, 21 + empty_gap, 1, 5, 0, 0.0);
        leader.strategy_type = BehaviourType::Block;
        session.add_vehicles(vec![head, leader, car(3, 18, 0, 5, 1, 0.0)]);
        session.step().expect("single-lane blocked leader");
        let expected = if empty_gap == 1 { 0 } else { 1 };
        assert_eq!(session.get_vehicles()[&1].speed, expected);
        assert_eq!(session.get_vehicles()[&1].cell_id, 20 + i64::from(expected));
    }
}
