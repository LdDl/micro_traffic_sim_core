use micro_traffic_sim_core::agents::{RelativeRandomization, Vehicle};
use micro_traffic_sim_core::behaviour::BehaviourType;
use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::grid::road_network::GridRoads;
use micro_traffic_sim_core::simulation::grids_storage::GridsStorage;
use micro_traffic_sim_core::simulation::session::Session;

/// Single lane road of cells 1..=40, speed limit 3.
fn straight_road() -> GridsStorage {
    let mut grid = GridRoads::new();
    for i in 1..=40 {
        let forward = if i < 40 { i + 1 } else { -1 };
        grid.add_cell(
            Cell::new(i)
                .with_point(new_point(i as f64, 0.0, None))
                .with_speed_limit(3)
                .with_forward_node(forward)
                .with_left_node(-1)
                .with_right_node(-1)
                .build(),
        );
    }
    GridsStorage::new().with_vehicles_net(grid).build()
}

/// A vehicle standing alone with p 0 and p0 1 that has already stood `stood` ticks; returns its
/// head cell after 10 steps. Under VDR it never starts; under KKW it starts unless it stood at
/// least the threshold.
fn lone_vehicle_head(randomization: RelativeRandomization, stood: i32) -> i64 {
    let mut session = Session::new(straight_road(), None);
    let mut vehicle = Vehicle::new(1)
        .with_cell(5)
        .with_destination(40)
        .with_speed(0)
        .with_speed_limit(3)
        .with_behaviour(BehaviourType::Aggressive)
        .with_slowdown(0.0)
        .with_slow_to_start(1.0)
        .with_relative_randomization(randomization)
        .build();
    vehicle.stop_ticks = stood;
    session.add_vehicles(vec![vehicle]);
    for _ in 0..10 {
        session.step().expect("step failed");
    }
    session.get_vehicles().get(&1).map(|v| v.cell_id).unwrap_or(-1)
}

#[test]
fn test_stop_time_threshold_gates_slow_to_start() {
    assert_eq!(lone_vehicle_head(RelativeRandomization::VDR, 0), 5, "VDR: p0 1 keeps a stopped vehicle in place");
    assert_eq!(lone_vehicle_head(RelativeRandomization::KKW, 4), 32, "KKW: a stop shorter than the threshold starts with p 0 (speeds 1, 2, then 3 for 8 ticks)");
    assert_eq!(lone_vehicle_head(RelativeRandomization::KKW, 5), 5, "KKW: after the threshold p0 applies");
}

#[test]
fn test_stop_ticks_count_standing_ticks() {
    let mut session = Session::new(straight_road(), None);
    session.add_vehicles(vec![
        Vehicle::new(1).with_cell(10).with_destination(40).with_speed(0).with_behaviour(BehaviourType::Block).build(),
        Vehicle::new(2)
            .with_cell(9)
            .with_destination(40)
            .with_speed(0)
            .with_speed_limit(3)
            .with_behaviour(BehaviourType::Aggressive)
            .with_slowdown(0.0)
            .with_slow_to_start(0.0)
            .build(),
    ]);
    for _ in 0..3 {
        session.step().expect("step failed");
    }
    assert_eq!(session.get_vehicles().get(&2).expect("follower").stop_ticks, 3);
}

/// A follower with p 1 (always dawdles) behind a leader driving away at speed 3; returns the
/// follower speed after 3 steps.
fn follower_speed(randomization: RelativeRandomization) -> i32 {
    let mut session = Session::new(straight_road(), None);
    session.add_vehicles(vec![
        Vehicle::new(1)
            .with_cell(10)
            .with_destination(40)
            .with_speed(3)
            .with_speed_limit(3)
            .with_behaviour(BehaviourType::Aggressive)
            .with_slowdown(0.0)
            .with_slow_to_start(0.0)
            .build(),
        Vehicle::new(2)
            .with_cell(8)
            .with_destination(40)
            .with_speed(1)
            .with_speed_limit(3)
            .with_behaviour(BehaviourType::Aggressive)
            .with_slowdown(1.0)
            .with_slow_to_start(1.0)
            .with_relative_randomization(randomization)
            .build(),
    ]);
    for _ in 0..3 {
        session.step().expect("step failed");
    }
    session.get_vehicles().get(&2).expect("follower").speed
}

#[test]
fn test_no_slowdown_while_catching_up() {
    assert_eq!(follower_speed(RelativeRandomization::VDR), 1, "VDR: the dawdle cancels every acceleration");
    assert_eq!(follower_speed(RelativeRandomization::KKW), 3, "KKW: no random slowdown behind a faster leader");
    let faster_only = RelativeRandomization { decel_slower: 1, ..RelativeRandomization::KKW };
    assert_eq!(follower_speed(faster_only), 1, "decel_slower 1 restores the dawdle while catching up");
}
