use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::behaviour::BehaviourType;
use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::cell::{Cell, CellID};
use micro_traffic_sim_core::grid::road_network::GridRoads;
use micro_traffic_sim_core::simulation::grids_storage::GridsStorage;
use micro_traffic_sim_core::simulation::session::Session;

/// Single lane road of cells 1..=20, speed limit 3.
fn straight_road() -> GridsStorage {
    let mut grid = GridRoads::new();
    for i in 1..=20 {
        let forward = if i < 20 { i + 1 } else { -1 };
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

/// A stopped Block leader on cell 10 and a deterministic follower (no dawdling, no slow to start)
/// starting from cell 2; returns the follower head after the queue has settled.
fn settled_follower_head(standstill_gap: i32) -> CellID {
    let mut session = Session::new(straight_road(), None);
    session.add_vehicles(vec![
        Vehicle::new(1)
            .with_cell(10)
            .with_destination(20)
            .with_speed(0)
            .with_behaviour(BehaviourType::Block)
            .build(),
        Vehicle::new(2)
            .with_cell(2)
            .with_destination(20)
            .with_speed(0)
            .with_speed_limit(3)
            .with_behaviour(BehaviourType::Aggressive)
            .with_slowdown(0.0)
            .with_slow_to_start(0.0)
            .with_standstill_gap(standstill_gap)
            .build(),
    ]);
    for _ in 0..15 {
        session.step().expect("step failed");
    }
    let vehicles = session.get_vehicles();
    assert_eq!(vehicles.get(&1).expect("leader").cell_id, 10, "the Block leader must not move");
    let follower = vehicles.get(&2).expect("follower");
    assert_eq!(follower.speed, 0, "the follower must have stopped");
    follower.cell_id
}

#[test]
fn test_zero_gap_stops_bumper_to_bumper() {
    assert_eq!(settled_follower_head(0), 9);
}

#[test]
fn test_gap_keeps_empty_cells_behind_stopped_leader() {
    assert_eq!(settled_follower_head(1), 8, "one empty cell between follower and leader");
    assert_eq!(settled_follower_head(2), 7, "two empty cells between follower and leader");
}
