use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::conflict_zones::{ConflictEdge, ConflictWinnerType, ConflictZone};
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::simulation::{session::Session, clearance::LongitudinalClearance};

fn traversed(vehicle: &Vehicle, start: i64, edge: (i64,i64)) -> bool {
    let mut previous=start;
    vehicle.intention.intermediate_cells.iter().copied()
        .chain(std::iter::once(vehicle.cell_id)).any(|cell| {
            let matched=(previous,cell)==edge;previous=cell;matched
        })
}

#[test]
fn every_conflict_zone_on_the_same_tick_path_is_enforced() {
    for policy in [LongitudinalClearance::Conservative,LongitudinalClearance::MovingLeader] {
        for first in [ConflictWinnerType::First,ConflictWinnerType::Second] {
            for second in [ConflictWinnerType::First,ConflictWinnerType::Second] {
                for order in [[1,2,3],[1,3,2],[2,1,3],[2,3,1],[3,1,2],[3,2,1]] {
                    let mut session=Session::default(None);session.set_longitudinal_clearance(policy);
                    session.add_cells((0..20).chain(100..120).chain(200..220).map(|id|Cell::new(id)
                        .with_forward_node(if id%100==19 {-1} else {id+1})
                        .with_left_node(-1).with_right_node(-1).with_speed_limit(5).build()).collect());
                    session.add_conflict_zone(ConflictZone::new(1,ConflictEdge{source:3,target:4},ConflictEdge{source:103,target:104}).with_winner_type(first.clone()).build());
                    session.add_conflict_zone(ConflictZone::new(2,ConflictEdge{source:4,target:5},ConflictEdge{source:203,target:204}).with_winner_type(second.clone()).build());
                    session.add_vehicles(order.into_iter().map(|id|Vehicle::new(id)
                        .with_cell(3+100*(id as i64-1)).with_speed(3).with_speed_limit(5)
                        .with_slowdown(0.0).with_slow_to_start(0.0).with_destination(-1).with_trip_destination(-1).build()).collect());
                    session.step().unwrap();let cars=session.get_vehicles();
                    assert!(!(traversed(&cars[&1],3,(3,4)) && traversed(&cars[&2],103,(103,104))),"zone one ignored: {policy:?}, priorities {first:?}/{second:?}, order {order:?}");
                    assert!(!(traversed(&cars[&1],3,(4,5)) && traversed(&cars[&3],203,(203,204))),"zone two ignored: {policy:?}, priorities {first:?}/{second:?}, order {order:?}");
                }
            }
        }
    }
}
