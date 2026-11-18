use super::zone_support::{parallel_roads, traversed};
use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::conflict_zones::{ConflictEdge, ConflictWinnerType, ConflictZone};
use micro_traffic_sim_core::simulation::clearance::LongitudinalClearance;

#[test]
fn zones_sharing_a_target_cell_keep_both_constraints() {
    // 3(A)   4
    // 103(B) 104
    // 203(C) 204
    // Z1 links (3, 4) with (103, 104).
    // Z2 links (3, 4) with (203, 204).
    // Both zones constrain A's same edge; neither may overwrite the other.
    for policy in [
        LongitudinalClearance::Conservative,
        LongitudinalClearance::MovingLeader,
    ] {
        for first in [ConflictWinnerType::First, ConflictWinnerType::Second] {
            for second in [ConflictWinnerType::First, ConflictWinnerType::Second] {
                for order in [
                    [1, 2, 3],
                    [1, 3, 2],
                    [2, 1, 3],
                    [2, 3, 1],
                    [3, 1, 2],
                    [3, 2, 1],
                ] {
                    let mut session = parallel_roads(policy, &[0, 100, 200]);
                    session.add_conflict_zone(
                        ConflictZone::new(
                            1,
                            ConflictEdge {
                                source: 3,
                                target: 4,
                            },
                            ConflictEdge {
                                source: 103,
                                target: 104,
                            },
                        )
                        .with_winner_type(first.clone())
                        .build(),
                    );
                    session.add_conflict_zone(
                        ConflictZone::new(
                            2,
                            ConflictEdge {
                                source: 3,
                                target: 4,
                            },
                            ConflictEdge {
                                source: 203,
                                target: 204,
                            },
                        )
                        .with_winner_type(second.clone())
                        .build(),
                    );
                    session.add_vehicles(
                        order
                            .into_iter()
                            .map(|id| {
                                Vehicle::new(id)
                                    .with_cell(3 + 100 * (id as i64 - 1))
                                    .with_speed(3)
                                    .with_speed_limit(5)
                                    .with_slowdown(0.0)
                                    .with_slow_to_start(0.0)
                                    .with_destination(-1)
                                    .with_trip_destination(-1)
                                    .build()
                            })
                            .collect(),
                    );
                    session.step().unwrap();
                    let cars = session.get_vehicles();
                    assert!(
                        !(traversed(&cars[&1], 3, (3, 4)) && traversed(&cars[&2], 103, (103, 104))),
                        "zone one ignored: {policy:?}, priorities {first:?}/{second:?}, order {order:?}"
                    );
                    assert!(
                        !(traversed(&cars[&1], 3, (3, 4)) && traversed(&cars[&3], 203, (203, 204))),
                        "zone two ignored: {policy:?}, priorities {first:?}/{second:?}, order {order:?}"
                    );
                }
            }
        }
    }
}
