use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::conflict_zones::{ConflictEdge, ConflictWinnerType, ConflictZone};
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::simulation::{clearance::LongitudinalClearance, session::Session};

fn crossing(
    policy: LongitudinalClearance,
    first_start: i64,
    second_start: i64,
    reverse: bool,
) -> Session {
    let mut session = Session::default(None);
    session.set_longitudinal_clearance(policy);
    session.add_cells(
        (0..20)
            .chain(100..120)
            .map(|id| {
                Cell::new(id)
                    .with_forward_node(if id == 19 || id == 119 { -1 } else { id + 1 })
                    .with_left_node(-1)
                    .with_right_node(-1)
                    .with_speed_limit(5)
                    .build()
            })
            .collect(),
    );
    session.add_conflict_zone(
        ConflictZone::new(
            1,
            ConflictEdge {
                source: 4,
                target: 5,
            },
            ConflictEdge {
                source: 104,
                target: 105,
            },
        )
        .with_winner_type(ConflictWinnerType::First)
        .build(),
    );
    let mut cars = vec![
        Vehicle::new(1)
            .with_cell(first_start)
            .with_speed(3)
            .with_speed_limit(5)
            .with_slowdown(0.0)
            .with_slow_to_start(0.0)
            .with_destination(-1)
            .with_trip_destination(-1)
            .build(),
        Vehicle::new(2)
            .with_cell(second_start)
            .with_speed(3)
            .with_speed_limit(5)
            .with_slowdown(0.0)
            .with_slow_to_start(0.0)
            .with_destination(-1)
            .with_trip_destination(-1)
            .build(),
    ];
    if reverse {
        cars.reverse();
    }
    session.add_vehicles(cars);
    session
}

#[test]
fn multi_cell_approaches_obey_the_conflicting_edge_priority() {
    for policy in [
        LongitudinalClearance::Conservative,
        LongitudinalClearance::MovingLeader,
    ] {
        for reverse in [false, true] {
            for first_start in 1..=4 {
                for second_start in 101..=104 {
                    let mut session = crossing(policy, first_start, second_start, reverse);
                    session.step().unwrap();
                    let cars = session.get_vehicles();
                    assert!(cars[&1].cell_id >= 5, "priority vehicle must cross");
                    assert!(
                        cars[&2].cell_id < 105,
                        "both conflict edges crossed from {first_start}/{second_start}"
                    );
                }
            }
        }
    }
}

#[test]
fn a_vehicle_held_by_a_crossing_conflict_records_a_stopped_tick() {
    let mut session = crossing(LongitudinalClearance::MovingLeader, 4, 104, false);
    session.step().unwrap();
    let yielding = &session.get_vehicles()[&2];
    assert_eq!(yielding.cell_id, 104);
    assert_eq!(yielding.speed, 0);
    assert_eq!(yielding.stop_ticks, 1);
}

#[test]
fn entering_a_zone_target_through_another_edge_does_not_claim_priority() {
    let mut session = crossing(LongitudinalClearance::MovingLeader, 3, 104, false);
    session.add_cells(vec![
        Cell::new(3)
            .with_forward_node(5)
            .with_left_node(-1)
            .with_right_node(-1)
            .with_speed_limit(5)
            .build(),
    ]);
    session.step().unwrap();
    assert!(session.get_vehicles()[&1].cell_id >= 5);
    assert!(session.get_vehicles()[&2].cell_id >= 105);
}
