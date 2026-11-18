use super::clearance_support::{self, car};
use micro_traffic_sim_core::simulation::clearance::LongitudinalClearance;
use micro_traffic_sim_core::simulation::session::Session;

fn road(end: i64) -> Session {
    let mut session = clearance_support::road(end);
    session.set_longitudinal_clearance(LongitudinalClearance::MovingLeader);
    session
}

fn grant(speed: i32, leader: i32, gap: i32, reserve: i32, sync: bool) -> i32 {
    let desired = if sync && speed > 0 && f64::from(gap) <= (2.55 * f64::from(speed)).floor() {
        speed + (leader - speed).signum()
    } else {
        speed + 1
    };
    let desired = desired.clamp(0, 5);
    // Baseline path scanning can encounter the stopped body before the requested
    // destination. Synchronization already clips to the physical gap before that scan.
    if !sync && leader == 0 && reserve > 0 && gap <= reserve && gap < desired {
        return 0;
    }
    desired.min(gap)
}

#[test]
fn three_vehicle_scalar_oracle_checks_both_frozen_lower_bounds() {
    // F@10   [gap]   L@(11 + gap)   [leader_gap]   A@(12 + gap + leader_gap)
    // Brackets count empty cells. A constrains L's guaranteed movement,
    // which constrains F's spacing credit without reusing L's initial body.
    let mut checked = 0;
    for sync in [false, true] {
        for follower in 0..=5 {
            for leader in 0..=5 {
                for ahead in 0..=5 {
                    for gap in 0..=6 {
                        for leader_gap in 0..=6 {
                            for reserve in 0..=2 {
                                for leader_reserve in 0..=2 {
                                    let mut session = road(40);
                                    session.add_vehicles(vec![
                                        car(1, 10, follower, reserve, sync, 40),
                                        car(
                                            2,
                                            11 + i64::from(gap),
                                            leader,
                                            leader_reserve,
                                            sync,
                                            40,
                                        ),
                                        car(
                                            3,
                                            12 + i64::from(gap + leader_gap),
                                            ahead,
                                            0,
                                            sync,
                                            40,
                                        ),
                                    ]);
                                    let u = grant(follower, leader, gap, reserve, sync);
                                    let w = grant(leader, ahead, leader_gap, leader_reserve, sync);
                                    let z = (ahead + 1).min(5);
                                    let lower = w.min((leader_gap - leader_reserve).max(0));
                                    let expected = u.min(
                                        (gap - reserve + if leader > 0 { lower } else { 0 }).max(0),
                                    );
                                    let leader_expected = w.min(
                                        (leader_gap - leader_reserve
                                            + if ahead > 0 { z } else { 0 })
                                        .max(0),
                                    );
                                    session.step().unwrap();
                                    let cars = session.get_vehicles();
                                    assert_eq!(
                                        cars[&1].speed, expected,
                                        "sync {sync}, speeds {follower}/{leader}/{ahead}, gaps {gap}/{leader_gap}, reserves {reserve}/{leader_reserve}"
                                    );
                                    assert_eq!(
                                        cars[&2].speed, leader_expected,
                                        "leader: sync {sync}, speeds {follower}/{leader}/{ahead}, gaps {gap}/{leader_gap}, reserves {reserve}/{leader_reserve}"
                                    );
                                    assert_eq!(cars[&3].speed, z);
                                    assert!(
                                        cars[&2].cell_id - cars[&1].cell_id - 1
                                            >= i64::from(gap.min(reserve))
                                    );
                                    assert!(
                                        cars[&3].cell_id - cars[&2].cell_id - 1
                                            >= i64::from(leader_gap.min(leader_reserve))
                                    );
                                    checked += 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    assert_eq!(checked, 190512);
}

#[test]
fn distant_stop_cannot_change_rear_motion_through_a_gap_cascade() {
    for count in [4, 16, 64, 256] {
        for gap in [0, 1] {
            let mut rear = Vec::new();
            for terminal_stopped in [false, true] {
                let end = 2 * count + 40;
                let mut session = road(end);
                let vehicles = (0..count)
                    .map(|i| {
                        let stopped = terminal_stopped && i + 1 == count;
                        let mut vehicle = car(
                            i as u64 + 1,
                            10 + 2 * i,
                            if stopped { 0 } else { 1 },
                            gap,
                            true,
                            end,
                        );
                        vehicle.slow_to_start_factor_p0 = 1.0;
                        vehicle.stop_ticks = if stopped { 8 } else { 0 };
                        vehicle
                    })
                    .collect();
                session.add_vehicles(vehicles);
                session.step().unwrap();
                let cars = session.get_vehicles();
                rear.push((cars[&1].cell_id, cars[&1].speed));
                for id in 1..count as u64 {
                    assert!(cars[&(id + 1)].cell_id - cars[&id].cell_id - 1 >= i64::from(gap));
                }
            }
            assert_eq!(rear[0], rear[1], "count {count}, gap {gap}");
            assert_eq!(rear[0], if gap == 0 { (11, 1) } else { (10, 0) });
        }
    }
}
