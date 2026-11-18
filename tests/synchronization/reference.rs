use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::cell::Cell;
use micro_traffic_sim_core::grid::road_network::GridRoads;
use micro_traffic_sim_core::intentions::find_intention;
use micro_traffic_sim_core::verbose::LocalLogger;
use std::collections::HashMap;

#[test]
fn kkw_noise_order_matches_straight_road_reference() {
    // 2(F)   [gap empty cells]   (3 + gap)(L)
    // The scalar expectation keeps acceleration, synchronization, safety and
    // noise in their specified order, independently of the path traversal.
    let mut net = GridRoads::new();
    for cell in 1..=40 {
        net.add_cell(
            Cell::new(cell)
                .with_point(new_point(cell as f64, 0.0, None))
                .with_forward_node(if cell < 40 { cell + 1 } else { -1 })
                .with_left_node(-1)
                .with_right_node(-1)
                .with_speed_limit(8)
                .build(),
        );
    }
    let logger = LocalLogger::none();
    for speed in 0..=8 {
        for gap in 0..=15 {
            for leader in 0..=8 {
                for k in [1.5, 2.55] {
                    for noise in [-1, 0, 1] {
                        let pb = if noise == -1 { 1.0 } else { 0.0 };
                        let pa = if noise == 1 { 1.0 } else { 0.0 };
                        let mut vehicle = Vehicle::new(1)
                            .with_cell(2)
                            .with_destination(-1)
                            .with_speed(speed)
                            .with_speed_limit(8)
                            .with_standstill_gap(0)
                            .with_slow_to_start(pb)
                            .build();
                        vehicle.synchronization.time_gap = k;
                        vehicle.synchronization.noise = pb;
                        vehicle.synchronization.over_acceleration = pa;
                        vehicle.synchronization.over_acceleration_low = pa;
                        vehicle.synchronization.pinch_speed = 4;
                        let candidate = if speed > 0 && gap <= (k * speed as f64).floor() as i32 {
                            speed + (leader - speed).signum()
                        } else {
                            speed + 1
                        };
                        let preliminary = candidate.clamp(0, 8).min(gap);
                        let expected = (preliminary + noise).max(0).min(speed + 1).min(8).min(gap);
                        let occupancy = HashMap::from([(2, 1), (3 + gap as i64, 2)]);
                        let speeds = HashMap::from([(2, speed), (3 + gap as i64, leader)]);
                        let actual =
                            find_intention(&net, &occupancy, &speeds, &vehicle, &logger).unwrap();
                        assert_eq!(
                            actual.intention_speed, expected,
                            "speed {speed}, gap {gap}, leader {leader}, k {k}, noise {noise}"
                        );
                        assert_eq!(actual.intention_cell_id, 2 + expected as i64);
                        if gap == 0 && noise == 0 {
                            assert!(
                                actual.should_stop,
                                "a physically blocked vehicle must retain alternative-route handling"
                            );
                        }
                    }
                }
            }
        }
    }
}
