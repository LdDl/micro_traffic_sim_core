use micro_traffic_sim_core::agents::Vehicle;
use micro_traffic_sim_core::geom::new_point;
use micro_traffic_sim_core::grid::cell::{Cell, CellState};
use micro_traffic_sim_core::grid::road_network::GridRoads;
use micro_traffic_sim_core::intentions::{find_intention, process_path_for_vehicle};
use micro_traffic_sim_core::maneuver::LaneChangeType;
use micro_traffic_sim_core::shortest_path::path::Path;
use micro_traffic_sim_core::verbose::LocalLogger;
use std::collections::HashMap;

fn measured(
    limits: &[i32],
    desired: i32,
    obstacle: usize,
    red: usize,
    turn: usize,
    own: bool,
) -> usize {
    let mut cells = vec![Cell::new(0).with_speed_limit(5).build()];
    for (index, limit) in limits.iter().enumerate() {
        let mut cell = Cell::new((index + 1) as i64)
            .with_speed_limit(*limit)
            .build();
        if index + 1 == red {
            cell.set_state(CellState::Banned);
        }
        cells.push(cell);
    }
    let maneuvers = (1..=limits.len())
        .map(|index| {
            if index == turn {
                LaneChangeType::ChangeLeft
            } else {
                LaneChangeType::NoChange
            }
        })
        .collect();
    let mut path = Path::new(cells.iter().collect(), maneuvers, 1.0);
    let occupancy = if obstacle == 0 {
        HashMap::new()
    } else {
        HashMap::from([(obstacle as i64, if own { 1 } else { 2 })])
    };
    process_path_for_vehicle(&mut path, desired, 100, &occupancy, Some(1))
        .trimmed_path
        .len()
}

#[test]
fn entering_a_lower_positive_limit_uses_its_legal_speed() {
    assert_eq!(measured(&[3, 3, 3, 3, 3], 5, 0, 0, 0, false), 3);
    assert_eq!(measured(&[5, 3, 3, 3, 3], 5, 0, 0, 0, false), 3);
    assert_eq!(measured(&[5, 5, 5, 1, 1], 5, 0, 0, 0, false), 3);
}

#[test]
fn movement_matches_an_independent_legal_prefix_search() {
    for encoded in 0..243 {
        let mut value = encoded;
        let mut limits = Vec::new();
        for _ in 0..5 {
            limits.push([0, 2, 5][value % 3]);
            value /= 3;
        }
        for desired in 0..=5 {
            for obstacle in [0, 1, 3] {
                for red in [0, 2, 4] {
                    for turn in [0, 1, 3] {
                        for own in [false, true] {
                            // A legal displacement obeys every traversed cell and physical barrier.
                            let expected = (0..=desired as usize)
                                .filter(|distance| {
                                    (1..=*distance).all(|step| {
                                        *distance <= limits[step - 1] as usize
                                            && (step != obstacle || own)
                                            && step != red
                                            && step != turn
                                    })
                                })
                                .max()
                                .unwrap();
                            assert_eq!(
                                measured(&limits, desired, obstacle, red, turn, own),
                                expected,
                                "limits={limits:?}, desired={desired}, obstacle={obstacle}, red={red}, turn={turn}, own={own}"
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn intention_obeys_the_longest_legal_prefix_at_a_speed_restriction() {
    for distance in 1..=6 {
        for limit in 0..=5 {
            let mut road = GridRoads::new();
            for cell in 1..=20 {
                road.add_cell(
                    Cell::new(cell)
                        .with_point(new_point(cell as f64, 0.0, None))
                        .with_forward_node(if cell < 20 { cell + 1 } else { -1 })
                        .with_left_node(-1)
                        .with_right_node(-1)
                        .with_speed_limit(if cell > distance { limit } else { 5 })
                        .build(),
                );
            }
            let vehicle = Vehicle::new(1)
                .with_cell(1)
                .with_destination(-1)
                .with_speed(5)
                .with_speed_limit(5)
                .with_slowdown(0.0)
                .with_slow_to_start(0.0)
                .build();
            let occupancy = HashMap::from([(1, 1)]);
            let speeds = HashMap::from([(1, 5)]);
            let intention =
                find_intention(&road, &occupancy, &speeds, &vehicle, &LocalLogger::none())
                    .expect("intention");
            // Enumerate legal displacements independently of the path scan.
            let expected = (0..=5)
                .filter(|speed| {
                    (1..=*speed)
                        .all(|step| *speed <= if step as i64 >= distance { limit } else { 5 })
                })
                .max()
                .unwrap();
            assert_eq!(
                intention.intention_speed, expected,
                "distance={distance}, limit={limit}"
            );
            assert_eq!(intention.intention_cell_id, 1 + expected as i64);
        }
    }
}
