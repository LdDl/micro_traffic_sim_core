use super::*;
use crate::geom::new_point;
use crate::grid::cell::Cell;
use crate::grid::road_network::GridRoads;
use crate::shortest_path::router::{shortest_path, shortest_path_with_heuristic};

/// A straight 5-cell chain 1->2->3->4->5 plus a parallel lane 2->6->7->4 (a left
/// detour off cell 2 that rejoins at 4). Euclidean coords, speed_limit 1 so each
/// edge time equals its length.
fn build_test_grid() -> GridRoads {
    let mut grid = GridRoads::new();
    let mk = |id: CellID, x: f64, y: f64, f: CellID, l: CellID, r: CellID| {
        Cell::new(id)
            .with_point(new_point(x, y, None))
            .with_speed_limit(1)
            .with_forward_node(f)
            .with_left_node(l)
            .with_right_node(r)
            .build()
    };
    // main lane
    grid.add_cell(mk(1, 0.0, 0.0, 2, -1, -1));
    // left to 6
    grid.add_cell(mk(2, 1.0, 0.0, 3, 6, -1));
    grid.add_cell(mk(3, 2.0, 0.0, 4, -1, -1));
    grid.add_cell(mk(4, 3.0, 0.0, 5, -1, -1));
    grid.add_cell(mk(5, 4.0, 0.0, -1, -1, -1));
    // detour lane (longer): 2 -> 6 -> 7 -> 4
    grid.add_cell(mk(6, 1.0, 1.0, 7, -1, -1));
    grid.add_cell(mk(7, 2.0, 1.0, 4, -1, -1));
    grid
}

#[test]
fn test_reverse_adj() {
    let grid = build_test_grid();
    let rev = build_reverse_adj(&grid);
    // 4 has predecessors 3 (forward) and 7 (forward).
    let mut preds4 = rev.get(&4).cloned().unwrap_or_default();
    preds4.sort();
    assert_eq!(preds4, vec![3, 7]);
    // 6 has predecessor 2 (via left).
    assert_eq!(rev.get(&6).cloned().unwrap_or_default(), vec![2]);
    // 1 has no predecessors.
    assert!(rev.get(&1).is_none());
}

#[test]
fn test_dijkstra_from_and_to() {
    let grid = build_test_grid();
    // Forward distances from cell 1.
    let df = dijkstra_from(1, &grid);
    assert!((df[&1] - 0.0).abs() < 1e-9);
    // 1->2 length 1
    assert!((df[&2] - 1.0).abs() < 1e-9);
    // 1->2->3
    assert!((df[&3] - 2.0).abs() < 1e-9);
    // 1->2->3->4 (3) beats 1->2->6->7->4 (1+sqrt2+1+1)
    assert!((df[&4] - 3.0).abs() < 1e-9);
    assert!((df[&5] - 4.0).abs() < 1e-9);
    // 6 is reachable via left from 2: 1->2 (1) + 2->6 (1, vertical) = 2.
    assert!((df[&6] - 2.0).abs() < 1e-9);

    // Distances TO cell 5 (reverse). Only the main lane reaches 5.
    let rev = build_reverse_adj(&grid);
    let dt = dijkstra_to(5, &grid, &rev);
    assert!((dt[&5] - 0.0).abs() < 1e-9);
    // 4->5
    assert!((dt[&4] - 1.0).abs() < 1e-9);
    // 1->2->3->4->5
    assert!((dt[&1] - 4.0).abs() < 1e-9);
}

#[test]
fn test_landmark_build() {
    let grid = build_test_grid();
    let table = LandmarkTable::build(&grid, 2);
    assert_eq!(table.len(), 2);
    assert!(!table.is_empty());
    // Each landmark must have both a from- and a to-table.
    assert_eq!(table.dist_from.len(), 2);
    assert_eq!(table.dist_to.len(), 2);
    // Landmarks are distinct.
    assert_ne!(table.landmarks()[0], table.landmarks()[1]);
    // k = 0 yields an empty table; k larger than the grid is capped.
    assert!(LandmarkTable::build(&grid, 0).is_empty());
    assert!(LandmarkTable::build(&grid, 999).len() <= 7);
}

#[test]
fn test_landmark_heuristic_admissible() {
    let grid = build_test_grid();
    let table = LandmarkTable::build(&grid, 3);
    // For every (from, goal) pair the estimate must never exceed the true
    // shortest travel time (admissibility), and must be >= the geometric bound.
    let ids = [1, 2, 3, 4, 5, 6, 7];
    for &a in &ids {
        for &b in &ids {
            let (ca, cb) = (grid.get_cell(&a).unwrap(), grid.get_cell(&b).unwrap());
            let est = table.estimate(ca, cb);
            let geo = ca.distance_to(cb) / grid.get_max_speed();
            assert!(est + 1e-9 >= geo, "ALT must be >= geometric ({a}->{b})");
            if let Ok(path) = shortest_path(ca, cb, &grid, true, None) {
                assert!(
                    est <= path.cost() + 1e-9,
                    "heuristic {est} overestimates true cost {} for {a}->{b}",
                    path.cost()
                );
            }
        }
    }
}

#[test]
fn test_routing_with_alt_matches_geometric() {
    let grid = build_test_grid();
    let table = LandmarkTable::build(&grid, 3);
    let ids = [1, 2, 3, 4, 5, 6, 7];
    for &a in &ids {
        for &b in &ids {
            let (ca, cb) = (grid.get_cell(&a).unwrap(), grid.get_cell(&b).unwrap());
            let geo = shortest_path(ca, cb, &grid, true, None);
            let alt = shortest_path_with_heuristic(ca, cb, &grid, true, None, &table);
            match (geo, alt) {
                (Ok(pg), Ok(pa)) => {
                    // Same optimal cost and same vertex sequence.
                    assert!(
                        (pg.cost() - pa.cost()).abs() < 1e-9,
                        "cost mismatch {a}->{b}: geo {} vs alt {}",
                        pg.cost(),
                        pa.cost()
                    );
                    let vg: Vec<CellID> = pg.vertices().iter().map(|c| c.get_id()).collect();
                    let va: Vec<CellID> = pa.vertices().iter().map(|c| c.get_id()).collect();
                    assert_eq!(vg, va, "route mismatch {a}->{b}");
                }
                // both agree it is unreachable
                (Err(_), Err(_)) => {}
                (g, a2) => panic!("reachability mismatch {a}->{b}: {g:?} vs {a2:?}"),
            }
        }
    }
}

#[test]
fn test_empty_table_degrades_to_geometric() {
    let grid = build_test_grid();
    let table = LandmarkTable::build(&grid, 0);
    let (c1, c5) = (grid.get_cell(&1).unwrap(), grid.get_cell(&5).unwrap());
    // Empty table -> estimate equals the geometric bound exactly.
    let est = table.estimate(c1, c5);
    let geo = c1.distance_to(c5) / grid.get_max_speed();
    assert!((est - geo).abs() < 1e-9);
}

#[test]
fn test_active_query_admissible_and_bounded() {
    let grid = build_test_grid();
    let table = LandmarkTable::build(&grid, 4);
    let ids = [1, 2, 3, 4, 5, 6, 7];
    for &a in &ids {
        for &b in &ids {
            let (ca, cb) = (grid.get_cell(&a).unwrap(), grid.get_cell(&b).unwrap());
            let active = table.active_for(ca, cb, 2);
            // At most 2 active landmarks (the requested cap).
            assert!(active.active().len() <= 2);
            let est = active.estimate(ca, cb);
            // Admissible: never exceeds the true cost.
            if let Ok(path) = shortest_path(ca, cb, &grid, true, None) {
                assert!(
                    est <= path.cost() + 1e-9,
                    "active estimate overestimates {a}->{b}"
                );
            }
            // A subset is never tighter than the full table.
            assert!(est <= table.estimate(ca, cb) + 1e-9);
            // Still at least the geometric bound.
            assert!(est + 1e-9 >= ca.distance_to(cb) / grid.get_max_speed());
        }
    }
}

#[test]
fn test_routing_with_active_landmarks_matches_geometric() {
    let grid = build_test_grid();
    let table = LandmarkTable::build(&grid, 4);
    let ids = [1, 2, 3, 4, 5, 6, 7];
    for &a in &ids {
        for &b in &ids {
            let (ca, cb) = (grid.get_cell(&a).unwrap(), grid.get_cell(&b).unwrap());
            let active = table.active_for(ca, cb, 2);
            let geo = shortest_path(ca, cb, &grid, true, None);
            let alt = shortest_path_with_heuristic(ca, cb, &grid, true, None, &active);
            match (geo, alt) {
                (Ok(pg), Ok(pa)) => {
                    assert!(
                        (pg.cost() - pa.cost()).abs() < 1e-9,
                        "cost mismatch {a}->{b}"
                    );
                    let vg: Vec<CellID> = pg.vertices().iter().map(|c| c.get_id()).collect();
                    let va: Vec<CellID> = pa.vertices().iter().map(|c| c.get_id()).collect();
                    assert_eq!(vg, va, "route mismatch {a}->{b}");
                }
                (Err(_), Err(_)) => {}
                (g, a2) => panic!("reachability mismatch {a}->{b}: {g:?} vs {a2:?}"),
            }
        }
    }
}
