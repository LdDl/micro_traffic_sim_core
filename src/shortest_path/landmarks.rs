//! ALT (A*, Landmarks, Triangle inequality) heuristic.
//!
//! A landmark is a fixed cell for which the shortest travel time to and from every
//! other cell is precomputed once. The triangle inequality then yields a much tighter
//! admissible lower bound than the straight-line geometric heuristic, so A* explores
//! far fewer cells on large networks.
//!
//! Landmark distances are computed on **free-flow** travel time. Because real travel
//! time is never less than free-flow time, the bound stays admissible even when the
//! routing edge costs reflect congestion - so the table is built once and never needs
//! rebuilding.
//!
//! This module depends only on [`Cell`]/[`GridRoads`] and the [`Heuristic`] trait, so
//! it can be lifted into a standalone crate (e.g. `micro_traffic_sim_alt`) unchanged.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use crate::grid::cell::{Cell, CellID};
use crate::grid::road_network::GridRoads;
use crate::shortest_path::heuristics::{edge_time, Heuristic};

/// Predecessor adjacency: for each cell, the cells that have an out-edge into it.
type ReverseAdj = HashMap<CellID, Vec<CellID>>;

/// Min-heap entry ordered by ascending distance (ties broken by cell id for
/// determinism). `BinaryHeap` is a max-heap, so the ordering is reversed on `dist`.
struct DijkstraEntry {
    dist: f64,
    cell: CellID,
}
impl PartialEq for DijkstraEntry {
    fn eq(&self, other: &Self) -> bool {
        self.dist == other.dist && self.cell == other.cell
    }
}
impl Eq for DijkstraEntry {}
impl Ord for DijkstraEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .dist
            .partial_cmp(&self.dist)
            .unwrap_or(Ordering::Equal)
            .then_with(|| self.cell.cmp(&other.cell))
    }
}
impl PartialOrd for DijkstraEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Builds the predecessor adjacency (incoming edges) of the network.
fn build_reverse_adj(net: &GridRoads) -> ReverseAdj {
    let mut rev: ReverseAdj = HashMap::new();
    for (id, c) in net.iter() {
        for nb in [c.get_forward_id(), c.get_left_id(), c.get_right_id()] {
            if nb >= 0 {
                rev.entry(nb).or_default().push(*id);
            }
        }
    }
    rev
}

/// Single-source shortest travel time from `source` to every reachable cell, over
/// the forward graph (out-edges), using free-flow edge time as the cost.
pub fn dijkstra_from(source: CellID, net: &GridRoads) -> HashMap<CellID, f64> {
    let mut dist: HashMap<CellID, f64> = HashMap::new();
    let mut heap = BinaryHeap::new();
    dist.insert(source, 0.0);
    heap.push(DijkstraEntry { dist: 0.0, cell: source });
    while let Some(DijkstraEntry { dist: d, cell }) = heap.pop() {
        if d > *dist.get(&cell).unwrap_or(&f64::INFINITY) {
            continue; // stale heap entry
        }
        let c = match net.get_cell(&cell) {
            Some(c) => c,
            None => continue,
        };
        for nb in [c.get_forward_id(), c.get_left_id(), c.get_right_id()] {
            if nb < 0 {
                continue;
            }
            if let Some(ncell) = net.get_cell(&nb) {
                let nd = d + edge_time(c, ncell);
                if nd < *dist.get(&nb).unwrap_or(&f64::INFINITY) {
                    dist.insert(nb, nd);
                    heap.push(DijkstraEntry { dist: nd, cell: nb });
                }
            }
        }
    }
    dist
}

/// Single-source shortest travel time from every cell *to* `source`, over the reverse
/// graph (in-edges). The cost of traversing a predecessor edge `u -> v` is the
/// forward edge time `edge_time(u, v)`.
pub fn dijkstra_to(source: CellID, net: &GridRoads, rev: &ReverseAdj) -> HashMap<CellID, f64> {
    let mut dist: HashMap<CellID, f64> = HashMap::new();
    let mut heap = BinaryHeap::new();
    dist.insert(source, 0.0);
    heap.push(DijkstraEntry { dist: 0.0, cell: source });
    while let Some(DijkstraEntry { dist: d, cell }) = heap.pop() {
        if d > *dist.get(&cell).unwrap_or(&f64::INFINITY) {
            continue;
        }
        let cur = match net.get_cell(&cell) {
            Some(c) => c,
            None => continue,
        };
        if let Some(preds) = rev.get(&cell) {
            for &u in preds {
                if let Some(ucell) = net.get_cell(&u) {
                    let nd = d + edge_time(ucell, cur);
                    if nd < *dist.get(&u).unwrap_or(&f64::INFINITY) {
                        dist.insert(u, nd);
                        heap.push(DijkstraEntry { dist: nd, cell: u });
                    }
                }
            }
        }
    }
    dist
}

/// Precomputed landmark distance tables and the ALT heuristic over them.
pub struct LandmarkTable {
    /// The chosen landmark cell ids.
    landmarks: Vec<CellID>,
    /// `dist_from[i][v]` = shortest free-flow time from landmark `i` to cell `v`.
    dist_from: Vec<HashMap<CellID, f64>>,
    /// `dist_to[i][v]` = shortest free-flow time from cell `v` to landmark `i`.
    dist_to: Vec<HashMap<CellID, f64>>,
    /// Network max speed, for combining with the geometric bound (ALT is never worse
    /// than geometric).
    max_speed: f64,
}

impl LandmarkTable {
    /// Builds a table with up to `k` landmarks chosen by farthest-first selection
    /// (each new landmark is the cell farthest, in free-flow time, from the ones
    /// chosen so far). Returns an empty table (heuristic falls back to geometric) for
    /// `k == 0` or an empty network. Cost: `2k` single-source Dijkstra runs.
    pub fn build(net: &GridRoads, k: usize) -> Self {
        let max_speed = net.get_max_speed();
        let cell_ids: Vec<CellID> = net.iter().map(|(id, _)| *id).collect();
        if k == 0 || cell_ids.is_empty() {
            return LandmarkTable {
                landmarks: Vec::new(),
                dist_from: Vec::new(),
                dist_to: Vec::new(),
                max_speed,
            };
        }
        let rev = build_reverse_adj(net);

        // Seed: the cell farthest (forward) from an arbitrary start gives a good corner.
        let seed = *cell_ids.iter().min().unwrap();
        let seed_dist = dijkstra_from(seed, net);
        let first = seed_dist
            .iter()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(Ordering::Equal))
            .map(|(id, _)| *id)
            .unwrap_or(seed);

        let mut landmarks = vec![first];
        let mut dist_from = vec![dijkstra_from(first, net)];

        while landmarks.len() < k && landmarks.len() < cell_ids.len() {
            // Next landmark = cell maximizing its minimum distance from chosen landmarks.
            let mut best: Option<CellID> = None;
            let mut best_score = -1.0_f64;
            for &v in &cell_ids {
                if landmarks.contains(&v) {
                    continue;
                }
                // Reachable-from-all score; unreachable from a landmark -> 0 (skip far-but-disconnected).
                let mut score = f64::INFINITY;
                for df in &dist_from {
                    let d = df.get(&v).copied().unwrap_or(0.0);
                    if d < score {
                        score = d;
                    }
                }
                if score > best_score {
                    best_score = score;
                    best = Some(v);
                }
            }
            match best {
                Some(v) => {
                    dist_from.push(dijkstra_from(v, net));
                    landmarks.push(v);
                }
                None => break,
            }
        }

        let dist_to = landmarks.iter().map(|&l| dijkstra_to(l, net, &rev)).collect();

        LandmarkTable {
            landmarks,
            dist_from,
            dist_to,
            max_speed,
        }
    }

    /// Number of landmarks in the table.
    pub fn len(&self) -> usize {
        self.landmarks.len()
    }

    /// Whether the table has no landmarks (heuristic degrades to geometric).
    pub fn is_empty(&self) -> bool {
        self.landmarks.is_empty()
    }

    /// The landmark cell ids.
    pub fn landmarks(&self) -> &[CellID] {
        &self.landmarks
    }
}

impl Heuristic for LandmarkTable {
    fn estimate(&self, from: &Cell, goal: &Cell) -> f64 {
        let v = from.get_id();
        let g = goal.get_id();
        // Always at least the geometric bound (also admissible); ALT only tightens it.
        let mut best = from.distance_to(goal) / self.max_speed;
        for i in 0..self.landmarks.len() {
            // Bound via distances TO landmark i:  d(v,goal) >= d(v,L) - d(goal,L).
            if let (Some(&dvl), Some(&dgl)) = (self.dist_to[i].get(&v), self.dist_to[i].get(&g)) {
                let lb = dvl - dgl;
                if lb > best {
                    best = lb;
                }
            }
            // Bound via distances FROM landmark i:  d(v,goal) >= d(L,goal) - d(L,v).
            if let (Some(&dlg), Some(&dlv)) = (self.dist_from[i].get(&g), self.dist_from[i].get(&v)) {
                let lb = dlg - dlv;
                if lb > best {
                    best = lb;
                }
            }
        }
        best.max(0.0)
    }
}

#[cfg(test)]
mod tests {
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
        grid.add_cell(mk(2, 1.0, 0.0, 3, 6, -1)); // left to 6
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
        assert!((df[&2] - 1.0).abs() < 1e-9); // 1->2 length 1
        assert!((df[&3] - 2.0).abs() < 1e-9); // 1->2->3
        assert!((df[&4] - 3.0).abs() < 1e-9); // 1->2->3->4 (3) beats 1->2->6->7->4 (1+sqrt2+1+1)
        assert!((df[&5] - 4.0).abs() < 1e-9);
        // 6 is reachable via left from 2: 1->2 (1) + 2->6 (1, vertical) = 2.
        assert!((df[&6] - 2.0).abs() < 1e-9);

        // Distances TO cell 5 (reverse). Only the main lane reaches 5.
        let rev = build_reverse_adj(&grid);
        let dt = dijkstra_to(5, &grid, &rev);
        assert!((dt[&5] - 0.0).abs() < 1e-9);
        assert!((dt[&4] - 1.0).abs() < 1e-9); // 4->5
        assert!((dt[&1] - 4.0).abs() < 1e-9); // 1->2->3->4->5
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
                    (Err(_), Err(_)) => {} // both agree it is unreachable
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
}
