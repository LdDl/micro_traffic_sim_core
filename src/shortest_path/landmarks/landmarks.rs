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
            // stale heap entry
            continue;
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
            let mut best_score = -1.0;
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

    /// The triangle-inequality lower bound on `d(v, g)` from landmark `i` alone
    /// (the larger of the to-landmark and from-landmark bounds; `0` if missing).
    fn landmark_bound(&self, i: usize, v: CellID, g: CellID) -> f64 {
        let mut b = 0.0;
        // Bound via distances TO landmark i:  d(v,goal) >= d(v,L) - d(goal,L).
        if let (Some(&dvl), Some(&dgl)) = (self.dist_to[i].get(&v), self.dist_to[i].get(&g)) {
            let lb = dvl - dgl;
            if lb > b {
                b = lb;
            }
        }
        // Bound via distances FROM landmark i:  d(v,goal) >= d(L,goal) - d(L,v).
        if let (Some(&dlg), Some(&dlv)) = (self.dist_from[i].get(&g), self.dist_from[i].get(&v)) {
            let lb = dlg - dlv;
            if lb > b {
                b = lb;
            }
        }
        b
    }

    /// Selects the `num_active` landmarks that give the tightest bound for the query
    /// `(start, goal)` and returns a lightweight per-query heuristic that consults only
    /// those - the standard "active landmarks" optimisation. Per-node estimates then
    /// cost `O(num_active)` table lookups instead of `O(all landmarks)`. Any subset of
    /// landmarks stays admissible, so routes are unchanged. Build this once per A*
    /// search (it borrows the table; no copying of distance data).
    pub fn active_for<'a>(&'a self, start: &Cell, goal: &Cell, num_active: usize) -> ActiveQuery<'a> {
        let s = start.get_id();
        let g = goal.get_id();
        let mut scored: Vec<(f64, usize)> = (0..self.landmarks.len())
            .map(|i| (self.landmark_bound(i, s, g), i))
            .collect();
        // Highest bound first; keep the top `num_active`.
        scored.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(Ordering::Equal));
        let active = scored.into_iter().take(num_active).map(|(_, i)| i).collect();
        ActiveQuery { table: self, active }
    }
}

impl Heuristic for LandmarkTable {
    fn estimate(&self, from: &Cell, goal: &Cell) -> f64 {
        let v = from.get_id();
        let g = goal.get_id();
        // Always at least the geometric bound (also admissible); ALT only tightens it.
        let mut best = from.distance_to(goal) / self.max_speed;
        for i in 0..self.landmarks.len() {
            let lb = self.landmark_bound(i, v, g);
            if lb > best {
                best = lb;
            }
        }
        best.max(0.0)
    }
}

/// A per-query view of a [`LandmarkTable`] that consults only the landmarks selected
/// as "active" for one `(start, goal)` pair (see [`LandmarkTable::active_for`]).
pub struct ActiveQuery<'a> {
    table: &'a LandmarkTable,
    active: Vec<usize>,
}

impl ActiveQuery<'_> {
    /// Indices of the active landmarks (into the parent table's landmark list).
    pub fn active(&self) -> &[usize] {
        &self.active
    }
}

impl Heuristic for ActiveQuery<'_> {
    fn estimate(&self, from: &Cell, goal: &Cell) -> f64 {
        let v = from.get_id();
        let g = goal.get_id();
        let mut best = from.distance_to(goal) / self.table.max_speed;
        for &i in &self.active {
            let lb = self.table.landmark_bound(i, v, g);
            if lb > best {
                best = lb;
            }
        }
        best.max(0.0)
    }
}

#[cfg(test)]
#[path = "tests.rs"]
mod tests;
