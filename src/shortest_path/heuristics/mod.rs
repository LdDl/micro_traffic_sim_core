//! Travel-time edge costs, the heuristic interface, and geometric lower bounds.

mod heuristics;

pub use self::heuristics::{GeometricHeuristic, Heuristic, edge_time, heuristic};
