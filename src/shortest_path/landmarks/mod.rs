//! Landmark distance tables and ALT travel-time heuristics.

mod landmarks;

pub use self::landmarks::{ActiveQuery, LandmarkTable, dijkstra_from, dijkstra_to};
