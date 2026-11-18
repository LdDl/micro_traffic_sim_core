//! A* road-network routing, travel-time heuristics, landmark tables, and route representation.
//!
//! Routing supports directional connections, lane changes, cached-route reconnection, and bounded forward traversal.

pub mod heuristics;
pub mod landmarks;
pub mod path;
pub mod router;
