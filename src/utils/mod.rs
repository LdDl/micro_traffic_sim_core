//! # Utils Module
//! 
//! Utility functions and test data generators for traffic simulation development and testing.
//!
//! ## Key Components
//! 
//! ### Grid Generators
//! - [`generators::generate_one_lane_cells`] - Creates multi-lane road segments
//! 
//! ### Test Grids  
//! - [`test_grids::create_pretty_simple_grid`] - Complex intersection for testing
//! - [`test_grids::create_conflict_zones_grid`] - Two-path intersection
//! - [`test_grids::create_conflict_zones_multiple_grid`] - Multi-path intersection  
//! - [`test_grids::create_simple_cross_shape_grid`] - Simple 4-way intersection
//!
//! ### Testing Support
//! - [`rand::thread_rng`] - Deterministic RNG for reproducible tests
//!
//! ## Example
//!
//! ```rust
//! use micro_traffic_sim_core::utils::generators::generate_one_lane_cells;
//! 
//! // Generate a 2-lane road, 100 meters long, 10m per cell
//! let cells = generate_one_lane_cells(100.0, 10.0, 2);
//! assert_eq!(cells.len(), 20); // 2 lanes × 10 cells each
//! ```
pub mod generators;
pub mod test_grids;
pub mod rand;