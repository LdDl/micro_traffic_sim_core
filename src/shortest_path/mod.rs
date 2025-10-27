//! # Shortest path Module
//! 
//! A* pathfinding algorithm implementation for cellular automata traffic simulation.
//!
//! This module provides efficient (kinda) shortest path calculation between cells in a road network,
//! supporting both straight movement and lane changes with configurable heuristics.
//!
//! ## Notice
//! 
//! Sometime I use term "vertex", "node", and "cell" which being used interchangeably in comments and documentation.
//! 
//! ## Components
//! 
//! ### Core Functions
//! - [`router::shortest_path`] - A* algorithm implementation with lane change support
//! - [`heuristics::heuristic`] - Distance-based heuristic function
//! 
//! ### Data Structures
//! - [`path::Path`] - Represents a complete path with vertices, maneuvers, and cost
//! - [`router::AStarNode`] - Internal A* node with g/f costs and parent tracking
//! - [`router::AStarError`] - Error types for pathfinding failures
//!
//! ## Algorithm Features
//!
//! ### A* Implementation
//! - **Optimal pathfinding**: Most of time guarantees shortest path when heuristic is admissible.
//! I am using distance-based heuristic currently which could be not that admissible.
//! - **Lane change support**: Handles forward, left, and right maneuvers. Penalizes lane changes.
//! - **Configurable maneuvers**: Can prohibit lane changes for straight-only routing.
//! - **Depth limiting**: Optional maximum search depth to prevent excessive computation
//! when it is acceptable to not find a path but just "guess" initial part of it.
//!
//! ### Heuristic Function
//! - **Distance-based**: Uses geometric distance between cell coordinates.
//! In future we can make some time-based heuristic considering speed limits, traffic jams,
//! traffic lights and etc.
//! - **Supports both coordinate systems**: Euclidean and WGS84 geographic distances.
//! 
//! ## Usage Examples
//!
//! ### Basic Pathfinding
//!
//! ```rust
//! use micro_traffic_sim_core::shortest_path::router::shortest_path;
//! use micro_traffic_sim_core::grid::{road_network::GridRoads, cell::Cell};
//! use micro_traffic_sim_core::geom::new_point;
//! 
//! // Create a simple grid
//! let mut grid = GridRoads::new();
//! let start_cell = Cell::new(1)
//!     .with_point(new_point(0.0, 0.0, None))
//!     .with_forward_node(2)
//!     .build();
//! let end_cell = Cell::new(2)
//!     .with_point(new_point(1.0, 0.0, None))
//!     .build();
//! 
//! grid.add_cell(start_cell.clone());
//! grid.add_cell(end_cell.clone());
//! 
//! // Find shortest path
//! let path_result = shortest_path(&start_cell, &end_cell, &grid, false, None);
//! 
//! match path_result {
//!     Ok(path) => {
//!         println!("Path found with cost: {}", path.cost());
//!         println!("Vertices: {:?}", path.vertices().len());
//!     },
//!     Err(e) => println!("Pathfinding failed: {}", e),
//! }
//! ```
//!
//! ### Lane Change Pathfinding
//!
//! ```rust
//! use micro_traffic_sim_core::shortest_path::router::shortest_path;
//! use micro_traffic_sim_core::grid::road_network::GridRoads;
//! 
//! # let grid = GridRoads::new();
//! # let start_cell = micro_traffic_sim_core::grid::cell::Cell::new(1).build();
//! # let end_cell = micro_traffic_sim_core::grid::cell::Cell::new(2).build();
//! // Enable lane changes during pathfinding
//! let path_result = shortest_path(
//!     &start_cell, 
//!     &end_cell, 
//!     &grid, 
//!     true,  // Allow maneuvers (lane changes)
//!     None   // No depth limit
//! );
//! ```
//!
//! ### Limited Depth Search
//!
//! ```rust
//! use micro_traffic_sim_core::shortest_path::router::shortest_path;
//! 
//! # let grid = micro_traffic_sim_core::grid::road_network::GridRoads::new();
//! # let start_cell = micro_traffic_sim_core::grid::cell::Cell::new(1).build();
//! # let end_cell = micro_traffic_sim_core::grid::cell::Cell::new(2).build();
//! // Limit search to prevent excessive computation
//! let path_result = shortest_path(
//!     &start_cell, 
//!     &end_cell, 
//!     &grid, 
//!     true, 
//!     Some(1000)  // Maximum 1000 vertices explored
//! );
//! ```
//!
//! ## Integration
//!
//! - **Grid System**: Works with [`GridRoads`](crate::grid::road_network::GridRoads) and [`Cell`](crate::grid::cell::Cell) as a road network graph
//! - **Geometry**: Uses distance calculations from the [`geom`](crate::geom) module
//! - **Maneuvers**: Returns [`LaneChangeType`](crate::maneuver::LaneChangeType) for each path segment
pub mod heuristics;
pub mod path;
pub mod router;
