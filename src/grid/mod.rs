//! # Grid Module
//! 
//! This module provides the core cellular automata grid system for traffic simulation,
//! implementing a network of interconnected cells representing road segments.
//!
//! ## Key Components
//! 
//! ### Core Structures
//! - [`cell::Cell`] - Individual road segment with position, connections, and properties
//! - [`road_network::GridRoads`] - Container managing the complete road network
//! - [`cell::CellBuilder`] - Fluent builder pattern for constructing cells
//! - [`cell::CellState`] - Runtime state of cells (free, banned)
//! - [`zones::ZoneType`] - Semantic classification of cell purposes
//!
//! ### Cell Network System
//! 
//! Cells are connected through directional relationships:
//! - **Forward connections** - Primary traffic flow direction
//! - **Left/Right connections** - Lane changing opportunities
//! - **Coordinate-based positioning** - Spatial relationships
//!
//! ## Architecture
//!
//! ### Cellular Automata Design
//! 
//! The grid follows cellular automata principles where:
//! - Each cell represents a discrete road segment
//! - Vehicles move between cells according to rules
//! - Time progresses in discrete steps
//! - Local interactions create emergent traffic behavior
//!
//! ### Zone Types
//!
//! Cells are classified by their role in traffic flow:
//! 
//! ```rust
//! use micro_traffic_sim_core::grid::zones::ZoneType;
//! 
//! // Core traffic zones
//! let zt = ZoneType::Birth;         // Vehicle spawn points
//! let zt = ZoneType::Death;         // Vehicle despawn points
//! let zt = ZoneType::Common;        // Regular road segments
//! let zt = ZoneType::Coordination;  // Junction/intersection cells
//! 
//! // Specialized zones (future use)
//! let zt = ZoneType::LaneForBus;    // Dedicated bus lanes
//! let zt = ZoneType::Transit;       // Public transport stops
//! let zt = ZoneType::Crosswalk;     // Pedestrian crossings
//! let zt = ZoneType::Isolated;      // Disconnected cells
//! ```
//!
//! ## Usage Examples
//!
//! ### Creating a Simple Road
//!
//! ```rust
//! use micro_traffic_sim_core::grid::{
//!     road_network::GridRoads,
//!     cell::Cell,
//!     zones::ZoneType
//! };
//! use micro_traffic_sim_core::geom::new_point;
//! 
//! let mut grid = GridRoads::new();
//! 
//! // Create a simple 3-cell road
//! let start_cell = Cell::new(1)
//!     .with_point(new_point(0.0, 0.0, None))
//!     .with_zone_type(ZoneType::Birth)
//!     .with_speed_limit(3)
//!     .with_forward_node(2)
//!     .build();
//! 
//! let middle_cell = Cell::new(2)
//!     .with_point(new_point(1.0, 0.0, None))
//!     .with_zone_type(ZoneType::Common)
//!     .with_speed_limit(3)
//!     .with_forward_node(3)
//!     .build();
//! 
//! let end_cell = Cell::new(3)
//!     .with_point(new_point(2.0, 0.0, None))
//!     .with_zone_type(ZoneType::Death)
//!     .with_speed_limit(3)
//!     .with_forward_node(-1)  // No forward connection
//!     .build();
//! 
//! grid.add_cell(start_cell);
//! grid.add_cell(middle_cell);
//! grid.add_cell(end_cell);
//! 
//! // Access cells
//! if let Some(cell) = grid.get_cell(&1) {
//!     println!("Cell 1 at: {:?}", cell.get_point());
//! }
//! ```
//!
//! ### Creating Multi-Lane Roads
//!
//! ```rust
//! use micro_traffic_sim_core::grid::{road_network::GridRoads, cell::Cell, zones::ZoneType};
//! use micro_traffic_sim_core::geom::new_point;
//! 
//! let mut grid = GridRoads::new();
//! 
//! // Lane 1 (bottom lane)
//! let lane1_cell = Cell::new(1)
//!     .with_point(new_point(0.0, 1.0, None))
//!     .with_zone_type(ZoneType::Birth)
//!     .with_forward_node(2)
//!     .with_left_node(3)  // Can change to upper/left lane
//!     .build();
//! 
//! // Lane 2 (upper lane)  
//! let lane2_cell = Cell::new(3)
//!     .with_point(new_point(0.0, 2.0, None))
//!     .with_zone_type(ZoneType::Birth)
//!     .with_forward_node(4)
//!     .with_right_node(1)  // Can change to lower/right lane
//!     .build();
//! 
//! grid.add_cell(lane1_cell);
//! grid.add_cell(lane2_cell);
//! ```
//!
//! ### Cell State Management
//!
//! **End-developer most of time do not need to manage cell states directly**,
//! but in some scenarios it can be useful to temporarily ban a cell.
//! Current engine internally changes some cell states during simulation, when
//! traffic lights have red signal.
//! ```rust
//! use micro_traffic_sim_core::grid::{road_network::GridRoads, cell::{Cell, CellState}};
//! 
//! let mut grid = GridRoads::new();
//! let mut cell = Cell::new(1).build();
//! 
//! // Temporarily close a road segment
//! cell.set_state(CellState::Banned);
//! println!("Cell state: {}", cell.get_state()); // Movement to the cell will be prohibited
//! 
//! // Reopen the road segment
//! cell.set_state(CellState::Free);
//! println!("Cell state: {}", cell.get_state()); // Cell is accessible
//! ```
//!
//! ## Connection Rules
//!
//! - Use `-1` to indicate "no connection available" for any cell reference
//! - Forward connections create the straight traffic flow direction
//! - Left/Right connections enable lane changing behavior  
//! - Each cell can have only one forward connection, left connection, and right connection.
//! - Same time each cell can have multiple incoming connections from other cells, but it is recommended two have only one left/right incoming connection to avoid ambiguity in lane changing (number forward connections is unlimited in that context).

//! ## Integration
//!
//! The grid module integrates with other parts of the simulation:
//! - **Geometry**: Cells use [`PointType`](crate::geom::PointType) for spatial positioning
//! - **Agents**: Vehicles movement between cells following connection rules  
//! - **Conflict zones**: Coordinate vehicle movement at intersections

pub mod cell;
pub mod road_network;
pub mod zones;