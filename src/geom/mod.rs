//! # Geometry Module
//! 
//! This module provides geometric primitives and spatial calculations for traffic simulation,
//! supporting both Euclidean and geographic coordinate systems.
//!
//! ## Key Components
//! 
//! ### Point System
//! - [`Point`] - Core trait defining point operations
//! - [`PointType`] - Enum supporting multiple coordinate systems
//! - [`EuclideanPoint`] - Points in Euclidean space (for grid-based simulations)
//! - [`WGS84Point`] - Geographic points using WGS84 coordinate system
//! - [`SRID`] - Spatial Reference System Identifiers
//!
//! ### Spatial Functions
//! - [`get_bearing`] - Calculate bearing between two geographic points
//! - [`gc_distance`] / [`gc_distance_pt`] - Great-circle distance calculations
//! - [`convert_epsg4326_to_3857`] / [`convert_epsg3857_to_4326`] - Coordinate system conversions
//!
//! ## Coordinate Systems
//! 
//! The module supports two primary coordinate systems:
//! 
//! - **Euclidean** (`SRID::Euclidean`): For abstract grid-based simulations
//! - **WGS84** (`SRID::WGS84`): For real-world geographic simulations
//!
//! ## Usage Examples
//!
//! ### Creating Points
//! 
//! ```rust
//! use micro_traffic_sim_core::geom::{new_point, SRID, Point};
//! 
//! // Euclidean point (default)
//! let grid_point = new_point(10.0, 20.0, None);
//! 
//! // Geographic point (WGS84)
//! let moscow = new_point(37.6176, 55.7558, Some(SRID::WGS84));
//! let spb = new_point(30.3141, 59.9386, Some(SRID::WGS84));
//! 
//! // Calculate distance
//! let distance = moscow.distance_to(&spb); // Great-circle distance in meters
//! ```
//!
//! ### Spatial Calculations
//! 
//! ```rust
//! use micro_traffic_sim_core::geom::{new_point, SRID, get_bearing, gc_distance};
//! 
//! let pt1 = new_point(37.6176, 55.7558, Some(SRID::WGS84)); // Moscow
//! let pt2 = new_point(30.3141, 59.9386, Some(SRID::WGS84)); // St. Petersburg
//! 
//! // Get bearing from Moscow to St. Petersburg
//! let bearing = get_bearing(&pt1, &pt2); // Degrees from North
//! 
//! // Calculate great-circle distance
//! let distance = gc_distance(37.6176, 55.7558, 30.3141, 59.9386); // Meters
//! ```
//!
//! ### Coordinate Conversions
//! 
//! ```rust
//! use micro_traffic_sim_core::geom::{convert_epsg4326_to_3857, convert_epsg3857_to_4326};
//! 
//! // Convert from WGS84 to Web Mercator (for web mapping)
//! let (x_mercator, y_mercator) = convert_epsg4326_to_3857(37.6176, 55.7558);
//! 
//! // Convert back to WGS84
//! let (lon, lat) = convert_epsg3857_to_4326(x_mercator, y_mercator);
//! ```
//!
//! ## Applications
//! 
//! - **Grid-based simulations**: Use Euclidean points for cellular automata
//! - **Real-world simulations**: Use WGS84 points with real geographic data
//! - **Web mapping integration**: Convert between coordinate systems for visualization
//! - **Navigation**: Calculate bearings and distances for route planning

mod point;
mod spatial;

pub use self::{point::*, spatial::*};