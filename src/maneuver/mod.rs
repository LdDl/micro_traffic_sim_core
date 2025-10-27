//! # Maneuver Module
//! 
//! Defines vehicle maneuver types for cellular automata traffic simulation.
//! 
//! ## Components
//! 
//! - [`LaneChangeType`] - Vehicle movement decisions (straight, left, right, block)
//! 
//! ## Example
//! 
//! ```rust
//! use micro_traffic_sim_core::maneuver::LaneChangeType;
//! 
//! let maneuver = LaneChangeType::ChangeRight;
//! println!("{}", maneuver); // "right"
//! ```
pub mod lane_change_type;

pub use self::{lane_change_type::*};