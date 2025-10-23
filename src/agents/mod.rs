// src/agents/mod.rs
//! # Agents Module
//! 
//! This module provides the core agent system for traffic simulation, implementing
//! cellular automata-based vehicle behavior and movement.
//!
//! ## Key Components
//! 
//! - [`Vehicle`] - The primary agent representing vehicles in the simulation
//! - [`VehicleIntention`] - Captures vehicle's planned actions and maneuvers
//!
//! ## Usage
//! 
//! ```rust
//! use micro_traffic_sim_core::agents::{Vehicle, VehicleIntention};
//! use micro_traffic_sim_core::agents_types::AgentType;
//! 
//! // Create a basic vehicle
//! let vehicle = Vehicle::new(1)
//!     .with_cell(10)
//!     .with_type(AgentType::Car)
//!     .with_destination(100)
//!     .build();
//! ```
//!
//! ## Features
//! 
//! - **Lane changing**: Complex maneuver intentions and tail behavior
//! - **Behavioral parameters**: Configurable aggressiveness, cooperation, timing
//! - **Builder pattern**: API for vehicle construction
mod vehicle_intention;
mod vehicle;

pub use self::{vehicle_intention::*, vehicle::*};
