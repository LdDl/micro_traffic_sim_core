//! # Trips Module
//! 
//! Vehicle generation and trip management for traffic simulation.
//!
//! This module provides trip definitions that control when, where, and how vehicles
//! are spawned in the simulation. Trips act as vehicle generators with configurable
//! timing, routing, and vehicle characteristics.
//!
//! ## Key Components
//! 
//! - [`trip::Trip`] - Vehicle generator with routing and timing configuration
//! - [`trip::TripBuilder`] - Builder pattern for creating trips
//! - [`trip::TripType`] - Generation patterns (constant frequency vs random)
//! - [`trip::TripID`] - Type alias for trip identifiers
//!
//! ## Generation Patterns
//!
//! ### Constant Generation
//! Spawns vehicles at regular intervals:
//! ```rust
//! use micro_traffic_sim_core::trips::trip::{Trip, TripType};
//! 
//! // Generate vehicle every 5 seconds from cell 1 to cell 100
//! let constant_trip = Trip::new(1, 100, TripType::Constant)
//!     .with_time(5)  // Every 5 seconds
//!     .build();
//! ```
//!
//! ### Random Generation  
//! Spawns vehicles probabilistically each time step:
//! ```rust
//! use micro_traffic_sim_core::trips::trip::{Trip, TripType};
//! 
//! // 30% chance to generate vehicle each time step
//! let random_trip = Trip::new(1, 100, TripType::Random)
//!     .with_probability(0.3)
//!     .build();
//! ```
//!
//! ## Integration
//! 
//! - **Grid System**: Uses [`CellID`](crate::grid::cell::CellID) for routing
//! - **Agents**: Generates [`Vehicle`](crate::agents::Vehicle) instances
//! - **Behavior**: Assigns [`BehaviourType`](crate::behaviour::BehaviourType) to vehicles
//! - **Agent Types**: Controls [`AgentType`](crate::agents_types::AgentType) (car, bus, etc.)
pub mod trip;
