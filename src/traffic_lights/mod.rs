//! # Traffic Lights Module
//! 
//! Traffic signal control system for managing vehicle flow at intersections in cellular automata simulation.
//!
//! This module provides a traffic light system with signal phases, timing control,
//! and group-based management of cells that share the same traffic signal behavior.
//!
//! ## Architecture
//!
//! ### Three-layer system. Example:
//! ```text
//! TrafficLight - Layer #1 (intersection)
//! ├── TrafficLightGroup (North-South) - Layer #2 (group of cells)
//! │   ├── Signal phases: [Red, Yellow, Green]  - Layer #3 (signal states)
//! │   └── Controlled cells: [1, 2, 3] - Layer #3 (cells IDs)
//! ├── TrafficLightGroup (East-West)
//! │   ├── Signal phases: [Green, Green, Red]
//! │   └── Controlled cells: [10, 11, 12]
//! └── Phase times: [10s, 3s, 15s] - Layer #1 (timing)
//! ```
//!
//! Phase times are shared across all groups in a traffic light, while each group
//! can have its own sequence of signal states. Number of phases times array must
//! match the length of signal states in each group.
//! 
//! ## Components
//! 
//! ### Core Structures
//! - [`lights::TrafficLight`] - Traffic light controller with timing and groups. One controller == one intersection
//! - [`groups::TrafficLightGroup`] - Group of cells sharing the same coordinated signal behavior
//! - [`signals::SignalType`] - Individual signal states (Red, Yellow, Green, etc.)
//! - [`lights::TrafficLightID`] - Unique identifier for traffic lights
//!
//! ### Builder Pattern
//! - [`lights::TrafficLightBuilder`] - API builder for traffic lights
//! - [`groups::TrafficLightGroupBuilder`] - API builder for signal groups
//!
//! ### Error Handling
//! - [`lights::TrafficLightError`] - Traffic light operation errors
//! - [`signals::SignalTypeError`] - Signal parsing and validation errors
//!
//! ## Signal Types
//! 
//! Reference for signal states based on SUMO definitions:
//! https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html#signal_state_definitions
//! 
//!
//! | Signal | Code | Meaning |
//! |--------|------|---------|
//! | `Red` | `r` | Vehicles must stop |
//! | `Yellow` | `y` | Prepare to stop |
//! | `Green` | `g` | Vehicles may proceed |
//! | `GreenPriority` | `G` | Vehicles proceed with priority |
//! | `GreenRight` | `s` | Vehicles may proceed only right turn |
//! | `RedYellow` | `u` | Prepare for green (vehicles still stop) |
//! | `Blinking` | `o` | Vehicles must yield |
//! | `NoSignal` | `O` | Vehicles have right of way |
//! | `Undefined` | - | Uninitialized state |
//!
//! ## Usage Examples
//!
//! ### Basic four-way intersection
//!
//! ```rust
//! use micro_traffic_sim_core::traffic_lights::{
//!     lights::{TrafficLight, TrafficLightID},
//!     groups::TrafficLightGroup,
//!     signals::SignalType
//! };
//! use micro_traffic_sim_core::geom::new_point;
//! 
//! // Create signal groups for each direction
//! let north_south_group = TrafficLightGroup::new(1)
//!     .with_label("North-South".to_string())
//!     .with_cells_ids(vec![1, 2, 3])  // Cells for N-S traffic
//!     .with_signal(vec![SignalType::Green, SignalType::Yellow, SignalType::Red])
//!     .build();
//! 
//! let east_west_group = TrafficLightGroup::new(2)
//!     .with_label("East-West".to_string())
//!     .with_cells_ids(vec![10, 11, 12])  // Cells for E-W traffic
//!     .with_signal(vec![SignalType::Red, SignalType::RedYellow, SignalType::Green])
//!     .build();
//! 
//! // Create traffic light with timing
//! let mut intersection = TrafficLight::new(TrafficLightID(1))
//!     .with_coordinates(new_point(100.0, 200.0, None))
//!     .with_groups(vec![north_south_group, east_west_group])
//!     .with_phases_times(vec![15, 3, 15])  // Green: 15s, Yellow: 3s, Red: 15s
//!     .build();
//! 
//! // Simulate traffic light operation
//! for step in 0..40 {
//!     intersection.step();
//!     let phase = intersection.get_active_phase();
//!     let time = intersection.get_current_time();
//!     println!("Step {}: Phase {}, Time {}", step, phase, time);
//! }
//! ```
//!
//! ## Timing and Phase Management
//!
//! ### Phase Cycling
//! Traffic lights automatically cycle through phases based on configured timing:
//! - **Timer increments** each simulation step
//! - **Phase changes** when timer reaches phase duration  
//! - **Automatic reset** to phase 0 after completing all phases
//!
//! ### Group Synchronization
//! All groups in a traffic light share the same phase timing but can have
//! different signal states per phase, enabling complex intersection patterns.
//!
//! ## Integration
//!
//! - **Grid System**: Controls cells via [`CellID`](crate::grid::cell::CellID) references
//! - **Geometry**: Uses [`PointType`](crate::geom::PointType) for spatial positioning  
//! - **Coordination**: Should be used with [`ZoneType::Coordination`](crate::grid::zones::ZoneType::Coordination) cells
//!
pub mod signals;
pub mod groups;
pub mod lights;