//! # Movement module
//! 
//! **Internal simulation module** - Vehicle movement execution for cellular automata traffic simulation.
//!
//! **This module is used internally by the simulation engine and should not be called directly by end-developer.**
//! It is invoked as part of the simulation step process in [`Session::step()`](crate::simulation::session::Session::step).
//!
//! ## Purpose
//!
//! This module executes the final movement phase of the cellular automata simulation:
//! 1. **Applies resolved vehicle intentions** (computed by intentions module)
//! 2. **Updates vehicle positions** based on conflict resolution outcomes
//! 3. **Handles special movement cases** (multi-cell vehicles, timers/counters)
//! 4. **Removes done vehicles** (reached destination or despawn zones)
//!
//! ## Simulation integration
//!
//! The movement module is the final step in the simulation pipeline:
//! ```text
//! Session::step() Pipeline:
//! 1. Generate vehicles (trips)
//! 2. Update positions  
//! 3. Traffic light updates
//! 4. Prepare intentions  ← intentions module
//! 5. Collect conflicts   ← conflicts module  
//! 6. Solve conflicts     ← conflicts module
//! 7. Execute movement    ← THIS MODULE
//! 8. Collect state dump
//! ```
//!
//! ## Components
//! 
//! - [`movement::movement`] - Main movement execution function
//! - [`movement::MovementError`] - Error types for movement failures
//!
//! ## Movement logic
//!
//! ### Vehicle state updates
//! - **Position updates**: Vehicle moves to intended cell
//! - **Bearing calculation**: Updates direction angle based on movement
//! - **Timers/counters management**: Decrements acceleration/maneuver/slowdown counters.
//! - **Tail cell management**: Updates multi-cell vehicle positions
//!
//! ### Special cases
//! - **Lane change timers**: Sets movement restrictions after lane changes
//! - **Transit logic**: FUTURE WORKS. Handles routes which contain cells for mandatory traversal
//! - **Relax countdown**: FUTURE WORKS. Delays movement for specific scenarios (bus waiting at stop)
//! - **Vehicle removal**: Removes vehicles at destinations or despawn zones
//!
//! ## Error handling
//!
//! - Vehicle references non-existent cell ID
//!
//! ## Usage context
//!
//! **DO NOT call directly** - use [`Session::step()`](crate::simulation::session::Session::step) instead:
//!
//! ```rust
//! use micro_traffic_sim_core::simulation::session::Session;
//! 
//! let mut session = Session::default(None);
//! // ... set up grid, trips, vehicles ...
//! 
//! // This internally calls movement::movement()
//! let result = session.step();
//! ```
pub mod movement;

pub use self::{movement::*};
