//! # Intentions module
//!
//! **Internal simulation module** – Computes vehicle movement intentions for each simulation step.
//!
//! **This module is used internally by the simulation engine and should not be called directly by end-developer.**
//! It is invoked as part of the simulation step process in [`Session::step()`](crate::simulation::session::Session::step).
//!
//! ## Purpose
//!
//! This module calculates the intended maneuvers and target cells for all vehicles:
//! 1. **Determines next move** for each vehicle (forward, lane change, stop, etc.)
//! 2. **Handles tail cells** for multi-cell vehicles
//! 3. **Supports fallback logic** when no route is found
//! 4. **Stores intentions** for later conflict resolution
//!
//! ## Simulation integration
//!
//! The intentions module is step 4 in the simulation pipeline:
//! ```text
//! Session::step() Pipeline:
//! 1. Generate vehicles (trips)
//! 2. Update positions  
//! 3. Traffic light updates
//! 4. Prepare intentions    ← THIS MODULE
//! 5. Collect conflicts     ← conflicts module  
//! 6. Solve conflicts       ← conflicts module
//! 7. Execute movement      ← movement module
//! 8. Collect state dump
//! ```
//!
//! ## Components
//!
//! - [`intentions_datastorage::Intentions`] – Storage for all cell intentions
//! - [`intention_type::IntentionType`] – Target, Transit, Tail, or Undefined
//! - [`intention_type::CellIntention`] – Vehicle + intention type for a cell
//! - [`intention::prepare_intentions`] – Main entry point for intention calculation (internal use)
//! - [`intention::find_intention`] – Computes intention for a single vehicle
//! - [`intention::find_alternate_intention`] – Attempts lane change if blocked
//! - [`intention_path::process_path`] – Trims and analyzes possible movement along a path
//! - [`intention_no_route::process_no_route_found`] – Fallback for unreachable destinations
//!
//! ## Intention types
//!
//! - **Target**: Vehicle wants to move to this cell
//! - **Transit**: Vehicle passes through this cell at speed > 1
//! - **Tail**: Cell is occupied by the tail of a multi-cell vehicle
//! - **Undefined**: No intention set
//!
//! ## Error handling
//!
//! - Returns detailed errors if cells are missing, speed limits are invalid, or no route is found
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
//! // This internally calls intentions::prepare_intentions()
//! let result = session.step();
//! ```
mod intention_type;
mod intentions_datastorage;
mod intention_no_route;
mod intention_path;
mod intention;

pub use self::{intention_type::*, intentions_datastorage::*, intention_no_route::*, intention_path::*, intention::*};