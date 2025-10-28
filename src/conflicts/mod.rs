//! # Conflicts module
//!
//! **Internal simulation module** – Detects and resolves vehicle movement conflicts for each simulation step.
//!
//! **This module is used internally by the simulation engine and should not be called directly by end-developer.**
//! It is invoked as part of the simulation step process in [`Session::step()`](crate::simulation::session::Session::step).
//!
//! ## Purpose
//!
//! This module analyzes all vehicle intentions and detects conflicts where multiple vehicles want to move to the same cell or their trajectories intersect.
//! It then resolves these conflicts according to priority rules, conflict zones, vehicle behavior, and maneuvers.
//!
//! ## Simulation integration
//!
//! The conflicts module is steps 5–6 in the simulation pipeline:
//! ```text
//! Session::step() Pipeline:
//! 1. Generate vehicles (trips)
//! 2. Update positions  
//! 3. Traffic light updates
//! 4. Prepare intentions      ← intentions module
//! 5. Collect conflicts       ← THIS MODULE
//! 6. Solve conflicts         ← THIS MODULE
//! 7. Execute movement        ← movement module
//! 8. Collect state dump
//! ```
//!
//! ## Components
//!
//! - [`conflicts::collect_conflicts`] – Collects all cell and trajectory conflicts from intentions
//! - [`conflicts_solver::solve_conflicts`] – Resolves conflicts and updates vehicle intentions
//! - [`conflicts::CellConflict`] – Represents a detected conflict and its participants
//! - [`conflicts::ConflictType`] – Describes the type of conflict (merge, cross, tail, etc.)
//! - [`conflicts::ConflictError`] – Error types for conflict detection
//! - [`conflicts_solver::ConflictSolverError`] – Error types for conflict resolution
//! - [`conflict_rule`](crate::conflicts::conflict_rule) – Priority rules for resolving conflicts
//!
//! ## Conflict types
//!
//! All of the following conflict types are described by the [`conflicts::ConflictType`](ConflictType) enum:
//! - **ForwardLaneChange**: Forward vs. lane change maneuver.
//! - **MergeForward**: Multiple vehicles merging forward.
//! - **MergeLaneChange**: Multiple vehicles merging via lane change.
//! - **CrossLaneChange**: Multiple vehicles crossing via trajectory lane change.
//! - **CrossConflictZone**: Multiple vehicles crossing via trajectory lane change in a conflict zone.
//! - **Tail**: Any single-cell vehicle met multi-cell vehicle's tail in front of it.
//! - **SelfTail**: Multi-cell vehicle moves with speed > 1 and there occurs self-tail conflict, since there are intermediate intentions.
//! - **BlockLaneChange**: Lane change blocked by stopped vehicle.
//! - **MergeForwardConflictZone**: Merge in a conflict zone.
//! - **TailCrossLaneChange**: Lane change trajectory intersects with another vehicle’s tail.
//!
//! ## Error handling
//!
//! - Returns errors if cells or vehicles are missing, or if conflict resolution fails
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
//! // This internally calls conflicts::collect_conflicts
//! // and conflicts_solver::solve_conflicts sequentially.
//! let result = session.step();
//! ```
mod conflicts;
mod conflicts_test;
mod conflicts_zones_test;
mod conflict_rule;
mod conflicts_solver;

pub use self::{conflicts::*, conflicts_test::*, conflicts_zones_test::*, conflict_rule::*, conflicts_solver::*};
