//! # Conflict Zones Module
//! 
//! This module provides conflict resolution mechanisms for **unregulated intersections** (most of times)
//! where multiple vehicles compete for the same physical space without being coordinated by traffic signals.
//! Or being coordinated by traffic signals but still requiring priority rules due conflicting green signal phases.
//!
//! ## Overview
//! 
//! Conflict zones handle priority-based traffic coordination. They determine which vehicle
//! has right-of-way when other vehicle arrives simultaneously from different direction.
//!
//! ## Key Components
//! 
//! ### Core Structures
//! - [`conflict_zones::ConflictZone`] - Represents a priority-controlled zone
//! - [`conflict_zones::ConflictEdge`] - Defines a movement path through the zone 
//! - [`conflict_zones::ConflictZoneBuilder`] - Builder API for creating conflict zones
//! - [`conflict_zones::ConflictWinnerType`] - Priority rules (main vs secondary flow)
//! - [`conflict_zones::ConflictZoneType`] - Classification of zones types (future works)
//!
//! ## Conflict Resolution Model
//!
//! ### Unregulated junctions priority
//! 
//! Conflict zones can model **main road vs secondary road** typical scenario:
//! 
//! ```text
//! Roads (graph edges) meet at same center:
//! 
//!      Secondary Road
//!               (20)
//!                │
//!                ↓
//! ─(10)─────────(50)─────(51)─ Main Road (has priority)
//!                │
//!                │
//!               (52)
//! ```
//! 
//! ```text
//! Roads (graph edges) just intersect geometrically, but logically target different cells.
//! Could also the case when cells nearly overlap due floating point precision issues:
//! 
//!      Secondary Road
//!               (20)
//!                │
//!                ↓
//! ─(10)──────────────────(51)─ Main Road (has priority)
//!                │
//!                │
//!               (52)
//! ```
//!
//! ### Two Conflict Scenarios
//!
//! #### 1. Same target cell
//! ```rust
//! use micro_traffic_sim_core::conflict_zones::{
//!     ConflictEdge
//! };
//! // Two edges pointing to the same cell
//! let main_flow = ConflictEdge { source: 10, target: 50 };      // Main road → edges goes to junction center point 
//! let secondary_flow = ConflictEdge { source: 20, target: 50 }; // Side road → same junction center point
//! ```
//!
//! #### 2. Different target cells  
//! ```rust
//! use micro_traffic_sim_core::conflict_zones::{
//!     ConflictEdge
//! };
//! // Different cells at nearly same coordinates (floating point tolerance)
//! let main_flow = ConflictEdge { source: 10, target: 51 };      // Main road → edge goes through junction center point
//! let secondary_flow = ConflictEdge { source: 20, target: 52 }; // Side road → edge goes through junction center point too.
//! ```
//!
//! ### Priority System
//! 
//! - **`First`** - Main road has priority
//! - **`Second`** - Secondary road has priority
//! - **`Equal`** - Equal priority. Both yield to each other: so decision who goes first is based on vehicles speed or random in case of tie
//! - **`Undefined`** - Priority not determined. Behavior "should" be same as `Equal` but this state indicates misconfiguration.
//!
//! ## Usage Examples
//!
//! ### Main Road Priority (Typical Scenario)
//!
//! ```rust
//! use micro_traffic_sim_core::conflict_zones::{
//!     ConflictZone, ConflictEdge, ConflictWinnerType
//! };
//! 
//! // Main road has continuous flow
//! let main_road = ConflictEdge { source: 1, target: 2 };
//! // Secondary road must yield  
//! let side_road = ConflictEdge { source: 10, target: 11 };
//! 
//! let intersection = ConflictZone::new(1, main_road, side_road)
//!     .with_winner_type(ConflictWinnerType::First)
//!     .build();
//! ```
//!
//! ### Same Cell Intersection
//!
//! ```rust
//! use micro_traffic_sim_core::conflict_zones::{
//!     ConflictZone, ConflictEdge, ConflictWinnerType
//! };
//! // Both roads lead to the same intersection cell
//! let north_approach = ConflictEdge { source: 5, target: 100 };
//! let east_approach = ConflictEdge { source: 15, target: 100 };
//! 
//! let intersection = ConflictZone::new(1, north_approach, east_approach)
//!     .with_winner_type(ConflictWinnerType::First)
//!     .build();
//! ```
//!
//! ## Integration
//!
//! - **Grid System**: Works with [`CellID`](crate::grid::cell::CellID) from cellular grid
//! - **Conflicts detection and solver**: Works with conflicts module at [`collect_conflicts`](crate::conflicts).
mod conflict_zones;

pub use self::conflict_zones::*;