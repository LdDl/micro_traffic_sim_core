//! Detection and resolution of cell, body, and trajectory conflicts.
//!
//! Shared conflict data is independent of collection, arbitration, and resolution.

mod conflict_rule;
mod conflict_types;
mod conflict_zone_index;
mod conflicts;
mod conflicts_solver;
mod merge_arbitration;

pub use self::conflict_rule::*;
pub use self::conflict_types::*;
pub use self::conflict_zone_index::CrossingZoneIndex;
pub use self::conflicts::*;
pub use self::conflicts_solver::*;
