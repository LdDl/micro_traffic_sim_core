//! Conflict classifications, participant records, and detection errors.

mod conflict_types;

pub use self::conflict_types::{CellConflict, ConflictError, ConflictType, TrajectoryConflictInfo};
