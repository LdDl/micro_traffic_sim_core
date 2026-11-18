//! Collection of cell, body, and trajectory conflicts from movement intentions.

mod conflicts;

pub use self::conflicts::{
    collect_conflicts, collect_conflicts_indexed, find_conflict_type,
    find_conflicts_in_conflict_zones, find_cross_trajectories_conflict_naive,
    find_zone_conflict_for_two_intentions, new_conflict_multiple,
};
