//! Vehicle state, builders, body movement, and driver waiting limits.

mod vehicle;

pub use self::vehicle::{
    AGGRESSOR_CUT_IN_THRESHOLD, DEFAULT_YIELD_CYCLE_DELAY, PATIENCE_MAX, PATIENCE_MIN,
    TAIL_PLACEHOLDER, Vehicle, VehicleBuilder, VehicleError, VehicleID, VehicleRef,
};

#[cfg(test)]
mod tests;
