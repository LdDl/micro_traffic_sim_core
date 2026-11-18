//! Vehicle generation schedules, route definitions, and vehicle parameter overrides.

mod trip;

pub use self::trip::{Trip, TripBuilder, TripID, TripType};

#[cfg(test)]
mod tests;
