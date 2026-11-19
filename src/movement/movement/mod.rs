//! Application of resolved intentions to vehicle positions, tails, and timers.

mod movement;

pub use self::movement::{MovementError, movement};

#[cfg(test)]
mod tests;
