//! Timed traffic light controllers and their signal groups.

mod lights;

pub use self::lights::{TrafficLight, TrafficLightBuilder, TrafficLightError, TrafficLightID};

#[cfg(test)]
mod tests;
