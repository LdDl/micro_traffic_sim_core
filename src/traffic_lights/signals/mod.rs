//! Traffic signal states, parsing, and display.

mod signals;

pub use self::signals::{SignalType, SignalTypeError};

#[cfg(test)]
mod tests;
