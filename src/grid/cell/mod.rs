//! Road cell state, geometry, directional connections, and construction.

mod cell;

pub use self::cell::{Cell, CellBuilder, CellID, CellState};

#[cfg(test)]
mod tests;
