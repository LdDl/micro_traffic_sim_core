//! Resolution of movement conflicts and restrictions on granted paths.

mod conflicts_solver;

pub use self::conflicts_solver::{ConflictSolverError, solve_conflicts};
