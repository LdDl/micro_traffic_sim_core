//! Deterministic random number generation for testing.
//! 
//! This module provides a drop-in replacement for `rand::thread_rng()` that uses
//! a fixed seed during testing to ensure reproducible results.
#[cfg(test)]
use rand::Rng;

#[cfg(not(test))]
pub use rand::thread_rng;

#[cfg(test)]
pub fn thread_rng() -> impl Rng {
    // Return a fixed-seed RNG for testing
    use rand::SeedableRng;
    rand::rngs::StdRng::seed_from_u64(42)
}
