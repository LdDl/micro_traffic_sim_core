//! Centralized, seedable RNG for the simulation's stochastic decisions: the NaSch random
//! slowdown and the conflict tie-break coin-flips. A single thread-local stream makes a whole
//! run reproducible.
//!
//! Set `MTSC_SEED=<u64>` for a deterministic run (needed to measure changes against a noisy
//! congested baseline). Otherwise it is entropy-seeded in production, and fixed at 42 under
//! `cfg(test)` so unit tests stay reproducible.

use std::cell::RefCell;
use rand::Rng;
use rand::SeedableRng;
use rand::rngs::StdRng;

thread_local! {
    static RNG: RefCell<StdRng> = RefCell::new(make_rng());
}

fn make_rng() -> StdRng {
    if let Ok(s) = std::env::var("MTSC_SEED") {
        if let Ok(seed) = s.parse::<u64>() {
            return StdRng::seed_from_u64(seed);
        }
    }
    #[cfg(test)]
    {
        StdRng::seed_from_u64(42)
    }
    #[cfg(not(test))]
    {
        StdRng::seed_from_u64(rand::random::<u64>())
    }
}

/// A coin flip that comes true with probability `p` (conflict tie-breaks).
pub fn random_bool(p: f64) -> bool {
    RNG.with(|r| r.borrow_mut().random_bool(p))
}

/// A uniform f64 in [0, 1) (the NaSch random-slowdown roll).
pub fn random_f64() -> f64 {
    RNG.with(|r| r.borrow_mut().random::<f64>())
}
