//! # Logging Module
//! 
//! Structured logging system for traffic simulation debugging and monitoring.
//!
//! This module provides hierarchical logging levels and structured event tracking
//! using the `tracing` crate with JSON output format.
//!
//! **Most of time end-developer should not use this module directly, except
//! for setting the global logging level and using logging macros.**
//! 
//! ## Components
//! 
//! - [`VerboseLevel`] - Hierarchical debug levels (None → Main → Additional → Detailed → All)
//! - [`verbose_log`] - Global logging functions
//! - Event constants - Predefined event types for simulation phases
//! - Macros - `log_main!`, `log_additional!`, `log_detailed!`, `log_all!`
//!
//! ## Quick Start
//!
//! ```rust
//! use micro_traffic_sim_core::verbose::{set_verbose_level, VerboseLevel, EVENT_STEP};
//! use micro_traffic_sim_core::log_main;
//! 
//! // Set global logging level
//! set_verbose_level(VerboseLevel::Main);
//! 
//! // Log simulation events
//! log_main!(EVENT_STEP, "Starting simulation step", step = 42);
//! ```
//!
//! ## Logging Levels
//!
//! - `None` - No logging
//! - `Main` - Major simulation phases only  
//! - `Additional` - Nested function details
//! - `Detailed` - Loop iterations and fine-grained operations
//! - `All` - Everything (trace level)
//!
//! **Note**: This module may be refactored (completely!) in future versions.
pub mod verbose;

pub use self::{verbose::*};

// Initialize logger when module is loaded
use std::sync::Once;

static INIT: Once = Once::new();

pub fn ensure_logger_init() {
    INIT.call_once(|| {
        init_logger();
    });
}