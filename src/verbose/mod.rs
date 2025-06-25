//! Export library
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