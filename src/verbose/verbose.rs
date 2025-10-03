// src/verbose/verbose.rs
use std::fmt;
use std::sync::OnceLock;
use tracing::{info, debug, trace, warn, error, Level};
use tracing_subscriber::{fmt as tracing_fmt, layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};

/// VerboseLevel is just distinction between very detailed debug information and not that detailed
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum VerboseLevel {
    /// No debug at all
    None = 0,
    /// Just print before each main function
    Main = 1,
    /// Nested output inside of each main function
    Additional = 2,
    /// Loop debug
    Detailed = 3,
    /// Log everything
    All = 4,
}

impl fmt::Display for VerboseLevel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            VerboseLevel::None => "none",
            VerboseLevel::Main => "main",
            VerboseLevel::Additional => "additional",
            VerboseLevel::Detailed => "detailed",
            VerboseLevel::All => "all",
        };
        write!(f, "{}", s)
    }
}

impl From<VerboseLevel> for Level {
    fn from(level: VerboseLevel) -> Self {
        match level {
            VerboseLevel::None => Level::ERROR,
            VerboseLevel::Main => Level::INFO,
            VerboseLevel::Additional => Level::DEBUG,
            VerboseLevel::Detailed => Level::DEBUG,
            VerboseLevel::All => Level::TRACE,
        }
    }
}

impl From<VerboseLevel> for String {
    fn from(level: VerboseLevel) -> Self {
        match level {
            VerboseLevel::None => "error".to_string(),
            VerboseLevel::Main => "info".to_string(),
            VerboseLevel::Additional => "debug".to_string(),
            VerboseLevel::Detailed => "debug".to_string(),
            VerboseLevel::All => "trace".to_string(),
        }
    }
}

// Event type constants
pub const EVENT_SIMULATION_RESET: &str = "simulation_reset";
pub const EVENT_STEP: &str = "step";
pub const EVENT_GEN_VEHICLES: &str = "generate_vehicles";
pub const EVENT_GEN_VEHICLE: &str = "generate_vehicle";
pub const EVENT_UPD_POS: &str = "update_pos";
pub const EVENT_TL_TICK: &str = "tl_tick";
pub const EVENT_INTENTIONS_CREATE: &str = "intentions_create";
pub const EVENT_INTENTION_VEHICLE: &str = "intention_vehicle";
pub const EVENT_INTENTION_ADD: &str = "intention_add";
pub const EVENT_CONFLICTS_COLLECT: &str = "conflicts_collect";
pub const EVENT_CONFLICT_CREATE: &str = "conflict_create";
pub const EVENT_CONFLICTS_SOLVE: &str = "conflicts_solve";
pub const EVENT_CONFLICT_SOLVE: &str = "conflict_solve";
pub const EVENT_MOVEMENT: &str = "movement";
pub const EVENT_MOVEMENT_VEHICLE: &str = "movement_vehicle";
pub const EVENT_MOVEMENT_DEAD_END: &str = "movement_dead_end";
pub const EVENT_MOVEMENT_DESTINATION: &str = "movement_destination";
pub const EVENT_SESSION_CREATE: &str = "session_create";
pub const EVENT_SESSION_EXPIRED: &str = "session_expired";
pub const EVENT_SESSION_EXTRACT_STATES: &str = "session_extract";

// Global verbose level storage
static VERBOSE_LEVEL: OnceLock<VerboseLevel> = OnceLock::new();
static LOGGER_INITIALIZED: OnceLock<bool> = OnceLock::new();

/// Initialize the tracing logger once
pub fn init_logger() {
    if LOGGER_INITIALIZED.set(true).is_ok() {
        tracing_subscriber::registry()
            .with(
                tracing_fmt::layer()
                    .json()
                    .with_target(false)
                    .with_thread_ids(false)
                    .with_thread_names(false)
                    .with_file(false)
                    .with_line_number(false)
            )
            .with(EnvFilter::from_default_env())
            .init();
    }
}

// ===== GLOBAL VERBOSE FUNCTIONS =====

/// Sets the global verbose level and updates tracing filter
pub fn set_verbose_level(level: VerboseLevel) {
    let _ = VERBOSE_LEVEL.set(level);
    init_logger();
}

/// Gets the current global verbose level
pub fn get_verbose_level() -> VerboseLevel {
    *VERBOSE_LEVEL.get().unwrap_or(&VerboseLevel::None)
}

/// Checks if current global verbose level is at least the specified level
pub fn is_verbose_level(level: VerboseLevel) -> bool {
    get_verbose_level() >= level
}

/// Logs a message if the global verbose level allows it
pub fn verbose_log(level: VerboseLevel, event: &str, message: &str) {
    if !is_verbose_level(level) {
        return;
    }

    match level {
        VerboseLevel::None => {}
        VerboseLevel::Main => {
            info!(event = event, message);
        }
        VerboseLevel::Additional => {
            debug!(event = event, message);
        }
        VerboseLevel::Detailed => {
            debug!(event = event, message);
        }
        VerboseLevel::All => {
            trace!(event = event, message);
        }
    }
}

/// Logs a message with additional fields using global verbose level
pub fn verbose_log_with_fields(
    level: VerboseLevel, 
    event: &str, 
    message: &str, 
    fields: &[(&str, &dyn fmt::Display)]
) {
    if !is_verbose_level(level) {
        return;
    }

    let mut field_map = std::collections::HashMap::new();
    for (key, value) in fields {
        field_map.insert(*key, format!("{}", value));
    }

    match level {
        VerboseLevel::None => {}
        VerboseLevel::Main => {
            info!(
                event = event,
                ?field_map,
                message
            );
        }
        VerboseLevel::Additional => {
            debug!(
                event = event,
                ?field_map,
                message
            );
        }
        VerboseLevel::Detailed => {
            debug!(
                event = event,
                ?field_map,
                message
            );
        }
        VerboseLevel::All => {
            trace!(
                event = event,
                ?field_map,
                message
            );
        }
    }
}

// ===== PER-SESSION VERBOSE METHODS =====

/// Session-specific logging functions
impl VerboseLevel {
    /// Logs a message if the session verbose level allows it
    pub fn log(self, event: &str, message: &str) {
        if self == VerboseLevel::None {
            return;
        }

        match self {
            VerboseLevel::None => {}
            VerboseLevel::Main => {
                info!(event = event, message);
            }
            VerboseLevel::Additional => {
                debug!(event = event, message);
            }
            VerboseLevel::Detailed => {
                debug!(event = event, message);
            }
            VerboseLevel::All => {
                trace!(event = event, message);
            }
        }
    }

    /// Logs a message with fields if the session verbose level allows it
    pub fn log_with_fields(self, event: &str, message: &str, fields: &[(&str, &dyn fmt::Display)]) {
        if self == VerboseLevel::None {
            return;
        }

        let mut field_map = std::collections::HashMap::new();
        for (key, value) in fields {
            field_map.insert(*key, format!("{}", value));
        }

        match self {
            VerboseLevel::None => {}
            VerboseLevel::Main => {
                info!(
                    event = event,
                    ?field_map,
                    message
                );
            }
            VerboseLevel::Additional => {
                debug!(
                    event = event,
                    ?field_map,
                    message
                );
            }
            VerboseLevel::Detailed => {
                debug!(
                    event = event,
                    ?field_map,
                    message
                );
            }
            VerboseLevel::All => {
                trace!(
                    event = event,
                    ?field_map,
                    message
                );
            }
        }
    }

    /// Checks if this level is at least the minimum level
    pub fn is_at_least(self, min_level: VerboseLevel) -> bool {
        self >= min_level
    }
}

// ===== CONVENIENCE MACROS =====

/// Convenience macro for global verbose logging
#[macro_export]
macro_rules! verbose_log {
    ($level:expr, $event:expr, $msg:literal) => {
        $crate::verbose::verbose_log($level, $event, $msg)
    };
    ($level:expr, $event:expr, $msg:literal, $($key:literal => $value:expr),+) => {
        $crate::verbose::verbose_log_with_fields(
            $level, 
            $event, 
            $msg, 
            &[$(($key, &$value)),+]
        )
    };
}

/// Global tracing-native macros for more idiomatic usage
#[macro_export]
macro_rules! log_main {
    ($event:expr, $msg:literal, $($key:ident = $value:expr),*) => {
        if $crate::verbose::is_verbose_level($crate::verbose::VerboseLevel::Main) {
            tracing::info!(
                event = $event,
                $($key = $value,)*
                $msg
            );
        }
    };
}

#[macro_export]
macro_rules! log_additional {
    ($event:expr, $msg:literal, $($key:ident = $value:expr),*) => {
        if $crate::verbose::is_verbose_level($crate::verbose::VerboseLevel::Additional) {
            tracing::debug!(
                event = $event,
                $($key = $value,)*
                $msg
            );
        }
    };
}

#[macro_export]
macro_rules! log_detailed {
    ($event:expr, $msg:literal, $($key:ident = $value:expr),*) => {
        if $crate::verbose::is_verbose_level($crate::verbose::VerboseLevel::Detailed) {
            tracing::debug!(
                event = $event,
                $($key = $value,)*
                $msg
            );
        }
    };
}

#[macro_export]
macro_rules! log_all {
    ($event:expr, $msg:literal, $($key:ident = $value:expr),*) => {
        if $crate::verbose::is_verbose_level($crate::verbose::VerboseLevel::All) {
            tracing::trace!(
                event = $event,
                $($key = $value,)*
                $msg
            );
        }
    };
}