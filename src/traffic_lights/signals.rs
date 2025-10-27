use std::collections::HashMap;
use std::fmt;
use lazy_static::lazy_static;

/// Custom error types for `SignalType`.
#[derive(Debug, Clone)]
pub enum SignalTypeError {
    /// Indicates that the provided signal type string is invalid.
    InvalidSignalType(String),
}

impl fmt::Display for SignalTypeError {
    /// Formats the error message for `SignalTypeError`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SignalTypeError::InvalidSignalType(value) => {
                write!(f, "Invalid signal type: '{}'", value)
            }
        }
    }
}

impl std::error::Error for SignalTypeError {}

lazy_static! {
    static ref SIGNAL_CONVERTER: HashMap<&'static str, SignalType> = {
        let mut m = HashMap::new();
        m.insert("r", SignalType::Red);
        m.insert("y", SignalType::Yellow);
        m.insert("g", SignalType::Green);
        m.insert("G", SignalType::GreenPriority);
        m.insert("s", SignalType::GreenRight);
        m.insert("u", SignalType::RedYellow);
        m.insert("o", SignalType::Blinking);
        m.insert("O", SignalType::NoSignal);
        m
    };
}

/// Represents the different states of traffic lights.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SignalType {
    /// Undefined signal type.
    Undefined,
    /// Red light - vehicles must stop.
    Red,
    /// Amber (yellow) light - vehicles start to decelerate if far from the junction; otherwise, they pass.
    Yellow,
    /// Green light with no priority - vehicles may pass the junction if no higher-priority vehicle is present.
    Green,
    /// Green light with priority - vehicles may pass the junction with priority.
    GreenPriority,
    /// Green right-turn arrow - vehicles may pass the junction after stopping.
    GreenRight,
    /// Red and yellow light - indicates upcoming green phase; vehicles must still stop.
    RedYellow,
    /// Blinking signal - vehicles must yield.
    Blinking,
    /// No signal - vehicles have the right of way.
    NoSignal,
}

impl SignalType {
    /// Converts a string representation to a `SignalType`.
    ///
    /// # Arguments
    /// * `signal_str` - The string representation of the signal.
    ///
    /// # Returns
    /// A `Result` containing the corresponding `SignalType` or an error if the string is invalid.
    /// 
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::traffic_lights::signals::{SignalType};
    /// let sig_type = SignalType::from_str("g");
    /// ```
    pub fn from_str(signal_str: &str) -> Result<Self, SignalTypeError> {
        SIGNAL_CONVERTER
            .get(signal_str)
            .copied()
            .ok_or(SignalTypeError::InvalidSignalType(signal_str.to_string()))
    }
}

impl fmt::Display for SignalType {
    /// Formats the signal type for display.
    /// 
    /// Returns a short, lowercase string representation suitable for
    /// logging, debugging, and user interfaces.
    /// 
    /// # Examples
    /// 
    /// ```rust
    /// use micro_traffic_sim_core::traffic_lights::signals::SignalType;
    /// 
    /// assert_eq!(format!("{}", SignalType::Undefined), "undefined");
    /// assert_eq!(format!("{}", SignalType::Red), "r");
    /// assert_eq!(format!("{}", SignalType::Yellow), "y");
    /// assert_eq!(format!("{}", SignalType::Green), "g");
    /// assert_eq!(format!("{}", SignalType::GreenPriority), "G");
    /// assert_eq!(format!("{}", SignalType::GreenRight), "s");
    /// assert_eq!(format!("{}", SignalType::RedYellow), "u");
    /// assert_eq!(format!("{}", SignalType::Blinking), "o");
    /// assert_eq!(format!("{}", SignalType::NoSignal), "O");
    /// ```
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let signal_type_str = match self {
            SignalType::Undefined => "undefined",
            SignalType::Red => "r",
            SignalType::Yellow => "y",
            SignalType::Green => "g",
            SignalType::GreenPriority => "G",
            SignalType::GreenRight => "s",
            SignalType::RedYellow => "u",
            SignalType::Blinking => "o",
            SignalType::NoSignal => "O",
        };
        write!(f, "{}", signal_type_str)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_parse_signal_valid() {
        // Test for valid signal strings
        assert_eq!(SignalType::from_str("r").unwrap(), SignalType::Red);
        assert_eq!(SignalType::from_str("y").unwrap(), SignalType::Yellow);
        assert_eq!(SignalType::from_str("g").unwrap(), SignalType::Green);
        assert_eq!(SignalType::from_str("G").unwrap(), SignalType::GreenPriority);
        assert_eq!(SignalType::from_str("s").unwrap(), SignalType::GreenRight);
        assert_eq!(SignalType::from_str("u").unwrap(), SignalType::RedYellow);
        assert_eq!(SignalType::from_str("o").unwrap(), SignalType::Blinking);
        assert_eq!(SignalType::from_str("O").unwrap(), SignalType::NoSignal);
    }
    #[test]
    fn test_parse_signal_invalid() {
        // Test for invalid signal strings
        let result = SignalType::from_str("z");
        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err().to_string(),
            "Invalid signal type: 'z'"
        );
    }
}