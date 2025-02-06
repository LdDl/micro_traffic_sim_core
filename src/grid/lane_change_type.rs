use std::fmt;

/// Represents the type of lane change maneuvers.
#[derive(Default, Debug, Clone, Copy, Eq, PartialEq)]
pub enum LaneChangeType {
    #[default]
    // Unknow lane change type
    Undefined = 0,
    // Straightforward
    NoChange,
    // Maneuver to the right
    ChangeRight,
    // Maneuver to the left
    ChangeLeft,
    // Simply block current position
    Block,
}

impl fmt::Display for LaneChangeType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let as_str = match self {
            LaneChangeType::Undefined => "undefined",
            LaneChangeType::NoChange => "no",
            LaneChangeType::ChangeRight => "right",
            LaneChangeType::ChangeLeft => "left",
            LaneChangeType::Block => "block",
        };
        write!(f, "{}", as_str)
    }
}
