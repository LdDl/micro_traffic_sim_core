use std::fmt;

/// ZoneType gives meaning to the cell in terms of its application.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ZoneType {
    Undefined = 0,
    Birth,
    Death,
    Coordination,
    Common,
    Isolated,
    LaneForBus,
    Transit,
    Crosswalk,
}

impl fmt::Display for ZoneType {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            ZoneType::Undefined => write!(f, "undefined"),
            ZoneType::Birth => write!(f, "birth"),
            ZoneType::Death => write!(f, "death"),
            ZoneType::Coordination => write!(f, "coordination"),
            ZoneType::Common => write!(f, "common"),
            ZoneType::Isolated => write!(f, "isolated"),
            ZoneType::LaneForBus => write!(f, "bus_lane"),
            ZoneType::Transit => write!(f, "transit"),
            ZoneType::Crosswalk => write!(f, "crosswalk"),
        }
    }
}