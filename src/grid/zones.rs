use std::fmt;

/// ZoneType gives meaning to the cell in terms of its application.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ZoneType {
    // Default uninitialized state
    Undefined = 0,
    // Vehicles spawn here
    Birth,
    // Vehicles despawn here
    Death,
    // Junction/intersection cell requiring conflict resolution
    Coordination,
    // Regular road segment
    Common,
    // Cell that is disconnected from the road network (for future use)
    Isolated,
    // Dedicated bus lane (for future use)
    LaneForBus,
    // Relaxation cells (stops basically) for public transport vehicles (for future use)
    Transit,
    // Pedestrian crossing area (for future use)
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