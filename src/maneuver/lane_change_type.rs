use std::fmt;

/// Represents the type of lane change maneuvers available to vehicles in traffic simulation.
///
/// This enum defines all possible movement decisions a vehicle can make during a single
/// simulation time step in a cellular automata traffic model.
#[derive(Default, Debug, Clone, Copy, Eq, PartialEq)]
pub enum LaneChangeType {
    #[default]
    /// **Default/Uninitialized state**
    /// 
    /// Indicates that no maneuver decision has been made yet, or the maneuver
    /// type is unknown. This is the default value when creating new instances.
    Undefined = 0,
    /// **Continue straight in current lane**
    /// 
    /// Vehicle maintains its current lane position and moves forward to the next
    /// cell in the same lane.
    NoChange,
    /// **Change to the right lane**
    /// 
    /// Vehicle attempts to move from its current lane to the adjacent right lane.
    ChangeRight,
    /// **Change to the left lane**
    /// 
    /// Vehicle attempts to move from its current lane to the adjacent left lane.
    ChangeLeft,
    /// **Block current position (stay stationary)**
    /// 
    /// Vehicle remains in its current cell without advancing. This occurs when
    /// forward movement is impossible due to traffic conditions.
    Block,
}

impl fmt::Display for LaneChangeType {
    /// Formats the lane change type for display.
    /// 
    /// Returns a short, lowercase string representation suitable for
    /// logging, debugging, and user interfaces.
    /// 
    /// # Examples
    /// 
    /// ```rust
    /// use micro_traffic_sim_core::maneuver::LaneChangeType;
    /// 
    /// assert_eq!(format!("{}", LaneChangeType::NoChange), "no");
    /// assert_eq!(format!("{}", LaneChangeType::ChangeRight), "right");
    /// assert_eq!(format!("{}", LaneChangeType::ChangeLeft), "left");
    /// assert_eq!(format!("{}", LaneChangeType::Block), "block");
    /// assert_eq!(format!("{}", LaneChangeType::Undefined), "undefined");
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
