use crate::agents::VehicleID;
use crate::grid::cell::CellID;
use std::fmt;

/// Describes the type of conflict detected between vehicles trying to move to the same cell or whose paths intersect.
///
/// Used to classify and resolve different traffic situations, such as merges, lane changes, tail blocking, and conflict zones.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConflictType {
    /// Just undefined conflict
    Undefined,
    /// One vehicle is going to move forward, another vehicle is going to change lane.
    ForwardLaneChange,
    /// One vehicle is going to change lane, another vehicle is simply stopped and can't move.
    BlockLaneChange,
    /// Both vehicles are going to move forward to the same cell without lane change.
    /// E.g. two roads are merging into one.
    MergeForward,
    /// Both vehicles are going to change lane to the same cell.
    /// It happens when three or more lane road becomes narrow in some point and merges in single lane.
    MergeLaneChange,
    /// Both vehicles are going to change lane to the same cell in conflict zone.
    MergeForwardConflictZone,
    /// Every contender must yield under the applicable merge-zone priorities.
    /// Its nominated source-cycle candidate may proceed after its own cycle delay.
    MutualYield,
    /// Both vehicles are going to change lane but theirs trajectories are going to intersect.
    CrossLaneChange,
    /// Both vehicles are going to change lane but theirs trajectories are going to intersect in conflict zone.
    CrossConflictZone,
    /// This type of the conflict could be caused by any other coflict.
    /// Any vehicle should wait until there is no other vehicle's tails ahead.
    Tail,
    /// This type of the conflict occurs when vehicle moving with speed > 1
    /// and both tail and transit intentions happen
    SelfTail,
    /// Physical crossing with a vehicle body, through lane changes or declared zone edges.
    /// The body owner has precedence regardless of patience or behavioral priority.
    TailCrossLaneChange,
}

impl fmt::Display for ConflictType {
    /// Formats the conflict type for display.
    /// 
    /// Returns a short, lowercase string representation suitable for
    /// logging, debugging, and user interfaces.
    /// 
    /// # Examples
    /// 
    /// ```rust
    /// use micro_traffic_sim_core::conflicts::ConflictType;
    ///
    /// assert_eq!(format!("{}", ConflictType::Undefined), "undefined");
    /// assert_eq!(format!("{}", ConflictType::ForwardLaneChange), "forward+lane_change");
    /// assert_eq!(format!("{}", ConflictType::BlockLaneChange), "block+lane_change");
    /// assert_eq!(format!("{}", ConflictType::MergeForward), "merge+forward");
    /// assert_eq!(format!("{}", ConflictType::MergeLaneChange), "merge+lane_change");
    /// assert_eq!(format!("{}", ConflictType::MergeForwardConflictZone), "merge+forward+conflict_zone");
    /// assert_eq!(format!("{}", ConflictType::CrossLaneChange), "cross+lane_change");
    /// assert_eq!(format!("{}", ConflictType::CrossConflictZone), "cross+conflict_zone");
    /// assert_eq!(format!("{}", ConflictType::Tail), "tail");
    /// assert_eq!(format!("{}", ConflictType::SelfTail), "self_tail");
    /// assert_eq!(format!("{}", ConflictType::TailCrossLaneChange), "tail+cross+lane_change");
    /// ```
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let as_str = match self {
            ConflictType::Undefined => "undefined",
            ConflictType::ForwardLaneChange => "forward+lane_change",
            ConflictType::BlockLaneChange => "block+lane_change",
            ConflictType::MergeForward => "merge+forward",
            ConflictType::MergeLaneChange => "merge+lane_change",
            ConflictType::MergeForwardConflictZone => "merge+forward+conflict_zone",
            ConflictType::MutualYield => "mutual_yield",
            ConflictType::CrossLaneChange => "cross+lane_change",
            ConflictType::CrossConflictZone => "cross+conflict_zone",
            ConflictType::Tail => "tail",
            ConflictType::SelfTail => "self_tail",
            ConflictType::TailCrossLaneChange => "tail+cross+lane_change",
        };
        write!(f, "{}", as_str)
    }
}

/// Represents a conflict between vehicles for a specific cell or trajectory.
///
/// Contains the cell where the conflict occurs, the involved vehicles, which vehicle has priority, and the conflict type.
/// Used for further resolution and movement decision.
#[derive(Debug)]
pub struct CellConflict {
    /// Cell ID where the conflict occurs
    pub cell_id: CellID,
    /// Agents that are involved in the conflict
    pub participants: Vec<VehicleID>,
    /// Priority participant, or the nominated cycle-break candidate for MutualYield.
    /// A cycle candidate must complete its own delay before receiving a grant.
    pub priority_participant_index: usize,
    /// Type of the conflict
    pub conflict_type: ConflictType,
}

impl fmt::Display for CellConflict {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let participants: Vec<String> =
            self.participants.iter().map(|p| p.to_string()).collect();
        let priority_participant_id = self.participants.get(self.priority_participant_index)
            .map_or("None".to_string(), |p| p.to_string());
        let priority_label = if self.conflict_type == ConflictType::MutualYield {
            "Cycle candidate"
        } else {
            "Priority participant"
        };
        let cell_id = self.cell_id.to_string();
        write!(
            f,
            "CellConflict{{CellID: {}, Type: {}, Participants: ({:?}), {}: {}}}",
            cell_id, self.conflict_type, participants, priority_label, priority_participant_id
        )
    }
}

/// Errors that can occur during conflict detection or resolution.
///
/// - `CellNotFound`: A referenced cell does not exist in the grid network.
/// - `InvalidVehicle`: A vehicle reference is invalid or inconsistent (e.g., missing tail).
#[derive(Debug, Clone)]
pub enum ConflictError {
    /// The specified cell does not exist in the grid.
    CellNotFound(CellID),
    /// The vehicle reference is invalid or inconsistent.
    InvalidVehicle(String),
}

impl std::fmt::Display for ConflictError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::CellNotFound(cell_id) => write!(f, "Cell {} not found in network", cell_id),
            Self::InvalidVehicle(msg) => write!(f, "Invalid vehicle: {}", msg),
        }
    }
}

/// Represents information about a detected trajectory conflict between two vehicles.
///
/// Used to avoid borrow checker issues when reporting crossing or tail conflicts.#[derive(Debug)]
pub struct TrajectoryConflictInfo {
    /// ID of the vehicle initiating the conflict
    pub vehicle_id: u64,
    /// ID of the vehicle on the side
    pub side_vehicle_id: u64,
    /// Type of the conflict detected
    pub conflict_type: ConflictType,
    /// ID of the vehicle that has priority in the conflict
    pub priority_vehicle_id: u64,
}

