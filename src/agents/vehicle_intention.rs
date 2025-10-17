use crate::grid::cell::CellID;
use crate::grid::lane_change_type::LaneChangeType;

/// Represents vehicle's intention to perform maneuver and other actions
#[derive(Debug, Default, Clone, PartialEq)]
pub struct VehicleIntention {
    /// Flag to repesent vehicle intentions to perform maneuver. See the ref. at `LaneChangeType`
    /// If the vehicle makes a maneuver, then it has no explicit advantage in possible conflicts.
    pub intention_maneuver: LaneChangeType,
    /// Possible speed.
    pub intention_speed: i32,
    /// Final cell for the vehicle's trip.
    pub destination: Option<CellID>,
    /// @todo: for further research and development needs.
    pub confusion: Option<bool>,
    /// Cell which vehicle wants to occupy.
    pub intention_cell_id: CellID,
    /// Cells which vehicle's tail wants to occupy (in case when vehicle has size more that one cell).
    pub tail_intention_cells: Vec<CellID>,
    /// Intention occupied cells in case when vehicle moving with speed more than one cell per time unit.
    pub intermediate_cells: Vec<CellID>,
    /// Tail's intention. See the ref. at `IntentionTailManeuever`.
    pub tail_maneuver: TailIntentionManeuver,
    /// Flag to stop vehicle. @todo: move it to intention?
    pub should_stop: bool,
}

/// Represents vehicle's tail intention
#[derive(Debug, Clone, PartialEq)]
pub struct TailIntentionManeuver {
    /// This field is for establishing source cell of maneuver for the vehicle's tail (in case when vehicle has size more that one cell)
    pub source_cell_maneuver: CellID,
    /// This field is for establishing target cell of maneuver for the vehicle's tail (in case when vehicle has size more that one cell)
    pub target_cell_maneuver: CellID,
    /// Flag to repesent vehicle's tail intentions to perform maneuver (in case when vehicle has size more that one cell). See the ref. at `LaneChangeType`
    pub intention_maneuver: LaneChangeType,
}

impl Default for TailIntentionManeuver {
    fn default() -> TailIntentionManeuver {
        TailIntentionManeuver {
            source_cell_maneuver: -1,
            target_cell_maneuver: -1,
            intention_maneuver: LaneChangeType::Undefined,
        }
    }
}
