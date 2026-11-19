use crate::grid::cell::CellID;
use crate::maneuver::LaneChangeType;
use std::fmt;

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

impl fmt::Display for VehicleIntention {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "VehicleIntention {{ intention_maneuver: {:?}, intention_speed: {}, destination: {:?}, confusion: {:?}, intention_cell_id: {}, tail_intention_cells: {:?}, intermediate_cells: {:?}, tail_maneuver: {:?}, should_stop: {} }}",
            self.intention_maneuver,
            self.intention_speed,
            self.destination,
            self.confusion,
            self.intention_cell_id,
            self.tail_intention_cells,
            self.intermediate_cells,
            self.tail_maneuver,
            self.should_stop
        )
    }
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
