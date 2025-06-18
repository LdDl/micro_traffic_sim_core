use crate::agents::{BehaviourType, VehicleID, VehicleRef};
use crate::conflict_zones::{ConflictWinnerType, ConflictZone, ConflictZoneID};
use crate::conflicts::resolve_simple_rules;
use crate::grid::cell::{Cell, CellID};
use crate::grid::road_network::GridRoads;
use crate::grid::lane_change_type::LaneChangeType;
use crate::intentions::{CellIntention, IntentionType, Intentions};
use crate::utils::rand::thread_rng;
use rand::Rng;

use std::collections::HashMap;
use std::{fmt, vec};

/// Different types of conflicts that can occur between vehicles
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
    /// Both vehicles are going to change lane but trajectories on one vehicle
    /// and other's tail are going to intersect and some.
    TailCrossLaneChange,
}

impl fmt::Display for ConflictType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let as_str = match self {
            ConflictType::Undefined => "undefined",
            ConflictType::ForwardLaneChange => "forward+lane_change",
            ConflictType::BlockLaneChange => "block+lane_change",
            ConflictType::MergeForward => "merge+forward",
            ConflictType::MergeLaneChange => "merge+lane_change",
            ConflictType::MergeForwardConflictZone => "merge+forward+conflict_zone",
            ConflictType::CrossLaneChange => "cross+lane_change",
            ConflictType::CrossConflictZone => "cross+conflict_zone",
            ConflictType::Tail => "tail",
            ConflictType::SelfTail => "self_tail",
            ConflictType::TailCrossLaneChange => "tail+cross+lane_change",
        };
        write!(f, "{}", as_str)
    }
}

/// Represents a conflict between vehicles
pub struct CellConflict {
    /// Cell ID where the conflict occurs
    pub cell_id: CellID,
    /// Agents that are involved in the conflict
    pub participants: Vec<VehicleRef>,
    /// Agent that has priority in the conflict
    pub priority_participant_index: usize,
    /// Type of the conflict
    pub conflict_type: ConflictType,
}

impl fmt::Display for CellConflict {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let participants: Vec<String> =
            self.participants.iter().map(|p| p.borrow().id.to_string()).collect();
        let priority_participant_id = self.participants
            .get(self.priority_participant_index)
            .map_or("None".to_string(), |p| p.borrow().id.to_string());
        let cell_id = self.cell_id.to_string();
        write!(
            f,
            "CellConflict{{CellID: {}, Type: {}, Participants: ({:?}), Priority participant: {}}}",
            cell_id, self.conflict_type, participants, priority_participant_id
        )
    }
}

#[derive(Debug)]
pub enum TrajectoryConflictError {
    CellNotFound(CellID),
    InvalidVehicle(String),
}

impl std::fmt::Display for TrajectoryConflictError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::CellNotFound(cell_id) => write!(f, "Cell {} not found in network", cell_id),
            Self::InvalidVehicle(msg) => write!(f, "Invalid vehicle: {}", msg),
        }
    }
}

/// Information about detected trajectory conflict (to avoid borrowing issues)
#[derive(Debug)]
pub struct TrajectoryConflictInfo {
    pub vehicle_id: u64,
    pub side_vehicle_id: u64,
    pub conflict_type: ConflictType,
    pub priority_vehicle_id: u64,
}

/// Find crossing trajectories conflicts (naive implementation)
/// Returns TrajectoryConflictInfo to avoid borrowing complexity
pub fn find_cross_trajectories_conflict_naive(
    cell_intention: &CellIntention,
    intention_cell: &Cell,
    collected_intentions: &Intentions,
    net: &GridRoads,
) -> Result<Option<TrajectoryConflictInfo>, TrajectoryConflictError> {
    /* No straight conflict, but could be crossing trajectories conflict */
    // A        B
    // |  L  R  |
    // |   \/   |
    // F   /\   F
    // |  /  \  |
    // X        Y

    // X, Y, A, B - positions
    // F - forward
    // L - left
    // R - right
    // Vehicle in position 'X'. Intention - position 'B'

    let vehicle = cell_intention.vehicle.borrow();

    // Check if vehicle maneuver is RIGHT or its tail is going to RIGHT
    let mut cell_x = vehicle.cell_id;
    let mut cell_b = intention_cell.get_id();
    let mut conflict_type = ConflictType::CrossLaneChange;

    // Handle tail maneuvers (equivalent to Go's tail logic)
    if !vehicle.tail_cells.is_empty() && 
       vehicle.intention.intention_maneuver != LaneChangeType::ChangeRight && 
       vehicle.intention.tail_maneuver.intention_maneuver != LaneChangeType::ChangeRight {
        return Ok(None);
    }
    
    if !vehicle.tail_cells.is_empty() && vehicle.intention.tail_maneuver.intention_maneuver == LaneChangeType::ChangeRight {
        cell_x = vehicle.intention.tail_maneuver.source_cell_maneuver;
        cell_b = vehicle.intention.tail_maneuver.target_cell_maneuver;
        conflict_type = ConflictType::TailCrossLaneChange;
    }
    
    if vehicle.tail_cells.is_empty() && vehicle.intention.intention_maneuver != LaneChangeType::ChangeRight {
        return Ok(None);
    }

    // Extract cell object by 'X' position
    let current_cell = net.get_cell(&cell_x)
        .ok_or(TrajectoryConflictError::CellNotFound(cell_x))?;

    // Check if position 'A' exists for 'X'
    if current_cell.get_forward_id() < 0 {
        // No forward cell at all for current vehicle's position
        return Ok(None);
    }

    // Get all intentions for position 'A' 
    let forward_intentions = collected_intentions.get(&current_cell.get_forward_id());
    if forward_intentions.is_none() || forward_intentions.unwrap().is_empty() {
        // No intentions for position 'A'
        return Ok(None);
    }

    // Scan any vehicle which is targeted for position 'A' and is doing LEFT maneuver or there is some tail
    let mut side_vehicle_left_ref: Option<VehicleRef> = None;
    let mut last_cell_intention = IntentionType::Target;
    
    for forward_intention in forward_intentions.unwrap() {
        let fwd_vehicle= forward_intention.vehicle.borrow();
        let ref_clone = forward_intention.vehicle.clone();
        if fwd_vehicle.intention.intention_maneuver == LaneChangeType::ChangeLeft || 
            forward_intention.int_type == IntentionType::Tail {
            last_cell_intention = forward_intention.int_type;
            side_vehicle_left_ref = Some(ref_clone);
            break;
        }
    }
    
    let side_vehicle_ref = match side_vehicle_left_ref {
        Some(vehicle_ref) => vehicle_ref,
        None => return Ok(None), // No conflict found
    };
    let side_vehicle = side_vehicle_ref.borrow();

    // Handle tail case
    if last_cell_intention == IntentionType::Tail {
        if side_vehicle.tail_cells.is_empty() {
            return Err(TrajectoryConflictError::InvalidVehicle(
                format!("Cell intention with 'INTENTION_TAIL' type has reference to vehicle which has no tail {}", side_vehicle.id)
            ));
        }

        // Check if any tail cell's forward node matches our target
        for &occ_cell_id in &side_vehicle.tail_cells {
            if occ_cell_id < 1 {
                continue;
            }
            
            let side_vehicle_cell = net.get_cell(&occ_cell_id)
                .ok_or(TrajectoryConflictError::CellNotFound(occ_cell_id))?;
            
            if side_vehicle_cell.get_forward_id() < 0 {
                continue;
            }
            
            if side_vehicle_cell.get_forward_id() == cell_b {
                // Found tail cross conflict - side vehicle has priority
                return Ok(Some(TrajectoryConflictInfo {
                    vehicle_id: vehicle.id,
                    side_vehicle_id: side_vehicle.id,
                    conflict_type: ConflictType::TailCrossLaneChange,
                    priority_vehicle_id: side_vehicle.id, // Side vehicle has priority in tail conflicts
                }));
            }
        }
        return Ok(None);
    }

    // Handle regular maneuver case
    // Extract cell object for side vehicle. It should be position 'Y'
    let side_vehicle_cell = net.get_cell(&side_vehicle.cell_id)
        .ok_or(TrajectoryConflictError::CellNotFound(side_vehicle.cell_id))?;

    // Check if forward cell for side vehicle exists. It should be position 'B'
    if side_vehicle_cell.get_forward_id() < 0 {
        return Ok(None);
    }

    // Check if position is 'B' == intention cell
    if side_vehicle_cell.get_forward_id() == cell_b {
        // Determine priority based on behavior types and maneuver types (matching Go logic)
        let priority_vehicle_id = if conflict_type == ConflictType::TailCrossLaneChange {
            // For tail conflicts, side vehicle has priority
            side_vehicle.id
        } else {
            // Check behavior types for priority (matching Go logic)
            if side_vehicle.strategy_type != vehicle.strategy_type && 
               vehicle.strategy_type == BehaviourType::Aggressive {
                // Aggressive vehicle has priority
                vehicle.id
            } else {
                // Left maneuver has priority over right maneuver (side_vehicle is doing LEFT)
                side_vehicle.id
            }
        };

        return Ok(Some(TrajectoryConflictInfo {
            vehicle_id: vehicle.id,
            side_vehicle_id: side_vehicle.id,
            conflict_type,
            priority_vehicle_id,
        }));
    }

    Ok(None)
}

/// Helper function to create actual CellConflict from TrajectoryConflictInfo
/// This would be called by the collect_conflicts function with proper mutable references
impl CellConflict {
    pub fn from_trajectory_conflict_info(
        info: TrajectoryConflictInfo,
        vehicle1: VehicleRef,
        vehicle2: VehicleRef,
    ) -> Self {
        // Determine participant order based on priority
        let (participants, priority_index) = if info.priority_vehicle_id == vehicle1.borrow().id {
            (vec![vehicle1, vehicle2], 0)
        } else {
            (vec![vehicle2, vehicle1], 0)
        };

        CellConflict {
            cell_id: -1, // No specific cell for trajectory conflicts
            participants,
            priority_participant_index: priority_index,
            conflict_type: info.conflict_type,
        }
    }
}

pub fn find_zone_conflict_for_two_intentions(
    intention_cell_id: CellID,
    conflict_zones: &HashMap<ConflictZoneID, ConflictZone>,
    cells_conflicts_zones: &HashMap<CellID, ConflictZoneID>,
) -> Option<CellID> {
    // Check if the intention cell is part of a conflict zone
    let conflict_zone_id = cells_conflicts_zones.get(&intention_cell_id)?;

    // Get the conflict zone
    let conflict_zone = conflict_zones.get(conflict_zone_id)?;

    let first_edge = conflict_zone.get_first_edge();
    let second_edge = conflict_zone.get_second_edge();

    // Check specific conflict conditions:
    // 1. First and second edges have different source cells
    // 2. Both edges target the same cell (the intention cell)
    // 3. If so, this is considered a conflict in a conflict zone
    // which overrides the normal conflict resolution (e.g. for ForwardLaneChange)
    if first_edge.source != second_edge.source
        && first_edge.target == second_edge.target
        && first_edge.target == intention_cell_id
    {
        // Determine winner based on conflict zone winner type
        match conflict_zone.get_winner_type() {
            ConflictWinnerType::First => return Some(first_edge.source),
            ConflictWinnerType::Second => return Some(second_edge.source),
            _ => {}
        }
        // Random selection (coin flip)
        let mut rng = thread_rng();
        if rng.gen_bool(0.5) {
            return Some(first_edge.source);
        }
        return Some(second_edge.source);
    }
    None // No conflict detected
}

/// Find conflict type between two intentions
/// Returns the winning intention and the conflict type
pub fn find_conflict_type<'a>(
    intention_cell_id: CellID,
    conflict_zones: &HashMap<ConflictZoneID, ConflictZone>,
    cells_conflicts_zones: &HashMap<CellID, ConflictZoneID>,
    intention_one: &'a CellIntention,
    intention_two: &'a CellIntention,
) -> (&'a CellIntention, ConflictType) {
    let intention_type_one = intention_one.int_type;
    let intention_type_two = intention_two.int_type;

    // Handle tail conflicts first
    if intention_type_one == IntentionType::Tail && intention_type_two == IntentionType::Transit {
        return (intention_one, ConflictType::Tail);
    }
    if intention_type_one == IntentionType::Transit && intention_type_two == IntentionType::Tail {
        return (intention_two, ConflictType::Tail);
    }
    if intention_type_one == IntentionType::Tail {
        return (intention_one, ConflictType::Tail);
    }
    if intention_type_two == IntentionType::Tail {
        return (intention_two, ConflictType::Tail);
    }
    let vehicle_one = intention_one.vehicle.borrow();
    let vehicle_two = intention_two.vehicle.borrow();

    // Check if there's a conflict zone for this cell
    let conflict_zone_winner_source_cell = find_zone_conflict_for_two_intentions(
        intention_cell_id, 
        conflict_zones, 
        cells_conflicts_zones
    );
    // The conflict zone is found - early return
    if let Some(winner_cell) = conflict_zone_winner_source_cell {
        if vehicle_one.cell_id == winner_cell {
            return (intention_one, ConflictType::MergeForwardConflictZone);
        }
        if vehicle_two.cell_id == winner_cell {
            return (intention_two, ConflictType::MergeForwardConflictZone);
        }
        // Check intermediate cells for vehicle one
        if !vehicle_one.intention.intermediate_cells.is_empty() {
            if let Some(elem_idx) = find_elem_in_slice(winner_cell, &vehicle_one.intention.intermediate_cells) {
                let trim_inter = &vehicle_one.intention.intermediate_cells[..=elem_idx];
                if let Some(&last_cell_id) = trim_inter.last() {
                    if last_cell_id == winner_cell {
                        return (intention_one, ConflictType::MergeForwardConflictZone);
                    }
                }
            }
        }
        // Check intermediate cells for vehicle two
        if !vehicle_two.intention.intermediate_cells.is_empty() {
            if let Some(elem_idx) = find_elem_in_slice(winner_cell, &vehicle_two.intention.intermediate_cells) {
                let trim_inter = &vehicle_two.intention.intermediate_cells[..=elem_idx];
                if let Some(&last_cell_id) = trim_inter.last() {
                    if last_cell_id == winner_cell {
                        return (intention_two, ConflictType::MergeForwardConflictZone);
                    }
                }
            }
        }
        // Log error and panic for unexpected cases
        eprintln!(
            "Can't happen in CONFLICT_TYPE_MERGE_CONFLICT_ZONE: winner_cell={}, vehicle_one={}, vehicle_two={}, pos_one={}, pos_two={}",
            winner_cell, vehicle_one.id, vehicle_two.id, vehicle_one.cell_id, vehicle_two.cell_id
        );
        panic!("Can't happen in CONFLICT_TYPE_MERGE_CONFLICT_ZONE");
    }

    // No conflict zone found, use simple rules
    resolve_simple_rules(intention_one, intention_two)
}

/// Helper function to find element index in slice
fn find_elem_in_slice(target: CellID, slice: &[CellID]) -> Option<usize> {
    slice.iter().position(|&x| x == target)
}

pub fn new_conflict_multiple(
    cell: &Cell,
    conflict_zones: &HashMap<ConflictZoneID, ConflictZone>,
    cells_conflicts_zones: &HashMap<CellID, ConflictZoneID>,
    cell_intentions: &[CellIntention],
) -> Result<CellConflict, TrajectoryConflictError> {
    if cell_intentions.len() < 2 {
        return Err(TrajectoryConflictError::InvalidVehicle(
            "Number of cell intentions is less than 2".to_string()
        ));
    }

    // Step 1: Deduplicate vehicles using map
    let mut participants_map: HashMap<VehicleID, VehicleRef> = HashMap::new();
    for intention in cell_intentions {
        let vehicle_id = intention.vehicle.borrow().id;
        participants_map.entry(vehicle_id).or_insert_with(|| intention.vehicle.clone());
    }

    if participants_map.len() < 2 {
        return Ok(CellConflict {
            cell_id: -1,
            participants: vec![],
            priority_participant_index: 0,
            conflict_type: ConflictType::SelfTail,
        });
    }

    // Step 2: Convert map to vector for indexed access
    let participants: Vec<VehicleRef> = participants_map.values().cloned().collect();

    // Step 3: Create conflict with temporary priority index
    let mut conflict = CellConflict {
        cell_id: cell.get_id(),
        participants,
        priority_participant_index: 0, // Will be determined below
        conflict_type: ConflictType::Undefined,
    };

    // Step 4: Determine priority participant and conflict type
    let mut priority_intention = &cell_intentions[0];
    let mut conflict_type = ConflictType::Undefined;
    
    for i in 1..cell_intentions.len() {
        let (winner, c_type) = find_conflict_type(
            cell.get_id(),
            conflict_zones,
            cells_conflicts_zones,
            priority_intention,
            &cell_intentions[i],
        );
        priority_intention = winner;
        conflict_type = c_type;
    }

    // Step 5: Find the index of the priority participant in the participants vector
    let priority_vehicle_id = priority_intention.vehicle.borrow().id;
    let mut priority_index = 0;
    for (i, participant) in conflict.participants.iter().enumerate() {
        if participant.borrow().id == priority_vehicle_id {
            priority_index = i;
            break;
        }
    }

    conflict.priority_participant_index = priority_index;
    conflict.conflict_type = conflict_type;
    
    Ok(conflict)
}

