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


fn find_zone_conflict_for_two_intentions(
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::conflict_zones::ConflictEdge;
    use crate::agents::{Vehicle, VehicleIntention};
    use crate::intentions;
    #[test]
    fn test_find_zone_conflict_for_two_intentions() {
        // Case 1: Both edges have the same target cell
        // Instead of basic conflict resolution, the conflict zone winner type should be used
        let mut conflict_zones: HashMap<ConflictZoneID, ConflictZone> = HashMap::new();
        let mut cells_conflicts_zones: HashMap<CellID, ConflictZoneID> = HashMap::new();
        let first_edge = ConflictEdge { source: 100, target: 200 };
        let second_edge = ConflictEdge { source: 111, target: 200 };
        let zone = ConflictZone::new(1, first_edge, second_edge)
            .with_winner_type(ConflictWinnerType::Second)
            .build();
        conflict_zones.insert(1, zone);
        cells_conflicts_zones.insert(200, 1);
        let correct_winner = 111;
        let winner = find_zone_conflict_for_two_intentions(200, &conflict_zones, &cells_conflicts_zones);
        assert_eq!(winner, Some(correct_winner), "Conflict winner is incorrect");
    }

    #[test]
    fn test_find_cross_trajectories_no_right_maneuver() {
        // Case: Vehicle at cell with no right maneuver should return None
        let mut vehicle = Vehicle::new(1)
            .with_cell(10)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeLeft,
            intention_cell_id: 11,
            ..Default::default()
        });
        let cell_intention = CellIntention::new(vehicle, IntentionType::Target);
        let intention_cell = Cell::new(15).build();
        let collected_intentions = Intentions::new();
        let grid = GridRoads::new();
        let result = find_cross_trajectories_conflict_naive(
            &cell_intention, &intention_cell, &collected_intentions, &grid
        );
        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
    }

    #[test]
    fn test_find_cross_trajectories_no_forward_cell() {
        // Case: Vehicle at cell with no forward node should return None
        let mut vehicle = Vehicle::new(1)
            .with_cell(10)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            intention_cell_id: 15,
            ..Default::default()
        });
        let cell_intention = CellIntention::new(vehicle, IntentionType::Target);
        let intention_cell = Cell::new(15).build();
        let collected_intentions = Intentions::new();
        let mut grid = GridRoads::new();
        let cell = Cell::new(10)
            .with_forward_node(-1)  // No forward cell
            .build();
        grid.add_cell(cell.clone());
        let result = find_cross_trajectories_conflict_naive(
            &cell_intention, &intention_cell, &collected_intentions, &grid
        );
        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
    }

    #[test]
    fn test_find_cross_trajectories_no_conflict_same_direction() {
        // Test case where both vehicles go in same direction - no conflict
        let vehicle1 = Vehicle::new(1)
            .with_cell(4)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle1.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            intention_cell_id: 2,
            intention_speed: 2,
            ..Default::default()
        });

        let vehicle2 = Vehicle::new(2)
            .with_cell(1)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle2.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight, // Same direction - no crossing
            intention_cell_id: 5,
            intention_speed: 2,
            ..Default::default()
        });

        let mut net = GridRoads::new();
        net.add_cell(Cell::new(4).with_forward_node(1).build());
        net.add_cell(Cell::new(1).with_forward_node(5).build());
        net.add_cell(Cell::new(2).build());
        net.add_cell(Cell::new(5).build());

        let mut collected_intentions = Intentions::new();
        collected_intentions.add_intention(vehicle2, IntentionType::Target);

        let vehicle1_intention = CellIntention::new(vehicle1, IntentionType::Target);
        let intention_cell = Cell::new(2).build();

        let result = find_cross_trajectories_conflict_naive(
            &vehicle1_intention, &intention_cell, &collected_intentions, &net
        );

        assert!(result.is_ok());
        let conflict_info = result.unwrap();
        assert!(conflict_info.is_none(), "Should not detect conflict when both vehicles go same direction");
    }

    #[test]
    fn test_find_cross_trajectories_conflict_naive() {
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

        // 1 - X
        // 2 - A
        // 3 - Y
        // 4 - B
        let mut net = GridRoads::new();
        net.add_cell(Cell::new(1).with_forward_node(2).with_right_node(4).build());
        net.add_cell(Cell::new(2).build());
        net.add_cell(Cell::new(3).with_forward_node(4).with_left_node(1).build());
        net.add_cell(Cell::new(4).build());

        // Vehicle 1: At position X (cell 1), wants to go RIGHT to B (cell 4)
        let vehicle1 = Vehicle::new(1)
            .with_cell(1)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle1.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            intention_cell_id: 4,
            ..Default::default()
        });

        // Vehicle 2: At position Y (cell 3), wants to go LEFT to A (cell 2)
        let vehicle2 = Vehicle::new(2)
            .with_cell(3)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle2.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeLeft,
            intention_cell_id: 2,
            ..Default::default()
        });

        // Set up intentions - vehicle2 wants cell 2
        let mut collected_intentions = Intentions::new();
        collected_intentions.add_intention(vehicle2, IntentionType::Target);

        // Test vehicle1's intention to go to cell 4
        let vehicle1_intention = CellIntention::new(vehicle1, IntentionType::Target);
        let intention_cell = net.get_cell(&4).unwrap(); // Cell B

        let result = find_cross_trajectories_conflict_naive(
            &vehicle1_intention, intention_cell, &collected_intentions, &net
        );

        // Assertions
        assert!(result.is_ok(), "Should not return error");
        let conflict_info = result.unwrap();
        assert!(conflict_info.is_some(), "Should detect crossing trajectories conflict");
        
        let info = conflict_info.unwrap();
        assert_eq!(info.conflict_type, ConflictType::CrossLaneChange, "Should be cross lane change conflict");
        
        // // Check that both vehicles are in the conflict
        assert_eq!(info.vehicle_id, 1, "Vehicle 1 should be primary vehicle");
        assert_eq!(info.side_vehicle_id, 2, "Vehicle 2 should be side vehicle");
        
        // Vehicle 2 (left maneuver) should have priority over vehicle 1 (right maneuver)
        assert_eq!(info.priority_vehicle_id, 2, "Left maneuver should have priority over right maneuver");
    }
    
    #[test]
    fn test_find_conflict_type() {
        // Test 1: Tail vs Transit conflict - Tail should win
        let vehicle_one = Vehicle::new(1)
            .with_cell(10)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle_one.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 15,
            ..Default::default()
        });

        let vehicle_two = Vehicle::new(2)
            .with_cell(11)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle_two.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 15,
            ..Default::default()
        });
        let correct_id = vehicle_one.borrow().id;
        let intention_one = CellIntention::new(vehicle_one, IntentionType::Tail);
        let intention_two = CellIntention::new(vehicle_two, IntentionType::Transit);

        let (winner, conflict_type) = find_conflict_type(
            15,
            &HashMap::new(),
            &HashMap::new(),
            &intention_one,
            &intention_two
        );

        assert_eq!(winner.vehicle.borrow_mut().id, correct_id, "Tail intention should win over transit");
        assert_eq!(conflict_type, ConflictType::Tail, "Conflict type should be Tail");

        // Test 2: Conflict zone winner - First edge source should win
        let mut conflict_zones = HashMap::new();
        let mut cells_conflicts_zones = HashMap::new();

        let first_edge = ConflictEdge { source: 10, target: 15 };
        let second_edge = ConflictEdge { source: 11, target: 15 };
        let zone = ConflictZone::new(1, first_edge, second_edge)
            .with_winner_type(ConflictWinnerType::First)
            .build();
        conflict_zones.insert(1, zone);
        cells_conflicts_zones.insert(15, 1);

        let vehicle_three = Vehicle::new(3)
            .with_cell(10)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle_three.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 15,
            ..Default::default()
        });

        let vehicle_four = Vehicle::new(4)
            .with_cell(11)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle_four.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 15,
            ..Default::default()
        });

        let correct_id = vehicle_three.borrow().id;
        let intention_three = CellIntention::new(vehicle_three, IntentionType::Target);
        let intention_four = CellIntention::new(vehicle_four, IntentionType::Target);

        let (winner, conflict_type) = find_conflict_type(
            15,
            &conflict_zones,
            &cells_conflicts_zones,
            &intention_three,
            &intention_four
        );

        assert_eq!(winner.vehicle.borrow().id, correct_id, "First edge source vehicle should win");
        assert_eq!(conflict_type, ConflictType::MergeForwardConflictZone, "Conflict type should be MergeForwardConflictZone");

        // Test 3: Merge lane change - Left maneuver should have priority over right
        let vehicle_five = Vehicle::new(5)
            .with_cell(20)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle_five.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeLeft,
            intention_cell_id: 25,
            ..Default::default()
        });

        let vehicle_six = Vehicle::new(6)
            .with_cell(21)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle_six.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            intention_cell_id: 25,
            ..Default::default()
        });

        let correct_id = vehicle_five.borrow().id;
        let intention_five = CellIntention::new(vehicle_five, IntentionType::Target);
        let intention_six = CellIntention::new(vehicle_six, IntentionType::Target);

        let (winner, conflict_type) = find_conflict_type(
            25,
            &HashMap::new(),
            &HashMap::new(),
            &intention_five,
            &intention_six
        );

        assert_eq!(winner.vehicle.borrow().id, correct_id, "Left maneuver should have priority over right");
        assert_eq!(conflict_type, ConflictType::MergeLaneChange, "Conflict type should be MergeLaneChange");

        // Test 4: Aggressive vs Cooperative behavior
        let vehicle_seven = Vehicle::new(7)
            .with_cell(30)
            .with_behaviour(BehaviourType::Aggressive)
            .with_speed(3)
            .build_ref();
        vehicle_seven.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 35,
            ..Default::default()
        });

        let vehicle_eight = Vehicle::new(8)
            .with_cell(31)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle_eight.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 35,
            ..Default::default()
        });

        let correct_id = vehicle_seven.borrow().id;
        let intention_seven = CellIntention::new(vehicle_seven, IntentionType::Target);
        let intention_eight = CellIntention::new(vehicle_eight, IntentionType::Target);

        let (winner, conflict_type) = find_conflict_type(
            35,
            &HashMap::new(),
            &HashMap::new(),
            &intention_seven,
            &intention_eight
        );

        assert_eq!(winner.vehicle.borrow().id, correct_id, "Aggressive vehicle should win over cooperative");
        assert_eq!(conflict_type, ConflictType::MergeForward, "Conflict type should be MergeForward");
    }
}
