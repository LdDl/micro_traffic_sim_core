use crate::conflicts::conflicts::*;
use crate::agents::{Vehicle, VehicleID};
use crate::grid::lane_change_type::LaneChangeType;
use crate::grid::cell::CellID;
use crate::verbose::*;

use std::fmt;

#[derive(Debug, Clone)]
pub enum ConflictSolverError {
    InvalidConflict(String),
    InsufficientParticipants(CellID, String),
    InvalidPriorityIndex(CellID, usize, usize),

}

impl fmt::Display for ConflictSolverError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConflict(msg) => write!(f, "Invalid conflict: {}", msg),
            Self::InsufficientParticipants(cell_id, conflict_type) => {
                write!(f, "Insufficient participants in conflict for cell {}, type: {}", cell_id, conflict_type)
            },
            Self::InvalidPriorityIndex(cell_id, index, len) => {
                write!(f, "Invalid priority index {} for conflict at cell {} (participants: {})", index, cell_id, len)
            }
        }
    }
}

/// Solves conflicts between vehicles based on their intentions and priority
///
/// # Arguments
/// * `conflicts_data` - A vector of `CellConflict` containing conflicts to be resolved
/// * `verbose` - If true, prints detailed information about the conflicts being resolved
/// 
/// # Returns
/// * `Result<(), ConflictSolverError>` - Returns Ok if conflicts are resolved successfully, or an error if there are issues
/// 
/// # Example
/// ```
/// use micro_traffic_sim_core::conflicts::{CellConflict, ConflictType, solve_conflicts};
/// use micro_traffic_sim_core::agents::{Vehicle, VehicleRef};
/// use micro_traffic_sim_core::grid::lane_change_type::LaneChangeType;
/// use micro_traffic_sim_core::grid::cell::CellID;
/// use micro_traffic_sim_core::verbose::VerboseLevel;
/// use std::rc::Rc;
/// use std::cell::RefCell;
/// let mut vehicle1 = Vehicle::new(1).with_speed(2).build();
/// let mut vehicle2 = Vehicle::new(2).with_speed(3).build();
/// let vehicles: Vec<VehicleRef> = vec![Rc::new(RefCell::new(vehicle1)), Rc::new(RefCell::new(vehicle2))];
/// let conflicts = vec![CellConflict {
///    cell_id: 1,
///    participants: vehicles,
///    priority_participant_index: 0,
///   conflict_type: ConflictType::ForwardLaneChange,
/// }];
/// let result = solve_conflicts(conflicts, VerboseLevel::None);
/// assert!(result.is_ok(), "Conflict resolution failed");
pub fn solve_conflicts<'b>(
    conflicts_data: Vec<CellConflict>,
    verbose: VerboseLevel,
) -> Result<(), ConflictSolverError> {
    if verbose.is_at_least(VerboseLevel::Main) {
        verbose.log_with_fields(
            EVENT_CONFLICTS_SOLVE,
            "Solve conflicts",
            &[("conflicts_num", &conflicts_data.len())]
        );
    }
    let mut trajectories_conflicts: Vec<CellConflict> = Vec::new();
    let mut conflict_zones_conflicts: Vec<CellConflict> = Vec::new();
    for mut conflict in conflicts_data {
        if conflict.participants.len() < 2 {
            return Err(ConflictSolverError::InsufficientParticipants(
                conflict.cell_id,
                format!("{:?}", conflict.conflict_type),
            ));
        }

        // Validate priority participant index
        if conflict.priority_participant_index >= conflict.participants.len() {
            return Err(ConflictSolverError::InvalidPriorityIndex(
                conflict.cell_id,
                conflict.priority_participant_index,
                conflict.participants.len(),
            ));
        }

        match conflict.conflict_type {
            ConflictType::SelfTail => {
                if verbose.is_at_least(VerboseLevel::Additional) {
                    verbose.log_with_fields(
                        EVENT_CONFLICT_SOLVE,
                        "Conflict - self tail",
                        &[
                            ("cell", &conflict.cell_id),
                            ("conflict_type", &format!("{:?}", conflict.conflict_type)),
                            ("participants", &format!("{:?}", conflict.participants.iter().map(|p| p.borrow().id).collect::<Vec<_>>())),
                        ]
                    );
                }
                continue;
            }
            ConflictType::CrossLaneChange | ConflictType::TailCrossLaneChange => {
                // This must be crossing trajectories conflict. Need to postprocess after
                if let Some(priority_participant_ref) = conflict.participants.get(conflict.priority_participant_index) {
                    let mut priority_participant = priority_participant_ref.borrow_mut();
                    priority_participant.is_conflict_participant = false;
                }
                if verbose.is_at_least(VerboseLevel::Additional) {
                    verbose.log_with_fields(
                        EVENT_CONFLICT_SOLVE,
                        "Found trajectories cross conflict",
                        &[
                            ("cell", &conflict.cell_id),
                            ("conflict_type", &format!("{:?}", conflict.conflict_type)),
                            ("participants", &format!("{:?}", conflict.participants.iter().map(|p| p.borrow().id).collect::<Vec<_>>())),
                            ("priority_participant_index", &conflict.priority_participant_index),
                        ]
                    );
                }
                trajectories_conflicts.push(conflict);
                continue;
            }
            ConflictType::CrossConflictZone => {
                if let Some(priority_participant_ref) = conflict.participants.get(conflict.priority_participant_index) {
                    let mut priority_participant = priority_participant_ref.borrow_mut();
                    priority_participant.is_conflict_participant = false;
                }
                if verbose.is_at_least(VerboseLevel::Additional) {
                    verbose.log_with_fields(
                        EVENT_CONFLICT_SOLVE,
                        "Found trajectories cross conflict in zone",
                        &[
                            ("cell", &conflict.cell_id),
                            ("conflict_type", &format!("{:?}", conflict.conflict_type)),
                            ("participants", &format!("{:?}", conflict.participants.iter().map(|p| p.borrow().id).collect::<Vec<_>>())),
                            ("priority_participant_index", &conflict.priority_participant_index),
                        ]
                    );
                }
                conflict_zones_conflicts.push(conflict);
                continue;
            }
            _ => {}           
        }


        if verbose.is_at_least(VerboseLevel::Additional) {
            verbose.log_with_fields(
                EVENT_CONFLICT_SOLVE,
                "Solving common conflicts",
                &[
                    ("cell", &conflict.cell_id),
                    ("conflict_type", &format!("{:?}", conflict.conflict_type)),
                    ("participants", &format!("{:?}", conflict.participants.iter().map(|p| p.borrow().id).collect::<Vec<_>>())),
                    ("priority_index", &conflict.priority_participant_index),
                ]
            );
        }

        let priority_index = conflict.priority_participant_index;
        for (i, participant_ref) in conflict.participants.iter_mut().enumerate() {
            let mut participant = participant_ref.borrow_mut();
            if i == priority_index {
                // Movement to cell is allowed
                // intention_cell remains the same (especially for vehicles for which conflict Cell ID is just intermediate cell)
                // intention_cell could be changed only if maneuver has been met
                if participant.intention.intention_maneuver != LaneChangeType::NoChange {
                    participant.intention.intention_cell_id = conflict.cell_id;
                    participant.intention.intention_speed = 1;
                }
            } else {
                // Stay still
                participant.block_with_speed(0);
            }
        }
    }
    
    // Postprocessing remaining possible trajectories conflicts
    for mut conflict in trajectories_conflicts {
        if verbose.is_at_least(VerboseLevel::Additional) {
            let vehicles: Vec<_> = conflict.participants.iter().map(|p| p.borrow().id).collect();
            verbose.log_with_fields(
                EVENT_CONFLICT_SOLVE,
                "Solve trajectories cross conflict",
                &[
                    ("cell", &conflict.cell_id),
                    ("participants_ids", &format!("{:?}", vehicles)),
                    ("conflict_type", &format!("{:?}", conflict.conflict_type)),
                ]
            );
        }
        match conflict.conflict_type {
            ConflictType::TailCrossLaneChange => {
                let priority_index = conflict.priority_participant_index;
                for (i, participant_ref) in conflict.participants.iter_mut().enumerate() {
                    let mut participant = participant_ref.borrow_mut();
                    if i != priority_index {
                        participant.block_with_speed(0);
                    }
                }
                continue;
            }
            _ => {}
        }
        // Solve plain trajectories conflict
        let mut need_to_solve = true;
        let mut left_maneuver_index: Option<usize> = None;
        let mut right_maneuver_index: Option<usize> = None;
        for (i, participant_ref) in conflict.participants.iter().enumerate() {
            let participant = participant_ref.borrow();
            match participant.intention.intention_maneuver {
                LaneChangeType::ChangeLeft => left_maneuver_index = Some(i),
                LaneChangeType::ChangeRight => right_maneuver_index = Some(i),
                LaneChangeType::Block => need_to_solve = false,
                _ => {}
            }
        }
        if need_to_solve && left_maneuver_index.is_some() && right_maneuver_index.is_some() {
            // Vehicle which is trying to do maneuver to the right should stop
            if let Some(right_index) = right_maneuver_index {
                conflict.participants[right_index].borrow_mut().block_with_speed(0);
            }
            // Vehicle which is trying to do maneuver to the left is allowed to do so
            // IntentionManeuver is saved
            // intention_cell is saved
            // Explicitly make speed to be equal 1 (just in case)
            if let Some(left_index) = left_maneuver_index {
                conflict.participants[left_index].borrow_mut().intention.intention_speed = 1;
            }
        }
    }

    for mut conflict in conflict_zones_conflicts {
        if verbose.is_at_least(VerboseLevel::Additional) {
            let vehicles: Vec<_> = conflict.participants.iter().map(|p| p.borrow().id).collect();
            verbose.log_with_fields(
                EVENT_CONFLICT_SOLVE,
                "Solve trajectories cross conflict in zone",
                &[
                    ("cell", &conflict.cell_id),
                    ("participants_ids", &format!("{:?}", vehicles)),
                    ("conflict_type", &format!("{:?}", conflict.conflict_type)),
                ]
            );
        }
        
        let priority_index = conflict.priority_participant_index;
        for (i, participant) in conflict.participants.iter_mut().enumerate() {
            if i != priority_index {
                participant.borrow_mut().block_with_speed(1);
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agents::{Vehicle, VehicleRef, VehicleIntention, BehaviourType};
    use crate::grid::lane_change_type::LaneChangeType;
    use crate::grid::cell::CellID;
    use crate::conflicts::conflicts::{CellConflict, ConflictType};

    fn create_test_vehicle(
        id: u64,
        speed: i32,
        maneuver: LaneChangeType,
        intention_cell: CellID,
    ) -> Vehicle {
        let mut vehicle = Vehicle::new(id)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(speed)
            .build();
        
        vehicle.set_intention(VehicleIntention {
            intention_maneuver: maneuver,
            intention_cell_id: intention_cell,
            intention_speed: speed,
            ..Default::default()
        });
        
        vehicle
    }

    fn create_test_vehicle_ref(
        id: u64,
        speed: i32,
        maneuver: LaneChangeType,
        intention_cell: CellID,
    ) -> VehicleRef {
        let vehicle = Vehicle::new(id)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(speed)
            .build_ref();
        
        vehicle.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: maneuver,
            intention_cell_id: intention_cell,
            intention_speed: speed,
            ..Default::default()
        });
        
        vehicle
    }

    #[test]
    fn test_solve_conflicts_empty_conflicts_list() {
        let conflicts: Vec<CellConflict> = vec![];
        let result = solve_conflicts(conflicts, VerboseLevel::None);
        assert!(result.is_ok(), "solve_conflicts should handle empty conflicts list");
    }

    #[test]
    fn test_solve_conflicts_insufficient_participants() {
        let vehicle_ref = create_test_vehicle_ref(1,2, LaneChangeType::NoChange, 5);

        let conflicts = vec![CellConflict {
            cell_id: 5,
            participants: vec![vehicle_ref],
            priority_participant_index: 0,
            conflict_type: ConflictType::MergeForward,
        }];

        let result = solve_conflicts(conflicts,VerboseLevel::None);
        assert!(result.is_err(), "solve_conflicts should return error for conflicts with insufficient participants");
        
        match result {
            Err(ConflictSolverError::InsufficientParticipants(cell_id, _)) => {
                assert_eq!(cell_id, 5, "Error should contain correct cell ID");
            }
            _ => panic!("Expected InsufficientParticipants error"),
        }
    }

    #[test]
    fn test_solve_conflicts_invalid_priority_index() {
        let vehicle1 = create_test_vehicle_ref(1, 2, LaneChangeType::NoChange, 5);
        let vehicle2 = create_test_vehicle_ref(2, 2, LaneChangeType::NoChange, 5);
        
        let conflicts = vec![CellConflict {
            cell_id: 5,
            participants: vec![vehicle1, vehicle2],
            priority_participant_index: 5, // Invalid index (only 0 and 1 are valid)
            conflict_type: ConflictType::MergeForward,
        }];

        let result = solve_conflicts(conflicts, VerboseLevel::None);
        assert!(result.is_err(), "solve_conflicts should return error for invalid priority index");
        
        match result {
            Err(ConflictSolverError::InvalidPriorityIndex(cell_id, index, len)) => {
                assert_eq!(cell_id, 5);
                assert_eq!(index, 5);
                assert_eq!(len, 2);
            }
            _ => panic!("Expected InvalidPriorityIndex error"),
        }
    }

    #[test]
    fn test_solve_conflicts_common_conflict() {
        let priority_vehicle = create_test_vehicle_ref(1, 3, LaneChangeType::ChangeLeft, 5);
        let blocked_vehicle = create_test_vehicle_ref(2, 2, LaneChangeType::NoChange, 5);
        
        let conflicts = vec![CellConflict {
            cell_id: 5,
            participants: vec![priority_vehicle.clone(), blocked_vehicle.clone()],
            priority_participant_index: 0, // First vehicle (index 0) has priority
            conflict_type: ConflictType::ForwardLaneChange,
        }];

        let result = solve_conflicts(conflicts, VerboseLevel::None);
        assert!(result.is_ok(), "solve_conflicts should handle common conflicts");
        
        // After solving, we can check the vehicle states
        assert_eq!(priority_vehicle.borrow().intention.intention_cell_id, 5, "Priority vehicle should have target cell");
        assert_eq!(priority_vehicle.borrow().intention.intention_speed, 1, "Priority vehicle should have speed 1");
        assert_eq!(blocked_vehicle.borrow().intention.intention_speed, 0, "Blocked vehicle should have speed 0");
    }

    #[test]
    fn test_solve_conflicts_trajectories_conflict() {
        let left_vehicle = create_test_vehicle_ref(1, 2, LaneChangeType::ChangeLeft, 5);
        let right_vehicle = create_test_vehicle_ref(2, 2, LaneChangeType::ChangeRight, 6);
        
        let conflicts = vec![CellConflict {
            cell_id: -1, // Trajectories conflict has no specific cell
            participants: vec![left_vehicle.clone(), right_vehicle.clone()],
            priority_participant_index: 0, // Left vehicle has priority
            conflict_type: ConflictType::CrossLaneChange,
        }];

        let result = solve_conflicts(conflicts, VerboseLevel::None);
        assert!(result.is_ok(), "solve_conflicts should handle trajectories conflicts");
        
        // Check that right vehicle is blocked (right maneuver loses to left maneuver)
        assert_eq!(right_vehicle.borrow().intention.intention_speed, 0, "Right vehicle should be blocked");
        assert_eq!(left_vehicle.borrow().intention.intention_speed, 1, "Left vehicle should continue");
    }

    #[test]
    fn test_solve_conflicts_conflict_zone() {
        let priority_vehicle = create_test_vehicle_ref(1,2, LaneChangeType::NoChange, 5);
        let blocked_vehicle = create_test_vehicle_ref(2, 2, LaneChangeType::NoChange, 6);
        
        let conflicts = vec![CellConflict {
            cell_id: 10,
            participants: vec![priority_vehicle.clone(), blocked_vehicle.clone()],
            priority_participant_index: 0, // First vehicle has priority
            conflict_type: ConflictType::CrossConflictZone,
        }];

        let result = solve_conflicts(conflicts, VerboseLevel::None);
        assert!(result.is_ok(), "solve_conflicts should handle conflict zone conflicts");
        
        // In conflict zones, blocked vehicles get speed 1 (not 0)
        assert_eq!(blocked_vehicle.borrow().intention.intention_speed, 1, "Blocked vehicle in conflict zone should have speed 1");
    }

    #[test]
    fn test_solve_conflicts_multiple_conflicts() {
        let vehicle1 = create_test_vehicle_ref(1, 2, LaneChangeType::ChangeLeft, 5);
        let vehicle2 = create_test_vehicle_ref(2, 2, LaneChangeType::NoChange, 5);
        let vehicle3 = create_test_vehicle_ref(3, 1, LaneChangeType::ChangeRight, 7);
        let vehicle4 = create_test_vehicle_ref(4, 1, LaneChangeType::NoChange, 7);
        
        let conflicts = vec![
            CellConflict {
                cell_id: 5,
                participants: vec![vehicle1.clone(), vehicle2.clone()],
                priority_participant_index: 0, // Vehicle 1 has priority
                conflict_type: ConflictType::ForwardLaneChange,
            },
            CellConflict {
                cell_id: 7,
                participants: vec![vehicle3.clone(), vehicle4.clone()],
                priority_participant_index: 1, // Vehicle 4 has priority (index 1)
                conflict_type: ConflictType::ForwardLaneChange,
            },
        ];

        let result = solve_conflicts(conflicts, VerboseLevel::None);
        assert!(result.is_ok(), "solve_conflicts should handle multiple conflicts");
        
        // Check first conflict resolution
        assert_eq!(vehicle1.borrow().intention.intention_cell_id, 5, "Vehicle 1 should get target cell");
        assert_eq!(vehicle1.borrow().intention.intention_speed, 1, "Vehicle 1 should have speed 1");
        assert_eq!(vehicle2.borrow().intention.intention_speed, 0, "Vehicle 2 should be blocked");
        
        // Check second conflict resolution
        assert_eq!(vehicle3.borrow().intention.intention_speed, 0, "Vehicle 3 should be blocked");
        assert_eq!(vehicle4.borrow().intention.intention_speed, 1, "Vehicle 4 should keep its original speed (has priority, NoChange)");
    }
}