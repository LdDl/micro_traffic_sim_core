use crate::conflicts::conflict_types::{CellConflict, ConflictType};
use crate::agents::{Vehicle, VehicleID};
use indexmap::IndexMap;
use crate::maneuver::LaneChangeType;
use crate::grid::cell::CellID;
use crate::verbose::*;
use std::collections::{HashSet, HashMap};
use std::cmp::Reverse;
use std::iter::once;

use std::fmt;

/// Error type for conflict resolution failures.
#[derive(Debug, Clone)]
pub enum ConflictSolverError {
    /// The conflict data is inconsistent or invalid.
    InvalidConflict(String),
    /// Not enough participants in the conflict to resolve it.
    InsufficientParticipants(CellID, String),
    /// The priority participant index is invalid.
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

/// Whether ordinary patience can nominate a participant in this conflict.
/// Physical bodies and agents participating in merge cycles are excluded.
fn conflict_has_desperate(
    c: &CellConflict,
    vehicles: &IndexMap<VehicleID, Vehicle>,
    cycle_participants: &HashSet<VehicleID>,
) -> bool {
    let is_physical = matches!(
        c.conflict_type,
        ConflictType::Tail | ConflictType::SelfTail | ConflictType::TailCrossLaneChange
    );
    !is_physical
        && c.conflict_type != ConflictType::MutualYield
        && c.participants
            .iter()
            .any(|id| vehicles.get(id).is_some_and(|v| {
                v.is_desperate() && !cycle_participants.contains(id)
            }))
}

/// Solves conflicts between vehicles based on their intentions and priority for the current simulation step.
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
/// use micro_traffic_sim_core::agents::{Vehicle, VehicleID, VehiclesStorage};
/// use micro_traffic_sim_core::verbose::LocalLogger;
/// 
/// // Prepare vehicles storage (owned Vehicles)
/// let mut vehicles: VehiclesStorage = VehiclesStorage::new();
/// vehicles.insert(1, Vehicle::new(1).with_speed(2).build());
/// vehicles.insert(2, Vehicle::new(2).with_speed(3).build());
/// 
/// // Conflict among vehicle IDs 1 and 2, first has priority
/// let conflicts = vec![CellConflict {
///    cell_id: 1,
///    participants: vec![1, 2],
///    priority_participant_index: 0,
///    conflict_type: ConflictType::ForwardLaneChange,
/// }];
/// 
/// let result = solve_conflicts(conflicts, &mut vehicles, &LocalLogger::none());
/// assert!(result.is_ok(), "Conflict resolution failed");
pub fn solve_conflicts<'b>(
    conflicts_data: Vec<CellConflict>,
    vehicles: &mut IndexMap<VehicleID, Vehicle>,
    verbose: &LocalLogger,
) -> Result<(), ConflictSolverError> {
    if verbose.is_at_least(VerboseLevel::Main) {
        verbose.log_with_fields(
            EVENT_CONFLICTS_SOLVE,
            "Solve conflicts",
            &[("conflicts_num", &conflicts_data.len())]
        );
    }
    // Cycle participants use their cycle delay throughout this tick. Another zone
    // must not reapply ordinary patience and bypass or duplicate their cycle grant.
    let mut cycle_participants = HashSet::new();
    for conflict in &conflicts_data {
        if conflict.conflict_type == ConflictType::MutualYield {
            cycle_participants.extend(conflict.participants.iter().copied());
        }
    }
    let mut trajectories_conflicts: Vec<CellConflict> = Vec::new();
    let mut conflict_zones_conflicts: Vec<CellConflict> = Vec::new();
    // Cells already granted to a desperate (patience-override) winner this tick, so no later
    // grant (desperate OR normal) can put a second vehicle on the same cell.
    let mut claimed_cells: HashSet<CellID> = HashSet::new();
    // Cells physically occupied RIGHT NOW (head + materialized tail of every vehicle). At
    // solve time vehicle positions have not changed yet (movement is a later pipeline step),
    // so this equals Session::current_position. In this CA a vehicle never enters a cell that
    // is occupied this tick (it stops behind the current position of the vehicle ahead - there
    // is no same-tick "following" into a vacated cell), so the desperate override must not be
    // granted a currently-occupied cell either. Mirrors Session::current_position's head+tail.
    let occupancy: HashMap<CellID, VehicleID> = {
        let mut occ = HashMap::new();
        for (id, v) in vehicles.iter() {
            occ.insert(v.cell_id, *id);
            for &tail_cell in &v.tail_cells {
                if tail_cell >= 0 {
                    occ.insert(tail_cell, *id);
                }
            }
        }
        occ
    };
    // Resolve cycle and desperate grants FIRST so reservations precede normal grants;
    // otherwise a normal winner processed earlier could take a cell a later desperate winner
    // also claims. Partition preserves relative order within each group (determinism).
    let conflicts_data: Vec<CellConflict> = {
        let (mut desperate_first, rest): (Vec<CellConflict>, Vec<CellConflict>) =
            conflicts_data.into_iter().partition(|c| {
                c.conflict_type == ConflictType::MutualYield || conflict_has_desperate(c, vehicles, &cycle_participants)
            });
        desperate_first.extend(rest);
        desperate_first
    };
    for conflict in conflicts_data {
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

        if conflict.conflict_type == ConflictType::MutualYield {
            let candidate_id = conflict.participants[conflict.priority_participant_index];
            let candidate = vehicles.get(&candidate_id).ok_or_else(|| {
                ConflictSolverError::InvalidConflict(format!("Vehicle {} not found", candidate_id))
            })?;
            let entered = candidate.first_motion_cell().unwrap_or(-1);
            let granted = candidate.yield_cycle_ready()
                && candidate.intention.intention_speed > 0
                && entered >= 0 && entered != candidate.cell_id
                && occupancy.get(&entered).is_none_or(|&owner| owner == candidate_id)
                && claimed_cells.insert(entered);
            for id in &conflict.participants {
                let vehicle = vehicles.get_mut(id).ok_or_else(|| {
                    ConflictSolverError::InvalidConflict(format!("Vehicle {} not found", id))
                })?;
                if granted && *id == candidate_id {
                    vehicle.restrict_intention_to_first_step();
                } else {
                    vehicle.hold_for_yield_cycle();
                    vehicle.block_with_speed(0);
                }
            }
            continue;
        }

        // Ordinary patience override, excluding physical bodies and merge cycles.
        // A vehicle stuck past its patience threshold (Vehicle::is_desperate) wins the
        // contested cell, overriding normal right-of-way, the fixed conflict-zone winner
        // AND the left/right rule. Among several desperate participants the most-starved
        // (highest wait_steps) wins, giving a single deterministic winner. It NEVER applies
        // to physical Tail conflicts (you cannot force through a body), and `claimed_cells`
        // keeps the one-occupant-per-cell invariant across conflicts. Non-desperate
        // conflicts fall through untouched to the usual resolution below.
        let is_physical = matches!(
            conflict.conflict_type,
            ConflictType::Tail | ConflictType::SelfTail | ConflictType::TailCrossLaneChange
        );
        if !is_physical {
            // Most-starved (highest wait_steps) wins. Ties are broken deterministically by
            // the LOWEST vehicle id (earliest spawn), NOT by participant iteration order -
            // `max_by_key` otherwise returns the last of several equal maxima, making the
            // winner depend on the order participants happen to sit in the conflict.
            let desperate_winner = conflict
                .participants
                .iter()
                .enumerate()
                .filter_map(|(i, id)| {
                    vehicles.get(id).filter(|v| {
                        v.is_desperate() && !cycle_participants.contains(id)
                    }).map(|v| (i, v.wait_steps, *id))
                })
                .max_by_key(|&(_, wait, id)| (wait, Reverse(id)));
            if let Some((win_idx, _, _)) = desperate_winner {
                let win_id = conflict.participants[win_idx];
                // The cell the winner actually ENTERS this tick is its first path cell - the
                // contested intermediate, not the far head cell of a multi-cell move. Reserving
                // that cell (and clamping the winner to a one-cell step below) keeps it on a
                // single known cell instead of sweeping unreserved intermediate cells.
                let entered = vehicles
                    .get(&win_id)
                    .and_then(Vehicle::first_motion_cell)
                    .unwrap_or(-1);
                if entered >= 0 {
                    // Granted unless the cell is unavailable, in which case block all
                    // participants here (preserve one-occupant-per-cell). It is unavailable if
                    // either (1) another desperate winner already claimed it this tick, or
                    // (2) it is physically occupied right now by some other vehicle - the
                    // override must not force a move into a standing vehicle (no same-tick
                    // following in this CA). `claimed_cells.insert` is only attempted when the
                    // cell is free, so a denied grant never reserves it.
                    let occupied_by_other = occupancy
                        .get(&entered)
                        .map(|&occ_id| occ_id != win_id)
                        .unwrap_or(false);
                    let granted = !occupied_by_other && claimed_cells.insert(entered);
                    if verbose.is_at_least(VerboseLevel::Additional) {
                        verbose.log_with_fields(
                            EVENT_CONFLICT_SOLVE,
                            if granted { "Patience override - desperate vehicle wins" } else { "Patience override - entered cell unavailable (claimed or occupied), blocking" },
                            &[
                                ("cell", &conflict.cell_id),
                                ("winner_id", &win_id),
                                ("entered_cell", &entered),
                                ("conflict_type", &format!("{:?}", conflict.conflict_type)),
                            ],
                        );
                    }
                    for (i, participant_id) in conflict.participants.iter().enumerate() {
                        if let Some(v) = vehicles.get_mut(participant_id) {
                            if granted && i == win_idx {
                                // Clamp the winner to a single-cell step into `entered` so it
                                // occupies exactly the reserved cell and never sweeps unreserved
                                // intermediate cells. This also rebuilds the tail so a stale
                                // multi-cell tail does not land ahead of the clamped head.
                                v.restrict_intention_to_first_step();
                            } else {
                                v.block_with_speed(0);
                            }
                        }
                    }
                    continue;
                }
            }
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
                            ("participants", &format!("{:?}", conflict.participants)),
                        ]
                    );
                }
                continue;
            }
            ConflictType::CrossLaneChange | ConflictType::TailCrossLaneChange => {
                // This must be crossing trajectories conflict. Need to postprocess after
                if let Some(priority_participant_id) = conflict.participants.get(conflict.priority_participant_index) {
                    if let Some(priority_participant) = vehicles.get_mut(priority_participant_id) {
                        priority_participant.is_conflict_participant = false;
                    }
                }
                if verbose.is_at_least(VerboseLevel::Additional) {
                    verbose.log_with_fields(
                        EVENT_CONFLICT_SOLVE,
                        "Found trajectories cross conflict",
                        &[
                            ("cell", &conflict.cell_id),
                            ("conflict_type", &format!("{:?}", conflict.conflict_type)),
                            ("participants", &format!("{:?}", conflict.participants)),
                            ("priority_participant_index", &conflict.priority_participant_index),
                        ]
                    );
                }
                trajectories_conflicts.push(conflict);
                continue;
            }
            ConflictType::CrossConflictZone => {
                if let Some(priority_participant_id) = conflict.participants.get(conflict.priority_participant_index) {
                    if let Some(priority_participant) = vehicles.get_mut(priority_participant_id) {
                        priority_participant.is_conflict_participant = false;
                    }
                }
                if verbose.is_at_least(VerboseLevel::Additional) {
                    verbose.log_with_fields(
                        EVENT_CONFLICT_SOLVE,
                        "Found trajectories cross conflict in zone",
                        &[
                            ("cell", &conflict.cell_id),
                            ("conflict_type", &format!("{:?}", conflict.conflict_type)),
                            ("participants", &format!("{:?}", conflict.participants)),
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
                    ("participants", &format!("{:?}", conflict.participants)),
                    ("priority_index", &conflict.priority_participant_index),
                ]
            );
        }

        // If the contested cell was already taken by a desperate winner (resolved first),
        // nobody in this conflict may enter it: block all participants.
        if conflict.cell_id >= 0 && claimed_cells.contains(&conflict.cell_id) {
            for participant_id in &conflict.participants {
                if let Some(v) = vehicles.get_mut(participant_id) {
                    v.block_with_speed(0);
                }
            }
            continue;
        }

        let priority_index = conflict.priority_participant_index;
        for (i, participant_id) in conflict.participants.iter().enumerate() {
            let participant = vehicles.get_mut(participant_id).ok_or_else(|| ConflictSolverError::InvalidConflict(format!("Vehicle {} not found", participant_id)))?;
            if i == priority_index {
                // Priority retains only an existing first step for a lane change.
                // Conflict metadata cannot supply a replacement destination.
                if matches!(participant.intention.intention_maneuver, LaneChangeType::ChangeLeft | LaneChangeType::ChangeRight) {
                    participant.restrict_intention_to_first_step();
                }
                // Sweep-through guard (one-occupant-per-cell): a multi-cell winner must not
                // drive THROUGH a cell already reserved by a desperate winner this tick. Scan
                // the cells it would enter (the intermediate cells, then the head); if one is
                // claimed, clamp the winner to a single step into its first cell - or block it
                // outright when that very first cell is the claimed one.
                let first_claimed = participant.intention.intermediate_cells.iter()
                    .chain(once(&participant.intention.intention_cell_id))
                    .position(|c| claimed_cells.contains(c));
                if let Some(j) = first_claimed {
                    if j == 0 {
                        participant.block_with_speed(0);
                    } else {
                        participant.restrict_intention_to_first_step();
                    }
                }
            } else {
                // Stay still
                participant.block_with_speed(0);
            }
        }
    }
    
    // Postprocessing remaining possible trajectories conflicts
    for conflict in trajectories_conflicts {
        if verbose.is_at_least(VerboseLevel::Additional) {
            let vehicles_ids: Vec<_> = conflict.participants.clone();
            verbose.log_with_fields(
                EVENT_CONFLICT_SOLVE,
                "Solve trajectories cross conflict",
                &[
                    ("cell", &conflict.cell_id),
                    ("participants_ids", &format!("{:?}", vehicles_ids)),
                    ("conflict_type", &format!("{:?}", conflict.conflict_type)),
                ]
            );
        }
        match conflict.conflict_type {
            ConflictType::TailCrossLaneChange => {
                let priority_index = conflict.priority_participant_index;
                for (i, participant_id) in conflict.participants.iter().enumerate() {
                    let participant = vehicles.get_mut(participant_id).ok_or_else(|| ConflictSolverError::InvalidConflict(format!("Vehicle {} not found", participant_id)))?;
                    if i != priority_index {
                        participant.block_with_speed(0);
                    }
                }
                continue;
            }
            _ => {}
        }
        // Solve plain trajectories conflict. Both participants are lane-changing (left vs right);
        // the winner is the precomputed priority participant - a CUT-IN aggressor if one out-
        // aggresses the other, otherwise the LEFT maneuver (see `find_cross_trajectories_conflict_naive`).
        // HONOR `priority_participant_index` here instead of unconditionally letting LEFT win, so the
        // aggressor gradient covers crossings just like every other rule.
        let mut has_block = false;
        let mut left_maneuver_index: Option<usize> = None;
        let mut right_maneuver_index: Option<usize> = None;
        for (i, participant_id) in conflict.participants.iter().enumerate() {
            let participant = vehicles.get(participant_id).ok_or_else(|| ConflictSolverError::InvalidConflict(format!("Vehicle {} not found", participant_id)))?;
            match participant.intention.intention_maneuver {
                LaneChangeType::ChangeLeft => left_maneuver_index = Some(i),
                LaneChangeType::ChangeRight => right_maneuver_index = Some(i),
                LaneChangeType::Block => has_block = true,
                _ => {}
            }
        }
        if !has_block && left_maneuver_index.is_some() && right_maneuver_index.is_some() {
            // Winner = the priority participant (aggressor cut-in, else left). If the winner's
            // target cell was already taken by a desperate winner this tick, neither may move into
            // it -> block all (one-occupant-per-cell).
            let winner_index = conflict.priority_participant_index;
            let winner_id = conflict.participants[winner_index];
            let winner_target = vehicles.get(&winner_id).map(|v| v.intention.intention_cell_id).unwrap_or(-1);
            if winner_target >= 0 && claimed_cells.contains(&winner_target) {
                for participant_id in &conflict.participants {
                    if let Some(v) = vehicles.get_mut(participant_id) { v.block_with_speed(0); }
                }
            } else {
                for (i, participant_id) in conflict.participants.iter().enumerate() {
                    if let Some(v) = vehicles.get_mut(participant_id) {
                        if i == winner_index {
                            // Priority only retains a currently permitted first step.
                            v.restrict_intention_to_first_step();
                        } else {
                            v.block_with_speed(0);
                        }
                    }
                }
            }
        }
    }

    for conflict in conflict_zones_conflicts {
        if verbose.is_at_least(VerboseLevel::Additional) {
            let vehicles_ids: Vec<_> = conflict.participants.clone();
            verbose.log_with_fields(
                EVENT_CONFLICT_SOLVE,
                "Solve trajectories cross conflict in zone",
                &[
                    ("cell", &conflict.cell_id),
                    ("participants_ids", &format!("{:?}", vehicles_ids)),
                    ("conflict_type", &format!("{:?}", conflict.conflict_type)),
                ]
            );
        }
        
        let priority_index = conflict.priority_participant_index;
        // If the zone winner's target cell was already taken by a desperate winner, it may
        // not enter it either: block all participants (preserve one-occupant-per-cell).
        let prio_id = conflict.participants[priority_index];
        let prio_target = vehicles.get(&prio_id).map(|v| v.intention.intention_cell_id).unwrap_or(-1);
        if prio_target >= 0 && claimed_cells.contains(&prio_target) {
            for participant_id in &conflict.participants {
                if let Some(v) = vehicles.get_mut(participant_id) { v.block_with_speed(0); }
            }
            continue;
        }
        for (i, participant_id) in conflict.participants.iter().enumerate() {
            if i != priority_index {
                // Yielding holds the current cell, so speed and standing time must record a stop.
                if let Some(v) = vehicles.get_mut(participant_id) { v.block_with_speed(0); }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
#[path = "tests.rs"]
mod tests;
