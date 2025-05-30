use crate::agents::Vehicle;
use crate::conflict_zones::{ConflictWinnerType, ConflictZone, ConflictZoneID};
use crate::grid::cell::CellID;
use crate::grid::road_network::GridRoads;
use crate::intentions::Intentions;
use crate::utils::rand::thread_rng;
use rand::Rng;

use std::collections::HashMap;
use std::fmt;
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
pub struct CellConflict<'a> {
    /// Cell ID where the conflict occurs
    pub cell_id: CellID,
    /// Agents that are involved in the conflict
    pub participants: Vec<&'a mut Vehicle>,
    /// Agent that has priority in the conflict
    pub priority_participant_index: usize,
    /// Type of the conflict
    pub conflict_type: ConflictType,
}

impl fmt::Display for CellConflict<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let participants: Vec<String> =
            self.participants.iter().map(|p| p.id.to_string()).collect();
        let priority_participant_id = self.participants
            .get(self.priority_participant_index)
            .map_or("None".to_string(), |p| p.id.to_string());
        let cell_id = self.cell_id.to_string();
        write!(
            f,
            "CellConflict{{CellID: {}, Type: {}, Participants: ({:?}), Priority participant: {}}}",
            cell_id, self.conflict_type, participants, priority_participant_id
        )
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::conflict_zones::ConflictEdge;
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
}
