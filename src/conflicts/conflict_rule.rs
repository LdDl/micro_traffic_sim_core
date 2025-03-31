use crate::agents::BehaviourType;
use crate::agents::Vehicle;
use crate::conflicts::ConflictType;
use crate::grid::lane_change_type::LaneChangeType;
use crate::intentions::{CellIntention, IntentionType};
use rand::Rng;

const EPS_COOP_LEVEL: f64 = 0.0001;

/// Checks if the vehicle is changing lane
pub fn changing_lane(agent: &Vehicle) -> bool {
    agent.intention.intention_maneuver == LaneChangeType::ChangeLeft
        || agent.intention.intention_maneuver == LaneChangeType::ChangeRight
}

/// Checks if one vehicle has aggressive strategy and another has cooperative
pub fn has_agressive_level_advantage(vehicle_one: &Vehicle, vehicle_two: &Vehicle) -> bool {
    vehicle_one.strategy_type == BehaviourType::Aggressive
        && vehicle_two.strategy_type == BehaviourType::Cooperative
}

pub fn resolve_merge_lane_change<'a>(
    intention_one: &'a CellIntention<'a>,
    intention_two: &'a CellIntention<'a>,
) -> (&'a CellIntention<'a>, ConflictType) {
    // Extract vehicles once (with early panic if missing)
    let vehicle_one = intention_one
        .get_vehicle()
        .expect("Vehicle missing in intention_one");
    let vehicle_two = intention_two
        .get_vehicle()
        .expect("Vehicle missing in intention_two");

    // Check aggressive behaviour advantage, so aggressive vehicle could even violate the traffic rules
    // and have advantage over the cooperative vehicle
    if has_agressive_level_advantage(vehicle_one, vehicle_two) {
        return (intention_one, ConflictType::MergeLaneChange);
    }
    if has_agressive_level_advantage(vehicle_two, vehicle_one) {
        return (intention_two, ConflictType::MergeLaneChange);
    }

    // Give priority to the one which does LEFT maneuver (so vehicle is in most right position of the road)
    // Check specific maneuver combinations
    match (
        vehicle_one.intention.intention_maneuver,
        vehicle_two.intention.intention_maneuver,
    ) {
        (LaneChangeType::ChangeLeft, LaneChangeType::ChangeRight) => {
            (intention_one, ConflictType::MergeLaneChange)
        }
        (LaneChangeType::ChangeRight, LaneChangeType::ChangeLeft) => {
            (intention_two, ConflictType::MergeLaneChange)
        }
        _ => {
            panic!("Unexpected lane change")
        }
    }
}

pub fn resolve_by_speed_and_cooperativity<'a>(
    intention_one: &'a CellIntention<'a>,
    intention_two: &'a CellIntention<'a>,
) -> (&'a CellIntention<'a>, ConflictType) {
    let vehicle_one = intention_one
        .get_vehicle()
        .expect("Vehicle missing in intention_one");
    let vehicle_two = intention_two
        .get_vehicle()
        .expect("Vehicle missing in intention_two");

    // Check speed advantage
    if vehicle_one.speed != vehicle_two.speed {
        // Should win the one with higher speed
        if vehicle_one.speed >= vehicle_two.speed {
            return (intention_one, ConflictType::MergeForward);
        }
        return (intention_two, ConflictType::MergeForward);
    }

    // Speeds are equal, check cooperativity
    let coop_diff = vehicle_one.cooperativity - vehicle_two.cooperativity;
    if coop_diff.abs() < EPS_COOP_LEVEL {
        // Random choice for equal cooperativity
        let mut rng = rand::thread_rng();
        if rng.gen_bool(0.5) {
            return (intention_one, ConflictType::MergeForward);
        }
        return (intention_two, ConflictType::MergeForward);
    }

    // Give priority to less cooperative vehicle (in other words: more cooperative vehicle should give way)
    if vehicle_one.cooperativity > vehicle_two.cooperativity {
        return (intention_two, ConflictType::MergeForward);
    }
    (intention_one, ConflictType::MergeForward)
}

pub fn resolve_merge_forward<'a>(
    intention_one: &'a CellIntention<'a>,
    intention_two: &'a CellIntention<'a>,
) -> (&'a CellIntention<'a>, ConflictType) {
    // Extract vehicles once (with early panic if missing)
    let vehicle_one = intention_one
        .get_vehicle()
        .expect("Vehicle missing in intention_one");
    let vehicle_two = intention_two
        .get_vehicle()
        .expect("Vehicle missing in intention_two");

    // Check aggressive behaviour advantage
    if has_agressive_level_advantage(vehicle_one, vehicle_two) {
        return (intention_one, ConflictType::MergeForward);
    }
    if has_agressive_level_advantage(vehicle_two, vehicle_one) {
        return (intention_two, ConflictType::MergeForward);
    }

    // Check intention types
    match (intention_one.int_type, intention_two.int_type) {
        // Both have same intention type (both target or both transit)
        (IntentionType::Target, IntentionType::Target)
        | (IntentionType::Transit, IntentionType::Transit) => {
            resolve_by_speed_and_cooperativity(intention_one, intention_two)
        }
        // First is target, second is transit (second is moving faster)
        (IntentionType::Target, IntentionType::Transit) => {
            (intention_two, ConflictType::MergeForward)
        }
        // First is transit, second is target (first is moving faster)
        (IntentionType::Transit, IntentionType::Target) => {
            (intention_one, ConflictType::MergeForward)
        }
        // Unexpected combination
        _ => {
            panic!("Unexpected intention type for CONFLICT_TYPE_MERGE");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_resolve_merge_lane_change() {
        todo!();
    }
    #[test]
    fn test_resolve_by_speed_and_cooperativity() {
        todo!();
    }
    #[test]
    fn test_resolve_merge_forward() {
        todo!();
    }
}
