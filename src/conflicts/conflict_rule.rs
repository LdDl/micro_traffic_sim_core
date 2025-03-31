use crate::agents::BehaviourType;
use crate::agents::Vehicle;
use crate::grid::lane_change_type::LaneChangeType;
use crate::intentions::CellIntention;

/// Checks if the vehicle is changing lane
pub fn changing_lane(agent: &Vehicle) -> bool {
    return agent.intention.intention_maneuver == LaneChangeType::ChangeLeft
        || agent.intention.intention_maneuver == LaneChangeType::ChangeRight;
}

/// Checks if one vehicle has aggressive strategy and another has cooperative
pub fn has_agressive_level_advantage(
    intention_one: &CellIntention,
    intention_two: &CellIntention,
) -> bool {
    return intention_one.get_vehicle().is_some_and(|vehicle_one| {
        intention_two.get_vehicle().is_some_and(|vehicle_two| {
            vehicle_one.strategy_type == BehaviourType::Aggressive
                && vehicle_two.strategy_type == BehaviourType::Cooperative
        })
    });
}

