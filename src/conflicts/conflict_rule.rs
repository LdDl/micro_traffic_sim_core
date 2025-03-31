use crate::agents::BehaviourType;
use crate::agents::Vehicle;
use crate::conflicts::ConflictType;
use crate::grid::lane_change_type::LaneChangeType;
use crate::intentions::{CellIntention, IntentionType};
use rand::Rng;

// At the top of your file
#[cfg(not(test))]
use rand::thread_rng;

#[cfg(test)]
pub fn thread_rng() -> impl Rng {
    // Return a fixed-seed RNG for testing
    use rand::SeedableRng;
    rand::rngs::StdRng::seed_from_u64(42)
}

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
        // let mut rng = rand::thread_rng(); // This is not working in the test because of the thread_rng() function
        let mut rng = thread_rng();
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
    use crate::agents::VehicleIntention;
    #[test]
    fn test_resolve_merge_lane_change() {
        // Case 1: Aggressive vehicle should win
        let vehicle_one = Vehicle::new(1)
            .with_behaviour(BehaviourType::Aggressive)
            .with_speed(1)
            .build();
        let vehicle_two = Vehicle::new(2)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(3)
            .build();
        let intention_one = CellIntention::new(Some(&vehicle_one), IntentionType::Target);
        let intention_two = CellIntention::new(Some(&vehicle_two), IntentionType::Target);

        let correct_winner = (intention_one.clone(), ConflictType::MergeLaneChange);
        let actual_winner = resolve_merge_lane_change(&intention_one, &intention_two);
        assert_eq!(
            correct_winner.0.get_vehicle().unwrap().id,
            actual_winner.0.get_vehicle().unwrap().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 2: Vehicle doing LEFT maneuver should win
        let mut vehicle_one = Vehicle::new(1)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(1)
            .build();
        vehicle_one.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeLeft,
            ..Default::default()
        });
        let mut vehicle_two = Vehicle::new(2)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(3)
            .build();
        vehicle_two.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            ..Default::default()
        });
        let intention_one = CellIntention::new(Some(&vehicle_one), IntentionType::Target);
        let intention_two = CellIntention::new(Some(&vehicle_two), IntentionType::Target);

        let correct_winner = (intention_one.clone(), ConflictType::MergeLaneChange);
        let actual_winner = resolve_merge_lane_change(&intention_one, &intention_two);
        assert_eq!(
            correct_winner.0.get_vehicle().unwrap().id,
            actual_winner.0.get_vehicle().unwrap().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );
    }
    #[test]
    fn test_resolve_by_speed_and_cooperativity() {
        // Case 1: Vehicle with higher speed should win
        let vehicle_one = Vehicle::new(1)
            .with_speed(5)
            .with_cooperative_level(0.5)
            .build();
        let vehicle_two = Vehicle::new(2)
            .with_speed(3)
            .with_cooperative_level(0.5)
            .build();
        let intention_one = CellIntention::new(Some(&vehicle_one), IntentionType::Target);
        let intention_two = CellIntention::new(Some(&vehicle_two), IntentionType::Target);

        let correct_winner = (intention_one.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_by_speed_and_cooperativity(&intention_one, &intention_two);
        assert_eq!(
            correct_winner.0.get_vehicle().unwrap().id,
            actual_winner.0.get_vehicle().unwrap().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 2: Same speed, less cooperative vehicle should win
        let vehicle_three = Vehicle::new(3)
            .with_speed(4)
            .with_cooperative_level(0.3)
            .build();
        let vehicle_four = Vehicle::new(4)
            .with_speed(4)
            .with_cooperative_level(0.8)
            .build();
        let intention_three = CellIntention::new(Some(&vehicle_three), IntentionType::Target);
        let intention_four = CellIntention::new(Some(&vehicle_four), IntentionType::Target);

        let correct_winner = (intention_three.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_by_speed_and_cooperativity(&intention_three, &intention_four);
        assert_eq!(
            correct_winner.0.get_vehicle().unwrap().id,
            actual_winner.0.get_vehicle().unwrap().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 3: Equal speed, equal cooperativity (random decision - cannot test deterministically)
        // Just verify it runs without errors
        let vehicle_five = Vehicle::new(5)
            .with_speed(3)
            .with_cooperative_level(0.5)
            .build();
        let vehicle_six = Vehicle::new(6)
            .with_speed(3)
            .with_cooperative_level(0.5)
            .build();
        let intention_five = CellIntention::new(Some(&vehicle_five), IntentionType::Target);
        let intention_six = CellIntention::new(Some(&vehicle_six), IntentionType::Target);

        // Send seed for random number generator to make test deterministicgg
        let correct_winner = (intention_six.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_by_speed_and_cooperativity(&intention_five, &intention_six);
        assert_eq!(
            correct_winner.0.get_vehicle().unwrap().id,
            actual_winner.0.get_vehicle().unwrap().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );
    }
    #[test]
    fn test_resolve_merge_forward() {
        // Case 1: Aggressive vehicle should win over cooperative
        let vehicle_one = Vehicle::new(1)
            .with_behaviour(BehaviourType::Aggressive)
            .with_speed(3)
            .build();
        let vehicle_two = Vehicle::new(2)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(3)
            .build();
        let intention_one = CellIntention::new(Some(&vehicle_one), IntentionType::Target);
        let intention_two = CellIntention::new(Some(&vehicle_two), IntentionType::Target);

        let coorect_winner = (intention_one.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_merge_forward(&intention_one, &intention_two);
        assert_eq!(
            coorect_winner.0.get_vehicle().unwrap().id,
            actual_winner.0.get_vehicle().unwrap().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            coorect_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 2: Transit intention should win over Target intention
        let vehicle_three = Vehicle::new(3)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build();
        let vehicle_four = Vehicle::new(4)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build();
        let intention_three = CellIntention::new(Some(&vehicle_three), IntentionType::Target);
        let intention_four = CellIntention::new(Some(&vehicle_four), IntentionType::Transit);

        let correct_winner = (intention_four.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_merge_forward(&intention_three, &intention_four);
        assert_eq!(
            correct_winner.0.get_vehicle().unwrap().id,
            actual_winner.0.get_vehicle().unwrap().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 3: For same intention types, should delegate to
        // resolve_by_speed_and_cooperativity() call
        let vehicle_five = Vehicle::new(5)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(5)
            .with_cooperative_level(0.5)
            .build();
        let vehicle_six = Vehicle::new(6)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .with_cooperative_level(0.5)
            .build();
        let intention_five = CellIntention::new(Some(&vehicle_five), IntentionType::Target);
        let intention_six = CellIntention::new(Some(&vehicle_six), IntentionType::Target);

        let correct_winner = (intention_five.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_merge_forward(&intention_five, &intention_six);
        assert_eq!(
            correct_winner.0.get_vehicle().unwrap().id,
            actual_winner.0.get_vehicle().unwrap().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );
    }
}
