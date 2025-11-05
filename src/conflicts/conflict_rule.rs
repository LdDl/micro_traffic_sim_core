use crate::behaviour::BehaviourType;
use crate::agents::Vehicle;
use crate::conflicts::ConflictType;
use crate::maneuver::LaneChangeType;
use crate::intentions::{CellIntention, IntentionType};
use rand::Rng;
use crate::utils::rand::rng;

const EPS_COOP_LEVEL: f64 = 0.0001;

/// Defines a rule for resolving a conflict between two vehicles.
///
/// Each rule consists of a condition (when the rule applies) and a resolver (how to pick the winner and conflict type).
struct ConflictRule {
    /// Returns true if this rule applies to the given pair of vehicles.
    condition: fn(&Vehicle, &Vehicle) -> bool,
    /// Resolves the conflict and returns the winning intention and conflict type.
    resolver: for<'a> fn(
        &'a CellIntention,
        &'a CellIntention,
    ) -> (&'a CellIntention, ConflictType),
}

macro_rules! conflict_rules {
    ( $( [ $condition:expr, $resolver:expr ] ),* ) => {
        &[
            $(
                ConflictRule {
                    condition: $condition,
                    resolver: $resolver,
                },
            )*
        ]
    }
}

static CONFLICT_RULES: &[ConflictRule] = conflict_rules![
    // Both vehicles are changing lane on a single road
    // Two source lanes on one road is going to merge into single lane on another road
    [ |v1, v2| changing_lane(v1) && changing_lane(v2), resolve_merge_lane_change ],
    
    // Both vehicles are moving forward on different lanes of different roads
    // Differet roads are going to merge into single road
    [ |v1, v2| v1.intention.intention_maneuver == LaneChangeType::NoChange && v2.intention.intention_maneuver == LaneChangeType::NoChange, resolve_merge_forward ],

    // First vehicle is moving forward, second is changing lane
    [ |v1, v2| v1.intention.intention_maneuver == LaneChangeType::NoChange && changing_lane(v2), |cin1, _cin2| {
        // First vehicle is not doing maneuver, when the second one is doing lane change.
        // Therefore the second vehicle should give way to the first one
        (cin1, ConflictType::ForwardLaneChange)
    } ],

    // First vehicle is changing lane, second is moving forward
    [ |v1, v2| changing_lane(v1) && v2.intention.intention_maneuver == LaneChangeType::NoChange, |_cin1, cin2| {
        // Second vehicle is not doing maneuver, when the first one is doing lane change.
        // Therefore the first vehicle should give way to the second one
        (cin2, ConflictType::ForwardLaneChange)
    } ],

    // First vehicle is changing lane, second is blocking its lane
    [ |v1, v2| changing_lane(v1) && v2.intention.intention_maneuver == LaneChangeType::Block, |_cin1, cin2| {
        // Second vehicle is not moving therefore it holds the position
        (cin2, ConflictType::BlockLaneChange)
    } ],

    // First vehicle is blocking its lane, second is changing lane
    [ |v1, v2| v1.intention.intention_maneuver == LaneChangeType::Block && changing_lane(v2), |cin1, _cin2| {
        // First vehicle is not moving therefore it holds the position
        (cin1, ConflictType::BlockLaneChange)
    } ]
];

// Dynamic approach: obsolete
// Create a static array of rules
// static CONFLICT_RULES: &[ConflictRule] = &[
//     ConflictRule {
//         // Both vehicles are changing lane on a single road
//         // Two source lanes on one road is going to merge into single lane on another road
//         condition: |v1, v2| changing_lane(v1) && changing_lane(v2),
//         resolver: |cin1, cin2| resolve_merge_lane_change(cin1, cin2),
//     },
//     ConflictRule {
//         // Both vehicles are moving forward on different lanes of different roads
//         // Differet roads are going to merge into single road
//         condition: |v1, v2| {
//             v1.intention.intention_maneuver == LaneChangeType::NoChange
//                 && v2.intention.intention_maneuver == LaneChangeType::NoChange
//         },
//         resolver: |cin1, cin2| resolve_merge_forward(cin1, cin2),
//     },
//     ConflictRule {
//         // First vehicle is moving forward, second is changing lane
//         condition: |v1, v2| {
//             v1.intention.intention_maneuver == LaneChangeType::NoChange && changing_lane(v2)
//         },
//         resolver: |cin1, cin2| {
//             // First vehicle is not doing maneuver, when the second one is doing lane change.
//             // Therefore the second vehicle should give way to the first one
//             (cin1, ConflictType::MergeForward)
//         },
//     },
//     ConflictRule {
//         // First vehicle is changing lane, second is moving forward
//         condition: |v1, v2| {
//             changing_lane(v1) && v2.intention.intention_maneuver == LaneChangeType::NoChange
//         },
//         resolver: |cin1, cin2| {
//             // Second vehicle is not doing maneuver, when the first one is doing lane change.
//             // Therefore the first vehicle should give way to the second one
//             (cin2, ConflictType::MergeForward)
//         },
//     },
//     ConflictRule {
//         // First vehicle is changing lane, second is blocking its lane
//         condition: |v1, v2| {
//             changing_lane(v1) && v2.intention.intention_maneuver == LaneChangeType::Block
//         },
//         resolver: |cin1, cin2| {
//             // Second vehicle is not moving therefore it holds the position
//             (cin2, ConflictType::BlockLaneChange)
//         },
//     },
//     ConflictRule {
//         // First vehicle is blocking its lane, second is changing lane
//         condition: |v1, v2| {
//             v1.intention.intention_maneuver == LaneChangeType::Block && changing_lane(v2)
//         },
//         resolver: |cin1, cin2| {
//             // First vehicle is not moving therefore it holds the position
//             (cin1, ConflictType::BlockLaneChange)
//         },
//     },
// ];

/// Applies all simple conflict rules to a pair of intentions and returns the winner and conflict type.
///
/// Rules cover lane changes, merges, blocking, and forward movement.
pub fn resolve_simple_rules<'a>(
    intention_one: &'a CellIntention,
    intention_two: &'a CellIntention,
) -> (&'a CellIntention, ConflictType) {
    let v1 = &*intention_one.vehicle.borrow();
    let v2 = &*intention_two.vehicle.borrow();
    for rule in CONFLICT_RULES {
        if (rule.condition)(v1, v2) {
            return (rule.resolver)(intention_one, intention_two);
        }
    }
    panic!("Unexpected conflict type")
}

/// Checks if the vehicle wants to perform a lane change maneuver.
///
/// # Visualization
/// ```text
/// Lane 1: →→→[Cell]
/// Lane 2: →→→↗
///            ↑
///        [Lane Change]
/// ```
pub fn changing_lane(agent: &Vehicle) -> bool {
    agent.intention.intention_maneuver == LaneChangeType::ChangeLeft
        || agent.intention.intention_maneuver == LaneChangeType::ChangeRight
}

/// Returns true if the first vehicle is aggressive and the second is cooperative.
/// Future works: aggressive vehicles may win conflicts even against traffic rules.
pub fn has_agressive_level_advantage(vehicle_one: &Vehicle, vehicle_two: &Vehicle) -> bool {
    vehicle_one.strategy_type == BehaviourType::Aggressive
        && vehicle_two.strategy_type == BehaviourType::Cooperative
}

/// Resolves a merge conflict where both vehicles are changing lanes into the same cell.
///
/// Priority is given to aggressive vehicles, or to the vehicle performing a left maneuver (simulating right-hand traffic).
/// 
/// # Visualization
/// ```text
/// Lane 1: →→→↘
///              [Cell]
/// Lane 2: →→→↗  ↑
///               ↑
///        [Merge Point]
/// Priority: Aggressive > Left Maneuver (right-hand traffic)
/// ```
pub fn resolve_merge_lane_change<'a>(
    intention_one: &'a CellIntention,
    intention_two: &'a CellIntention,
) -> (&'a CellIntention, ConflictType) {
    // Extract vehicles once (with early panic if missing)
    let vehicle_one = intention_one.vehicle.borrow();
    let vehicle_two = intention_two.vehicle.borrow();

    // Check aggressive behaviour advantage, so aggressive vehicle could even violate the traffic rules
    // and have advantage over the cooperative vehicle
    if has_agressive_level_advantage(&vehicle_one, &vehicle_two) {
        return (intention_one, ConflictType::MergeLaneChange);
    }
    if has_agressive_level_advantage(&vehicle_two, &vehicle_one) {
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

/// Resolves a merge conflict by comparing speed and cooperativity.
///
/// Used when both vehicles have the same intention type (target or transit).
///
/// # Visualization
/// ```text
/// Road A: →→→ (speed 5, coop 0.3) ↘
///                                 [Cell]
/// Road B: →→→ (speed 3, coop 0.8) ↗
/// Priority: Faster > Less cooperative > Random
/// ```
pub fn resolve_by_speed_and_cooperativity<'a>(
    intention_one: &'a CellIntention,
    intention_two: &'a CellIntention,
) -> (&'a CellIntention, ConflictType) {
    let vehicle_one = intention_one.vehicle.borrow();
    let vehicle_two = intention_two.vehicle.borrow();

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
        // let mut rng = rand::rng(); // This is not working in the test because of the rng() function
        let mut rng = rng();
        if rng.random_bool(0.5) {
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

/// Resolves a merge conflict where both vehicles are moving forward into the same cell.
///
/// Priority is given to aggressive vehicles, then to the vehicle with higher speed, then to the less cooperative vehicle.
/// If all else is equal, the winner is chosen randomly.
///
/// # Visualization
/// ```text
/// Road A: →→→
///             ↘
///              [Merge Cell]
///             ↗
/// Road B: →→→
/// Priority: Aggressive > Faster > Less cooperative > Random
/// ```
pub fn resolve_merge_forward<'a>(
    intention_one: &'a CellIntention,
    intention_two: &'a CellIntention,
) -> (&'a CellIntention, ConflictType) {
    // Extract vehicles once (with early panic if missing)
    let vehicle_one = intention_one.vehicle.borrow();
    let vehicle_two = intention_two.vehicle.borrow();

    // Check aggressive behaviour advantage
    if has_agressive_level_advantage(&vehicle_one, &vehicle_two) {
        return (intention_one, ConflictType::MergeForward);
    }
    if has_agressive_level_advantage(&vehicle_two, &vehicle_one) {
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
            .build_ref();
        let vehicle_two = Vehicle::new(2)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(3)
            .build_ref();
        let intention_one = CellIntention::new(vehicle_one, IntentionType::Target);
        let intention_two = CellIntention::new(vehicle_two, IntentionType::Target);
        let correct_winner = (intention_one.clone(), ConflictType::MergeLaneChange);
        let actual_winner = resolve_merge_lane_change(&intention_one, &intention_two);
        assert_eq!(
            correct_winner.0.vehicle.borrow().id,
            actual_winner.0.vehicle.borrow().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 2: Vehicle doing LEFT maneuver should win
        let vehicle_one = Vehicle::new(1)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(1)
            .build_ref();
        vehicle_one.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeLeft,
            ..Default::default()
        });
        let vehicle_two = Vehicle::new(2)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(3)
            .build_ref();
        vehicle_two.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            ..Default::default()
        });
        let intention_one = CellIntention::new(vehicle_one, IntentionType::Target);
        let intention_two = CellIntention::new(vehicle_two, IntentionType::Target);
        let correct_winner = (intention_one.clone(), ConflictType::MergeLaneChange);
        let actual_winner = resolve_merge_lane_change(&intention_one, &intention_two);
        assert_eq!(
            correct_winner.0.vehicle.borrow().id,
            actual_winner.0.vehicle.borrow().id,
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
            .build_ref();
        let vehicle_two = Vehicle::new(2)
            .with_speed(3)
            .with_cooperative_level(0.5)
            .build_ref();
        let intention_one = CellIntention::new(vehicle_one, IntentionType::Target);
        let intention_two = CellIntention::new(vehicle_two, IntentionType::Target);
        let correct_winner = (intention_one.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_by_speed_and_cooperativity(&intention_one, &intention_two);
        assert_eq!(
            correct_winner.0.vehicle.borrow().id,
            actual_winner.0.vehicle.borrow().id,
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
            .build_ref();
        let vehicle_four = Vehicle::new(4)
            .with_speed(4)
            .with_cooperative_level(0.8)
            .build_ref();
        let intention_three = CellIntention::new(vehicle_three, IntentionType::Target);
        let intention_four = CellIntention::new(vehicle_four, IntentionType::Target);
        let correct_winner = (intention_three.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_by_speed_and_cooperativity(&intention_three, &intention_four);
        assert_eq!(
            correct_winner.0.vehicle.borrow().id,
            actual_winner.0.vehicle.borrow().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 3: Equal speed, equal cooperativity
        let vehicle_five = Vehicle::new(5)
            .with_speed(3)
            .with_cooperative_level(0.5)
            .build_ref();
        let vehicle_six = Vehicle::new(6)
            .with_speed(3)
            .with_cooperative_level(0.5)
            .build_ref();
        let intention_five = CellIntention::new(vehicle_five, IntentionType::Target);
        let intention_six = CellIntention::new(vehicle_six, IntentionType::Target);
        let correct_winner = (intention_six.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_by_speed_and_cooperativity(&intention_five, &intention_six);
        assert_eq!(
            correct_winner.0.vehicle.borrow().id,
            actual_winner.0.vehicle.borrow().id,
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
            .build_ref();
        let vehicle_two = Vehicle::new(2)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(3)
            .build_ref();
        let intention_one = CellIntention::new(vehicle_one, IntentionType::Target);
        let intention_two = CellIntention::new(vehicle_two, IntentionType::Target);
        let coorect_winner = (intention_one.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_merge_forward(&intention_one, &intention_two);
        assert_eq!(
            coorect_winner.0.vehicle.borrow().id,
            actual_winner.0.vehicle.borrow().id,
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
            .build_ref();
        let vehicle_four = Vehicle::new(4)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build_ref();
        let intention_three = CellIntention::new(vehicle_three, IntentionType::Target);
        let intention_four = CellIntention::new(vehicle_four, IntentionType::Transit);
        let correct_winner = (intention_four.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_merge_forward(&intention_three, &intention_four);
        assert_eq!(
            correct_winner.0.vehicle.borrow().id,
            actual_winner.0.vehicle.borrow().id,
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
            .build_ref();
        let vehicle_six = Vehicle::new(6)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .with_cooperative_level(0.5)
            .build_ref();
        let intention_five = CellIntention::new(vehicle_five, IntentionType::Target);
        let intention_six = CellIntention::new(vehicle_six, IntentionType::Target);
        let correct_winner = (intention_five.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_merge_forward(&intention_five, &intention_six);
        assert_eq!(
            correct_winner.0.vehicle.borrow().id,
            actual_winner.0.vehicle.borrow().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );
    }
    #[test]
    fn test_resolve_simple_rules() {
        // Case 1: both vehicles are changing lane on a single road
        // Two source lanes on one road is going to merge into single lane on another road
        // Winner: vehicle who is doing LEFT maneuver
        let vehicle_one = Vehicle::new(1)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build_ref();
        vehicle_one.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            ..Default::default()
        });
        let vehicle_two = Vehicle::new(2)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build_ref();
        vehicle_two.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeLeft,
            ..Default::default()
        });
        let intention_one = CellIntention::new(vehicle_one, IntentionType::Target);
        let intention_two = CellIntention::new(vehicle_two, IntentionType::Target);
        let correct_winner = (intention_two.clone(), ConflictType::MergeLaneChange);
        let actual_winner = resolve_simple_rules(&intention_one, &intention_two);
        assert_eq!(
            correct_winner.0.vehicle.borrow().id,
            actual_winner.0.vehicle.borrow().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 2: both vehicles are moving forward on different lanes of different roads
        // Differet roads are going to merge into single road
        // Winner: vehicle who is moving faster
        let vehicle_three = Vehicle::new(3)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(5)
            .build_ref();
        vehicle_three.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            ..Default::default()
        });
        let vehicle_four = Vehicle::new(4)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build_ref();
        vehicle_four.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            ..Default::default()
        });
        let intention_three = CellIntention::new(vehicle_three, IntentionType::Target);
        let intention_four = CellIntention::new(vehicle_four, IntentionType::Target);
        let correct_winner = (intention_three.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_simple_rules(&intention_three, &intention_four);
        assert_eq!(
            correct_winner.0.vehicle.borrow().id,
            actual_winner.0.vehicle.borrow().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 3: First vehicle is moving forward, second is changing lane
        // Winner: first vehicle (who is moving forward)
        let vehicle_five = Vehicle::new(5)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build_ref();
        vehicle_five.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            ..Default::default()
        });
        let vehicle_six = Vehicle::new(6)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build_ref();
        vehicle_six.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            ..Default::default()
        });
        let intention_five = CellIntention::new(vehicle_five, IntentionType::Target);
        let intention_six = CellIntention::new(vehicle_six, IntentionType::Target);
        let correct_winner = (intention_five.clone(), ConflictType::ForwardLaneChange);  
        let actual_winner = resolve_simple_rules(&intention_five, &intention_six);
        assert_eq!(
            correct_winner.0.vehicle.borrow().id,
            actual_winner.0.vehicle.borrow().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 4: First vehicle is changing lane, second is moving forward
        // Winner: second vehicle (who is moving forward)
        let vehicle_seven = Vehicle::new(7)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build_ref();
        vehicle_seven.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            ..Default::default()
        });
        let vehicle_eight = Vehicle::new(8)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build_ref();
        vehicle_eight.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            ..Default::default()
        });
        let intention_seven = CellIntention::new(vehicle_seven, IntentionType::Target);
        let intention_eight = CellIntention::new(vehicle_eight, IntentionType::Target);
        let correct_winner = (intention_eight.clone(), ConflictType::ForwardLaneChange);
        let actual_winner = resolve_simple_rules(&intention_seven, &intention_eight);
        assert_eq!(
            correct_winner.0.vehicle.borrow().id,
            actual_winner.0.vehicle.borrow().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 5: First vehicle is changing lane, second is blocking its lane
        // Winner: second vehicle (who is blocking the lane)
        let vehicle_nine = Vehicle::new(9)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build_ref();
        vehicle_nine.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeLeft,
            ..Default::default()
        });
        let vehicle_ten = Vehicle::new(10)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build_ref();
        vehicle_ten.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::Block,
            ..Default::default()
        });
        let intention_nine = CellIntention::new(vehicle_nine, IntentionType::Target);
        let intention_ten = CellIntention::new(vehicle_ten, IntentionType::Target);
        let correct_winner = (intention_ten.clone(), ConflictType::BlockLaneChange);
        let actual_winner = resolve_simple_rules(&intention_nine, &intention_ten);
        assert_eq!(
            correct_winner.0.vehicle.borrow().id,
            actual_winner.0.vehicle.borrow().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 6: First vehicle is blocking its lane, second is changing lane
        // Winner: first vehicle (who is blocking the lane)
        let vehicle_eleven = Vehicle::new(11)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build_ref();
        vehicle_eleven.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::Block,
            ..Default::default()
        });
        let vehicle_twelve = Vehicle::new(12)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build_ref();
        vehicle_twelve.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            ..Default::default()
        });
        let intention_eleven = CellIntention::new(vehicle_eleven, IntentionType::Target);
        let intention_twelve = CellIntention::new(vehicle_twelve, IntentionType::Target);
        let correct_winner = (intention_eleven.clone(), ConflictType::BlockLaneChange);
        let actual_winner = resolve_simple_rules(&intention_eleven, &intention_twelve);
        assert_eq!(
            correct_winner.0.vehicle.borrow().id,
            actual_winner.0.vehicle.borrow().id,
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );
    }
}
