use crate::agents::{Vehicle, VehicleID};
use crate::conflicts::ConflictType;
use crate::maneuver::LaneChangeType;
use crate::intentions::{CellIntention, IntentionType};
use indexmap::IndexMap;

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
        &IndexMap<VehicleID, Vehicle>,
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

    // First vehicle is moving forward, second is changing lane.
    // The forward vehicle normally has priority (the changer gives way), UNLESS the changer is a
    // CUT-IN aggressor that out-aggresses it (`aggressor_advantage`): then the aggressor squeezes
    // in and the forward vehicle is the one forced to brake. Equal aggression keeps priority with
    // the forward vehicle (the comparison is strict) - "higher aggression wins, ties go straight".
    [ |v1, v2| v1.intention.intention_maneuver == LaneChangeType::NoChange && changing_lane(v2), |cin1, cin2, vehicles| {
        let v_forward = vehicles.get(&cin1.get_vehicle_id()).expect("Vehicle not found");
        let v_changer = vehicles.get(&cin2.get_vehicle_id()).expect("Vehicle not found");
        if aggressor_advantage(v_changer, v_forward) {
            (cin2, ConflictType::ForwardLaneChange)
        } else {
            (cin1, ConflictType::ForwardLaneChange)
        }
    } ],

    // First vehicle is changing lane, second is moving forward.
    // Symmetric to the rule above: the forward vehicle keeps priority unless the changer is a
    // cut-in aggressor that out-aggresses it.
    [ |v1, v2| changing_lane(v1) && v2.intention.intention_maneuver == LaneChangeType::NoChange, |cin1, cin2, vehicles| {
        let v_changer = vehicles.get(&cin1.get_vehicle_id()).expect("Vehicle not found");
        let v_forward = vehicles.get(&cin2.get_vehicle_id()).expect("Vehicle not found");
        if aggressor_advantage(v_changer, v_forward) {
            (cin1, ConflictType::ForwardLaneChange)
        } else {
            (cin2, ConflictType::ForwardLaneChange)
        }
    } ],

    // First vehicle is changing lane, second is blocking its lane
    [ |v1, v2| changing_lane(v1) && v2.intention.intention_maneuver == LaneChangeType::Block, |_cin1, cin2, _| {
        // Second vehicle is not moving therefore it holds the position
        (cin2, ConflictType::BlockLaneChange)
    } ],

    // First vehicle is blocking its lane, second is changing lane
    [ |v1, v2| v1.intention.intention_maneuver == LaneChangeType::Block && changing_lane(v2), |cin1, _cin2, _| {
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
    vehicles: &IndexMap<VehicleID, Vehicle>,
) -> (&'a CellIntention, ConflictType) {
    let v1 = vehicles.get(&intention_one.get_vehicle_id()).expect("Vehicle not found");
    let v2 = vehicles.get(&intention_two.get_vehicle_id()).expect("Vehicle not found");
    for rule in CONFLICT_RULES {
        if (rule.condition)(v1, v2) {
            return (rule.resolver)(intention_one, intention_two, vehicles);
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

/// True when `vehicle_one` is a CUT-IN aggressor that out-aggresses `vehicle_two`: its
/// aggressive_level exceeds `AGGRESSOR_CUT_IN_THRESHOLD` (i.e. cooperativity < (1 - AGGRESSOR_CUT_IN_THRESHOLD))
/// AND is STRICTLY greater than the other's. Such a vehicle wins a contested cell even against the
/// normal right-of-way rules (it "cuts in"; the other must brake).
/// The comparison is a gradient over `aggressive_level` (= 1 - cooperativity),
/// replacing the old coarse "is Aggressive enum vs Cooperative enum" check,
/// so e.g. an aggressor still loses to an even more aggressive driver.
/// Strictness matters: equal aggression yields NO advantage to either side, so the conflict falls
/// through to the normal rule (faster / less-cooperative / left-beats-right / forward-has-priority).
pub fn aggressor_advantage(vehicle_one: &Vehicle, vehicle_two: &Vehicle) -> bool {
    vehicle_one.is_aggressor() && vehicle_one.aggressive_level() > vehicle_two.aggressive_level()
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
    vehicles: &IndexMap<VehicleID, Vehicle>,
) -> (&'a CellIntention, ConflictType) {
    // Extract vehicles once (with early panic if missing)
    let vehicle_one = vehicles.get(&intention_one.get_vehicle_id()).expect("Vehicle not found");
    let vehicle_two = vehicles.get(&intention_two.get_vehicle_id()).expect("Vehicle not found");

    // Check aggressive behaviour advantage, so aggressive vehicle could even violate the traffic rules
    // and have advantage over the cooperative vehicle
    if aggressor_advantage(&vehicle_one, &vehicle_two) {
        return (intention_one, ConflictType::MergeLaneChange);
    }
    if aggressor_advantage(&vehicle_two, &vehicle_one) {
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
/// Priority: Faster > Less cooperative > Lowest id (deterministic, order-invariant tie-break)
/// ```
pub fn resolve_by_speed_and_cooperativity<'a>(
    intention_one: &'a CellIntention,
    intention_two: &'a CellIntention,
    vehicles: &IndexMap<VehicleID, Vehicle>,
) -> (&'a CellIntention, ConflictType) {
    let vehicle_one = vehicles.get(&intention_one.get_vehicle_id()).expect("Vehicle not found");
    let vehicle_two = vehicles.get(&intention_two.get_vehicle_id()).expect("Vehicle not found");

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
        // Full tie (equal speed AND equal cooperativity): break it DETERMINISTICALLY by the
        // LOWEST vehicle id, NOT by a coin flip. A coin flip is reproducible under a fixed seed
        // but is tied to participant POSITION, so the winner of a 3+ vehicle conflict would depend
        // on the order participants happen to sit in the left-fold (`new_conflict_multiple`).
        // Lowest-id makes "beats" a TOTAL order -> the fold's winner is order-invariant, and it
        // mirrors the desperate-override tie-break (see `solve_conflicts`).
        if vehicle_one.id <= vehicle_two.id {
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
    vehicles: &IndexMap<VehicleID, Vehicle>,
) -> (&'a CellIntention, ConflictType) {
    // Extract vehicles once (with early panic if missing)
    let vehicle_one = vehicles.get(&intention_one.get_vehicle_id()).expect("Vehicle not found");
    let vehicle_two = vehicles.get(&intention_two.get_vehicle_id()).expect("Vehicle not found");

    // Check aggressive behaviour advantage
    if aggressor_advantage(&vehicle_one, &vehicle_two) {
        return (intention_one, ConflictType::MergeForward);
    }
    if aggressor_advantage(&vehicle_two, &vehicle_one) {
        return (intention_two, ConflictType::MergeForward);
    }

    // Check intention types
    match (intention_one.int_type, intention_two.int_type) {
        // Both have same intention type (both target or both transit)
        (IntentionType::Target, IntentionType::Target)
        | (IntentionType::Transit, IntentionType::Transit) => {
            resolve_by_speed_and_cooperativity(intention_one, intention_two, vehicles)
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
    use crate::behaviour::BehaviourType;
    #[test]
    fn test_resolve_merge_lane_change() {
        // Case 1: Aggressive vehicle should win
        let vehicle_one = Vehicle::new(1)
            .with_behaviour(BehaviourType::Aggressive)
            .with_aggressive_level(0.9) // aggressive_level > 0.8 -> cut-in aggressor
            .with_speed(1)
            .build();
        let vehicle_two = Vehicle::new(2)
            .with_behaviour(BehaviourType::Cooperative)
            .with_aggressive_level(0.0) // fully cooperative -> out-aggressed
            .with_speed(3)
            .build();
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(1, vehicle_one);
        vehicles.insert(2, vehicle_two);
        let intention_one = CellIntention::new(1, IntentionType::Target);
        let intention_two = CellIntention::new(2, IntentionType::Target);
        let correct_winner = (intention_one.clone(), ConflictType::MergeLaneChange);
        let actual_winner = resolve_merge_lane_change(&intention_one, &intention_two, &vehicles);
        assert_eq!(
            correct_winner.0.get_vehicle_id(),
            actual_winner.0.get_vehicle_id(),
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
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(1, vehicle_one);
        vehicles.insert(2, vehicle_two);
        let intention_one = CellIntention::new(1, IntentionType::Target);
        let intention_two = CellIntention::new(2, IntentionType::Target);
        let correct_winner = (intention_one.clone(), ConflictType::MergeLaneChange);
        let actual_winner = resolve_merge_lane_change(&intention_one, &intention_two, &vehicles);
        assert_eq!(
            correct_winner.0.get_vehicle_id(),
            actual_winner.0.get_vehicle_id(),
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
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(1, vehicle_one);
        vehicles.insert(2, vehicle_two);
        let intention_one = CellIntention::new(1, IntentionType::Target);
        let intention_two = CellIntention::new(2, IntentionType::Target);
        let correct_winner = (intention_one.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_by_speed_and_cooperativity(&intention_one, &intention_two, &vehicles);
        assert_eq!(
            correct_winner.0.get_vehicle_id(),
            actual_winner.0.get_vehicle_id(),
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
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(3, vehicle_three);
        vehicles.insert(4, vehicle_four);
        let intention_three = CellIntention::new(3, IntentionType::Target);
        let intention_four = CellIntention::new(4, IntentionType::Target);
        let correct_winner = (intention_three.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_by_speed_and_cooperativity(&intention_three, &intention_four, &vehicles);
        assert_eq!(
            correct_winner.0.get_vehicle_id(),
            actual_winner.0.get_vehicle_id(),
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 3: Full tie (equal speed, equal cooperativity) -> LOWEST id wins, and the result
        // is invariant to argument order (no coin flip).
        let vehicle_five = Vehicle::new(5)
            .with_speed(3)
            .with_cooperative_level(0.5)
            .build();
        let vehicle_six = Vehicle::new(6)
            .with_speed(3)
            .with_cooperative_level(0.5)
            .build();
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(5, vehicle_five);
        vehicles.insert(6, vehicle_six);
        let intention_five = CellIntention::new(5, IntentionType::Target);
        let intention_six = CellIntention::new(6, IntentionType::Target);
        // Lower id (5) wins regardless of which intention is passed first.
        let (w1, t1) = resolve_by_speed_and_cooperativity(&intention_five, &intention_six, &vehicles);
        assert_eq!(w1.get_vehicle_id(), 5, "full tie -> lowest id (5) wins");
        assert_eq!(t1, ConflictType::MergeForward, "Conflict type is not correct");
        let (w2, _) = resolve_by_speed_and_cooperativity(&intention_six, &intention_five, &vehicles);
        assert_eq!(w2.get_vehicle_id(), 5, "full-tie tie-break is order-invariant (same winner when args are swapped)");
    }
    #[test]
    fn test_resolve_merge_forward() {
        // Case 1: Aggressive vehicle should win over cooperative
        let vehicle_one = Vehicle::new(1)
            .with_behaviour(BehaviourType::Aggressive)
            // aggressive_level > 0.8 -> cut-in aggressor
            .with_aggressive_level(0.9)
            .with_speed(3)
            .build();
        let vehicle_two = Vehicle::new(2)
            .with_behaviour(BehaviourType::Cooperative)
            // fully cooperative -> out-aggressed
            .with_aggressive_level(0.0)
            .with_speed(3)
            .build();
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(1, vehicle_one);
        vehicles.insert(2, vehicle_two);
        let intention_one = CellIntention::new(1, IntentionType::Target);
        let intention_two = CellIntention::new(2, IntentionType::Target);
        let coorect_winner = (intention_one.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_merge_forward(&intention_one, &intention_two, &vehicles);
        assert_eq!(
            coorect_winner.0.get_vehicle_id(),
            actual_winner.0.get_vehicle_id(),
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
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(3, vehicle_three);
        vehicles.insert(4, vehicle_four);
        let intention_three = CellIntention::new(3, IntentionType::Target);
        let intention_four = CellIntention::new(4, IntentionType::Transit);
        let correct_winner = (intention_four.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_merge_forward(&intention_three, &intention_four, &vehicles);
        assert_eq!(
            correct_winner.0.get_vehicle_id(),
            actual_winner.0.get_vehicle_id(),
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
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(5, vehicle_five);
        vehicles.insert(6, vehicle_six);
        let intention_five = CellIntention::new(5, IntentionType::Target);
        let intention_six = CellIntention::new(6, IntentionType::Target);
        let correct_winner = (intention_five.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_merge_forward(&intention_five, &intention_six, &vehicles);
        assert_eq!(
            correct_winner.0.get_vehicle_id(),
            actual_winner.0.get_vehicle_id(),
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
        let mut vehicle_one = Vehicle::new(1)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build();
        vehicle_one.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            ..Default::default()
        });
        let mut vehicle_two = Vehicle::new(2)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build();
        vehicle_two.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeLeft,
            ..Default::default()
        });
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(1, vehicle_one);
        vehicles.insert(2, vehicle_two);
        let intention_one = CellIntention::new(1, IntentionType::Target);
        let intention_two = CellIntention::new(2, IntentionType::Target);
        let correct_winner = (intention_two.clone(), ConflictType::MergeLaneChange);
        let actual_winner = resolve_simple_rules(&intention_one, &intention_two, &vehicles);
        assert_eq!(
            correct_winner.0.get_vehicle_id(),
            actual_winner.0.get_vehicle_id(),
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 2: both vehicles are moving forward on different lanes of different roads
        // Differet roads are going to merge into single road
        // Winner: vehicle who is moving faster
        let mut vehicle_three = Vehicle::new(3)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(5)
            .build();
        vehicle_three.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            ..Default::default()
        });
        let mut vehicle_four = Vehicle::new(4)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build();
        vehicle_four.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            ..Default::default()
        });
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(3, vehicle_three);
        vehicles.insert(4, vehicle_four);
        let intention_three = CellIntention::new(3, IntentionType::Target);
        let intention_four = CellIntention::new(4, IntentionType::Target);
        let correct_winner = (intention_three.clone(), ConflictType::MergeForward);
        let actual_winner = resolve_simple_rules(&intention_three, &intention_four, &vehicles);
        assert_eq!(
            correct_winner.0.get_vehicle_id(),
            actual_winner.0.get_vehicle_id(),
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 3: First vehicle is moving forward, second is changing lane
        // Winner: first vehicle (who is moving forward)
        let mut vehicle_five = Vehicle::new(5)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build();
        vehicle_five.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            ..Default::default()
        });
        let mut vehicle_six = Vehicle::new(6)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build();
        vehicle_six.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            ..Default::default()
        });
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(5, vehicle_five);
        vehicles.insert(6, vehicle_six);
        let intention_five = CellIntention::new(5, IntentionType::Target);
        let intention_six = CellIntention::new(6, IntentionType::Target);
        let correct_winner = (intention_five.clone(), ConflictType::ForwardLaneChange);  
        let actual_winner = resolve_simple_rules(&intention_five, &intention_six, &vehicles);
        assert_eq!(
            correct_winner.0.get_vehicle_id(),
            actual_winner.0.get_vehicle_id(),
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 4: First vehicle is changing lane, second is moving forward
        // Winner: second vehicle (who is moving forward)
        let mut vehicle_seven = Vehicle::new(7)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build();
        vehicle_seven.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            ..Default::default()
        });
        let mut vehicle_eight = Vehicle::new(8)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build();
        vehicle_eight.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            ..Default::default()
        });
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(7, vehicle_seven);
        vehicles.insert(8, vehicle_eight);
        let intention_seven = CellIntention::new(7, IntentionType::Target);
        let intention_eight = CellIntention::new(8, IntentionType::Target);
        let correct_winner = (intention_eight.clone(), ConflictType::ForwardLaneChange);
        let actual_winner = resolve_simple_rules(&intention_seven, &intention_eight, &vehicles);
        assert_eq!(
            correct_winner.0.get_vehicle_id(),
            actual_winner.0.get_vehicle_id(),
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 5: First vehicle is changing lane, second is blocking its lane
        // Winner: second vehicle (who is blocking the lane)
        let mut vehicle_nine = Vehicle::new(9)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build();
        vehicle_nine.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeLeft,
            ..Default::default()
        });
        let mut vehicle_ten = Vehicle::new(10)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build();
        vehicle_ten.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::Block,
            ..Default::default()
        });
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(9, vehicle_nine);
        vehicles.insert(10, vehicle_ten);
        let intention_nine = CellIntention::new(9, IntentionType::Target);
        let intention_ten = CellIntention::new(10, IntentionType::Target);
        let correct_winner = (intention_ten.clone(), ConflictType::BlockLaneChange);
        let actual_winner = resolve_simple_rules(&intention_nine, &intention_ten, &vehicles);
        assert_eq!(
            correct_winner.0.get_vehicle_id(),
            actual_winner.0.get_vehicle_id(),
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );

        // Case 6: First vehicle is blocking its lane, second is changing lane
        // Winner: first vehicle (who is blocking the lane)
        let mut vehicle_eleven = Vehicle::new(11)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build();
        vehicle_eleven.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::Block,
            ..Default::default()
        });
        let mut vehicle_twelve = Vehicle::new(12)
            .with_behaviour(BehaviourType::Undefined)
            .with_speed(3)
            .build();
        vehicle_twelve.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            ..Default::default()
        });
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(11, vehicle_eleven);
        vehicles.insert(12, vehicle_twelve);
        let intention_eleven = CellIntention::new(11, IntentionType::Target);
        let intention_twelve = CellIntention::new(12, IntentionType::Target);
        let correct_winner = (intention_eleven.clone(), ConflictType::BlockLaneChange);
        let actual_winner = resolve_simple_rules(&intention_eleven, &intention_twelve, &vehicles);
        assert_eq!(
            correct_winner.0.get_vehicle_id(),
            actual_winner.0.get_vehicle_id(),
            "Vehicle ID for the winner is not correct"
        );
        assert_eq!(
            correct_winner.1, actual_winner.1,
            "Conflict type is not correct"
        );
    }

    /// Builds a forward-moving (NoChange) or lane-changing vehicle with an explicit
    /// aggressive_level, for the cut-in tests below.
    fn fwd_or_changer(id: u64, aggressive_level: f64, maneuver: LaneChangeType) -> Vehicle {
        let mut v = Vehicle::new(id)
            .with_behaviour(BehaviourType::Undefined)
            .with_aggressive_level(aggressive_level)
            .with_speed(3)
            .build();
        v.set_intention(VehicleIntention {
            intention_maneuver: maneuver,
            ..Default::default()
        });
        v
    }

    /// A CUT-IN aggressor (aggressive_level > 0.8) changing lane WINS over a less aggressive
    /// vehicle moving straight - overturning the normal "forward traffic has priority" rule. With
    /// EQUAL aggression the forward vehicle keeps priority ("higher aggression wins, ties go
    /// straight"). Verified through the public `resolve_simple_rules` entry point.
    #[test]
    fn test_aggressor_cuts_in_forward_lane_change() {
        // changer (id 2, aggressive_level 0.9) cuts in front of the forward follower (id 1, 0.0).
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(1, fwd_or_changer(1, 0.0, LaneChangeType::NoChange));   // forward, meek
        vehicles.insert(2, fwd_or_changer(2, 0.9, LaneChangeType::ChangeRight)); // aggressor cutting in
        let i_fwd = CellIntention::new(1, IntentionType::Target);
        let i_chg = CellIntention::new(2, IntentionType::Target);
        let (winner, kind) = resolve_simple_rules(&i_fwd, &i_chg, &vehicles);
        assert_eq!(winner.get_vehicle_id(), 2, "aggressor cutting in wins over the meek forward vehicle");
        assert_eq!(kind, ConflictType::ForwardLaneChange);

        // Equal aggression (both 0.9): the forward vehicle keeps priority (strict comparison).
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(1, fwd_or_changer(1, 0.9, LaneChangeType::NoChange));
        vehicles.insert(2, fwd_or_changer(2, 0.9, LaneChangeType::ChangeRight));
        let i_fwd = CellIntention::new(1, IntentionType::Target);
        let i_chg = CellIntention::new(2, IntentionType::Target);
        let (winner, _) = resolve_simple_rules(&i_fwd, &i_chg, &vehicles);
        assert_eq!(winner.get_vehicle_id(), 1, "equal aggression -> the vehicle going straight wins");

        // A merely-assertive changer (0.8, NOT strictly above the 0.8 threshold) yields to a
        // meek forward vehicle: it is not a cut-in aggressor.
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(1, fwd_or_changer(1, 0.0, LaneChangeType::NoChange));
        vehicles.insert(2, fwd_or_changer(2, 0.8, LaneChangeType::ChangeLeft));
        let i_fwd = CellIntention::new(1, IntentionType::Target);
        let i_chg = CellIntention::new(2, IntentionType::Target);
        let (winner, _) = resolve_simple_rules(&i_fwd, &i_chg, &vehicles);
        assert_eq!(winner.get_vehicle_id(), 1, "aggressive_level == 0.8 is not above the threshold -> forward keeps priority");

        // Symmetric arrangement: the aggressor is the FIRST participant (changing left).
        let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
        vehicles.insert(1, fwd_or_changer(1, 0.9, LaneChangeType::ChangeLeft)); // aggressor cutting in
        vehicles.insert(2, fwd_or_changer(2, 0.0, LaneChangeType::NoChange));   // forward, meek
        let i_chg = CellIntention::new(1, IntentionType::Target);
        let i_fwd = CellIntention::new(2, IntentionType::Target);
        let (winner, kind) = resolve_simple_rules(&i_chg, &i_fwd, &vehicles);
        assert_eq!(winner.get_vehicle_id(), 1, "aggressor wins regardless of participant order");
        assert_eq!(kind, ConflictType::ForwardLaneChange);
    }
}
