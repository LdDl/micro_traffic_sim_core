use crate::agents::{Vehicle, VehicleID};
use crate::conflicts::conflict_types::ConflictType;
use crate::intentions::{CellIntention, IntentionType};
use crate::maneuver::LaneChangeType;
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
    [
        |v1, v2| changing_lane(v1) && changing_lane(v2),
        resolve_merge_lane_change
    ],
    // Both vehicles are moving forward on different lanes of different roads
    // Differet roads are going to merge into single road
    [
        |v1, v2| v1.intention.intention_maneuver == LaneChangeType::NoChange
            && v2.intention.intention_maneuver == LaneChangeType::NoChange,
        resolve_merge_forward
    ],
    // First vehicle is moving forward, second is changing lane.
    // The forward vehicle normally has priority (the changer gives way), UNLESS the changer is a
    // CUT-IN aggressor that out-aggresses it (`aggressor_advantage`): then the aggressor squeezes
    // in and the forward vehicle is the one forced to brake. Equal aggression keeps priority with
    // the forward vehicle (the comparison is strict) - "higher aggression wins, ties go straight".
    [
        |v1, v2| v1.intention.intention_maneuver == LaneChangeType::NoChange && changing_lane(v2),
        |cin1, cin2, vehicles| {
            let v_forward = vehicles
                .get(&cin1.get_vehicle_id())
                .expect("Vehicle not found");
            let v_changer = vehicles
                .get(&cin2.get_vehicle_id())
                .expect("Vehicle not found");
            if aggressor_advantage(v_changer, v_forward) {
                (cin2, ConflictType::ForwardLaneChange)
            } else {
                (cin1, ConflictType::ForwardLaneChange)
            }
        }
    ],
    // First vehicle is changing lane, second is moving forward.
    // Symmetric to the rule above: the forward vehicle keeps priority unless the changer is a
    // cut-in aggressor that out-aggresses it.
    [
        |v1, v2| changing_lane(v1) && v2.intention.intention_maneuver == LaneChangeType::NoChange,
        |cin1, cin2, vehicles| {
            let v_changer = vehicles
                .get(&cin1.get_vehicle_id())
                .expect("Vehicle not found");
            let v_forward = vehicles
                .get(&cin2.get_vehicle_id())
                .expect("Vehicle not found");
            if aggressor_advantage(v_changer, v_forward) {
                (cin1, ConflictType::ForwardLaneChange)
            } else {
                (cin2, ConflictType::ForwardLaneChange)
            }
        }
    ],
    // First vehicle is changing lane, second is blocking its lane
    [
        |v1, v2| changing_lane(v1) && v2.intention.intention_maneuver == LaneChangeType::Block,
        |_cin1, cin2, _| {
            // Second vehicle is not moving therefore it holds the position
            (cin2, ConflictType::BlockLaneChange)
        }
    ],
    // First vehicle is blocking its lane, second is changing lane
    [
        |v1, v2| v1.intention.intention_maneuver == LaneChangeType::Block && changing_lane(v2),
        |cin1, _cin2, _| {
            // First vehicle is not moving therefore it holds the position
            (cin1, ConflictType::BlockLaneChange)
        }
    ]
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
    let v1 = vehicles
        .get(&intention_one.get_vehicle_id())
        .expect("Vehicle not found");
    let v2 = vehicles
        .get(&intention_two.get_vehicle_id())
        .expect("Vehicle not found");
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
    let vehicle_one = vehicles
        .get(&intention_one.get_vehicle_id())
        .expect("Vehicle not found");
    let vehicle_two = vehicles
        .get(&intention_two.get_vehicle_id())
        .expect("Vehicle not found");

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
        (LaneChangeType::ChangeLeft, LaneChangeType::ChangeLeft)
        | (LaneChangeType::ChangeRight, LaneChangeType::ChangeRight) => {
            // Distinct branches can converge through equally directed maneuvers.
            // Without an opposing left/right priority, use the ordinary merge tie-break.
            let (winner, _) =
                resolve_by_speed_and_cooperativity(intention_one, intention_two, vehicles);
            (winner, ConflictType::MergeLaneChange)
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
    let vehicle_one = vehicles
        .get(&intention_one.get_vehicle_id())
        .expect("Vehicle not found");
    let vehicle_two = vehicles
        .get(&intention_two.get_vehicle_id())
        .expect("Vehicle not found");

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
    let vehicle_one = vehicles
        .get(&intention_one.get_vehicle_id())
        .expect("Vehicle not found");
    let vehicle_two = vehicles
        .get(&intention_two.get_vehicle_id())
        .expect("Vehicle not found");

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
#[path = "tests.rs"]
mod tests;
