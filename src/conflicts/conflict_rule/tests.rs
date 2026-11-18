use super::*;
use crate::agents::VehicleIntention;
use crate::behaviour::BehaviourType;
#[test]
fn test_resolve_merge_lane_change() {
    // Case 1: Aggressive vehicle should win
    let vehicle_one = Vehicle::new(1)
        .with_behaviour(BehaviourType::Aggressive)
        // aggressive_level > 0.8 -> cut-in aggressor
        .with_aggressive_level(0.9)
        .with_speed(1)
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
    let actual_winner =
        resolve_by_speed_and_cooperativity(&intention_one, &intention_two, &vehicles);
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
    let actual_winner =
        resolve_by_speed_and_cooperativity(&intention_three, &intention_four, &vehicles);
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
    assert_eq!(
        t1,
        ConflictType::MergeForward,
        "Conflict type is not correct"
    );
    let (w2, _) = resolve_by_speed_and_cooperativity(&intention_six, &intention_five, &vehicles);
    assert_eq!(
        w2.get_vehicle_id(),
        5,
        "full-tie tie-break is order-invariant (same winner when args are swapped)"
    );
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
    // forward, meek
    vehicles.insert(1, fwd_or_changer(1, 0.0, LaneChangeType::NoChange));
    // aggressor cutting in
    vehicles.insert(2, fwd_or_changer(2, 0.9, LaneChangeType::ChangeRight));
    let i_fwd = CellIntention::new(1, IntentionType::Target);
    let i_chg = CellIntention::new(2, IntentionType::Target);
    let (winner, kind) = resolve_simple_rules(&i_fwd, &i_chg, &vehicles);
    assert_eq!(
        winner.get_vehicle_id(),
        2,
        "aggressor cutting in wins over the meek forward vehicle"
    );
    assert_eq!(kind, ConflictType::ForwardLaneChange);

    // Equal aggression (both 0.9): the forward vehicle keeps priority (strict comparison).
    let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
    vehicles.insert(1, fwd_or_changer(1, 0.9, LaneChangeType::NoChange));
    vehicles.insert(2, fwd_or_changer(2, 0.9, LaneChangeType::ChangeRight));
    let i_fwd = CellIntention::new(1, IntentionType::Target);
    let i_chg = CellIntention::new(2, IntentionType::Target);
    let (winner, _) = resolve_simple_rules(&i_fwd, &i_chg, &vehicles);
    assert_eq!(
        winner.get_vehicle_id(),
        1,
        "equal aggression -> the vehicle going straight wins"
    );

    // A merely-assertive changer (0.8, NOT strictly above the 0.8 threshold) yields to a
    // meek forward vehicle: it is not a cut-in aggressor.
    let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
    vehicles.insert(1, fwd_or_changer(1, 0.0, LaneChangeType::NoChange));
    vehicles.insert(2, fwd_or_changer(2, 0.8, LaneChangeType::ChangeLeft));
    let i_fwd = CellIntention::new(1, IntentionType::Target);
    let i_chg = CellIntention::new(2, IntentionType::Target);
    let (winner, _) = resolve_simple_rules(&i_fwd, &i_chg, &vehicles);
    assert_eq!(
        winner.get_vehicle_id(),
        1,
        "aggressive_level == 0.8 is not above the threshold -> forward keeps priority"
    );

    // Symmetric arrangement: the aggressor is the FIRST participant (changing left).
    let mut vehicles: IndexMap<VehicleID, Vehicle> = IndexMap::new();
    // aggressor cutting in
    vehicles.insert(1, fwd_or_changer(1, 0.9, LaneChangeType::ChangeLeft));
    // forward, meek
    vehicles.insert(2, fwd_or_changer(2, 0.0, LaneChangeType::NoChange));
    let i_chg = CellIntention::new(1, IntentionType::Target);
    let i_fwd = CellIntention::new(2, IntentionType::Target);
    let (winner, kind) = resolve_simple_rules(&i_chg, &i_fwd, &vehicles);
    assert_eq!(
        winner.get_vehicle_id(),
        1,
        "aggressor wins regardless of participant order"
    );
    assert_eq!(kind, ConflictType::ForwardLaneChange);
}
