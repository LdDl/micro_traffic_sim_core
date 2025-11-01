use super::*;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::conflict_zones::{ConflictEdge, ConflictWinnerType, ConflictZone, ConflictZoneID};
    use crate::behaviour::BehaviourType;
    use crate::agents::{Vehicle, VehicleIntention};
    use crate::intentions::{CellIntention, IntentionType, Intentions};
    use crate::grid::cell::{Cell, CellID};
    use crate::grid::road_network::GridRoads;
    use crate::maneuver::LaneChangeType;
    use std::collections::HashMap;

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

    #[test]
    fn test_find_cross_trajectories_no_right_maneuver() {
        // Case: Vehicle at cell with no right maneuver should return None
        let vehicle = Vehicle::new(1)
            .with_cell(10)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeLeft,
            intention_cell_id: 11,
            ..Default::default()
        });
        let cell_intention = CellIntention::new(vehicle, IntentionType::Target);
        let intention_cell = Cell::new(15).build();
        let collected_intentions = Intentions::new();
        let grid = GridRoads::new();
        let result = find_cross_trajectories_conflict_naive(
            &cell_intention, &intention_cell, &collected_intentions, &grid
        );
        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
    }

    #[test]
    fn test_find_cross_trajectories_no_forward_cell() {
        // Case: Vehicle at cell with no forward node should return None
        let vehicle = Vehicle::new(1)
            .with_cell(10)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            intention_cell_id: 15,
            ..Default::default()
        });
        let cell_intention = CellIntention::new(vehicle, IntentionType::Target);
        let intention_cell = Cell::new(15).build();
        let collected_intentions = Intentions::new();
        let mut grid = GridRoads::new();
        let cell = Cell::new(10)
            .with_forward_node(-1)  // No forward cell
            .build();
        grid.add_cell(cell.clone());
        let result = find_cross_trajectories_conflict_naive(
            &cell_intention, &intention_cell, &collected_intentions, &grid
        );
        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
    }

    #[test]
    fn test_find_cross_trajectories_no_conflict_same_direction() {
        // Test case where both vehicles go in same direction - no conflict
        let vehicle1 = Vehicle::new(1)
            .with_cell(4)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle1.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            intention_cell_id: 2,
            intention_speed: 2,
            ..Default::default()
        });

        let vehicle2 = Vehicle::new(2)
            .with_cell(1)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle2.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight, // Same direction - no crossing
            intention_cell_id: 5,
            intention_speed: 2,
            ..Default::default()
        });

        let mut net = GridRoads::new();
        net.add_cell(Cell::new(4).with_forward_node(1).build());
        net.add_cell(Cell::new(1).with_forward_node(5).build());
        net.add_cell(Cell::new(2).build());
        net.add_cell(Cell::new(5).build());

        let mut collected_intentions = Intentions::new();
        collected_intentions.add_intention(vehicle2, IntentionType::Target);

        let vehicle1_intention = CellIntention::new(vehicle1, IntentionType::Target);
        let intention_cell = Cell::new(2).build();

        let result = find_cross_trajectories_conflict_naive(
            &vehicle1_intention, &intention_cell, &collected_intentions, &net
        );

        assert!(result.is_ok());
        let conflict_info = result.unwrap();
        assert!(conflict_info.is_none(), "Should not detect conflict when both vehicles go same direction");
    }

    #[test]
    fn test_find_cross_trajectories_conflict_naive() {
        // A        B
        // |  L  R  |
        // |   \/   |
        // F   /\   F
        // |  /  \  |
        // X        Y

        // X, Y, A, B - positions
        // F - forward
        // L - left
        // R - right
        // Vehicle in position 'X'. Intention - position 'B'

        // 1 - X
        // 2 - A
        // 3 - Y
        // 4 - B
        let mut net = GridRoads::new();
        net.add_cell(Cell::new(1).with_forward_node(2).with_right_node(4).build());
        net.add_cell(Cell::new(2).build());
        net.add_cell(Cell::new(3).with_forward_node(4).with_left_node(1).build());
        net.add_cell(Cell::new(4).build());

        // Vehicle 1: At position X (cell 1), wants to go RIGHT to B (cell 4)
        let vehicle1 = Vehicle::new(1)
            .with_cell(1)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle1.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            intention_cell_id: 4,
            ..Default::default()
        });

        // Vehicle 2: At position Y (cell 3), wants to go LEFT to A (cell 2)
        let vehicle2 = Vehicle::new(2)
            .with_cell(3)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle2.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeLeft,
            intention_cell_id: 2,
            ..Default::default()
        });

        // Set up intentions - vehicle2 wants cell 2
        let mut collected_intentions = Intentions::new();
        collected_intentions.add_intention(vehicle2, IntentionType::Target);

        // Test vehicle1's intention to go to cell 4
        let vehicle1_intention = CellIntention::new(vehicle1, IntentionType::Target);
        let intention_cell = net.get_cell(&4).unwrap(); // Cell B

        let result = find_cross_trajectories_conflict_naive(
            &vehicle1_intention, intention_cell, &collected_intentions, &net
        );

        // Assertions
        assert!(result.is_ok(), "Should not return error");
        let conflict_info = result.unwrap();
        assert!(conflict_info.is_some(), "Should detect crossing trajectories conflict");
        
        let info = conflict_info.unwrap();
        assert_eq!(info.conflict_type, ConflictType::CrossLaneChange, "Should be cross lane change conflict");
        
        // // Check that both vehicles are in the conflict
        assert_eq!(info.vehicle_id, 1, "Vehicle 1 should be primary vehicle");
        assert_eq!(info.side_vehicle_id, 2, "Vehicle 2 should be side vehicle");
        
        // Vehicle 2 (left maneuver) should have priority over vehicle 1 (right maneuver)
        assert_eq!(info.priority_vehicle_id, 2, "Left maneuver should have priority over right maneuver");
    }
    
    #[test]
    fn test_find_conflict_type() {
        // Test 1: Tail vs Transit conflict - Tail should win
        let vehicle_one = Vehicle::new(1)
            .with_cell(10)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle_one.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 15,
            ..Default::default()
        });

        let vehicle_two = Vehicle::new(2)
            .with_cell(11)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle_two.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 15,
            ..Default::default()
        });
        let correct_id = vehicle_one.borrow().id;
        let intention_one = CellIntention::new(vehicle_one, IntentionType::Tail);
        let intention_two = CellIntention::new(vehicle_two, IntentionType::Transit);

        let (winner, conflict_type) = find_conflict_type(
            15,
            &HashMap::new(),
            &HashMap::new(),
            &intention_one,
            &intention_two
        );

        assert_eq!(winner.vehicle.borrow_mut().id, correct_id, "Tail intention should win over transit");
        assert_eq!(conflict_type, ConflictType::Tail, "Conflict type should be Tail");

        // Test 2: Conflict zone winner - First edge source should win
        let mut conflict_zones = HashMap::new();
        let mut cells_conflicts_zones = HashMap::new();

        let first_edge = ConflictEdge { source: 10, target: 15 };
        let second_edge = ConflictEdge { source: 11, target: 15 };
        let zone = ConflictZone::new(1, first_edge, second_edge)
            .with_winner_type(ConflictWinnerType::First)
            .build();
        conflict_zones.insert(1, zone);
        cells_conflicts_zones.insert(15, 1);

        let vehicle_three = Vehicle::new(3)
            .with_cell(10)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle_three.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 15,
            ..Default::default()
        });

        let vehicle_four = Vehicle::new(4)
            .with_cell(11)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle_four.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 15,
            ..Default::default()
        });

        let correct_id = vehicle_three.borrow().id;
        let intention_three = CellIntention::new(vehicle_three, IntentionType::Target);
        let intention_four = CellIntention::new(vehicle_four, IntentionType::Target);

        let (winner, conflict_type) = find_conflict_type(
            15,
            &conflict_zones,
            &cells_conflicts_zones,
            &intention_three,
            &intention_four
        );

        assert_eq!(winner.vehicle.borrow().id, correct_id, "First edge source vehicle should win");
        assert_eq!(conflict_type, ConflictType::MergeForwardConflictZone, "Conflict type should be MergeForwardConflictZone");

        // Test 3: Merge lane change - Left maneuver should have priority over right
        let vehicle_five = Vehicle::new(5)
            .with_cell(20)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle_five.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeLeft,
            intention_cell_id: 25,
            ..Default::default()
        });

        let vehicle_six = Vehicle::new(6)
            .with_cell(21)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle_six.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            intention_cell_id: 25,
            ..Default::default()
        });

        let correct_id = vehicle_five.borrow().id;
        let intention_five = CellIntention::new(vehicle_five, IntentionType::Target);
        let intention_six = CellIntention::new(vehicle_six, IntentionType::Target);

        let (winner, conflict_type) = find_conflict_type(
            25,
            &HashMap::new(),
            &HashMap::new(),
            &intention_five,
            &intention_six
        );

        assert_eq!(winner.vehicle.borrow().id, correct_id, "Left maneuver should have priority over right");
        assert_eq!(conflict_type, ConflictType::MergeLaneChange, "Conflict type should be MergeLaneChange");

        // Test 4: Aggressive vs Cooperative behavior
        let vehicle_seven = Vehicle::new(7)
            .with_cell(30)
            .with_behaviour(BehaviourType::Aggressive)
            .with_speed(3)
            .build_ref();
        vehicle_seven.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 35,
            ..Default::default()
        });

        let vehicle_eight = Vehicle::new(8)
            .with_cell(31)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle_eight.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 35,
            ..Default::default()
        });

        let correct_id = vehicle_seven.borrow().id;
        let intention_seven = CellIntention::new(vehicle_seven, IntentionType::Target);
        let intention_eight = CellIntention::new(vehicle_eight, IntentionType::Target);

        let (winner, conflict_type) = find_conflict_type(
            35,
            &HashMap::new(),
            &HashMap::new(),
            &intention_seven,
            &intention_eight
        );

        assert_eq!(winner.vehicle.borrow().id, correct_id, "Aggressive vehicle should win over cooperative");
        assert_eq!(conflict_type, ConflictType::MergeForward, "Conflict type should be MergeForward");
    }

    #[test]
    fn test_new_conflict_multiple_basic() {
        let vehicle1 = Vehicle::new(1)
            .with_cell(10)
            .with_behaviour(BehaviourType::Aggressive)
            .with_speed(2)
            .build_ref();
        vehicle1.borrow_mut().set_intention(VehicleIntention {
            intention_cell_id: 15,
            intention_maneuver: LaneChangeType::NoChange,
            ..Default::default()
        });

        let vehicle2 = Vehicle::new(2)
            .with_cell(11)
            .with_behaviour(BehaviourType::Cooperative)
            .with_speed(2)
            .build_ref();
        vehicle2.borrow_mut().set_intention(VehicleIntention {
            intention_cell_id: 15,
            intention_maneuver: LaneChangeType::NoChange,
            ..Default::default()
        });

        let intentions = vec![
            CellIntention::new(vehicle1.clone(), IntentionType::Target),
            CellIntention::new(vehicle2.clone(), IntentionType::Target),
        ];

        let cell = Cell::new(15).build();
        let conflict_zones = HashMap::new();
        let cells_conflicts_zones = HashMap::new();

        let result = new_conflict_multiple(&cell, &conflict_zones, &cells_conflicts_zones, &intentions);
        
        assert!(result.is_ok());
        let conflict = result.unwrap();
        
        assert_eq!(conflict.cell_id, 15);
        assert_eq!(conflict.participants.len(), 2);
        assert_eq!(conflict.conflict_type, ConflictType::MergeForward);
        
        // Aggressive vehicle should have priority
        let priority_vehicle = &conflict.participants[conflict.priority_participant_index];
        assert_eq!(priority_vehicle.borrow().id, 1);
    }

    #[test]
    fn test_new_conflict_multiple_deduplication() {
        let vehicle1 = Vehicle::new(1)
            .with_cell(10)
            .build_ref();

        // Same vehicle appears in multiple intentions
        let intentions = vec![
            CellIntention::new(vehicle1.clone(), IntentionType::Target),
            CellIntention::new(vehicle1.clone(), IntentionType::Transit),
        ];

        let cell = Cell::new(15).build();
        let result = new_conflict_multiple(&cell, &HashMap::new(), &HashMap::new(), &intentions);
        
        assert!(result.is_ok());
        let conflict = result.unwrap();
        
        assert_eq!(conflict.conflict_type, ConflictType::SelfTail);
    }
}
