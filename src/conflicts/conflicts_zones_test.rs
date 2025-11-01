use super::*;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::conflict_zones::{ConflictEdge, ConflictWinnerType, ConflictZone};
    use crate::agents::{Vehicle, VehicleIntention, TailIntentionManeuver};
    use crate::intentions::{CellIntention, IntentionType, Intentions};
    use crate::geom::new_point;
    use crate::grid::cell::{Cell};
    use crate::grid::zones::ZoneType;
    use crate::grid::road_network::GridRoads;
    use crate::maneuver::LaneChangeType;
    use crate::verbose::{LocalLogger, VerboseLevel};
    use std::collections::{HashMap, HashSet};
    use crate::utils::test_grids::{
        create_conflict_zones_grid, create_conflict_zones_multiple_grid, create_simple_cross_shape_grid,
    };
    use crate::utils::generators::generate_one_lane_cells;
    #[test]
    fn test_find_conflicts_in_conflict_zones_trajectories() {
        let grid = create_conflict_zones_grid();

        let vehicle1 = Vehicle::new(1)
            .with_cell(9)
            .with_speed(2)
            .with_destination(12)
            .build_ref();
        vehicle1.borrow_mut().set_intention(VehicleIntention {
            intention_cell_id: 11,
            intermediate_cells: vec![10],
            ..Default::default()
        });

        let vehicle2 = Vehicle::new(2)
            .with_cell(3)
            .with_speed(2)
            .with_destination(6)
            .build_ref();
        vehicle2.borrow_mut().set_intention(VehicleIntention {
            intention_cell_id: 5,
            intermediate_cells: vec![4],
            ..Default::default()
        });

        let mut conflict_zones = HashMap::new();
        let zone = ConflictZone::new(
            1,
            ConflictEdge { source: 3, target: 4 },
            ConflictEdge { source: 9, target: 10 },
        )
        .with_winner_type(ConflictWinnerType::First)
        .build();
        conflict_zones.insert(1, zone);

        let mut cells_conflicts_zones = HashMap::new();
        cells_conflicts_zones.insert(10, 1);
        cells_conflicts_zones.insert(4, 1);

        let mut collected_intentions = Intentions::new();
        collected_intentions.add_intention(vehicle2.clone(), IntentionType::Target);

        let mut explored_conflict_zones = HashSet::new();

        let vehicle1_intention = CellIntention::new(vehicle1.clone(), IntentionType::Target);
        let intention_cell = grid.get_cell(&10).unwrap();

        let result = find_conflicts_in_conflict_zones(
            &vehicle1_intention,
            intention_cell,
            &collected_intentions,
            &conflict_zones,
            &cells_conflicts_zones,
            &mut explored_conflict_zones,
        );

        assert!(result.is_ok());
        let conflict_result = result.unwrap();
        assert!(conflict_result.is_some());

        let (conflict, conflict_zone_id) = conflict_result.unwrap();
        assert_eq!(conflict_zone_id, 1);
        assert_eq!(conflict.conflict_type, ConflictType::CrossConflictZone);
    }

    #[test]
    fn test_find_conflicts_in_conflict_zones_trajectories_invalidated() {
        let grid = create_conflict_zones_multiple_grid();

        let vehicle1 = Vehicle::new(1)
            .with_cell(9)
            .with_speed(2)
            .with_destination(12)
            .build_ref();
        vehicle1.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            intention_cell_id: 10,
            ..Default::default()
        });

        let vehicle2 = Vehicle::new(2)
            .with_cell(3)
            .with_speed(2)
            .with_destination(6)
            .build_ref();
        vehicle2.borrow_mut().set_intention(VehicleIntention {
            intention_cell_id: 5,
            intermediate_cells: vec![4],
            ..Default::default()
        });

        let vehicle3 = Vehicle::new(3)
            .with_cell(14)
            .with_speed(2)
            .with_destination(12)
            .build_ref();
        vehicle3.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 11,
            intermediate_cells: vec![10],
            ..Default::default()
        });

        let mut conflict_zones = HashMap::new();
        let zone = ConflictZone::new(
            1,
            ConflictEdge { source: 3, target: 4 },
            ConflictEdge { source: 9, target: 10 },
        )
        .with_winner_type(ConflictWinnerType::First)
        .build();
        conflict_zones.insert(1, zone);

        let mut cells_conflicts_zones = HashMap::new();
        cells_conflicts_zones.insert(10, 1);
        cells_conflicts_zones.insert(4, 1);

        let mut collected_intentions = Intentions::new();
        collected_intentions.add_intention(vehicle2.clone(), IntentionType::Target);
        collected_intentions.add_intention(vehicle3.clone(), IntentionType::Target);

        let mut explored_conflict_zones = HashSet::new();

        // Test vehicle1's intention to cell 10 (should detect cross conflict zone with vehicle2)
        let vehicle1_intention = CellIntention::new(vehicle1.clone(), IntentionType::Target);
        let intention_cell = grid.get_cell(&10).unwrap();

        let result = find_conflicts_in_conflict_zones(
            &vehicle1_intention,
            intention_cell,
            &collected_intentions,
            &conflict_zones,
            &cells_conflicts_zones,
            &mut explored_conflict_zones,
        );

        assert!(result.is_ok());
        let conflict_result = result.unwrap();
        assert!(conflict_result.is_some());

        let (conflict, conflict_zone_id) = conflict_result.unwrap();
        assert_eq!(conflict_zone_id, 1);
        assert_eq!(conflict.conflict_type, ConflictType::CrossConflictZone);
        assert_eq!(conflict.participants.len(), 2);

        // Vehicle2 should have priority (CONFLICT_WINNER_FIRST means first edge wins)
        let priority_vehicle = &conflict.participants[conflict.priority_participant_index];
        assert_eq!(priority_vehicle.borrow().id, 2);
    }

    #[test]
    fn test_find_conflicts_in_conflict_zones_cross_grid() {
        let grid = create_simple_cross_shape_grid();

        let vehicle1 = Vehicle::new(1)
            .with_cell(1)
            .with_speed(2)
            .with_destination(5)
            .build_ref();
        vehicle1.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 2,
            intermediate_cells: vec![3],
            ..Default::default()
        });

        let vehicle2 = Vehicle::new(2)
            .with_cell(6)
            .with_speed(2)
            .with_destination(9)
            .build_ref();
        vehicle2.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 3,
            intermediate_cells: vec![7],
            ..Default::default()
        });

        let mut conflict_zones = HashMap::new();
        let zone = ConflictZone::new(
            1,
            ConflictEdge { source: 2, target: 3 },
            ConflictEdge { source: 7, target: 3 },
        )
        .with_winner_type(ConflictWinnerType::Second)
        .build();
        conflict_zones.insert(1, zone);

        let mut cells_conflicts_zones = HashMap::new();
        cells_conflicts_zones.insert(3, 1);

        let mut collected_intentions = Intentions::new();
        collected_intentions.add_intention(vehicle2.clone(), IntentionType::Target);

        let mut explored_conflict_zones = HashSet::new();

        // Test vehicle1's intention to cell 2 (intermediate cell targeting same merge point)
        let vehicle1_intention = CellIntention::new(vehicle1.clone(), IntentionType::Target);
        let intention_cell = grid.get_cell(&2).unwrap();

        let result = find_conflicts_in_conflict_zones(
            &vehicle1_intention,
            intention_cell,
            &collected_intentions,
            &conflict_zones,
            &cells_conflicts_zones,
            &mut explored_conflict_zones,
        );

        // Since vehicle1 is targeting cell 2 and vehicle2 is targeting cell 3,
        // and cell 2 is not in conflict zone, there should be no conflict from this function
        assert!(result.is_ok());
        let conflict_result = result.unwrap();
        assert!(conflict_result.is_none());
    }

    #[test]
    fn test_find_conflicts_in_conflict_zones_cross_tail() {
        let cells_set = generate_one_lane_cells(31.5, 4.5, 3);
        let mut grid = GridRoads::new();
        for cell in cells_set {
            grid.add_cell(cell);
        }

        let vehicle1 = Vehicle::new(1)
            .with_cell(12)
            .with_speed(1)
            .with_destination(14)
            .with_tail_size(2, vec![3, 4])
            .build_ref();
        {
            let mut v1 = vehicle1.borrow_mut();
            v1.tail_cells = vec![3, 4];
            v1.set_intention(VehicleIntention {
                intention_cell_id: 13,
                tail_maneuver: TailIntentionManeuver {
                    source_cell_maneuver: 4,
                    target_cell_maneuver: 12,
                    intention_maneuver: LaneChangeType::ChangeRight,
                },
                ..Default::default()
            });
        }

        let vehicle2 = Vehicle::new(2)
            .with_cell(11)
            .with_speed(1)
            .with_destination(7)
            .build_ref();
        vehicle2.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeLeft,
            intention_cell_id: 5,
            ..Default::default()
        });

        let mut conflict_zones = HashMap::new();
        let zone = ConflictZone::new(
            1,
            ConflictEdge { source: 4, target: 12 },
            ConflictEdge { source: 11, target: 5 },
        )
        .with_winner_type(ConflictWinnerType::Second)
        .build();
        conflict_zones.insert(1, zone);

        let mut cells_conflicts_zones = HashMap::new();
        cells_conflicts_zones.insert(5, 1);
        cells_conflicts_zones.insert(12, 1);

        let mut collected_intentions = Intentions::new();
        collected_intentions.add_intention(vehicle2.clone(), IntentionType::Target);

        let mut explored_conflict_zones = HashSet::new();

        // Test vehicle1's tail intention to cell 12
        let vehicle1_tail_intention = CellIntention::new(vehicle1.clone(), IntentionType::Tail);
        let intention_cell = grid.get_cell(&12).unwrap();

        let result = find_conflicts_in_conflict_zones(
            &vehicle1_tail_intention,
            intention_cell,
            &collected_intentions,
            &conflict_zones,
            &cells_conflicts_zones,
            &mut explored_conflict_zones,
        );

        assert!(result.is_ok());
        let conflict_result = result.unwrap();
        assert!(conflict_result.is_some());

        let (conflict, conflict_zone_id) = conflict_result.unwrap();
        assert_eq!(conflict_zone_id, 1);
        assert_eq!(conflict.conflict_type, ConflictType::CrossConflictZone);
        assert_eq!(conflict.participants.len(), 2);

        // Vehicle1 should have priority because it has INTENTION_TAIL (tail conflicts have higher priority)
        let priority_vehicle = &conflict.participants[conflict.priority_participant_index];
        assert_eq!(priority_vehicle.borrow().id, 1);
    }

    #[test]
    fn test_find_conflicts_in_conflict_zones_cross_tail_forward() {
        let cells_set = vec![
            Cell::new(1)
                .with_point(new_point(1.0, 1.0, None))
                .with_zone_type(ZoneType::Birth)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(2)
                .with_right_node(-1)
                .with_meso_link(1)
                .build(),
            Cell::new(2)
                .with_point(new_point(2.0, 1.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(3)
                .with_right_node(-1)
                .with_meso_link(1)
                .build(),
            Cell::new(3)
                .with_point(new_point(3.0, 1.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(4)
                .with_right_node(-1)
                .with_meso_link(1)
                .build(),
            Cell::new(4)
                .with_point(new_point(4.0, 1.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(5)
                .with_right_node(-1)
                .with_meso_link(1)
                .build(),
            Cell::new(5)
                .with_point(new_point(5.0, 1.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(6)
                .with_right_node(-1)
                .with_meso_link(1)
                .build(),
            Cell::new(6)
                .with_point(new_point(6.0, 1.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(7)
                .with_right_node(-1)
                .with_meso_link(1)
                .build(),
            Cell::new(7)
                .with_point(new_point(7.0, 1.0, None))
                .with_zone_type(ZoneType::Death)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(-1)
                .with_right_node(-1)
                .with_meso_link(1)
                .build(),
            Cell::new(8)
                .with_point(new_point(1.0, 0.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(9)
                .with_right_node(-1)
                .with_meso_link(2)
                .build(),
            Cell::new(9)
                .with_point(new_point(2.0, 0.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(10)
                .with_right_node(-1)
                .with_meso_link(2)
                .build(),
            Cell::new(10)
                .with_point(new_point(3.0, 0.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(11)
                .with_right_node(-1)
                .with_meso_link(2)
                .build(),
            Cell::new(11)
                .with_point(new_point(4.0, 0.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(5)
                .with_right_node(-1)
                .with_meso_link(3)
                .build(),
            Cell::new(12)
                .with_point(new_point(5.0, 0.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(13)
                .with_right_node(-1)
                .with_meso_link(3)
                .build(),
            Cell::new(13)
                .with_point(new_point(6.0, 0.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(14)
                .with_right_node(-1)
                .with_meso_link(3)
                .build(),
            Cell::new(14)
                .with_point(new_point(7.0, 0.0, None))
                .with_zone_type(ZoneType::Death)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(-1)
                .with_right_node(-1)
                .with_meso_link(3)
                .build(),
        ];

        // Convert to grid
        let mut grid = GridRoads::new();
        for cell in cells_set {
            grid.add_cell(cell);
        }

        let vehicle1 = Vehicle::new(1)
            .with_cell(12)
            .with_speed(1)
            .with_destination(14)
            .with_tail_intention_maneuver(4, 12, LaneChangeType::ChangeRight)
            .with_tail_size(2, vec![3, 4])
            .build_ref();
        vehicle1.borrow_mut().set_intention(VehicleIntention {
            intention_cell_id: 13,
            ..Default::default()
        });

        let vehicle2 = Vehicle::new(2)
            .with_cell(11)
            .with_speed(1)
            .with_destination(7)
            .build_ref();
        vehicle2.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 5,
            ..Default::default()
        });

        let mut conflict_zones = HashMap::new();
        let zone = ConflictZone::new(
            1,
            ConflictEdge { source: 4, target: 12 },
            ConflictEdge { source: 11, target: 5 },
        )
        .with_winner_type(ConflictWinnerType::Second)
        .build();
        conflict_zones.insert(1, zone);

        let mut cells_conflicts_zones = HashMap::new();
        cells_conflicts_zones.insert(5, 1);
        cells_conflicts_zones.insert(12, 1);

        let mut collected_intentions = Intentions::new();
        collected_intentions.add_intention(vehicle2.clone(), IntentionType::Target);

        let mut explored_conflict_zones = HashSet::new();

        // Test vehicle1's tail intention to cell 12
        let vehicle1_tail_intention = CellIntention::new(vehicle1.clone(), IntentionType::Tail);
        let intention_cell = grid.get_cell(&12).unwrap();

        let result = find_conflicts_in_conflict_zones(
            &vehicle1_tail_intention,
            intention_cell,
            &collected_intentions,
            &conflict_zones,
            &cells_conflicts_zones,
            &mut explored_conflict_zones,
        );

        assert!(result.is_ok());
        let conflict_result = result.unwrap();
        assert!(conflict_result.is_some());

        let (conflict, conflict_zone_id) = conflict_result.unwrap();
        assert_eq!(conflict_zone_id, 1);
        assert_eq!(conflict.conflict_type, ConflictType::CrossConflictZone);
        assert_eq!(conflict.participants.len(), 2);

        // Vehicle1 should have priority because it has INTENTION_TAIL
        let priority_vehicle = &conflict.participants[conflict.priority_participant_index];
        assert_eq!(priority_vehicle.borrow().id, 1);
    }

    #[derive(Debug)]
    struct CorrectConflict {
        conflict_type: ConflictType,
        priority_participant_id: u64,
    }

    #[test]
    fn test_conflict_zones_trajectories() {
        // Based on TestConflictZonesTrajectories
        let verbose = LocalLogger::new(VerboseLevel::None);
        let grid = create_conflict_zones_grid();

        let vehicle1 = Vehicle::new(1)
            .with_cell(9)
            .with_speed(2)
            .with_speed_limit(3)
            .with_destination(12)
            .build_ref();
        vehicle1.borrow_mut().set_intention(VehicleIntention {
            intention_cell_id: 11,
            intermediate_cells: vec![10],
            ..Default::default()
        });

        let vehicle2 = Vehicle::new(2)
            .with_cell(3)
            .with_speed(2)
            .with_speed_limit(3)
            .with_destination(6)
            .build_ref();
        vehicle2.borrow_mut().set_intention(VehicleIntention {
            intention_cell_id: 5,
            intermediate_cells: vec![4],
            ..Default::default()
        });

        let mut conflict_zones = HashMap::new();
        conflict_zones.insert(1, ConflictZone::new(
            1,
            ConflictEdge { source: 3, target: 4 },
            ConflictEdge { source: 9, target: 10 },
        )
        .with_winner_type(ConflictWinnerType::First)
        .build());

        let mut cells_conflicts_zones = HashMap::new();
        cells_conflicts_zones.insert(10, 1);
        cells_conflicts_zones.insert(4, 1);

        let mut intentions_data = Intentions::new();
        intentions_data.add_intention(vehicle1, IntentionType::Target);
        intentions_data.add_intention(vehicle2, IntentionType::Target);

        let correct_conflicts = vec![
            CorrectConflict {
                conflict_type: ConflictType::CrossConflictZone,
                priority_participant_id: 2, // Vehicle 2 has priority (CONFLICT_WINNER_FIRST)
            }
        ];

        let conflicts_data = collect_conflicts(
            &intentions_data, 
            &grid, 
            &conflict_zones, 
            &cells_conflicts_zones, 
            &verbose
        );

        assert!(conflicts_data.is_ok());
        let conflicts = conflicts_data.unwrap();

        assert_eq!(correct_conflicts.len(), conflicts.len(), 
            "Incorrect number of conflicts. Expected {}, got {}", 
            correct_conflicts.len(), conflicts.len());

        for (idx, conflict) in conflicts.iter().enumerate() {
            println!("{}", conflict);
            assert_eq!(conflict.conflict_type, correct_conflicts[idx].conflict_type,
                "Incorrect conflict type at pos #{}. Expected {:?}, got {:?}", 
                idx, correct_conflicts[idx].conflict_type, conflict.conflict_type);
            
            let priority_participant_id = conflict.participants[conflict.priority_participant_index].borrow().id;
            assert_eq!(priority_participant_id, correct_conflicts[idx].priority_participant_id,
                "Incorrect conflict priority participant at pos #{}. Expected {}, got {}", 
                idx, correct_conflicts[idx].priority_participant_id, priority_participant_id);
        }
    }

    #[test]
    fn test_conflict_zones_trajectories_invalidated() {
        // Based on TestConflictZonesTrajectoriesInvalidated
        let verbose = LocalLogger::new(VerboseLevel::None);
        let grid = create_conflict_zones_multiple_grid();

        let vehicle1 = Vehicle::new(1)
            .with_cell(9)
            .with_speed(2)
            .with_speed_limit(3)
            .with_destination(12)
            .build_ref();
        vehicle1.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeRight,
            intention_cell_id: 10,
            ..Default::default()
        });

        let vehicle2 = Vehicle::new(2)
            .with_cell(3)
            .with_speed(2)
            .with_speed_limit(3)
            .with_destination(6)
            .build_ref();
        vehicle2.borrow_mut().set_intention(VehicleIntention {
            intention_cell_id: 5,
            intermediate_cells: vec![4],
            ..Default::default()
        });

        let vehicle3 = Vehicle::new(3)
            .with_cell(14)
            .with_speed(2)
            .with_speed_limit(3)
            .with_destination(12)
            .build_ref();
        vehicle3.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 11,
            intermediate_cells: vec![10],
            ..Default::default()
        });

        let mut conflict_zones = HashMap::new();
        conflict_zones.insert(1, ConflictZone::new(
            1,
            ConflictEdge { source: 3, target: 4 },
            ConflictEdge { source: 9, target: 10 },
        )
        .with_winner_type(ConflictWinnerType::First)
        .build());

        let mut cells_conflicts_zones = HashMap::new();
        cells_conflicts_zones.insert(10, 1);
        cells_conflicts_zones.insert(4, 1);

        let mut intentions_data = Intentions::new();
        intentions_data.add_intention(vehicle1, IntentionType::Target);
        intentions_data.add_intention(vehicle2, IntentionType::Target);
        intentions_data.add_intention(vehicle3, IntentionType::Target);

        let correct_conflicts = vec![
            CorrectConflict {
                conflict_type: ConflictType::ForwardLaneChange,
                priority_participant_id: 3, // Vehicle 3 has priority (forward vs lane change)
            },
            CorrectConflict {
                conflict_type: ConflictType::CrossConflictZone,
                priority_participant_id: 2, // Vehicle 2 has priority (CONFLICT_WINNER_FIRST)
            }
        ];

        let conflicts_data = collect_conflicts(
            &intentions_data, 
            &grid, 
            &conflict_zones, 
            &cells_conflicts_zones, 
            &verbose
        );

        assert!(conflicts_data.is_ok());
        let conflicts = conflicts_data.unwrap();

        if correct_conflicts.len() != conflicts.len() {
            for conflict in &conflicts {
                println!("{}", conflict);
            }
        }

        assert_eq!(correct_conflicts.len(), conflicts.len(), 
            "Incorrect number of conflicts. Expected {}, got {}", 
            correct_conflicts.len(), conflicts.len());

        for (idx, conflict) in conflicts.iter().enumerate() {
            println!("{}", conflict);
            assert_eq!(conflict.conflict_type, correct_conflicts[idx].conflict_type,
                "Incorrect conflict type at pos #{}. Expected {:?}, got {:?}", 
                idx, correct_conflicts[idx].conflict_type, conflict.conflict_type);
            
            let priority_participant_id = conflict.participants[conflict.priority_participant_index].borrow().id;
            assert_eq!(priority_participant_id, correct_conflicts[idx].priority_participant_id,
                "Incorrect conflict priority participant at pos #{}. Expected {}, got {}", 
                idx, correct_conflicts[idx].priority_participant_id, priority_participant_id);
        }

        let solve_result = solve_conflicts(conflicts, &verbose);
        assert!(solve_result.is_ok(), "Can't solve conflicts");
    }

    #[test]
    fn test_conflict_zones_cross_grid() {
        // Based on TestConflictZonesCrossGrid
        let verbose = LocalLogger::new(VerboseLevel::None);
        let grid = create_simple_cross_shape_grid();

        let vehicle1 = Vehicle::new(1)
            .with_cell(1)
            .with_speed(2)
            .with_speed_limit(3)
            .with_destination(5)
            .build_ref();
        vehicle1.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 2,
            intermediate_cells: vec![3],
            ..Default::default()
        });

        let vehicle2 = Vehicle::new(2)
            .with_cell(6)
            .with_speed(2)
            .with_speed_limit(3)
            .with_destination(9)
            .build_ref();
        vehicle2.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 3,
            intermediate_cells: vec![7],
            ..Default::default()
        });

        let mut conflict_zones = HashMap::new();
        conflict_zones.insert(1, ConflictZone::new(
            1,
            ConflictEdge { source: 2, target: 3 },
            ConflictEdge { source: 7, target: 3 },
        )
        .with_winner_type(ConflictWinnerType::Second)
        .build());

        let mut cells_conflicts_zones = HashMap::new();
        cells_conflicts_zones.insert(3, 1);

        let mut intentions_data = Intentions::new();
        intentions_data.add_intention(vehicle1, IntentionType::Target);
        intentions_data.add_intention(vehicle2, IntentionType::Target);

        let correct_conflicts = vec![
            CorrectConflict {
                conflict_type: ConflictType::MergeForwardConflictZone,
                priority_participant_id: 2, // Vehicle 2 has priority (CONFLICT_WINNER_SECOND)
            }
        ];

        let conflicts_data = collect_conflicts(
            &intentions_data, 
            &grid, 
            &conflict_zones, 
            &cells_conflicts_zones, 
            &verbose
        );

        assert!(conflicts_data.is_ok());
        let conflicts = conflicts_data.unwrap();

        if correct_conflicts.len() != conflicts.len() {
            for conflict in &conflicts {
                println!("{}", conflict);
            }
        }

        assert_eq!(correct_conflicts.len(), conflicts.len(), 
            "Incorrect number of conflicts. Expected {}, got {}", 
            correct_conflicts.len(), conflicts.len());

        for (idx, conflict) in conflicts.iter().enumerate() {
            println!("{}", conflict);
            assert_eq!(conflict.conflict_type, correct_conflicts[idx].conflict_type,
                "Incorrect conflict type at pos #{}. Expected {:?}, got {:?}", 
                idx, correct_conflicts[idx].conflict_type, conflict.conflict_type);
            
            let priority_participant_id = conflict.participants[conflict.priority_participant_index].borrow().id;
            assert_eq!(priority_participant_id, correct_conflicts[idx].priority_participant_id,
                "Incorrect conflict priority participant at pos #{}. Expected {}, got {}", 
                idx, correct_conflicts[idx].priority_participant_id, priority_participant_id);
        }

        let solve_result = solve_conflicts(conflicts, &verbose);
        assert!(solve_result.is_ok(), "Can't solve conflicts");
    }

    #[test]
    fn test_conflict_zones_cross_tail() {
        // Based on TestConflictZonesCrossTail
        let verbose = LocalLogger::new(VerboseLevel::None);

        // Generate 3x7 grid
        let cells_set = generate_one_lane_cells(31.5, 4.5, 3);
        let mut grid = GridRoads::new();
        for cell in cells_set {
            grid.add_cell(cell);
        }

        let vehicle1 = Vehicle::new(1)
            .with_cell(12)
            .with_speed(1)
            .with_speed_limit(3)
            .with_destination(14)
            .with_tail_size(2, vec![3, 4])
            .build_ref();
        {
            let mut v1 = vehicle1.borrow_mut();
            v1.set_intention(VehicleIntention {
                intention_cell_id: 13,
                tail_maneuver: TailIntentionManeuver {
                    source_cell_maneuver: 4,
                    target_cell_maneuver: 12,
                    intention_maneuver: LaneChangeType::ChangeRight,
                },
                ..Default::default()
            });
        }

        let vehicle2 = Vehicle::new(2)
            .with_cell(11)
            .with_speed(1)
            .with_speed_limit(3)
            .with_destination(7)
            .build_ref();
        vehicle2.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::ChangeLeft,
            intention_cell_id: 5,
            ..Default::default()
        });

        let mut conflict_zones = HashMap::new();
        conflict_zones.insert(1, ConflictZone::new(
            1,
            ConflictEdge { source: 4, target: 12 },
            ConflictEdge { source: 11, target: 5 },
        )
        .with_winner_type(ConflictWinnerType::Second)
        .build());

        let mut cells_conflicts_zones = HashMap::new();
        cells_conflicts_zones.insert(5, 1);
        cells_conflicts_zones.insert(12, 1);

        let mut intentions_data = Intentions::new();
        intentions_data.add_intention(vehicle1, IntentionType::Target);
        intentions_data.add_intention(vehicle2, IntentionType::Target);

        let correct_conflicts = vec![
            CorrectConflict {
                conflict_type: ConflictType::TailCrossLaneChange,
                priority_participant_id: 1, // Vehicle 1 has priority (tail conflict)
            }
        ];

        let conflicts_data = collect_conflicts(
            &intentions_data, 
            &grid, 
            &conflict_zones, 
            &cells_conflicts_zones, 
            &verbose
        );

        assert!(conflicts_data.is_ok());
        let conflicts = conflicts_data.unwrap();

        assert_eq!(correct_conflicts.len(), conflicts.len(), 
            "Incorrect number of conflicts. Expected {}, got {}", 
            correct_conflicts.len(), conflicts.len());

        for (idx, conflict) in conflicts.iter().enumerate() {
            println!("{}", conflict);
            assert_eq!(conflict.conflict_type, correct_conflicts[idx].conflict_type,
                "Incorrect conflict type at pos #{}. Expected {:?}, got {:?}", 
                idx, correct_conflicts[idx].conflict_type, conflict.conflict_type);
            
            let priority_participant_id = conflict.participants[conflict.priority_participant_index].borrow().id;
            assert_eq!(priority_participant_id, correct_conflicts[idx].priority_participant_id,
                "Incorrect conflict priority participant at pos #{}. Expected {}, got {}", 
                idx, correct_conflicts[idx].priority_participant_id, priority_participant_id);
        }
    }

    #[test]
    fn test_conflict_zones_cross_tail_forward() {
        // Based on TestConflictZonesCrossTailForward
        let verbose = LocalLogger::new(VerboseLevel::None);

        let cells_set = vec![
            Cell::new(1)
                .with_point(new_point(1.0, 1.0, None))
                .with_zone_type(ZoneType::Birth)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(2)
                .with_right_node(-1)
                .with_meso_link(1)
                .build(),
            Cell::new(2)
                .with_point(new_point(2.0, 1.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(3)
                .with_right_node(-1)
                .with_meso_link(1)
                .build(),
            Cell::new(3)
                .with_point(new_point(3.0, 1.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(4)
                .with_right_node(-1)
                .with_meso_link(1)
                .build(),
            Cell::new(4)
                .with_point(new_point(4.0, 1.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(5)
                .with_right_node(-1)
                .with_meso_link(1)
                .build(),
            Cell::new(5)
                .with_point(new_point(5.0, 1.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(6)
                .with_right_node(-1)
                .with_meso_link(1)
                .build(),
            Cell::new(6)
                .with_point(new_point(6.0, 1.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(7)
                .with_right_node(-1)
                .with_meso_link(1)
                .build(),
            Cell::new(7)
                .with_point(new_point(7.0, 1.0, None))
                .with_zone_type(ZoneType::Death)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(-1)
                .with_right_node(-1)
                .with_meso_link(1)
                .build(),
            Cell::new(8)
                .with_point(new_point(1.0, 0.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(9)
                .with_right_node(-1)
                .with_meso_link(2)
                .build(),
            Cell::new(9)
                .with_point(new_point(2.0, 0.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(10)
                .with_right_node(-1)
                .with_meso_link(2)
                .build(),
            Cell::new(10)
                .with_point(new_point(3.0, 0.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(11)
                .with_right_node(-1)
                .with_meso_link(2)
                .build(),
            Cell::new(11)
                .with_point(new_point(4.0, 0.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(5)
                .with_right_node(-1)
                .with_meso_link(3)
                .build(),
            Cell::new(12)
                .with_point(new_point(5.0, 0.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(13)
                .with_right_node(-1)
                .with_meso_link(3)
                .build(),
            Cell::new(13)
                .with_point(new_point(6.0, 0.0, None))
                .with_zone_type(ZoneType::Common)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(14)
                .with_right_node(-1)
                .with_meso_link(3)
                .build(),
            Cell::new(14)
                .with_point(new_point(7.0, 0.0, None))
                .with_zone_type(ZoneType::Death)
                .with_speed_limit(3)
                .with_left_node(-1)
                .with_forward_node(-1)
                .with_right_node(-1)
                .with_meso_link(3)
                .build(),
        ];

        let mut grid = GridRoads::new();
        for cell in cells_set {
            grid.add_cell(cell);
        }

        let vehicle1 = Vehicle::new(1)
            .with_cell(12)
            .with_speed(1)
            .with_speed_limit(3)
            .with_destination(14)
            .with_tail_size(2, vec![3, 4])
            .build_ref();
        {
            let mut v1 = vehicle1.borrow_mut();
            v1.set_intention(VehicleIntention {
                intention_cell_id: 13,
                tail_maneuver: TailIntentionManeuver {
                    source_cell_maneuver: 4,
                    target_cell_maneuver: 12,
                    intention_maneuver: LaneChangeType::ChangeRight,
                },
                ..Default::default()
            });
        }

        let vehicle2 = Vehicle::new(2)
            .with_cell(11)
            .with_speed(1)
            .with_speed_limit(3)
            .with_destination(7)
            .build_ref();
        vehicle2.borrow_mut().set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 5,
            ..Default::default()
        });

        let mut conflict_zones = HashMap::new();
        conflict_zones.insert(1, ConflictZone::new(
            1,
            ConflictEdge { source: 4, target: 12 },
            ConflictEdge { source: 11, target: 5 },
        )
        .with_winner_type(ConflictWinnerType::Second)
        .build());

        let mut cells_conflicts_zones = HashMap::new();
        cells_conflicts_zones.insert(5, 1);
        cells_conflicts_zones.insert(12, 1);

        let mut intentions_data = Intentions::new();
        intentions_data.add_intention(vehicle1, IntentionType::Target);
        intentions_data.add_intention(vehicle2, IntentionType::Target);

        let correct_conflicts = vec![
            CorrectConflict {
                conflict_type: ConflictType::CrossConflictZone,
                priority_participant_id: 1, // Vehicle 1 has priority (tail intention)
            }
        ];

        let conflicts_data = collect_conflicts(
            &intentions_data, 
            &grid, 
            &conflict_zones, 
            &cells_conflicts_zones, 
            &verbose
        );

        assert!(conflicts_data.is_ok());
        let conflicts = conflicts_data.unwrap();

        assert_eq!(correct_conflicts.len(), conflicts.len(), 
            "Incorrect number of conflicts. Expected {}, got {}", 
            correct_conflicts.len(), conflicts.len());

        for (idx, conflict) in conflicts.iter().enumerate() {
            println!("{}", conflict);
            assert_eq!(conflict.conflict_type, correct_conflicts[idx].conflict_type,
                "Incorrect conflict type at pos #{}. Expected {:?}, got {:?}", 
                idx, correct_conflicts[idx].conflict_type, conflict.conflict_type);
            
            let priority_participant_id = conflict.participants[conflict.priority_participant_index].borrow().id;
            assert_eq!(priority_participant_id, correct_conflicts[idx].priority_participant_id,
                "Incorrect conflict priority participant at pos #{}. Expected {}, got {}", 
                idx, correct_conflicts[idx].priority_participant_id, priority_participant_id);
        }
    }
}
