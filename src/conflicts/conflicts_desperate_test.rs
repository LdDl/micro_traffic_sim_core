//! Tests for the patience-based right-of-way override ("desperate" vehicles), resolved
//! authoritatively in `solve_conflicts`.
//!
//! A vehicle stuck past its cooperativity-scaled patience threshold (`Vehicle::is_desperate`)
//! wins a contested cell, overriding normal right-of-way, the fixed conflict-zone winner
//! and the left/right rule - across EVERY conflict type - but NEVER a `Tail` conflict
//! (physical body), and never two vehicles into the same cell (claimed-cell dedup).

#[cfg(test)]
mod tests {
    use crate::agents::{Vehicle, VehicleIntention, VehiclesStorage, PATIENCE_MAX, PATIENCE_MIN};
    use crate::conflicts::{solve_conflicts, CellConflict, ConflictType};
    use crate::grid::cell::CellID;
    use crate::maneuver::LaneChangeType;
    use crate::verbose::LocalLogger;

    /// Build a vehicle with explicit cooperativity (via aggressive level), stuck counter,
    /// maneuver and target cell. `wait` >= patience makes it desperate.
    fn veh(id: u64, cell: CellID, aggr: f64, speed: i32, wait: i32, maneuver: LaneChangeType, target: CellID) -> Vehicle {
        let mut v = Vehicle::new(id)
            .with_cell(cell)
            .with_aggressive_level(aggr)
            .with_speed(speed)
            .build();
        v.wait_ticks = wait;
        v.set_intention(VehicleIntention {
            intention_maneuver: maneuver,
            intention_cell_id: target,
            intention_speed: speed,
            ..Default::default()
        });
        v
    }

    fn speed(v: &VehiclesStorage, id: u64) -> i32 {
        v.get(&id).unwrap().intention.intention_speed
    }

    /// A stuck vehicle wins a contested cell even though it is NOT the priority participant.
    #[test]
    fn test_desperate_overrides_right_of_way() {
        let log = LocalLogger::none();

        // Baseline: nobody desperate -> the priority participant (index 0) wins.
        let mut vehicles = VehiclesStorage::new();
        vehicles.insert(1, veh(1, 10, 0.0, 3, 0, LaneChangeType::NoChange, 15));
        vehicles.insert(2, veh(2, 11, 0.0, 1, 0, LaneChangeType::NoChange, 15));
        let conflicts = vec![CellConflict { cell_id: 15, participants: vec![1, 2], priority_participant_index: 0, conflict_type: ConflictType::MergeForward }];
        solve_conflicts(conflicts, &mut vehicles, &log).unwrap();
        assert_ne!(speed(&vehicles, 1), 0, "priority vehicle moves when nobody is desperate");
        assert_eq!(speed(&vehicles, 2), 0);

        // With vehicle 2 desperate it wins, despite vehicle 1 being the priority participant.
        let mut vehicles = VehiclesStorage::new();
        vehicles.insert(1, veh(1, 10, 0.0, 3, 0, LaneChangeType::NoChange, 15));
        vehicles.insert(2, veh(2, 11, 0.0, 1, 9999, LaneChangeType::NoChange, 15));
        let conflicts = vec![CellConflict { cell_id: 15, participants: vec![1, 2], priority_participant_index: 0, conflict_type: ConflictType::MergeForward }];
        solve_conflicts(conflicts, &mut vehicles, &log).unwrap();
        assert_eq!(speed(&vehicles, 1), 0, "patient priority vehicle yields to the desperate one");
        assert_ne!(speed(&vehicles, 2), 0, "desperate vehicle wins the contested cell");
    }

    /// Desperation is orthogonal to aggression: a cooperative-but-desperate vehicle beats
    /// an aggressive-but-patient one even when the latter is the priority participant.
    #[test]
    fn test_cooperative_desperate_beats_aggressive() {
        let log = LocalLogger::none();
        let mut vehicles = VehiclesStorage::new();
        vehicles.insert(1, veh(1, 10, 0.0, 2, 9999, LaneChangeType::NoChange, 15)); // cooperative, desperate
        vehicles.insert(2, veh(2, 11, 1.0, 2, 0, LaneChangeType::NoChange, 15)); // aggressive, patient
        let conflicts = vec![CellConflict { cell_id: 15, participants: vec![1, 2], priority_participant_index: 1, conflict_type: ConflictType::MergeForward }];
        solve_conflicts(conflicts, &mut vehicles, &log).unwrap();
        assert_ne!(speed(&vehicles, 1), 0, "cooperative-desperate wins");
        assert_eq!(speed(&vehicles, 2), 0, "aggressive-patient yields");
    }

    /// Desperation NEVER overrides a Tail conflict: you cannot force through a body.
    #[test]
    fn test_desperate_does_not_override_tail() {
        let log = LocalLogger::none();
        let mut vehicles = VehiclesStorage::new();
        vehicles.insert(1, veh(1, 10, 0.0, 2, 0, LaneChangeType::NoChange, 15)); // owns the cell as a tail
        vehicles.insert(2, veh(2, 11, 0.0, 2, 9999, LaneChangeType::NoChange, 15)); // desperate, but only transit
        let conflicts = vec![CellConflict { cell_id: 15, participants: vec![1, 2], priority_participant_index: 0, conflict_type: ConflictType::Tail }];
        solve_conflicts(conflicts, &mut vehicles, &log).unwrap();
        assert_ne!(speed(&vehicles, 1), 0, "tail (physical body) keeps the cell");
        assert_eq!(speed(&vehicles, 2), 0, "desperate vehicle must NOT force through a body");
    }

    /// Several desperate vehicles still yield exactly ONE winner: the most-starved
    /// (highest wait_ticks) one.
    #[test]
    fn test_most_starved_wins() {
        let log = LocalLogger::none();
        let mut vehicles = VehiclesStorage::new();
        vehicles.insert(1, veh(1, 10, 0.0, 2, 5000, LaneChangeType::NoChange, 15)); // desperate
        vehicles.insert(2, veh(2, 11, 0.0, 2, 9999, LaneChangeType::NoChange, 15)); // MORE desperate
        let conflicts = vec![CellConflict { cell_id: 15, participants: vec![1, 2], priority_participant_index: 0, conflict_type: ConflictType::MergeForward }];
        solve_conflicts(conflicts, &mut vehicles, &log).unwrap();
        assert_eq!(speed(&vehicles, 1), 0, "less-starved desperate vehicle yields");
        assert_ne!(speed(&vehicles, 2), 0, "most-starved desperate vehicle wins (single winner)");
    }

    /// Desperation overrides a fixed conflict-zone winner (breaks junction right-of-way
    /// starvation). The zone conflict is intercepted before its normal handling.
    #[test]
    fn test_desperate_overrides_conflict_zone() {
        let log = LocalLogger::none();
        let mut vehicles = VehiclesStorage::new();
        vehicles.insert(3, veh(3, 10, 0.0, 2, 0, LaneChangeType::NoChange, 15)); // zone-favoured, patient
        vehicles.insert(4, veh(4, 11, 0.0, 2, 9999, LaneChangeType::NoChange, 16)); // zone-disfavoured, desperate
        let conflicts = vec![CellConflict { cell_id: -1, participants: vec![3, 4], priority_participant_index: 0, conflict_type: ConflictType::CrossConflictZone }];
        solve_conflicts(conflicts, &mut vehicles, &log).unwrap();
        assert_ne!(speed(&vehicles, 4), 0, "desperate vehicle overrides the fixed conflict-zone winner");
        assert_eq!(speed(&vehicles, 3), 0, "zone-favoured but patient vehicle yields");
    }

    /// Desperation overrides the left-beats-right rule in a crossing-trajectory conflict.
    #[test]
    fn test_desperate_overrides_cross_lane_change() {
        let log = LocalLogger::none();
        let mut vehicles = VehiclesStorage::new();
        vehicles.insert(1, veh(1, 10, 0.0, 2, 0, LaneChangeType::ChangeLeft, 20)); // left normally wins, patient
        vehicles.insert(2, veh(2, 11, 0.0, 2, 9999, LaneChangeType::ChangeRight, 21)); // right, desperate
        let conflicts = vec![CellConflict { cell_id: -1, participants: vec![1, 2], priority_participant_index: 0, conflict_type: ConflictType::CrossLaneChange }];
        solve_conflicts(conflicts, &mut vehicles, &log).unwrap();
        assert_eq!(speed(&vehicles, 2), 1, "desperate right-maneuver wins (speed 1) over the patient left-maneuver");
        assert_eq!(speed(&vehicles, 1), 0, "patient left-maneuver yields to the desperate one");
    }

    /// Two separate conflicts cannot grant the same cell to two desperate vehicles: the
    /// second is blocked (claimed-cell dedup).
    #[test]
    fn test_claimed_cells_dedup() {
        let log = LocalLogger::none();
        let mut vehicles = VehiclesStorage::new();
        vehicles.insert(1, veh(1, 10, 0.0, 2, 9999, LaneChangeType::NoChange, 30)); // desperate, wants 30
        vehicles.insert(2, veh(2, 11, 0.0, 2, 0, LaneChangeType::NoChange, 30));
        vehicles.insert(3, veh(3, 12, 0.0, 2, 9999, LaneChangeType::NoChange, 30)); // desperate, also wants 30
        vehicles.insert(4, veh(4, 13, 0.0, 2, 0, LaneChangeType::NoChange, 30));
        let conflicts = vec![
            CellConflict { cell_id: 30, participants: vec![1, 2], priority_participant_index: 0, conflict_type: ConflictType::MergeForward },
            CellConflict { cell_id: 30, participants: vec![3, 4], priority_participant_index: 0, conflict_type: ConflictType::MergeForward },
        ];
        solve_conflicts(conflicts, &mut vehicles, &log).unwrap();
        let movers: Vec<u64> = [1u64, 2, 3, 4].into_iter().filter(|&id| speed(&vehicles, id) > 0).collect();
        assert_eq!(movers, vec![1], "exactly one vehicle (the first desperate winner) enters cell 30; the second desperate vehicle is deduped");
    }

    /// A desperate winner moving multiple cells (speed>1) is clamped to a single-cell step
    /// into the cell it actually ENTERS (its first path cell), and only that cell is reserved
    /// -so it never sweeps unreserved intermediate cells.
    #[test]
    fn test_desperate_multicell_winner_clamped_to_one_cell() {
        let log = LocalLogger::none();
        let mut vehicles = VehiclesStorage::new();
        // Desperate vehicle 1 intends to move 3 cells: current 15 -> 16 -> 17 -> head 18.
        let mut w = Vehicle::new(1).with_cell(15).with_aggressive_level(0.0).with_speed(3).build();
        w.wait_ticks = 9999;
        w.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 18,
            intermediate_cells: vec![16, 17],
            intention_speed: 3,
            ..Default::default()
        });
        vehicles.insert(1, w);
        // Vehicle 2 contends the FIRST swept cell (16); conflict is keyed on 16.
        vehicles.insert(2, veh(2, 11, 0.0, 1, 0, LaneChangeType::NoChange, 16));
        let conflicts = vec![CellConflict { cell_id: 16, participants: vec![1, 2], priority_participant_index: 1, conflict_type: ConflictType::MergeForward }];
        solve_conflicts(conflicts, &mut vehicles, &log).unwrap();
        let w = vehicles.get(&1).unwrap();
        assert_eq!(w.intention.intention_cell_id, 16, "winner clamped to its first path cell (the entered cell)");
        assert!(w.intention.intermediate_cells.is_empty(), "no multi-cell sweep of unreserved cells");
        assert_eq!(w.intention.intention_speed, 1, "single-cell step");
        assert_eq!(speed(&vehicles, 2), 0, "the other contender for cell 16 is blocked");
    }

    /// Regression: clamping a TAILED desperate multi-cell winner must rebuild its tail for
    /// the one-cell step, otherwise the stale multi-cell tail lands ahead of / on the head.
    #[test]
    fn test_desperate_tailed_winner_tail_stays_behind_head() {
        let log = LocalLogger::none();
        let mut vehicles = VehiclesStorage::new();
        // Tailed vehicle: head at 15, tail [13, 14] (furthest..nearest). Intends a 3-cell
        // move 15 -> 16 -> 17 -> head 18; its stale tail intention (multi-cell) would be [16,17].
        let mut w = Vehicle::new(1).with_cell(15).with_aggressive_level(0.0).with_speed(3).build();
        w.wait_ticks = 9999;
        w.tail_cells = vec![13, 14];
        w.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 18,
            intermediate_cells: vec![16, 17],
            tail_intention_cells: vec![16, 17], // what add_intention computes for the 3-cell move
            intention_speed: 3,
            ..Default::default()
        });
        vehicles.insert(1, w);
        vehicles.insert(2, veh(2, 11, 0.0, 1, 0, LaneChangeType::NoChange, 16));
        let conflicts = vec![CellConflict { cell_id: 16, participants: vec![1, 2], priority_participant_index: 1, conflict_type: ConflictType::MergeForward }];
        solve_conflicts(conflicts, &mut vehicles, &log).unwrap();
        let w = vehicles.get(&1).unwrap();
        assert_eq!(w.intention.intention_cell_id, 16, "head clamped to the entered cell");
        // One-cell step: tail shifts forward, vacated cell 15 becomes the nearest tail cell.
        assert_eq!(w.intention.tail_intention_cells, vec![14, 15], "tail rebuilt for the 1-cell step (stays behind the head)");
        assert!(!w.intention.tail_intention_cells.contains(&16), "the head cell is not also occupied by the tail");
    }

    /// A desperate winner of one conflict blocks the NORMAL winner of a *separate* conflict
    /// that targets the same cell -and the desperate-first reordering makes this hold
    /// regardless of input order.
    #[test]
    fn test_desperate_blocks_normal_winner_on_same_cell() {
        let log = LocalLogger::none();
        let mut vehicles = VehiclesStorage::new();
        vehicles.insert(1, veh(1, 10, 0.0, 2, 9999, LaneChangeType::NoChange, 30)); // desperate, wants 30
        vehicles.insert(2, veh(2, 11, 0.0, 2, 0, LaneChangeType::NoChange, 30));
        vehicles.insert(3, veh(3, 12, 0.0, 2, 0, LaneChangeType::NoChange, 30)); // NORMAL priority for 30, separate conflict
        vehicles.insert(4, veh(4, 13, 0.0, 2, 0, LaneChangeType::NoChange, 30));
        // The NORMAL conflict is listed FIRST to prove desperate conflicts are resolved first.
        let conflicts = vec![
            CellConflict { cell_id: 30, participants: vec![3, 4], priority_participant_index: 0, conflict_type: ConflictType::MergeForward },
            CellConflict { cell_id: 30, participants: vec![1, 2], priority_participant_index: 0, conflict_type: ConflictType::MergeForward },
        ];
        solve_conflicts(conflicts, &mut vehicles, &log).unwrap();
        let movers: Vec<u64> = [1u64, 2, 3, 4].into_iter().filter(|&id| speed(&vehicles, id) > 0).collect();
        assert_eq!(movers, vec![1], "only the desperate vehicle enters cell 30; the normal winner of the other conflict is blocked");
    }

    /// Patience threshold scales linearly with cooperativity; every value is finite.
    #[test]
    fn test_patience_scales_with_cooperativity() {
        let aggressive = Vehicle::new(1).with_aggressive_level(1.0).build(); // cooperativity 0
        let cooperative = Vehicle::new(2).with_aggressive_level(0.0).build(); // cooperativity 1
        let middling = Vehicle::new(3).with_aggressive_level(0.5).build(); // cooperativity 0.5
        assert_eq!(aggressive.patience(), PATIENCE_MIN, "aggressive vehicle uses the impatient threshold");
        assert_eq!(cooperative.patience(), PATIENCE_MAX, "cooperative vehicle waits the longest");
        assert_eq!(middling.patience(), (PATIENCE_MIN + PATIENCE_MAX) / 2, "middling cooperativity is halfway");
    }

    /// `is_desperate` flips exactly at the threshold, never before (no premature forcing).
    #[test]
    fn test_is_desperate_threshold() {
        let mut aggressive = Vehicle::new(1).with_aggressive_level(1.0).build(); // patience 300
        aggressive.wait_ticks = PATIENCE_MIN - 1;
        assert!(!aggressive.is_desperate(), "below threshold -> not desperate");
        aggressive.wait_ticks = PATIENCE_MIN;
        assert!(aggressive.is_desperate(), "at threshold -> desperate");
    }
}
