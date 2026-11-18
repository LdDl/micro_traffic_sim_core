use super::*;
use crate::agents::vehicle_intention::VehicleIntention;
use crate::agents_types::AgentType;
use crate::maneuver::LaneChangeType;

// Length scaling preserves (L + gap)(1 - p) = (1 + gap)(1 - p0) for the sampled inputs. a
// one-cell vehicle keeps p0.
#[test]
fn test_slow_to_start_for_length() {
    // A one cell vehicle keeps the behaviour value.
    assert_eq!(Vehicle::slow_to_start_for_length(0.425, 1, 0), 0.425);
    assert_eq!(Vehicle::slow_to_start_for_length(0.425, 1, 1), 0.425);
    for &(p0, len, gap) in &[
        (0.425, 2, 0),
        (0.425, 2, 1),
        (0.425, 3, 1),
        (0.6, 3, 0),
        (0.0, 2, 1),
    ] {
        let p = Vehicle::slow_to_start_for_length(p0, len, gap);
        // Space per escape delay equals that of a one cell vehicle: (L + g)(1 - p) = (1 + g)(1 - p0).
        let lhs = (len as f64 + gap as f64) * (1.0 - p);
        let rhs = (1.0 + gap as f64) * (1.0 - p0);
        assert!((lhs - rhs).abs() < 1e-12, "invariance for {p0} {len} {gap}");
        assert!(
            p >= p0 && p < 1.0,
            "a longer vehicle departs later but eventually departs"
        );
    }
    // Longer vehicles depart later.
    assert!(
        Vehicle::slow_to_start_for_length(0.425, 3, 1)
            > Vehicle::slow_to_start_for_length(0.425, 2, 1)
    );
    // Out of range inputs.
    assert_eq!(Vehicle::slow_to_start_for_length(1.5, 1, 0), 1.0);
    assert_eq!(
        Vehicle::slow_to_start_for_length(0.425, 2, -3),
        Vehicle::slow_to_start_for_length(0.425, 2, 0)
    );
}

// Trimming keeps a prefix of the requested forward path, with matching speed and
// destination. a larger limit does not extend it.
#[test]
fn test_trim_forward_intention() {
    // 3(head) 4 5 6(requested)
    // Trimming to two cells retains [4, 5]. Trimming to zero retains the head at 3.
    let mut vehicle = Vehicle::new(1).with_cell(3).with_speed(3).build();
    vehicle.set_intention(VehicleIntention {
        intention_maneuver: LaneChangeType::NoChange,
        intention_cell_id: 6,
        intermediate_cells: vec![4, 5],
        intention_speed: 3,
        ..Default::default()
    });
    vehicle.trim_forward_intention(5);
    assert_eq!(
        vehicle.intention.intention_cell_id, 6,
        "a longer length keeps the move"
    );
    vehicle.trim_forward_intention(2);
    assert_eq!(vehicle.intention.intention_cell_id, 5);
    assert_eq!(vehicle.intention.intermediate_cells, vec![4]);
    assert_eq!(vehicle.intention.intention_speed, 2);
    vehicle.trim_forward_intention(0);
    assert_eq!(
        vehicle.intention.intention_cell_id, 3,
        "length 0 keeps the vehicle in place"
    );
    assert!(vehicle.intention.intermediate_cells.is_empty());
    assert_eq!(vehicle.intention.intention_speed, 0);
}

// Shortening a truck's forward intention recomputes its projected tail. Trimming to zero
// retains the stored tail projection, which movement must not apply.
#[test]
fn test_trim_forward_intention_moves_tail() {
    // cell            1     2     3     4     5
    // body            tail  tail  head  .     .
    // requested head  .     .     .     .     head
    // trimmed to 1    .     tail  tail  head  .
    // The shortened intention must move the whole body consistently.
    let mut truck = Vehicle::new(1)
        .with_cell(3)
        .with_tail_size(2, vec![1, 2])
        .with_speed(2)
        .build();
    truck.set_intention(VehicleIntention {
        intention_maneuver: LaneChangeType::NoChange,
        intention_cell_id: 5,
        intermediate_cells: vec![4],
        intention_speed: 2,
        ..Default::default()
    });
    truck.trim_forward_intention(1);
    assert_eq!(truck.intention.intention_cell_id, 4);
    assert_eq!(truck.intention.tail_intention_cells, vec![2, 3]);
    // Staying in place follows the intention storage convention for a vehicle that does not
    // move. Movement applies the tail intention only when the head moves, so the body stays.
    truck.trim_forward_intention(0);
    assert_eq!(truck.intention.intention_cell_id, 3);
    assert_eq!(truck.intention.tail_intention_cells, vec![2, 3]);
}

// Each countdown call removes one dwell tick. Reset restores the configured dwell
// duration.
#[test]
fn test_relaxation_countdown() {
    let relax_time = 9;
    let sim_steps = 3;
    let mut vehicle = Vehicle::new(1)
        .with_cell(3)
        .with_destination(100)
        .with_type(AgentType::Car)
        .with_transit_cells(vec![3, 8])
        .with_relax_time(relax_time)
        .build();
    for _ in 0..sim_steps {
        vehicle.relax_countdown_dec();
    }
    assert_eq!(
        relax_time - sim_steps,
        vehicle.get_relax_countdown(),
        "Incorrect countdown after specified number of steps"
    );
    vehicle.relax_countdown_reset();
    assert_eq!(
        relax_time,
        vehicle.get_relax_countdown(),
        "Incorrect countdown after reset"
    );
}

// The vehicle builder maps aggression to its complementary cooperativity: 1 - aggression.
#[test]
fn test_cooperative_level() {
    let coop_level = 0.75;
    let aggr_level = 1.0 - coop_level;
    let vehicle = Vehicle::new(1).with_aggressive_level(aggr_level).build();
    assert_eq!(
        coop_level, vehicle.cooperativity,
        "Incorrect cooperativity level"
    )
}
