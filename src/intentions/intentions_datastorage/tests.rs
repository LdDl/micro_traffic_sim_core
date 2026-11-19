use super::*;
use crate::agents::VehicleIntention;
use crate::agents_types::AgentType;

// The first target claim creates one cell entry containing the vehicle ID and Target
// category.
#[test]
fn test_add_to_empty_intentions() {
    let mut intentions = Intentions::new();
    let mut vehicle = Vehicle::new(1).with_type(AgentType::Car).build();
    let intention = VehicleIntention {
        intention_cell_id: 1,
        ..Default::default()
    };
    vehicle.set_intention(intention);
    intentions.add_intention(&mut vehicle, IntentionType::Target);

    // Validate
    assert_eq!(intentions.cells_intentions.len(), 1);
    assert_eq!(intentions.cells_intentions.get(&1).unwrap().len(), 1);
    assert_eq!(
        intentions.cells_intentions.get(&1).unwrap()[0].vehicle_id,
        1
    );
    assert_eq!(
        intentions.cells_intentions.get(&1).unwrap()[0].int_type,
        IntentionType::Target
    );
}

// Two claims for the same cell coexist in insertion order and retain their distinct Target
// and Transit categories.
#[test]
fn test_add_multiple_to_same_cell() {
    let mut intentions = Intentions::new();

    // Add first vehicle
    let mut vehicle1 = Vehicle::new(1).with_type(AgentType::Car).build();
    let intention1 = VehicleIntention {
        intention_cell_id: 1,
        ..Default::default()
    };
    vehicle1.set_intention(intention1);
    intentions.add_intention(&mut vehicle1, IntentionType::Target);

    // Add second vehicle
    let mut vehicle2 = Vehicle::new(2).with_type(AgentType::Car).build();
    let intention2 = VehicleIntention {
        intention_cell_id: 1,
        ..Default::default()
    };
    vehicle2.set_intention(intention2);
    intentions.add_intention(&mut vehicle2, IntentionType::Transit);

    // Validate
    assert_eq!(intentions.cells_intentions.len(), 1);
    assert_eq!(intentions.cells_intentions.get(&1).unwrap().len(), 2);
    assert_eq!(
        intentions.cells_intentions.get(&1).unwrap()[0].vehicle_id,
        1
    );
    assert_eq!(
        intentions.cells_intentions.get(&1).unwrap()[0].int_type,
        IntentionType::Target
    );
    assert_eq!(
        intentions.cells_intentions.get(&1).unwrap()[1].vehicle_id,
        2
    );
    assert_eq!(
        intentions.cells_intentions.get(&1).unwrap()[1].int_type,
        IntentionType::Transit
    );
}

// Claims for different cells remain in separate entries with their own vehicle IDs.
#[test]
fn test_add_to_different_cells() {
    let mut intentions = Intentions::new();

    // Add to first cell
    let mut vehicle1 = Vehicle::new(1).with_type(AgentType::Car).build();
    let intention1 = VehicleIntention {
        intention_cell_id: 1,
        ..Default::default()
    };
    vehicle1.set_intention(intention1);
    intentions.add_intention(&mut vehicle1, IntentionType::Target);

    // Add to second cell
    let mut vehicle2 = Vehicle::new(2).with_type(AgentType::Car).build();
    let intention2 = VehicleIntention {
        intention_cell_id: 2,
        ..Default::default()
    };
    vehicle2.set_intention(intention2);
    intentions.add_intention(&mut vehicle2, IntentionType::Target);

    // Validate
    assert_eq!(intentions.cells_intentions.len(), 2);
    assert_eq!(intentions.cells_intentions.get(&1).unwrap().len(), 1);
    assert_eq!(intentions.cells_intentions.get(&2).unwrap().len(), 1);
    assert_eq!(
        intentions.cells_intentions.get(&1).unwrap()[0].vehicle_id,
        1
    );
    assert_eq!(
        intentions.cells_intentions.get(&1).unwrap()[0].int_type,
        IntentionType::Target
    );
    assert_eq!(
        intentions.cells_intentions.get(&2).unwrap()[0].vehicle_id,
        2
    );
    assert_eq!(
        intentions.cells_intentions.get(&2).unwrap()[0].int_type,
        IntentionType::Target
    );
}

// A truck's projected tail shares cell 10 with another vehicle's target claim. The vacated
// rear cell 8 has no projected claim.
#[test]
fn test_add_multiple_to_same_cell_tail() {
    // cell             8     9     10      11
    // truck body       tail  tail  head    .
    // truck intention  .     tail  tail    head
    // other claim      .     .     target  .
    // This test checks stored intentions. Body occupancy is checked separately.
    let mut intentions = Intentions::new();

    // Setup first vehicle with tail
    let mut vehicle1 = Vehicle::new(1)
        .with_cell(10)
        .with_tail_size(2, vec![8, 9])
        .build();
    let intention1 = VehicleIntention {
        intention_cell_id: 11,
        ..Default::default()
    };
    vehicle1.set_intention(intention1);
    intentions.add_intention(&mut vehicle1, IntentionType::Target);

    // Add second vehicle
    let mut vehicle2 = Vehicle::new(2).with_type(AgentType::Car).build();
    let intention2 = VehicleIntention {
        intention_cell_id: 10,
        ..Default::default()
    };
    vehicle2.set_intention(intention2);
    intentions.add_intention(&mut vehicle2, IntentionType::Target);

    // Validate
    // Cells 11, 10, 9
    assert_eq!(intentions.cells_intentions.len(), 3);
    // Cell 8 should not be included
    assert!(!intentions.cells_intentions.contains_key(&8));
    // Tail end
    assert_eq!(intentions.cells_intentions.get(&9).unwrap().len(), 1);
    // Tail start + vehicle 2
    assert_eq!(intentions.cells_intentions.get(&10).unwrap().len(), 2);
    // Vehicle 1 head
    assert_eq!(intentions.cells_intentions.get(&11).unwrap().len(), 1);

    // Check vehicle 1 tail and intentions
    assert_eq!(
        intentions.cells_intentions.get(&9).unwrap()[0].vehicle_id,
        1
    );
    assert_eq!(
        intentions.cells_intentions.get(&9).unwrap()[0].int_type,
        IntentionType::Tail
    );
    assert_eq!(
        intentions.cells_intentions.get(&10).unwrap()[0].vehicle_id,
        1
    );
    assert_eq!(
        intentions.cells_intentions.get(&10).unwrap()[0].int_type,
        IntentionType::Tail
    );

    // Check vehicle 2
    assert_eq!(
        intentions.cells_intentions.get(&10).unwrap()[1].vehicle_id,
        2
    );
    assert_eq!(
        intentions.cells_intentions.get(&10).unwrap()[1].int_type,
        IntentionType::Target
    );

    // Check vehicle 1 head
    assert_eq!(
        intentions.cells_intentions.get(&11).unwrap()[0].vehicle_id,
        1
    );
    assert_eq!(
        intentions.cells_intentions.get(&11).unwrap()[0].int_type,
        IntentionType::Target
    );
}
