use super::*;
use crate::agents_types::AgentType;
use crate::behaviour::BehaviourType;

#[test]
fn test_tail_size_auto_resolution() {
    // Car should have tail_size = 0
    let car_trip = Trip::new(1, 10, TripType::Constant)
        .with_allowed_agent_type(AgentType::Car)
        .build();
    assert_eq!(car_trip.vehicle_tail_size, 0);

    // Bus should have tail_size = 1
    let bus_trip = Trip::new(1, 10, TripType::Constant)
        .with_allowed_agent_type(AgentType::Bus)
        .build();
    assert_eq!(bus_trip.vehicle_tail_size, 1);

    // Truck should have tail_size = 1
    let truck_trip = Trip::new(1, 10, TripType::Constant)
        .with_allowed_agent_type(AgentType::Truck)
        .build();
    assert_eq!(truck_trip.vehicle_tail_size, 1);

    // LargeBus should have tail_size = 2
    let large_bus_trip = Trip::new(1, 10, TripType::Constant)
        .with_allowed_agent_type(AgentType::LargeBus)
        .build();
    assert_eq!(large_bus_trip.vehicle_tail_size, 2);
}

#[test]
fn test_tail_size_explicit_override() {
    // Explicit tail size should not be overridden
    let trip = Trip::new(1, 10, TripType::Constant)
        .with_allowed_agent_type(AgentType::Car)
        .with_vehicle_tail_size(5)
        .build();
    assert_eq!(trip.vehicle_tail_size, 5);
}

#[test]
fn test_speed_limit_default() {
    // Default speed_limit should be -1
    let trip = Trip::new(1, 10, TripType::Constant).build();
    assert_eq!(trip.speed_limit, -1);
}

#[test]
fn test_speed_limit_explicit() {
    // Explicit speed_limit should be set
    let trip = Trip::new(1, 10, TripType::Constant)
        .with_speed_limit(5)
        .build();
    assert_eq!(trip.speed_limit, 5);
}

#[test]
fn test_speed_limit_with_behaviour_first() {
    // Speed limit set after behaviour should still apply
    let trip = Trip::new(1, 10, TripType::Constant)
        .with_allowed_behaviour_type(BehaviourType::Aggressive)
        .with_speed_limit(3)
        .build();
    assert_eq!(trip.speed_limit, 3);
}

#[test]
fn test_speed_limit_before_behaviour() {
    // Speed limit set before behaviour should still apply
    let trip = Trip::new(1, 10, TripType::Constant)
        .with_speed_limit(3)
        .with_allowed_behaviour_type(BehaviourType::Aggressive)
        .build();
    assert_eq!(trip.speed_limit, 3);
}
