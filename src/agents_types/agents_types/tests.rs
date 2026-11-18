use super::*;

#[test]
fn test_agent_to_string() {
    assert_eq!(AgentType::Car.to_string(), "car");
    assert_eq!(AgentType::Bus.to_string(), "bus");
    assert_eq!(AgentType::Truck.to_string(), "truck");
    assert_eq!(AgentType::LargeBus.to_string(), "large_bus");
    assert_eq!(AgentType::Pedestrian.to_string(), "pedestrian");
}

#[test]
fn test_random_agent() {
    for _ in 0..100 {
        let random_agent = AgentType::random();
        assert!(random_agent != AgentType::Undefined);
    }
}

#[test]
fn test_tail_size_default() {
    assert_eq!(AgentType::Undefined.tail_size_default(), 0);
    assert_eq!(AgentType::Car.tail_size_default(), 0);
    assert_eq!(AgentType::Taxi.tail_size_default(), 0);
    assert_eq!(AgentType::Pedestrian.tail_size_default(), 0);
    assert_eq!(AgentType::Bus.tail_size_default(), 1);
    assert_eq!(AgentType::Truck.tail_size_default(), 1);
    assert_eq!(AgentType::LargeBus.tail_size_default(), 2);
}
