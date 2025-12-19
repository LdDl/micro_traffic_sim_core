use std::fmt;
use rand::Rng;

/// Represents the type of an agent in Cellular Automata (CA).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AgentType {
    /// Undefined agent type.
    Undefined,
    /// A car agent.
    Car,
    /// A bus agent.
    Bus,
    /// A taxi agent.
    Taxi,
    /// A truck agent.
    Truck,
    /// A large bus agent (e.g., articulated bus).
    LargeBus,
    /// A pedestrian agent (is not used anywhere currently).
    Pedestrian,
}

impl fmt::Display for AgentType {
    /// Formats the agent type for display.
    /// 
    /// Returns a short, lowercase string representation suitable for
    /// logging, debugging, and user interfaces.
    /// 
    /// # Examples
    /// 
    /// ```rust
    /// use micro_traffic_sim_core::agents_types::AgentType;
    ///
    /// assert_eq!(format!("{}", AgentType::Undefined), "undefined");
    /// assert_eq!(format!("{}", AgentType::Car), "car");
    /// assert_eq!(format!("{}", AgentType::Bus), "bus");
    /// assert_eq!(format!("{}", AgentType::Taxi), "taxi");
    /// assert_eq!(format!("{}", AgentType::Truck), "truck");
    /// assert_eq!(format!("{}", AgentType::LargeBus), "large_bus");
    /// assert_eq!(format!("{}", AgentType::Pedestrian), "pedestrian");
    /// ```
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let as_str = match self {
            AgentType::Undefined => "undefined",
            AgentType::Car => "car",
            AgentType::Bus => "bus",
            AgentType::Taxi => "taxi",
            AgentType::Truck => "truck",
            AgentType::LargeBus => "large_bus",
            AgentType::Pedestrian => "pedestrian",
        };
        write!(f, "{}", as_str)
    }
}

impl AgentType {
    /// Generates a random agent type, excluding `Undefined`.
    ///
    /// # Examples
    ///
    /// ```
    /// use micro_traffic_sim_core::agents_types::AgentType;
    ///
    /// let random_agent = AgentType::random();
    /// ```
    pub fn random() -> Self {
        let mut rng = rand::rng();
        match rng.random_range(1..=6) {
            1 => AgentType::Car,
            2 => AgentType::Bus,
            3 => AgentType::Taxi,
            4 => AgentType::Truck,
            5 => AgentType::LargeBus,
            6 => AgentType::Pedestrian,
            _ => unreachable!(), // Should never happen
        }
    }

    /// Returns the default tail size for this agent type.
    ///
    /// Multi-cell vehicles like buses, trucks, and large buses occupy
    /// more than one cell in the simulation grid. The tail size indicates
    /// how many additional cells (behind the head) the vehicle occupies.
    ///
    /// # Returns
    ///
    /// - `0` for single-cell agents: `Car`, `Taxi`, `Pedestrian`, `Undefined`
    /// - `1` for medium multi-cell agents: `Bus`, `Truck`
    /// - `2` for large multi-cell agents: `LargeBus`
    ///
    /// # Examples
    ///
    /// ```
    /// use micro_traffic_sim_core::agents_types::AgentType;
    ///
    /// assert_eq!(AgentType::Car.tail_size_default(), 0);
    /// assert_eq!(AgentType::Bus.tail_size_default(), 1);
    /// assert_eq!(AgentType::Truck.tail_size_default(), 1);
    /// assert_eq!(AgentType::LargeBus.tail_size_default(), 2);
    /// ```
    pub fn tail_size_default(&self) -> usize {
        match self {
            AgentType::Undefined | AgentType::Car | AgentType::Taxi | AgentType::Pedestrian => 0,
            AgentType::Bus | AgentType::Truck => 1,
            AgentType::LargeBus => 2,
        }
    }
}

#[cfg(test)]
mod tests {
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
}
