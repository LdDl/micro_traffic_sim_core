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
    /// assert_eq!(format!("{}", AgentType::Pedestrian), "pedestrian");
    /// ```
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let as_str = match self {
            AgentType::Undefined => "undefined",
            AgentType::Car => "car",
            AgentType::Bus => "bus",
            AgentType::Taxi => "taxi",
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
        match rng.random_range(1..=4) {
            1 => AgentType::Car,
            2 => AgentType::Bus,
            3 => AgentType::Taxi,
            4 => AgentType::Pedestrian,
            _ => unreachable!(), // Should never happen
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_agent_to_string() {
        assert_eq!(AgentType::Car.to_string(), "car");
        assert_eq!(AgentType::Pedestrian.to_string(), "pedestrian");
    }

    #[test]
    fn test_random_agent() {
        for _ in 0..100 {
            let random_agent = AgentType::random();
            assert!(random_agent != AgentType::Undefined);
        }
    }
}
