use std::fmt;
use rand::Rng;

/// Represents the behaviour type of an agent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BehaviourType {
    /// Undefined behaviour type.
    Undefined,
    /// Movement is prohibited due to external factors (e.g., block, broken vehicle, etc.).
    Block,
    /// Agents minimizing travel time.
    Aggressive,
    /// Agents who do not always minimize travel time.
    Cooperative,
    /// Agent speed will be limited with a specified value given by the trip.
    LimitSpeedByTrip,
}

impl BehaviourType {
    /// Generates a random behaviour type for vehicle agents based on provided ratios.
    ///
    /// # Arguments
    ///
    /// - `ratio_a`: Ratio for selecting `Aggressive` behaviour.
    /// - `ratio_b`: Ratio for selecting `Cooperative` behaviour.
    ///
    /// # Returns
    ///
    /// A `BehaviourType` randomly chosen based on the given ratios.
    /// # Examples
    ///
    /// ```
    /// use micro_traffic_sim_core::behaviour::BehaviourType;
    ///
    /// let ratio_aggressive = 0.7;
    /// let ratio_cooperative = 0.3;
    /// let random_behaviour = BehaviourType::random_vehicle_behaviour_type(ratio_aggressive, ratio_cooperative);
    /// ```
    pub fn random_vehicle_behaviour_type(ratio_a: f64, ratio_b: f64) -> Self {
        let mut rng = rand::thread_rng();
        let random_number = rng.gen_range(0.0..(ratio_a + ratio_b));
        if random_number < ratio_a {
            BehaviourType::Aggressive
        } else {
            BehaviourType::Cooperative
        }
    }
}

impl fmt::Display for BehaviourType {
    /// Formats the behaviour type for display.
    /// 
    /// Returns a short, lowercase string representation suitable for
    /// logging, debugging, and user interfaces.
    /// 
    /// # Examples
    /// 
    /// ```rust
    /// use micro_traffic_sim_core::behaviour::BehaviourType;
    /// 
    /// assert_eq!(format!("{}", BehaviourType::Undefined), "undefined");
    /// assert_eq!(format!("{}", BehaviourType::Block), "block");
    /// assert_eq!(format!("{}", BehaviourType::Aggressive), "aggressive");
    /// assert_eq!(format!("{}", BehaviourType::Cooperative), "cooperative");
    /// assert_eq!(format!("{}", BehaviourType::LimitSpeedByTrip), "limit_speed");
    /// ```
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let as_str = match self {
            BehaviourType::Undefined => "undefined",
            BehaviourType::Block => "block",
            BehaviourType::Aggressive => "aggressive",
            BehaviourType::Cooperative => "cooperative",
            BehaviourType::LimitSpeedByTrip => "limit_speed",
        };
        write!(f, "{}", as_str)
    }
}

/// Represents behaviour parameters for an agent.
#[derive(Debug, Clone, Copy)]
pub struct BehaviourParameters {
    /// Factor affecting the slowdown of the agent.
    slowdown_factor: f64,
    /// Speed limit for the agent.
    speed_limit: i32,
    /// Aggressiveness level of the agent.
    aggressive_level: f64,
    /// Minimum safe distance required by the agent.
    min_safe_distance: i32,
}

impl BehaviourParameters {
    /// Constructs new behaviour parameters based on the behaviour type.
    ///
    /// # Arguments
    ///
    /// - `behaviour`: The `BehaviourType` for which parameters are to be generated.
    ///
    /// # Returns
    ///
    /// A new instance of `BehaviourParameters`.
    ///
    /// # Examples
    ///
    /// ```
    /// use micro_traffic_sim_core::behaviour::{BehaviourType, BehaviourParameters};
    ///
    /// let behaviour_params = BehaviourParameters::from_behaviour_type(BehaviourType::Aggressive);
    /// ```
    pub fn from_behaviour_type(behaviour: BehaviourType) -> Self {
        match behaviour {
            BehaviourType::Block => Self::new(1.0, 0, 1.0, 0),
            BehaviourType::Aggressive => Self::new(0.1, 5, 0.9, 0),
            BehaviourType::Cooperative => Self::new(0.5, 4, 0.0, 1),
            BehaviourType::LimitSpeedByTrip => Self::new(0.7, 3, 0.1, 1),
            BehaviourType::Undefined => Self::new(0.5, 2, 0.5, 0),
        }
    }

    /// Constructs a new instance of `BehaviourParameters`.
    ///
    /// # Arguments
    ///
    /// - `slowdown_factor`: Factor affecting the slowdown of the agent.
    /// - `speed_limit`: Speed limit for the agent.
    /// - `aggressive_level`: Aggressiveness level of the agent.
    /// - `min_safe_distance`: Minimum safe distance required by the agent.
    ///
    /// # Returns
    ///
    /// A new instance of `BehaviourParameters`.
    ///
    /// # Examples
    ///
    /// ```
    /// use micro_traffic_sim_core::behaviour::{BehaviourType, BehaviourParameters};
    ///
    /// let behaviour_params = BehaviourParameters::new(0.1, 2, 0.4, 1);
    /// ```
    pub fn new(slowdown_factor: f64, speed_limit: i32, aggressive_level: f64, min_safe_distance: i32) -> Self {
        Self {
            slowdown_factor,
            speed_limit,
            aggressive_level,
            min_safe_distance,
        }
    }

    /// Returns the speed limit.
    pub fn speed_limit(&self) -> i32 {
        self.speed_limit
    }

    /// Returns the slowdown factor.
    pub fn slowdown_factor(&self) -> f64 {
        self.slowdown_factor
    }

    /// Returns the minimum safe distance.
    pub fn min_safe_distance(&self) -> i32 {
        self.min_safe_distance
    }

    /// Returns the aggressiveness level.
    pub fn aggressive_level(&self) -> f64 {
        self.aggressive_level
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_behaviour_display() {
        assert_eq!(format!("{}", BehaviourType::Aggressive), "aggressive");
        assert_eq!(format!("{}", BehaviourType::Cooperative), "cooperative");
    }
    #[test]
    fn test_behaviour_parameters_from() {
        let params = BehaviourParameters::from_behaviour_type(BehaviourType::Aggressive);
        assert_eq!(params.slowdown_factor(), 0.1);
        assert_eq!(params.speed_limit(), 5);
        assert_eq!(params.aggressive_level(), 0.9);
        assert_eq!(params.min_safe_distance(), 0);
    }
    #[test]
    fn test_random_vehicle_behaviour_type() {
        let ratio_a = 0.7;
        let ratio_b = 0.3;

        for _ in 0..100 {
            let random_behaviour: BehaviourType = BehaviourType::random_vehicle_behaviour_type(ratio_a, ratio_b);
            assert!(
                random_behaviour == BehaviourType::Aggressive || random_behaviour == BehaviourType::Cooperative,
                "Random behaviour must be Aggressive or Cooperative. Other have not been implemented yet, got: {:?}",
                random_behaviour
            );
        }
    }
}
