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
        let mut rng = rand::rng();
        let random_number = rng.random_range(0.0..(ratio_a + ratio_b));
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
    /// Factor affecting the slowdown of the agent (probability of a random dawdle while moving).
    slowdown_factor_p: f64,
    /// VDR slow-to-start probability `p0` used while the agent is STOPPED.
    /// `p0 >= p` (the moving slowdown) produces the realistic capacity drop / metastable jam outflow.
    slow_to_start_factor_p0: f64,
    /// Speed limit for the agent.
    speed_limit: i32,
    /// Aggressiveness level of the agent.
    aggressive_level: f64,
    /// Minimum safe distance required by the agent.
    min_safe_distance: i32,
    /// Reactive-lane-change cooldown duration in steps (anti-weaving).
    lc_cooldown: i64,
}

impl BehaviourParameters {
    /// Constructs `BehaviourParameters` for a behaviour type.
    ///
    /// Each type maps to a tuple
    /// `(p, p0, speed_limit, aggressive_level, min_safe_distance, lc_cooldown)` where:
    /// - `p`  - NaSch moving-dawdle probability;
    /// - `p0` - VDR slow-to-start probability while stopped.
    ///   `p0 >= p` is the VDR asymmetry that produces the capacity drop; widen `(p0 - p)` to
    ///   strengthen the drop / metastability (without amplifying gridlock);
    /// - `speed_limit`, `aggressive_level`, `min_safe_distance` - per-type movement defaults;
    /// - `lc_cooldown` - reactive-lane-change cooldown in steps (anti-weaving; 0 = changes freely).
    ///
    /// The concrete per-type values are the `match` arms below (the single source of truth).
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
        let (p, p0, speed_limit, aggressive, min_safe, lc_cooldown) = match behaviour {
            BehaviourType::Block => (1.0, 1.0, 0, 1.0, 0, 0),
            BehaviourType::Aggressive => (0.1, 0.35, 5, 0.9, 0, 0),
            BehaviourType::Cooperative => (0.5, 0.65, 4, 0.0, 1, 1),
            BehaviourType::LimitSpeedByTrip => (0.7, 0.8, 3, 0.1, 1, 2),
            BehaviourType::Undefined => (0.5, 0.6, 2, 0.5, 0, 1),
        };
        Self {
            slowdown_factor_p: p,
            slow_to_start_factor_p0: p0,
            speed_limit,
            aggressive_level: aggressive,
            min_safe_distance: min_safe,
            lc_cooldown,
        }
    }

    /// Constructs `BehaviourParameters` directly from raw values.
    ///
    /// `slow_to_start_factor_p0` is defaulted to `slowdown_factor_p` (`p0 == p`, i.e. NO VDR
    /// asymmetry and so no capacity drop). Use `from_behaviour_type` for the per-type presets that
    /// raise `p0` above `p`. When `p0 = p``: no VDR asymmetry by default.
    ///
    /// # Arguments
    ///
    /// - `slowdown_factor_p`: the NaSch random-slowdown probability `p` (while moving).
    /// - `speed_limit`: Speed limit for the agent.
    /// - `aggressive_level`: Aggressiveness level of the agent.
    /// - `min_safe_distance`: Minimum safe distance required by the agent.
    /// - `lc_cooldown`: reactive-lane-change cooldown in steps (anti-weaving; `0` = changes freely).
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
    /// let behaviour_params = BehaviourParameters::new(0.1, 2, 0.4, 1, 0);
    /// ```
    pub fn new(
        slowdown_factor_p: f64,
        speed_limit: i32,
        aggressive_level: f64,
        min_safe_distance: i32,
        lc_cooldown: i64,
    ) -> Self {
        Self {
            slowdown_factor_p,
            slow_to_start_factor_p0: slowdown_factor_p,
            speed_limit,
            aggressive_level,
            min_safe_distance,
            lc_cooldown,
        }
    }

    /// Returns the speed limit.
    pub fn speed_limit(&self) -> i32 {
        self.speed_limit
    }

    /// Returns the reactive-lane-change cooldown duration in steps (anti-weaving).
    pub fn lc_cooldown(&self) -> i64 {
        self.lc_cooldown
    }

    /// Returns the slowdown factor (random dawdle probability while moving).
    pub fn slowdown_factor_p(&self) -> f64 {
        self.slowdown_factor_p
    }

    /// Returns the VDR slow-to-start probability used while stopped.
    pub fn slow_to_start_factor_p0(&self) -> f64 {
        self.slow_to_start_factor_p0
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
        assert_eq!(params.slowdown_factor_p(), 0.1);
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
