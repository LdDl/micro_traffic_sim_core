use crate::agents::RelativeRandomization;
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
    /// Standstill gap in cells: empty cells kept in front of a stopped leader.
    standstill_gap: i32,
    /// Randomization relative to the leader and the stop time threshold (three phase layer).
    relative_randomization: RelativeRandomization,
}

impl BehaviourParameters {
    /// Constructs `BehaviourParameters` for a behaviour type.
    ///
    /// Each type maps to a tuple
    /// `(p, p0, speed_limit, aggressive_level, min_safe_distance, lc_cooldown)` where:
    /// - `p`  - NaSch moving-dawdle probability, the style part of the randomization: 0.1 for
    ///   the aggressive preset up to 0.4 for the speed limiting one, always at most `p0` so that
    ///   the VDR asymmetry holds for every preset;
    /// - `p0` - VDR slow-to-start probability while stopped.
    ///   `p0 >= p` is the VDR asymmetry that produces the capacity drop. The value is not a
    ///   driver style but a characteristic of the road: the mean escape delay from a jam
    ///   tau / (1 - p0) sets the velocity of the downstream jam front, which is the same for
    ///   every jam under the same road conditions (Kerner 2004, The Physics of Traffic,
    ///   doi:10.1007/978-3-540-40986-1, ch. 11). Every moving preset therefore uses the line J
    ///   value 0.425 of Kerner, Klenov, Wolf 2002 (doi:10.1088/0305-4470/35/47/303, Table 3),
    ///   which with a 7.5 m jam spacing gives a front velocity of 15.5 km/h; a long vehicle
    ///   derives its own value from it (`Vehicle::slow_to_start_for_length`). Driver style lives
    ///   in `p`, the aggression gradient, the cooldown and the standstill gap;
    /// - `speed_limit`, `aggressive_level`, `min_safe_distance` - per-type movement defaults;
    /// - `lc_cooldown` - reactive-lane-change cooldown in steps (anti-weaving; 0 = changes freely);
    /// - `standstill_gap` - empty cells kept in front of a stopped leader (see
    ///   `Vehicle::standstill_gap`). Careful and undefined types keep one cell, aggressive ones
    ///   stop bumper to bumper, so a mixed flow with about two thirds of careful drivers gets the
    ///   7.5 m jam spacing of the line J calibration (133 veh/km).
    ///
    /// Every preset uses `RelativeRandomization::KKW` (see `agents::RelativeRandomization`), the
    /// three phase layer; `Block` never moves, so its value does not matter. `with_relative_randomization`
    /// switches a preset back to the plain VDR rule.
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
        let (p, p0, speed_limit, aggressive, min_safe, lc_cooldown, standstill_gap) = match behaviour {
            BehaviourType::Block => (1.0, 1.0, 0, 1.0, 0, 0, 0),
            BehaviourType::Aggressive => (0.1, 0.425, 5, 0.9, 0, 0, 0),
            BehaviourType::Cooperative => (0.3, 0.425, 4, 0.0, 1, 1, 1),
            BehaviourType::LimitSpeedByTrip => (0.4, 0.425, 3, 0.1, 1, 2, 1),
            BehaviourType::Undefined => (0.2, 0.425, 2, 0.5, 0, 1, 1),
        };
        Self {
            slowdown_factor_p: p,
            slow_to_start_factor_p0: p0,
            speed_limit,
            aggressive_level: aggressive,
            min_safe_distance: min_safe,
            lc_cooldown,
            standstill_gap,
            relative_randomization: RelativeRandomization::KKW,
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
            standstill_gap: 0,
            relative_randomization: RelativeRandomization::VDR,
        }
    }

    /// Returns a copy with the given standstill gap.
    ///
    /// # Arguments
    ///
    /// - `standstill_gap`: empty cells kept in front of a stopped leader, `0` keeps bumper to bumper.
    ///
    /// # Examples
    ///
    /// ```
    /// use micro_traffic_sim_core::behaviour::BehaviourParameters;
    ///
    /// let params = BehaviourParameters::new(0.1, 4, 0.2, 0, 0).with_standstill_gap(1);
    /// assert_eq!(params.standstill_gap(), 1);
    /// ```
    pub fn with_standstill_gap(mut self, standstill_gap: i32) -> Self {
        self.standstill_gap = standstill_gap.max(0);
        self
    }

    /// Returns the standstill gap in cells.
    pub fn standstill_gap(&self) -> i32 {
        self.standstill_gap
    }

    /// Returns a copy with the given randomization relative to the leader and stop time threshold.
    ///
    /// # Arguments
    ///
    /// - `randomization`: the parameter set; `RelativeRandomization::VDR` restores the plain rule.
    ///
    /// # Examples
    ///
    /// ```
    /// use micro_traffic_sim_core::agents::RelativeRandomization;
    /// use micro_traffic_sim_core::behaviour::{BehaviourParameters, BehaviourType};
    ///
    /// let params = BehaviourParameters::from_behaviour_type(BehaviourType::Cooperative)
    ///     .with_relative_randomization(RelativeRandomization::VDR);
    /// assert_eq!(params.relative_randomization(), RelativeRandomization::VDR);
    /// ```
    pub fn with_relative_randomization(mut self, randomization: RelativeRandomization) -> Self {
        self.relative_randomization = randomization.clamped();
        self
    }

    /// Returns the randomization relative to the leader and the stop time threshold.
    pub fn relative_randomization(&self) -> RelativeRandomization {
        self.relative_randomization
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

    /// Stochastic lane-change probability `P1` (`< 1`) for this behaviour. A DISCRETIONARY
    /// (non-route) lane change that already passed incentive + safety is committed only with
    /// probability `P1`; otherwise the driver waits this step and retries (see `lc_p1_pass`).
    /// Route-mandatory changes bypass `P1` entirely. `P1 < 1` models driver hesitation and damps
    /// weaving (the "tailgating dance").
    ///
    /// The `P1` gate itself is the CA lane-change rule; the affine mapping to
    /// aggressiveness here is a LOCAL calibration (not from a paper): `P1 = 0.5 + 0.5 * aggressive_level`,
    /// in `[0.5, 1.0]`. Cooperative drivers hesitate (floor `0.5`); aggressive ones commit readily
    /// (the `Aggressive` preset, `aggressive_level = 0.9`, gives `~0.95`). The `0.5` floor keeps
    /// even the meekest driver from freezing in indecision.
    pub fn change_p1(&self) -> f64 {
        0.5 + 0.5 * self.aggressive_level
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
    fn test_slow_to_start_presets_use_line_j_value() {
        let p0 = |b| BehaviourParameters::from_behaviour_type(b).slow_to_start_factor_p0();
        for b in [BehaviourType::Aggressive, BehaviourType::Cooperative, BehaviourType::LimitSpeedByTrip, BehaviourType::Undefined] {
            assert_eq!(p0(b), 0.425);
            assert!(p0(b) >= BehaviourParameters::from_behaviour_type(b).slowdown_factor_p());
        }
        assert_eq!(p0(BehaviourType::Block), 1.0);
    }

    #[test]
    fn test_standstill_gap_presets() {
        let gap = |b| BehaviourParameters::from_behaviour_type(b).standstill_gap();
        assert_eq!(gap(BehaviourType::Cooperative), 1);
        assert_eq!(gap(BehaviourType::LimitSpeedByTrip), 1);
        assert_eq!(gap(BehaviourType::Aggressive), 0);
        assert_eq!(gap(BehaviourType::Undefined), 1);
        assert_eq!(gap(BehaviourType::Block), 0);
        assert_eq!(BehaviourParameters::new(0.1, 4, 0.2, 0, 0).standstill_gap(), 0);
        assert_eq!(BehaviourParameters::new(0.1, 4, 0.2, 0, 0).with_standstill_gap(-3).standstill_gap(), 0);
    }

    #[test]
    fn test_relative_randomization_presets() {
        for b in [BehaviourType::Aggressive, BehaviourType::Cooperative, BehaviourType::LimitSpeedByTrip, BehaviourType::Undefined] {
            assert_eq!(BehaviourParameters::from_behaviour_type(b).relative_randomization(), RelativeRandomization::KKW);
        }
        assert_eq!(BehaviourParameters::new(0.1, 4, 0.2, 0, 0).relative_randomization(), RelativeRandomization::VDR);
        let custom = RelativeRandomization { stop_time_threshold: -1, decel_slower: -2, decel_equal: 1, decel_faster: 2, lookahead: 10 };
        let params = BehaviourParameters::new(0.1, 4, 0.2, 0, 0).with_relative_randomization(custom);
        assert_eq!(params.relative_randomization().stop_time_threshold, 0);
        assert_eq!(params.relative_randomization().decel_slower, 0);
        assert_eq!(params.relative_randomization().decel_faster, 2);
    }

    #[test]
    fn test_change_p1_maps_aggressiveness() {
        // P1 = 0.5 + 0.5 * aggressive_level, in [0.5, 1.0].
        let coop = BehaviourParameters::from_behaviour_type(BehaviourType::Cooperative); // aggr 0.0
        assert_eq!(coop.change_p1(), 0.5, "cooperative -> floor 0.5");
        let undef = BehaviourParameters::from_behaviour_type(BehaviourType::Undefined); // aggr 0.5
        assert_eq!(undef.change_p1(), 0.75, "middling -> 0.75");
        let aggr = BehaviourParameters::from_behaviour_type(BehaviourType::Aggressive); // aggr 0.9
        assert!((aggr.change_p1() - 0.95).abs() < 1e-9, "aggressive -> ~0.95");
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
