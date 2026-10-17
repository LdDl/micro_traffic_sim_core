/// Longitudinal speed adaptation with a configurable synchronization distance.
///
/// Within floor(time_gap * speed) empty cells, a vehicle adapts by one speed unit
/// toward its leader. Safety clipping precedes the stochastic correction, following
/// Kerner, Klenov and Wolf (2002), equations (11)-(15), doi:10.1088/0305-4470/35/47/303.
/// The coarse cell size, standing threshold and network rules differ from that paper.
/// This configuration describes a KKW-inspired extension, not a literal reproduction.
///
/// A nonpositive time_gap disables the extension and preserves relative randomization.
/// All current behaviour presets and Vehicle::new keep it disabled. When enabled,
/// the extension replaces the moving slowdown and RelativeRandomization rule; the
/// vehicle's slow_to_start_factor_p0 still sets its long-stop restart probability.
/// One random draw selects a negative correction first and a positive correction
/// second. If their requested probabilities sum above one, negative noise has
/// priority and the positive interval uses only the remaining probability mass.
///
/// # Examples
///
/// ```
/// use micro_traffic_sim_core::agents::SynchronizationParameters;
/// use micro_traffic_sim_core::trips::trip::{Trip, TripType};
///
/// let trip = Trip::new(1, 50, TripType::Constant)
///     .with_synchronization(SynchronizationParameters::COARSE_GRID)
///     .with_standstill_gap_share(0.667)
///     .with_speed_limit(5)
///     .build();
/// assert_eq!(trip.synchronization, Some(SynchronizationParameters::COARSE_GRID));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SynchronizationParameters {
    /// Synchronization distance coefficient in ticks; speed is in cells per tick.
    pub time_gap: f64,
    /// Negative-correction probability below the positive effective speed limit.
    pub noise: f64,
    /// Optional negative-correction probability at the positive effective speed limit.
    /// Eligibility uses current speed. None keeps the ordinary noise probability.
    pub free_speed_noise: Option<f64>,
    /// Positive-correction probability at or above pinch_speed.
    pub over_acceleration: f64,
    /// Positive-correction probability below pinch_speed.
    pub over_acceleration_low: f64,
    /// Current speed threshold in cells per tick, after clipping to the local limit.
    pub pinch_speed: i32,
    /// Standing ticks required before slow_to_start_factor_p0 applies.
    pub start_delay: i32,
}

impl SynchronizationParameters {
    /// Preserves the existing acceleration and relative-randomization rules.
    pub const DISABLED: Self = Self {
        time_gap: 0.0,
        noise: 0.0,
        free_speed_noise: None,
        over_acceleration: 0.0,
        over_acceleration_low: 0.0,
        pinch_speed: 0,
        start_delay: 0,
    };

    /// Research calibration for 4.5 m cells and one-second ticks.
    ///
    /// Use p0=0.425 and a one-cell standstill gap for about two thirds of cars to
    /// reproduce the tested car population. These remain separate vehicle settings.
    /// The probability values are coarse-grid calibration choices, not direct paper
    /// constants. At the positive effective limit, negative noise is disabled.
    pub const COARSE_GRID: Self = Self {
        time_gap: 2.55,
        noise: 0.2 / 9.0,
        free_speed_noise: Some(0.0),
        over_acceleration: 0.052 / 9.0,
        over_acceleration_low: 0.2 / 9.0,
        pinch_speed: 4,
        start_delay: 2,
    };

    /// Clamps probabilities to [0,1] and integer thresholds to nonnegative values.
    /// Non-finite probabilities become zero; non-finite time_gap disables adaptation.
    pub fn clamped(self) -> Self {
        fn probability(value: f64) -> f64 {
            if value.is_finite() {
                value.clamp(0.0, 1.0)
            } else {
                0.0
            }
        }
        Self {
            time_gap: if self.time_gap.is_finite() {
                self.time_gap.max(0.0)
            } else {
                0.0
            },
            noise: probability(self.noise),
            free_speed_noise: self.free_speed_noise.map(probability),
            over_acceleration: probability(self.over_acceleration),
            over_acceleration_low: probability(self.over_acceleration_low),
            pinch_speed: self.pinch_speed.max(0),
            start_delay: self.start_delay.max(0),
        }
    }
}

impl Default for SynchronizationParameters {
    fn default() -> Self {
        Self::DISABLED
    }
}
