use crate::geom::{PointType, new_point};
use crate::traffic_lights::groups::TrafficLightGroup;
use std::fmt;

/// Type alias for TrafficLight identifiers.
pub type TrafficLightID = i64;

/// Error types for TrafficLight operations.
#[derive(Debug)]
pub enum TrafficLightError {
    NotFound,
    AlreadyExists,
}

impl fmt::Display for TrafficLightError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TrafficLightError::NotFound => write!(f, "No traffic light was found"),
            TrafficLightError::AlreadyExists => write!(f, "Traffic light already exists"),
        }
    }
}

impl std::error::Error for TrafficLightError {}

/// Represents a traffic light that controls one or more groups of cells.
///
/// # Examples
///
/// ```rust
/// use micro_traffic_sim_core::traffic_lights::{
///     lights::{TrafficLight, TrafficLightID},
///     groups::TrafficLightGroup,
///     signals::SignalType
/// };
/// use micro_traffic_sim_core::geom::new_point;
///
/// // Create signal groups for each direction
/// let north_south_group = TrafficLightGroup::new(1)
///     .with_label("North-South".to_string())
/// // Cells for N-S traffic
///     .with_cells_ids(vec![1, 2, 3])
///     .with_signal(vec![SignalType::Green, SignalType::Yellow, SignalType::Red])
///     .build();
///
/// let east_west_group = TrafficLightGroup::new(2)
///     .with_label("East-West".to_string())
/// // Cells for E-W traffic
///     .with_cells_ids(vec![10, 11, 12])
///     .with_signal(vec![SignalType::Red, SignalType::RedYellow, SignalType::Green])
///     .build();
///
/// // Create traffic light with timing
/// let mut intersection = TrafficLight::new(1)
///     .with_coordinates(new_point(100.0, 200.0, None))
///     .with_groups(vec![north_south_group, east_west_group])
/// // Green: 15s, Yellow: 3s, Red: 15s
///     .with_phases_times(vec![15, 3, 15])
///     .build();
///
/// // Simulate traffic light operation
/// for step in 0..40 {
///     intersection.step();
///     let phase = intersection.get_active_phase();
///     let time = intersection.get_current_time();
///     println!("Step {}: Phase {}, Time {}", step, phase, time);
/// }
/// ```
///
#[derive(Debug, Clone)]
pub struct TrafficLight {
    /// Groups of cells (so it is just signal groups for specific directions in the junction) controlled by the traffic light.
    groups: Vec<TrafficLightGroup>,
    /// Times for each signal phase.
    times: Vec<i32>,
    /// Coordinates (longitude/latitude).
    coordinates: PointType,
    /// Traffic light identifier.
    id: TrafficLightID,
    /// Internal timer.
    timer: i32,
    /// Index of the current active signal phase.
    active_phase_idx: usize,
}

impl TrafficLight {
    /// Creates a new `TrafficLightBuilder` for constructing a `TrafficLight`.
    ///
    /// # Arguments
    /// * `id` - The identifier for the traffic light.
    ///
    /// # Returns
    /// A `TrafficLightBuilder` instance for configuring and building the traffic light.
    pub fn new(id: TrafficLightID) -> TrafficLightBuilder {
        TrafficLightBuilder {
            traffic_light: TrafficLight {
                groups: Vec::new(),
                times: Vec::new(),
                coordinates: new_point(-1.0, -1.0, None),
                id,
                timer: 0,
                active_phase_idx: 0,
            },
        }
    }
    /// Returns the unique identifier (ID) of the traffic light
    pub fn get_id(&self) -> TrafficLightID {
        self.id
    }
    /// Increments internal timer and changes active phase if needed
    pub fn step(&mut self) {
        self.timer += 1;
        let current_phase = self.active_phase_idx;
        if self.timer >= self.times[current_phase] {
            self.active_phase_idx = (current_phase + 1) % self.times.len();
            self.timer = 0;
        }
    }

    /// Resets the traffic light's internal timer and phase index.
    pub fn reset(&mut self) {
        self.timer = 0;
        self.active_phase_idx = 0;
    }

    /// Returns the current active phase index.
    pub fn get_active_phase(&self) -> usize {
        self.active_phase_idx
    }

    /// Returns the current time of the traffic light's internal timer.
    pub fn get_current_time(&self) -> i32 {
        self.timer
    }

    /// Return number of signal groups
    pub fn get_groups_num(&self) -> usize {
        self.groups.len()
    }

    /// Returns a reference to the traffic light's groups.
    pub fn get_groups(&self) -> &Vec<TrafficLightGroup> {
        &self.groups
    }

    /// Return coordinates of the traffic light
    pub fn get_coordinates(&self) -> PointType {
        self.coordinates
    }
}

/// A builder for constructing `TrafficLight` instances.
pub struct TrafficLightBuilder {
    traffic_light: TrafficLight,
}

impl TrafficLightBuilder {
    /// Sets the coordinates for the traffic light.
    pub fn with_coordinates(mut self, point: PointType) -> Self {
        self.traffic_light.coordinates = point;
        self
    }

    /// Sets the signal groups for the traffic light.
    /// @todo: consider &Vec<TrafficLightGroup> and groups.clone() may be?
    pub fn with_groups(mut self, groups: Vec<TrafficLightGroup>) -> Self {
        self.traffic_light.groups = groups;
        self
    }

    /// Appends signal groups to the existing groups for the traffic light.
    pub fn with_groups_append(mut self, groups: Vec<TrafficLightGroup>) -> Self {
        self.traffic_light.groups.extend(groups);
        self
    }

    /// Sets the signal phase times for the traffic light.
    pub fn with_phases_times(mut self, phases_time: Vec<i32>) -> Self {
        self.traffic_light.times = phases_time;
        self
    }

    /// Appends signal phase times to the existing ones for the traffic light.
    pub fn with_phases_times_append(mut self, phases_time: Vec<i32>) -> Self {
        self.traffic_light.times.extend(phases_time);
        self
    }

    /// Sets the active phase index for the traffic light.
    pub fn with_active_phase(mut self, phase_idx: usize) -> Self {
        self.traffic_light.active_phase_idx = phase_idx;
        self
    }

    /// Builds and returns the final `TrafficLight` instance.
    pub fn build(self) -> TrafficLight {
        self.traffic_light
    }
}
