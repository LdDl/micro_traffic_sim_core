use crate::geom::{PointType, new_point};
use crate::traffic_lights::groups::TrafficLightGroup;
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TrafficLightID(pub i64);

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::traffic_lights::signals::SignalType;
    #[test]
    fn test_traffic_light() {
        let groups = vec![
            TrafficLightGroup::new(1)
                .with_label("Group 1".to_string())
                .with_cells_ids(vec![20])
                .with_signal(vec![SignalType::Red, SignalType::Yellow, SignalType::Green])
                .build(),
            TrafficLightGroup::new(2)
                .with_label("Group 2".to_string())
                .with_cells_ids(vec![16])
                .with_signal(vec![SignalType::Green, SignalType::Green, SignalType::Green])
                .build(),
            TrafficLightGroup::new(3)
                .with_label("Group 3".to_string())
                .with_cells_ids(vec![11])
                .with_signal(vec![SignalType::Red, SignalType::Red, SignalType::Red])
                .build(),
            TrafficLightGroup::new(4)
                .with_label("Group 4".to_string())
                .with_cells_ids(vec![18, 20])
                .with_signal(vec![SignalType::Red, SignalType::Red, SignalType::Red])
                .build(),
        ];

        let mut traffic_light = TrafficLight::new(TrafficLightID(1))
            .with_groups(groups)
            .with_phases_times(vec![2, 3, 10])
            .with_active_phase(0)
            .build();

        assert_eq!(traffic_light.get_current_time(), 0);
        assert_eq!(traffic_light.get_active_phase(), 0);

        // Simulate time steps
        traffic_light.step();
        assert_eq!(traffic_light.get_current_time(), 1);
        assert_eq!(traffic_light.get_active_phase(), 0);

        // Timer should reset because first time is 2
        // Phase has to be changed to 1
        traffic_light.step();
        assert_eq!(traffic_light.get_current_time(), 0);
        assert_eq!(traffic_light.get_active_phase(), 1);

        traffic_light.step();
        assert_eq!(traffic_light.get_current_time(), 1);
        assert_eq!(traffic_light.get_active_phase(), 1);

        traffic_light.step();
        assert_eq!(traffic_light.get_current_time(), 2);
        assert_eq!(traffic_light.get_active_phase(), 1);

        // Timer should reset because second time is 3
        // Phase has to be changed to 2
        traffic_light.step();
        assert_eq!(traffic_light.get_current_time(), 0);
        assert_eq!(traffic_light.get_active_phase(), 2);

        traffic_light.step();
        assert_eq!(traffic_light.get_current_time(), 1);
        assert_eq!(traffic_light.get_active_phase(), 2);

        traffic_light.step();
        assert_eq!(traffic_light.get_current_time(), 2);
        assert_eq!(traffic_light.get_active_phase(), 2);

        traffic_light.step();
        assert_eq!(traffic_light.get_current_time(), 3);
        assert_eq!(traffic_light.get_active_phase(), 2);

        traffic_light.step();
        assert_eq!(traffic_light.get_current_time(), 4);
        assert_eq!(traffic_light.get_active_phase(), 2);

        traffic_light.step();
        assert_eq!(traffic_light.get_current_time(), 5);
        assert_eq!(traffic_light.get_active_phase(), 2);

        traffic_light.step();
        assert_eq!(traffic_light.get_current_time(), 6);
        assert_eq!(traffic_light.get_active_phase(), 2);

        traffic_light.step();
        assert_eq!(traffic_light.get_current_time(), 7);
        assert_eq!(traffic_light.get_active_phase(), 2);

        traffic_light.step();
        assert_eq!(traffic_light.get_current_time(), 8);
        assert_eq!(traffic_light.get_active_phase(), 2);

        traffic_light.step();
        assert_eq!(traffic_light.get_current_time(), 9);
        assert_eq!(traffic_light.get_active_phase(), 2);

        // Timer should reset because third time is 10
        // Phase has to be changed to 0 (reset) since there are only 3 phases and we've reached the end of the second one
        traffic_light.step();
        assert_eq!(traffic_light.get_current_time(), 0);
        assert_eq!(traffic_light.get_active_phase(), 0);
    }
}