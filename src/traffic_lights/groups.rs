use crate::geom::{Point, PointType};
use crate::grid::cell::CellID;
use crate::traffic_lights::signals::SignalType;

/// Represents a group of cells that share the same traffic light.
///
/// A `TrafficLightGroup` includes geometry, a list of IDs for controlled cells,
/// user-defined label, traffic signal phases, and a unique group ID.
#[derive(Debug, Clone)]
pub struct TrafficLightGroup {
    /// Geometry representing the spatial location of the group.
    geometry: Vec<PointType>,
    /// List of controlled cell IDs.
    cells_ids: Vec<CellID>,
    /// User-defined label for the group.
    label: String,
    /// List of signal phases (cycles).
    signal: Vec<SignalType>,
    /// Unique ID of the group.
    id: i64,
}

impl TrafficLightGroup {
    /// Creates a new `TrafficLightGroupBuilder` for constructing a `TrafficLightGroup`.
    ///
    /// # Arguments
    /// * `id` - A unique identifier for the traffic light group.
    ///
    /// # Returns
    /// A `TrafficLightGroupBuilder` instance for configuring and building the `TrafficLightGroup`.
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::geom::new_point;
    /// use micro_traffic_sim_core::grid::cell::CellID;
    /// use micro_traffic_sim_core::traffic_lights::signals::SignalType;
    /// use micro_traffic_sim_core::traffic_lights::groups::TrafficLightGroup;
    /// 
    /// let group = TrafficLightGroup::new(1)
    ///     .with_label("new signal group".to_string())
    ///     .with_geometry(vec![new_point(0.0, 0.0, None), new_point(1.0, 1.0, None)])
    ///     .with_cells_ids(vec![20])
    ///     .with_signal(vec![SignalType::Red, SignalType::Yellow, SignalType::Green])
    ///     .build();
    /// ```
    pub fn new(id: i64) -> TrafficLightGroupBuilder {
        TrafficLightGroupBuilder {
            group: TrafficLightGroup {
                geometry: Vec::new(),
                cells_ids: Vec::new(),
                label: String::new(),
                signal: Vec::new(),
                id,
            },
        }
    }
    /// Returns the unique identifier (ID) of the traffic light group
    pub fn get_id(&self) -> i64 {
        self.id
    }
}

/// A builder for constructing `TrafficLightGroup` instances.
pub struct TrafficLightGroupBuilder {
    group: TrafficLightGroup,
}

impl TrafficLightGroupBuilder {
    /// Sets the user-defined label for the group.
    ///
    /// # Arguments
    /// * `label` - A string representing the label.
    ///
    /// # Returns
    /// A `TrafficLightGroupBuilder` instance for method chaining.
    pub fn with_label(mut self, label: String) -> Self {
        self.group.label = label;
        self
    }

    /// Sets the geometry (points) for the group, overwriting any existing values.
    ///
    /// # Arguments
    /// * `geometry` - A vector of `Point` instances.
    ///
    /// # Returns
    /// A `TrafficLightGroupBuilder` instance for method chaining.
    pub fn with_geometry(mut self, geometry: Vec<PointType>) -> Self {
        self.group.geometry = geometry;
        self
    }

    /// Sets the controlled cell IDs for the group, overwriting any existing values.
    ///
    /// # Arguments
    /// * `cells_ids` - A vector of `CellID` instances.
    ///
    /// # Returns
    /// A `TrafficLightGroupBuilder` instance for method chaining.
    pub fn with_cells_ids(mut self, cells_ids: Vec<CellID>) -> Self {
        self.group.cells_ids = cells_ids;
        self
    }

    /// Sets the signal phases for the group, overwriting any existing values.
    ///
    /// # Arguments
    /// * `signal` - A vector of `SignalType` instances.
    ///
    /// # Returns
    /// A `TrafficLightGroupBuilder` instance for method chaining.
    pub fn with_signal(mut self, signal: Vec<SignalType>) -> Self {
        self.group.signal = signal;
        self
    }

    /// Builds the final `TrafficLightGroup` object.
    ///
    /// # Returns
    /// A fully constructed `TrafficLightGroup` instance.
    pub fn build(self) -> TrafficLightGroup {
        self.group
    }
}