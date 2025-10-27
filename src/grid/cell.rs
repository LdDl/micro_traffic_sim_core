use crate::geom::{new_point, Point, PointType};
use crate::grid::zones::ZoneType;
use std::fmt;

/// Represents different possible states of a cell.
///
/// `CellState` indicates the current condition of a cell, such as whether it is free or banned.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CellState {
    /// The cell is free and available for use.
    Free = 0,
    /// The cell is banned and cannot be used by agents.
    Banned,
}

impl fmt::Display for CellState {
    /// Formats the cell state for display.
    /// 
    /// Returns a short, lowercase string representation suitable for
    /// logging, debugging, and user interfaces.
    /// 
    /// # Examples
    /// 
    /// ```rust
    /// use micro_traffic_sim_core::grid::cell::CellState;
    ///
    /// assert_eq!(format!("{}", CellState::Free), "free");
    /// assert_eq!(format!("{}", CellState::Banned), "banned");
    /// ```
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            CellState::Free => write!(f, "free"),
            CellState::Banned => write!(f, "banned"),
        }
    }
}

pub type CellID = i64; // Alias for CellID

/// Represents a single cell in the road network.
///
/// A `Cell` represents a location within the road network and contains properties like its ID, position, type,
/// speed limit, neighboring cells, and its current state.
#[derive(Debug, Clone)]
pub struct Cell {
    /// Unique identifier for the cell.
    id: CellID,
    /// Coordinates of the cell in the road network.
    point: PointType,
    /// The zone type to which the cell belongs (e.g., Birth, Death, etc.).
    type_zone: ZoneType,
    /// The speed limit for vehicles moving through the cell. Since cell is cellular automata entity it should be integer rather that decimal
    speed_limit: i32,
    /// Identifier for the left neighboring cell.
    left_cell: CellID,
    /// Identifier for the forward neighboring cell.
    forward_cell: CellID,
    /// Identifier for the right neighboring cell.
    right_cell: CellID,
    /// Identifier linking the cell to a mesoscopic graph (if applicable).
    meso_link_id: i64,
    /// Current state of the cell (e.g., free, banned).
    state: CellState,
}

impl Cell {
    /// Constructs a new `CellBuilder` for building a `Cell` object.
    ///
    /// # Arguments
    /// * `id` - A unique identifier for the cell.
    ///
    /// # Returns
    /// A `CellBuilder` struct which is used to configure and build the `Cell` object.
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::geom::{new_point};
    /// use micro_traffic_sim_core::grid::zones::ZoneType;
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// let cell = Cell::new(1)
    ///     .with_point(new_point(10.0, 20.0, None))
    ///     .with_zone_type(ZoneType::Birth)
    ///     .build();
    /// ```
    pub fn new(id: CellID) -> CellBuilder {
        CellBuilder {
            cell: Cell {
                id,
                point: new_point(-1.0, -1.0, None),
                type_zone: ZoneType::Undefined,
                speed_limit: -1,
                left_cell: -1,
                forward_cell: -1,
                right_cell: -1,
                meso_link_id: -1,
                state: CellState::Free,
            },
        }
    }

    /// Calculates the Euclidean distance to another cell.
    ///
    /// # Arguments
    /// * `other` - Another `Cell` object to which the distance is computed.
    ///
    /// # Returns
    /// A floating-point value representing the distance between the two cells in the road network.
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::geom::{new_point, SRID};
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// let cell1 = Cell::new(1)
    ///     .with_point(new_point(37.61556, 55.75222, Some(SRID::WGS84)))
    ///     .build();
    /// let cell2 = Cell::new(2)
    ///     .with_point(new_point(30.31413, 59.93863, Some(SRID::WGS84)))
    ///     .build();
    /// let distance = cell1.distance_to(&cell2);
    /// println!("Distance: {}", distance);
    /// ```
    pub fn distance_to(&self, other: &Cell) -> f64 {
        self.point.distance_to(&other.point)
    }

    /// Sets a new state for the cell.
    ///
    /// # Arguments
    /// * `new_state` - The new `CellState` to set for the cell.
    pub fn set_state(&mut self, new_state: CellState) {
        self.state = new_state;
    }

    /// Returns the current state of the cell.
    ///
    /// # Returns
    /// The current `CellState` of the cell.
    pub fn get_state(&self) -> CellState {
        self.state
    }

    /// Returns the unique identifier (ID) of the cell.
    ///
    /// # Returns
    /// The `CellID` of the cell.
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// let cell = Cell::new(1).build();
    /// println!("Cell ID: {}", cell.get_id());
    /// ```
    pub fn get_id(&self) -> CellID {
        self.id
    }

    /// Returns the unique identifier (ID) of the cell in front of the current cell.
    ///
    /// # Returns
    /// The `CellID` of the cell. It could be "-1" if ID has not been set
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// let cell = Cell::new(1).with_forward_node(2).build();
    /// println!("Cell ID in front: {}", cell.get_forward_id());
    /// ```
    pub fn get_forward_id(&self) -> CellID {
        self.forward_cell
    }

    /// Sets the unique identifier (ID) of the cell in front of the current cell.
    ///
    /// # Parameters
    /// * `id` - The `CellID` of the cell in front.
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// let mut cell = Cell::new(1).build();
    /// cell.set_forward_id(2);
    /// println!("Forward Cell ID: {}", cell.get_forward_id());
    /// ```
    pub fn set_forward_id(&mut self, id: CellID) {
        self.forward_cell = id;
    }

    /// Returns the unique identifier (ID) of the cell to the left from the current cell.
    ///
    /// # Returns
    /// The `CellID` of the cell. It could be "-1" if ID has not been set
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// let cell = Cell::new(1).with_left_node(3).build();
    /// println!("Cell ID: {} to the left", cell.get_left_id());
    /// ```
    pub fn get_left_id(&self) -> CellID {
        self.left_cell
    }

    /// Sets the unique identifier (ID) of the cell to the left of the current cell.
    ///
    /// # Parameters
    /// * `id` - The `CellID` of the cell to the left.
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// let mut cell = Cell::new(1).build();
    /// cell.set_left_id(3);
    /// println!("Left Cell ID: {}", cell.get_left_id());
    /// ```
    pub fn set_left_id(&mut self, id: CellID) {
        self.left_cell = id;
    }

    /// Returns the unique identifier (ID) of the cell to the right from the current cell.
    ///
    /// # Returns
    /// The `CellID` of the cell. It could be "-1" if ID has not been set
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// let cell = Cell::new(1).with_left_node(4).build();
    /// println!("Cell ID: {} to the right", cell.get_right_id());
    /// ```
    pub fn get_right_id(&self) -> CellID {
        self.right_cell
    }

    /// Sets the unique identifier (ID) of the cell to the right of the current cell.
    ///
    /// # Parameters
    /// * `id` - The `CellID` of the cell to the right.
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// let mut cell = Cell::new(1).build();
    /// cell.set_right_id(4);
    /// println!("Right Cell ID: {}", cell.get_right_id());
    /// ```
    pub fn set_right_id(&mut self, id: CellID) {
        self.right_cell = id;
    }

    /// Returns speed limit in the cell.
    ///
    /// # Returns
    /// The speed limit in the cell.
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// let cell = Cell::new(1).with_speed_limit(60).build();
    /// println!("Speed limit: {}", cell.get_speed_limit());
    /// ```
    pub fn get_speed_limit(&self) -> i32 {
        self.speed_limit
    }

    /// Returns the coordinates of the cell.
    ///
    /// # Returns
    /// A `PointType` representing the coordinates of the cell.
    /// 
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::geom::{new_point, SRID};
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// let cell = Cell::new(1)
    ///    .with_point(new_point(37.61556, 55.75222, Some(SRID::WGS84)))
    ///   .build();
    /// println!("Cell coordinates: {:?}", cell.get_point());
    /// ```
    pub fn get_point(&self) -> &PointType {
        &self.point
    }

    /// Returns the zone type of the cell.
    /// 
    /// # Returns
    /// The `ZoneType` of the cell.
    /// 
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// use micro_traffic_sim_core::grid::zones::ZoneType;
    /// let cell = Cell::new(1).with_zone_type(ZoneType::Birth).build();
    /// println!("Cell zone type: {:?}", cell.get_zone_type());
    /// ```
    pub fn get_zone_type(&self) -> ZoneType {
        self.type_zone
    }
}

/// A builder pattern implementation for constructing `Cell` objects.
///
/// `CellBuilder` allows for optional configuration of `Cell` fields before building the final `Cell` object.
pub struct CellBuilder {
    cell: Cell,
}

impl CellBuilder {
    /// Sets the point (coordinates) for the cell.
    ///
    /// # Arguments
    /// * `point` - The `Point` struct containing the cell's coordinates.
    ///
    /// # Returns
    /// A `CellBuilder` instance for further method chaining.
    pub fn with_point(mut self, point: PointType) -> Self {
        self.cell.point = point;
        self
    }

    /// Sets the zone type for the cell.
    ///
    /// # Arguments
    /// * `zone_type` - The `ZoneType` to assign to the cell.
    ///
    /// # Returns
    /// A `CellBuilder` instance for further method chaining.
    pub fn with_zone_type(mut self, zone_type: ZoneType) -> Self {
        self.cell.type_zone = zone_type;
        self
    }

    /// Sets the speed limit for the cell.
    ///
    /// # Arguments
    /// * `speed_limit` - An integer value representing the speed limit in the cell.
    ///
    /// # Returns
    /// A `CellBuilder` instance for further method chaining.
    pub fn with_speed_limit(mut self, speed_limit: i32) -> Self {
        self.cell.speed_limit = speed_limit;
        self
    }

    /// Sets the identifier for the forward (neighboring) cell.
    ///
    /// # Arguments
    /// * `forward_cell` - The `CellID` representing the forward neighbor.
    ///
    /// # Returns
    /// A `CellBuilder` instance for further method chaining.
    pub fn with_forward_node(mut self, forward_cell: CellID) -> Self {
        self.cell.forward_cell = forward_cell;
        self
    }

    /// Sets the identifier for the left (neighboring) cell.
    ///
    /// # Arguments
    /// * `left_cell` - The `CellID` representing the left neighbor.
    ///
    /// # Returns
    /// A `CellBuilder` instance for further method chaining.
    pub fn with_left_node(mut self, left_cell: CellID) -> Self {
        self.cell.left_cell = left_cell;
        self
    }

    /// Sets the identifier for the right (neighboring) cell.
    ///
    /// # Arguments
    /// * `right_cell` - The `CellID` representing the right neighbor.
    ///
    /// # Returns
    /// A `CellBuilder` instance for further method chaining.
    pub fn with_right_node(mut self, right_cell: CellID) -> Self {
        self.cell.right_cell = right_cell;
        self
    }

    /// Sets the mesoscopic link identifier for the cell.
    ///
    /// # Arguments
    /// * `meso_link_id` - The integer value representing the mesoscopic link.
    ///
    /// # Returns
    /// A `CellBuilder` instance for further method chaining.
    pub fn with_meso_link(mut self, meso_link_id: i64) -> Self {
        self.cell.meso_link_id = meso_link_id;
        self
    }

    /// Builds the final `Cell` object with the configured properties.
    ///
    /// # Returns
    /// The fully constructed `Cell` object.
    pub fn build(self) -> Cell {
        self.cell
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::SRID;
    #[test]
    fn test_cells_gc_distance() {
        let correct_distance = 634430.92026;
        let cell1 = Cell::new(1)
            .with_point(new_point(37.61556, 55.75222, Some(SRID::WGS84)))
            .build();
        let cell2 = Cell::new(2)
            .with_point(new_point(30.31413, 59.93863, Some(SRID::WGS84)))
            .build();
        let distance = cell1.distance_to(&cell2);
        // Assert that the absolute difference is less than a small threshold
        assert!(
            (distance - correct_distance).abs() < 0.001,
            "Distance should be {}, but got {}",
            correct_distance,
            distance
        );
    }
}
