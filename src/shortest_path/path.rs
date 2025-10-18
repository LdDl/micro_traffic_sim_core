use crate::grid::cell::Cell;
use crate::maneuver::LaneChangeType;

/// Represents a path between two cells.
#[derive(Debug)]
pub struct Path<'a> {
    /// A sequence of references to cells that define the path.
    vertices: Vec<&'a Cell>,
    /// A list of maneuvers between cells in the path. Length = vertices.len() - 1.
    maneuvers: Vec<LaneChangeType>,
    /// The total cost of the path.
    cost: f64,
}

impl<'a> Path<'a> {
    /// Creates a new `Path` instance.
    ///
    /// # Arguments
    /// * `vertices` - A vector of references to cells representing the path.
    /// * `maneuvers` - A vector of lane change maneuvers for the path.
    /// * `cost` - The total cost of the path.
    ///
    /// # Panics
    /// Panics if the length of `maneuvers` is not `vertices.len() - 1`.
    ///
    /// # Examples
    ///
    /// ```
    /// use micro_traffic_sim_core::shortest_path::path::Path;
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// use micro_traffic_sim_core::maneuver::LaneChangeType;
    ///
    /// let cell1 = Cell::new(1).build();
    /// let cell2 = Cell::new(2).build();
    /// let cell3 = Cell::new(3).build();
    /// let vertices = vec![&cell1, &cell2, &cell3];
    /// let maneuvers = vec![LaneChangeType::NoChange, LaneChangeType::ChangeLeft];
    /// let cost = 10.0;
    ///
    /// let path = Path::new(vertices, maneuvers, cost);
    /// println!("Path: {:?}", path);
    /// ```
    pub fn new(vertices: Vec<&'a Cell>, maneuvers: Vec<LaneChangeType>, cost: f64) -> Self {
        assert_eq!(
            vertices.len().saturating_sub(1),
            maneuvers.len(),
            "Number of maneuvers must be one less than the number of vertices"
        );

        Path {
            vertices,
            maneuvers,
            cost,
        }
    }

    /// Returns a reference to the vertices in the path.
    pub fn vertices(&self) -> &Vec<&'a Cell> {
        &self.vertices
    }

    /// Returns a reference to the maneuvers in the path.
    pub fn maneuvers(&self) -> &Vec<LaneChangeType> {
        &self.maneuvers
    }

    /// Returns a mutable reference to the vertices in the path.
    pub fn vertices_mut(&mut self) -> &mut Vec<&'a Cell> {
        &mut self.vertices
    }

    /// Returns a mutable reference to the maneuvers in the path.
    pub fn maneuvers_mut(&mut self) -> &mut Vec<LaneChangeType> {
        &mut self.maneuvers
    }
    
    /// Returns the total cost of the path.
    pub fn cost(&self) -> f64 {
        self.cost
    }
}