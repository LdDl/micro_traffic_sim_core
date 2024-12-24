use std::collections::HashMap;
use crate::grid::cell::{CellID, Cell};

/// `Grid` is a struct representing a 2D grid of cells of the road network.
/// It holds a collection of cells identified by unique `CellID`s.
///
/// This struct is used to store, retrieve, and manipulate individual cells
///
#[derive(Debug)]
pub struct Grid {
    // A `HashMap` mapping each `CellID` to its corresponding `Cell` object.
    cells: HashMap<CellID, Cell>,
}

impl Grid {
    /// Creates a new, empty `Grid`.
    ///
    /// # Returns
    /// A new instance of `Grid` with an empty `cells` collection.
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::road_network::Grid;
    /// let grid = Grid::new();
    /// ```
    pub fn new() -> Self {
        Grid {
            cells: HashMap::new(),
        }
    }

    /// Adds a `Cell` to the grid.
    ///
    /// This method inserts the `Cell` into the `cells` collection,
    /// using its `CellID` as the key. If a cell with the same ID already exists,
    /// it will be replaced with the new one.
    ///
    /// # Arguments
    /// - `cell`: The `Cell` to be added to the grid.
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// use micro_traffic_sim_core::grid::road_network::Grid;
    /// let mut grid = Grid::new();
    /// let cell = Cell::new(1).build();
    /// grid.add_cell(cell);
    /// ```
    pub fn add_cell(&mut self, cell: Cell) {
        self.cells.insert(cell.get_id(), cell);
    }

    /// Retrieves a reference to a `Cell` in the grid by its `CellID`.
    ///
    /// This method checks if the `CellID` exists in the grid's `cells` map
    /// and returns an `Option` containing a reference to the `Cell` if found,
    /// or `None` if the `CellID` is not present in the grid.
    ///
    /// # Arguments
    /// - `id`: A reference to the `CellID` to look up.
    ///
    /// # Returns
    /// - `Option<&Cell>`: `Some(&Cell)` if the `Cell` is found, `None` otherwise.
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// use micro_traffic_sim_core::grid::road_network::Grid;
    /// let mut grid = Grid::new();
    /// let cell = Cell::new(1).build();
    /// grid.add_cell(cell);
    ///
    /// if let Some(c) = grid.get_cell(&1) {
    ///     // Use the retrieved cell.
    /// } else {
    ///     // Handle the case where the cell is not found.
    /// }
    /// ```
    pub fn get_cell(&self, id: &CellID) -> Option<&Cell> {
        self.cells.get(id)
    }
}