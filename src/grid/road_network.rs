use std::collections::HashMap;
use crate::grid::cell::{CellID, Cell};

/// `GridRoads` is a struct representing a 2D grid of cells of the road network.
/// It holds a collection of cells identified by unique `CellID`s.
///
/// This struct is used to store, retrieve, and manipulate individual cells
///
#[derive(Debug)]
pub struct GridRoads {
    // A `HashMap` mapping each `CellID` to its corresponding `Cell` object.
    cells: HashMap<CellID, Cell>,
}

impl GridRoads {
    /// Creates a new, empty `GridRoads`.
    ///
    /// # Returns
    /// A new instance of `GridRoads` with an empty `cells` collection.
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::road_network::GridRoads;
    /// let grid = GridRoads::new();
    /// ```
    pub fn new() -> Self {
        GridRoads {
            cells: HashMap::new(),
        }
    }

    /// Adds a `GridRoads` to the grid.
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
    /// use micro_traffic_sim_core::grid::road_network::GridRoads;
    /// let mut grid = GridRoads::new();
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
    /// use micro_traffic_sim_core::grid::road_network::GridRoads;
    /// let mut grid = GridRoads::new();
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

    /// Retrieves a mutable reference to a `Cell` in the grid by its `CellID`.
    ///
    /// This method provides mutable access to a cell for modification operations
    /// like updating cell state or properties.
    ///
    /// # Arguments
    /// - `id`: The `CellID` to look up.
    ///
    /// # Returns
    /// - `Option<&mut Cell>`: `Some(&mut Cell)` if found, `None` otherwise.
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::{cell::{Cell, CellState}, road_network::GridRoads};
    /// let mut grid = GridRoads::new();
    /// let cell = Cell::new(1).build();
    /// grid.add_cell(cell);
    /// 
    /// if let Some(cell) = grid.get_cell_mut(1) {
    ///     cell.set_state(CellState::Banned);
    /// }
    /// ```
    pub fn get_cell_mut(&mut self, id: CellID) -> Option<&mut Cell> {
        self.cells.get_mut(&id)
    }

    /// Returns an iterator over all cells in the grid.
    ///
    /// This method provides read-only access to all cells and their IDs
    /// for iteration and inspection.
    ///
    /// # Returns
    /// An iterator yielding `(&CellID, &Cell)` tuples.
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::{cell::Cell, road_network::GridRoads};
    /// let mut grid = GridRoads::new();
    /// grid.add_cell(Cell::new(1).build());
    /// grid.add_cell(Cell::new(2).build());
    /// 
    /// for (id, cell) in grid.iter() {
    ///     println!("Cell {}: {:?}", id, cell.get_point());
    /// }
    /// ```
    pub fn iter(&self) -> impl Iterator<Item = (&CellID, &Cell)>
    {
        self.cells.iter()
    }
}