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
    /// Maximum cell speed limit across the network (cells/tick, clamped to >= 1).
    /// Maintained incrementally in `add_cell`; used as the divisor in the
    /// time-based A* heuristic to keep it admissible. Defaults to 1.0.
    max_speed: f64,
    /// Largest cell id seen so far (`-1` if empty). Maintained in `add_cell`; used to
    /// size the A* closed-set marker array.
    max_cell_id: CellID,
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
            max_speed: 1.0,
            max_cell_id: -1,
        }
    }

    /// Maximum cell speed limit across the network (cells/tick, >= 1).
    /// Divisor for the admissible time-based A* heuristic.
    pub fn get_max_speed(&self) -> f64 {
        self.max_speed
    }

    /// Largest cell id in the network (`-1` if empty). Used to size O(1) per-cell
    /// scratch arrays (e.g. the A* closed set).
    pub fn get_max_cell_id(&self) -> CellID {
        self.max_cell_id
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
        let speed = (cell.get_speed_limit() as f64).max(1.0);
        if speed > self.max_speed {
            self.max_speed = speed;
        }
        if cell.get_id() > self.max_cell_id {
            self.max_cell_id = cell.get_id();
        }
        self.cells.insert(cell.get_id(), cell);
    }

    /// Precomputes the edge cost from every cell to its forward, left and right
    /// neighbour and stores it on the cell. The cost is free-flow TRAVEL TIME
    /// (edge length / source-cell speed limit, see [`crate::shortest_path::heuristics::edge_time`]).
    /// The graph is static, so these costs never change during a session;
    /// precomputing them removes the per-relaxation haversine + division from the
    /// A* hot path. Idempotent and O(cells).
    pub fn precompute_edge_costs(&mut self) {
        use crate::geom::{Point, PointType};
        // Snapshot neighbour points first (immutable), then write costs back.
        let points: HashMap<CellID, PointType> =
            self.cells.iter().map(|(id, c)| (*id, *c.get_point())).collect();
        let mut max_speed = 1.0_f64;
        for cell in self.cells.values_mut() {
            let here = *cell.get_point();
            let speed = (cell.get_speed_limit() as f64).max(1.0);
            if speed > max_speed {
                max_speed = speed;
            }
            // Edge cost = travel time = distance / source speed.
            let time = |to_id: CellID| -> f64 {
                match points.get(&to_id) {
                    Some(p) => here.distance_to(p) / speed,
                    None => f64::NAN,
                }
            };
            let f = time(cell.get_forward_id());
            let l = time(cell.get_left_id());
            let r = time(cell.get_right_id());
            cell.set_edge_costs(f, l, r);
        }
        self.max_speed = max_speed;
    }

    /// Recomputes per-cell edge costs as congestion-aware travel time:
    /// `edge length / smoothed speed of the cell`. `cell_speed` holds the smoothed
    /// per-cell speed (cells/tick); a cell with no entry keeps free-flow time. This
    /// is the per-cell congestion model (no dependency on client-supplied link ids).
    /// Called every `adaptation_interval` ticks by the session. The heuristic stays
    /// free-flow (max speed), so it remains an admissible lower bound (real >= free-flow).
    pub fn apply_congestion(&mut self, cell_speed: &HashMap<CellID, f64>) {
        use crate::geom::{Point, PointType};
        let points: HashMap<CellID, PointType> =
            self.cells.iter().map(|(id, c)| (*id, *c.get_point())).collect();
        for cell in self.cells.values_mut() {
            let here = *cell.get_point();
            let free_flow = (cell.get_speed_limit() as f64).max(1.0);
            let speed = cell_speed
                .get(&cell.get_id())
                .copied()
                .map(|s| s.clamp(0.1, free_flow))
                .unwrap_or(free_flow);
            let time = |to_id: CellID| -> f64 {
                match points.get(&to_id) {
                    Some(p) => here.distance_to(p) / speed,
                    None => f64::NAN,
                }
            };
            let f = time(cell.get_forward_id());
            let l = time(cell.get_left_id());
            let r = time(cell.get_right_id());
            cell.set_edge_costs(f, l, r);
        }
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

    /// Returns the total number of cells in the grid.
    /// 
    /// # Returns
    /// The number of cells as a `usize`.
    /// 
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// use micro_traffic_sim_core::grid::road_network::GridRoads;
    /// let mut grid = GridRoads::new();
    /// grid.add_cell(Cell::new(1).build());
    /// grid.add_cell(Cell::new(2).build());
    /// assert_eq!(grid.get_cells_num(), 2);
    /// ```
    pub fn get_cells_num(&self) -> usize {
        self.cells.len()
    }
}