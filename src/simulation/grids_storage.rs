use crate::grid::road_network::GridRoads;

#[derive(Debug)]
pub struct GridsStorage {
    // The grid intented for vehicles only
    vehicles_net: GridRoads,
    // @future: there will be other grids and data: for pedestrian network, for crosswalks, for
    // traffic lights and etc.
}

/// A builder pattern implementation for constructing `GridsStorage` objects.
///
/// `GridsStorageBuilder` allows for optional configuration of `GridsStorage` fields before building the final `GridsStorage` object.
pub struct GridsStorageBuilder {
    storage: GridsStorage,
}

impl GridsStorage {
    /// Constructs a new `GridsStorageBuilder` for building a `GridsStorage` object.
    ///
    /// # Arguments
    ///
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::simulation::grids_storage::GridsStorage;
    /// let storage = GridsStorage::new()
    ///     .build();
    /// println!("{:?}", storage);
    /// ```
    pub fn new() -> GridsStorageBuilder {
        GridsStorageBuilder {
            storage: GridsStorage {
                vehicles_net: GridRoads::new(),
            },
        }
    }
}

impl GridsStorageBuilder {
    /// Puts the grid for vehicles only into the storage
    ///
    /// # Arguments
    ///
    /// * `vehicle_grid` - the grid intented for vehicles only
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::grid::cell::Cell;
    /// use micro_traffic_sim_core::grid::road_network::GridRoads;
    /// use micro_traffic_sim_core::simulation::grids_storage::GridsStorage;
    ///
    /// let mut grid = GridRoads::new();
    /// let cell1 = Cell::new(1).build();
    /// let cell2 = Cell::new(2).build();
    /// grid.add_cell(cell1);
    /// grid.add_cell(cell2);
    ///
    /// let storage = GridsStorage::new()
    ///     .with_vehicles_net(grid)
    ///     .build();
    /// println!("{:?}", storage);
    /// ```
    pub fn with_vehicles_net(mut self, vehicles_grid: GridRoads) -> Self {
        self.storage.vehicles_net = vehicles_grid;
        self
    }

    /// Builds the final `GridsStorage` object with the configured properties.
    ///
    /// # Returns
    /// The fully constructed `GridsStorage` object.
    pub fn build(self) -> GridsStorage {
        self.storage
    }
}
