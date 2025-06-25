use std::collections::HashMap;

use crate::grid::road_network::GridRoads;
use crate::grid::cell::{CellID};
use crate::traffic_lights::lights::{TrafficLightID, TrafficLight};
use crate::traffic_lights::signals::SignalType;
use crate::simulation::states::{TrafficLightGroupState};
use crate::verbose::*;
use std::fmt;

/// Custom error types for `Session`.
#[derive(Debug, Clone)]
pub enum GridsStorageError {
    /// Indicates that a cell with the given ID was not found
    CellInGroupNotFound(CellID, i64),
}

impl fmt::Display for GridsStorageError {
    /// Formats the error message for `GridsStorageError`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GridsStorageError::CellInGroupNotFound(cell_id, group_id) => {
                write!(f, "Cell with ID {} not found in group with ID {}", cell_id, group_id)
            },
        }
    }
}

impl std::error::Error for GridsStorageError {}

#[derive(Debug)]
pub struct GridsStorage {
    // The grid intented for vehicles only
    vehicles_net: GridRoads,
    // @future: there will be other grids and data: for pedestrian network, for crosswalks, for
    // traffic lights and etc.
    tls: HashMap<TrafficLightID, TrafficLight>,
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
                tls: HashMap::new(),
            },
        }
    }

    /// Returns the number of traffic lights in the storage.
    pub fn tls_num(&self) -> usize {
        self.tls.len()
    }

    /// Resets timers and  active phases for every traffic light
    pub fn tls_reset(&mut self) {
        for light in self.tls.values_mut() {
            light.reset();
        }
    }

    /// Adds set of cells to the vehicles grid.
    pub fn add_cells(&mut self, cells_data: Vec<crate::grid::cell::Cell>) {
        for cell in cells_data {
            self.vehicles_net.add_cell(cell);
        }
    }

    /// Add single traffic light to the storage.
    pub fn add_traffic_light(&mut self, traffic_light: TrafficLight) {
        self.tls.insert(traffic_light.get_id(), traffic_light);
    }

    /// Returns a reference to the vehicles grid.
    pub fn get_vehicles_net_ref(&self) -> &GridRoads {
        &self.vehicles_net
    }

    /// Returns a mutable reference to the vehicles grid.
    pub fn get_vehicles_net_mut(&mut self) -> &mut GridRoads {
        &mut self.vehicles_net
    }

    /// Returns a reference to the traffic lights storage.
    pub fn get_tls_ref(&self) -> &HashMap<TrafficLightID, TrafficLight> {
        &self.tls
    }

    /// Returns a mutable reference to the traffic lights storage.
    pub fn get_tls_mut(&mut self) -> &mut HashMap<TrafficLightID, TrafficLight> {
        &mut self.tls
    }

    pub fn tick_traffic_lights(&mut self, verbose: VerboseLevel) -> Result<HashMap<TrafficLightID, Vec<TrafficLightGroupState>>, GridsStorageError> {
        if verbose.is_at_least(VerboseLevel::Main) {
            verbose.log_with_fields(
                EVENT_TL_TICK,
                "Tick on traffic lights",
                &[("tl_num", &self.tls.len())]
            );
        }
        let mut tl_states = HashMap::new();
        for (tl_id, tl) in self.tls.iter_mut() {
            if verbose.is_at_least(VerboseLevel::Additional) {
                verbose.log_with_fields(
                    EVENT_TL_TICK,
                    "Tick on previous step on traffic light",
                    &[
                        ("tl_id", &format!("{:?}", tl_id)),
                        ("active_phase", &tl.get_active_phase()),
                        ("tl_timer", &tl.get_current_time()),
                    ]
                );
            }
            tl.step();
            let active_phase_idx = tl.get_active_phase();
            let mut group_states = Vec::new();
            let tl_groups = tl.get_groups();
            for group in tl_groups {
                let active_signal = group.get_signal_at(active_phase_idx);
                let is_banned = match active_signal {
                    SignalType::Red |
                    SignalType::Yellow |
                    SignalType::RedYellow |
                    SignalType::Blinking => true,
                    _ => false,
                };
                let cells_ids = group.get_cells_ids();
                for &cell_id in cells_ids {
                    if let Some(cell) = self.vehicles_net.get_cell_mut(cell_id) {
                        if is_banned {
                            cell.set_state(crate::grid::cell::CellState::Banned);
                        } else {
                            cell.set_state(crate::grid::cell::CellState::Free);
                        }
                    } else {
                        return Err(GridsStorageError::CellInGroupNotFound(cell_id, group.get_id()));
                    }
                }
                group_states.push(TrafficLightGroupState {
                    group_id: group.get_id(),
                    last_signal: *active_signal,
                });
            }
            tl_states.insert(*tl_id, group_states);
        }
        Ok(tl_states)
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
