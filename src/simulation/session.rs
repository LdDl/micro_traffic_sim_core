use crate::agents::{VehicleID, VehicleRef};
use crate::conflict_zones::{ConflictZone, ConflictZoneID};
use crate::grid::cell::CellID;
use crate::traffic_lights::lights::{TrafficLightID};
use crate::trips::trip::{Trip, TripID};
use crate::simulation::grids_storage::GridsStorage;
use crate::geom::SRID;
use std::collections::HashMap;
use uuid::Uuid;
use std::fmt;
use std::time::{SystemTime, UNIX_EPOCH};

/// Custom error types for `Session`.
#[derive(Debug, Clone)]
pub enum SessionError {
    /// Indicates that some error occurred
    ErrorPlaceholder(String),
}

impl fmt::Display for SessionError {
    /// Formats the error message for `SessionError`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SessionError::ErrorPlaceholder(value) => {
                write!(f, "ErrorPlaceholder: {}", value)
            }
        }
    }
}

impl std::error::Error for SessionError {}

/// Session - representation of session for Cellular Automata with Traffic lights control management
pub struct Session {
    /// Current position mapping from cell ID to vehicle ID
    current_position: HashMap<CellID, VehicleID>,

    /// Cellular automata grid storage
    grids_storage: GridsStorage,

    /// Trips for automatic vehicle generation
    trips_data: HashMap<TripID, Trip>,

    /// Vehicles storage
    vehicles: HashMap<VehicleID, VehicleRef>,

    /// Cells under traffic lights control
    /// It could be just Cell, but we'll use CellID for now
    coordination_cells: HashMap<CellID, CellID>,

    /// Information about conflicts zones and corresponding cells
    conflict_zones: HashMap<ConflictZoneID, ConflictZone>,
    cells_conflicts_zones: HashMap<CellID, ConflictZoneID>,

    /// Unique session identifier
    id: Uuid,

    /// Simulation info - number of steps executed
    steps: i32,

    /// Last applied vehicle identifier
    last_vehicle_id: VehicleID,

    /// Debugging information level
    verbose: bool,

    /// Time when this session has been created or updated (nanoseconds)
    updated_at: i64,

    /// Time when to destroy this session (nanoseconds)
    expire_at: i64,

    /// Defines the SRID of the world
    world_srid: SRID,
}

impl Session {
    /// Creates new session with default values for provided cell grid
    pub fn default(srid: Option<SRID>) -> Self {
        let picked_srid = srid.unwrap_or(SRID::Euclidean);
        Session {
            id: Uuid::new_v4(),
            last_vehicle_id: 1,
            vehicles: HashMap::new(),
            grids_storage: GridsStorage::new().build(),
            trips_data: HashMap::new(),
            verbose: false,
            coordination_cells: HashMap::new(),
            conflict_zones: HashMap::new(),
            cells_conflicts_zones: HashMap::new(),
            current_position: HashMap::new(),
            updated_at: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as i64,
            expire_at: 0,
            steps: 0,
            world_srid: picked_srid,
        }
    }

    /// Creates new session for cellular automata for provided cell grid
    /// Equivalent to Go's NewSessionFrom
    pub fn new(grids_storage: GridsStorage, srid: Option<SRID>) -> Self {
        let picked_srid = srid.unwrap_or(SRID::Euclidean);
        
        Session {
            id: Uuid::new_v4(),
            last_vehicle_id: 1,
            vehicles: HashMap::new(),
            grids_storage,
            trips_data: HashMap::new(),
            verbose: false,
            coordination_cells: HashMap::new(),
            conflict_zones: HashMap::new(),
            cells_conflicts_zones: HashMap::new(),
            current_position: HashMap::new(),
            updated_at: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as i64,
            expire_at: 0,
            steps: 0,
            world_srid: picked_srid,
        }
    }

    /// Gets the unique session identifier
    pub fn get_id(&self) -> Uuid {
        self.id
    }

    /// Gets the current step count
    pub fn get_steps(&self) -> i32 {
        self.steps
    }

    /// Gets the last vehicle ID used
    pub fn get_last_vehicle_id(&self) -> VehicleID {
        self.last_vehicle_id
    }

    /// Gets the world SRID
    pub fn get_world_srid(&self) -> SRID {
        self.world_srid
    }

    /// Gets the verbose level
    pub fn get_verbose_level(&self) -> bool {
        self.verbose
    }

    /// Sets verbose level for the session
    pub fn set_verbose_level(&mut self, verbose: bool) {
        self.verbose = verbose;
    }
}
