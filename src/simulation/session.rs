use crate::agents::{VehicleID, VehicleRef, Vehicle, BehaviourType};
use crate::conflict_zones::{ConflictZone, ConflictZoneID};
use crate::grid::cell::CellID;
use crate::traffic_lights::lights::{TrafficLightID};
use crate::trips::trip::{Trip, TripID, TripType};
use crate::simulation::grids_storage::GridsStorage;
use crate::geom::SRID;
use std::collections::HashMap;
use uuid::Uuid;
use std::fmt;
use std::time::{SystemTime, UNIX_EPOCH};
use rand::Rng;

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

    /// Adds given trip to the session. It also checks if trip's end time is valid and returns '0' if it is not.
    pub fn add_trip(&mut self, trip: Trip) -> TripID {
        let mut trip = trip;
        
        // Set default end time if not set
        if trip.end_time == 0 {
            trip.end_time = i32::MAX;
        }
        
        // Check if end time is valid
        if trip.end_time < trip.start_time {
            return 0;
        }
        
        // Generate new trip ID
        let new_trip_id = (self.trips_data.len() as i64) + 1;
        
        // Add trip to storage
        self.trips_data.insert(new_trip_id, trip);
        
        new_trip_id
    }

    /// Adds given vehicles to the session vehicles storage
    fn add_vehicles(&mut self, vehicles: Vec<VehicleRef>) {
        for vehicle in vehicles {
            let vehicle_id = vehicle.borrow().id;
            self.vehicles.insert(vehicle_id, vehicle);
            self.last_vehicle_id = vehicle_id;
        }
    }

    /// Adds cells to the grids. It is shortcut to GridsStorage's add_cells method
    pub fn add_cells(&mut self, cells_data: Vec<crate::grid::cell::Cell>) {
        self.grids_storage.add_cells(cells_data);
    }

    /// Resets current/done vehicles, steps number, last vehicle ID, traffic lights states, trips
    pub fn reset(&mut self) {
        if self.verbose {
            println!(
                "Reset simulation: step={}, vehicles_num={}, trips_num={}, tls_num={}",
                self.steps,
                self.vehicles.len(),
                self.trips_data.len(),
                self.grids_storage.tls_num()
            );
        }

        // Clear vehicles
        self.vehicles.clear();

        // Reset traffic lights
        self.grids_storage.tls_reset();

        // Clear trips
        self.trips_data.clear();

        // Reset counters
        self.steps = 0;
        self.last_vehicle_id = 1;
    }

    /// Adds traffic lights to the traffic lights storage.
    /// It is shortcut to GridsStorage's add_traffic_light method
    pub fn add_traffic_light(&mut self, tl: crate::traffic_lights::lights::TrafficLight) {
        self.grids_storage.add_traffic_light(tl);
    }

    /// Adds conflict zone to the session storage and maps cells to the conflict zone
    pub fn add_conflict_zone(&mut self, conflict_zone: ConflictZone) {
        let conflict_zone_id = conflict_zone.get_id();
        // Map cells to conflict zone
        let first_edge = conflict_zone.get_first_edge();
        let second_edge = conflict_zone.get_second_edge();
        if first_edge.target >= 0 {
            self.cells_conflicts_zones.insert(first_edge.target, conflict_zone_id);
        }
        if second_edge.target >= 0 {
            self.cells_conflicts_zones.insert(second_edge.target, conflict_zone_id);
        }
        // Add conflict zone to storage
        self.conflict_zones.insert(conflict_zone_id, conflict_zone);
    }

    /// Generates a single vehicle based on trip parameters
    fn generate_vehicle(&self, trip: &Trip, trip_id: TripID) -> Option<VehicleRef> {
        // Check if current time step is within trip time bounds
        if self.steps < trip.start_time || self.steps > trip.end_time {
            return None;
        }

        // Determine if vehicle should be generated based on trip type
        let should_generate = match trip.trip_type {
            TripType::Constant => {
                // Generate vehicle every 'time' seconds
                if trip.time <= 0 {
                    false
                } else {
                    self.steps % trip.time == 0
                }
            }
            TripType::Random => {
                // Generate vehicle based on probability
                let mut rng = rand::thread_rng();
                let norm_value: f64 = rng.gen();
                norm_value < trip.probability
            }
            _ => {
                if self.verbose {
                    println!(
                        "Trip type is not supported: trip_id={}, trip_type={:?}",
                        trip_id, trip.trip_type
                    );
                }
                false
            }
        };

        if !should_generate {
            return None;
        }

        // Determine target node
        let target_node = if trip.allowed_agent_type == crate::agents::AgentType::Bus 
            && !trip.transit_cells.is_empty() {
            trip.transit_cells[0] // First transit cell for buses
        } else {
            trip.to_node
        };

        // Create behaviour parameters based on allowed behaviour type
        let behaviour_params = crate::agents::BehaviourParameters::from_behaviour_type(trip.allowed_behaviour_type);

        // Create vehicle using builder pattern
        let vehicle = Vehicle::new(self.last_vehicle_id)
            .with_type(trip.allowed_agent_type)
            .with_behaviour(trip.allowed_behaviour_type)
            .with_cell(trip.from_node)
            .with_speed(trip.initial_speed)
            .with_speed_limit(behaviour_params.speed_limit())
            .with_slowdown(behaviour_params.slowdown_factor())
            .with_min_safe_distance(behaviour_params.min_safe_distance())
            .with_aggressive_level(behaviour_params.aggressive_level())
            .with_destination(target_node)
            .with_trip(trip_id)
            .with_tail_size(trip.vehicle_tail_size, vec![]) // Empty tail cells initially
            .with_transit_cells(trip.transit_cells.clone())
            .with_relax_time(trip.relax_time)
            .build_ref();

        Some(vehicle)
    }

    /// Generates vehicles based on the trips data
    pub fn generate_vehicles(&mut self) {
        if self.verbose {
            println!(
                "Generate vehicles: step={}, vehicles_num={}, trips_num={}",
                self.steps,
                self.vehicles.len(),
                self.trips_data.len()
            );
        }
        for (trip_id, trip) in &self.trips_data {
            // Check if there's already a vehicle at the source node
            let mut create = true;
            for vehicle in self.vehicles.values() {
                if vehicle.borrow().cell_id == trip.from_node {
                    create = false;
                    break;
                }
            }
            if !create {
                continue;
            }
            // Generate vehicle for this trip
            if let Some(generated_vehicle) = self.generate_vehicle(trip, *trip_id) {
                if self.verbose {
                    println!(
                        "Generate vehicle for trip: step={}, vehicles_num={}, trips_num={}, trip_id={}, vehicle_id={}",
                        self.steps,
                        self.vehicles.len(),
                        self.trips_data.len(),
                        trip_id,
                        generated_vehicle.borrow().id
                    );
                }

                let vehicle_id = generated_vehicle.borrow().id;
                self.vehicles.insert(vehicle_id, generated_vehicle);
                self.last_vehicle_id = vehicle_id + 1; // Increment for next vehicle
            }
        }
    }
}
