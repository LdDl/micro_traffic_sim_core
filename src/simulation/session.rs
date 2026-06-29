use crate::behaviour::BehaviourParameters;
use crate::agents_types::AgentType;
use crate::agents::{VehicleID, Vehicle, VehiclesStorage};
use crate::conflict_zones::{ConflictZone, ConflictZoneID};
use crate::grid::cell::{CellID, Cell};
use crate::trips::trip::{Trip, TripID, TripType};
use crate::simulation::grids_storage::{GridsStorage, GridsStorageError};
use crate::geom::{Point, SRID};
use crate::intentions::{IntentionError, prepare_intentions};
use crate::conflicts::{ConflictError, ConflictSolverError, collect_conflicts, solve_conflicts};
use crate::movement::{MovementError, movement};
use crate::shortest_path::router::shortest_path;
use crate::simulation::states::{AutomataState, VehicleState};
use crate::traffic_lights::lights::{TrafficLightID, TrafficLight};
use crate::verbose::*;
use std::collections::{HashMap, BTreeMap};
use uuid::Uuid;
use std::fmt;
use std::time::{SystemTime, UNIX_EPOCH};
use rand::Rng;
use rand::SeedableRng;
use rand::rngs::StdRng;

/// Fixed seed for the per-session spawn RNG so vehicle generation is reproducible
/// run-to-run (replaces the previous unseeded thread-local RNG).
const SPAWN_SEED: u64 = 0x00C0_FFEE;

/// Custom error types for `Session`.
#[derive(Debug, Clone)]
pub enum SessionError {
    /// Indicates that some error occurred
    ErrorPlaceholder(String),
    /// Grid storage related error
    GridsStorageError(GridsStorageError),
    /// Intention processing error
    IntentionError(IntentionError),
    /// Conflict error
    ConflictError(ConflictError),
    /// Conflict solver error
    ConflictSolverError(ConflictSolverError),
    /// Cell not found error
    CellNotFound(CellID),
    /// Movement error
    MovementError(MovementError),
}

impl fmt::Display for SessionError {
    /// Formats the error message for `SessionError`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SessionError::ErrorPlaceholder(value) => {
                write!(f, "ErrorPlaceholder: {}", value)
            },
            SessionError::GridsStorageError(err) => {
                write!(f, "GridsStorage error: {}", err)
            },
            SessionError::IntentionError(err) => {
                write!(f, "Intention error: {}", err)
            },
            SessionError::ConflictError(err) => {
                write!(f, "Conflict error: {}", err)
            },
            SessionError::ConflictSolverError(err) => {
                write!(f, "Conflict solver error: {}", err)
            },
            SessionError::CellNotFound(cell_id) => {
                write!(f, "Cell with ID {} not found", cell_id)
            },
            SessionError::MovementError(err) => {
                write!(f, "Movement error: {}", err)
            },
        }
    }
}

impl std::error::Error for SessionError {}

impl From<GridsStorageError> for SessionError {
    fn from(err: GridsStorageError) -> Self {
        SessionError::GridsStorageError(err)
    }
}

impl From<IntentionError> for SessionError {
    fn from(err: IntentionError) -> Self {
        SessionError::IntentionError(err)
    }
}

impl From<ConflictError> for SessionError {
    fn from(err: ConflictError) -> Self {
        SessionError::ConflictError(err)
    }
}

impl From<ConflictSolverError> for SessionError {
    fn from(err: ConflictSolverError) -> Self {
        SessionError::ConflictSolverError(err)
    }
}

impl From<MovementError> for SessionError {
    fn from(err: MovementError) -> Self {
        SessionError::MovementError(err)
    }
}

/// Session - representation of session for Cellular Automata with Traffic lights control management
pub struct Session {
    /// Current position mapping from cell ID to vehicle ID
    current_position: HashMap<CellID, VehicleID>,

    /// Cellular automata grid storage
    grids_storage: GridsStorage,

    /// Trips for automatic vehicle generation
    trips_data: HashMap<TripID, Trip>,

    /// Vehicles storage
    vehicles: VehiclesStorage,

    /// Cells under traffic lights control
    /// It could be just Cell, but we'll use CellID for now
    _coordination_cells: HashMap<CellID, CellID>,

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
    verbose: LocalLogger,

    /// Time when this session has been created or updated (nanoseconds)
    _updated_at: i64,

    /// Time when to destroy this session (nanoseconds)
    _expire_at: i64,

    /// Defines the SRID of the world
    world_srid: SRID,

    /// Cumulative count of vehicles that reached their destination
    vehicles_completed: i32,

    /// Cumulative count of vehicles that were lost (reached death zone without reaching destination)
    vehicles_lost: i32,

    /// Routing configuration (SUMO-style). See [`RoutingOptions`].
    routing: RoutingOptions,

    /// Smoothed per-cell travel speed (cells/tick), updated every
    /// `routing.adaptation_interval` ticks from current occupancy. Empty until the
    /// first congestion update; only used when `adaptation_interval > 0`.
    cell_speed: HashMap<CellID, f64>,

    /// Seeded RNG for vehicle generation: spawn-probability rolls and the fair selection
    /// among trips that share a source cell. Seeded so spawning is reproducible.
    spawn_rng: StdRng,
}

/// SUMO-style routing configuration. Mirrors `device.rerouting.*` options.
#[derive(Debug, Clone)]
pub struct RoutingOptions {
    /// How often (in ticks) a vehicle re-plans its cached route. `0` (the SUMO
    /// default) disables periodic rerouting - the spawn route is kept and only
    /// refreshed when the vehicle falls off it. `>0` re-runs a full A* every
    /// `reroute_period` ticks (staggered by spawn time), the hook congestion-aware
    /// routing (smoothed weights) will use.
    pub reroute_period: i32,
    /// Maximum BFS depth for `reconnect_to_cache` when a vehicle falls off its
    /// route, before giving up and doing a full A*.
    pub reconnect_max_depth: usize,
    /// How often (in ticks) per-cell congestion (smoothed speed) is recomputed and
    /// pushed into the routing edge costs. `-1` (default) disables congestion-aware
    /// routing entirely - edge costs stay at free-flow time (current behaviour).
    /// Mirrors SUMO `device.rerouting.adaptation-interval`.
    pub adaptation_interval: i32,
    /// Exponential-moving-average weight for smoothing per-cell speeds:
    /// `smoothed = old * weight + current * (1 - weight)`. `0.0` = use the latest
    /// observation; closer to `1.0` = slower to react (more damping against
    /// reroute oscillation). Mirrors SUMO `device.rerouting.adaptation-weight`.
    pub adaptation_weight: f64,
    /// Number of forward cells to average a cell's current speed over, smoothing the
    /// per-cell signal spatially (a stop-line cell is diluted by the free-flowing
    /// cells just ahead, like SUMO's whole-edge mean speed). `0` = pure per-cell.
    pub congestion_window: usize,
}

impl Default for RoutingOptions {
    fn default() -> Self {
        RoutingOptions {
            reroute_period: 0,
            reconnect_max_depth: 10,
            adaptation_interval: -1,
            adaptation_weight: 0.5,
            congestion_window: 2,
        }
    }
}

impl Session {
    /// Creates new session with default values for provided cell grid
    pub fn default(srid: Option<SRID>) -> Self {
        let picked_srid = srid.unwrap_or(SRID::Euclidean);
        let session_id = Uuid::new_v4();
        let verbose = LocalLogger::with_session(VerboseLevel::None, session_id.to_string());
        Session {
            id: session_id,
            last_vehicle_id: 1,
            vehicles: VehiclesStorage::new(),
            grids_storage: GridsStorage::new().build(),
            trips_data: HashMap::new(),
            verbose,
            _coordination_cells: HashMap::new(),
            conflict_zones: HashMap::new(),
            cells_conflicts_zones: HashMap::new(),
            current_position: HashMap::new(),
            _updated_at: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as i64,
            _expire_at: 0,
            steps: 0,
            world_srid: picked_srid,
            vehicles_completed: 0,
            vehicles_lost: 0,
            routing: RoutingOptions::default(),
            cell_speed: HashMap::new(),
            spawn_rng: StdRng::seed_from_u64(SPAWN_SEED),
        }
    }

    /// Creates new session for cellular automata for provided cell grid
    pub fn new(grids_storage: GridsStorage, srid: Option<SRID>) -> Self {
        let picked_srid = srid.unwrap_or(SRID::Euclidean);
        let session_id = Uuid::new_v4();
        let verbose = LocalLogger::with_session(VerboseLevel::None, session_id.to_string());

        Session {
            id: session_id,
            last_vehicle_id: 1,
            vehicles: VehiclesStorage::new(),
            grids_storage,
            trips_data: HashMap::new(),
            verbose,
            _coordination_cells: HashMap::new(),
            conflict_zones: HashMap::new(),
            cells_conflicts_zones: HashMap::new(),
            current_position: HashMap::new(),
            _updated_at: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as i64,
            _expire_at: 0,
            steps: 0,
            world_srid: picked_srid,
            vehicles_completed: 0,
            vehicles_lost: 0,
            routing: RoutingOptions::default(),
            cell_speed: HashMap::new(),
            spawn_rng: StdRng::seed_from_u64(SPAWN_SEED),
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
    pub fn get_verbose_level(&self) -> VerboseLevel {
        self.verbose.level()
    }

    /// Sets verbose level for the session
    pub fn set_verbose_level(&mut self, verbose: VerboseLevel) {
        self.verbose.set_level(verbose);
    }

    /// Sets the SUMO-style routing options (rerouting period, reconnect depth).
    pub fn set_routing_options(&mut self, options: RoutingOptions) {
        self.routing = options;
    }

    /// Returns the current routing options.
    pub fn get_routing_options(&self) -> &RoutingOptions {
        &self.routing
    }

    /// Returns a reference to the cell with the given ID if it exists in the vehicles grid.
    pub fn get_cell(&self, cell_id: &CellID) -> Option<&Cell> {
        self.grids_storage.get_cell(cell_id)
    }

    /// Returns a reference to the traffic light with the given ID if it exists in the traffic lights storage.
    pub fn get_tls_ref(&self, ) -> &HashMap<TrafficLightID, TrafficLight> {
        self.grids_storage.get_tls_ref()
    }

    /// Adds given trip to the session. It also checks if trip's end time is valid and returns '0' if it is not.
    /// Uses the trip's ID field for storage key.
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

        // Use trip's ID for storage
        let trip_id = trip.id;

        // Add trip to storage
        self.trips_data.insert(trip_id, trip);

        trip_id
    }

    /// Adds given vehicles to the session vehicles storage
    pub fn add_vehicles(&mut self, vehicles: Vec<Vehicle>) {
        for vehicle in vehicles {
            let vehicle_id = vehicle.id;
            self.vehicles.insert(vehicle_id, vehicle);
            if vehicle_id >= self.last_vehicle_id {
                self.last_vehicle_id = vehicle_id + 1;
            }
        }
    }

    /// Returns a reference to the vehicles storage
    pub fn get_vehicles(&self) -> &VehiclesStorage { &self.vehicles }

    /// Adds cells to the grids. It is shortcut to GridsStorage's add_cells method
    pub fn add_cells(&mut self, cells_data: Vec<crate::grid::cell::Cell>) {
        self.grids_storage.add_cells(cells_data);
    }

    /// Resets current/done vehicles, steps number, last vehicle ID, traffic lights states, trips
    pub fn reset(&mut self) {
        self.verbose.log_with_fields(
            EVENT_SIMULATION_RESET,
            "Reset simulation",
            &[
                ("step", &self.steps),
                ("vehicles_num", &self.vehicles.len()),
                ("trips_num", &self.trips_data.len()),
                ("tls_num", &self.grids_storage.tls_num()),
            ]
        );

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
    /// Builds the vehicle for a trip. No eligibility / probability roll happens here -
    /// the caller (`generate_vehicles`) decides whether and which trip spawns this tick.
    fn build_vehicle(&self, trip: &Trip, trip_id: TripID) -> Vehicle {
        // Determine target node
        let target_node = if trip.allowed_agent_type == AgentType::Bus
            && !trip.transit_cells.is_empty() {
            trip.transit_cells[0] // First transit cell for buses
        } else {
            trip.to_node
        };

        // Create behaviour parameters based on allowed behaviour type
        let behaviour_params = BehaviourParameters::from_behaviour_type(trip.allowed_behaviour_type);

        // Determine speed limit: use trip's explicit value if set, otherwise from behaviour
        let speed_limit = if trip.speed_limit >= 0 {
            trip.speed_limit
        } else {
            behaviour_params.speed_limit()
        };

        // Create vehicle using builder pattern
        Vehicle::new(self.last_vehicle_id)
            .with_type(trip.allowed_agent_type)
            .with_behaviour(trip.allowed_behaviour_type)
            .with_cell(trip.from_node)
            .with_speed(trip.initial_speed)
            .with_speed_limit(speed_limit)
            .with_slowdown(behaviour_params.slowdown_factor())
            .with_min_safe_distance(behaviour_params.min_safe_distance())
            .with_aggressive_level(behaviour_params.aggressive_level())
            .with_destination(target_node)
            .with_trip(trip_id)
            .with_tail_size(trip.vehicle_tail_size, vec![]) // Empty tail cells initially
            .with_transit_cells(trip.transit_cells.clone())
            .with_relax_time(trip.relax_time)
            .build()
    }

    /// Generates vehicles based on the trips data
    pub fn generate_vehicles(&mut self) {
        if self.verbose.is_at_least(VerboseLevel::Main) {
            self.verbose.log_with_fields(
                EVENT_GEN_VEHICLES,
                "Generate vehicles",
                &[
                    ("step", &self.steps),
                    ("vehicles_num", &self.vehicles.len()),
                    ("trips_num", &self.trips_data.len()),
                ]
            );
        }
        // Group trips by their source cell, in deterministic (sorted) order. A source cell
        // holds at most one vehicle, so at most ONE trip may spawn there per tick. Each trip
        // rolls its own probability; among the trips that want to spawn we pick ONE weighted
        // by probability, so a busy route is not starved by a rarer route that merely has a
        // smaller trip id (the old HashMap-order, first-wins loop ignored the probabilities of
        // every trip after the first on a shared source).
        let mut by_source: BTreeMap<CellID, Vec<TripID>> = BTreeMap::new();
        for (id, trip) in &self.trips_data {
            by_source.entry(trip.from_node).or_default().push(*id);
        }
        let by_source: Vec<(CellID, Vec<TripID>)> = by_source
            .into_iter()
            .map(|(src, mut ids)| {
                ids.sort_unstable();
                (src, ids)
            })
            .collect();

        for (from_node, trip_ids) in by_source {
            // Source occupied (by any vehicle's head or tail from a previous tick)? skip.
            // current_position is the start-of-tick occupancy (rebuilt after this phase last
            // tick), so this is an O(1) check that also replaces the old O(trips*vehicles) scan.
            if self.current_position.contains_key(&from_node) {
                continue;
            }

            // Collect the trips that want to spawn this tick, with their selection weight.
            let mut candidates: Vec<(TripID, f64)> = Vec::new();
            let mut total_weight = 0.0f64;
            for &tid in &trip_ids {
                // Read the Copy fields and drop the trips_data borrow before touching spawn_rng.
                let (in_bounds, ttype, ttime, prob) = match self.trips_data.get(&tid) {
                    Some(t) => (
                        self.steps >= t.start_time && self.steps <= t.end_time,
                        t.trip_type,
                        t.time,
                        t.probability,
                    ),
                    None => continue,
                };
                if !in_bounds {
                    continue;
                }
                let wants = match ttype {
                    TripType::Constant => ttime > 0 && self.steps % ttime == 0,
                    TripType::Random => self.spawn_rng.random::<f64>() < prob,
                    TripType::Undefined => false,
                };
                if wants {
                    // Random trips compete by probability; Constant trips that fire share evenly.
                    let weight = match ttype {
                        TripType::Random => prob.max(f64::MIN_POSITIVE),
                        _ => 1.0,
                    };
                    candidates.push((tid, weight));
                    total_weight += weight;
                }
            }
            if candidates.is_empty() {
                continue;
            }

            // Fair weighted pick of a single trip. Candidates are in sorted trip-id order, so
            // with the seeded RNG the choice is deterministic.
            let chosen_id = if candidates.len() == 1 {
                candidates[0].0
            } else {
                let threshold = self.spawn_rng.random::<f64>() * total_weight;
                let mut acc = 0.0;
                let mut pick = candidates[candidates.len() - 1].0;
                for &(tid, w) in &candidates {
                    acc += w;
                    if threshold < acc {
                        pick = tid;
                        break;
                    }
                }
                pick
            };

            // Build the chosen vehicle (the trips_data borrow ends with the match).
            let mut generated_vehicle = match self.trips_data.get(&chosen_id) {
                Some(trip) => self.build_vehicle(trip, chosen_id),
                None => continue,
            };

            // Build the cached route once at spawn (full A* to the destination).
            // Per-tick the vehicle follows this route instead of re-running A*.
            if generated_vehicle.destination >= 0 {
                let net = self.grids_storage.get_vehicles_net_ref();
                if let (Some(s), Some(g)) = (
                    net.get_cell(&generated_vehicle.cell_id),
                    net.get_cell(&generated_vehicle.destination),
                ) {
                    if let Ok(path) = shortest_path(s, g, net, true, None) {
                        generated_vehicle.cached_route =
                            path.vertices().iter().map(|c| c.get_id()).collect();
                        generated_vehicle.route_idx = 0;
                        generated_vehicle.last_reroute = self.steps;
                    }
                }
            }

            if self.verbose.is_at_least(VerboseLevel::Additional) {
                self.verbose.log_with_fields(
                    EVENT_GEN_VEHICLES,
                    "Generate vehicle for trip",
                    &[
                        ("step", &self.steps),
                        ("trip_id", &chosen_id),
                        ("from_node", &from_node),
                        ("vehicle_id", &generated_vehicle.id),
                    ]
                );
            }

            let vehicle_id = generated_vehicle.id;
            self.vehicles.insert(vehicle_id, generated_vehicle);
            self.last_vehicle_id = vehicle_id + 1; // Increment for next vehicle
        }
    }

    /// Updates current position mapping
    fn update_current_positions(&mut self) {
        if self.verbose.is_at_least(VerboseLevel::Main) {
            self.verbose.log_with_fields(
                EVENT_UPD_POS,
                "Update positions",
                &[
                    ("step", &self.steps),
                    ("vehicles_num", &self.vehicles.len()),
                    ("trips_num", &self.trips_data.len()),
                ]
            );
        }
        self.current_position.clear();
        for vehicle in self.vehicles.values() {
            if self.verbose.is_at_least(VerboseLevel::Detailed) {
                self.verbose.log_with_fields(
                    EVENT_UPD_POS,
                    "Vehicle position",
                    &[
                        ("vehicle_id", &vehicle.id),
                        ("cell_id", &vehicle.cell_id),
                    ]
                );
                for (id, &tail_cell) in vehicle.tail_cells.iter().enumerate() {
                    if self.verbose.is_at_least(VerboseLevel::All) {
                        self.verbose.log_with_fields(
                            EVENT_UPD_POS,
                            "Vehicle tail position",
                            &[
                                ("vehicle_id", &vehicle.id),
                                ("tail_idx", &id),
                                ("tail_cell_id", &tail_cell),
                            ]
                        );
                    }
                }
            }
            self.current_position.insert(vehicle.cell_id, vehicle.id);
            for &tail_cell in &vehicle.tail_cells {
                // A freshly spawned tailed vehicle carries placeholder tail cells (0) until
                // its tail materializes as it moves; skip non-positive ids so cell 0 is not
                // marked as a phantom occupant in the occupancy map.
                if tail_cell > 0 {
                    self.current_position.insert(tail_cell, vehicle.id);
                }
            }
        }
    }

    /// Recomputes smoothed per-cell travel speed from current occupancy and pushes
    /// the result into the routing edge costs (`GridRoads::apply_congestion`).
    ///
    /// Per-cell congestion (no dependency on client-supplied link ids), measured -
    /// not modelled - like SUMO: a cell's current speed is the speed of the vehicle
    /// on it, averaged spatially over the next `congestion_window` forward cells so a
    /// lone stop-line cell does not dominate (the SUMO whole-edge averaging,
    /// reconstructed from our own forward topology). Empty cells decay to free-flow,
    /// so a cleared jam recovers. The value is then blended into the previous one via
    /// the EMA weight. Global, O(vehicles + cells * window).
    fn update_cell_speeds(&mut self) {
        let alpha = self.routing.adaptation_weight;
        let window = self.routing.congestion_window;
        // Current occupancy speed per cell (only cells with a vehicle).
        let mut occ: HashMap<CellID, f64> = HashMap::with_capacity(self.vehicles.len());
        for (_, v) in self.vehicles.iter() {
            occ.insert(v.cell_id, v.speed as f64);
        }
        // Current sample per cell = forward-window mean of occupied speeds, else free-flow.
        let mut cur_map: HashMap<CellID, f64> = HashMap::new();
        {
            let net = self.grids_storage.get_vehicles_net_ref();
            for (id, cell) in net.iter() {
                let free_flow = (cell.get_speed_limit() as f64).max(1.0);
                let mut sum = 0.0;
                let mut cnt = 0u32;
                let mut c = *id;
                for _ in 0..=window {
                    if let Some(&sp) = occ.get(&c) {
                        sum += sp;
                        cnt += 1;
                    }
                    match net.get_cell(&c).map(|cc| cc.get_forward_id()) {
                        Some(f) if f >= 0 => c = f,
                        _ => break,
                    }
                }
                let cur = if cnt > 0 {
                    (sum / cnt as f64).clamp(0.1, free_flow)
                } else {
                    free_flow
                };
                cur_map.insert(*id, cur);
            }
        }
        // EMA-update the stored smoothed speeds.
        for (id, cur) in cur_map {
            let new = match self.cell_speed.get(&id) {
                Some(&old) => old * alpha + cur * (1.0 - alpha),
                None => cur,
            };
            self.cell_speed.insert(id, new);
        }
        // Push smoothed speeds into the grid's edge costs for routing.
        let cell_speed = self.cell_speed.clone();
        self.grids_storage
            .get_vehicles_net_mut()
            .apply_congestion(&cell_speed);
    }
    
    /// Main simulation step function
    /// 
    /// Pipeline is:
    /// ```text
    /// 1. Generate vehicles (trips)
    /// 2. Update positions
    /// 3. Traffic light updates
    /// 4. Prepare intentions      ← intentions module
    /// 5. Collect conflicts       ← conflicts module
    /// 6. Solve conflicts         ← conflicts module
    /// 7. Execute movement        ← movement module
    /// 8. Collect state dump
    /// ```
    pub fn step(&mut self) -> Result<AutomataState, SessionError> {
        if self.verbose.is_at_least(VerboseLevel::Main) {
            self.verbose.log_with_fields(
                EVENT_STEP,
                "Run Step",
                &[
                    ("step", &self.steps),
                    ("vehicles_num", &self.vehicles.len()),
                    ("trips_num", &self.trips_data.len()),
                    ("tls_num", &self.grids_storage.tls_num()),
                ]
            );
        }
        
        // 1. Generate vehicles for given trips
        self.generate_vehicles();
        
        // 2. Update current positions
        self.update_current_positions();

        // 2b. Congestion update: every adaptation_interval ticks, recompute smoothed
        // per-cell speeds and push them into the routing edge costs (no-op when
        // adaptation_interval <= 0, i.e. free-flow routing).
        if self.routing.adaptation_interval > 0
            && self.steps % self.routing.adaptation_interval == 0
        {
            self.update_cell_speeds();
        }

        // 3. Update and collect TLS state
        let tl_states_dump = self.grids_storage.tick_traffic_lights(&self.verbose)?;

        // 4. Create intentions for all vehicles
    let collected_intentions = prepare_intentions(self.grids_storage.get_vehicles_net_ref(), &self.current_position, &mut self.vehicles, &self.verbose, self.steps, self.routing.reroute_period, self.routing.reconnect_max_depth)?;

        // 5. Collect conflicts
        let conflicts_data = collect_conflicts(
            &collected_intentions,
            self.grids_storage.get_vehicles_net_ref(),
            &self.conflict_zones,
            &self.cells_conflicts_zones,
            &self.verbose,
            &mut self.vehicles,
        )?;

        // 6. Solve conflicts
    solve_conflicts(conflicts_data, &mut self.vehicles, &self.verbose)?;

        // 7. Move vehicles
        let vehicles_grid = self.grids_storage.get_vehicles_net_ref();
        let (completed, lost) = movement(vehicles_grid, &mut self.vehicles, &self.verbose)?;
        self.vehicles_completed += completed;
        self.vehicles_lost += lost;

        // 8. Collect current vehicles positions for state dump
        let mut states_dump: Vec<VehicleState> = Vec::with_capacity(self.vehicles.len());
        for vehicle in self.vehicles.values() {
            let mut occupied_points: Vec<[f64; 2]> = Vec::with_capacity(vehicle.tail_cells.len());
            for &tail_cell_id in &vehicle.tail_cells {
                if tail_cell_id > 0 {
                    if let Some(opt_cell) = vehicles_grid.get_cell(&tail_cell_id) {
                        let opt_pt = opt_cell.get_point();
                        occupied_points.push([opt_pt.x(), opt_pt.y()]);
                    }
                }
            }
            states_dump.push(VehicleState {
                occupied_points,
                last_cell: vehicle.cell_id,
                tail_cells: vehicle.tail_cells.clone(),
                last_intermediate_cells: vehicle.intention.intermediate_cells.clone(),
                last_speed: vehicle.speed,
                last_angle: vehicle.bearing,
                vehicle_type: vehicle.vehicle_type,
                travel_time: vehicle.travel_time,
                id: vehicle.id,
                trip_id: vehicle.trip,
            });
        }

        // 9. Increment step counter
        let timestamp = self.steps;
        self.steps += 1;

        // Log vehicle completion statistics
        if self.verbose.is_at_least(VerboseLevel::Main) {
            self.verbose.log_with_fields(
                EVENT_STEP_COMPLETE,
                "Step completed with vehicle statistics",
                &[
                    ("timestamp", &timestamp),
                    ("active_vehicles", &self.vehicles.len()),
                    ("vehicles_completed", &self.vehicles_completed),
                    ("vehicles_lost", &self.vehicles_lost),
                ]
            );
        }

        Ok(AutomataState {
            timestamp: timestamp,
            vehicles: states_dump,
            tls: tl_states_dump,
            vehicles_completed: self.vehicles_completed,
            vehicles_lost: self.vehicles_lost,
        })
    }

    /// Gets the expiration time of the session
    pub fn get_expire_at(&self) -> i64 {
        self._expire_at
    }

    /// Sets the expiration time of the session
    pub fn set_expire_at(&mut self, expire_at: i64) {
        self._expire_at = expire_at;
    }

    /// Gets the last updated time of the session
    pub fn get_updated_at(&self) -> i64 {
        self._updated_at
    }

    /// Sets the last updated time of the session
    pub fn set_updated_at(&mut self, updated_at: i64) {
        self._updated_at = updated_at;
    }
}
