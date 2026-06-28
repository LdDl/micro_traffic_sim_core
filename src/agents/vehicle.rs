use crate::behaviour::BehaviourType;
use crate::agents_types::AgentType;
use crate::agents::{VehicleIntention, TailIntentionManeuver};
use crate::grid::cell::CellID;
use crate::maneuver::LaneChangeType;
use crate::grid::road_network::GridRoads;
use crate::trips::trip::TripID;
use std::fmt;
use std::rc::Rc;
use std::cell::RefCell;

/// Errors returned by vehicle-related operations.
#[derive(Debug, Clone)]
pub enum VehicleError {
    /// Indicates that a tail cell with the given ID was not found for the vehicle
    TailCellNotFound {
        /// The ID of the tail cell that was not found
        cell_id: CellID,
        /// The position (index) of the tail cell in the vehicle's tail cells list
        position: usize,
        /// The vehicle's identifier
        vehicle_id: VehicleID,
    },
    /// Indicates that the provided cell ID is invalid
    InvalidCell(CellID),
}

impl fmt::Display for VehicleError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            VehicleError::TailCellNotFound {
                cell_id,
                position,
                vehicle_id,
            } => {
                write!(
                    f,
                    "Can't find tail cell with ID '{}' at position '{}' for vehicle '{}'",
                    cell_id, position, vehicle_id
                )
            }
            VehicleError::InvalidCell(cell_id) => {
                write!(f, "Invalid cell ID '{}'", cell_id)
            }
        }
    }
}

/// Just a shorthand for vehicle reference type
/// Obsolete in terms of internal usage (it was used before passing VehiclesStorage
/// to the simulation engine). Could be useful for external usage.
pub type VehicleRef = Rc<RefCell<Vehicle>>;

/// Vehicle unique identifier type
pub type VehicleID = u64;

/// Patience threshold (in steps) for the most aggressive vehicle (cooperativity 0):
/// how long it loses contested cells before overriding right-of-way. SUMO's
/// `--time-to-teleport` default is 300 s; we reuse it as the impatient end.
pub const PATIENCE_MIN: i32 = 300;
/// Patience threshold (in steps) for the fully cooperative vehicle (cooperativity 1):
/// the most patient driver waits longer before forcing, but still finite (no permanent
/// stall). Vehicles in between scale linearly with cooperativity.
pub const PATIENCE_MAX: i32 = 700;

/// Represents basic agent in simulation
#[derive(Debug)]
pub struct Vehicle {
    /// Unique identifier
    pub id: VehicleID,
    /// Agent type. See the ref. at `AgentType`
    pub vehicle_type: AgentType,
    /// Movement strategy. See the ref. at `BehaviourType`
    pub strategy_type: BehaviourType,

    /// Currently occupied cell
    pub cell_id: CellID,

    /// Currently occupied cells by tail (in case when vehicle has size more that one cell)
    /// Order: [furthest from head, ..., closest to head]
    /// E.g. grid [1->2->3->4->5], vehicle's head in cell 4, size is 2. Then the tail cells are [2, 3]
    /// where index 0 (cell 2) is furthest and index 1 (cell 3) is closest to head
    ///
    /// NOTE: Gradual Materialization - When a vehicle is spawned from a trip generator, this vector
    /// starts empty even if tail_size > 0. The tail "materializes" gradually as the vehicle moves:
    /// each cell the head passes through becomes a tail cell. This models a vehicle "entering" the
    /// network head-first. During this growth period, len(tail_cells) < tail_size is expected.
    /// See also: Trip::vehicle_tail_size.
    pub tail_cells: Vec<CellID>,

    /// Current speed
    pub speed: i32,
    /// Maximum speed which can be reached by the vehicle. If value is greater than maximum speed
    /// in cell then the vehicle will be limited to cell's speed limit
    pub speed_limit: i32,
    /// Current bearing (direction angle)
    pub bearing: f64,
    /// Minimal safe distance (in cells) to the vehicle in front
    pub min_safe_distance: i32,
    /// Final cell for the vehicle's trip
    pub destination: CellID,
    /// Final trip destination (immutable, used for completion check; destination may change for transit cells)
    pub trip_destination: CellID,

    /// A boolean indicating if the vehicle is a confclict participant
    pub is_conflict_participant: bool,
    /// Consecutive steps the vehicle has failed to move. Drives the patience-based
    /// right-of-way override: once `wait_ticks` reaches the vehicle's cooperativity-scaled
    /// patience threshold, the vehicle wins contested cells in conflict resolution
    /// ("desperate" - see `Vehicle::is_desperate`). Reset to 0 on any actual move.
    pub wait_ticks: i32,
    /// Corresponding trip identifier
    pub trip: TripID,
    /// Number of transits have been made by the vehicle
    transits_made: u64,
    /// Cells which must be traversed in exact given order by the vehicle
    pub transit_cells: Vec<CellID>,
    /// NaSch random-slowdown (dawdle) probability `p`: a MOVING vehicle (speed > 0) decelerates by one
    /// with this probability each step (`v -> max(v-1, 0)`). This is the canonical
    /// Nagel-Schreckenberg randomization parameter. In `[0, 1]` (0 = deterministic behaviour).
    /// The VDR stopped-vehicle counterpart is `p0` (field `slow_to_start_factor_p0`).
    pub slow_down_factor_p: f64,
    /// VDR (Velocity-Dependent Randomization) parameter `p0`, (I use "slow-to-start" probability elsewhere):
    /// the dawdle probability used while the vehicle is STOPPED (speed == 0),
    /// as opposed to `p` (the moving probability, field `slow_down_factor_p`).
    /// VDR makes the random-slowdown probability depend on speed (here two levels: `p0` for stopped, `p` for moving).
    /// When `p0 > p` a stopped vehicle is more reluctant to start than a moving one dawdles,
    /// so a jam discharges slower than it fills: this yields the realistic capacity drop
    /// and the metastable / hysteretic flow near the critical density (the VDR signature).
    /// `p0 == p` disables the asymmetry and the model reduces to plain NaSch.
    /// In `[0, 1]`.
    pub slow_to_start_factor_p0: f64,
    /// Lane-change probability: once the incentive + safety conditions hold, the
    /// change is taken only with this probability - the canonical stochastic anti-weaving gate.
    /// `1.0` (default) = always change when conditions hold (= current behaviour, no refusal).
    /// Trucks use a smaller value (P2 <= P1) [@todo: not implemented and tested yet].
    /// In (0, 1].
    pub change_p1: f64,
    /// Lane-change hysteresis: steps remaining during which a REACTIVE (blocked-escape) lane
    /// change is suppressed after the vehicle just changed lanes - so a car cannot immediately
    /// weave back (A->B->A) or hop across lanes step-by-step. Does NOT block route-following
    /// maneuvers or the forward roll. 0 = free to react.
    pub timer_reactive_lane_change: i64,
    /// Lane-change cooldown DURATION (steps): the value `timer_reactive_lane_change` is armed to
    /// right after a REACTIVE lane change - how many steps another reactive (blocked-escape) change
    /// stays suppressed (anti-weaving: no A->B->A weave-back / lane-hopping). Route-following
    /// maneuvers are NOT affected. Per-vehicle / per-driver-type: `0` = changes freely (aggressive),
    /// larger = more hesitant (cooperative / unsure).
    /// Could be set from `BehaviourParameters`.
    pub lc_cooldown: i64,
    /// A value in (0; 1] representing cooperative behaviour of the vehicle.
    /// 0 - when behaviour considered to be "aggressive"
    /// 1 - fully cooperative
    pub cooperativity: f64,
    /// Delay time (in time units) between two possible accelerations vehicle can do.
    /// It could be used to prohibit to the vehicle to do sequential acceleration during simulation.
    pub timer_non_acceleration: i64,
    /// Delay time (in time units) between two possible maneuvers vehicle can perform.
    /// It could be used to prohibit to the vehicle to perform sequential maneuvers during simulation.
    pub timer_non_maneuvers: i64,
    /// Delay time (in time units) between two possible slowdowns vehicle can do.
    /// It could be used to prohibit to the vehicle to do sequential maneuvers during simulation.
    pub timer_non_slowdown: i64,
    /// Relaxation time (in time units) in the transit cells (in case when they are set)
    pub relax_time: i32,
    // Timer for `relax_time` field
    relax_countdown: i32,

    /// Travel time (in time units) which vehicle has been in movement state.
    pub travel_time: i64,

    /// @todo: for further research and development needs
    pub confusion: bool,

    /// Cached route to the destination as an ordered list of cell IDs
    /// (origin first, destination last), built by a full A* at spawn / on reroute.
    /// Per-step the vehicle follows this list cheaply instead of re-running A*.
    /// Empty means "no cached route" (fall back to per-step routing).
    pub cached_route: Vec<CellID>,
    /// Index into `cached_route` of the vehicle's current head cell.
    pub route_idx: usize,
    /// Simulation step at which the route was last (re)computed; used to schedule
    /// the next staggered reroute.
    pub last_reroute: i32,

    /// Vehicle's intention to perform maneuver and other actions
    pub intention: VehicleIntention,
}

impl Vehicle {
    /// Constructs a new `VehicleBuilder` for building a `Vehicle` object.
    ///
    /// # Arguments
    /// * `id` - A unique identifier for the vehicle.
    ///
    /// # Returns
    /// A `VehicleBuilder` struct which is used to configure and build the `Vehicle` object.
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::agents_types::AgentType;
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_cell(1)
    ///     .with_destination(100)
    ///     .with_type(AgentType::Car)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn new(id: u64) -> VehicleBuilder {
        VehicleBuilder {
            vehicle: Vehicle {
                id,
                vehicle_type: AgentType::Car,
                strategy_type: BehaviourType::Aggressive,
                cell_id: -1,
                tail_cells: Vec::new(),
                speed: 1,
                speed_limit: 4,
                bearing: 0.0,
                min_safe_distance: 0,
                destination: -1,
                trip_destination: -1,
                is_conflict_participant: false,
                wait_ticks: 0,
                trip: -1,
                transits_made: 0,
                transit_cells: Vec::new(),
                slow_down_factor_p: 0.1,
                slow_to_start_factor_p0: 0.1,
                change_p1: 1.0,
                timer_reactive_lane_change: 0,
                lc_cooldown: 0,
                cooperativity: 0.0,
                timer_non_acceleration: 0,
                timer_non_maneuvers: 0,
                timer_non_slowdown: 0,
                relax_time: 0,
                relax_countdown: 0,
                travel_time: 0,
                confusion: false,
                cached_route: Vec::new(),
                route_idx: 0,
                last_reroute: -1,
                intention: VehicleIntention::default(),
            },
        }
    }

    /// Advances the cached-route cursor (`route_idx`) so that
    /// `cached_route[route_idx] == cell_id`, searching forward from the current
    /// cursor (the vehicle only ever moves forward along its route). No-op if the
    /// route is empty. Returns `true` if the cursor now points at the current cell,
    /// `false` if the current cell was not found ahead on the route (the vehicle
    /// fell off its cached route - the caller falls back to full routing / reconnect).
    pub fn advance_route_cursor(&mut self) -> bool {
        if self.cached_route.is_empty() {
            return false;
        }
        let mut i = self.route_idx.min(self.cached_route.len() - 1);
        while i < self.cached_route.len() {
            if self.cached_route[i] == self.cell_id {
                self.route_idx = i;
                return true;
            }
            i += 1;
        }
        false
    }

    /// Clears the cached route (e.g. when the destination changes), forcing the
    /// next step to fall back to full routing.
    pub fn clear_cached_route(&mut self) {
        self.cached_route.clear();
        self.route_idx = 0;
    }

    /// Increments number of transit have been made by vehicle
    ///
    /// # Returns
    /// Nothing
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::agents_types::AgentType;
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let mut vehicle = Vehicle::new(1)
    ///     .with_cell(1)
    ///     .with_destination(100)
    ///     .with_type(AgentType::Car)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// vehicle.transits_made_inc();
    /// ```
    pub fn transits_made_inc(&mut self) {
        self.transits_made += 1;
    }
    /// Returns number of transits have been made by vehicle
    ///
    /// # Returns
    /// Number of transits have been made by vehicle
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::agents_types::AgentType;
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let mut vehicle = Vehicle::new(1)
    ///     .with_cell(1)
    ///     .with_destination(100)
    ///     .with_type(AgentType::Car)
    ///     .build();
    /// vehicle.transits_made_inc();
    /// vehicle.transits_made_inc();
    /// println!("Vehicle: {:?}", vehicle);
    /// println!("Transits have been made: {:?}", vehicle.get_transits_made()); // Should output 2
    /// ```
    pub fn get_transits_made(&self) -> u64 {
        self.transits_made
    }
    /// Decrements decrements countdown timer for relaxation in occupied transit cell
    ///
    /// # Returns
    /// Nothing
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::agents_types::AgentType;
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let mut vehicle = Vehicle::new(1)
    ///     .with_cell(3)
    ///     .with_destination(100)
    ///     .with_type(AgentType::Car)
    ///     .with_transit_cells(vec![3, 8])
    ///     .with_relax_time(9)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// vehicle.relax_countdown_dec();
    /// ```
    pub fn relax_countdown_dec(&mut self) {
        self.relax_countdown -= 1;
    }

    /// Resets resets countdown timer for relaxation (to a value of `RelaxTime“ field)  in occupied transit cell
    ///
    /// # Returns
    /// Nothing
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::agents_types::AgentType;
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let mut vehicle = Vehicle::new(1)
    ///     .with_cell(3)
    ///     .with_destination(100)
    ///     .with_type(AgentType::Car)
    ///     .with_transit_cells(vec![3, 8])
    ///     .with_relax_time(9)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// vehicle.relax_countdown_reset();
    /// ```
    pub fn relax_countdown_reset(&mut self) {
        self.relax_countdown = self.relax_time
    }

    /// Returns number of time units which is remaining for the vehicle to be in occupied transit cell
    ///
    /// # Returns
    /// Number of time units which is remaining for the vehicle to be in occupied transit cell
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::agents_types::AgentType;
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let mut vehicle = Vehicle::new(1)
    ///     .with_cell(3)
    ///     .with_destination(100)
    ///     .with_type(AgentType::Car)
    ///     .with_transit_cells(vec![3, 8])
    ///     .with_relax_time(9)
    ///     .build();
    /// vehicle.relax_countdown_dec();
    /// vehicle.relax_countdown_dec();
    /// println!("Vehicle: {:?}", vehicle);
    /// println!("Remaining time units to be in transit cell: {:?}", vehicle.get_relax_countdown()); // Should output 7
    /// vehicle.relax_countdown_reset();
    /// println!("Remaining time units to be in transit cell: {:?}", vehicle.get_relax_countdown()); // Should output 9
    /// ```
    pub fn get_relax_countdown(&self) -> i32 {
        self.relax_countdown
    }

    /// Sets the vehicle's intention
    ///
    /// # Arguments
    /// * `intention` - The vehicle's intention. See the ref. at `VehicleIntention`
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::maneuver::LaneChangeType;
    /// use micro_traffic_sim_core::agents_types::AgentType;
    /// use micro_traffic_sim_core::agents::{Vehicle, VehicleIntention};
    /// let mut vehicle = Vehicle::new(1)
    ///   .with_cell(3)
    ///   .with_destination(100)
    ///   .with_type(AgentType::Car)
    ///   .build();
    /// let mut intention = VehicleIntention::default();
    /// intention.intention_maneuver = LaneChangeType::ChangeRight;
    /// intention.intention_cell_id = 10;
    /// vehicle.set_intention(intention);
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn set_intention(&mut self, intention: VehicleIntention) {
        self.intention = intention;
    }
    /// Blocks the vehicle in its current cell with a specified speed.
    pub fn block_with_speed(&mut self, speed: i32) {
        self.intention.intention_cell_id = self.cell_id;
        self.intention.intention_speed = speed;
        self.intention.intention_maneuver = LaneChangeType::Block;
        self.intention.intermediate_cells = vec![];
    }
    /// Updates the vehicle's tail maneuver by scanning occupied cells and determining if vehicle is in lane changing process
    ///
    /// # Arguments
    /// * `net` - The road network grid
    ///
    /// Returns [`VehicleError::TailCellNotFound`] if a tail cell ID doesn't exist in the grid,
    /// or [`VehicleError::InvalidCell`] if a cell ID is invalid.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// use micro_traffic_sim_core::grid::road_network::GridRoads;
    ///
    /// let net = GridRoads::new();
    /// let mut vehicle = Vehicle::new(1)
    ///     .with_cell(10)
    ///     .with_tail_size(2, vec![8, 9])
    ///     .build();
    /// let tail_maneuver = vehicle.scan_tail_maneuver(&net);
    /// println!("Tail maneuver: {:?}", tail_maneuver);
    /// ```
    pub fn scan_tail_maneuver(
        &self,
        net: &GridRoads,
    ) -> Result<TailIntentionManeuver, VehicleError> {
        if self.tail_cells.is_empty() {
            return Ok(TailIntentionManeuver::default());
        }

        // Scan sequential cells
        for (idx, window) in self.tail_cells.windows(2).enumerate() {
            let [from_id, to_id] = window else { continue };

            if *from_id < 1 {
                continue;
            }

            let from_cell = net
                .get_cell(from_id)
                .ok_or(VehicleError::TailCellNotFound {
                    cell_id: *from_id,
                    position: idx,
                    vehicle_id: self.id,
                })?;

            match (*to_id, from_cell) {
                (id, _) if id == from_cell.get_forward_id() => continue,
                (id, _) if id == from_cell.get_right_id() => {
                    return Ok(TailIntentionManeuver {
                        source_cell_maneuver: *from_id,
                        target_cell_maneuver: *to_id,
                        intention_maneuver: LaneChangeType::ChangeRight,
                    })
                }
                (id, _) if id == from_cell.get_left_id() => {
                    return Ok(TailIntentionManeuver {
                        source_cell_maneuver: *from_id,
                        target_cell_maneuver: *to_id,
                        intention_maneuver: LaneChangeType::ChangeLeft,
                    })
                }
                _ => continue,
            }
        }

        // Check final cell
        let last_id = self.tail_cells[self.tail_cells.len() - 1];
        if last_id > 0 {
            let last_cell = net
                .get_cell(&last_id)
                .ok_or(VehicleError::TailCellNotFound {
                    cell_id: last_id,
                    position: self.tail_cells.len() - 1,
                    vehicle_id: self.id,
                })?;

            if self.cell_id == last_cell.get_right_id() {
                return Ok(TailIntentionManeuver {
                    source_cell_maneuver: last_id,
                    target_cell_maneuver: self.cell_id,
                    intention_maneuver: LaneChangeType::ChangeRight,
                });
            }
            if self.cell_id == last_cell.get_left_id() {
                return Ok(TailIntentionManeuver {
                    source_cell_maneuver: last_id,
                    target_cell_maneuver: self.cell_id,
                    intention_maneuver: LaneChangeType::ChangeLeft,
                });
            }
        }
        Ok(TailIntentionManeuver {
            source_cell_maneuver: self.cell_id,
            target_cell_maneuver: if self.intention.intention_cell_id < 1 {
                self.cell_id
            } else {
                self.intention.intention_cell_id
            },
            intention_maneuver: self.intention.intention_maneuver,
        })
    }

    /// Applies vehicle's intention to the vehicle's current state 
    /// 
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// use micro_traffic_sim_core::maneuver::LaneChangeType;
    /// 
    /// let mut vehicle = Vehicle::new(1)
    ///     .with_cell(10)
    ///     .with_destination(100)
    ///     .with_speed(1)
    ///     .build();
    /// 
    /// // Set some intention
    /// vehicle.intention.intention_speed = 3;
    /// vehicle.intention.destination = Some(200);
    /// vehicle.intention.confusion = Some(true);
    /// 
    /// // Apply the intention
    /// vehicle.apply_intention();
    /// 
    /// // Now vehicle state is updated
    /// assert_eq!(vehicle.speed, 3);
    /// assert_eq!(vehicle.destination, 200);
    /// assert_eq!(vehicle.confusion, true);
    /// ```
    pub fn apply_intention(&mut self) {
        // No need to update cell_id. It is done in movement.rs
        self.speed = self.intention.intention_speed;
        // Apply destination if set in intention
        if let Some(destination) = self.intention.destination {
            self.destination = destination;
        }
        // Apply confusion state if set in intention
        if let Some(confusion) = self.intention.confusion {
            self.confusion = confusion;
        }
        // No need to update tails cells. It is done in movement.rs
    }

    /// Rewrites this vehicle's intention to a single forward step into `target` (an
    /// adjacent cell). Used by the patience override to clamp a desperate winner to one
    /// cell. Crucially it also rebuilds `tail_intention_cells` consistently with the
    /// one-cell step - the tail shifts forward and the vacated current cell becomes the
    /// new nearest tail cell (same convention as `extract_tail_intention`'s 1-cell case).
    /// Without this, a stale multi-cell `tail_intention_cells` from the original (longer)
    /// intention would be applied in movement and land the body ahead of / on top of the
    /// clamped head.
    pub fn set_single_step_intention(&mut self, target: CellID) {
        self.intention.intention_cell_id = target;
        self.intention.intermediate_cells.clear();
        self.intention.intention_speed = 1;
        if self.tail_cells.is_empty() {
            self.intention.tail_intention_cells = Vec::new();
        } else {
            // Drop the furthest tail cell, shift the rest forward, append the cell the
            // head is vacating (current cell_id) as the new nearest-to-head tail cell.
            let mut tail = self.tail_cells[1..].to_vec();
            tail.push(self.cell_id);
            self.intention.tail_intention_cells = tail;
        }
    }

    /// The vehicle's patience threshold in steps: how long it tolerates losing
    /// contested cells before overriding right-of-way. Scaled by cooperativity so
    /// every vehicle eventually forces (the threshold is always finite), but more
    /// cooperative drivers wait longer: aggressive (cooperativity 0) -> `PATIENCE_MIN`,
    /// fully cooperative (cooperativity 1) -> `PATIENCE_MAX`.
    pub fn patience(&self) -> i32 {
        let coop = self.cooperativity.clamp(0.0, 1.0);
        PATIENCE_MIN + (coop * (PATIENCE_MAX - PATIENCE_MIN) as f64).round() as i32
    }

    /// True when the vehicle has been stuck for at least its patience threshold and is
    /// therefore allowed to win a contested (free) cell in conflict resolution. It only
    /// ever takes effect inside the conflict solver, which exists solely for free cells,
    /// so a vehicle blocked by an occupied cell (a queue follower) is never desperate in
    /// any useful sense; and it never overrides a `Tail` (physical body) conflict.
    pub fn is_desperate(&self) -> bool {
        self.wait_ticks >= self.patience()
    }
}

/// A builder pattern implementation for constructing `Vehicle` objects.
///
/// `VehicleBuilder` allows for optional configuration of `Vehicle` fields before building the final `Vehicle` object.
pub struct VehicleBuilder {
    vehicle: Vehicle,
}

impl VehicleBuilder {
    /// Sets the agent type of the vehicle.
    ///
    /// # Arguments
    /// * `typ` - The type of the agent. See the ref. at `AgentType`
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents_types::AgentType;
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_type(AgentType::Bus)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_type(mut self, typ: AgentType) -> Self {
        self.vehicle.vehicle_type = typ;
        self
    }

    /// Sets the movement strategy for the vehicle.
    ///
    /// # Arguments
    /// * `typ` - The behavior type. See the ref. at `BehaviourType`
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::behaviour::BehaviourType;
    /// use micro_traffic_sim_core::agents::{Vehicle};
    /// let vehicle = Vehicle::new(1)
    ///     .with_behaviour(BehaviourType::Cooperative)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_behaviour(mut self, typ: BehaviourType) -> Self {
        self.vehicle.strategy_type = typ;
        self
    }

    /// Sets the current cell ID where the vehicle is located.
    ///
    /// # Arguments
    /// * `cell` - The ID of the cell where the vehicle is currently positioned.
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_cell(10)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_cell(mut self, cell_id: CellID) -> Self {
        self.vehicle.cell_id = cell_id;
        self
    }

    /// Sets currently occupied cells by tail (in case when vehicle has size more that one cell).
    /// The occupied cells must be provided in order: [furthest from head, ..., closest to head].
    ///
    /// # Arguments
    /// * `size` - The size of the vehicle's tail (number of cells behind head).
    /// * `cells` - A list of cell IDs currently occupied by the tail.
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// // For head at cell 13 with tail size 2, cell 11 is furthest, cell 12 is closest
    /// let vehicle = Vehicle::new(1)
    ///     .with_cell(13)
    ///     .with_tail_size(2, vec![11, 12])
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_tail_size(mut self, size: usize, cells: Vec<CellID>) -> Self {
        self.vehicle.tail_cells = vec![0; size];
        if !cells.is_empty() {
            self.vehicle.tail_cells.copy_from_slice(&cells);
        }
        self
    }

    /// Sets the current speed of the vehicle.
    ///
    /// # Arguments
    /// * `speed` - The speed (in cells per time unit)
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_speed(5)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_speed(mut self, speed: i32) -> Self {
        self.vehicle.speed = speed;
        self
    }

    /// Sets the speed limit for the vehicle, which is the maximum speed it can travel at.
    ///
    /// # Arguments
    /// * `speed_limit` - The speed limit (in cells per time unit)
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_speed_limit(10)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_speed_limit(mut self, speed_limit: i32) -> Self {
        self.vehicle.speed_limit = speed_limit;
        self
    }

    /// Sets the bearing (direction angle) for the vehicle.
    ///
    /// # Arguments
    /// * `bearing` - The bearing in degrees (clockwise angle from the north).
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_bearing(45.0)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_bearing(mut self, bearing: f64) -> Self {
        self.vehicle.bearing = bearing;
        self
    }

    /// Sets the minimum safe distance (in cells) to the vehicle in front
    ///
    /// # Arguments
    /// * `min_safe_distance` - The minimum safe distance (in cells)
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_min_safe_distance(3)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_min_safe_distance(mut self, min_safe_distance: i32) -> Self {
        self.vehicle.min_safe_distance = min_safe_distance;
        self
    }

    /// Sets final cell for the vehicle's trip
    ///
    /// # Arguments
    /// * `cell_id` - Cell's identifier
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_destination(20)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_destination(mut self, cell_id: CellID) -> Self {
        self.vehicle.destination = cell_id;
        self.vehicle.trip_destination = cell_id;
        self
    }

    /// Overrides the trip-completion destination independently of the immediate `destination`.
    ///
    /// `with_destination` sets both the immediate `destination` and the `trip_destination`
    /// (where the trip is considered complete) to the same cell. For a multi-stop bus these
    /// differ: the immediate destination starts at the FIRST transit stop (and is advanced
    /// stop-by-stop during movement), while the trip only completes at the LAST stop. Call
    /// this AFTER `with_destination` to set the completion cell separately.
    ///
    /// # Arguments
    /// * `cell_id` - Cell at which the trip is considered complete
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    pub fn with_trip_destination(mut self, cell_id: CellID) -> Self {
        self.vehicle.trip_destination = cell_id;
        self
    }

    /// Establishes source and target cells of maneuver for the vehicle's tail and vehicle's tail intention maneuver (in case when vehicle has size more that one cell)
    ///
    /// # Arguments
    /// * `source_cell` - The ID of the source cell
    /// * `target_cell` - The ID of the target cell
    /// * `maneuver` - The type of lane change the vehicle's tail is performing. This can be `Left`, `Right`, or `None`. Should not be 'Undefined' (except initialization). See the ref. at `LaneChangeType`
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// use micro_traffic_sim_core::maneuver::LaneChangeType;
    /// let vehicle = Vehicle::new(1)
    ///     .with_tail_intention_maneuver(10, 20, LaneChangeType::ChangeRight)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_tail_intention_maneuver(
        mut self,
        source_cell: CellID,
        target_cell: CellID,
        maneuver: LaneChangeType,
    ) -> Self {
        self.vehicle.intention.tail_maneuver.source_cell_maneuver = source_cell;
        self.vehicle.intention.tail_maneuver.target_cell_maneuver = target_cell;
        self.vehicle.intention.tail_maneuver.intention_maneuver = maneuver;
        self
    }

    /// Sets whether the vehicle is a conflict participant.
    ///
    /// # Arguments
    /// * `is_in_conflict` - A boolean indicating if the vehicle is in a conflict state.
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_conflict_participation(true)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_conflict_participation(mut self, is_in_conflict: bool) -> Self {
        self.vehicle.is_conflict_participant = is_in_conflict;
        self
    }

    /// Sets corresponding trip identifier
    ///
    /// # Arguments
    /// * `trip` - The ID of the trip
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_trip(12345)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_trip(mut self, trip_id: TripID) -> Self {
        self.vehicle.trip = trip_id;
        self
    }

    /// Sets the cells IDs list in which each cell must be traversed in exact given order by the vehicle
    ///
    /// # Arguments
    /// * `cells` - A list of IDs of cells
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_transit_cells(vec![3, 50, 77])
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_transit_cells(mut self, cells: Vec<CellID>) -> Self {
        self.vehicle.transit_cells = cells;
        self
    }

    /// Sets the probability of the vehicle slowing down randomly.
    ///
    /// # Arguments
    /// * `p` - A value in (0; 1]. 0 - never slowdowns, 1 - slowdowns every time unit
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_slowdown(0.5)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_slowdown(mut self, p: f64) -> Self {
        self.vehicle.slow_down_factor_p = p;
        self
    }

    /// Sets the VDR slow-to-start probability `p0` used while the vehicle is stopped.
    /// Should be `>= p` (the moving slowdown) for a capacity drop.
    /// `p0 == p` disables the VDR asymmetry.
    pub fn with_slow_to_start(mut self, p0: f64) -> Self {
        self.vehicle.slow_to_start_factor_p0 = p0;
        self
    }

    /// Sets the lane-change probability `p1` stochastic anti-weaving gate).
    /// `1.0` (default) = always change when incentive + safety hold.
    /// Trucks: P2 <= P1 [@todo: implement trucks' P2]
    pub fn with_change_p1(mut self, p: f64) -> Self {
        self.vehicle.change_p1 = p;
        self
    }

    /// Sets the reactive-lane-change cooldown DURATION in steps (see `Vehicle::lc_cooldown`).
    /// `0` (default) = no cooldown (changes freely);
    /// larger = more hesitant after a change.
    pub fn with_lc_cooldown(mut self, steps: i64) -> Self {
        self.vehicle.lc_cooldown = steps;
        self
    }

    /// Sets the level of cooperative behavior for the vehicle.
    ///
    /// # Arguments
    /// * `level` - A value in (0; 1]
    ///   `0` being non-cooperative (fully aggressive) and `1` being fully cooperative
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_cooperative_level(0.4)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    ///
    pub fn with_cooperative_level(mut self, level: f64) -> Self {
        self.vehicle.cooperativity = level;
        self
    }

    /// Sets the level of aggressive behavior (which is (1.0 - cooperativity) basically) for the vehicle
    ///
    /// # Arguments
    /// * `level` - A value in (0; 1]
    ///   `1` being fully agressive and `1` being non-aggressive (fully cooperative).
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_aggressive_level(0.8) // So cooperativity becomes 0.2
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_aggressive_level(mut self, level: f64) -> Self {
        self.vehicle.cooperativity = 1.0 - level;
        self
    }

    /// Sets the delay time between two possible accelerations vehicle can do. Use it when it is needed
    /// to prohibit to the vehicle to do sequential accelerations during simulation.
    ///
    /// # Arguments
    /// * `delay` - The delay time (in time units)
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_acceleration_delay(3)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_acceleration_delay(mut self, delay: i64) -> Self {
        self.vehicle.timer_non_acceleration = delay;
        self
    }

    /// Sets the delay time between two possible maneuvers vehicle can perform. Use it when it is needed
    /// to prohibit to the vehicle to perform sequential maneuvers during simulation.
    ///
    /// # Arguments
    /// * `delay` - The delay time (in time units)
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_maneuver_delay(3)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_maneuver_delay(mut self, delay: i64) -> Self {
        self.vehicle.timer_non_maneuvers = delay;
        self
    }

    /// Sets the delay time between two possible slowdowns vehicle can do. Use it when it is needed
    /// to prohibit to the vehicle to do sequential slowdowns during simulation.
    ///
    /// # Arguments
    /// * `delay` - The delay time (in time units)
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_slowdown_delay(3)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_slowdown_delay(mut self, delay: i64) -> Self {
        self.vehicle.timer_non_slowdown = delay;
        self
    }

    /// Sets the relaxation time (in time units) in transit cells (in case when they are set)
    ///
    /// # Arguments
    /// * `t` - The time (in time units) should be spent at each transit stop.
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_relax_time(5)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_relax_time(mut self, t: i32) -> Self {
        self.vehicle.relax_time = t;
        self.vehicle.relax_countdown = t;
        self
    }

    /// Sets the travel time (in time units) which vehicle has been in movement state.
    ///
    /// # Arguments
    /// * `t` - The travel time (in time units).
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_travel_time(30)
    ///     .build();
    /// println!("Vehicle: {:?}", vehicle);
    /// ```
    pub fn with_travel_time(mut self, t: i64) -> Self {
        self.vehicle.travel_time = t;
        self
    }

    /// Builds the final `Vehicle` object with the configured properties.
    ///
    /// # Returns
    /// The fully constructed `Vehicle` object.
    pub fn build(self) -> Vehicle {
        self.vehicle
    }

    /// Builds a reference to the `Vehicle` object.
    /// 
    /// Obsolete in terms of internal usage (it was used before passing VehiclesStorage
    /// to the simulation engine). Could be useful for external usage.
    ///     
    /// # Returns
    /// A reference to the `Vehicle` object.
    pub fn build_ref(self) -> VehicleRef {
        Rc::new(RefCell::new(self.vehicle))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_relaxation_countdown() {
        let relax_time = 9;
        let sim_steps = 3;
        let mut vehicle = Vehicle::new(1)
            .with_cell(3)
            .with_destination(100)
            .with_type(AgentType::Car)
            .with_transit_cells(vec![3, 8])
            .with_relax_time(relax_time)
            .build();
        for _ in 0..sim_steps {
            vehicle.relax_countdown_dec();
        }
        assert_eq!(
            relax_time - sim_steps,
            vehicle.get_relax_countdown(),
            "Incorrect countdown after specified number of steps"
        );
        vehicle.relax_countdown_reset();
        assert_eq!(
            relax_time,
            vehicle.get_relax_countdown(),
            "Incorrect countdown after reset"
        );
    }
    #[test]
    fn test_cooperative_level() {
        let coop_level = 0.75;
        let aggr_level = 1.0 - coop_level;
        let vehicle = Vehicle::new(1).with_aggressive_level(aggr_level).build();
        assert_eq!(
            coop_level, vehicle.cooperativity,
            "Incorrect cooperativity level"
        )
    }
}
