use crate::agents::{AgentType, BehaviourType};
use crate::grid::cell::CellID;
use crate::grid::lane_change_type::LaneChangeType;
use crate::grid::road_network::GridRoads;
use crate::trips::trip::TripID;
use std::fmt;
use std::rc::Rc;
use std::cell::RefCell;

#[derive(Debug, Clone)]
pub enum VehicleError {
    TailCellNotFound {
        cell_id: CellID,
        position: usize,
        vehicle_id: VehicleID,
    },
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

pub type VehicleRef = Rc<RefCell<Vehicle>>;

pub type VehicleID = u64; // Alias for VehicleID

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
    /// E.g. grid [1->2->3->4->5], vehicle's head in cell 4, size is 2. Then the tail cells are [2, 3]
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

    /// A boolean indicating if the vehicle is a confclict participant
    pub is_conflict_participant: bool,
    /// Corresponding trip identifier
    pub trip: TripID,
    /// Number of transits have been made by the vehicle
    transits_made: u64,
    /// Cells which must be traversed in exact given order by the vehicle
    pub transit_cells: Vec<CellID>,
    /// A value in (0; 1] representing the probability of the vehicle randomly slowing down.
    pub slow_down_factor: f64,
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

    /// Vehicle's intention to perform maneuver and other actions
    pub intention: VehicleIntention,
}

/// Represents vehicle's tail intention
#[derive(Debug, Clone, PartialEq)]
pub struct TailIntentionManeuver {
    /// This field is for establishing source cell of maneuver for the vehicle's tail (in case when vehicle has size more that one cell)
    pub source_cell_maneuver: CellID,
    /// This field is for establishing target cell of maneuver for the vehicle's tail (in case when vehicle has size more that one cell)
    pub target_cell_maneuver: CellID,
    /// Flag to repesent vehicle's tail intentions to perform maneuver (in case when vehicle has size more that one cell). See the ref. at `LaneChangeType`
    pub intention_maneuver: LaneChangeType,
}

impl Default for TailIntentionManeuver {
    fn default() -> TailIntentionManeuver {
        TailIntentionManeuver {
            source_cell_maneuver: -1,
            target_cell_maneuver: -1,
            intention_maneuver: LaneChangeType::Undefined,
        }
    }
}

/// Represents vehicle's intention to perform maneuver and other actions
#[derive(Debug, Default, Clone, PartialEq)]
pub struct VehicleIntention {
    /// Flag to repesent vehicle intentions to perform maneuver. See the ref. at `LaneChangeType`
    /// If the vehicle makes a maneuver, then it has no explicit advantage in possible conflicts
    pub intention_maneuver: LaneChangeType,
    /// Possible speed
    pub intention_speed: i32,
    /// Final cell for the vehicle's trip. It it only used when vehicle
    /// has no other choice other than to change destination.
    pub destination: Option<CellID>,
    /// @todo: for further research and development needs
    pub confusion: Option<bool>,
    // pub cells: Vec<(CellID, IntentionType)>,
    /// Cell which vehicle wants to occupy
    pub intention_cell_id: CellID, // IntentionType::Target
    /// Cells which vehicle's tail wants to occupy (in case when vehicle has size more that one cell)
    pub tail_intention_cells: Vec<CellID>,
    /// Intention occupied cells in case when vehicle moving with speed more than one cell per time unit.
    pub intermediate_cells: Vec<CellID>, // IntentionType::Transit
    /// Tail's intention. See the ref. at `IntentionTailManeuever`
    pub tail_maneuver: TailIntentionManeuver,
    /// Flag to stop vehicle
    pub should_stop: bool,
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
    /// use micro_traffic_sim_core::agents::{AgentType, Vehicle};
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
                is_conflict_participant: false,
                trip: -1,
                transits_made: 0,
                transit_cells: Vec::new(),
                slow_down_factor: 0.1,
                cooperativity: 0.0,
                timer_non_acceleration: 0,
                timer_non_maneuvers: 0,
                timer_non_slowdown: 0,
                relax_time: 0,
                relax_countdown: 0,
                travel_time: 0,
                confusion: false,
                intention: VehicleIntention::default(),
            },
        }
    }

    /// Increments number of transit have been made by vehicle
    ///
    /// # Returns
    /// Nothing
    ///
    /// # Example
    /// ```
    /// use micro_traffic_sim_core::agents::{AgentType, Vehicle};
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
    /// use micro_traffic_sim_core::agents::{AgentType, Vehicle};
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
    /// use micro_traffic_sim_core::agents::{AgentType, Vehicle};
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
    /// use micro_traffic_sim_core::agents::{AgentType, Vehicle};
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
    /// use micro_traffic_sim_core::agents::{AgentType, Vehicle};
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
    /// use micro_traffic_sim_core::agents::{AgentType, Vehicle, VehicleIntention};
    /// use micro_traffic_sim_core::grid::lane_change_type::LaneChangeType;
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
    /// # Returns
    /// Updated lane change type and potential error
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
    /// use micro_traffic_sim_core::agents::{AgentType, Vehicle};
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
    /// use micro_traffic_sim_core::agents::{BehaviourType, Vehicle};
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

    /// Sets currently occupied cells by tail (in case when vehicle has size more that one cell)
    ///
    /// # Arguments
    /// * `size` - The size of the vehicle's occupied cells list.
    /// * `cells` - A list of cells
    ///
    /// # Returns
    /// A `VehicleBuilder` instance for further method chaining.
    ///
    /// # Example
    /// ```rust
    /// use micro_traffic_sim_core::agents::Vehicle;
    /// let vehicle = Vehicle::new(1)
    ///     .with_tail_size(3, vec![10, 11, 12])
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
    /// use micro_traffic_sim_core::grid::lane_change_type::LaneChangeType;
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
        self.vehicle.slow_down_factor = p;
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
