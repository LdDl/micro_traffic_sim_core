use crate::agents::{AgentType, BehaviourType};
use crate::grid::cell::CellID;
use std::fmt;

// Represents trip type generator
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TripType {
    // Uknown
    Undefined,
    // For generating vehicles at a given frequency on a regular basis
    Constant,
    // For generating vehicles at random moments of time
    Random,
}

impl fmt::Display for TripType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let trip_type_str = match self {
            TripType::Undefined => "undefined",
            TripType::Constant => "constant",
            TripType::Random => "random",
        };
        write!(f, "{}", trip_type_str)
    }
}

pub type TripID = i64; // Alias for TripID

/// Represents a trip with vehicles generator
#[derive(Debug, Clone)]
pub struct Trip {
    // Cells which must be traversed in exact given order by the agent
    pub transit_cells: Vec<CellID>,
    // Initial speed which generated vehicle would have after appearing on the road network
    pub initial_speed: i32,
    // Probability with which vehicles are generated if trip type is TripType::Random
    pub probability: f64,
    // Source vertex of microscopic graph (`birth` zone)
    pub from_node: CellID,
    // Target vertex of microscopic graph (`death` zone)
    pub to_node: CellID,
    // Allowed agent type for the generator
    pub allowed_agent_type: AgentType,
    // Allowed behaviour type for the generator
    pub allowed_behaviour_type: BehaviourType,
    // Type trip for generating different vehicles. See TripType description
    pub trip_type: TripType,
    // Frequency (seconds) with which vehicles are generated if trip type is TripType::Constant
    pub time: i32,
    // Start time (second) for the trip. Until that no vehicles will be generated. Default: 0
    pub start_time: i32,
    // End time (second) for the trip. After that no vehicles will be generated. Default:: i32::MAX
    pub end_time: i32,
    // Relaxation time in the transit cells
    pub relax_time: i32,
    // Size of tail of the vehicle. By default every vehicle in cellular automata has size 1 itself and tail 0. When it
    // is needed to make larger vehicles (e.g. limousine) it could be handy to know extra size for
    // modeling heterogeneous road traffic flow instead of homogeneous one.
    pub vehicle_tail_size: i32,
}

/// A builder pattern implementation for constructing `Trip` objects.
///
/// `TripBuilder` allows for optional configuration of `Trip` fields before building the final `Trip` object.
pub struct TripBuilder {
    trip: Trip,
}

impl Trip {
    /// Constructs a new `TripBuilder` for building a `Trip` object.
    ///
    /// # Arguments
    ///
    /// * `from_node` - The starting point of the trip.
    /// * `to_node` - The ending point of the trip.
    /// * `trip_type` - The type of the trip (e.g., static or random generator).
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::agents::AgentType;
    /// use micro_traffic_sim_core::agents::BehaviourType;
    /// use micro_traffic_sim_core::trips::trip::{Trip, TripType};
    /// let trip = Trip::new(1, 10, TripType::Constant)
    ///     .with_allowed_agent_type(AgentType::Car)
    ///     .with_allowed_behaviour_type(BehaviourType::Cooperative)
    ///     .with_time(10)
    ///     .build();
    /// println!("{:?}", trip);
    /// ```
    pub fn new(from_node: CellID, to_node: CellID, trip_type: TripType) -> TripBuilder {
        TripBuilder {
            trip: Trip {
                transit_cells: Vec::new(),
                initial_speed: 0,
                probability: 0.5,
                from_node,
                to_node,
                allowed_agent_type: AgentType::Undefined,
                allowed_behaviour_type: BehaviourType::Undefined,
                trip_type,
                time: -1,
                start_time: 0,
                end_time: i32::MAX,
                relax_time: -1,
                vehicle_tail_size: 0,
            },
        }
    }
}

impl TripBuilder {
    /// Sets the cells IDs list in which each cell must be traversed in exact given order by the agent and relaxation time for the trip.
    ///
    /// # Arguments
    ///
    /// * `cells` - A list of IDs of cells
    /// * `t` - The time (in time units) should be spent at each transit stop.
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::trips::trip::{Trip, TripType};
    /// let trip = Trip::new(1, 10, TripType::Constant)
    ///     .with_transits_cells(vec![2, 3, 4], 10)
    ///     .build();
    /// println!("{:?}", trip);
    /// ```
    pub fn with_transits_cells(mut self, cells: Vec<CellID>, t: i32) -> Self {
        self.trip.transit_cells = cells;
        self.trip.relax_time = t;
        self
    }

    /// Sets the initial speed of the trip.
    ///
    /// # Arguments
    ///
    /// * `speed` - The initial speed for the trip.
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::trips::trip::{Trip, TripType};
    /// let trip = Trip::new(1, 10, TripType::Constant)
    ///     .with_initial_speed(60)
    ///     .build();
    /// println!("{:?}", trip);
    /// ```
    ///
    pub fn with_initial_speed(mut self, speed: i32) -> Self {
        self.trip.initial_speed = speed;
        self
    }

    /// Sets the allowed agent type for the trip.
    ///
    /// # Arguments
    ///
    /// * `agent_type` - The agent type (e.g., car, bus) allowed for this trip.
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::agents::AgentType;
    /// use micro_traffic_sim_core::trips::trip::{Trip, TripType};
    /// let trip = Trip::new(1, 10, TripType::Constant)
    ///     .with_allowed_agent_type(AgentType::Car)
    ///     .build();
    /// println!("{:?}", trip);
    /// ```
    ///
    pub fn with_allowed_agent_type(mut self, agent_type: AgentType) -> Self {
        self.trip.allowed_agent_type = agent_type;
        self
    }

    /// Sets the allowed behavior type for the trip.
    ///
    /// # Arguments
    ///
    /// * `behaviour_type` - The behavior type (e.g., aggressive, cautious).
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::agents::BehaviourType;
    /// use micro_traffic_sim_core::trips::trip::{Trip, TripType};
    /// let trip = Trip::new(1, 10, TripType::Constant)
    ///     .with_allowed_behaviour_type(BehaviourType::Aggressive)
    ///     .build();
    /// println!("{:?}", trip);
    /// ```
    ///
    pub fn with_allowed_behaviour_type(mut self, behaviour_type: BehaviourType) -> Self {
        self.trip.allowed_behaviour_type = behaviour_type;
        self
    }

    /// Sets the frequency for generating vehicle
    ///
    /// # Arguments
    ///
    /// * `time` - Frequency (seconds) with which vehicles are generated if trip type is TripType::Constant
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::trips::trip::{Trip, TripType};
    /// let trip = Trip::new(1, 10, TripType::Constant) // Trip type must be TripType::Constant
    ///     .with_time(10) // Vehicle will be generated every 10 seconds
    ///     .build();
    /// println!("{:?}", trip);
    /// ```
    pub fn with_time(mut self, time: i32) -> Self {
        self.trip.time = time;
        self
    }

    /// Sets the probablity for generating vehicle
    ///
    /// # Arguments
    ///
    /// * `probablity` - Probability with which vehicles are generated if trip type is TripType::Random
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::trips::trip::{Trip, TripType};
    /// let trip = Trip::new(1, 10, TripType::Random) // Trip type must be TripType::Random
    ///     .with_probability(0.7) // Vehicle will be generated with 70% probablity
    ///     .build();
    /// println!("{:?}", trip);
    /// ```
    pub fn with_probability(mut self, probability: f64) -> Self {
        self.trip.probability = probability;
        self
    }

    /// Sets the start time (second) for the trip. Until that time no vehicles will be generated.
    ///
    /// # Arguments
    ///
    /// * `start_time` - start second. Until that time no vehicles will be generated.
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::trips::trip::{Trip, TripType};
    /// let trip = Trip::new(1, 10, TripType::Constant)
    ///     .with_start_time(5) // Until then vehicles won't be generated
    ///     .with_time(2) // Vehicles will be generated on 7th, 9th and so so seconds.
    ///     .build();
    /// println!("{:?}", trip);
    /// ```
    pub fn with_start_time(mut self, start_time: i32) -> Self {
        self.trip.start_time = start_time;
        self
    }

    /// Sets the end time (second) for the trip. After that time no vehicles will be generated.
    ///
    /// # Arguments
    ///
    /// * `end_time` - end second. After that time no vehicles will be generated.
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::trips::trip::{Trip, TripType};
    /// let trip = Trip::new(1, 10, TripType::Random) // Trip type must be TripType::Random
    ///     .with_start_time(5) // Until then vehicles won't be generated
    ///     .with_time(2) // Vehicles will be generated on 7th, 9th and 11th seconds.
    ///     .with_end_time(12) // Vehicles will stop to be generated after this second ticks
    ///     .build();
    /// println!("{:?}", trip);
    /// ```
    pub fn with_end_time(mut self, end_time: i32) -> Self {
        self.trip.end_time = end_time;
        self
    }

    /// Sets the tail size of the vehicle.
    ///
    /// # Arguments
    ///
    /// * `size` - The tail size of the vehicle (default: 0).
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::trips::trip::{Trip, TripType};
    /// let trip = Trip::new(1, 10, TripType::Constant)
    ///     .with_vehicle_tail_size(3)
    ///     .build();
    /// println!("{:?}", trip);
    /// ```
    pub fn with_vehicle_tail_size(mut self, len: i32) -> Self {
        self.trip.vehicle_tail_size = len;
        self
    }

    /// Builds the final `Trip` object with the configured properties.
    ///
    /// # Returns
    /// The fully constructed `Trip` object.
    pub fn build(self) -> Trip {
        self.trip
    }
}
