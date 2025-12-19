use crate::behaviour::{BehaviourType};
use crate::agents_types::AgentType;
use crate::grid::cell::CellID;
use std::fmt;

/// Vehicle generation patterns for trip scheduling.
/// 
/// Determines how frequently vehicles are spawned during simulation.
///
/// # Examples
/// 
/// ```rust
/// use micro_traffic_sim_core::trips::trip::{Trip, TripType};
/// 
/// // Predictable traffic flow
/// let steady_flow = Trip::new(1, 10, TripType::Constant).with_time(5);
/// 
/// // Variable traffic flow  
/// let variable_flow = Trip::new(1, 10, TripType::Random).with_probability(0.3);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TripType {
    /// Default uninitialized state
    Undefined,
    /// Generate vehicles at regular intervals (use `time` field)
    Constant,
    /// Generate vehicles probabilistically each time step (use `probability` field)
    Random,
}

impl fmt::Display for TripType {
    /// Formats the trip type for display.
    /// 
    /// Returns a short, lowercase string representation suitable for
    /// logging, debugging, and user interfaces.
    /// 
    /// # Examples
    /// 
    /// ```rust
    /// use micro_traffic_sim_core::trips::trip::{TripType};
    /// 
    /// assert_eq!(format!("{}", TripType::Undefined), "undefined");
    /// assert_eq!(format!("{}", TripType::Constant), "constant");
    /// assert_eq!(format!("{}", TripType::Random), "random");
    /// ```
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

/// Vehicle generator that spawns vehicles along specified routes.
///
/// A `Trip` defines a vehicle generation pattern including timing, routing,
/// and vehicle characteristics. It acts as a factory for creating vehicles
/// during simulation runtime.
///
/// # Generation Modes
///
/// - **Constant**: Regular interval spawning using `time` field  
/// - **Random**: Probabilistic spawning using `probability` field
/// - **Time-bounded**: Optional `start_time` and `end_time` limits
///
/// # Examples
///
/// ```rust
/// use micro_traffic_sim_core::trips::trip::{Trip, TripType};
/// use micro_traffic_sim_core::agents_types::AgentType;
/// 
/// // Rush hour traffic: frequent car generation
/// let rush_hour = Trip::new(1, 50, TripType::Constant)
///     .with_time(5)  // Every 5 time units
///     .with_allowed_agent_type(AgentType::Car)
///     .build();
/// ```
#[derive(Debug, Clone)]
pub struct Trip {
    // Numeric identifier of the trip
    pub id: TripID,
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
    pub vehicle_tail_size: usize,
    // Speed limit for generated vehicles. If >= 0, overrides the behaviour-derived speed limit.
    // Default: -1 (meaning "resolve from behaviour type")
    pub speed_limit: i32,
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
    /// use micro_traffic_sim_core::agents_types::AgentType;
    /// use micro_traffic_sim_core::behaviour::BehaviourType;
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
                id: 0,
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
                speed_limit: -1,
            },
        }
    }
}

impl TripBuilder {
    /// Sets the trip ID.
    ///
    /// # Arguments
    ///
    /// * `id` - The numeric identifier of the trip.
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::trips::trip::{Trip, TripType};
    /// let trip = Trip::new(1, 10, TripType::Constant)
    ///     .with_id(42)
    ///     .build();
    /// assert_eq!(trip.id, 42);
    /// ```
    pub fn with_id(mut self, id: TripID) -> Self {
        self.trip.id = id;
        self
    }

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
    /// use micro_traffic_sim_core::agents_types::AgentType;
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
    /// use micro_traffic_sim_core::behaviour::BehaviourType;
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

    /// Sets the probability for generating vehicle
    ///
    /// # Arguments
    ///
    /// * `probability` - Probability with which vehicles are generated if trip type is TripType::Random
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
    /// DEPRECATED: Tail size will be auto-resolved from the agent type.
    /// Setting with_vehicle_tail_size(0) for multi-cell agents (Bus, Truck, LargeBus) will be overridden by the default.
    /// There's no way to explicitly force a Bus to have zero tail size - the auto-resolution always runs when vehicle_tail_size == 0
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
    pub fn with_vehicle_tail_size(mut self, len: usize) -> Self {
        self.trip.vehicle_tail_size = len;
        self
    }

    /// Sets the speed limit for generated vehicles.
    ///
    /// If set to a value >= 0, this speed limit will be used instead of the
    /// behaviour-derived speed limit. The default value of -1 means the speed
    /// limit will be resolved from the behaviour type.
    ///
    /// # Arguments
    ///
    /// * `limit` - The speed limit value. Use -1 to resolve from behaviour type.
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::trips::trip::{Trip, TripType};
    /// use micro_traffic_sim_core::behaviour::BehaviourType;
    /// let trip = Trip::new(1, 10, TripType::Constant)
    ///     .with_allowed_behaviour_type(BehaviourType::Aggressive)
    ///     // Override behaviour speed limit with 5
    ///     .with_speed_limit(5)
    ///     .build();
    /// assert_eq!(trip.speed_limit, 5);
    /// ```
    pub fn with_speed_limit(mut self, limit: i32) -> Self {
        self.trip.speed_limit = limit;
        self
    }

    /// Builds the final `Trip` object with the configured properties.
    ///
    /// If `vehicle_tail_size` was not explicitly set, it will be automatically
    /// resolved from the `allowed_agent_type` using [`AgentType::tail_size_default()`].
    ///
    /// # Returns
    /// The fully constructed `Trip` object.
    pub fn build(mut self) -> Trip {
        // Auto-resolve tail size from agent type if not explicitly set
        if self.trip.vehicle_tail_size == 0 {
            self.trip.vehicle_tail_size = self.trip.allowed_agent_type.tail_size_default();
        }
        self.trip
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tail_size_auto_resolution() {
        // Car should have tail_size = 0
        let car_trip = Trip::new(1, 10, TripType::Constant)
            .with_allowed_agent_type(AgentType::Car)
            .build();
        assert_eq!(car_trip.vehicle_tail_size, 0);

        // Bus should have tail_size = 1
        let bus_trip = Trip::new(1, 10, TripType::Constant)
            .with_allowed_agent_type(AgentType::Bus)
            .build();
        assert_eq!(bus_trip.vehicle_tail_size, 1);

        // Truck should have tail_size = 1
        let truck_trip = Trip::new(1, 10, TripType::Constant)
            .with_allowed_agent_type(AgentType::Truck)
            .build();
        assert_eq!(truck_trip.vehicle_tail_size, 1);

        // LargeBus should have tail_size = 2
        let large_bus_trip = Trip::new(1, 10, TripType::Constant)
            .with_allowed_agent_type(AgentType::LargeBus)
            .build();
        assert_eq!(large_bus_trip.vehicle_tail_size, 2);
    }

    #[test]
    fn test_tail_size_explicit_override() {
        // Explicit tail size should not be overridden
        let trip = Trip::new(1, 10, TripType::Constant)
            .with_allowed_agent_type(AgentType::Car)
            .with_vehicle_tail_size(5)
            .build();
        assert_eq!(trip.vehicle_tail_size, 5);
    }

    #[test]
    fn test_speed_limit_default() {
        // Default speed_limit should be -1
        let trip = Trip::new(1, 10, TripType::Constant).build();
        assert_eq!(trip.speed_limit, -1);
    }

    #[test]
    fn test_speed_limit_explicit() {
        // Explicit speed_limit should be set
        let trip = Trip::new(1, 10, TripType::Constant)
            .with_speed_limit(5)
            .build();
        assert_eq!(trip.speed_limit, 5);
    }

    #[test]
    fn test_speed_limit_with_behaviour_first() {
        // Speed limit set after behaviour should still apply
        let trip = Trip::new(1, 10, TripType::Constant)
            .with_allowed_behaviour_type(BehaviourType::Aggressive)
            .with_speed_limit(3)
            .build();
        assert_eq!(trip.speed_limit, 3);
    }

    #[test]
    fn test_speed_limit_before_behaviour() {
        // Speed limit set before behaviour should still apply
        let trip = Trip::new(1, 10, TripType::Constant)
            .with_speed_limit(3)
            .with_allowed_behaviour_type(BehaviourType::Aggressive)
            .build();
        assert_eq!(trip.speed_limit, 3);
    }
}
