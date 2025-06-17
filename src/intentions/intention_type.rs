use crate::agents::{Vehicle, VehicleRef};

use std::fmt;

/// Represents the intention type of the agent for the cell
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntentionType {
    Undefined,
    /// Indicates whether a cell is wanted to be a target destination
    Target,
    /// Indicates whether a cell is wanted to be a transit cell for vehicles moving with a speed greater than 1
    Transit,
    /// Indicates whether a cell is wanted to be occupied by tail of some vehicle
    Tail,
}

impl fmt::Display for IntentionType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let intention_str = match self {
            IntentionType::Undefined => "undefined",
            IntentionType::Target => "target",
            IntentionType::Transit => "transit",
            IntentionType::Tail => "tail",
        };
        write!(f, "{}", intention_str)
    }
}

/// Represents the intention of the agent for the cell
#[derive(Debug, Clone)]
pub struct CellIntention{
    pub vehicle: VehicleRef,
    pub int_type: IntentionType,
}

impl CellIntention {
    /// Creates a new cell intention with the specified vehicle and intention type.
    ///
    /// # Arguments
    ///
    /// * `vehicle` - A reference to the vehicle that has the intention
    /// * `int_type` - The type of intention for the cell (target, transit, or tail)
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::agents::{Vehicle, AgentType};
    /// use micro_traffic_sim_core::intentions::{CellIntention, IntentionType};
    ///
    /// let vehicle = Vehicle::new(1)
    ///     .with_type(AgentType::Car)
    ///     .build();
    /// let cell_intention = CellIntention::new(&vehicle, IntentionType::Target);
    /// println!("Cell intention: {:?}", cell_intention);
    /// ```
    pub fn new(vehicle: VehicleRef, int_type: IntentionType) -> Self {
        CellIntention { vehicle, int_type }
    }

    /// Returns a reference to the associated vehicle if present.
    ///
    /// # Returns
    ///
    /// * `Some(&Vehicle)` - Reference to the vehicle if one exists
    /// * `None` - If no vehicle is associated with this intention
    ///
    /// # Example
    ///
    /// ```
    /// use micro_traffic_sim_core::agents::{Vehicle, AgentType};
    /// use micro_traffic_sim_core::intentions::{CellIntention, IntentionType};
    ///
    /// let vehicle = Vehicle::new(1)
    ///     .with_type(AgentType::Car)
    ///     .build();
    /// let cell_intention = CellIntention::new(Some(&vehicle), IntentionType::Target);
    /// println!("Cell intention: {:?}", cell_intention);
    ///
    /// match cell_intention.get_vehicle() {
    ///     Some(v) => println!("Vehicle ID: {}", v.id),
    ///     None => println!("No vehicle associated"),
    /// }
    /// ```
    // pub fn get_vehicle(&self) -> &Vehicle {
    //     self.vehicle
    // }
    pub fn get_vehicle_id(&self) -> u64 {
        self.vehicle.borrow().id
    }
}
