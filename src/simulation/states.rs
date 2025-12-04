use crate::grid::cell::CellID;
use crate::traffic_lights::lights::{TrafficLightID};
use crate::agents_types::AgentType;
use crate::agents::VehicleID;
use crate::traffic_lights::signals::SignalType;
use std::collections::HashMap;

/// Result of a single step execution
#[derive(Debug)]
pub struct StepResult {
    /// Indicates if the step was successful or if there was an error
    pub err: Option<Box<dyn std::error::Error>>,
    /// State data after the step
    pub data: AutomataState,
}

/// State of the automata at a specific timestamp
#[derive(Debug, Clone)]
pub struct AutomataState {
    /// Timestamp of the simulation step
    pub timestamp: i32,
    /// States of all vehicles at this timestamp
    pub vehicles: Vec<VehicleState>,
    /// States of all traffic light groups at this timestamp
    pub tls: HashMap<TrafficLightID, Vec<TrafficLightGroupState>>,
}

/// State of a single vehicle at a specific timestamp
#[derive(Debug, Clone)]
pub struct VehicleState {
    /// List of occupied points by the vehicle
    pub occupied_points: Vec<[f64; 2]>,
    /// Last known position of the vehicle
    pub last_point: [f64; 2],
    /// Last known cell ID of the vehicle
    pub last_cell: CellID,
    /// Cells occupied by tail. Order: [furthest from head, ..., closest to head]
    pub tail_cells: Vec<CellID>,
    /// List of last known intermediate cells traversed by the vehicle with speed > 1
    pub last_intermediate_cells: Vec<CellID>,
    /// Last known speed of the vehicle
    pub last_speed: i32,
    /// Last known bearing angle of the vehicle
    pub last_angle: f64,
    /// Type of the vehicle
    pub vehicle_type: AgentType,
    /// Total travel time of the vehicle in the simulation
    pub travel_time: i64,
    /// Unique identifier of the vehicle
    pub id: VehicleID,
}

/// State of a traffic light group at a specific timestamp
#[derive(Debug, Clone)]
pub struct TrafficLightGroupState {
    /// Signal group identifier
    pub group_id: i64,
    /// Last known signal type of the traffic light group
    pub last_signal: SignalType
}
