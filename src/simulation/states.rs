use crate::grid::cell::CellID;
use crate::traffic_lights::lights::{TrafficLightID};
use crate::agents::{AgentType, VehicleID};
use crate::traffic_lights::signals::SignalType;
use std::collections::HashMap;

/// Result of a single step execution
#[derive(Debug)]
pub struct StepResult {
    pub err: Option<Box<dyn std::error::Error>>,
    pub data: AutomataState,
}

/// State of the automata at a specific timestamp
#[derive(Debug, Clone)]
pub struct AutomataState {
    pub timestamp: i32,
    pub vehicles: Vec<VehicleState>,
    pub tls: HashMap<TrafficLightID, Vec<TrafficLightGroupState>>,
}

/// State of a single vehicle at a specific timestamp
#[derive(Debug, Clone)]
pub struct VehicleState {
    pub occupied_points: Vec<[f64; 2]>,
    pub last_point: [f64; 2],
    pub last_cell: CellID,
    pub last_intermediate_cells: Vec<CellID>,
    pub last_speed: i32,
    pub last_angle: f64,
    pub vehicle_type: AgentType,
    pub travel_time: i64,
    pub id: VehicleID,
}

/// State of a traffic light group at a specific timestamp
#[derive(Debug, Clone)]
pub struct TrafficLightGroupState {
    pub group_id: i64,
    pub last_signal: SignalType
}
