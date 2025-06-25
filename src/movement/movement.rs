// In src/movement/mod.rs or src/movement/movement.rs
use crate::agents::{VehicleID, VehicleRef, AgentType};
use crate::grid::road_network::GridRoads;
use crate::grid::cell::CellID;
use crate::grid::zones::ZoneType;
use crate::grid::lane_change_type::LaneChangeType;
use crate::geom::get_bearing;
use indexmap::IndexMap;
use std::fmt;

#[derive(Debug, Clone)]
pub enum MovementError {
    CellNotFound { cell_id: CellID, vehicle_id: VehicleID },
}

impl fmt::Display for MovementError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MovementError::CellNotFound { cell_id, vehicle_id } => {
                write!(f, "movement(): Can't find cell {} in the network for vehicle with ID {}", cell_id, vehicle_id)
            }
        }
    }
}

impl std::error::Error for MovementError {}

/// Executes movement for all vehicles based on their resolved intentions
pub fn movement(
    net: &GridRoads,
    vehicles: &mut IndexMap<VehicleID, VehicleRef>,
    verbose: bool,
) -> Result<(), MovementError> {
    if verbose {
        println!(
            "Start movement process: vehicles_num={}",
            vehicles.len()
        );
    }

    // Collect vehicles to remove (to avoid borrowing issues during iteration)
    let mut vehicles_to_remove = Vec::new();

    for (vehicle_id, vehicle_ref) in vehicles.iter() {
        let mut vehicle = vehicle_ref.borrow_mut();
        
        if verbose {
            println!(
                "Moving vehicle: vehicle_id={}, current_cell={}, next_cell={}, intermediate_cells={:?}",
                vehicle.id, vehicle.cell_id, vehicle.intention.intention_cell_id, vehicle.intention.intermediate_cells
            );
        }

        // Apply the vehicle's intention
        vehicle.apply_intention();
        vehicle.is_conflict_participant = false;

        // Update bearing only when next cell is different from current
        if vehicle.cell_id != vehicle.intention.intention_cell_id {
            let cell_from = net.get_cell(&vehicle.cell_id)
                .ok_or(MovementError::CellNotFound { 
                    cell_id: vehicle.cell_id, 
                    vehicle_id: vehicle.id 
                })?;
            let cell_to = net.get_cell(&vehicle.intention.intention_cell_id)
                .ok_or(MovementError::CellNotFound { 
                    cell_id: vehicle.intention.intention_cell_id, 
                    vehicle_id: vehicle.id 
                })?;

            let pt_from = cell_from.get_point();
            let pt_to = cell_to.get_point();
            vehicle.bearing = get_bearing(pt_from, pt_to);

            // Decrement timers
            if vehicle.timer_non_acceleration > 0 {
                vehicle.timer_non_acceleration -= 1;
            }
            if vehicle.timer_non_maneuvers > 0 {
                vehicle.timer_non_maneuvers -= 1;
            }
            if vehicle.timer_non_slowdown > 0 {
                vehicle.timer_non_slowdown -= 1;
            }
        }

        // Update tail cells only if vehicle is actually moving
        if vehicle.cell_id != vehicle.intention.intention_cell_id {
            let tail_size = vehicle.tail_cells.len();
            if tail_size > 0 {
                let tail_intention = vehicle.intention.tail_intention_cells.clone();
                vehicle.tail_cells = tail_intention;
            }
        }

        // Set timers for lane change maneuvers
        if vehicle.intention.intention_maneuver == LaneChangeType::ChangeLeft || 
           vehicle.intention.intention_maneuver == LaneChangeType::ChangeRight {
            let tail_size = vehicle.tail_cells.len() as i64;
            vehicle.timer_non_acceleration = tail_size;
            vehicle.timer_non_maneuvers = tail_size;
            vehicle.timer_non_slowdown = tail_size;
        }

        // Determine final cell (considering relax countdown)
        let final_cell = if vehicle.get_relax_countdown() > 0 {
            vehicle.relax_countdown_dec();
            vehicle.cell_id // Stay in current cell
        } else {
            vehicle.intention.intention_cell_id
        };

        vehicle.cell_id = final_cell;

        // Get the cell to check zone type
        let cell = net.get_cell(&vehicle.cell_id)
            .ok_or(MovementError::CellNotFound { 
                cell_id: vehicle.cell_id, 
                vehicle_id: vehicle.id 
            })?;
        if verbose {
            println!(
                "Done movement for vehicle: vehicle_id={}, current_cell={}, next_cell={}, intermediate_cells={:?}",
                vehicle.id, vehicle.cell_id, vehicle.intention.intention_cell_id, vehicle.intention.intermediate_cells
            );
        }

        let zone_type = cell.get_zone_type();

        // Handle bus transit logic
        if vehicle.vehicle_type == AgentType::Bus && zone_type == ZoneType::Transit && vehicle.cell_id == vehicle.destination {
            vehicle.transits_made_inc();
            let transits_made = vehicle.get_transits_made();
            if (transits_made as usize) < vehicle.transit_cells.len() && vehicle.get_relax_countdown() == 0 {
                vehicle.destination = vehicle.transit_cells[transits_made as usize];
                vehicle.relax_countdown_reset();
            }
        }

        vehicle.travel_time += 1;

        // Check for vehicle removal conditions
        if zone_type == ZoneType::Death && vehicle.cell_id != vehicle.destination {
            // Vehicle has reached the death zone
            if verbose {
                println!(
                    "Vehicle done movement due going to dead-end: vehicle_id={}",
                    vehicle.id
                );
            }
            vehicles_to_remove.push(*vehicle_id);
        }
        if vehicle.cell_id == vehicle.destination {
            // Vehicle has reached the destination
            if verbose {
                println!(
                    "Vehicle done movement due reaching destination: vehicle_id={}",
                    vehicle.id
                );
            }
            vehicles_to_remove.push(*vehicle_id);
        }
    }

    // Remove vehicles that have reached their destination or death zone
    for vehicle_id in vehicles_to_remove {
        // - Disrupts order - swaps with last element before removing
        // - Non-deterministic
        vehicles.swap_remove(&vehicle_id);
    }

    Ok(())
}