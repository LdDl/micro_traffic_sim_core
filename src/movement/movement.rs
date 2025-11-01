// In src/movement/mod.rs or src/movement/movement.rs
use crate::agents_types::AgentType;
use crate::agents::{VehicleID, VehicleRef};
use crate::grid::road_network::GridRoads;
use crate::grid::cell::CellID;
use crate::grid::zones::ZoneType;
use crate::maneuver::LaneChangeType;
use crate::geom::get_bearing;
use crate::verbose::*;
use indexmap::IndexMap;
use std::fmt;

/// Errors that can occur during vehicle movement execution.
///
/// These errors indicate serious problems with simulation state consistency
/// and typically require debugging the grid setup or intention computation.
#[derive(Debug, Clone)]
pub enum MovementError {
    /// Vehicle references a cell ID that doesn't exist in the grid network.
    ///
    /// This error indicates grid network corruption or inconsistent state:
    /// - Vehicle was assigned an invalid cell ID during intention computation
    /// - Grid network was modified during simulation without updating vehicles
    /// - Cell references became invalid due to network topology changes
    ///
    /// # Recovery
    /// This is typically a fatal error requiring simulation reset and
    /// investigation of grid setup or intention computation logic.
    CellNotFound {
        /// The cell ID that could not be found
        cell_id: CellID,
        /// The vehicle ID that referenced the invalid cell
        vehicle_id: VehicleID
    },
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

/// Executes movement for all vehicles based on their resolved intentions.
///
/// This function applies the final movement phase of cellular automata simulation,
/// updating vehicle positions according to their computed intentions and handling
/// all special movement cases.
///
/// **Internal function** - Do not call directly. Use [`Session::step()`](crate::simulation::session::Session::step) instead.
///
/// # Arguments
///
/// * `net` - The road network grid containing all cells and connections.
/// * `vehicles` - Mutable reference to all active vehicles in the simulation.
/// * `verbose` - Logging level for debug output.
///
/// # Returns
///
/// * `Ok(())` - Movement completed successfully for all vehicles.
/// * `Err(MovementError)` - Movement failed due to data inconsistency.
///
/// # Movement process
///
/// ## 1. Apply Vehicle Intentions
/// For each vehicle:
/// - Applies the intention computed by the intentions module.
/// - Resets conflict participation flag.
/// - Updates position to intended cell.
///
/// ## 2. Update Vehicle State
/// - **Bearing calculation**: Updates direction angle based on movement vector
/// - **Timers/counters management**: Decrements acceleration/maneuver/slowdown counters.
/// - **Tail cell updates**: Updates multi-cell vehicle positions.
/// - **Travel time increment**: Tracks how long vehicle has been in simulation.
///
/// ## 3. Handle Special Cases
///
/// ### Lane Change Timers
/// After lane changes, vehicles get temporary movement restrictions:
/// ```rust
/// // After left/right lane change
/// // vehicle.timer_non_acceleration = tail_size;  // Can't accelerate
/// // vehicle.timer_non_maneuvers = tail_size;     // Can't change lanes  
/// // vehicle.timer_non_slowdown = tail_size;      // Can't slow down
/// ```
///
/// ### Transit Logic **FUTURE WORKS**
/// Some vehicles e.g. busses follow multi-stop routes through transit cells:
/// - When reaching a transit stop, increments `transits_made` counter.
/// - Updates destination to next transit cell in sequence.
/// - Uses relax countdown for dwell time at stops.
///
/// ### Relax countdown **FUTURE WORKS**
/// Some vehicles must wait before moving:
/// - Buses at transit stops (dwell time).
/// - Vehicles after certain maneuvers.
/// - While countdown > 0, vehicle stays in current cell.
///
/// ## 4. Vehicle removal
/// Vehicles are removed from simulation when:
/// - **Despawn zone reached**: Vehicle enters [`ZoneType::Death`](ZoneType::Death) cell.
/// - **Destination reached**: Vehicle arrives at intended destination.
///
/// # Performance Characteristics
/// I did not measured actual performance yet, but in theory:
/// - **Time complexity**: O(n) where n = number of active vehicles.
/// - **Space complexity**: O(1) additional space.
/// - **Memory management**: Uses `swap_remove()` for efficient vehicle removal.
///
/// # Integration Notes
///
/// This function is called as step 7 in the simulation pipeline which is:
/// ```text
/// Session::step() Pipeline:
/// 1. Generate vehicles (trips)
/// 2. Update positions  
/// 3. Traffic light updates
/// 4. Prepare intentions  ← intentions module
/// 5. Collect conflicts   ← conflicts module  
/// 6. Solve conflicts     ← conflicts module
/// 7. Execute movement    ← THIS MODULE
/// 8. Collect state dump
/// ```
/// - Vehicle intentions have been computed and conflicts resolved.
/// - Each vehicle has a valid field `intention.intention_cell_id` to move to.
/// - Movement simply applies these pre-computed decisions.
/// - No pathfinding or decision-making occurs here.
///
/// # Logging
///
/// Provides structured logging at multiple verbosity levels:
/// - **Main**: Vehicle removal events (despawn zones/destination).
/// - **Additional**: Individual vehicle movements.
/// - **Detailed**: Tail cell updates and timer changes.
pub fn movement(
    net: &GridRoads,
    vehicles: &mut IndexMap<VehicleID, VehicleRef>,
    verbose: &LocalLogger,
) -> Result<(), MovementError> {
    if verbose.is_at_least(VerboseLevel::Main) {
        verbose.log_with_fields(
            EVENT_MOVEMENT,
            "Start movement process",
            &[("vehicles_num", &vehicles.len())]
        );
    }

    // Collect vehicles to remove (to avoid borrowing issues during iteration)
    let mut vehicles_to_remove = Vec::new();

    for (vehicle_id, vehicle_ref) in vehicles.iter() {
        let mut vehicle = vehicle_ref.borrow_mut();
        
        if verbose.is_at_least(VerboseLevel::Additional) {
            verbose.log_with_fields(
                EVENT_MOVEMENT_VEHICLE,
                "Moving vehicle",
                &[
                    ("vehicle_id", &vehicle.id),
                    ("current_cell", &vehicle.cell_id),
                    ("next_cell", &vehicle.intention.intention_cell_id),
                    ("intermediate_cells", &format!("{:?}", vehicle.intention.intermediate_cells)),
                ]
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
        if verbose.is_at_least(VerboseLevel::Additional) {
            verbose.log_with_fields(
                EVENT_MOVEMENT_VEHICLE,
                "Done movement for vehicle",
                &[
                    ("vehicle_id", &vehicle.id),
                    ("current_cell", &vehicle.cell_id),
                    ("next_cell", &vehicle.intention.intention_cell_id),
                    ("intermediate_cells", &format!("{:?}", vehicle.intention.intermediate_cells)),
                ]
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
            if verbose.is_at_least(VerboseLevel::Main) {
                verbose.log_with_fields(
                    EVENT_MOVEMENT_DEAD_END,
                    "Vehicle done movement due going to dead-end",
                    &[("vehicle_id", &vehicle.id)]
                );
            }
            vehicles_to_remove.push(*vehicle_id);
        }
        if vehicle.cell_id == vehicle.destination {
            // Vehicle has reached the destination
            if verbose.is_at_least(VerboseLevel::Main) {
                verbose.log_with_fields(
                    EVENT_MOVEMENT_DESTINATION,
                    "Vehicle done movement due reaching destination",
                    &[("vehicle_id", &vehicle.id)]
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