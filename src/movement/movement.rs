// In src/movement/mod.rs or src/movement/movement.rs
use crate::agents_types::AgentType;
use crate::agents::{VehicleID, Vehicle};
use crate::grid::road_network::GridRoads;
use crate::grid::cell::{CellID, CellState};
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
    vehicles: &mut IndexMap<VehicleID, Vehicle>,
    verbose: &LocalLogger,
) -> Result<(i32, i32), MovementError> {
    if verbose.is_at_least(VerboseLevel::Main) {
        verbose.log_with_fields(
            EVENT_MOVEMENT,
            "Start movement process",
            &[("vehicles_num", &vehicles.len())]
        );
    }

    // Collect vehicles to remove (to avoid borrowing issues during iteration)
    let mut vehicles_to_remove = Vec::new();
    let mut vehicles_completed = 0i32;
    let mut vehicles_lost = 0i32;

    for (vehicle_id, vehicle) in vehicles.iter_mut() {
        
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

        // The cell the vehicle ACTUALLY ends on this tick: a dwelling vehicle (relax_countdown > 0)
        // stays put even if it intended to move. Head, tail, timers and bearing must all
        // key off this single decision-otherwise a dwelling tailed vehicle's tail would
        // advance while its head stays, leaving the head inside its own tail.
        let was_dwelling = vehicle.get_relax_countdown() > 0;
        let final_cell = if was_dwelling {
            vehicle.relax_countdown_dec();
            // Stay in current cell
            vehicle.cell_id
        } else {
            vehicle.intention.intention_cell_id
        };
        let moved = final_cell != vehicle.cell_id;

        // Update bearing depending on intention maneuver
        if moved {
            // vehicle is moving? then set bearing based on actual movement
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
        } else {
            // vehicle is stopped or blocked? bearing = angle to forward direction
            use crate::maneuver::LaneChangeType;
            match vehicle.intention.intention_maneuver {
                LaneChangeType::Block | LaneChangeType::NoChange => {
                    let current_cell = net.get_cell(&vehicle.cell_id)
                        .ok_or(MovementError::CellNotFound {
                            cell_id: vehicle.cell_id,
                            vehicle_id: vehicle.id
                        })?;
                    let forward_id = current_cell.get_forward_id();
                    if forward_id > 0 {
                        if let Some(forward_cell) = net.get_cell(&forward_id) {
                            let pt_from = current_cell.get_point();
                            let pt_to = forward_cell.get_point();
                            vehicle.bearing = get_bearing(pt_from, pt_to);
                        }
                    }
                }
                _ => {
                    // keep bearing as is, dunno if it's good
                }
            }
        }

        // Decrement timers only if vehicle moved
        if moved {
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
        if moved {
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

        // Patience accrual: reset on any real move, otherwise the vehicle is stuck this
        // tick. Once wait_ticks reaches the patience threshold the vehicle is "desperate"
        // and overrides right-of-way in the conflict solver (see Vehicle::is_desperate).
        //
        // A non-contention stop must NOT accrue patience: a vehicle held by a red light
        // ahead (forward cell Banned) or still dwelling at a stop (relax_countdown) is not
        // being starved of right-of-way, so it must not be pushed into the desperate
        // override for a reason that has nothing to do with conflicts. Pause accrual in
        // those cases (do not reset, so genuine prior starvation is preserved).
        if moved {
            vehicle.wait_ticks = 0;
        } else {
            let held_by_red = net
                .get_cell(&vehicle.cell_id)
                .map(|c| c.get_forward_id())
                .filter(|&fwd| fwd > 0)
                .and_then(|fwd| net.get_cell(&fwd))
                .map(|fwd_cell| fwd_cell.get_state() == CellState::Banned)
                .unwrap_or(false);
            if !was_dwelling && !held_by_red {
                vehicle.wait_ticks = vehicle.wait_ticks.saturating_add(1);
            }
        }

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
                // Confusion means "current destination proven unreachable"; the verdict
                // does not transfer to a newly assigned destination
                vehicle.confusion = false;
                // The cached route still ends at the OLD stop the bus is sitting on; without
                // clearing it, advance_route_cursor would report on_route=true (current cell == last route cell)
                // and refresh_route would keep the dead cache, stalling the bus at the stop.
                // Clear it so next tick rebuilds a route to the new destination.
                vehicle.clear_cached_route();
                vehicle.relax_countdown_reset();
            }
        }

        vehicle.travel_time += 1;

        // Check for vehicle removal conditions
        if vehicle.cell_id == vehicle.trip_destination {
            // Vehicle has reached the destination (priority check)
            if verbose.is_at_least(VerboseLevel::Main) {
                verbose.log_with_fields(
                    EVENT_MOVEMENT_DESTINATION,
                    "Vehicle done movement due reaching destination",
                    &[("vehicle_id", &vehicle.id)]
                );
            }
            vehicles_to_remove.push(*vehicle_id);
            vehicles_completed += 1;
        } else if zone_type == ZoneType::Death {
            // Vehicle has reached the death zone without reaching destination (lost)
            if verbose.is_at_least(VerboseLevel::Main) {
                verbose.log_with_fields(
                    EVENT_MOVEMENT_DEAD_END,
                    "Vehicle done movement due going to dead-end",
                    &[("vehicle_id", &vehicle.id)]
                );
            }
            vehicles_to_remove.push(*vehicle_id);
            vehicles_lost += 1;
        }
    }

    // Remove vehicles that have reached their destination or death zone
    for vehicle_id in vehicles_to_remove {
        // - Disrupts order - swaps with last element before removing
        // - Non-deterministic
        vehicles.swap_remove(&vehicle_id);
    }

    Ok((vehicles_completed, vehicles_lost))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agents::{VehicleIntention, VehiclesStorage};
    use crate::geom::new_point;
    use crate::grid::cell::Cell;

    /// Regression: a dwelling tailed vehicle (relax_countdown > 0) that intended to move
    /// must NOT advance its tail while its head stays - head and tail share the single
    /// `final_cell`/`moved` decision. Without it, the tail moved (head ending inside its
    /// own tail).
    #[test]
    fn test_dwelling_tailed_vehicle_tail_does_not_advance() {
        // Linear road 1 -> 2 -> 3 -> 4.
        let mut net = GridRoads::new();
        for (id, fwd, x) in [(1i64, 2i64, 0.0), (2, 3, 1.0), (3, 4, 2.0), (4, -1, 3.0)] {
            net.add_cell(
                Cell::new(id)
                    .with_point(new_point(x, 0.0, None))
                    .with_forward_node(fwd)
                    .with_speed_limit(3)
                    .build(),
            );
        }

        // Tailed vehicle: head at 3, tail [1,2]; dwelling (relax_countdown > 0) but its
        // intention is a forward move to 4 with the already-advanced tail [2,3].
        let mut v = Vehicle::new(1)
            .with_cell(3)
            .with_tail_size(2, vec![1, 2])
            .with_destination(4)
            .with_relax_time(5)
            .build();
        v.relax_countdown_reset();
        assert!(v.get_relax_countdown() > 0, "precondition: vehicle is dwelling");
        v.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 4,
            intention_speed: 1,
            // the tail it WOULD have if it actually moved
            tail_intention_cells: vec![2, 3],
            ..Default::default()
        });

        let mut vehicles = VehiclesStorage::new();
        vehicles.insert(1, v);
        movement(&net, &mut vehicles, &LocalLogger::none()).unwrap();

        let v = vehicles.get(&1).expect("vehicle stays (dwelling, not removed)");
        assert_eq!(v.cell_id, 3, "head stays put while dwelling");
        assert_eq!(v.tail_cells, vec![1, 2], "tail must NOT advance while the head dwells");
        assert!(!v.tail_cells.contains(&v.cell_id), "head cell is not inside its own tail");
    }

    /// Regression: a multi-stop bus must complete its trip at the LAST transit stop, not the
    /// first. `trip_destination` used to be set to the FIRST stop, so a bus despawned as
    /// "completed" the instant it reached stop #1 and never served the rest of its route.
    #[test]
    fn test_bus_completes_at_last_stop_not_first() {
        // Linear road 1 -> 2 -> 3 -> 4; cells 2 and 4 are Transit stops.
        let mut net = GridRoads::new();
        for (id, fwd, x, zone) in [
            (1, 2, 0.0, ZoneType::Common),
            (2, 3, 1.0, ZoneType::Transit),
            (3, 4, 2.0, ZoneType::Common),
            (4, -1, 3.0, ZoneType::Transit),
        ] {
            net.add_cell(
                Cell::new(id)
                    .with_point(new_point(x, 0.0, None))
                    .with_forward_node(fwd)
                    .with_speed_limit(3)
                    .with_zone_type(zone)
                    .build(),
            );
        }

        // Bus #1 arriving at the FIRST stop (cell 2): its immediate destination is the first
        // stop, but the trip only completes at the LAST stop (cell 4). It must survive the
        // stop and re-target to the next one.
        let mut bus_first = Vehicle::new(1)
            .with_type(AgentType::Bus)
            .with_cell(1)
            .with_speed_limit(3)
            .with_destination(2)
            .with_trip_destination(4)
            .with_transit_cells(vec![2, 4])
            .build();
        bus_first.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 2,
            intention_speed: 1,
            ..Default::default()
        });

        // Bus #2 arriving at the LAST stop (cell 4), one transit already served: it must
        // despawn as completed here.
        let mut bus_last = Vehicle::new(2)
            .with_type(AgentType::Bus)
            .with_cell(3)
            .with_speed_limit(3)
            .with_destination(4)
            .with_trip_destination(4)
            .with_transit_cells(vec![2, 4])
            .build();
        bus_last.transits_made_inc(); // already served stop #1
        bus_last.set_intention(VehicleIntention {
            intention_maneuver: LaneChangeType::NoChange,
            intention_cell_id: 4,
            intention_speed: 1,
            ..Default::default()
        });

        let mut vehicles = VehiclesStorage::new();
        vehicles.insert(1, bus_first);
        vehicles.insert(2, bus_last);
        let (completed, _lost) = movement(&net, &mut vehicles, &LocalLogger::none()).unwrap();

        // Only the bus at its LAST stop completes.
        assert_eq!(completed, 1, "exactly one bus (at its last stop) completes");

        let bus_first = vehicles.get(&1).expect("bus must NOT despawn at the first stop");
        assert_eq!(bus_first.cell_id, 2, "bus #1 reached the first stop");
        assert_eq!(bus_first.destination, 4, "bus #1 re-targeted to the next (last) stop");
        assert_eq!(bus_first.get_transits_made(), 1, "bus #1 registered one transit");

        assert!(
            vehicles.get(&2).is_none(),
            "bus #2 completed and was removed at the last stop"
        );
    }

    /// Patience (`wait_ticks`) must accrue only under right-of-way starvation, not for a
    /// non-contention stop. A vehicle held by a red light ahead (forward cell Banned) or
    /// still dwelling at a stop must NOT gain patience (else it would wrongly become
    /// "desperate" and override right-of-way); a vehicle blocked by traffic still does.
    #[test]
    fn test_wait_ticks_not_accrued_on_red_or_dwell() {
        let mut net = GridRoads::new();
        // Red-light lane: 1 -> 2 (Banned).
        net.add_cell(Cell::new(1).with_speed_limit(3).with_forward_node(2).with_point(new_point(0.0, 0.0, None)).build());
        let mut banned = Cell::new(2).with_speed_limit(3).with_point(new_point(1.0, 0.0, None)).build();
        banned.set_state(CellState::Banned);
        net.add_cell(banned);
        // Traffic-block lane: 3 -> 4 (Free).
        net.add_cell(Cell::new(3).with_speed_limit(3).with_forward_node(4).with_point(new_point(0.0, 1.0, None)).build());
        net.add_cell(Cell::new(4).with_speed_limit(3).with_point(new_point(1.0, 1.0, None)).build());
        // Dwell lane: 5 -> 6.
        net.add_cell(Cell::new(5).with_speed_limit(3).with_forward_node(6).with_point(new_point(0.0, 2.0, None)).build());
        net.add_cell(Cell::new(6).with_speed_limit(3).with_point(new_point(1.0, 2.0, None)).build());

        // v1: held by the red light (forward Banned), blocked in place.
        let mut v1 = Vehicle::new(1).with_cell(1).build();
        v1.wait_ticks = 10;
        v1.set_intention(VehicleIntention { intention_maneuver: LaneChangeType::Block, intention_cell_id: 1, intention_speed: 0, ..Default::default() });
        // v2: blocked by traffic (forward Free, not red), blocked in place -> control.
        let mut v2 = Vehicle::new(2).with_cell(3).build();
        v2.wait_ticks = 10;
        v2.set_intention(VehicleIntention { intention_maneuver: LaneChangeType::Block, intention_cell_id: 3, intention_speed: 0, ..Default::default() });
        // v3: dwelling at a stop - intends to move but stays put this tick.
        let mut v3 = Vehicle::new(3).with_cell(5).with_relax_time(5).build();
        v3.relax_countdown_reset();
        v3.wait_ticks = 10;
        v3.set_intention(VehicleIntention { intention_maneuver: LaneChangeType::NoChange, intention_cell_id: 6, intention_speed: 1, ..Default::default() });

        let mut vehicles = VehiclesStorage::new();
        vehicles.insert(1, v1);
        vehicles.insert(2, v2);
        vehicles.insert(3, v3);
        movement(&net, &mut vehicles, &LocalLogger::none()).unwrap();

        assert_eq!(vehicles.get(&1).unwrap().wait_ticks, 10, "red light: patience must NOT accrue");
        assert_eq!(vehicles.get(&2).unwrap().wait_ticks, 11, "traffic block: patience accrues");
        assert_eq!(vehicles.get(&3).unwrap().wait_ticks, 10, "dwell: patience must NOT accrue");
    }
}