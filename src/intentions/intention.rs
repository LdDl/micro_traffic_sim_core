use super::{process_no_route_found, process_path, NoRouteError};
use crate::behaviour::BehaviourType;
use crate::agents::{
    TailIntentionManeuver, Vehicle, VehicleError, VehicleID, VehicleIntention,
};
use crate::grid::cell::{Cell, CellState};
use crate::maneuver::LaneChangeType;
use crate::grid::{cell::CellID, road_network::GridRoads};
use crate::intentions::{intention_type::IntentionType, Intentions};
use crate::shortest_path;
use crate::shortest_path::router::{shortest_path, path_no_goal, reconnect_to_cache};
use crate::shortest_path::heuristics::edge_time;
use crate::shortest_path::path::Path;
use crate::shortest_path::router::AStarError;
use crate::verbose::*;
use indexmap::IndexMap;
use crate::utils::rand::random_f64;
use std::collections::HashMap;
use std::f64::INFINITY;
use std::fmt;

/// Lane-change SAFETY (model physics): enforce a front gap (>= v_n) and a rear gap before a change,
/// so a merger never cuts in front of a car that would have to brake hard (real-world collision
/// risk). Always on (model physics), with a speed-aware rear gap (see `rear_safe`).
const LC_SAFETY_ENABLED: bool = true;

/// INCENTIVE depth (cells): a DISCRETIONARY change is taken only if the target lane is genuinely
/// more open this many cells ahead - else it is useless weaving and we WAIT. Route-mandatory changes
/// bypass this (see route_required), so navigation is unaffected. Always on (anti-weaving behaviour).
const LC_INCENTIVE_DEPTH: i32 = 2;

/// Error types for intention calculation failures.
#[derive(Debug, Clone)]
pub enum IntentionError {
    /// Source cell not found in the grid.
    NoSourceCell(CellID),
    /// Target cell not found in the grid.
    NoTargetCell(CellID),
    /// Left cell not found in the grid.
    NoLeftCell(CellID),
    /// Right cell not found in the grid.
    NoRightCell(CellID),
    /// Failed to find alternative path via left cell.
    LeftPathFind(CellID),
    /// Failed to find alternative path via right cell.
    RightPathFind(CellID),
    /// Vehicle-related error.
    VehicleError(VehicleError),
    /// No path found between source and target.
    NoPathFound(AStarError),
    /// No path found in no-route scenario.
    NoPathForNoRoute(NoRouteError),
    /// Cell has invalid speed limit.
    BadSpeedLimit(i64, i32),
}

impl fmt::Display for IntentionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoSourceCell(id) => write!(f, "Source cell {} not found", id),
            Self::NoTargetCell(id) => write!(f, "Target cell {} not found", id),
            Self::NoLeftCell(id) => write!(f, "Left cell {} not found", id),
            Self::NoRightCell(id) => write!(f, "Right cell {} not found", id),
            Self::LeftPathFind(msg) => {
                write!(f, "Can't find alternative path via left cell: {}", msg)
            }
            Self::RightPathFind(msg) => {
                write!(f, "Can't find alternative path via right cell: {}", msg)
            }
            Self::VehicleError(e) => write!(f, "Vehicle error: {}", e),
            Self::NoPathFound(e) => write!(f, "No path found: {}", e),
            Self::NoPathForNoRoute(e) => write!(f, "No path found for no route case: {}", e),
            Self::BadSpeedLimit(cell_id, speed_limit) => write!(
                f,
                "Cell {} has bad (negative) speed limit: {}",
                cell_id, speed_limit
            ),
        }
    }
}

/// Calculates all vehicle intentions for the current simulation step.
///
/// For each vehicle, determines the desired maneuver and target cell,
/// handling blocked vehicles and alternate maneuvers if needed.
/// Returns a storage of all intentions for conflict resolution.
pub fn prepare_intentions<'a, 'b>(
    net: &'a GridRoads,
    current_state: &HashMap<CellID, VehicleID>,
    speed_snapshot: &HashMap<CellID, i32>,
    vehicles: &'b mut IndexMap<VehicleID, Vehicle>,
    verbose: &LocalLogger,
    steps: i32,
    reroute_period: i32,
    reconnect_max_depth: usize,
) -> Result<Intentions, IntentionError> {
    let mut intentions = Intentions::new();
    let track_routing = verbose.is_at_least(VerboseLevel::Main);
    let mut routing_count: u64 = 0;
    let mut routing_sum_us: u64 = 0;
    let mut routing_min_us: u64 = u64::MAX;
    let mut routing_max_us: u64 = 0;

    if track_routing {
        verbose.log_with_fields(
            EVENT_INTENTIONS_CREATE,
            "Collect intentions for vehicles",
            &[
                ("vehicles_num", &vehicles.len()),
            ]
        );
    }
    for (_, vehicle) in vehicles.iter_mut() {
        if verbose.is_at_least(VerboseLevel::Additional) {
            verbose.log_with_fields(
                EVENT_INTENTION_VEHICLE,
                &format!("Processing vehicle {}", vehicle.id),
                &[
                    ("vehicle_id", &vehicle.id),
                    ("current_cell_id", &vehicle.cell_id),
                    ("speed", &vehicle.speed),
                    ("destination", &vehicle.destination),
                ]
            );
        }
        // Keep the cached route fresh: advance the cursor, periodically reroute,
        // and reconnect (or full-A* rebuild) when the vehicle fell off its route.
        refresh_route(net, vehicle, steps, reroute_period, reconnect_max_depth);
        let routing_start = std::time::Instant::now();
        let possible_intention = find_intention(net, current_state, speed_snapshot, &vehicle, verbose)?;
        if possible_intention.should_stop {
            // Calculate maneuvers_allowed for find_alternate_intention
            // Maneuvers are blocked if tail is still completing a previous maneuver
            let tail_maneuver = possible_intention.tail_maneuver.intention_maneuver;
            let maneuvers_allowed = vehicle.timer_non_maneuvers <= 0
                && tail_maneuver != LaneChangeType::ChangeRight
                && tail_maneuver != LaneChangeType::ChangeLeft;
            let alt = find_alternate_intention(net, current_state, speed_snapshot, &vehicle, maneuvers_allowed, reconnect_max_depth, steps)?;
            // Lane-change hysteresis: just after a change, 
            // suppress a REACTIVE re-weave (keep the forward roll / block).
            // Routing maneuvers go through the cached route, not here.
            // Cooldown duration is the baked-in lc_cooldown_cfg constant.
            let alternate_possible_intention = if vehicle.timer_reactive_lane_change > 0
                && matches!(alt.intention_maneuver, LaneChangeType::ChangeLeft | LaneChangeType::ChangeRight)
            {
                possible_intention
            } else {
                alt
            };
            if track_routing {
                let elapsed_us = routing_start.elapsed().as_micros() as u64;
                routing_count += 1;
                routing_sum_us += elapsed_us;
                routing_min_us = routing_min_us.min(elapsed_us);
                routing_max_us = routing_max_us.max(elapsed_us);
            }
            vehicle.set_intention(alternate_possible_intention);
            intentions.add_intention(vehicle, IntentionType::Target);
            continue;
        }
        if track_routing {
            let elapsed_us = routing_start.elapsed().as_micros() as u64;
            routing_count += 1;
            routing_sum_us += elapsed_us;
            routing_min_us = routing_min_us.min(elapsed_us);
            routing_max_us = routing_max_us.max(elapsed_us);
        }
        if verbose.is_at_least(VerboseLevel::Additional) {
            verbose.log_with_fields(
                EVENT_INTENTION_ADD,
                &format!("Adding intentions with vehicle {}", vehicle.id),
                &[
                    ("vehicle_id", &vehicle.id),
                    ("intention", &possible_intention),
                ]
            );
        }
        vehicle.set_intention(possible_intention);
        intentions.add_intention(vehicle, IntentionType::Target);
    }
    if track_routing && routing_count > 0 {
        let routing_avg_us = routing_sum_us / routing_count;
        verbose.log_with_fields(
            EVENT_ROUTING_STATS,
            "Routing timing stats",
            &[
                ("routing_count", &routing_count),
                ("routing_sum_us", &routing_sum_us),
                ("routing_min_us", &routing_min_us),
                ("routing_max_us", &routing_max_us),
                ("routing_avg_us", &routing_avg_us),
            ]
        );
    }
    Ok(intentions)
}

/// Keeps a vehicle's cached route usable for this tick:
/// 1. advances the route cursor to the current cell;
/// 2. if the cursor still matches and no periodic reroute is due, keeps the cache;
/// 3. if the vehicle fell off its route, tries a cheap bounded reconnect back onto it;
/// 4. otherwise (periodic reroute due, or reconnect failed) rebuilds the route with a
///    fresh full A*.
/// A no-op for destination-less or confused vehicles.
fn refresh_route(
    net: &GridRoads,
    vehicle: &mut Vehicle,
    steps: i32,
    reroute_period: i32,
    reconnect_max_depth: usize,
) {
    if vehicle.destination < 0 || vehicle.confusion {
        return;
    }
    let on_route = vehicle.advance_route_cursor();
    let due = reroute_period > 0 && (steps - vehicle.last_reroute) >= reroute_period;
    if on_route && !due {
        return;
    }
    // Off-route with a cache present: try a cheap bounded reconnect first.
    if !on_route && !vehicle.cached_route.is_empty() {
        if let Some(spliced) = reconnect_to_cache(
            vehicle.cell_id,
            &vehicle.cached_route,
            vehicle.route_idx,
            net,
            reconnect_max_depth,
        ) {
            vehicle.cached_route = spliced;
            vehicle.route_idx = 0;
            return; // a reconnect is not a reroute - keep last_reroute
        }
    }
    // Periodic reroute, or reconnect failed: rebuild the full route with a fresh A*.
    if let (Some(s), Some(g)) = (
        net.get_cell(&vehicle.cell_id),
        net.get_cell(&vehicle.destination),
    ) {
        if let Ok(path) = shortest_path(s, g, net, true, None) {
            vehicle.cached_route = path.vertices().iter().map(|c| c.get_id()).collect();
            vehicle.route_idx = 0;
            vehicle.last_reroute = steps;
        }
    }
}

/// Classifies the edge from `a` to `b` as a forward/left/right maneuver by matching
/// `b` against `a`'s neighbour links. Defaults to forward for a non-adjacent pair
/// (which should not occur on a valid cached route).
fn maneuver_between(a: &Cell, b: &Cell) -> LaneChangeType {
    let bid = b.get_id();
    if a.get_left_id() == bid {
        LaneChangeType::ChangeLeft
    } else if a.get_right_id() == bid {
        LaneChangeType::ChangeRight
    } else {
        LaneChangeType::NoChange
    }
}

/// Builds a short `Path` slice from the vehicle's cached route, starting at its
/// current cell (`cached_route[route_idx]`), at most `max_len` cells long. Returns
/// `None` when there is no cache, the cursor does not point at the current cell
/// (the vehicle fell off its route), or no cell resolves - in all of which the
/// caller falls back to a full A*. The returned path carries `cost = 0.0` (the
/// per-tick follow does not need the route's total cost; only `process_path`'s
/// obstacle/speed scan of the slice matters).
fn build_path_from_cache<'a>(
    vehicle: &Vehicle,
    net: &'a GridRoads,
    max_len: usize,
) -> Option<Path<'a>> {
    let route = &vehicle.cached_route;
    let start = vehicle.route_idx;
    // The cursor must point at the vehicle's current cell (advance_route_cursor ran).
    if route.get(start) != Some(&vehicle.cell_id) {
        return None;
    }
    let end = (start + max_len).min(route.len());
    let mut vertices: Vec<&Cell> = Vec::with_capacity(end - start);
    for &id in &route[start..end] {
        match net.get_cell(&id) {
            Some(c) => vertices.push(c),
            None => break, // dangling id - stop; the prefix collected so far is valid
        }
    }
    if vertices.is_empty() {
        return None;
    }
    let mut maneuvers = Vec::with_capacity(vertices.len().saturating_sub(1));
    for w in vertices.windows(2) {
        maneuvers.push(maneuver_between(w[0], w[1]));
    }
    Some(Path::new(vertices, maneuvers, 0.0))
}

/// Returns true when rolling onto `forward_id` keeps the vehicle's destination
/// reachable, i.e. `forward_id` is still on (or can rejoin) the cached route. Used to
/// stop the forward-fallback from driving a non-confused vehicle into a one-way pocket
/// (the sole cause of lost vehicles). Cheap O(1) fast paths first, then a bounded BFS.
///
/// `max_depth` MUST be the same `reconnect_max_depth` the per-tick reconnect uses: this
/// guard authorizes a roll only if next tick's `reconnect_to_cache` (run from the very
/// cell we roll onto) will succeed. A larger bound here green-lights rolls the next tick
/// then cannot follow up on, forcing a needless full A* rebuild; a smaller one stalls
/// rolls that would in fact reconnect. Equal bounds make "authorized" mean "reconnects".
fn forward_keeps_reachable(vehicle: &Vehicle, forward_id: CellID, net: &GridRoads, max_depth: usize) -> bool {
    let route = &vehicle.cached_route;
    let idx = vehicle.route_idx;
    // The route continues straight onto forward_id (the common case).
    if route.get(idx + 1) == Some(&forward_id) {
        return true;
    }
    // forward_id is the destination itself (the last route cell, whose neighbours are
    // not on the cache ahead, so reconnect_to_cache would miss it).
    if route.last() == Some(&forward_id) {
        return true;
    }
    // Otherwise: can we still get back onto the route from forward_id within the bound?
    reconnect_to_cache(forward_id, route, idx, net, max_depth).is_some()
}

/// Computes the movement intention for a single vehicle.
///
/// Determines the best maneuver (forward, lane change, block, etc.)
/// and target cell, considering speed, acceleration, obstacles, and pathfinding.
pub fn find_intention<'a>(
    net: &'a GridRoads,
    current_state: &HashMap<CellID, VehicleID>,
    _speed_snapshot: &HashMap<CellID, i32>,
    vehicle: &'a Vehicle,
    _verbose: &LocalLogger,
) -> Result<VehicleIntention, IntentionError> {
    if vehicle.strategy_type == BehaviourType::Block {
        let result = VehicleIntention {
            intention_maneuver: LaneChangeType::Block,
            intention_speed: 0,
            destination: None,
            confusion: None,
            intention_cell_id: vehicle.cell_id,
            tail_intention_cells: vec![],
            intermediate_cells: Vec::with_capacity(0),
            tail_maneuver: TailIntentionManeuver::default(),
            should_stop: false,
        };
        return Ok(result);
    }

    let tail_maneuver = match vehicle.scan_tail_maneuver(net) {
        Ok(maneuver) => maneuver,
        Err(e) => return Err(IntentionError::VehicleError(e)),
    };

    let source_cell = net
        .get_cell(&vehicle.cell_id)
        .ok_or(IntentionError::NoSourceCell(vehicle.cell_id))?;

    // Deadend check
    if source_cell.get_forward_id() < 0 && source_cell.get_right_id() < 0  && source_cell.get_left_id() < 0 {
        let result = VehicleIntention {
            intention_maneuver: LaneChangeType::Block,
            intention_speed: 0,
            destination: None,
            confusion: None,
            intention_cell_id: vehicle.cell_id,
            tail_intention_cells: vec![],
            intermediate_cells: Vec::with_capacity(0),
            tail_maneuver: tail_maneuver,
            should_stop: false,
        };
        return Ok(result);
    }

    let speed_limit = source_cell.get_speed_limit().min(vehicle.speed_limit);

    if speed_limit < 0 {
        return Err(IntentionError::BadSpeedLimit(
            source_cell.get_id(),
            speed_limit,
        ));
    }
    if speed_limit == 0 {
        let result = VehicleIntention {
            intention_maneuver: LaneChangeType::Block,
            intention_speed: 0,
            destination: None,
            confusion: None,
            intention_cell_id: vehicle.cell_id,
            tail_intention_cells: vec![],
            intermediate_cells: Vec::with_capacity(0),
            tail_maneuver: tail_maneuver,
            should_stop: false,
        };
        return Ok(result);
    }

    // If stopped at red traffic light do early return
    let forward_cell_id = source_cell.get_forward_id();
    if forward_cell_id > 0 {
        if let Some(forward_cell) = net.get_cell(&forward_cell_id) {
            if forward_cell.get_state() == CellState::Banned {
                let result = VehicleIntention {
                    intention_maneuver: LaneChangeType::Block,
                    intention_speed: 0,
                    destination: None,
                    confusion: None,
                    intention_cell_id: vehicle.cell_id,
                    tail_intention_cells: vec![],
                    intermediate_cells: Vec::with_capacity(0),
                    tail_maneuver: tail_maneuver,
                    should_stop: false,
                };
                return Ok(result);
            }
        }
    }

    // Vehicle's speed should not be greater than speed limit
    let mut intention_speed = vehicle.speed.min(speed_limit);

    // NaSch acceleration: a moving vehicle tries to speed up by one cell/step, bounded by the speed limit.
    // Stopped vehicle (speed == 0) may start even while its acceleration timer is active.
    let mut speed_possible = intention_speed;
    let acceleration_allowed = vehicle.timer_non_acceleration <= 0 || vehicle.speed == 0;
    if acceleration_allowed {
        speed_possible = (speed_possible + 1).min(speed_limit);
    }

    // VDR randomization (velocity-dependent / slow-to-start), the canonical NaSch dawdle
    // `v -> max(v-1, 0)` with prob p.
    // Stopped vehicle uses the higher probability `p0`.
    // Moving one uses `p`. p0 > p makes a jam
    // discharge slower than it fills => CAPACITY DROP and metastable / hysteretic flow near the
    // critical density (the VDR signature). The gate is `speed_possible > 0`, so it also applies to a
    // just-started stopped vehicle (speed_possible == 1) - that is where slow-to-start acts.
    let slowdown_allowed = vehicle.timer_non_slowdown <= 0;
    let slowdown_prob = if vehicle.speed == 0 {
        vehicle.slow_to_start_factor_p0
    } else {
        vehicle.slow_down_factor_p
    };
    if slowdown_allowed && speed_possible > 0 && random_f64() < slowdown_prob {
        speed_possible = (speed_possible - 1).max(0);
    }

    // Considering that vehicle always wants to accelerate:
    let _observe_distance = speed_possible + vehicle.min_safe_distance;

    // Check if maneuvers are allowed (they could be prohibeted due the vehicle's tail is not done previous maneuver yet)
    let maneuvers_allowed = vehicle.timer_non_maneuvers <= 0
        && tail_maneuver.intention_maneuver != LaneChangeType::ChangeRight
        && tail_maneuver.intention_maneuver != LaneChangeType::ChangeLeft;

    let destination: Option<CellID> = None;
    let mut confusion: Option<bool> = None;

    // println!(
    //     "Vehicle {} at cell {} with speed {} (possible {}) towards {} with observe distance {} and slow factor {}: {}",
    //     vehicle.id,
    //     source_cell.get_id(),
    //     intention_speed,
    //     speed_possible,
    //     vehicle.destination,
    //     observe_distance,
    //     slow_down_factor_p,
    //     if _is_slowdown { " (slowdown)" } else { "" }
    // );

    // Try to follow the cached route (O(speed) read) instead of a per-tick full A*.
    // None when the vehicle has no cache or fell off it (then we fall back to A*).
    let cache_path = if vehicle.destination >= 0 && !vehicle.confusion {
        build_path_from_cache(vehicle, net, (speed_possible.max(1) + 2) as usize)
    } else {
        None
    };

    let mut path = match vehicle.destination {
        // Handle case when vehicle has no destination,H
        // therefore it should be considered as keep going where possible
        dest if dest < 0 => {
            match path_no_goal(
                source_cell,
                net,
                maneuvers_allowed,
                // Only the cells the vehicle can actually reach this step are needed by process_path
                // (it scans at most `speed_possible` cells). A fixed 1000 built the ENTIRE ring/road
                // per vehicle per step = O(L) and was the dominant per-step cost; bound it like the
                // cache path (speed_possible + 2).
                speed_possible.max(1) + 2,
            ) {
                Ok(path) => path,
                Err(e) => {
                    println!(
                        "----->No path found error: {}", e
                    );
                    return Err(IntentionError::NoPathFound(e))
                }
            }
        },
        // Reachability over directed edges is monotone: once the destination is
        // unreachable from the current cell, it is unreachable from every cell the
        // vehicle can ever get to. Full A* would just fail again (the most expensive
        // failure mode - it exhausts the whole reachable component), so skip routing
        // for confused vehicles entirely. NOTE: this holds only while the grid is
        // static during a session and the destination is not reassigned.
        _ if vehicle.confusion => {
            let new_path = match process_no_route_found(source_cell, net) {
                Ok(path) => path,
                Err(e) => return Err(IntentionError::NoPathForNoRoute(e)),
            };
            intention_speed = 1;
            speed_possible = intention_speed;
            new_path
        },
        // Follow the cached route if the vehicle is on it (built above): the slice is
        // fed through the same process_path/assembly below, replacing the per-tick
        // full A* with an O(speed) cache read.
        _ if cache_path.is_some() => cache_path.unwrap(),
        // Off the cached route (or no cache): fall back to a full A*.
        _ => {
            let target_cell = net
                .get_cell(&vehicle.destination)
                .ok_or(IntentionError::NoTargetCell(vehicle.destination))?;
            match shortest_path(
                source_cell,
                target_cell,
                net,
                maneuvers_allowed,
                None,
                // Some(observe_distance + 1),  // depth-limited
            ) {
                Ok(path) => path,
                Err(e)
                    if e != shortest_path::router::AStarError::NoPathFound {
                        start_id: source_cell.get_id(),
                        end_id: target_cell.get_id(),
                    } =>
                {
                    return Err(IntentionError::NoPathFound(e));
                }
                Err(_) => {
                    // A* with maneuvers disabled (tail still completing a previous lane change)
                    // can report NoPathFound merely because the only continuation needs a
                    // maneuver that is briefly on cooldown - a TIMING constraint, not an
                    // unreachable destination. `confusion` is a PERMANENT "unreachable" verdict
                    // (confused vehicles skip routing entirely and are eventually despawned as
                    // `lost`), so latch it ONLY when the destination is unreachable even WITH
                    // maneuvers allowed. Otherwise we still roll forward (process_no_route_found,
                    // a NoChange step) so the tail finishes its maneuver and the cooldown clears,
                    // and we leave confusion unset so next tick re-routes onto the real path
                    // instead of writing the trip off as lost.
                    let truly_unreachable = maneuvers_allowed
                        || shortest_path(source_cell, target_cell, net, true, None).is_err();
                    let new_path = match process_no_route_found(source_cell, net) {
                        Ok(path) => path,
                        Err(e) => return Err(IntentionError::NoPathForNoRoute(e)),
                    };
                    // Do NOT overwrite destination - keep original trip destination
                    intention_speed = 1;
                    speed_possible = intention_speed;
                    if truly_unreachable {
                        confusion = Some(true);
                    }
                    new_path
                }
            }
        }
    };

    // println!(
    //     "  -> Found path with cost {} and vertices: {:?}",
    //     path.cost(),
    //     path.vertices()
    //         .iter()
    //         .map(|cell| cell.get_id())
    //         .collect::<Vec<CellID>>()
    // );
    // Process path to find wanted maneuver, success forward movement and to trim path
    let observable_path = process_path(
        &mut path,
        speed_possible,
        vehicle.destination,
        current_state,
    );
    // println!(
    //     "  -> Observable path: wanted_maneuver={:?}, last_cell_state={:?}, trimmed_path={:?}, has_vehicle_on_path={}, speed_limit_reached={}, stopped_on_maneuver={}, stopped_speed_possible={}",
    //     observable_path.wanted_maneuver,
    //     observable_path.last_cell_state,
    //     observable_path.trimmed_path.iter().map(|cell| cell.get_id()).collect::<Vec<CellID>>(),
    //     observable_path.has_vehicle_on_path,
    //     observable_path.speed_limit_reached,
    //     observable_path.stopped_on_maneuver,
    //     observable_path.stopped_speed_possible,
    // );
    let wanted_maneuver = observable_path.wanted_maneuver;
    let last_cell_state = observable_path.last_cell_state;
    let vertices = observable_path.trimmed_path;

    // Possible speed should not be greater than success forward movement counter
    // If len(vertices) < speed, then it means that vehicle slow downed due conflict #1.1. Otherwise vehicle could accelerate
    speed_possible = speed_possible.min(vertices.len() as i32);

    if vertices.len() > 0 {
        let wanted_cell_id = vertices[vertices.len() - 1].get_id();
        let result = VehicleIntention {
            intention_maneuver: wanted_maneuver,
            intention_speed: speed_possible,
            destination: destination,
            confusion: confusion,
            intention_cell_id: wanted_cell_id,
            tail_intention_cells: vec![],
            intermediate_cells: vertices[..vertices.len() - 1]
                .iter()
                .map(|cell| cell.get_id())
                .collect(),
            tail_maneuver: tail_maneuver,
            should_stop: false,
        };
        return Ok(result);
    }
    // Stopped due the traffic light ahead
    if last_cell_state != CellState::Free {
        let result = VehicleIntention {
            intention_maneuver: LaneChangeType::Block,
            intention_speed: 0,
            destination: destination,
            confusion: confusion,
            intention_cell_id: source_cell.get_id(),
            tail_intention_cells: vec![],
            intermediate_cells: Vec::with_capacity(0),
            tail_maneuver: tail_maneuver,
            should_stop: false,
        };
        return Ok(result);
    }
    // Otherwise collect vehicles which can't move forward. This includes:
    // - Vehicles with speed_possible = 0 (e.g., acceleration blocked by timer after lane change)
    // - Vehicles blocked by other reasons (has_vehicle_on_path handled earlier)
    // Then they are trying to lane change in separate loop
    // (in further we could make cooperative drives which allow other vehicles to change lane).
    // Then we know vehicles which can't move at all.
    let result = VehicleIntention {
        intention_maneuver: LaneChangeType::Block,
        intention_speed: 0,
        destination: destination,
        confusion: confusion,
        intention_cell_id: source_cell.get_id(),
        tail_intention_cells: vec![],
        intermediate_cells: Vec::with_capacity(0),
        tail_maneuver: tail_maneuver,
        should_stop: true,
    };
    Ok(result)
}

/* Change it according to right-hand or left-hand traffic (driving side) */
/* @todo: should be an argument in further  */
const UNDEFINED_MANEUVER: LaneChangeType = LaneChangeType::ChangeRight;

/// Attempts to find an alternate maneuver (lane change) for a blocked vehicle.
///
/// Helper: Creates a block intention
fn create_block_intention(cell_id: CellID, should_stop: bool) -> VehicleIntention {
    VehicleIntention {
        intention_maneuver: LaneChangeType::Block,
        intention_speed: 0,
        destination: None,
        confusion: None,
        intention_cell_id: cell_id,
        tail_intention_cells: vec![],
        intermediate_cells: Vec::with_capacity(0),
        tail_maneuver: TailIntentionManeuver::default(),
        should_stop,
    }
}

/// INCENTIVE predicate: change only if the target lane is more open `LC_INCENTIVE_DEPTH`
/// cells ahead.
fn lc_incentive_pass(target_cell_id: CellID, net: &GridRoads, occ: &HashMap<CellID, VehicleID>) -> bool {
    if LC_INCENTIVE_DEPTH <= 0 {
        return true;
    }
    let fwd = net.get_cell(&target_cell_id).map(|c| c.get_forward_id()).unwrap_or(-1);
    clear_forward(fwd, net, occ, LC_INCENTIVE_DEPTH)
}

/// P1 stochastic lane change: a found change is COMMITTED only with probability
/// `change_p1` (a per-vehicle behaviour attribute);
/// otherwise the vehicle waits this step and retries next step.
/// Models real hesitation - drivers do not merge 100% of the steps they could.
/// The draw is attribute-driven so seeded runs in a reproducible way.
fn lc_p1_pass(change_p1: f64) -> bool {
    if change_p1 >= 1.0 {
        return true;
    }
    random_f64() < change_p1
}

/// True if at least `need` cells ahead of `start` (inclusive) along the forward chain are free, or
/// the road ends first (open road = clear).
/// Reads start-of-step occupancy.
/// Front safety gap.
fn clear_forward(start: CellID, net: &GridRoads, occ: &HashMap<CellID, VehicleID>, need: i32) -> bool {
    let mut id = start;
    for _ in 0..need {
        if id < 0 {
            return true;
        }
        if occ.get(&id).map(|&v| v > 0).unwrap_or(false) {
            return false;
        }
        id = net.get_cell(&id).map(|c| c.get_forward_id()).unwrap_or(-1);
    }
    true
}

/// Speed-aware REAR safety gap for a lane change. Walks back from `start` (the target lane's
/// predecessor of the merge cell) up to `max_look` cells via the forward-predecessor index.
/// If a vehicle is found at distance `d` (1-indexed), the change is safe
/// only if `d >= rear_speed + min_safe` - the rear car has room to decelerate and follow without an emergency stop.
/// No vehicle within `max_look` => safe. Reads start-of-step occupancy + speed snapshots
/// it is synchronous and deterministic.
/// This replaces the old fixed `Vmax + A` look-back, which blocked nearly
/// every dense-traffic merge because it ignored that the rear car is usually slow.
fn rear_safe(
    start: CellID,
    net: &GridRoads,
    occ: &HashMap<CellID, VehicleID>,
    speed_snapshot: &HashMap<CellID, i32>,
    min_safe: i32,
    max_look: i32,
) -> bool {
    let mut id = start;
    for d in 1..=max_look {
        if id < 0 {
            // road begins - no follower
            return true;
        }
        if occ.get(&id).map(|&v| v > 0).unwrap_or(false) {
            let rear_speed = *speed_snapshot.get(&id).unwrap_or(&0);
            return d >= rear_speed + min_safe;
        }
        id = net.get_back_id(id);
    }
    // no vehicle within the look-back window
    true
}

/// Helper: Checks if alternate path (left or right) is available and calculates cost
fn check_alternate_direction(
    cell_id: CellID,
    source_cell: &Cell,
    target_cell: &Cell,
    net: &GridRoads,
    current_state: &HashMap<CellID, VehicleID>,
    speed_snapshot: &HashMap<CellID, i32>,
    direction: &str,
    v_n: i32,
    min_safe: i32,
    aggressor_cut_in: bool,
    global_vmax: i32,
    max_depth: Option<i32>,
) -> Result<(CellID, f64), IntentionError> {
    if cell_id <= 0 {
        return Ok((-1, INFINITY));
    }

    let cell = net.get_cell(&cell_id).ok_or_else(|| {
        if direction == "left" {
            IntentionError::NoLeftCell(source_cell.get_id())
        } else {
            IntentionError::NoRightCell(source_cell.get_id())
        }
    })?;

    let is_blocked = current_state
        .get(&cell_id)
        .map(|&id| id > 0)
        .unwrap_or(false);

    if is_blocked || cell.get_state() != CellState::Free {
        return Ok((-1, INFINITY));
    }

    // SAFETY condition: only change into the side lane with a clear FRONT gap
    // (>= v_n - the merger keeps its speed) AND a safe REAR gap.
    // The rear gap is SPEED-AWARE: the nearest car behind in the target lane
    // must have at least its own speed + A cells of room, so it can decelerate to
    // follow without an emergency stop (a real collision risk / unsafe cut-in).
    // A fixed global-Vmax look-back wrongly blocked nearly every dense-traffic merge (slow rear cars
    // need little room); the speed-aware gap allows dense merges while still forbidding cutting in
    // front of a fast approacher.
    // Reads start-of-step snapshots only.
    //
    // CUT-IN aggressors (aggressive_level > AGGRESSOR_CUT_IN_THRESHOLD, i.e. `aggressor_cut_in`)
    // DELIBERATELY ignore the rear gap: they squeeze in front of the follower, who is then forced
    // to brake (the resolver hands them the contested cell via `aggressor_advantage`).
    // The FRONT gap still applies even to them - you cannot drive into the car ahead - and the cell must be
    // physically free, so this never causes a same-cell collision, only an unsafe cut-in.
    if LC_SAFETY_ENABLED {
        let front_ok = clear_forward(cell.get_forward_id(), net, current_state, v_n);
        let max_look = (global_vmax + min_safe).max(1);
        let rear_ok = aggressor_cut_in
            || rear_safe(net.get_back_id(cell_id), net, current_state, speed_snapshot, min_safe, max_look);
        if !front_ok || !rear_ok {
            return Ok((-1, INFINITY));
        }
    }

    match shortest_path(cell, target_cell, net, true, max_depth) {
        Ok(path) => {
            // previously source_cell.distance_to(cell) was used as additiona. now it is edge_time(source_cell, cell)
            // to match units with path.cost() (travel time).
            let cost = path.cost() + edge_time(source_cell, cell);
            Ok((cell_id, cost))
        }
        Err(shortest_path::router::AStarError::NoPathFound { .. }) => {
            Ok((-1, INFINITY))
        }
        Err(_) => {
            if direction == "left" {
                Err(IntentionError::LeftPathFind(cell_id))
            } else {
                Err(IntentionError::RightPathFind(cell_id))
            }
        }
    }
}

/// If the vehicle cannot move forward, tries left or right lane changes
/// and selects the best available option.
///
/// # Arguments
/// * `maneuvers_allowed` - Whether lane changes are allowed (false if tail is still completing a maneuver)
/// * `reconnect_max_depth` - Bound for the forward-roll reachability guard; MUST match the
///   per-tick reconnect bound so an authorized roll is guaranteed to reconnect next tick
///   (see [`forward_keeps_reachable`]).
pub fn find_alternate_intention<'a>(
    net: &'a GridRoads,
    current_state: &HashMap<CellID, VehicleID>,
    speed_snapshot: &HashMap<CellID, i32>,
    vehicle: &'a Vehicle,
    maneuvers_allowed: bool,
    reconnect_max_depth: usize,
    _steps: i32,
) -> Result<VehicleIntention, IntentionError> {
    let source_cell_id = vehicle.cell_id;
    let target_cell_id = vehicle.destination;

    // If maneuvers are not allowed (tail still completing previous maneuver), block immediately
    if !maneuvers_allowed {
        return Ok(create_block_intention(source_cell_id, false));
    }

    // Destination-less vehicles (no goal, e.g. ring-road circulation) have no route to probe
    // for a route-distance-based lane change, and `get_cell(-1)` below would error with
    // NoTargetCell(-1). They simply wait for the gap ahead to open (correct NaSch congested
    // behaviour). When the incentive-based rule lands, the no-goal case will use a speed
    // incentive instead of a route distance.
    if target_cell_id < 0 {
        return Ok(create_block_intention(source_cell_id, false));
    }

    let source_cell = net
        .get_cell(&source_cell_id)
        .ok_or(IntentionError::NoSourceCell(source_cell_id))?;

    let target_cell = net
        .get_cell(&target_cell_id)
        .ok_or(IntentionError::NoTargetCell(target_cell_id))?;

    // Global system Vmax for the rear safety gap `D >= Vmax + A` (worst-case follower speed).
    let global_vmax = net.get_max_speed() as i32;

    // Check left and right alternate paths (no depth limit to see full route).
    // A confused vehicle's destination is already proven unreachable, and
    // reachability is monotone along directed edges - both probes would run a
    // full failed A* just to return INFINITY, so skip them.
    let (left_cell_id, min_left_dist) = if vehicle.confusion {
        (-1, INFINITY)
    } else {
        check_alternate_direction(
            source_cell.get_left_id(),
            source_cell,
            target_cell,
            net,
            current_state,
            speed_snapshot,
            "left",
            vehicle.speed,
            vehicle.min_safe_distance,
            vehicle.is_aggressor(),
            global_vmax,
            None,
            // Some(vehicle.speed),  // depth-limited
        )?
    };

    let (right_cell_id, min_right_dist) = if vehicle.confusion {
        (-1, INFINITY)
    } else {
        check_alternate_direction(
            source_cell.get_right_id(),
            source_cell,
            target_cell,
            net,
            current_state,
            speed_snapshot,
            "right",
            vehicle.speed,
            vehicle.min_safe_distance,
            vehicle.is_aggressor(),
            global_vmax,
            None,
            // Some(vehicle.speed),  // depth-limited
        )?
    };

    // If both paths are impossible (infinite distance), don't attempt a lane change.
    // Before blocking, try to keep rolling forward: a driver stuck next to a jammed
    // lane drives along it and merges at a gap further ahead.
    //
    // An unguarded roll can push the vehicle off its route into a one-way pocket from
    // which the destination is unreachable, after which the per-tick A* returns
    // NoPathFound, confusion latches, and the vehicle is despawned as `lost` (before this
    // guard existed, this was measured to be the sole cause of lost vehicles). So a
    // non-confused vehicle is allowed to roll forward only while it can still get back to
    // its route (`forward_keeps_reachable`); otherwise it waits in place (a recoverable
    // stall) instead of driving into a trap.
    //
    // A vehicle that is ALREADY confused has no reachable route to protect, so it keeps
    // rolling unconditionally - that is what carries it to a Death zone for removal
    // (without it, a confused vehicle would block forever as a zombie).
    if min_left_dist == INFINITY && min_right_dist == INFINITY {
        let forward_cell_id = source_cell.get_forward_id();
        if forward_cell_id > 0 {
            if let Some(forward_cell) = net.get_cell(&forward_cell_id) {
                let is_occupied = current_state
                    .get(&forward_cell_id)
                    .map(|&id| id > 0)
                    .unwrap_or(false);
                let safe_to_roll = vehicle.confusion
                    || forward_keeps_reachable(vehicle, forward_cell_id, net, reconnect_max_depth);
                if !is_occupied
                    && forward_cell.get_state() == CellState::Free
                    && forward_cell.get_speed_limit() > 0
                    && safe_to_roll
                {
                    return Ok(VehicleIntention {
                        intention_maneuver: LaneChangeType::NoChange,
                        intention_speed: 1,
                        destination: None,
                        confusion: None,
                        intention_cell_id: forward_cell_id,
                        tail_intention_cells: vec![],
                        intermediate_cells: Vec::with_capacity(0),
                        tail_maneuver: TailIntentionManeuver::default(),
                        should_stop: false,
                    });
                }
            }
        }
        return Ok(create_block_intention(source_cell_id, true));
    }

    // Choose best maneuver based on distance comparison
    let (min_cell, intention_maneuver) = if min_left_dist < min_right_dist {
        (left_cell_id, LaneChangeType::ChangeLeft)
    } else if min_right_dist < min_left_dist {
        (right_cell_id, LaneChangeType::ChangeRight)
    } else {
        // Equal distances - use UNDEFINED_MANEUVER as tiebreaker
        if UNDEFINED_MANEUVER == LaneChangeType::ChangeRight {
            (right_cell_id, LaneChangeType::ChangeRight)
        } else {
            (left_cell_id, LaneChangeType::ChangeLeft)
        }
    };

    // Apply the chosen maneuver if valid
    if min_cell > 0 {
        // ROUTE-NECESSITY: is the chosen side the cell the cached route requires next?
        // A mandatory route change must NOT be suppressed by the discretionary gates below (else the vehicle
        // misses its turn -> would have to reroute -> the historical cause of `lost`).
        // Discretionary (gap-escape) changes go through incentive + P1; route changes bypass them.
        let route_required = vehicle.cached_route.get(vehicle.route_idx + 1).copied() == Some(min_cell);
        if !route_required {
            // INCENTIVE: change only if the chosen lane is genuinely more open ahead (else the side
            // is equally congested -> changing is useless weaving -> WAIT). Surgical anti-weaving.
            if !lc_incentive_pass(min_cell, net, current_state) {
                return Ok(create_block_intention(source_cell_id, true));
            }
            // P1 stochastic lane change: commit a DISCRETIONARY change only with
            // probability change_p1; otherwise wait this step and retry.
            // Models hesitation and damps reactive oscillation (шашечка) without forbidding the change.
            if !lc_p1_pass(vehicle.change_p1) {
                return Ok(create_block_intention(source_cell_id, true));
            }
        }
        return Ok(VehicleIntention {
            intention_maneuver,
            intention_speed: 1,
            destination: None,
            confusion: None,
            intention_cell_id: min_cell,
            tail_intention_cells: vec![],
            intermediate_cells: Vec::with_capacity(0),
            tail_maneuver: TailIntentionManeuver::default(),
            should_stop: true,
        })
    }
    Ok(create_block_intention(source_cell_id, false))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::new_point;
    use crate::grid::cell::Cell;
    use crate::utils::test_grids::create_pretty_simple_grid;
    #[test]
    fn test_intention() {
        let net = create_pretty_simple_grid();

        // Case 1: basics
        let current_state: HashMap<CellID, VehicleID> = HashMap::from([(101, 1)]);
        let vehicle_1 = Vehicle::new(1)
            .with_cell(101)
            .with_speed(1)
            .with_speed_limit(1)
            .with_destination(7)
            .build();
        let intention = find_intention(&net, &current_state, &HashMap::new(), &vehicle_1, &LocalLogger::none()).unwrap();
        let correct_intention = VehicleIntention {
            intention_cell_id: 1,
            intention_speed: 1,
            intention_maneuver: LaneChangeType::NoChange,
            ..Default::default()
        };
        assert_eq!(intention, correct_intention);

        // Case 2: move with speed > 1
        let current_state: HashMap<CellID, VehicleID> = HashMap::from([(101, 1)]);
        let vehicle_1 = Vehicle::new(1)
            .with_cell(101)
            .with_speed(3)
            .with_speed_limit(3)
            .with_destination(7)
            .build();
        let intention = find_intention(&net, &current_state, &HashMap::new(), &vehicle_1, &LocalLogger::none()).unwrap();
        let correct_intention = VehicleIntention {
            intention_cell_id: 3,
            intention_speed: 3,
            intention_maneuver: LaneChangeType::NoChange,
            intermediate_cells: vec![1, 2],
            ..Default::default()
        };
        assert_eq!(intention, correct_intention);

        // Case 3: move with speed > 1, but intention speed will be less due maneuver
        // before last cell in path. Intention cell will be cell
        // before last cell in path due the same reason.
        let current_state: HashMap<CellID, VehicleID> = HashMap::from([(101, 1)]);
        let vehicle_1 = Vehicle::new(1)
            .with_cell(101)
            .with_speed(4)
            .with_destination(8)
            .build();
        let intention = find_intention(&net, &current_state, &HashMap::new(), &vehicle_1, &LocalLogger::none()).unwrap();
        let correct_intention = VehicleIntention {
            intention_cell_id: 3,
            intention_speed: 3,
            intention_maneuver: LaneChangeType::NoChange,
            intermediate_cells: vec![1, 2],
            ..Default::default()
        };
        assert_eq!(intention, correct_intention);

        // Case 4: vehicle could not move due other vehicle in front
        let current_state: HashMap<CellID, VehicleID> = HashMap::from([(101, 1), (1, 2)]);
        let vehicle_1 = Vehicle::new(1)
            .with_cell(101)
            .with_speed(3)
            .with_destination(7)
            .build();
        let intention = find_intention(&net, &current_state, &HashMap::new(), &vehicle_1, &LocalLogger::none()).unwrap();
        let correct_intention = VehicleIntention {
            intention_cell_id: 101,
            intention_speed: 0,
            intention_maneuver: LaneChangeType::Block,
            should_stop: true,
            ..Default::default()
        };
        assert_eq!(intention, correct_intention);

        // Case 5: vehicle could move but not that far and will decrease speed due the other vehicle in front
        let current_state: HashMap<CellID, VehicleID> = HashMap::from([(101, 1), (3, 2)]);
        let vehicle_1 = Vehicle::new(1)
            .with_cell(101)
            .with_speed(3)
            .with_destination(7)
            .build();
        let intention = find_intention(&net, &current_state, &HashMap::new(), &vehicle_1, &LocalLogger::none()).unwrap();
        let correct_intention = VehicleIntention {
            intention_cell_id: 2,
            intention_speed: 2,
            intention_maneuver: LaneChangeType::NoChange,
            intermediate_cells: vec![1],
            ..Default::default()
        };
        assert_eq!(intention, correct_intention);

        // Case 6: vehicle has speed more than is needed to reach destination (vehicle should slow down)
        let current_state: HashMap<CellID, VehicleID> = HashMap::from([(101, 1)]);
        let vehicle_1 = Vehicle::new(1)
            .with_cell(101)
            .with_speed(3)
            .with_destination(2)
            .build();
        let intention = find_intention(&net, &current_state, &HashMap::new(), &vehicle_1, &LocalLogger::none()).unwrap();
        let correct_intention = VehicleIntention {
            intention_cell_id: 2,
            intention_speed: 2,
            intention_maneuver: LaneChangeType::NoChange,
            intermediate_cells: vec![1],
            ..Default::default()
        };
        assert_eq!(intention, correct_intention);

        // Case 7: vehicle can reach destination
        let current_state: HashMap<CellID, VehicleID> = HashMap::from([(101, 1)]);
        let vehicle_1 = Vehicle::new(1)
            .with_cell(101)
            .with_speed(4)
            .with_destination(7)
            .build();
        let intention = find_intention(&net, &current_state, &HashMap::new(), &vehicle_1, &LocalLogger::none()).unwrap();
        let correct_intention = VehicleIntention {
            intention_cell_id: 7,
            intention_speed: 4,
            intention_maneuver: LaneChangeType::NoChange,
            intermediate_cells: vec![1, 2, 3],
            ..Default::default()
        };
        assert_eq!(intention, correct_intention);
    }
    #[test]
    fn test_alternate_intention() {
        let speed_limit = 4;
        let mut net = GridRoads::new();
        net.add_cell(
            Cell::new(1)
                .with_speed_limit(speed_limit)
                .with_forward_node(2)
                .with_point(new_point(0.0, 0.0, None))
                .build(),
        );
        net.add_cell(
            Cell::new(2)
                .with_speed_limit(speed_limit)
                .with_right_node(8) /* Maneuver allowed */
                .with_forward_node(3)
                .with_point(new_point(1.0, 0.0, None))
                .build(),
        );
        net.add_cell(
            Cell::new(3)
                .with_speed_limit(speed_limit)
                .with_forward_node(4)
                .with_point(new_point(2.0, 0.0, None))
                .build(),
        );
        net.add_cell(
            Cell::new(4)
                .with_speed_limit(speed_limit)
                .with_forward_node(5)
                .with_point(new_point(3.0, 0.0, None))
                .build(),
        );
        net.add_cell(
            Cell::new(5)
                .with_speed_limit(speed_limit)
                .with_forward_node(6)
                .with_point(new_point(4.0, 0.0, None))
                .build(),
        );
        net.add_cell(
            Cell::new(6)
                .with_speed_limit(speed_limit)
                .with_point(new_point(5.0, 0.0, None))
                .build(),
        );

        // Other lane
        net.add_cell(
            Cell::new(7)
                .with_speed_limit(speed_limit)
                .with_forward_node(8)
                .with_point(new_point(0.0, 1.0, None))
                .build(),
        );
        net.add_cell(
            Cell::new(8)
                .with_speed_limit(speed_limit)
                .with_forward_node(9)
                .with_point(new_point(1.0, 1.0, None))
                .build(),
        );
        net.add_cell(
            Cell::new(9)
                .with_speed_limit(speed_limit)
                .with_forward_node(10)
                .with_point(new_point(2.0, 1.0, None))
                .build(),
        );
        net.add_cell(
            Cell::new(10)
                .with_speed_limit(speed_limit)
                .with_forward_node(11)
                .with_point(new_point(3.0, 1.0, None))
                .build(),
        );
        net.add_cell(
            Cell::new(11)
                .with_speed_limit(speed_limit)
                .with_forward_node(12)
                .with_point(new_point(4.0, 1.0, None))
                .build(),
        );
        net.add_cell(
            Cell::new(12)
                .with_speed_limit(speed_limit)
                .with_point(new_point(5.0, 1.0, None))
                .build(),
        );

        // Unreachable
        net.add_cell(
            Cell::new(500)
                .with_speed_limit(speed_limit)
                .with_point(new_point(10.0, 10.0, None))
                .build(),
        );

        // Case 1: Vehicle tries to find path via right maneuver (because of vehicle in front) and founds it
        let source_cell = net.get_cell(&2).unwrap();
        let blocked_cell = net.get_cell(&3).unwrap();
        let dest_cell = net.get_cell(&12).unwrap();
        let mut vehicle = Vehicle::new(42)
            .with_cell(source_cell.get_id())
            .with_speed(3)
            .with_destination(dest_cell.get_id())
            .build();
        let blocking_vehicle = Vehicle::new(78).with_cell(blocked_cell.get_id()).build();

        let mut current_state: HashMap<CellID, VehicleID> = HashMap::new();
    current_state.insert(source_cell.get_id(), vehicle.id);
        current_state.insert(blocked_cell.get_id(), blocking_vehicle.id);

        let mut intentions: Intentions = Intentions::new();
    let collected_intention = find_alternate_intention(&net, &current_state, &HashMap::new(), &vehicle, true, 10, 0);
        assert!(collected_intention.is_ok());
        let unwrapped_intention = collected_intention.unwrap();
    vehicle.set_intention(unwrapped_intention);
    intentions.add_intention(&mut vehicle, IntentionType::Target);

        // Speed should be 1 for cases when the vehicle can move, and 0 for cases when it could not move
        let mut correct_vehicle = Vehicle::new(42)
            .with_cell(source_cell.get_id())
            .with_destination(dest_cell.get_id())
            .build();
        let mut correct_intention = VehicleIntention::default();
        correct_intention.intention_cell_id = source_cell.get_right_id();
        correct_intention.intention_maneuver = LaneChangeType::ChangeRight;
        correct_intention.intention_speed = 1;
        correct_vehicle.set_intention(correct_intention);
        let mut correct_intentions = Intentions::new();
        correct_intentions.add_intention(&mut correct_vehicle, IntentionType::Target);
        assert_eq!(
            correct_intentions.len(),
            intentions.len(),
            "Incorrect number of intentions"
        );
        for (i, _corr_int) in correct_intentions.iter() {
            assert_eq!(
                intentions.get(i).is_some(),
                true,
                "No intention for cell #{} in correct intentions. Intention: {:?}",
                i,
                intentions.get(i)
            );
        }
        for (i, _int) in intentions.iter() {
            assert_eq!(
                correct_intentions.get(i).is_some(),
                true,
                "No intention for cell #{} in found intentions. Correct intention: {:?}",
                i,
                correct_intentions.get(i)
            );
        }
        for (i, int) in intentions.iter() {
            assert_eq!(
                correct_intentions.get(i).unwrap().len(),
                int.len(),
                "Incorrect number of intentions for cell #{}",
                i
            );
            for (j, intention) in int.iter().enumerate() {
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j].int_type,
                    intention.int_type,
                    "Incorrect intention type for cell #{} at pos #{}",
                    i,
                    j
                );
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j].vehicle_id,
                    intention.vehicle_id,
                    "Incorrect vehicle ID for cell #{} at pos #{}",
                    i,
                    j
                );
            }
        }

        // Case 2: Vehicle tries to find path via right maneuver (because of vehicle in front) but cannot find it
        // Could be demonstrated by trying to find path from vehicle's cell to any isolated cell
        let source_cell = net.get_cell(&2).unwrap();
        let blocked_cell = net.get_cell(&3).unwrap();
        let dest_cell = net.get_cell(&500).unwrap();
        let mut vehicle = Vehicle::new(42)
            .with_cell(source_cell.get_id())
            .with_speed(3)
            .with_destination(dest_cell.get_id())
            .build();
        let blocking_vehicle = Vehicle::new(78).with_cell(blocked_cell.get_id()).build();

        let mut current_state: HashMap<CellID, VehicleID> = HashMap::new();
    current_state.insert(source_cell.get_id(), vehicle.id);
        current_state.insert(blocked_cell.get_id(), blocking_vehicle.id);

        let mut intentions = Intentions::new();
    let collected_intention = find_alternate_intention(&net, &current_state, &HashMap::new(), &vehicle, true, 10, 0);
        assert!(collected_intention.is_ok());
        let unwrapped_intention = collected_intention.unwrap();
    vehicle.set_intention(unwrapped_intention);
    intentions.add_intention(&mut vehicle, IntentionType::Target);

        // Speed should be 1 for cases when the vehicle can move, and 0 for cases when it could not move
        let mut correct_vehicle = Vehicle::new(42)
            .with_cell(source_cell.get_id())
            .with_destination(dest_cell.get_id())
            .build();
        let mut correct_intention = VehicleIntention::default();
        correct_intention.intention_cell_id = source_cell.get_right_id();
        correct_intention.intention_maneuver = LaneChangeType::ChangeRight;
        correct_intention.intention_speed = 1;
        let mut correct_intentions = Intentions::new();
        correct_vehicle.set_intention(correct_intention);
        correct_intentions.add_intention(&mut correct_vehicle, IntentionType::Target);
        assert_eq!(
            correct_intentions.len(),
            intentions.len(),
            "Incorrect number of intentions"
        );
        for (i, _corr_int) in correct_intentions.iter() {
            assert_eq!(
                intentions.get(i).is_some(),
                true,
                "No intention for cell #{} in correct intentions. Intention: {:?}",
                i,
                intentions.get(i)
            );
        }
        for (i, _int) in intentions.iter() {
            assert_eq!(
                correct_intentions.get(i).is_some(),
                true,
                "No intention for cell #{} in found intentions. Correct intention: {:?}",
                i,
                correct_intentions.get(i)
            );
        }
        for (i, int) in intentions.iter() {
            assert_eq!(
                correct_intentions.get(i).unwrap().len(),
                int.len(),
                "Incorrect number of intentions for cell #{}",
                i
            );
            for (j, intention) in int.iter().enumerate() {
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j].int_type,
                    intention.int_type,
                    "Incorrect intention type for cell #{} at pos #{}",
                    i,
                    j
                );
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j].vehicle_id,
                    intention.vehicle_id,
                    "Incorrect vehicle ID for cell #{} at pos #{}",
                    i,
                    j
                );
            }
        }

        // Case 3: Vehicle tries to find path via right maneuver (because of vehicle in front) but cannot find it
        // and there is also another vehicle in the right maneuver cell
        let source_cell = net.get_cell(&2).unwrap();
        let blocked_cell = net.get_cell(&3).unwrap();
        let dest_cell = net.get_cell(&500).unwrap();
        let mut vehicle = Vehicle::new(42)
            .with_cell(source_cell.get_id())
            .with_speed(3)
            .with_destination(dest_cell.get_id())
            .build();
        let blocking_vehicle = Vehicle::new(78).with_cell(blocked_cell.get_id()).build();
        let blocking_vehicle2 = Vehicle::new(4278)
            .with_cell(source_cell.get_right_id())
            .build();

        let mut current_state: HashMap<CellID, VehicleID> = HashMap::new();
    current_state.insert(source_cell.get_id(), vehicle.id);
        current_state.insert(blocked_cell.get_id(), blocking_vehicle.id);
        current_state.insert(source_cell.get_right_id(), blocking_vehicle2.id);

        let mut intentions = Intentions::new();
    let collected_intention = find_alternate_intention(&net, &current_state, &HashMap::new(), &vehicle, true, 10, 0);
        assert!(collected_intention.is_ok());
        let unwrapped_intention = collected_intention.unwrap();
    vehicle.set_intention(unwrapped_intention);
    intentions.add_intention(&mut vehicle, IntentionType::Target);

        // Speed should be 1 for cases when the vehicle can move, and 0 for cases when it could not move
        // Vehicle could not move either forward or right
        let mut correct_vehicle = Vehicle::new(42)
            .with_cell(source_cell.get_id())
            .with_destination(dest_cell.get_id())
            .build();
        let mut correct_intention = VehicleIntention::default();
        correct_intention.intention_cell_id = source_cell.get_id();
        correct_intention.intention_maneuver = LaneChangeType::Block;
        correct_intention.intention_speed = 0;
        let mut correct_intentions = Intentions::new();
        correct_vehicle.set_intention(correct_intention);
        correct_intentions.add_intention(&mut correct_vehicle, IntentionType::Target);
        assert_eq!(
            correct_intentions.len(),
            intentions.len(),
            "Incorrect number of intentions"
        );
        for (i, _corr_int) in correct_intentions.iter() {
            assert_eq!(
                intentions.get(i).is_some(),
                true,
                "No intention for cell #{} in correct intentions. Intention: {:?}",
                i,
                intentions.get(i)
            );
        }
        for (i, _int) in intentions.iter() {
            assert_eq!(
                correct_intentions.get(i).is_some(),
                true,
                "No intention for cell #{} in found intentions. Correct intention: {:?}",
                i,
                correct_intentions.get(i)
            );
        }
        for (i, int) in intentions.iter() {
            assert_eq!(
                correct_intentions.get(i).unwrap().len(),
                int.len(),
                "Incorrect number of intentions for cell #{}",
                i
            );
            for (j, intention) in int.iter().enumerate() {
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j].int_type,
                    intention.int_type,
                    "Incorrect intention type for cell #{} at pos #{}",
                    i,
                    j
                );
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j].vehicle_id,
                    intention.vehicle_id,
                    "Incorrect vehicle ID for cell #{} at pos #{}",
                    i,
                    j
                );
            }
        }
    }

    /// The forward-roll reachability guard must honour the SAME bound the per-tick reconnect
    /// uses (`reconnect_max_depth`), not a separate hard-coded constant. A roll whose rejoin
    /// point sits at BFS depth 2 is authorized only when the bound is >= 2; otherwise the
    /// guard would green-light a roll that next tick's reconnect (same bound) cannot follow,
    /// forcing a needless full A*. Fast paths (route continues straight / forward is the
    /// destination) bypass the BFS and ignore the bound entirely.
    #[test]
    fn test_forward_guard_uses_reconnect_depth_bound() {
        let sl = 3;
        let mut net = GridRoads::new();
        // Cached route 1 -> 2 -> 3. The vehicle sits at route[0]=1.
        net.add_cell(Cell::new(1).with_speed_limit(sl).with_forward_node(2).with_point(new_point(0.0, 0.0, None)).build());
        net.add_cell(Cell::new(2).with_speed_limit(sl).with_forward_node(3).with_point(new_point(1.0, 0.0, None)).build());
        net.add_cell(Cell::new(3).with_speed_limit(sl).with_point(new_point(2.0, 0.0, None)).build());
        // Off-route detour 10 -> 11 -> 2: rejoins the cache (cell 2) at BFS depth 2.
        net.add_cell(Cell::new(10).with_speed_limit(sl).with_forward_node(11).with_point(new_point(0.0, 1.0, None)).build());
        net.add_cell(Cell::new(11).with_speed_limit(sl).with_forward_node(2).with_point(new_point(1.0, 1.0, None)).build());

        let mut vehicle = Vehicle::new(1).with_cell(1).with_speed(3).build();
        vehicle.cached_route = vec![1, 2, 3];
        vehicle.route_idx = 0;

        // BFS path: rejoin is at depth 2, so the bound decides.
        assert!(!forward_keeps_reachable(&vehicle, 10, &net, 1), "bound 1 < rejoin depth 2 -> not authorized");
        assert!(forward_keeps_reachable(&vehicle, 10, &net, 2), "bound 2 == rejoin depth 2 -> authorized");

        // Fast paths ignore the bound: route continues straight onto 2, and 3 is the destination.
        assert!(forward_keeps_reachable(&vehicle, 2, &net, 0), "route continues straight onto forward -> always ok");
        assert!(forward_keeps_reachable(&vehicle, 3, &net, 0), "forward is the destination (last route cell) -> always ok");
    }

    /// Regression (lost-for-tailed): an off-cache vehicle whose only route to the destination
    /// needs a lane change that is momentarily on cooldown (tail mid-maneuver, modelled here by
    /// `timer_non_maneuvers > 0`) must NOT latch `confusion`. The maneuvers-disabled A* fails,
    /// but the destination is still reachable WITH a maneuver, so the vehicle rolls forward and
    /// stays un-confused (recoverable) instead of being written off as lost. Confusion must
    /// still latch when the destination is genuinely unreachable even with maneuvers.
    #[test]
    fn test_offcache_cooldown_reachable_via_maneuver_does_not_confuse() {
        let sl = 3;
        let mut net = GridRoads::new();
        // From cell 1 the ONLY route to 4 is the lane change 1 -(right)-> 3 -> 4.
        // Going straight (1 -> 2) is a dead-end, so a maneuvers-disabled A* finds no path.
        net.add_cell(Cell::new(1).with_speed_limit(sl).with_forward_node(2).with_right_node(3).with_point(new_point(0.0, 0.0, None)).build());
        net.add_cell(Cell::new(2).with_speed_limit(sl).with_point(new_point(1.0, 0.0, None)).build()); // dead-end
        net.add_cell(Cell::new(3).with_speed_limit(sl).with_forward_node(4).with_point(new_point(0.0, 1.0, None)).build());
        net.add_cell(Cell::new(4).with_speed_limit(sl).with_point(new_point(1.0, 1.0, None)).build()); // destination
        net.add_cell(Cell::new(500).with_speed_limit(sl).with_point(new_point(9.0, 9.0, None)).build()); // disconnected

        let state: HashMap<CellID, VehicleID> = HashMap::new();

        // Reachable-only-via-maneuver + maneuver on cooldown -> must NOT confuse.
        let mut v = Vehicle::new(1).with_cell(1).with_speed(3).with_destination(4).build();
        v.timer_non_maneuvers = 5; // tail still completing a previous maneuver -> maneuvers disabled
        let intention = find_intention(&net, &state, &HashMap::new(), &v, &LocalLogger::none()).unwrap();
        assert_ne!(intention.confusion, Some(true), "reachable-via-maneuver must not latch confusion (would be lost)");

        // Control: genuinely unreachable destination, maneuvers allowed -> confusion DOES latch.
        let v = Vehicle::new(2).with_cell(1).with_speed(3).with_destination(500).build();
        let intention = find_intention(&net, &state, &HashMap::new(), &v, &LocalLogger::none()).unwrap();
        assert_eq!(intention.confusion, Some(true), "truly unreachable destination still latches confusion");
    }
}
