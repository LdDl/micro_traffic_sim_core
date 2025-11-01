use super::{process_no_route_found, process_path, NoRouteError};
use crate::behaviour::BehaviourType;
use crate::agents::{
    TailIntentionManeuver, Vehicle, VehicleRef, VehicleError, VehicleID, VehicleIntention,
};
use crate::grid::cell::CellState;
use crate::maneuver::LaneChangeType;
use crate::grid::{cell::CellID, cell::Cell, road_network::GridRoads};
use crate::intentions::{intention_type::IntentionType, Intentions};
use crate::shortest_path;
use crate::shortest_path::router::{shortest_path, path_no_goal};
use crate::shortest_path::router::AStarError;
use crate::verbose::*;
use indexmap::IndexMap;
use rand::random;
use std::collections::HashMap;
use std::f64::INFINITY;
use std::fmt;

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
    vehicles: &'b mut IndexMap<VehicleID, VehicleRef>,
    verbose: VerboseLevel,
) -> Result<Intentions, IntentionError> {
    let mut intentions = Intentions::new();
    if verbose.is_at_least(VerboseLevel::Main) {
        verbose.log_with_fields(
            EVENT_INTENTIONS_CREATE,
            "Collect intentions for vehicles",
            &[
                ("vehicles_num", &vehicles.len()),
            ]
        );
    }
    for (_, vehicle_ref) in vehicles.iter_mut() {
        if verbose.is_at_least(VerboseLevel::Additional) {
            verbose.log_with_fields(
                EVENT_INTENTION_VEHICLE,
                &format!("Processing vehicle {}", vehicle_ref.borrow().id),
                &[
                    ("vehicle_id", &vehicle_ref.borrow().id),
                    ("current_cell_id", &vehicle_ref.borrow().cell_id),
                    ("speed", &vehicle_ref.borrow().speed),
                    ("destination", &vehicle_ref.borrow().destination),
                ]
            );
        }
        let possible_intention = find_intention(net, current_state, &vehicle_ref.borrow(), verbose)?;
        if possible_intention.should_stop {
            let alternate_possible_intention = find_alternate_intention(net, current_state, &vehicle_ref.borrow())?;
            vehicle_ref.borrow_mut().set_intention(alternate_possible_intention);
            intentions.add_intention(vehicle_ref.clone(), IntentionType::Target);
            continue;
        }
        if verbose.is_at_least(VerboseLevel::Additional) {
            verbose.log_with_fields(
                EVENT_INTENTION_ADD,
                &format!("Adding intentions with vehicle {}", vehicle_ref.borrow().id),
                &[
                    ("vehicle_id", &vehicle_ref.borrow().id),
                    ("intention", &possible_intention),
                ]
            );
        }
        vehicle_ref.borrow_mut().set_intention(possible_intention);
        intentions.add_intention(vehicle_ref.clone(), IntentionType::Target);
    }
    Ok(intentions)
}

/// Computes the movement intention for a single vehicle.
///
/// Determines the best maneuver (forward, lane change, block, etc.)
/// and target cell, considering speed, acceleration, obstacles, and pathfinding.
pub fn find_intention<'a>(
    net: &'a GridRoads,
    current_state: &HashMap<CellID, VehicleID>,
    vehicle: &'a Vehicle,
    verbose: VerboseLevel,
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

    // Vehicle's speed should not be greater than speed limit
    let mut intention_speed = vehicle.speed.min(speed_limit);

    // Consider acceleration
    let mut speed_possible = intention_speed;
    let acceleration_allowed = vehicle.timer_non_acceleration <= 0;
    if acceleration_allowed {
        // Vehicle should could have (speed + 1) as possible speed untill it reaches speed limit
        speed_possible = (speed_possible + 1).min(speed_limit);
    }

    // Random slowdown
    let mut isSlowdown = false;
    let slowdown_allowed = vehicle.timer_non_slowdown <= 0;
    // tmp code:
    let slow_down_factor = vehicle.slow_down_factor;
    if slowdown_allowed && intention_speed > 0 && random::<f64>() < slow_down_factor {
        // @todo: consider to switch two lines below.
        speed_possible = intention_speed;
        intention_speed = (intention_speed - 1).max(0);
        isSlowdown = true;
    }

    // Considering that vehicle always wants to accelerate:
    let observe_distance = speed_possible + vehicle.min_safe_distance;

    // Check if maneuvers are allowed (they could be prohibeted due the vehicle's tail is not done previous maneuver yet)
    let maneuvers_allowed = vehicle.timer_non_maneuvers <= 0
        && tail_maneuver.intention_maneuver != LaneChangeType::ChangeRight
        && tail_maneuver.intention_maneuver != LaneChangeType::ChangeLeft;

    let mut destination: Option<CellID> = None;
    let mut confusion: Option<bool> = None;

    // println!(
    //     "Vehicle {} at cell {} with speed {} (possible {}) towards {} with observe distance {} and slow factor {}: {}",
    //     vehicle.id,
    //     source_cell.get_id(),
    //     intention_speed,
    //     speed_possible,
    //     vehicle.destination,
    //     observe_distance,
    //     slow_down_factor,
    //     if isSlowdown { " (slowdown)" } else { "" }
    // );

    let mut path = match vehicle.destination {
        // Handle case when vehicle has no destination,H
        // therefore it should be considered as keep going where possible
        dest if dest < 0 => {
            // println!(
            //     "  -> Vehicle {} has no destination, finding path without goal",
            //     vehicle.id
            // );
            match path_no_goal(
                source_cell,
                net,
                maneuvers_allowed,
                observe_distance + 1,
            ) {
                Ok(path) => path,
                Err(e) => return Err(IntentionError::NoPathFound(e)),
            }
        },
        _ => {
            let target_cell = net
                .get_cell(&vehicle.destination)
                .ok_or(IntentionError::NoTargetCell(vehicle.destination))?;
            match shortest_path(
                source_cell,
                target_cell,
                net,
                maneuvers_allowed,
                Some(observe_distance + 1),
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
                    let new_path = match process_no_route_found(source_cell, net) {
                        Ok(path) => path,
                        Err(e) => return Err(IntentionError::NoPathForNoRoute(e)),
                    };
                    destination = Some(new_path.vertices()[new_path.vertices().len() - 1].get_id());
                    intention_speed = 1;
                    speed_possible = intention_speed;
                    confusion = Some(true);
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
    // Otherwise just collect vehicles which can't move forward since not free cells ahead.
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
/// If the vehicle cannot move forward, tries left or right lane changes
/// and selects the best available option.
pub fn find_alternate_intention<'a>(
    net: &'a GridRoads,
    current_state: &HashMap<CellID, VehicleID>,
    vehicle: &'a Vehicle,
) -> Result<VehicleIntention, IntentionError> {
    let source_cell_id = vehicle.cell_id;
    let target_cell_id = vehicle.destination;

    let source_cell = net
        .get_cell(&source_cell_id)
        .ok_or(IntentionError::NoSourceCell(source_cell_id))?;

    let target_cell = net
        .get_cell(&target_cell_id)
        .ok_or(IntentionError::NoTargetCell(target_cell_id))?;

    let mut min_left_dist = INFINITY;
    let mut min_right_dist = INFINITY;

    // Check left maneuver
    let mut left_cell_id = source_cell.get_left_id();
    if left_cell_id > 0 {
        let left_cell = net
            .get_cell(&left_cell_id)
            .ok_or(IntentionError::NoLeftCell(vehicle.cell_id))?;

        // Check if possible maneuver can't be made
        let is_blocked = current_state
            .get(&left_cell_id)
            .map(|&id| id > 0)
            .unwrap_or(false);

        if !is_blocked && left_cell.get_state() == CellState::Free {
            match shortest_path(left_cell, target_cell, net, true, Some(vehicle.speed)) {
                Ok(path) => {
                    let cost = path.cost();
                    min_left_dist = cost + source_cell.distance_to(left_cell);
                }
                Err(e)
                    if e != shortest_path::router::AStarError::NoPathFound {
                        start_id: left_cell_id,
                        end_id: target_cell_id,
                    } =>
                {
                    return Err(IntentionError::LeftPathFind(left_cell_id))
                }
                Err(_) => {
                    min_left_dist = INFINITY;
                }
            }
        } else {
            left_cell_id = -1;
        }
    }

    // Check right maneuver
    let mut right_cell_id = source_cell.get_right_id();
    if right_cell_id > 0 {
        let right_cell = net
            .get_cell(&right_cell_id)
            .ok_or(IntentionError::NoRightCell(vehicle.cell_id))?;

        let is_blocked = current_state
            .get(&right_cell_id)
            .map(|&id| id > 0)
            .unwrap_or(false);

        if !is_blocked && right_cell.get_state() == CellState::Free {
            match shortest_path(right_cell, target_cell, net, true, Some(vehicle.speed)) {
                Ok(path) => {
                    let cost = path.cost();
                    min_right_dist = cost + source_cell.distance_to(right_cell);
                }
                Err(e)
                    if e != shortest_path::router::AStarError::NoPathFound {
                        start_id: right_cell_id,
                        end_id: target_cell_id,
                    } =>
                {
                    return Err(IntentionError::RightPathFind(right_cell_id))
                }
                Err(_) => {
                    min_right_dist = INFINITY;
                }
            }
        } else {
            right_cell_id = -1;
        }
    }

    // Choose best maneuver
    let mut min_cell: CellID;
    let mut intention_maneuver: LaneChangeType;
    if UNDEFINED_MANEUVER == LaneChangeType::ChangeRight {
        min_cell = right_cell_id;
        intention_maneuver = LaneChangeType::ChangeRight;
        if min_left_dist < min_right_dist {
            min_cell = left_cell_id;
            intention_maneuver = LaneChangeType::ChangeLeft;
        }
    } else {
        min_cell = left_cell_id;
        intention_maneuver = LaneChangeType::ChangeLeft;
        if min_right_dist < min_left_dist {
            min_cell = right_cell_id;
            intention_maneuver = LaneChangeType::ChangeRight;
        }
    }

    // Apply the chosen maneuver
    if min_cell > 0 {
        let result = VehicleIntention {
            intention_maneuver: intention_maneuver,
            intention_speed: 1,
            destination: None,
            confusion: None,
            intention_cell_id: min_cell,
            tail_intention_cells: vec![],
            intermediate_cells: Vec::with_capacity(0),
            tail_maneuver: TailIntentionManeuver::default(),
            should_stop: true,
        };
        return Ok(result);
    }
    let result = VehicleIntention {
        intention_maneuver: LaneChangeType::Block,
        intention_speed: 0,
        destination: None,
        confusion: None,
        intention_cell_id: source_cell_id,
        tail_intention_cells: vec![],
        intermediate_cells: Vec::with_capacity(0),
        tail_maneuver: TailIntentionManeuver::default(),
        should_stop: false,
    };

    Ok(result)
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
            .with_destination(7)
            .build();
        let intention = find_intention(&net, &current_state, &vehicle_1, VerboseLevel::None).unwrap();
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
            .with_destination(7)
            .build();
        let intention = find_intention(&net, &current_state, &vehicle_1, VerboseLevel::None).unwrap();
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
        let intention = find_intention(&net, &current_state, &vehicle_1, VerboseLevel::None).unwrap();
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
        let intention = find_intention(&net, &current_state, &vehicle_1, VerboseLevel::None).unwrap();
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
        let intention = find_intention(&net, &current_state, &vehicle_1, VerboseLevel::None).unwrap();
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
        let intention = find_intention(&net, &current_state, &vehicle_1, VerboseLevel::None).unwrap();
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
        let intention = find_intention(&net, &current_state, &vehicle_1, VerboseLevel::None).unwrap();
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
            .build_ref();
        let blocking_vehicle = Vehicle::new(78).with_cell(blocked_cell.get_id()).build();

        let mut current_state: HashMap<CellID, VehicleID> = HashMap::new();
        current_state.insert(source_cell.get_id(), vehicle.borrow().id);
        current_state.insert(blocked_cell.get_id(), blocking_vehicle.id);

        let mut intentions: Intentions = Intentions::new();
        let collected_intention = find_alternate_intention(&net, &current_state, &vehicle.borrow());
        assert!(collected_intention.is_ok());
        let unwrapped_intention = collected_intention.unwrap();
        vehicle.borrow_mut().set_intention(unwrapped_intention);
        intentions.add_intention(vehicle, IntentionType::Target);

        // Speed should be 1 for cases when the vehicle can move, and 0 for cases when it could not move
        let mut correct_vehicle = Vehicle::new(42)
            .with_cell(source_cell.get_id())
            .with_destination(dest_cell.get_id())
            .build_ref();
        let mut correct_intention = VehicleIntention::default();
        correct_intention.intention_cell_id = source_cell.get_right_id();
        correct_intention.intention_maneuver = LaneChangeType::ChangeRight;
        correct_intention.intention_speed = 1;
        correct_vehicle.borrow_mut().set_intention(correct_intention);
        let mut correct_intentions = Intentions::new();
        correct_intentions.add_intention(correct_vehicle, IntentionType::Target);
        assert_eq!(
            correct_intentions.len(),
            intentions.len(),
            "Incorrect number of intentions"
        );
        for (i, corr_int) in correct_intentions.iter() {
            assert_eq!(
                intentions.get(i).is_some(),
                true,
                "No intention for cell #{} in correct intentions. Intention: {:?}",
                i,
                intentions.get(i)
            );
        }
        for (i, int) in intentions.iter() {
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
                    correct_intentions.get(i).unwrap()[j].vehicle.borrow().id,
                    intention.vehicle.borrow().id,
                    "Incorrect vehicle ID for cell #{} at pos #{}",
                    i,
                    j
                );
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j]
                        .vehicle
                        .borrow()
                        .intention
                        .intention_speed,
                    intention.vehicle.borrow().intention.intention_speed,
                    "Incorrect vehicle speed for cell #{} at pos #{}",
                    i,
                    j
                );
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j]
                        .vehicle
                        .borrow()
                        .intention
                        .intention_maneuver,
                    intention.vehicle.borrow().intention.intention_maneuver,
                    "Incorrect vehicle maneuver for cell #{} at pos #{}",
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
            .build_ref();
        let blocking_vehicle = Vehicle::new(78).with_cell(blocked_cell.get_id()).build();

        let mut current_state: HashMap<CellID, VehicleID> = HashMap::new();
        current_state.insert(source_cell.get_id(), vehicle.borrow().id);
        current_state.insert(blocked_cell.get_id(), blocking_vehicle.id);

        let mut intentions = Intentions::new();
        let collected_intention = find_alternate_intention(&net, &current_state, &vehicle.borrow());
        assert!(collected_intention.is_ok());
        let unwrapped_intention = collected_intention.unwrap();
        vehicle.borrow_mut().set_intention(unwrapped_intention);
        intentions.add_intention(vehicle, IntentionType::Target);

        // Speed should be 1 for cases when the vehicle can move, and 0 for cases when it could not move
        let mut correct_vehicle = Vehicle::new(42)
            .with_cell(source_cell.get_id())
            .with_destination(dest_cell.get_id())
            .build_ref();
        let mut correct_intention = VehicleIntention::default();
        correct_intention.intention_cell_id = source_cell.get_right_id();
        correct_intention.intention_maneuver = LaneChangeType::ChangeRight;
        correct_intention.intention_speed = 1;
        let mut correct_intentions = Intentions::new();
        correct_vehicle.borrow_mut().set_intention(correct_intention);
        correct_intentions.add_intention(correct_vehicle, IntentionType::Target);
        assert_eq!(
            correct_intentions.len(),
            intentions.len(),
            "Incorrect number of intentions"
        );
        for (i, corr_int) in correct_intentions.iter() {
            assert_eq!(
                intentions.get(i).is_some(),
                true,
                "No intention for cell #{} in correct intentions. Intention: {:?}",
                i,
                intentions.get(i)
            );
        }
        for (i, int) in intentions.iter() {
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
                    correct_intentions.get(i).unwrap()[j].vehicle.borrow().id,
                    intention.vehicle.borrow().id,
                    "Incorrect vehicle ID for cell #{} at pos #{}",
                    i,
                    j
                );
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j]
                        .vehicle
                        .borrow()
                        .intention
                        .intention_speed,
                    intention.vehicle.borrow().intention.intention_speed,
                    "Incorrect vehicle speed for cell #{} at pos #{}",
                    i,
                    j
                );
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j]
                        .vehicle
                        .borrow()
                        .intention
                        .intention_maneuver,
                    intention.vehicle.borrow().intention.intention_maneuver,
                    "Incorrect vehicle maneuver for cell #{} at pos #{}",
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
            .build_ref();
        let blocking_vehicle = Vehicle::new(78).with_cell(blocked_cell.get_id()).build();
        let blocking_vehicle2 = Vehicle::new(4278)
            .with_cell(source_cell.get_right_id())
            .build();

        let mut current_state: HashMap<CellID, VehicleID> = HashMap::new();
        current_state.insert(source_cell.get_id(), vehicle.borrow().id);
        current_state.insert(blocked_cell.get_id(), blocking_vehicle.id);
        current_state.insert(source_cell.get_right_id(), blocking_vehicle2.id);

        let mut intentions = Intentions::new();
        let collected_intention = find_alternate_intention(&net, &current_state, &vehicle.borrow());
        assert!(collected_intention.is_ok());
        let unwrapped_intention = collected_intention.unwrap();
        vehicle.borrow_mut().set_intention(unwrapped_intention);
        intentions.add_intention(vehicle, IntentionType::Target);

        // Speed should be 1 for cases when the vehicle can move, and 0 for cases when it could not move
        // Vehicle could not move either forward or right
        let mut correct_vehicle = Vehicle::new(42)
            .with_cell(source_cell.get_id())
            .with_destination(dest_cell.get_id())
            .build_ref();
        let mut correct_intention = VehicleIntention::default();
        correct_intention.intention_cell_id = source_cell.get_id();
        correct_intention.intention_maneuver = LaneChangeType::Block;
        correct_intention.intention_speed = 0;
        let mut correct_intentions = Intentions::new();
        correct_vehicle.borrow_mut().set_intention(correct_intention);
        correct_intentions.add_intention(correct_vehicle, IntentionType::Target);
        assert_eq!(
            correct_intentions.len(),
            intentions.len(),
            "Incorrect number of intentions"
        );
        for (i, corr_int) in correct_intentions.iter() {
            assert_eq!(
                intentions.get(i).is_some(),
                true,
                "No intention for cell #{} in correct intentions. Intention: {:?}",
                i,
                intentions.get(i)
            );
        }
        for (i, int) in intentions.iter() {
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
                    correct_intentions.get(i).unwrap()[j].vehicle.borrow().id,
                    intention.vehicle.borrow().id,
                    "Incorrect vehicle ID for cell #{} at pos #{}",
                    i,
                    j
                );
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j]
                        .vehicle
                        .borrow()
                        .intention
                        .intention_speed,
                    intention.vehicle.borrow().intention.intention_speed,
                    "Incorrect vehicle speed for cell #{} at pos #{}",
                    i,
                    j
                );
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j]
                        .vehicle
                        .borrow()
                        .intention
                        .intention_maneuver,
                    intention.vehicle.borrow().intention.intention_maneuver,
                    "Incorrect vehicle maneuver for cell #{} at pos #{}",
                    i,
                    j
                );
            }
        }
    }
}
