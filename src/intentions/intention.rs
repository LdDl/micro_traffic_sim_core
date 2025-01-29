use crate::agents::{Vehicle, VehicleID};
use crate::grid::cell::CellState;
use crate::grid::lane_change_type::LaneChangeType;
use crate::grid::{cell::CellID, road_network::GridRoads};
use crate::intentions::{intention_type::IntentionType, Intentions};
use crate::shortest_path;
use crate::shortest_path::router::shortest_path;
use indexmap::IndexMap;
use std::f64::INFINITY;
use std::fmt;

#[derive(Debug)]
pub enum IntentionError {
    NoSourceCell(CellID),
    NoTargetCell(CellID),
    NoLeftCell(CellID),
    NoRightCell(CellID),
    LeftPathFind(CellID),
    RightPathFind(CellID),
}

struct IntentionInfo<'a> {
    cell_id: CellID,
    vehicle: &'a mut Vehicle,
    intention_type: IntentionType,
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
        }
    }
}

/* Change it according to right-hand or left-hand traffic (driving side) */
/* @todo: should be an argument in further  */
const undefined_maneuver: LaneChangeType = LaneChangeType::ChangeRight;

pub fn find_alternate_intention<'a>(
    net: &'a GridRoads,
    current_state: &IndexMap<CellID, VehicleID>,
    vehicle: &'a mut Vehicle,
) -> Result<Vec<IntentionInfo<'a>>, IntentionError> {
    let source_cell_id = vehicle.cell_id;
    let target_cell_id = vehicle.destination;
    
    let mut collected_intentions: Vec<IntentionInfo> = Vec::new();

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
                        start_id: 1,
                        end_id: 2,
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
                        start_id: 1,
                        end_id: 2,
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
    if undefined_maneuver == LaneChangeType::ChangeRight {
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
    vehicle.intention_maneuver = intention_maneuver;

    // Apply the chosen maneuver
    if min_cell > 0 {
        vehicle.speed = 1;
        // Old code: borrowing issues
        // intentions.add_intention(min_cell, vehicle, IntentionType::Target);
        collected_intentions.push(IntentionInfo {
            cell_id: min_cell,
            vehicle: vehicle,
            intention_type: IntentionType::Target,
        });
    } else {
        vehicle.intention_maneuver = LaneChangeType::Block;
        vehicle.speed = 0;
        // Old code: borrowing issues
        // intentions.add_intention(source_cell_id, vehicle, IntentionType::Target);
        collected_intentions.push(IntentionInfo {
            cell_id: source_cell_id,
            vehicle: vehicle,
            intention_type: IntentionType::Target,
        });
    }

    Ok(collected_intentions)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::new_point;
    use crate::grid::cell::Cell;
    #[test]
    fn test_alternate_intention() {
        // Golang code migration
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

        let mut current_state: IndexMap<CellID, VehicleID> = IndexMap::new();
        current_state.insert(source_cell.get_id(), vehicle.id);
        current_state.insert(blocked_cell.get_id(), blocking_vehicle.id);

        let mut intentions = Intentions::new();
        let collected_intentions = find_alternate_intention(&net, &current_state, &mut vehicle);
        assert!(collected_intentions.is_ok());
        for intention in collected_intentions.unwrap() {
            intentions.add_intention(intention.cell_id, intention.vehicle, intention.intention_type);
        }

        // Speed should be 1 for cases when the vehicle can move, and 0 for cases when it could not move
        let correct_speed = 1;
        let mut correct_vehicle = Vehicle::new(42)
            .with_cell(source_cell.get_id())
            .with_speed(correct_speed)
            .with_destination(dest_cell.get_id())
            .with_intention_maneuver(LaneChangeType::ChangeRight)
            .build();
        let mut correct_intentions = Intentions::new();
        correct_intentions.add_intention(
            source_cell.get_right_id(),
            &mut correct_vehicle,
            IntentionType::Target,
        );
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
                    correct_intentions.get(i).unwrap()[j].vehicle.unwrap().id,
                    intention.vehicle.unwrap().id,
                    "Incorrect vehicle ID for cell #{} at pos #{}",
                    i,
                    j
                );
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j].vehicle.unwrap().speed,
                    intention.vehicle.unwrap().speed,
                    "Incorrect vehicle speed for cell #{} at pos #{}",
                    i,
                    j
                );
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j]
                        .vehicle
                        .unwrap()
                        .intention_maneuver,
                    intention.vehicle.unwrap().intention_maneuver,
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
            .build();
        let blocking_vehicle = Vehicle::new(78).with_cell(blocked_cell.get_id()).build();

        let mut current_state: IndexMap<CellID, VehicleID> = IndexMap::new();
        current_state.insert(source_cell.get_id(), vehicle.id);
        current_state.insert(blocked_cell.get_id(), blocking_vehicle.id);

        let mut intentions = Intentions::new();
        let collected_intentions = find_alternate_intention(&net, &current_state, &mut vehicle);
        assert!(collected_intentions.is_ok());
        for intention in collected_intentions.unwrap() {
            intentions.add_intention(intention.cell_id, intention.vehicle, intention.intention_type);
        }

        // Speed should be 1 for cases when the vehicle can move, and 0 for cases when it could not move
        let correct_speed = 1;
        let mut correct_vehicle = Vehicle::new(42)
            .with_cell(source_cell.get_id())
            .with_speed(correct_speed)
            .with_destination(dest_cell.get_id())
            .with_intention_maneuver(LaneChangeType::ChangeRight)
            .build();
        let mut correct_intentions = Intentions::new();
        correct_intentions.add_intention(
            source_cell.get_right_id(),
            &mut correct_vehicle,
            IntentionType::Target,
        );
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
                    correct_intentions.get(i).unwrap()[j].vehicle.unwrap().id,
                    intention.vehicle.unwrap().id,
                    "Incorrect vehicle ID for cell #{} at pos #{}",
                    i,
                    j
                );
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j].vehicle.unwrap().speed,
                    intention.vehicle.unwrap().speed,
                    "Incorrect vehicle speed for cell #{} at pos #{}",
                    i,
                    j
                );
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j]
                        .vehicle
                        .unwrap()
                        .intention_maneuver,
                    intention.vehicle.unwrap().intention_maneuver,
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
            .build();
        let blocking_vehicle = Vehicle::new(78).with_cell(blocked_cell.get_id()).build();
        let blocking_vehicle2 = Vehicle::new(4278)
            .with_cell(source_cell.get_right_id())
            .build();

        let mut current_state: IndexMap<CellID, VehicleID> = IndexMap::new();
        current_state.insert(source_cell.get_id(), vehicle.id);
        current_state.insert(blocked_cell.get_id(), blocking_vehicle.id);
        current_state.insert(source_cell.get_right_id(), blocking_vehicle2.id);

        let mut intentions = Intentions::new();
        let collected_intentions = find_alternate_intention(&net, &current_state, &mut vehicle);
        assert!(collected_intentions.is_ok());
        for intention in collected_intentions.unwrap() {
            intentions.add_intention(intention.cell_id, intention.vehicle, intention.intention_type);
        }

        // Speed should be 1 for cases when the vehicle can move, and 0 for cases when it could not move
        let correct_speed = 0; // Vehicle could not move either forward or right
        let mut correct_vehicle = Vehicle::new(42)
            .with_cell(source_cell.get_id())
            .with_speed(correct_speed)
            .with_destination(dest_cell.get_id())
            .with_intention_maneuver(LaneChangeType::Block)
            .build();
        let mut correct_intentions = Intentions::new();
        correct_intentions.add_intention(
            source_cell.get_id(),
            &mut correct_vehicle,
            IntentionType::Target,
        );
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
                    correct_intentions.get(i).unwrap()[j].vehicle.unwrap().id,
                    intention.vehicle.unwrap().id,
                    "Incorrect vehicle ID for cell #{} at pos #{}",
                    i,
                    j
                );
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j].vehicle.unwrap().speed,
                    intention.vehicle.unwrap().speed,
                    "Incorrect vehicle speed for cell #{} at pos #{}",
                    i,
                    j
                );
                assert_eq!(
                    correct_intentions.get(i).unwrap()[j]
                        .vehicle
                        .unwrap()
                        .intention_maneuver,
                    intention.vehicle.unwrap().intention_maneuver,
                    "Incorrect vehicle maneuver for cell #{} at pos #{}",
                    i,
                    j
                );
            }
        }
    }
}
