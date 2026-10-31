use crate::maneuver::LaneChangeType;
use crate::{
    agents::VehicleID,
    grid::{
        cell::{Cell, CellID, CellState},
    },
    shortest_path::path::Path,
};
use std::collections::HashMap;

/// Contains details about a vehicle's observable movement along a path.
///
/// Includes the desired maneuver, last cell state, trimmed path,
/// and flags for obstacles, speed limits, and maneuvers.
#[derive(Debug, Clone)]
pub struct ObservablePath<'a> {
    /// Desired lane change maneuver
    pub wanted_maneuver: LaneChangeType,
    /// State of the last cell in path
    pub last_cell_state: CellState,
    /// Path vertices (cells) after trimming
    pub trimmed_path: Vec<&'a Cell>,
    /// Indicates if another vehicle is on the path
    pub has_vehicle_on_path: bool,
    /// Indicates if speed limit was reached
    pub speed_limit_reached: bool,
    /// Indicates if movement was stopped due to a maneuver
    pub stopped_on_maneuver: bool,
    /// Indicates if movement was stopped due to speed restrictions
    pub stopped_speed_possible: bool,
    /// Cell occupied by another vehicle where the scan stopped, -1 when no vehicle blocked the path
    pub blocking_cell_id: CellID,
}

/// Analyzes a path and determines how far a vehicle can move.
///
/// Trims the path based on speed, obstacles, maneuvers, and cell states.
/// Returns details about the possible movement and any stopping conditions.
///
/// # Arguments
/// * `shortest_path` - Reference to path to process
/// * `speed_possible` - Maximum possible speed
/// * `current_state` - Current occupancy state mapping cells to vehicles
///
/// # Returns
/// Returns ObservablePath containing movement analysis
///
/// # Examples
///
/// ```
/// use micro_traffic_sim_core::{
///    grid::{cell::{Cell, CellState}},
///    shortest_path::path::Path,
///    maneuver::LaneChangeType,
///    intentions::{process_path, ObservablePath},
/// };
/// use std::collections::HashMap;
/// let cell1 = Cell::new(1).with_speed_limit(2).build();
/// let cell2 = Cell::new(2).with_speed_limit(2).build();
/// let cell3 = Cell::new(3).with_speed_limit(2).build();
/// let cell4 = Cell::new(4).with_speed_limit(2).build();
///
/// let mut path = Path::new(
///     vec![&cell1, &cell2, &cell3, &cell4],
///     vec![LaneChangeType::NoChange, LaneChangeType::NoChange, LaneChangeType::ChangeLeft],
///     10.0
/// );
///
/// let mut current_state = HashMap::new();
/// let observable_path = process_path(&mut path, 2, 4, &current_state);
///
/// println!("Observable path without obstacles: {:?}", observable_path); // Should print 3 cells forward
///
/// current_state.insert(cell3.get_id(), 1);
/// let observable_path = process_path(&mut path, 2, 4, &current_state);
///
/// println!("Observable path with obstacles: {:?}", observable_path); // Should print 1 cell forward
/// ```
pub fn process_path<'a>(
    shortest_path: &'a mut Path,
    speed_possible: i32,
    destination: CellID,
    current_state: &HashMap<CellID, VehicleID>,
) -> ObservablePath<'a> {
    process_path_for_vehicle(shortest_path, speed_possible, destination, current_state, None)
}

/// Same as `process_path`, but cells claimed by the vehicle itself (its front standstill buffer)
/// do not block it.
///
/// # Arguments
/// * `shortest_path` - Reference to path to process
/// * `speed_possible` - Maximum possible speed
/// * `destination` - Vehicle destination cell
/// * `current_state` - Current occupancy state mapping cells to vehicles (bodies and buffers)
/// * `self_id` - The vehicle whose own claimed cells are passable, None to treat every claim as an obstacle
pub fn process_path_for_vehicle<'a>(
    shortest_path: &'a mut Path,
    speed_possible: i32,
    destination: CellID,
    current_state: &HashMap<CellID, VehicleID>,
    self_id: Option<VehicleID>,
) -> ObservablePath<'a> {
    // Remove first cell from path since it's vehicle's position
    shortest_path.vertices_mut().remove(0);
    // Handle edge case where path is empty or speed is 0
    if shortest_path.vertices().is_empty() || speed_possible == 0 {
        return ObservablePath {
            wanted_maneuver: LaneChangeType::NoChange,
            last_cell_state: CellState::Free,
            trimmed_path: vec![],
            has_vehicle_on_path: false,
            speed_limit_reached: false,
            stopped_on_maneuver: false,
            stopped_speed_possible: false,
            blocking_cell_id: -1,
        };
    }
    let speed_limit = (speed_possible as usize - 1).min(shortest_path.vertices().len() - 1);
    // Remove last cell if path has more than 1 cell and last cell in possible path (untill speed
    // limit would has been reached) is not vehicle's destination
    if shortest_path.vertices().len() > 1
        && shortest_path.vertices()[speed_limit].get_id() != destination
    {
        // @todo: is this neccessary?
        // shortest_path.vertices_mut().pop();
        // shortest_path.maneuvers_mut().pop();
    }

    let maneuvers = shortest_path.maneuvers();
    let vertices = shortest_path.vertices();
    let mut wanted_maneuver = LaneChangeType::NoChange;
    let mut last_cell_state = CellState::Free;

    // Find max number of cells vehicle can move forward
    let mut success_forward_movement = 0;
    let mut path_speed = speed_possible;

    let mut has_vehicle_on_path = false;
    let mut speed_limit_reached = false;
    let mut stopped_on_maneuver = false;
    let stopped_speed_possible = false;
    let mut blocking_cell_id: CellID = -1;

    for (i, cell) in vertices.iter().enumerate() {
        let maneuver = maneuvers[i];
        if maneuver != LaneChangeType::NoChange {
            if success_forward_movement == 0 {
                wanted_maneuver = maneuver;
            }
            stopped_on_maneuver = true;
            break;
        }

        // Check if cell is occupied by another vehicle. Cells claimed by the vehicle itself
        // (its front standstill buffer) do not block it.
        if let Some(&occupant) = current_state.get(&cell.get_id()) {
            if Some(occupant) != self_id {
                has_vehicle_on_path = true;
                blocking_cell_id = cell.get_id();
                break;
            }
        }

        // Check traffic light state
        if cell.get_state() != CellState::Free {
            last_cell_state = cell.get_state();
            break;
        }

        // Keep the longest prefix whose displacement obeys every traversed cell limit.
        if path_speed > cell.get_speed_limit() {
            speed_limit_reached = true;
            path_speed = cell.get_speed_limit().max(0);
            if path_speed <= success_forward_movement {
                break;
            }
        }

        success_forward_movement += 1;
        if success_forward_movement >= path_speed {
            break;
        }
    }
    ObservablePath {
        wanted_maneuver,
        last_cell_state,
        trimmed_path: vertices[..success_forward_movement as usize].to_vec(),
        has_vehicle_on_path,
        speed_limit_reached,
        stopped_on_maneuver,
        stopped_speed_possible,
        blocking_cell_id,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_observable_path() {
        let speed_limit = 4;
        let cell1 = Cell::new(1).with_speed_limit(speed_limit).build();
        let cell2 = Cell::new(2).with_speed_limit(speed_limit).build();
        let mut cell3 = Cell::new(3).with_speed_limit(speed_limit).build();
        let cell4 = Cell::new(4).with_speed_limit(speed_limit).build();
        let cell5 = Cell::new(5).with_speed_limit(speed_limit).build();

        let mut path = Path::new(
            vec![&cell1, &cell2, &cell3, &cell4, &cell5],
            vec![
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::ChangeLeft,
            ],
            10.0,
        );

        let mut current_state = HashMap::new();
        let observable_path = process_path(&mut path, speed_limit, cell5.get_id(), &current_state);

        let correct_path = ObservablePath {
            wanted_maneuver: LaneChangeType::NoChange,
            last_cell_state: CellState::Free,
            trimmed_path: vec![&cell2, &cell3, &cell4],
            has_vehicle_on_path: false,
            speed_limit_reached: false,
            stopped_on_maneuver: false,
            stopped_speed_possible: false,
            blocking_cell_id: -1,
        };

        assert_eq!(
            observable_path.wanted_maneuver, correct_path.wanted_maneuver,
            "Incorrect maneuver for observable path"
        );

        assert_eq!(
            observable_path.last_cell_state, correct_path.last_cell_state,
            "Incorrect last cell state for observable path"
        );

        assert_eq!(
            observable_path.trimmed_path.len(),
            correct_path.trimmed_path.len(),
            "Incorrect number of cells vehicle can move forward"
        );

        for i in 0..observable_path.trimmed_path.len() {
            assert_eq!(
                observable_path.trimmed_path[i].get_id(),
                correct_path.trimmed_path[i].get_id(),
                "Incorrect cell at pos #{} in observable path",
                i
            );
        }

        // Add vehicle as obstacle
        current_state.insert(cell4.get_id(), 1);
        // Update path since it has been mutated in process_path function
        path = Path::new(
            vec![&cell1, &cell2, &cell3, &cell4, &cell5],
            vec![
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::ChangeLeft,
            ],
            10.0,
        );
        let observable_path = process_path(&mut path, speed_limit, cell5.get_id(), &current_state);

        let correct_path = ObservablePath {
            wanted_maneuver: LaneChangeType::NoChange,
            last_cell_state: CellState::Free,
            trimmed_path: vec![&cell2, &cell3],
            has_vehicle_on_path: false,
            speed_limit_reached: false,
            stopped_on_maneuver: false,
            stopped_speed_possible: false,
            blocking_cell_id: -1,
        };

        assert_eq!(
            observable_path.wanted_maneuver, correct_path.wanted_maneuver,
            "Incorrect maneuver for observable path"
        );

        assert_eq!(
            observable_path.last_cell_state, correct_path.last_cell_state,
            "Incorrect last cell state for observable path"
        );

        assert_eq!(
            observable_path.trimmed_path.len(),
            correct_path.trimmed_path.len(),
            "Incorrect number of cells vehicle can move forward"
        );

        for i in 0..observable_path.trimmed_path.len() {
            assert_eq!(
                observable_path.trimmed_path[i].get_id(),
                correct_path.trimmed_path[i].get_id(),
                "Incorrect cell at pos #{} in observable path",
                i
            );
        }
        // Clean current states
        current_state.remove(&cell4.get_id());

        // Add maneuver in the middle of the path
        path = Path::new(
            vec![&cell1, &cell2, &cell3, &cell4, &cell5],
            vec![
                LaneChangeType::NoChange,
                LaneChangeType::ChangeLeft,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
            ],
            10.0,
        );
        let observable_path = process_path(&mut path, speed_limit, cell5.get_id(), &current_state);

        let correct_path = ObservablePath {
            wanted_maneuver: LaneChangeType::NoChange,
            last_cell_state: CellState::Free,
            trimmed_path: vec![&cell2],
            has_vehicle_on_path: false,
            speed_limit_reached: false,
            stopped_on_maneuver: true,
            stopped_speed_possible: false,
            blocking_cell_id: -1,
        };

        assert_eq!(
            observable_path.wanted_maneuver, correct_path.wanted_maneuver,
            "Incorrect maneuver for observable path"
        );

        assert_eq!(
            observable_path.last_cell_state, correct_path.last_cell_state,
            "Incorrect last cell state for observable path"
        );

        assert_eq!(
            observable_path.trimmed_path.len(),
            correct_path.trimmed_path.len(),
            "Incorrect number of cells vehicle can move forward"
        );

        for i in 0..observable_path.trimmed_path.len() {
            assert_eq!(
                observable_path.trimmed_path[i].get_id(),
                correct_path.trimmed_path[i].get_id(),
                "Incorrect cell at pos #{} in observable path",
                i
            );
        }

        // Add manuever to the start of the path
        path = Path::new(
            vec![&cell1, &cell2, &cell3, &cell4, &cell5],
            vec![
                LaneChangeType::ChangeLeft,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
            ],
            10.0,
        );
        let observable_path = process_path(&mut path, speed_limit, cell5.get_id(), &current_state);

        let correct_path = ObservablePath {
            wanted_maneuver: LaneChangeType::ChangeLeft,
            last_cell_state: CellState::Free,
            trimmed_path: vec![],
            has_vehicle_on_path: false,
            speed_limit_reached: false,
            stopped_on_maneuver: true,
            stopped_speed_possible: false,
            blocking_cell_id: -1,
        };

        assert_eq!(
            observable_path.wanted_maneuver, correct_path.wanted_maneuver,
            "Incorrect maneuver for observable path"
        );

        assert_eq!(
            observable_path.last_cell_state, correct_path.last_cell_state,
            "Incorrect last cell state for observable path"
        );

        assert_eq!(
            observable_path.trimmed_path.len(),
            correct_path.trimmed_path.len(),
            "Incorrect number of cells vehicle can move forward"
        );

        for i in 0..observable_path.trimmed_path.len() {
            assert_eq!(
                observable_path.trimmed_path[i].get_id(),
                correct_path.trimmed_path[i].get_id(),
                "Incorrect cell at pos #{} in observable path",
                i
            );
        }

        // Check speed restriction
        let vehicle_speed = 1;
        path = Path::new(
            vec![&cell1, &cell2, &cell3, &cell4, &cell5],
            vec![
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
            ],
            10.0,
        );
        let observable_path =
            process_path(&mut path, vehicle_speed, cell5.get_id(), &current_state);

        let correct_path = ObservablePath {
            wanted_maneuver: LaneChangeType::NoChange,
            last_cell_state: CellState::Free,
            trimmed_path: vec![&cell2],
            has_vehicle_on_path: false,
            speed_limit_reached: true,
            stopped_on_maneuver: false,
            stopped_speed_possible: false,
            blocking_cell_id: -1,
        };

        assert_eq!(
            observable_path.wanted_maneuver, correct_path.wanted_maneuver,
            "Incorrect maneuver for observable path"
        );

        assert_eq!(
            observable_path.last_cell_state, correct_path.last_cell_state,
            "Incorrect last cell state for observable path"
        );

        assert_eq!(
            observable_path.trimmed_path.len(),
            correct_path.trimmed_path.len(),
            "Incorrect number of cells vehicle can move forward"
        );

        for i in 0..observable_path.trimmed_path.len() {
            assert_eq!(
                observable_path.trimmed_path[i].get_id(),
                correct_path.trimmed_path[i].get_id(),
                "Incorrect cell at pos #{} in observable path",
                i
            );
        }

        // Check traffic light banned state
        cell3.set_state(CellState::Banned);
        path = Path::new(
            vec![&cell1, &cell2, &cell3, &cell4, &cell5],
            vec![
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
                LaneChangeType::NoChange,
            ],
            10.0,
        );
        let observable_path = process_path(&mut path, speed_limit, cell5.get_id(), &current_state);

        let correct_path = ObservablePath {
            wanted_maneuver: LaneChangeType::NoChange,
            last_cell_state: CellState::Banned,
            trimmed_path: vec![&cell2],
            has_vehicle_on_path: false,
            speed_limit_reached: false,
            stopped_on_maneuver: false,
            stopped_speed_possible: false,
            blocking_cell_id: -1,
        };

        assert_eq!(
            observable_path.wanted_maneuver, correct_path.wanted_maneuver,
            "Incorrect maneuver for observable path"
        );

        assert_eq!(
            observable_path.last_cell_state, correct_path.last_cell_state,
            "Incorrect last cell state for observable path"
        );

        assert_eq!(
            observable_path.trimmed_path.len(),
            correct_path.trimmed_path.len(),
            "Incorrect number of cells vehicle can move forward"
        );

        for i in 0..observable_path.trimmed_path.len() {
            assert_eq!(
                observable_path.trimmed_path[i].get_id(),
                correct_path.trimmed_path[i].get_id(),
                "Incorrect cell at pos #{} in observable path",
                i
            );
        }
    }
}
